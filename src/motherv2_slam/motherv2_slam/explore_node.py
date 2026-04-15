"""
explore_node.py
──────────────────────────────────────────────────────────────────────────────
Frontier 기반 자율 맵핑 노드 (Nav2 없이 LiDAR만 사용)

알고리즘:
  1. OccupancyGrid에서 frontier 탐지
     (FREE 셀 중 UNKNOWN 이웃이 있는 셀 → 클러스터링)
  2. 가장 가까운 frontier 선택
  3. 그 방향으로 회전 → 직진
  4. 전방 장애물 감지 시 정지 + 재계획
  5. frontier 소진 시 탐색 완료 → tracking 모드 자동 전환

활성화:
  /motherv2/slam_mode 토픽에 "explore" 발행 시 탐색 시작
  "mapping" / "tracking" 발행 시 즉시 정지

퍼블리시:
  /motherv2/cmd_motor   motherv2_interfaces/MotorCommand
  /motherv2/slam_mode   std_msgs/String  (완료 시 "tracking")

구독:
  /map                  nav_msgs/OccupancyGrid   (TRANSIENT_LOCAL)
  /scan                 sensor_msgs/LaserScan
  /motherv2/slam_mode   std_msgs/String
"""

import math
import time

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import (
    Buffer, TransformListener,
    LookupException, ConnectivityException, ExtrapolationException,
)
from motherv2_interfaces.msg import MotorCommand


class ExploreNode(Node):

    # ── 파라미터 상수 ──────────────────────────────────────────────────────
    SPEED_ROTATE = 110          # 회전 속도 (0-255)
    SPEED_DRIVE  = 140          # 직진 속도
    STOP_DIST    = 0.35         # 전방 장애물 정지 거리 (m)
    ARRIVE_DIST  = 0.6          # frontier 도달 판정 거리 (m)
    ROTATE_TOL   = 0.12         # 회전 허용 오차 (rad, ≈ 7°)
    HEADING_TOL  = 0.30         # 직진 중 경로 이탈 허용치 (rad)
    CLUSTER_R    = 0.8          # frontier 클러스터 반경 (m)
    MIN_CLUSTER  = 5            # 유효 클러스터 최소 셀 수
    STUCK_TIME   = 8.0          # 정체 판정 시간 (s)
    STUCK_MOVE   = 0.12         # 정체 판정 최소 이동량 (m)
    FRONT_HALF   = 0.52         # 전방 검사 반각 (rad, ≈ 30°)
    FRONTIER_HZ  = 1.0          # frontier 재계산 주기 (Hz)

    def __init__(self):
        super().__init__('explore_node')

        # ── 상태 머신 ──────────────────────────────────────────────────────
        # idle → rotating → driving → idle  (반복)
        # stuck_recovery → idle
        self._state  = 'idle'
        self._active = False

        self._target_x: float | None = None
        self._target_y: float | None = None
        self._frontiers: list = []       # [(cx, cy, size), ...]

        # ── 센서 / 포즈 ────────────────────────────────────────────────────
        self._latest_map:  OccupancyGrid | None = None
        self._latest_scan: LaserScan | None = None
        self._robot_pose:  tuple | None = None  # (x, y, yaw)

        # ── 정체 감지 ──────────────────────────────────────────────────────
        self._last_move_pos  = (0.0, 0.0)
        self._last_move_time = 0.0
        self._recovery_start = 0.0

        # ── QoS ────────────────────────────────────────────────────────────
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 구독 ────────────────────────────────────────────────────────────
        self.map_sub  = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, scan_qos)
        self.mode_sub = self.create_subscription(
            String, '/motherv2/slam_mode', self._mode_cb, 1)

        # ── 퍼블리셔 ────────────────────────────────────────────────────────
        self.motor_pub = self.create_publisher(
            MotorCommand, '/motherv2/cmd_motor', 1)
        self.mode_pub = self.create_publisher(
            String, '/motherv2/slam_mode', 1)

        # ── TF2 ────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 타이머 ────────────────────────────────────────────────────────
        self.create_timer(0.1, self._control_loop)   # 10Hz 제어
        self.create_timer(1.0 / self.FRONTIER_HZ,
                          self._update_frontiers)     # frontier 재계산

        self.get_logger().info('explore_node 시작 — slam_mode="explore" 대기 중')

    # ── 콜백 ──────────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = msg

    def _mode_cb(self, msg: String):
        mode = msg.data.strip().lower()
        if mode == 'explore' and not self._active:
            self._active = True
            self._state  = 'idle'
            self.get_logger().info('[Explore] 탐색 시작')
        elif mode != 'explore' and self._active:
            self._active = False
            self._stop()
            self.get_logger().info('[Explore] 탐색 중지')

    # ── 제어 루프 (10Hz) ──────────────────────────────────────────────────

    def _control_loop(self):
        if not self._active:
            return

        self._update_pose()
        if self._robot_pose is None:
            return

        rx, ry, ryaw = self._robot_pose

        # ── idle: 다음 frontier 선택 ─────────────────────────────────────
        if self._state == 'idle':
            if not self._frontiers:
                self.get_logger().info('[Explore] frontier 없음 — 탐색 완료')
                self._stop()
                self._active = False
                self._publish_mode('tracking')
                return

            # 가장 가까운 frontier
            best = min(self._frontiers,
                       key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
            self._target_x, self._target_y = best[0], best[1]
            self._state = 'rotating'
            self._reset_stuck(rx, ry)
            self.get_logger().info(
                f'[Explore] 목표 ({best[0]:.2f}, {best[1]:.2f}) '
                f'거리 {math.hypot(best[0]-rx, best[1]-ry):.2f}m '
                f'클러스터 {best[2]}셀'
            )

        # ── rotating: 목표 방향으로 회전 ─────────────────────────────────
        elif self._state == 'rotating':
            dx = self._target_x - rx
            dy = self._target_y - ry
            dist = math.hypot(dx, dy)

            if dist < self.ARRIVE_DIST:
                self._stop()
                self._state = 'idle'
                return

            target_yaw = math.atan2(dy, dx)
            err = self._angle_diff(target_yaw, ryaw)

            if abs(err) < self.ROTATE_TOL:
                self._stop()
                self._state = 'driving'
            elif err > 0:
                self._rotate_left()
            else:
                self._rotate_right()

        # ── driving: 전진 ─────────────────────────────────────────────────
        elif self._state == 'driving':
            dx = self._target_x - rx
            dy = self._target_y - ry
            dist = math.hypot(dx, dy)

            if dist < self.ARRIVE_DIST:
                self._stop()
                self._state = 'idle'
                return

            # 장애물 확인
            if not self._front_clear():
                self._stop()
                self.get_logger().info('[Explore] 전방 장애물 → 재계획')
                self._state = 'idle'
                # 현재 frontier 제거 (막힌 방향)
                self._frontiers = [
                    f for f in self._frontiers
                    if math.hypot(f[0] - self._target_x,
                                  f[1] - self._target_y) > 0.3
                ]
                return

            # 진행 방향 보정
            target_yaw = math.atan2(dy, dx)
            err = self._angle_diff(target_yaw, ryaw)
            if abs(err) > self.HEADING_TOL:
                self._stop()
                self._state = 'rotating'
                return

            self._drive_forward()

            # 정체 감지
            self._check_stuck(rx, ry)

        # ── stuck_recovery: 후진 후 재계획 ───────────────────────────────
        elif self._state == 'stuck_recovery':
            elapsed = time.time() - self._recovery_start
            if elapsed < 1.5:
                self._drive_backward()
            elif elapsed < 3.0:
                self._rotate_right()
            else:
                self._stop()
                self._state = 'idle'
                rx_, ry_, _ = self._robot_pose
                self._reset_stuck(rx_, ry_)

    # ── Frontier 탐지 (1Hz) ────────────────────────────────────────────────

    def _update_frontiers(self):
        if not self._active or self._latest_map is None:
            return
        self._frontiers = self._find_frontiers(self._latest_map)

    def _find_frontiers(self, grid: OccupancyGrid) -> list:
        """
        FREE(0) 셀 중 UNKNOWN(-1) 이웃이 있는 셀 탐지 → 클러스터링.
        Returns: [(cx, cy, size), ...]
        """
        w   = grid.info.width
        h   = grid.info.height
        res = grid.info.resolution
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y
        data = grid.data

        cells = []
        # 2셀 간격으로 샘플링 (성능 최적화)
        for gy in range(1, h - 1, 2):
            for gx in range(1, w - 1, 2):
                if data[gy * w + gx] != 0:
                    continue
                neighbors = (
                    data[(gy - 1) * w + gx],
                    data[(gy + 1) * w + gx],
                    data[gy * w + (gx - 1)],
                    data[gy * w + (gx + 1)],
                )
                if -1 in neighbors:
                    cells.append((ox + (gx + 0.5) * res,
                                  oy + (gy + 0.5) * res))

        return self._cluster(cells)

    def _cluster(self, cells: list) -> list:
        """거리 기반 클러스터링. 큰 클러스터 우선 반환."""
        if not cells:
            return []

        used = [False] * len(cells)
        clusters = []

        for i, pt in enumerate(cells):
            if used[i]:
                continue
            grp = [pt]
            used[i] = True
            for j in range(i + 1, len(cells)):
                if used[j]:
                    continue
                if math.hypot(cells[j][0] - pt[0],
                              cells[j][1] - pt[1]) <= self.CLUSTER_R:
                    grp.append(cells[j])
                    used[j] = True

            if len(grp) >= self.MIN_CLUSTER:
                cx = sum(p[0] for p in grp) / len(grp)
                cy = sum(p[1] for p in grp) / len(grp)
                clusters.append((cx, cy, len(grp)))

        # 큰 클러스터 우선 (탐색 영역 넓은 쪽)
        clusters.sort(key=lambda c: -c[2])
        return clusters

    # ── 헬퍼 ──────────────────────────────────────────────────────────────

    def _update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            x   = tf.transform.translation.x
            y   = tf.transform.translation.y
            qz  = tf.transform.rotation.z
            qw  = tf.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            self._robot_pose = (x, y, yaw)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._robot_pose = None

    def _front_clear(self) -> bool:
        """전방 ±30° 섹터에 STOP_DIST 이내 장애물 없으면 True."""
        scan = self._latest_scan
        if scan is None:
            return True
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= self.FRONT_HALF and r < self.STOP_DIST:
                return False
        return True

    def _check_stuck(self, rx: float, ry: float):
        now = time.time()
        if now - self._last_move_time < self.STUCK_TIME:
            return
        moved = math.hypot(rx - self._last_move_pos[0],
                           ry - self._last_move_pos[1])
        if moved < self.STUCK_MOVE:
            self.get_logger().warn('[Explore] 정체 → 복구 시도')
            self._stop()
            self._state = 'stuck_recovery'
            self._recovery_start = now
        else:
            self._reset_stuck(rx, ry)

    def _reset_stuck(self, rx: float, ry: float):
        self._last_move_pos  = (rx, ry)
        self._last_move_time = time.time()

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """[-π, π] 정규화된 각도 차."""
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def _publish_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

    # ── 모터 명령 ──────────────────────────────────────────────────────────

    def _stop(self):
        self._motor(0, 0, 0, 0)

    def _drive_forward(self):
        self._motor(self.SPEED_DRIVE, self.SPEED_DRIVE, 1, 1)

    def _drive_backward(self):
        self._motor(self.SPEED_DRIVE, self.SPEED_DRIVE, 2, 2)

    def _rotate_left(self):
        self._motor(self.SPEED_ROTATE, self.SPEED_ROTATE, 2, 1)

    def _rotate_right(self):
        self._motor(self.SPEED_ROTATE, self.SPEED_ROTATE, 1, 2)

    def _motor(self, ls: int, rs: int, ld: int, rd: int):
        msg = MotorCommand()
        msg.left_speed  = ls
        msg.right_speed = rs
        msg.left_dir    = ld
        msg.right_dir   = rd
        self.motor_pub.publish(msg)

    def destroy_node(self):
        self._stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
