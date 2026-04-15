"""
slam_localization_node.py
──────────────────────────────────────────────────────────────────────────────
Hector SLAM 기반 객체 위치 추정 · 궤적 예측 · 장소 인덱싱 노드

[핵심 목표]
  1) 라이다 깊이로 카메라 bbox 추정보다 정확한 실거리 측정
  2) 맵 프레임에서 객체 궤적(velocity) 추정
     → 카메라에서 사라졌을 때 "어디로 갔는지" 예측
  3) 장소 인덱싱: 객체가 자주 감지된 위치를 파일에 저장
     → 장기 탐색 패턴 수립에 활용

[토픽]
  구독:
    /scan                     sensor_msgs/LaserScan
    /motherv2/detections      motherv2_interfaces/DetectionArray

  퍼블리시:
    /motherv2/object_estimates     motherv2_interfaces/ObjectEstimateArray
      - 실시간 라이다 깊이 + 맵 좌표 + EMA 평활화값
    /motherv2/target_prediction    geometry_msgs/PoseStamped
      - 객체 로스트 시 예측 위치 (dead-reckoning)
      - follower_node가 이 위치로 복구 탐색 수행

[TF 의존]
  hector_mapping → map → (odom →) base_link TF 퍼블리시 필요
  /scan의 frame_id가 base_link 또는 TF로 연결된 프레임이어야 함
"""

import json
import math
import os
import time
from collections import deque

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String
from tf2_ros import (
    Buffer, TransformListener,
    LookupException, ConnectivityException, ExtrapolationException,
)
from motherv2_interfaces.msg import DetectionArray, ObjectEstimate, ObjectEstimateArray


# ──────────────────────────────────────────────────────────────────────────────
# 궤적 트래커: 위치 이력 → 속도 벡터 → 미래 위치 예측
# ──────────────────────────────────────────────────────────────────────────────
class TrajectoryTracker:
    """
    최근 N개 (x, y, t) 좌표를 저장해 선형 속도 벡터를 추정하고
    dt 초 후 예측 위치를 반환한다.
    """

    def __init__(self, max_len: int = 20, min_len: int = 3):
        self.buf: deque[tuple[float, float, float]] = deque(maxlen=max_len)
        self.min_len = min_len
        self.last_seen: float = 0.0

    def update(self, x: float, y: float):
        now = time.time()
        self.buf.append((x, y, now))
        self.last_seen = now

    def predict(self, dt: float) -> tuple[float | None, float | None]:
        """
        현재 속도 벡터로 dt 초 후 위치 예측.
        데이터 부족 시 마지막 위치 반환.
        """
        if not self.buf:
            return None, None

        last_x, last_y, _ = self.buf[-1]

        if len(self.buf) < self.min_len:
            return last_x, last_y

        # 최근 절반 구간으로 속도 추정 (노이즈 감소)
        half = max(self.min_len, len(self.buf) // 2)
        recent = list(self.buf)[-half:]
        x0, y0, t0 = recent[0]
        x1, y1, t1 = recent[-1]
        elapsed = t1 - t0
        if elapsed < 0.01:
            return last_x, last_y

        vx = (x1 - x0) / elapsed
        vy = (y1 - y0) / elapsed

        # 최대 속도 클리핑 (m/s) — 사람 걷기 속도 최대 1.5 m/s
        speed = math.hypot(vx, vy)
        max_speed = 1.5
        if speed > max_speed:
            vx *= max_speed / speed
            vy *= max_speed / speed

        pred_x = last_x + vx * dt
        pred_y = last_y + vy * dt
        return pred_x, pred_y

    def last_position(self) -> tuple[float | None, float | None]:
        if not self.buf:
            return None, None
        x, y, _ = self.buf[-1]
        return x, y

    def reset(self):
        self.buf.clear()

    @property
    def has_data(self) -> bool:
        return len(self.buf) > 0


# ──────────────────────────────────────────────────────────────────────────────
# EMA 깊이 평활화
# ──────────────────────────────────────────────────────────────────────────────
class DepthEMA:
    def __init__(self, alpha: float = 0.4):
        self.alpha = alpha
        self._val: float | None = None

    def update(self, depth: float) -> float:
        if self._val is None:
            self._val = depth
        else:
            self._val = self.alpha * depth + (1.0 - self.alpha) * self._val
        return self._val

    def reset(self):
        self._val = None


# ──────────────────────────────────────────────────────────────────────────────
# 장소 인덱스: 객체 감지 위치를 JSON 파일로 누적
# ──────────────────────────────────────────────────────────────────────────────
class LocationIndex:
    """
    {class_id: [(x, y, timestamp), ...]} 를 JSON 파일에 저장.
    같은 클러스터(반경 cluster_radius 이내)에 있는 기존 위치는
    카운트만 증가 (중복 저장 방지).
    """

    def __init__(self, filepath: str, cluster_radius: float = 0.5):
        self.filepath = filepath
        self.cluster_radius = cluster_radius
        self._data: dict[str, list[dict]] = {}
        self._dirty = False
        self._load()

    def _load(self):
        try:
            if os.path.exists(self.filepath):
                with open(self.filepath, 'r') as f:
                    self._data = json.load(f)
        except Exception:
            self._data = {}

    def add(self, class_id: int, x: float, y: float):
        key = str(class_id)
        if key not in self._data:
            self._data[key] = []

        # 같은 클러스터가 있으면 카운트만 증가
        for entry in self._data[key]:
            dist = math.hypot(entry['x'] - x, entry['y'] - y)
            if dist <= self.cluster_radius:
                entry['count'] = entry.get('count', 1) + 1
                entry['last_seen'] = time.time()
                self._dirty = True
                return

        # 새 위치 추가
        self._data[key].append({
            'x': round(x, 3),
            'y': round(y, 3),
            'count': 1,
            'first_seen': time.time(),
            'last_seen': time.time(),
        })
        self._dirty = True

    def save_if_dirty(self):
        if not self._dirty:
            return
        try:
            os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
            with open(self.filepath, 'w') as f:
                json.dump(self._data, f, indent=2)
            self._dirty = False
        except Exception:
            pass

    def get_locations(self, class_id: int) -> list[dict]:
        return self._data.get(str(class_id), [])


# ──────────────────────────────────────────────────────────────────────────────
# 메인 노드
# ──────────────────────────────────────────────────────────────────────────────
class SlamLocalizationNode(Node):

    def __init__(self):
        super().__init__('slam_localization_node')

        # ── 파라미터 선언 ──────────────────────────────────────────────────
        self.declare_parameter('camera_hfov_deg', 62.2)
        self.declare_parameter('image_width', 640)
        # LiDAR 마운트 각도 오프셋 (라디안). 카메라 광축 vs LiDAR 정면 차이.
        self.declare_parameter('lidar_angle_offset_rad', 0.0)
        # 라이다 섹터 검색 퍼센타일 (0=최솟값, 50=중앙값)
        self.declare_parameter('depth_percentile', 15)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 8.0)
        # EMA 계수
        self.declare_parameter('ema_alpha', 0.4)
        # 객체 로스트 판정 시간 (초)
        self.declare_parameter('lost_timeout_s', 1.5)
        # dead-reckoning 예측 시간 (초) — 로스트 후 이 시간만큼 앞을 예측
        self.declare_parameter('prediction_horizon_s', 2.0)
        # TF 프레임
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # 섹터 패딩 (라디안)
        self.declare_parameter('sector_padding_rad', 0.05)
        # 장소 인덱스 저장 경로
        self.declare_parameter(
            'location_index_path',
            os.path.expanduser('~/.ros/motherv2/location_index.json'),
        )
        # 장소 인덱스 저장 주기 (초)
        self.declare_parameter('index_save_interval_s', 10.0)
        # 장소 인덱스 클러스터 반경 (미터)
        self.declare_parameter('index_cluster_radius_m', 0.5)
        # 추적 대상 class_id (-1 = detections에서 첫 번째 객체)
        self.declare_parameter('target_class_id', 39)

        # ── 파라미터 읽기 ──────────────────────────────────────────────────
        self.camera_hfov = math.radians(
            self.get_parameter('camera_hfov_deg').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.lidar_offset = float(
            self.get_parameter('lidar_angle_offset_rad').value)
        self.depth_pct = int(self.get_parameter('depth_percentile').value)
        self.min_depth = float(self.get_parameter('min_depth_m').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout_s').value)
        self.pred_horizon = float(
            self.get_parameter('prediction_horizon_s').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.sector_padding = float(
            self.get_parameter('sector_padding_rad').value)
        self.target_class = int(self.get_parameter('target_class_id').value)

        # ── 상태 변수 ──────────────────────────────────────────────────────
        self._latest_scan: LaserScan | None = None
        self._last_detection_time: float = 0.0
        self._is_lost: bool = True

        # class_id → DepthEMA
        self._depth_ema: dict[int, DepthEMA] = {}
        # class_id → TrajectoryTracker
        self._trajectory: dict[int, TrajectoryTracker] = {}

        # ── 3단계 추적 상태 ────────────────────────────────────────────────
        # 카메라 → LiDAR 전용 → 마지막 위치 기억
        self._last_known_map_x: float | None = None
        self._last_known_map_y: float | None = None
        self._last_known_depth: float | None = None
        self._last_known_angle: float | None = None  # 마지막 카메라 기준 방위각 (로봇 프레임)
        self._last_known_pub_time: float = 0.0  # last_known 퍼블리시 rate limit

        # 객체가 카메라 가장자리에서 나갔는지 여부
        # True  → 화면 끝에서 이탈, 즉시 LiDAR 추적 전환 필요
        # False → 화면 중앙 소실, 가림 가능성으로 lost_timeout 대기
        self._exited_from_edge: bool = False
        # 가장자리 판정 기준: 화면 좌우 10% 이내
        self._edge_threshold: float = 0.10

        # ── 장소 인덱스 ────────────────────────────────────────────────────
        idx_path = self.get_parameter('location_index_path').value
        cluster_r = float(
            self.get_parameter('index_cluster_radius_m').value)
        self._loc_index = LocationIndex(idx_path, cluster_radius=cluster_r)
        save_interval = float(
            self.get_parameter('index_save_interval_s').value)

        # ── TF2 ────────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── SLAM 모드 / 정적 맵 ──────────────────────────────────────────────
        # 'mapping'  : 궤적 기반 추적 (기존 방식)
        # 'tracking' : 정적 맵 change detection (맵 고정 후 동적 객체 탐지)
        self._slam_mode: str = 'mapping'
        self._latest_map: OccupancyGrid | None = None   # slam_toolbox에서 수신하는 최신 맵
        self._frozen_map: OccupancyGrid | None = None   # tracking 모드 진입 시 스냅샷

        # ── QoS ────────────────────────────────────────────────────────────
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── 구독 ────────────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, scan_qos)
        self.det_sub = self.create_subscription(
            DetectionArray, '/motherv2/detections',
            self._detections_callback, 1)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)
        self.mode_sub = self.create_subscription(
            String, '/motherv2/slam_mode', self._mode_callback, 1)

        # ── 퍼블리셔 ────────────────────────────────────────────────────────
        self.estimates_pub = self.create_publisher(
            ObjectEstimateArray, '/motherv2/object_estimates', 10)
        # 로스트 시 예측 위치 (맵 프레임) — nav2 또는 고급 플래너용
        self.prediction_pub = self.create_publisher(
            PoseStamped, '/motherv2/target_prediction', 10)
        # 로봇 기준 탐색 방향 (라디안) — follower_node 복구 탐색에 직접 사용
        # 양수=좌, 음수=우 (ROS 관례)
        from std_msgs.msg import Float32
        self.search_dir_pub = self.create_publisher(
            Float32, '/motherv2/search_direction', 10)
        self._Float32 = Float32

        # ── 타이머 ────────────────────────────────────────────────────────
        # 로스트 판정 + 예측 퍼블리시 (10Hz)
        self.create_timer(0.1, self._lost_check_loop)
        # 장소 인덱스 저장 (주기적)
        self.create_timer(save_interval, self._save_index)

        self.get_logger().info(
            f'slam_localization_node 시작\n'
            f'  camera_hfov={math.degrees(self.camera_hfov):.1f}° '
            f'| lidar_offset={math.degrees(self.lidar_offset):.1f}°\n'
            f'  depth_pct={self.depth_pct} '
            f'| lost_timeout={self.lost_timeout}s '
            f'| pred_horizon={self.pred_horizon}s\n'
            f'  location_index → {idx_path}'
        )

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg

    def _map_callback(self, msg: OccupancyGrid):
        self._latest_map = msg

    def _mode_callback(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode == self._slam_mode:
            return
        if new_mode == 'tracking':
            if self._latest_map is None:
                self.get_logger().warn('[Mode] 맵이 아직 없어 tracking 모드 전환 불가')
                return
            self._frozen_map = self._latest_map
            self._slam_mode = 'tracking'
            w, h = self._frozen_map.info.width, self._frozen_map.info.height
            self.get_logger().info(
                f'[Mode] → tracking (change detection 활성화, '
                f'맵 {w}x{h} 스냅샷)'
            )
        elif new_mode == 'mapping':
            self._frozen_map = None
            self._slam_mode = 'mapping'
            self.get_logger().info('[Mode] → mapping (궤적 기반 추적)')

    def _detections_callback(self, msg: DetectionArray):
        estimates: list[ObjectEstimate] = []

        detected_target = False
        for det in msg.detections:
            est = self._process_detection(det, msg.header)
            estimates.append(est)

            if det.class_id == self.target_class:
                detected_target = True
                # 카메라 bbox로 방위각은 항상 갱신
                self._last_known_angle = est.angle
                if est.map_valid:
                    # 궤적 업데이트
                    traj = self._get_trajectory(det.class_id)
                    traj.update(est.map_x, est.map_y)
                    # 장소 인덱싱
                    self._loc_index.add(det.class_id, est.map_x, est.map_y)
                    # 마지막 위치 기억 (3단계 추적용)
                    self._last_known_map_x = est.map_x
                    self._last_known_map_y = est.map_y
                    if est.depth_valid:
                        self._last_known_depth = est.depth

        if detected_target:
            self._last_detection_time = time.time()
            self._is_lost = False
            # bbox 중심이 화면 가장자리 10% 이내면 이탈 예고
            cx_norm = 0.5
            for det in msg.detections:
                if det.class_id == self.target_class:
                    cx_norm = (det.x + det.w / 2.0) / self.image_width
                    break
            self._exited_from_edge = (
                cx_norm < self._edge_threshold or
                cx_norm > (1.0 - self._edge_threshold)
            )

        if estimates:
            out = ObjectEstimateArray()
            out.header = Header()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.map_frame
            out.estimates = estimates
            self.estimates_pub.publish(out)

    def _lost_check_loop(self):
        """
        3단계 추적:
          1) 카메라 감지 중  → _detections_callback이 담당 (여기서 아무것도 안 함)
          2) 카메라 lost     → LiDAR 전용으로 마지막 방향 탐색
          3) LiDAR도 실패   → 마지막 알려진 위치를 estimate로 유지
        """
        elapsed = time.time() - self._last_detection_time
        if elapsed < self.lost_timeout:
            return  # 카메라 추적 중

        # ── 카메라 신호 없음 ──────────────────────────────────────────────
        if not self._is_lost:
            self._is_lost = True
            self.get_logger().info(
                f'[LOST] class={self.target_class} 카메라 없음 ({elapsed:.1f}s) '
                f'→ LiDAR 전용 추적 시도'
            )

        # ── 2단계: LiDAR 전용 추적 ───────────────────────────────────────
        if self._last_known_map_x is None:
            return  # 카메라가 한 번도 물체를 못 본 것

        result = self._lidar_only_estimate()
        if result is not None:
            depth, map_x, map_y, angle = result
            self._publish_lidar_estimate(depth, map_x, map_y, angle)
            return  # 추적 성공 → prediction 불필요

        # ── 3단계: 마지막 위치 유지 ──────────────────────────────────────
        now = time.time()
        if now - self._last_known_pub_time >= 1.0:   # 1Hz로 rate-limit
            self.get_logger().info(
                f'[LAST_KNOWN] ({self._last_known_map_x:.2f}, {self._last_known_map_y:.2f}) '
                f'유지 중 (elapsed={elapsed:.1f}s)'
            )
            self._publish_last_known_estimate()
            self._last_known_pub_time = now

        # 예측 위치 퍼블리시 (follower_node 복구 탐색용)
        traj = self._trajectory.get(self.target_class)
        if traj is None or not traj.has_data:
            return

        px, py = traj.predict(self.pred_horizon)
        if px is None:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.prediction_pub.publish(pose)

        search_angle = self._compute_search_angle(px, py)
        if search_angle is not None:
            msg_f32 = self._Float32()
            msg_f32.data = search_angle
            self.search_dir_pub.publish(msg_f32)

    # ── LiDAR 전용 추적 ───────────────────────────────────────────────────

    def _lidar_only_estimate(self):
        """
        궤적 예측 위치 방향으로 LiDAR 스캔하여 객체 추적.

        거리 비교 대신 방향 기반 추적:
          - 궤적으로 예측한 맵 좌표 → 로봇 기준 방위각 계산
          - 경과 시간에 비례해 스캔 섹터 확대 (불확실성 반영)
          - 해당 방향에서 유효한 LiDAR 값이 있으면 그게 객체
          - 거리 비교 없음 → 객체가 이동해도 계속 추적 가능

        Returns (depth, map_x, map_y, angle) or None.
        """
        scan = self._latest_scan
        if scan is None:
            return None

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        robot_yaw = 2.0 * math.atan2(
            robot_tf.transform.rotation.z, robot_tf.transform.rotation.w)

        elapsed = time.time() - self._last_detection_time
        traj = self._trajectory.get(self.target_class)
        pred_x, pred_y = None, None
        if traj and traj.has_data:
            pred_x, pred_y = traj.predict(elapsed)
            if pred_x is None:
                pred_x, pred_y = traj.last_position()

        # ── tracking 모드: 맵 change detection ────────────────────────────
        if self._slam_mode == 'tracking' and self._frozen_map is not None:
            result = self._change_detect_estimate(
                scan, robot_tf, pred_x, pred_y, rx, ry)
            return result

        # ── mapping 모드: 궤적 방향 LiDAR 스캔 ───────────────────────────
        if pred_x is None:
            return None

        dx = pred_x - rx
        dy = pred_y - ry
        world_angle = math.atan2(dy, dx)
        search_angle = world_angle - robot_yaw
        search_angle = math.atan2(math.sin(search_angle), math.cos(search_angle))

        # 경과 시간이 길수록 섹터 확대 (초당 5°, 최대 ±60°)
        sector = min(math.radians(60), math.radians(20) + elapsed * math.radians(5))

        raw_depth = self._measure_depth(search_angle, sector)
        if raw_depth <= 0.0:
            self.get_logger().debug(
                f'[LiDAR] 방향({math.degrees(search_angle):.0f}°)'
                f' ±{math.degrees(sector):.0f}° 스캔 없음'
            )
            return None

        self.get_logger().info(
            f'[LiDAR] 추적: {raw_depth:.2f}m @ {math.degrees(search_angle):.0f}° '
            f'(±{math.degrees(sector):.0f}°, elapsed={elapsed:.1f}s)'
        )

        map_x, map_y = self._depth_to_map(raw_depth, search_angle)
        if map_x is None:
            return None

        return raw_depth, map_x, map_y, search_angle

    # ── Change Detection (tracking 모드) ──────────────────────────────────

    def _find_dynamic_points(self, scan: LaserScan, robot_tf) -> list:
        """
        현재 LiDAR 스캔에서 정적 맵의 FREE 셀에 찍힌 점 추출.
        맵에 없던 물체(동적 장애물 = 사람)만 남는다.

        Returns: [(map_x, map_y, range, beam_angle), ...]
        """
        grid = self._frozen_map
        if grid is None:
            return []

        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        gw = grid.info.width
        gh = grid.info.height

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        yaw = 2.0 * math.atan2(
            robot_tf.transform.rotation.z,
            robot_tf.transform.rotation.w,
        )
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        dynamic = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max
                    and self.min_depth <= r <= self.max_depth):
                continue
            beam_angle = scan.angle_min + i * scan.angle_increment
            lx = r * math.cos(beam_angle)
            ly = r * math.sin(beam_angle)

            # 맵 프레임 변환
            mx = rx + cos_y * lx - sin_y * ly
            my = ry + sin_y * lx + cos_y * ly

            # 그리드 셀 인덱스
            cx = int((mx - ox) / res)
            cy = int((my - oy) / res)
            if not (0 <= cx < gw and 0 <= cy < gh):
                continue

            # FREE(0)인 셀에 포인트 → 동적 물체
            if grid.data[cy * gw + cx] == 0:
                dynamic.append((mx, my, r, beam_angle))

        return dynamic

    def _cluster_points(self, pts: list, radius: float = 0.4) -> list:
        """
        거리 기반 단순 클러스터링.
        Returns: [(cx, cy, avg_range, avg_angle, point_count), ...]
        """
        if not pts:
            return []
        used = [False] * len(pts)
        clusters = []
        for i, pt in enumerate(pts):
            if used[i]:
                continue
            cluster = [pt]
            used[i] = True
            for j in range(i + 1, len(pts)):
                if used[j]:
                    continue
                if math.hypot(pt[0] - pts[j][0], pt[1] - pts[j][1]) <= radius:
                    cluster.append(pts[j])
                    used[j] = True
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            ar = sum(p[2] for p in cluster) / len(cluster)
            aa = sum(p[3] for p in cluster) / len(cluster)
            clusters.append((cx, cy, ar, aa, len(cluster)))
        return clusters

    def _change_detect_estimate(self, scan: LaserScan, robot_tf,
                                pred_x: float | None,
                                pred_y: float | None,
                                rx: float, ry: float):
        """
        Change detection으로 동적 물체 위치 추정.
        가장 큰 클러스터 중 예측 위치에 가장 가까운 것을 반환.
        Returns (depth, map_x, map_y, angle) or None.
        """
        dynamic = self._find_dynamic_points(scan, robot_tf)
        clusters = self._cluster_points(dynamic)
        if not clusters:
            return None

        # 최소 3포인트 이상 클러스터만 유효
        valid = [c for c in clusters if c[4] >= 3]
        if not valid:
            return None

        # 예측 위치 또는 마지막 위치 기준으로 가장 가까운 클러스터
        ref_x = pred_x if pred_x is not None else rx
        ref_y = pred_y if pred_y is not None else ry
        best = min(valid, key=lambda c: math.hypot(c[0] - ref_x, c[1] - ref_y))

        cx, cy, avg_r, avg_a, count = best
        self.get_logger().info(
            f'[ChangeDetect] 클러스터: ({cx:.2f},{cy:.2f}) '
            f'{count}pts {avg_r:.2f}m @ {math.degrees(avg_a):.0f}°'
        )
        return avg_r, cx, cy, avg_a

    def _publish_lidar_estimate(self, depth: float, map_x: float, map_y: float,
                                angle: float):
        """LiDAR 전용 위치 추정 퍼블리시 (confidence=0.3으로 구분)."""
        ema = self._get_depth_ema(self.target_class)
        smooth = ema.update(depth)

        traj = self._get_trajectory(self.target_class)
        traj.update(map_x, map_y)

        # 마지막 위치 갱신 (다음 LiDAR 탐색 기준점)
        self._last_known_map_x = map_x
        self._last_known_map_y = map_y
        self._last_known_depth = smooth
        self._last_known_angle = angle

        est = ObjectEstimate()
        est.header.stamp = self.get_clock().now().to_msg()
        est.header.frame_id = self.map_frame
        est.class_id = self.target_class
        est.confidence = 0.3        # LiDAR 전용 식별자 (카메라 threshold 0.35 미만)
        est.depth = smooth
        est.angle = angle
        est.map_x = map_x
        est.map_y = map_y
        est.depth_valid = True
        est.map_valid = True

        out = ObjectEstimateArray()
        out.header.stamp = est.header.stamp
        out.header.frame_id = self.map_frame
        out.estimates = [est]
        self.estimates_pub.publish(out)
        self._loc_index.add(self.target_class, map_x, map_y)

    def _publish_last_known_estimate(self):
        """마지막으로 알려진 위치를 estimate로 유지 퍼블리시 (confidence=0.1)."""
        if self._last_known_map_x is None:
            return

        est = ObjectEstimate()
        est.header.stamp = self.get_clock().now().to_msg()
        est.header.frame_id = self.map_frame
        est.class_id = self.target_class
        est.confidence = 0.1        # 마지막 위치 식별자
        est.depth = self._last_known_depth if self._last_known_depth is not None else -1.0
        est.angle = self._last_known_angle if self._last_known_angle is not None else 0.0
        est.map_x = self._last_known_map_x
        est.map_y = self._last_known_map_y
        est.depth_valid = (
            self._last_known_depth is not None and self._last_known_depth > 0
        )
        est.map_valid = True

        out = ObjectEstimateArray()
        out.header.stamp = est.header.stamp
        out.header.frame_id = self.map_frame
        out.estimates = [est]
        self.estimates_pub.publish(out)

    def _compute_search_angle(self, target_map_x: float, target_map_y: float) -> float | None:
        """
        맵 프레임의 예측 목표 위치(target_map_x, target_map_y)를
        로봇 기준 상대 각도(라디안)로 변환.

        결과:
          0   = 로봇 정면
          +π/2 = 로봇 왼쪽 90°
          -π/2 = 로봇 오른쪽 90°
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.map_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        # map → base_link 변환의 yaw
        yaw = 2.0 * math.atan2(qz, qw)

        # 목표를 base_link 프레임으로 변환
        dx_map = target_map_x - (-tx)   # map 원점 기준 로봇 위치 보정
        dy_map = target_map_y - (-ty)

        # 맵 → 로봇 변환: 회전 역변환
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        # 로봇이 맵 (rx, ry)에 있고 yaw 방향을 바라볼 때:
        # 목표의 로봇-상대 벡터
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        rqz = robot_tf.transform.rotation.z
        rqw = robot_tf.transform.rotation.w
        robot_yaw = 2.0 * math.atan2(rqz, rqw)

        # 목표까지의 맵 기준 방향
        dx = target_map_x - rx
        dy = target_map_y - ry
        world_angle = math.atan2(dy, dx)

        # 로봇 기준 상대 각도
        rel_angle = world_angle - robot_yaw
        # [-π, π] 정규화
        rel_angle = math.atan2(math.sin(rel_angle), math.cos(rel_angle))
        return rel_angle

    def _save_index(self):
        self._loc_index.save_if_dirty()

    # ── 핵심 로직 ─────────────────────────────────────────────────────────────

    def _process_detection(self, det, header) -> ObjectEstimate:
        """단일 Detection → ObjectEstimate 변환."""
        est = ObjectEstimate()
        est.header = header
        est.class_id = det.class_id
        est.confidence = det.confidence
        est.depth = -1.0
        est.angle = 0.0
        est.map_x = 0.0
        est.map_y = 0.0
        est.depth_valid = False
        est.map_valid = False

        # 1) bbox 중심 → 로봇 기준 수평 각도
        cx_norm = (det.x + det.w / 2.0) / self.image_width
        half_w_norm = (det.w / 2.0) / self.image_width
        # ROS 관례: x=전방 / 반시계=양수 / 우측=음수
        object_angle = -(cx_norm - 0.5) * self.camera_hfov + self.lidar_offset
        half_sector = half_w_norm * self.camera_hfov + self.sector_padding
        est.angle = object_angle

        # 2) LiDAR 깊이 측정
        raw_depth = self._measure_depth(object_angle, half_sector)
        if raw_depth > 0.0:
            depth_ema = self._get_depth_ema(det.class_id)
            smooth_depth = depth_ema.update(raw_depth)
            est.depth = smooth_depth
            est.depth_valid = True

            # 3) 맵 좌표 변환
            map_x, map_y = self._depth_to_map(smooth_depth, object_angle)
            if map_x is not None:
                est.map_x = map_x
                est.map_y = map_y
                est.map_valid = True

        return est

    def _measure_depth(self, center_angle: float, half_sector: float) -> float:
        """
        LiDAR 스캔에서 해당 각도 섹터의 유효 거리값 중
        depth_percentile 번째 값 반환. 유효값 없으면 -1.0.
        """
        scan = self._latest_scan
        if scan is None:
            return -1.0

        angle_lo = center_angle - half_sector
        angle_hi = center_angle + half_sector

        valid_ranges: list[float] = []
        for i, r in enumerate(scan.ranges):
            beam_angle = scan.angle_min + i * scan.angle_increment
            if angle_lo <= beam_angle <= angle_hi:
                if (scan.range_min <= r <= scan.range_max
                        and self.min_depth <= r <= self.max_depth):
                    valid_ranges.append(r)

        if not valid_ranges:
            return -1.0

        valid_ranges.sort()
        idx = max(0, min(
            int(len(valid_ranges) * self.depth_pct / 100),
            len(valid_ranges) - 1,
        ))
        return valid_ranges[idx]

    def _depth_to_map(
        self, depth: float, angle: float
    ) -> tuple[float | None, float | None]:
        """
        base_link 기준 (depth, angle) → map 프레임 (x, y).
        TF2 lookup 실패 시 (None, None) 반환.
        """
        x_base = depth * math.cos(angle)
        y_base = depth * math.sin(angle)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        map_x = tx + cos_y * x_base - sin_y * y_base
        map_y = ty + sin_y * x_base + cos_y * y_base
        return map_x, map_y

    def _get_depth_ema(self, class_id: int) -> DepthEMA:
        if class_id not in self._depth_ema:
            self._depth_ema[class_id] = DepthEMA(alpha=self.ema_alpha)
        return self._depth_ema[class_id]

    def _get_trajectory(self, class_id: int) -> TrajectoryTracker:
        if class_id not in self._trajectory:
            self._trajectory[class_id] = TrajectoryTracker()
        return self._trajectory[class_id]


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SlamLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._loc_index.save_if_dirty()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
