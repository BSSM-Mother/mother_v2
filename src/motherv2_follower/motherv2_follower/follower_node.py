import math
import rclpy
from rclpy.node import Node
from motherv2_interfaces.msg import DetectionArray, MotorCommand, ObjectEstimateArray
from std_msgs.msg import Bool, Float32, String
import time


class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, error):
        now = time.time()
        if self._prev_time is None:
            dt = 0.033
        else:
            dt = now - self._prev_time
            dt = max(dt, 0.001)
        self._prev_time = now

        self._integral += error * dt
        # Anti-windup
        self._integral = max(-50.0, min(50.0, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Image dimensions (must match camera node)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Angular PID (for centering person in frame)
        self.declare_parameter('angular_kp', 1.2)
        self.declare_parameter('angular_ki', 0.01)
        self.declare_parameter('angular_kd', 0.1)

        # Distance PID (for maintaining distance based on bbox height)
        self.declare_parameter('distance_kp', 0.6)
        self.declare_parameter('distance_ki', 0.005)
        self.declare_parameter('distance_kd', 0.08)

        # Target bbox height ratio (fraction of image height) — SLAM 없을 때 사용
        self.declare_parameter('target_bbox_ratio', 0.90)

        # Dead zone: don't move if error is small enough
        self.declare_parameter('angular_deadzone', 0.08)
        self.declare_parameter('distance_deadzone', 0.05)

        # Speed limits
        self.declare_parameter('max_speed', 180)
        self.declare_parameter('turn_speed', 70)
        self.declare_parameter('min_speed', 130)

        # 후진은 bbox가 target보다 이 비율 이상 초과할 때만 허용 (0.05 = 5% 초과시 후진)
        self.declare_parameter('backward_threshold', 0.03)

        # API 없이 실행 시 초기부터 follow 활성화
        self.declare_parameter('follow_default', True)

        # Lost person timeout
        self.declare_parameter('lost_timeout', 1.5)

        # Search rotation when person is lost
        self.declare_parameter('search_speed', 110)
        self.declare_parameter('search_enabled', False)

        # ── SLAM 연동 파라미터 ─────────────────────────────────────────────
        # True 이면 /motherv2/object_estimates 의 라이다 깊이로 거리 제어
        self.declare_parameter('use_slam_depth', True)
        # 라이다 깊이 기반 목표 거리 (미터)
        self.declare_parameter('target_depth_m', 0.8)
        # 라이다 깊이 deadzone (미터)
        self.declare_parameter('depth_deadzone_m', 0.05)
        # 라이다 깊이 backward_threshold (미터)
        self.declare_parameter('depth_backward_threshold_m', 0.03)
        # 로스트 상태에서 search_direction 사용 여부
        self.declare_parameter('use_slam_search', True)

        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        self.target_ratio = float(self.get_parameter('target_bbox_ratio').value)
        self.ang_deadzone = float(self.get_parameter('angular_deadzone').value)
        self.dist_deadzone = float(self.get_parameter('distance_deadzone').value)
        self.max_speed = int(self.get_parameter('max_speed').value)
        self.turn_speed = int(self.get_parameter('turn_speed').value)
        self.min_speed = int(self.get_parameter('min_speed').value)
        self.backward_threshold = float(self.get_parameter('backward_threshold').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.search_speed = int(self.get_parameter('search_speed').value)
        self.search_enabled = bool(self.get_parameter('search_enabled').value)

        self.use_slam_depth = bool(self.get_parameter('use_slam_depth').value)
        self.target_depth_m = float(self.get_parameter('target_depth_m').value)
        self.depth_deadzone_m = float(self.get_parameter('depth_deadzone_m').value)
        self.depth_backward_thresh = float(
            self.get_parameter('depth_backward_threshold_m').value)
        self.use_slam_search = bool(self.get_parameter('use_slam_search').value)

        # PID controllers
        self.angular_pid = PIDController(
            float(self.get_parameter('angular_kp').value),
            float(self.get_parameter('angular_ki').value),
            float(self.get_parameter('angular_kd').value),
            -255.0, 255.0
        )
        self.distance_pid = PIDController(
            float(self.get_parameter('distance_kp').value),
            float(self.get_parameter('distance_ki').value),
            float(self.get_parameter('distance_kd').value),
            -255.0, 255.0
        )

        # ── 퍼블리셔 / 구독 ────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(MotorCommand, '/motherv2/cmd_motor', 1)
        self.relay_pub = self.create_publisher(String, '/motherv2/relay_cmd', 1)
        self.sub = self.create_subscription(
            DetectionArray, '/motherv2/detections', self.detection_callback, 1
        )
        self.follow_sub = self.create_subscription(
            Bool, '/motherv2/follow_enabled', self._follow_enabled_callback, 1
        )
        follow_default = bool(self.get_parameter('follow_default').value)
        self._follow_enabled = follow_default

        # SLAM 노드에서 오는 라이다 깊이 + 맵 좌표 추정값
        self.estimates_sub = self.create_subscription(
            ObjectEstimateArray, '/motherv2/object_estimates',
            self._estimates_callback, 1,
        )

        # SLAM 노드에서 오는 복구 탐색 방향 (로봇 기준 라디안)
        self.search_dir_sub = self.create_subscription(
            Float32, '/motherv2/search_direction',
            self._search_dir_callback, 1,
        )

        # ── 상태 변수 ──────────────────────────────────────────────────────
        self.last_person_time = 0.0
        self.last_person_x = 0.5  # last known direction (for search rotation)

        # SLAM 데이터
        self._latest_detections = None
        self._latest_estimates: ObjectEstimateArray | None = None
        # SLAM 로스트 시 탐색 방향 (라디안, None = SLAM 미사용)
        self._search_direction: float | None = None
        self._search_dir_time: float = 0.0

        self._last_cmd_was_stop = False

        # Control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        # LED pulse every 30s while following
        self.declare_parameter('led_interval', 30.0)
        self.declare_parameter('led_pulse_duration', 1.0)
        self._led_interval = float(self.get_parameter('led_interval').value)
        self._led_pulse_duration = float(self.get_parameter('led_pulse_duration').value)
        self.led_timer = self.create_timer(self._led_interval, self._led_pulse)
        self._led_off_timer = None

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _led_pulse(self):
        if not self._follow_enabled:
            return
        on_msg = String()
        on_msg.data = 'on'
        self.relay_pub.publish(on_msg)
        self.get_logger().info('LED pulse ON')
        self._led_off_timer = self.create_timer(self._led_pulse_duration, self._led_off_once)

    def _led_off_once(self):
        off_msg = String()
        off_msg.data = 'off'
        self.relay_pub.publish(off_msg)
        self.get_logger().info('LED pulse OFF')
        self._led_off_timer.cancel()
        self._led_off_timer = None

    def detection_callback(self, msg: DetectionArray):
        self._latest_detections = msg

    def _estimates_callback(self, msg: ObjectEstimateArray):
        self._latest_estimates = msg

    def _search_dir_callback(self, msg: Float32):
        self._search_direction = msg.data
        self._search_dir_time = time.time()

    def _follow_enabled_callback(self, msg: Bool):
        if self._follow_enabled != msg.data:
            self._follow_enabled = msg.data
            self.get_logger().info(
                f'Follow mode: {"ON" if msg.data else "OFF"}'
            )

    # ── 제어 루프 ─────────────────────────────────────────────────────────────

    def _select_person(self, detections):
        """Select the largest (closest) bottle detection."""
        persons = [d for d in detections if d.class_id == 39]
        if not persons:
            # Fallback: any detected object
            persons = [d for d in detections if d.class_id >= 0]
        if not persons:
            return None
        return max(persons, key=lambda d: d.w * d.h)

    def _find_estimate(self, class_id: int):
        """ObjectEstimateArray에서 class_id 매칭 추정값 반환."""
        if self._latest_estimates is None:
            return None
        for est in self._latest_estimates.estimates:
            if est.class_id == class_id:
                return est
        return None

    def control_loop(self):
        if not self._follow_enabled:
            cmd = MotorCommand()
            self._publish_cmd(cmd)
            return

        msg = self._latest_detections
        self._latest_detections = None

        cmd = MotorCommand()
        now = time.time()

        if msg is not None and len(msg.detections) > 0:
            person = self._select_person(msg.detections)
            if person is not None:
                self.last_person_time = now
                # SLAM 추정값 조회 (있으면 깊이 기반 거리 제어)
                estimate = self._find_estimate(person.class_id)
                self._follow_person(person, cmd, estimate)
                self._publish_cmd(cmd)
                return

        # No person detected
        elapsed = now - self.last_person_time if self.last_person_time > 0 else 999.0

        if elapsed > self.lost_timeout:
            # SLAM 탐색 방향 유효 여부 확인 (2초 이내 수신)
            slam_search_valid = (
                self.use_slam_search
                and self._search_direction is not None
                and (now - self._search_dir_time) < 2.0
            )

            if slam_search_valid:
                # SLAM이 알려준 방향으로 회전 (사용자가 마지막으로 사라진 방향)
                self._slam_search_rotate(self._search_direction, cmd)
            elif self.search_enabled:
                # Fallback: 마지막 알려진 x 방향으로 회전
                self._search_rotate(cmd)
            else:
                cmd.left_speed = 0
                cmd.right_speed = 0
                cmd.left_dir = 0
                cmd.right_dir = 0
        else:
            # Brief loss: stop and wait
            cmd.left_speed = 0
            cmd.right_speed = 0
            cmd.left_dir = 0
            cmd.right_dir = 0

        self._publish_cmd(cmd)

    def _publish_cmd(self, cmd: MotorCommand):
        """STOP 커맨드는 처음 한 번만 퍼블리시해 PWM 버저 소음 방지."""
        is_stop = cmd.left_speed == 0 and cmd.right_speed == 0
        if is_stop and self._last_cmd_was_stop:
            return
        self._last_cmd_was_stop = is_stop
        self.cmd_pub.publish(cmd)

    def _follow_person(self, person, cmd: MotorCommand, estimate=None):
        # Calculate person center x as fraction of image width (0 = left, 1 = right)
        cx = (person.x + person.w / 2.0) / self.img_w
        # Angular error: positive = person is to the right, need to turn right
        ang_error = cx - 0.5

        # ── 거리 오차 계산 ────────────────────────────────────────────────
        # SLAM 라이다 깊이 사용 가능 여부 판단
        use_depth = (
            self.use_slam_depth
            and estimate is not None
            and estimate.depth_valid
            and estimate.depth > 0.0
        )

        if use_depth:
            # 실제 거리 기반 오차 (미터 단위)
            depth_err_abs = estimate.depth - self.target_depth_m
            # PID 입력 정규화 (÷ target_depth 로 무차원화)
            dist_error = depth_err_abs / self.target_depth_m
            dist_deadzone = self.depth_deadzone_m / self.target_depth_m
            dist_backward_thresh = self.depth_backward_thresh / self.target_depth_m
            self.get_logger().info(
                f'[SLAM] depth={estimate.depth:.2f}m '
                f'target={self.target_depth_m:.2f}m '
                f'err={depth_err_abs:.3f}m'
            )
        else:
            # bbox 높이 비율 기반 오차 (SLAM 없을 때 fallback)
            bbox_ratio = person.h / self.img_h
            dist_error = self.target_ratio - bbox_ratio
            dist_deadzone = self.dist_deadzone
            dist_backward_thresh = self.backward_threshold

        self.last_person_x = cx

        if abs(ang_error) < self.ang_deadzone:
            side = 'CENTER'
        elif ang_error > 0:
            side = 'RIGHT'
        else:
            side = 'LEFT'
        self.get_logger().info(
            f'Object {side} (cx={cx:.2f}, ang_err={ang_error:.2f}) | '
            f'dist_err={dist_error:.3f}'
        )

        # Check dead zones
        in_angular_deadzone = abs(ang_error) < self.ang_deadzone
        in_distance_deadzone = abs(dist_error) < dist_deadzone

        if in_angular_deadzone and in_distance_deadzone:
            cmd.left_speed = 0
            cmd.right_speed = 0
            cmd.left_dir = 0
            cmd.right_dir = 0
            self.angular_pid.reset()
            self.distance_pid.reset()
            return

        # Compute PID outputs
        angular_output = self.angular_pid.compute(ang_error)
        distance_output = self.distance_pid.compute(dist_error)

        # If angular error is large, rotation only
        if abs(ang_error) > 0.25:
            in_distance_deadzone = True

        # backward threshold 처리
        if dist_error < 0 and abs(dist_error) < dist_backward_thresh:
            in_distance_deadzone = True

        if not in_angular_deadzone and in_distance_deadzone:
            self._apply_rotation(angular_output, cmd)
        elif in_angular_deadzone and not in_distance_deadzone:
            self._apply_linear(distance_output, cmd)
        else:
            self._apply_mixed(angular_output, distance_output, cmd)

    def _curve_speed(self, raw_output, max_speed):
        """PID output [0, 255] → [min_speed, max_speed] with smoothstep S-curve."""
        t = min(abs(raw_output) / 255.0, 1.0)
        curved = 3 * t**2 - 2 * t**3
        return int(self.min_speed + (max_speed - self.min_speed) * curved)

    def _apply_rotation(self, angular_output, cmd: MotorCommand):
        """Rotate in place: differential drive."""
        speed = self._curve_speed(angular_output, self.turn_speed)

        if angular_output > 0:
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 1
            cmd.right_dir = 2
            self.get_logger().info(f'Rotate RIGHT (angular={angular_output:.2f}, speed={speed})')
        else:
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 2
            cmd.right_dir = 1
            self.get_logger().info(f'Rotate LEFT (angular={angular_output:.2f}, speed={speed})')

    def _apply_linear(self, distance_output, cmd: MotorCommand):
        """Move forward or backward."""
        speed = self._curve_speed(distance_output, self.max_speed)

        if distance_output > 0:
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 1
            cmd.right_dir = 1
        else:
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 2
            cmd.right_dir = 2

    def _apply_mixed(self, angular_output, distance_output, cmd: MotorCommand):
        """Combine rotation and linear motion: arc movement."""
        base_speed = self._curve_speed(distance_output, self.max_speed)
        turn_ratio = angular_output / 255.0
        turn_delta = int(turn_ratio * self.turn_speed)

        left_speed = base_speed + turn_delta
        right_speed = base_speed - turn_delta

        if distance_output > 0:
            base_dir = 1
        else:
            base_dir = 2

        if left_speed >= 0:
            cmd.left_dir = base_dir
            cmd.left_speed = int(min(abs(left_speed), self.max_speed))
        else:
            cmd.left_dir = 2 if base_dir == 1 else 1
            cmd.left_speed = int(min(abs(left_speed), self.max_speed))

        if right_speed >= 0:
            cmd.right_dir = base_dir
            cmd.right_speed = int(min(abs(right_speed), self.max_speed))
        else:
            cmd.right_dir = 2 if base_dir == 1 else 1
            cmd.right_speed = int(min(abs(right_speed), self.max_speed))

    def _slam_search_rotate(self, search_angle: float, cmd: MotorCommand):
        """
        SLAM 예측 방향(search_angle)으로 회전.
        search_angle: 로봇 기준 라디안 (양수=좌, 음수=우, ROS 관례)
        deadzone 이내면 직진 탐색.
        """
        SEARCH_DEADZONE = 0.15  # 라디안 (~8.6°)

        if abs(search_angle) < SEARCH_DEADZONE:
            # 예측 방향이 정면 — 직진
            cmd.left_speed = self.search_speed
            cmd.right_speed = self.search_speed
            cmd.left_dir = 1
            cmd.right_dir = 1
            self.get_logger().info(
                f'[SLAM SEARCH] 직진 탐색 (angle={math.degrees(search_angle):.1f}°)')
        elif search_angle > 0:
            # 왼쪽으로 회전
            cmd.left_speed = self.search_speed
            cmd.right_speed = self.search_speed
            cmd.left_dir = 2
            cmd.right_dir = 1
            self.get_logger().info(
                f'[SLAM SEARCH] 좌회전 (angle={math.degrees(search_angle):.1f}°)')
        else:
            # 오른쪽으로 회전
            cmd.left_speed = self.search_speed
            cmd.right_speed = self.search_speed
            cmd.left_dir = 1
            cmd.right_dir = 2
            self.get_logger().info(
                f'[SLAM SEARCH] 우회전 (angle={math.degrees(search_angle):.1f}°)')

    def _search_rotate(self, cmd: MotorCommand):
        """Rotate to search for lost person (SLAM 미사용 fallback)."""
        if self.last_person_x >= 0.5:
            cmd.left_speed = self.search_speed
            cmd.right_speed = self.search_speed
            cmd.left_dir = 1
            cmd.right_dir = 2
        else:
            cmd.left_speed = self.search_speed
            cmd.right_speed = self.search_speed
            cmd.left_dir = 2
            cmd.right_dir = 1


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
