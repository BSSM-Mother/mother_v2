import rclpy
from rclpy.node import Node
from motherv2_interfaces.msg import DetectionArray, MotorCommand
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

        # Target bbox height ratio (fraction of image height)
        self.declare_parameter('target_bbox_ratio', 0.90)

        # Dead zone: don't move if error is small enough
        self.declare_parameter('angular_deadzone', 0.08)
        self.declare_parameter('distance_deadzone', 0.08)

        # Speed limits
        self.declare_parameter('max_speed', 180)
        self.declare_parameter('turn_speed', 70)
        self.declare_parameter('min_speed', 130)

        # 후진은 bbox가 target보다 이 비율 이상 초과할 때만 허용 (0.05 = 5% 초과시 후진)
        self.declare_parameter('backward_threshold', 0.05)

        # Lost person timeout
        self.declare_parameter('lost_timeout', 1.5)

        # Search rotation when person is lost
        self.declare_parameter('search_speed', 110)
        self.declare_parameter('search_enabled', False)

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

        self.cmd_pub = self.create_publisher(MotorCommand, '/motherv2/cmd_motor', 1)
        self.sub = self.create_subscription(
            DetectionArray, '/motherv2/detections', self.detection_callback, 1
        )

        self.last_person_time = 0.0
        self.last_person_x = 0.5  # last known direction (for search rotation)

        # Control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self._latest_detections = None
        self._last_cmd_was_stop = False

    def detection_callback(self, msg: DetectionArray):
        self._latest_detections = msg

    def _select_person(self, detections):
        """Select the largest (closest) bottle detection."""
        persons = [d for d in detections if d.class_id == 39]
        if not persons:
            # Fallback: any detected object
            persons = [d for d in detections if d.class_id >= 0]
        if not persons:
            return None
        return max(persons, key=lambda d: d.w * d.h)

    def control_loop(self):
        msg = self._latest_detections
        self._latest_detections = None

        cmd = MotorCommand()
        now = time.time()

        if msg is not None and len(msg.detections) > 0:
            person = self._select_person(msg.detections)
            if person is not None:
                self.last_person_time = now
                self._follow_person(person, cmd)
                self._publish_cmd(cmd)
                return

        # No person detected
        elapsed = now - self.last_person_time if self.last_person_time > 0 else 999.0

        if elapsed > self.lost_timeout and self.search_enabled:
            # Search: rotate toward last known direction
            self._search_rotate(cmd)
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

    def _follow_person(self, person, cmd: MotorCommand):
        # Calculate person center x as fraction of image width (0 = left, 1 = right)
        cx = (person.x + person.w / 2.0) / self.img_w
        # Angular error: positive = person is to the right, need to turn right
        ang_error = cx - 0.5

        # Distance: bbox height as fraction of image height
        bbox_ratio = person.h / self.img_h
        # Distance error: positive = too far (bbox too small), negative = too close
        dist_error = self.target_ratio - bbox_ratio

        self.last_person_x = cx

        if abs(ang_error) < self.ang_deadzone:
            side = 'CENTER'
        elif ang_error > 0:
            side = 'RIGHT'
        else:
            side = 'LEFT'
        self.get_logger().info(
            f'Object {side} (cx={cx:.2f}, ang_err={ang_error:.2f}) | '
            f'bbox_ratio={bbox_ratio:.2f}, dist_err={dist_error:.2f}'
        )

        # Check dead zones
        in_angular_deadzone = abs(ang_error) < self.ang_deadzone
        in_distance_deadzone = abs(dist_error) < self.dist_deadzone

        if in_angular_deadzone and in_distance_deadzone:
            # Person is centered and at right distance → stop
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

        # If angular error is large, rotation only (don't lurch forward while object is at edge)
        if abs(ang_error) > 0.25:
            in_distance_deadzone = True

        # bbox가 backward_threshold 이내로만 가까우면 후진 안 함 (deadzone 처리)
        # dist_error < 0 이고 abs(dist_error) < backward_threshold → 그냥 멈춤
        if dist_error < 0 and abs(dist_error) < self.backward_threshold:
            in_distance_deadzone = True

        if not in_angular_deadzone and in_distance_deadzone:
            # Priority: rotate toward person (no forward/backward)
            self._apply_rotation(angular_output, cmd)
        elif in_angular_deadzone and not in_distance_deadzone:
            # Move forward to adjust distance
            self._apply_linear(distance_output, cmd)
        else:
            # Both: mix rotation and linear
            self._apply_mixed(angular_output, distance_output, cmd)

    def _apply_rotation(self, angular_output, cmd: MotorCommand):
        """Rotate in place: differential drive."""
        speed = int(min(abs(angular_output) * self.turn_speed / 255.0 * 255, self.turn_speed))
        speed = max(self.min_speed, speed)

        if angular_output > 0:
            # Turn right: left wheel forward (1), right wheel backward (2)
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 1
            cmd.right_dir = 2
            self.get_logger().info(f'Rotate RIGHT (angular={angular_output:.2f}, speed={speed})')
        else:
            # Turn left: left wheel backward (2), right wheel forward (1)
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 2
            cmd.right_dir = 1
            self.get_logger().info(f'Rotate LEFT (angular={angular_output:.2f}, speed={speed})')

    def _apply_linear(self, distance_output, cmd: MotorCommand):
        """Move forward or backward."""
        speed = int(min(abs(distance_output) * self.max_speed / 255.0 * 255, self.max_speed))
        speed = max(self.min_speed, speed)

        if distance_output > 0:
            # Too far → move forward
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 1
            cmd.right_dir = 1
        else:
            # Too close → move backward
            cmd.left_speed = speed
            cmd.right_speed = speed
            cmd.left_dir = 2
            cmd.right_dir = 2

    def _apply_mixed(self, angular_output, distance_output, cmd: MotorCommand):
        """Combine rotation and linear motion: arc movement."""
        base_speed = abs(distance_output) * self.max_speed
        turn_factor = angular_output * self.turn_speed

        left_speed = base_speed + turn_factor
        right_speed = base_speed - turn_factor

        # Determine direction based on distance
        if distance_output > 0:
            base_dir = 1  # forward
        else:
            base_dir = 2  # backward

        # Handle speed sign (if a wheel speed goes negative, reverse its direction)
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

        cmd.left_speed = max(self.min_speed, cmd.left_speed)
        cmd.right_speed = max(self.min_speed, cmd.right_speed)

    def _search_rotate(self, cmd: MotorCommand):
        """Rotate to search for lost person."""
        if self.last_person_x >= 0.5:
            # Last seen on right → rotate right
            cmd.left_speed = self.search_speed
            cmd.right_speed = self.search_speed
            cmd.left_dir = 1
            cmd.right_dir = 2
        else:
            # Last seen on left → rotate left
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
