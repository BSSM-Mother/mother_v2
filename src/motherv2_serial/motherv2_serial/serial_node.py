import rclpy
from rclpy.node import Node
from motherv2_interfaces.msg import MotorCommand
import serial
import signal


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('min_speed', 80)  # dead zone compensation
        self.declare_parameter('max_speed', 200)  # limit for corridor safety
        self.declare_parameter('cmd_timeout', 0.5)  # seconds without cmd → stop

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        self.min_speed = int(self.get_parameter('min_speed').value)
        self.max_speed = int(self.get_parameter('max_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'Serial opened: {port} @ {baud}')

        self.sub = self.create_subscription(
            MotorCommand, '/motherv2/cmd_motor', self.cmd_callback, 1
        )

        self.last_cmd_time = self.get_clock().now()
        self.last_sent = ''

        # Safety timer: stop if no commands received
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        # Register shutdown handler
        self._shutdown = False
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        self._shutdown = True
        self.send_stop()
        self.get_logger().info('Shutdown signal received, motors stopped')
        raise SystemExit(0)

    def compensate_speed(self, speed):
        """Apply dead zone compensation: map (1~255) to (min_speed~max_speed)."""
        if speed <= 0:
            return 0
        speed = min(speed, 255)
        ratio = speed / 255.0
        compensated = int(self.min_speed + ratio * (self.max_speed - self.min_speed))
        return min(compensated, 255)

    def send_stop(self):
        """Send stop command to STM32."""
        cmd = 's\n'
        if self.last_sent != cmd:
            self.ser.write(cmd.encode())
            self.last_sent = cmd
            self.get_logger().debug('Sent: s')

    def send_motor(self, rs, ls, rd, ld):
        """Send motor command to STM32."""
        cmd = f'{rs},{ls},{rd},{ld}\n'
        if self.last_sent != cmd:
            self.ser.write(cmd.encode())
            self.last_sent = cmd
            self.get_logger().debug(f'Sent: {cmd.strip()}')

    def cmd_callback(self, msg: MotorCommand):
        self.last_cmd_time = self.get_clock().now()

        ls = msg.left_speed
        rs = msg.right_speed
        ld = msg.left_dir
        rd = msg.right_dir

        # Both wheels stop → use 's' command
        if (ld == 0 and rd == 0) or (ls == 0 and rs == 0):
            self.send_stop()
            return

        # Apply dead zone compensation
        rs_comp = self.compensate_speed(rs)
        ls_comp = self.compensate_speed(ls)

        self.send_motor(rs_comp, ls_comp, rd, ld)

    def safety_check(self):
        """Stop motors if no command received within timeout."""
        if self._shutdown:
            return
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.send_stop()

    def destroy_node(self):
        self.send_stop()
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
