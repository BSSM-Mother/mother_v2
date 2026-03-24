import subprocess
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('flip', False)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.flip = bool(self.get_parameter('flip').value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/motherv2/image_raw', 1)

        # Use rpicam-vid to capture BGR frames via stdout pipe
        frame_size = self.width * self.height * 3
        self._frame_size = frame_size

        # rpicam-vid: output raw BGR to stdout
        # --hflip --vflip = 180° rotation
        cmd = [
            'rpicam-vid',
            '--camera', str(int(self.get_parameter('device_id').value)),
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.fps),
            '--codec', 'yuv420',
            '--timeout', '0',
            '--nopreview',
            '-o', '-',
        ]
        if self.flip:
            cmd += ['--hflip', '--vflip']

        self.get_logger().info(f'Starting: {" ".join(cmd)}')

        self._process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
        )

        # YUV420 frame size: width * height * 1.5
        self._yuv_size = self.width * self.height * 3 // 2

        # Capture thread
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.get_logger().info(
            f'Camera started: {self.width}x{self.height} @ {self.fps}fps, flip={self.flip}'
        )

        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.timer_callback)

    def _capture_loop(self):
        while self._running:
            raw = self._process.stdout.read(self._yuv_size)
            if len(raw) != self._yuv_size:
                if not self._running:
                    break
                continue

            yuv = np.frombuffer(raw, dtype=np.uint8).reshape(
                (self.height * 3 // 2, self.width)
            )
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)

            with self._frame_lock:
                self._latest_frame = bgr

    def timer_callback(self):
        with self._frame_lock:
            frame = self._latest_frame
            self._latest_frame = None

        if frame is None:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self._running = False
        try:
            self._process.terminate()
            self._process.wait(timeout=3)
        except Exception:
            self._process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
