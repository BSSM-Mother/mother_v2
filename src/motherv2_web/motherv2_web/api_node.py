import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import requests
except ImportError:
    requests = None


class ApiNode(Node):
    """API 폴링 노드.

    API_URL 환경변수가 가리키는 엔드포인트를 주기적으로 GET 요청하여
    {"follow": 0|1, "buzzer": 0|1} 응답을 읽고 ROS2 토픽으로 전달한다.

    Published topics:
      /motherv2/follow_enabled  (std_msgs/Bool)  — 팔로우 모드 on/off
      /motherv2/relay_cmd       (std_msgs/String) — "on"/"off" buzzer 명령
    """

    def __init__(self):
        super().__init__('api_node')

        self.declare_parameter('poll_interval', 1.0)
        self._interval = float(self.get_parameter('poll_interval').value)

        self._api_url = os.environ.get('API_URL', '').strip()
        if not self._api_url:
            self.get_logger().warn(
                'API_URL 환경변수가 설정되지 않았습니다. '
                'export API_URL=http://... 로 설정하세요.'
            )

        if requests is None:
            self.get_logger().error(
                'requests 패키지가 없습니다: pip install requests'
            )

        self._follow_pub = self.create_publisher(Bool, '/motherv2/follow_enabled', 1)
        self._relay_pub = self.create_publisher(String, '/motherv2/relay_cmd', 1)

        self._prev_follow = None
        self._prev_buzzer = None

        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'API polling node 시작  url={self._api_url or "(미설정)"}  '
            f'interval={self._interval}s'
        )

    def _poll_loop(self):
        while self._running and rclpy.ok():
            if self._api_url and requests is not None:
                self._fetch_and_publish()
            time.sleep(self._interval)

    def _fetch_and_publish(self):
        try:
            resp = requests.get(self._api_url, timeout=3.0)
            resp.raise_for_status()
            data = resp.json()
        except Exception as e:
            self.get_logger().warn(f'API 요청 실패: {e}')
            return

        if isinstance(data, list):
            if not data:
                self.get_logger().warn('API 응답이 빈 리스트입니다.')
                return
            data = data[0]

        if not isinstance(data, dict):
            self.get_logger().warn(f'API 응답 형식 오류: {type(data).__name__}')
            return

        follow = int(data.get('follow', 0))
        buzzer = int(data.get('buzzer', 0))

        # follow 변경 시만 발행
        if follow != self._prev_follow:
            msg = Bool()
            msg.data = bool(follow)
            self._follow_pub.publish(msg)
            self.get_logger().info(f'follow_enabled → {bool(follow)}')
            self._prev_follow = follow

        # buzzer 변경 시만 MQTT 릴레이 명령 발행
        if buzzer != self._prev_buzzer:
            cmd = String()
            cmd.data = 'on' if buzzer else 'off'
            self._relay_pub.publish(cmd)
            self.get_logger().info(f'relay_cmd → {cmd.data} (buzzer={buzzer})')
            self._prev_buzzer = buzzer

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ApiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
