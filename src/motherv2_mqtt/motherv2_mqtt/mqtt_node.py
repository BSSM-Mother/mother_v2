import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt


VALID_COMMANDS = {'on', 'off', '0', '1'}


class MqttNode(Node):
    def __init__(self):
        super().__init__('mqtt_node')

        self.declare_parameter('mqtt_broker', 'broker.emqx.io')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'bssm/relay')
        self.declare_parameter('mqtt_client_id', 'ros2_mqtt_node')
        self.declare_parameter('reconnect_interval', 3.0)

        self._broker = str(self.get_parameter('mqtt_broker').value)
        self._port = int(self.get_parameter('mqtt_port').value)
        self._topic = str(self.get_parameter('mqtt_topic').value)
        self._client_id = str(self.get_parameter('mqtt_client_id').value)
        self._reconnect_interval = float(self.get_parameter('reconnect_interval').value)

        self._connected = False
        self._lock = threading.Lock()

        self._client = mqtt.Client(client_id=self._client_id)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect

        self._connect_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._connect_thread.start()

        # /motherv2/relay_cmd 로부터 "on" / "off" / "1" / "0" 수신
        self.sub = self.create_subscription(
            String, '/motherv2/relay_cmd', self._relay_cmd_callback, 10
        )

        # MQTT 수신 메시지를 ROS2로 브리지 (옵션)
        self._client.on_message = self._on_mqtt_message
        self.pub = self.create_publisher(String, '/motherv2/relay_state', 10)

        self.get_logger().info(
            f'MQTT node ready  broker={self._broker}:{self._port}  topic={self._topic}'
        )

    # ── MQTT 연결 ──────────────────────────────────────────────────────────────

    def _connect_loop(self):
        while rclpy.ok():
            with self._lock:
                connected = self._connected
            if not connected:
                try:
                    self._client.connect(self._broker, self._port, keepalive=60)
                    self._client.loop_start()
                    self.get_logger().info(
                        f'MQTT 연결 시도: {self._broker}:{self._port}'
                    )
                except Exception as e:
                    self.get_logger().warn(f'MQTT 연결 실패: {e}')
            time.sleep(self._reconnect_interval)

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            with self._lock:
                self._connected = True
            self._client.subscribe(self._topic)
            self.get_logger().info('MQTT 연결됨')
        else:
            self.get_logger().warn(f'MQTT 연결 거부: rc={rc}')

    def _on_disconnect(self, client, userdata, rc):
        with self._lock:
            self._connected = False
        if rc != 0:
            self.get_logger().warn(f'MQTT 연결 끊김 (rc={rc}), 재연결 대기...')

    # ── ROS2 → MQTT ───────────────────────────────────────────────────────────

    def _relay_cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd not in VALID_COMMANDS:
            self.get_logger().warn(
                f'알 수 없는 명령: "{cmd}"  (유효: on/off/0/1)'
            )
            return
        self._publish_mqtt(cmd)

    def _publish_mqtt(self, payload: str):
        with self._lock:
            connected = self._connected
        if not connected:
            self.get_logger().warn(f'MQTT 미연결 — 명령 드롭: {payload}')
            return
        result = self._client.publish(self._topic, payload)
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().info(f'MQTT 발행: {payload}  → {self._topic}')
        else:
            self.get_logger().error(f'MQTT 발행 실패: rc={result.rc}')

    # ── MQTT → ROS2 ───────────────────────────────────────────────────────────

    def _on_mqtt_message(self, client, userdata, message):
        payload = message.payload.decode('utf-8', errors='replace').strip()
        self.get_logger().info(f'MQTT 수신: {payload}  ← {message.topic}')
        out = String()
        out.data = payload
        self.pub.publish(out)

    # ── 종료 ──────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._client.loop_stop()
        self._client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
