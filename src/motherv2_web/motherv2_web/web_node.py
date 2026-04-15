import base64
import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from motherv2_interfaces.msg import DetectionArray, MotorCommand, ObjectEstimateArray
from std_msgs.msg import Bool, String


# ──────────────────────────────────────────────────────────────────────────────
# HTTP 핸들러
# ──────────────────────────────────────────────────────────────────────────────
class MJPEGHandler(BaseHTTPRequestHandler):
    latest_frame = None
    frame_lock = threading.Lock()
    motor_state = {'left_speed': 0, 'left_dir': 0, 'right_speed': 0, 'right_dir': 0, 'state': 'STOP'}
    motor_lock = threading.Lock()
    status_state = {'follow_enabled': True, 'detecting': False}
    status_lock = threading.Lock()

    # SLAM 시각화용 공유 상태
    slam_map_jpeg = None       # 맵 이미지 JPEG bytes
    slam_map_lock = threading.Lock()
    slam_state = {}            # JSON-serializable dict
    slam_state_lock = threading.Lock()

    # 장소 인덱스
    _places_file = os.path.expanduser('~/.ros/motherv2/places.json')
    places_data: list = []
    places_lock = threading.Lock()

    # 로봇 이벤트 로그
    _log_entries: deque = deque(maxlen=100)
    _log_lock = threading.Lock()

    @classmethod
    def push_log(cls, level: str, msg: str):
        ts = time.strftime('%H:%M:%S')
        with cls._log_lock:
            cls._log_entries.append({'t': ts, 'l': level, 'm': msg})

    @classmethod
    def load_places(cls):
        try:
            if os.path.exists(cls._places_file):
                with open(cls._places_file, 'r', encoding='utf-8') as f:
                    cls.places_data = json.load(f)
        except Exception:
            cls.places_data = []

    @classmethod
    def _save_places(cls):
        try:
            os.makedirs(os.path.dirname(cls._places_file), exist_ok=True)
            with open(cls._places_file, 'w', encoding='utf-8') as f:
                json.dump(cls.places_data, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/stream':
            self._stream()
        elif self.path == '/snapshot':
            self._snapshot()
        elif self.path == '/motor':
            self._motor_json()
        elif self.path == '/state':
            self._state_json()
        elif self.path == '/slam_map':
            self._slam_map()
        elif self.path == '/slam_state':
            self._slam_state_json()
        elif self.path == '/places':
            self._places_json()
        elif self.path == '/log':
            self._log_json()
        else:
            self._index()

    def do_POST(self):
        if self.path == '/add_place':
            self._add_place()
        elif self.path == '/delete_place':
            self._delete_place()
        elif self.path == '/save_map':
            self._save_map_handler()
        else:
            self.send_error(404)

    # ── 기존 엔드포인트 ───────────────────────────────────────────────────

    def _index(self):
        html = _INDEX_HTML.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _snapshot(self):
        with self.frame_lock:
            frame = self.latest_frame
        if frame is None:
            self.send_error(503, 'No frame available')
            return
        _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        data = jpeg.tobytes()
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _motor_json(self):
        with self.motor_lock:
            data = dict(self.motor_state)
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _state_json(self):
        with self.motor_lock:
            data = dict(self.motor_state)
        with self.status_lock:
            data.update(self.status_state)
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                with self.frame_lock:
                    frame = self.latest_frame
                if frame is not None:
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    data = jpeg.tobytes()
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(data)}\r\n\r\n'.encode())
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError):
            pass

    # ── SLAM 엔드포인트 ───────────────────────────────────────────────────

    def _slam_map(self):
        """점유 격자 맵 이미지 (JPEG)."""
        with self.slam_map_lock:
            data = self.slam_map_jpeg
        if data is None:
            self.send_error(503, 'No SLAM map available')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(data)

    def _slam_state_json(self):
        """로봇/객체/예측 위치 JSON."""
        with self.slam_state_lock:
            data = dict(self.slam_state)
        # 장소 인덱스 포함
        with self.places_lock:
            data['places'] = list(self.places_data)
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(body)

    def _places_json(self):
        with self.places_lock:
            data = list(self.places_data)
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _log_json(self):
        with self._log_lock:
            data = list(self._log_entries)
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(body)

    def _json_response(self, data: dict, status: int = 200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(length)
        return json.loads(raw)

    def _add_place(self):
        """현재 로봇 위치를 이름으로 저장."""
        try:
            data = self._read_json_body()
            name = str(data.get('name', '')).strip()
        except Exception:
            self._json_response({'ok': False, 'error': 'bad json'}, 400)
            return
        if not name:
            self._json_response({'ok': False, 'error': 'name required'}, 400)
            return
        with self.slam_state_lock:
            robot = self.slam_state.get('robot', {})
        if not robot.get('valid'):
            self._json_response({'ok': False, 'error': 'robot TF not available'}, 503)
            return
        place = {
            'name': name,
            'x': round(robot['x'], 3),
            'y': round(robot['y'], 3),
            'time': int(time.time()),
        }
        with self.places_lock:
            # 같은 이름 있으면 덮어쓰기
            self.places_data = [p for p in self.places_data if p['name'] != name]
            self.places_data.append(place)
            self._save_places()
        MJPEGHandler.push_log('PLACE', f'Indexed "{name}" at ({place["x"]}, {place["y"]})')
        self._json_response({'ok': True, 'place': place})

    def _delete_place(self):
        """이름으로 장소 삭제."""
        try:
            data = self._read_json_body()
            name = str(data.get('name', '')).strip()
        except Exception:
            self._json_response({'ok': False, 'error': 'bad json'}, 400)
            return
        with self.places_lock:
            before = len(self.places_data)
            self.places_data = [p for p in self.places_data if p['name'] != name]
            self._save_places()
            deleted = before - len(self.places_data)
        self._json_response({'ok': True, 'deleted': deleted})

    def _save_map_handler(self):
        """slam_toolbox로 맵 저장."""
        map_dir = os.path.expanduser('~/maps')
        os.makedirs(map_dir, exist_ok=True)
        map_path = os.path.join(map_dir, 'slam_map')
        try:
            result = subprocess.run(
                ['ros2', 'service', 'call',
                 '/slam_toolbox/serialize_map',
                 'slam_toolbox/srv/SerializePoseGraph',
                 f'{{filename: "{map_path}"}}'],
                capture_output=True, text=True, timeout=15,
            )
            ok = result.returncode == 0
            self._json_response({'ok': ok, 'path': map_path,
                                  'msg': (result.stdout if ok else result.stderr)[:200]})
        except Exception as e:
            self._json_response({'ok': False, 'error': str(e)}, 500)


# ──────────────────────────────────────────────────────────────────────────────
# 메인 노드
# ──────────────────────────────────────────────────────────────────────────────
class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('stream_width', 640)
        port = int(self.get_parameter('port').value)
        self.stream_width = int(self.get_parameter('stream_width').value)

        self.bridge = CvBridge()

        # ── 카메라/디텍션 상태 ─────────────────────────────────────────────
        self._det_lock = threading.Lock()
        self._det_boxes = []
        self._det_time = 0.0
        self._det_timeout = 1.0

        # ── SLAM 상태 ─────────────────────────────────────────────────────
        self._map_lock = threading.Lock()
        self._occupancy_grid = None        # nav_msgs/OccupancyGrid

        self._slam_obj_lock = threading.Lock()
        self._slam_objects = []             # list of dict
        self._slam_prediction = None        # dict {x, y}
        self._slam_trajectory = deque(maxlen=200)

        self._robot_pose_lock = threading.Lock()
        self._robot_pose = None             # (x, y, yaw)

        # ── LiDAR 스캔 ────────────────────────────────────────────────────
        self._scan_lock = threading.Lock()
        self._scan_points = []              # list of [x, y] (robot-relative, downsampled)

        # ── 맵 b64 전송 주기 카운터 ─────────────────────────────────────────
        self._slam_state_tick = 0

        # ── TF2 ────────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── QoS ────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── 구독: 기존 ──────────────────────────────────────────────────────
        self.cam_sub = self.create_subscription(
            Image, '/motherv2/image_raw', self.camera_callback, sensor_qos)
        self.det_sub = self.create_subscription(
            DetectionArray, '/motherv2/detections', self.detection_callback, 1)
        self.motor_sub = self.create_subscription(
            MotorCommand, '/motherv2/cmd_motor', self.motor_callback, 1)
        # follow 상태 구독 → 웹 상태 표시용
        self.follow_sub = self.create_subscription(
            Bool, '/motherv2/follow_enabled', self.follow_callback, 1
        )
        # MQTT 릴레이 명령 구독 → 로그용
        self.relay_sub = self.create_subscription(
            String, '/motherv2/relay_cmd', self._relay_callback, 1
        )

        # ── 구독: SLAM ──────────────────────────────────────────────────────
        # slam_toolbox는 /map을 TRANSIENT_LOCAL(latching)로 퍼블리시함.
        # 구독자도 TRANSIENT_LOCAL이어야 연결되며, 노드 시작 전에 이미
        # 발행된 맵도 즉시 수신할 수 있음.
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, sensor_qos)
        self.estimates_sub = self.create_subscription(
            ObjectEstimateArray, '/motherv2/object_estimates',
            self._estimates_callback, 1)
        self.prediction_sub = self.create_subscription(
            PoseStamped, '/motherv2/target_prediction',
            self._prediction_callback, 1)

        # ── 타이머 ────────────────────────────────────────────────────────
        # 맵 이미지 렌더링 (2초 주기 — 맵은 천천히 변함)
        self.create_timer(2.0, self._render_map_image)
        # SLAM 상태 JSON 업데이트 (5Hz)
        self.create_timer(0.2, self._update_slam_state)
        # 로봇 포즈 TF 조회 (10Hz)
        self.create_timer(0.1, self._update_robot_pose)

        # ── HTTP 서버 ──────────────────────────────────────────────────────
        MJPEGHandler.load_places()
        self.server = ThreadingHTTPServer(('0.0.0.0', port), MJPEGHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f'Web server started on port {port}')

    # ── 기존 콜백 ─────────────────────────────────────────────────────────

    def follow_callback(self, msg: Bool):
        val = bool(msg.data)
        with MJPEGHandler.status_lock:
            prev = MJPEGHandler.status_state.get('follow_enabled')
            MJPEGHandler.status_state['follow_enabled'] = val
        if prev != val:
            MJPEGHandler.push_log('FOLLOW', f'Follow {"ON" if val else "OFF"}')

    def _relay_callback(self, msg: String):
        MJPEGHandler.push_log('MQTT', f'Relay → {msg.data.upper()}')

    def detection_callback(self, msg: DetectionArray):
        boxes = []
        for det in msg.detections:
            boxes.append((det.x, det.y, det.w, det.h, det.confidence, det.class_id))
        with self._det_lock:
            self._det_boxes = boxes
            self._det_time = time.time()
        with MJPEGHandler.status_lock:
            prev = MJPEGHandler.status_state.get('detecting')
            detecting = len(boxes) > 0
            MJPEGHandler.status_state['detecting'] = detecting
        if prev != detecting:
            MJPEGHandler.push_log('DETECT', f'Object {"detected" if detecting else "lost"}')

    def motor_callback(self, msg: MotorCommand):
        ls, ld = msg.left_speed, msg.left_dir
        rs, rd = msg.right_speed, msg.right_dir
        if ls == 0 and rs == 0:
            state = 'STOP'
        elif ld == 1 and rd == 1:
            state = 'FORWARD'
        elif ld == 2 and rd == 2:
            state = 'BACKWARD'
        elif ld == 1 and rd == 2:
            state = 'ROTATE_R'
        elif ld == 2 and rd == 1:
            state = 'ROTATE_L'
        else:
            state = 'ARC'
        data = {'left_speed': ls, 'left_dir': ld, 'right_speed': rs, 'right_dir': rd, 'state': state}
        with MJPEGHandler.motor_lock:
            prev_state = MJPEGHandler.motor_state.get('state')
            MJPEGHandler.motor_state = data
        if prev_state != state:
            MJPEGHandler.push_log('MOTOR', f'Motor → {state}')

    def camera_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        if w > self.stream_width:
            scale = self.stream_width / w
            frame = cv2.resize(frame, (self.stream_width, int(h * scale)))

        with self._det_lock:
            timed_out = (time.time() - self._det_time) >= self._det_timeout
            boxes = [] if timed_out else self._det_boxes

        if timed_out:
            with MJPEGHandler.status_lock:
                MJPEGHandler.status_state['detecting'] = False

        scale_x = frame.shape[1] / w if w > self.stream_width else 1.0
        scale_y = frame.shape[0] / h if w > self.stream_width else 1.0

        for (bx, by, bw, bh, conf, cls_id) in boxes:
            ix = int(bx * scale_x)
            iy = int(by * scale_y)
            iw = int(bw * scale_x)
            ih = int(bh * scale_y)
            cv2.rectangle(frame, (ix, iy), (ix + iw, iy + ih), (0, 255, 0), 2)
            cv2.putText(
                frame, f'{conf:.2f}',
                (ix, max(iy - 5, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )

        with MJPEGHandler.frame_lock:
            MJPEGHandler.latest_frame = frame

    # ── SLAM 콜백 ─────────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        with self._map_lock:
            first = self._occupancy_grid is None
            self._occupancy_grid = msg
        if first:
            MJPEGHandler.push_log('SLAM', f'Map received ({msg.info.width}x{msg.info.height}, res={msg.info.resolution:.2f}m)')

    def _scan_callback(self, msg: LaserScan):
        """LiDAR 스캔 → 로봇 기준 XY 포인트로 변환 (180포인트로 다운샘플)."""
        pts = []
        total = len(msg.ranges)
        step = max(1, total // 180)
        for i in range(0, total, step):
            r = msg.ranges[i]
            if msg.range_min <= r <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                pts.append([round(r * math.cos(angle), 2),
                             round(r * math.sin(angle), 2)])
        with self._scan_lock:
            self._scan_points = pts

    def _estimates_callback(self, msg: ObjectEstimateArray):
        objs = []
        for est in msg.estimates:
            obj = {
                'class_id': est.class_id,
                'confidence': round(est.confidence, 2),
                'depth': round(est.depth, 3),
                'angle': round(est.angle, 3),
                'map_x': round(est.map_x, 3),
                'map_y': round(est.map_y, 3),
                'depth_valid': est.depth_valid,
                'map_valid': est.map_valid,
            }
            objs.append(obj)
            # 궤적 누적 (맵 좌표 유효 시)
            if est.map_valid:
                self._slam_trajectory.append((est.map_x, est.map_y))
        with self._slam_obj_lock:
            self._slam_objects = objs

    def _prediction_callback(self, msg: PoseStamped):
        with self._slam_obj_lock:
            self._slam_prediction = {
                'x': round(msg.pose.position.x, 3),
                'y': round(msg.pose.position.y, 3),
            }

    def _update_robot_pose(self):
        """TF2로 map → base_link 조회하여 로봇 포즈 갱신."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)
            with self._robot_pose_lock:
                self._robot_pose = (x, y, yaw)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    # ── SLAM 렌더링 ───────────────────────────────────────────────────────

    def _render_map_image(self):
        """OccupancyGrid → 크롭된 JPEG 이미지로 변환."""
        with self._map_lock:
            grid = self._occupancy_grid
        if grid is None:
            return

        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        # OccupancyGrid → numpy (row-major, 행=y 아래→위)
        raw = np.array(grid.data, dtype=np.int8).reshape((h, w))

        # 탐색된 영역(알려진 셀) 바운딩박스 찾기
        known_mask = raw >= 0
        if not np.any(known_mask):
            return

        rows = np.any(known_mask, axis=1)
        cols = np.any(known_mask, axis=0)
        r_min, r_max = np.where(rows)[0][[0, -1]]
        c_min, c_max = np.where(cols)[0][[0, -1]]

        # 패딩 추가
        pad = 20
        r_min = max(0, r_min - pad)
        r_max = min(h - 1, r_max + pad)
        c_min = max(0, c_min - pad)
        c_max = min(w - 1, c_max + pad)

        cropped = raw[r_min:r_max + 1, c_min:c_max + 1]
        ch, cw = cropped.shape

        # 색상 변환 (BGR)
        # 미지: #161616 (매우 어두움), 빈 공간: #c8c8c8 (밝은 회색), 점유: #00cc44 (초록)
        img = np.zeros((ch, cw, 3), dtype=np.uint8)
        unknown = cropped < 0
        free = cropped == 0
        occupied = cropped > 0

        img[unknown] = (20, 20, 20)      # 검정 (미탐색)
        img[free] = (200, 200, 200)      # 밝은 회색 (탐색된 빈 공간)
        img[occupied] = (20, 20, 20)     # 검정 (벽/장애물) — unknown과 동일

        # y축 뒤집기 (ROS 맵: 아래→위, 이미지: 위→아래)
        img = cv2.flip(img, 0)

        # 적절한 크기로 리사이즈 (최대 400px)
        max_dim = 400
        scale = min(max_dim / max(ch, 1), max_dim / max(cw, 1), 4.0)
        if scale != 1.0:
            img = cv2.resize(img, (int(cw * scale), int(ch * scale)),
                             interpolation=cv2.INTER_NEAREST)

        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])

        # 크롭 메타데이터 (JS에서 좌표 변환에 사용)
        # 크롭된 이미지의 맵 좌표 범위
        crop_origin_x = ox + c_min * res
        crop_origin_y = oy + r_min * res
        crop_width_m = cw * res
        crop_height_m = ch * res

        with MJPEGHandler.slam_map_lock:
            MJPEGHandler.slam_map_jpeg = jpeg.tobytes()

        # 맵 메타 정보를 slam_state에서 사용하도록 저장
        self._map_render_info = {
            'img_w': int(cw * scale),
            'img_h': int(ch * scale),
            'crop_origin_x': round(crop_origin_x, 4),
            'crop_origin_y': round(crop_origin_y, 4),
            'crop_w_m': round(crop_width_m, 4),
            'crop_h_m': round(crop_height_m, 4),
            'resolution': res,
            'available': True,
        }

    def _update_slam_state(self):
        """SLAM 상태 JSON 업데이트 (로봇 포즈, 객체, 예측, 궤적, 스캔)."""
        with self._robot_pose_lock:
            rp = self._robot_pose

        with self._slam_obj_lock:
            objs = list(self._slam_objects)
            pred = self._slam_prediction

        with self._scan_lock:
            scan_pts = list(self._scan_points)

        traj = list(self._slam_trajectory)
        map_info = getattr(self, '_map_render_info', {'available': False})

        # 맵 이미지를 2초마다 (10틱) base64로 포함 — 별도 /slam_map 요청 불필요
        self._slam_state_tick += 1
        map_b64 = None
        if self._slam_state_tick % 10 == 1:  # 첫 틱 포함하여 10틱마다 전송
            with MJPEGHandler.slam_map_lock:
                jpeg_data = MJPEGHandler.slam_map_jpeg
            if jpeg_data is not None:
                map_b64 = base64.b64encode(jpeg_data).decode('ascii')

        state = {
            'map_info': map_info,
            'map_b64': map_b64,
            'robot': {
                'x': round(rp[0], 3) if rp else 0,
                'y': round(rp[1], 3) if rp else 0,
                'yaw': round(rp[2], 3) if rp else 0,
                'valid': rp is not None,
            },
            'objects': objs,
            'prediction': pred,
            'trajectory': [[round(x, 3), round(y, 3)] for x, y in traj[-100:]],
            'scan': scan_pts,
        }

        with MJPEGHandler.slam_state_lock:
            MJPEGHandler.slam_state = state

    def destroy_node(self):
        self.server.shutdown()
        # TF2 리스너 스레드를 먼저 정리해야 SIGINT 시 SIGABRT 방지
        self.tf_listener = None
        self.tf_buffer = None
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
# HTML / JS 대시보드
# ──────────────────────────────────────────────────────────────────────────────
_INDEX_HTML = '''<!DOCTYPE html>
<html>
<head>
<title>MotherV2 Dashboard</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box}
body{background:#111;color:#eee;font-family:'Courier New',monospace;margin:0;padding:16px;text-align:center}
h1{color:#0f0;margin:0 0 12px;font-size:1.3em}
.panels{display:flex;flex-wrap:wrap;justify-content:center;gap:16px;max-width:1100px;margin:0 auto}
.panel{background:#1a1a1a;border:1px solid #333;border-radius:8px;padding:12px;flex-shrink:0}
.panel h2{color:#888;font-size:.7em;margin:0 0 8px;text-transform:uppercase;letter-spacing:2px}
#stream-img{max-width:100%;border-radius:6px;display:block}
canvas{display:block;margin:0 auto;border-radius:6px}
#slam-canvas{background:#161616;border:1px solid #444}
.slam-info{font-size:.75em;color:#888;margin-top:8px;text-align:left;line-height:1.6}
.slam-info span{color:#0f0}
.legend{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;font-size:.65em;color:#aaa;margin-top:6px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:3px;vertical-align:middle}
.place-ctrl{display:flex;gap:6px;margin-top:10px;align-items:center;flex-wrap:wrap}
.place-ctrl input{background:#222;border:1px solid #444;color:#eee;padding:5px 8px;border-radius:4px;font-family:monospace;font-size:.8em;flex:1;min-width:120px}
.place-ctrl button{background:#1e3a1e;border:1px solid #2a5a2a;color:#0f0;padding:5px 10px;border-radius:4px;cursor:pointer;font-size:.8em;white-space:nowrap}
.place-ctrl button:hover{background:#2a5a2a}
#place-list{margin-top:8px;font-size:.72em;color:#aaa;text-align:left;max-height:80px;overflow-y:auto}
#place-list .pi{display:flex;justify-content:space-between;padding:2px 0;border-bottom:1px solid #222}
#place-list .pi .pname{color:#aa66ff}
#place-list .pi .pdel{color:#555;cursor:pointer;padding:0 4px}
#place-list .pi .pdel:hover{color:#f66}
#log-box{width:320px;height:460px;overflow-y:auto;text-align:left;font-size:.7em;line-height:1.5;scroll-behavior:smooth}
#log-box .le{display:flex;gap:6px;padding:2px 0;border-bottom:1px solid #1e1e1e}
#log-box .lt{color:#555;flex-shrink:0;width:56px}
#log-box .ll{flex-shrink:0;width:50px;font-weight:bold;text-align:center;border-radius:3px;padding:0 2px}
#log-box .lm{color:#ccc;word-break:break-all}
.ll-FOLLOW{color:#00ccff}
.ll-MQTT{color:#cc88ff}
.ll-MOTOR{color:#ffcc00}
.ll-DETECT{color:#ff8800}
.ll-SLAM{color:#00ff88}
.ll-PLACE{color:#aa66ff}
.ll-INFO{color:#aaa}
</style>
</head>
<body>
<h1>MotherV2 Dashboard</h1>
<div class="panels">

<!-- 카메라 스트림 -->
<div class="panel">
<h2>Camera Stream</h2>
<img id="stream-img" src="/stream" alt="Detection Stream">
</div>

<!-- 모터 상태 -->
<div class="panel">
<h2>Drive Status</h2>
<canvas id="robot-canvas" width="300" height="230"></canvas>
</div>

<!-- SLAM 맵 -->
<div class="panel" id="slam-panel">
<h2>SLAM Map / LiDAR</h2>
<canvas id="slam-canvas" width="420" height="420"></canvas>
<div class="legend">
  <div><i style="background:#c8c8c8"></i>Free</div>
  <div><i style="background:#00cc44"></i>Wall</div>
  <div><i style="background:#00ccff"></i>Robot</div>
  <div><i style="background:#ff3366"></i>CAM</div>
  <div><i style="background:#ff8800"></i>LiDAR</div>
  <div><i style="background:#ffcc00"></i>Last</div>
  <div><i style="background:#ffaa00;border-radius:0;border:1px dashed #ffaa00"></i>Pred</div>
  <div><i style="background:#aa66ff"></i>Place</div>
</div>
<div class="slam-info" id="slam-info"></div>
<div class="place-ctrl">
  <input id="place-name" type="text" placeholder="장소 이름 (예: 거실)" maxlength="20">
  <button onclick="addPlace()">📍 인덱싱</button>
  <button onclick="saveMap()" id="btn-savemap">💾 맵 저장</button>
</div>
<div id="place-list"></div>
</div>

<!-- 로봇 로그 -->
<div class="panel">
<h2>Robot Log</h2>
<div id="log-box"></div>
</div>

</div>

<script>
// ── 모터 캔버스 (기존) ───────────────────────────────────────────────────
var mc=document.getElementById("robot-canvas"),mctx=mc.getContext("2d");
var W=mc.width,H=mc.height,CX=W/2,CY=118,RW=60,RH=100,WW=14,WH=90;
var WY=CY-WH/2,LX=CX-RW/2-WW-5,RX=CX+RW/2+5;

function wc(dir,s){if(!s||!dir)return"#2a2a2a";return dir===1?"#00bb44":"#cc3300";}
function sc(s){var m={STOP:"#666",FORWARD:"#00cc44",BACKWARD:"#ff6600",ROTATE_R:"#00aaff",ROTATE_L:"#00aaff",ARC:"#ffcc00",SEARCH:"#ff00ff"};return m[s]||"#fff";}
function dw(x,sp,dir){
  var fh=Math.round(sp/255*WH);
  mctx.fillStyle="#1c1c1c";mctx.strokeStyle="#555";mctx.lineWidth=1;
  mctx.fillRect(x,WY,WW,WH);mctx.strokeRect(x,WY,WW,WH);
  if(sp>0&&dir>0){
    mctx.fillStyle=wc(dir,sp);
    if(dir===1)mctx.fillRect(x+1,WY+WH-fh,WW-2,fh);
    else mctx.fillRect(x+1,WY,WW-2,fh);
    var tx=x+WW/2;
    if(dir===1){var ty=WY+WH/4;mctx.beginPath();mctx.moveTo(tx,ty-6);mctx.lineTo(tx-5,ty+5);mctx.lineTo(tx+5,ty+5);}
    else{var ty=WY+3*WH/4;mctx.beginPath();mctx.moveTo(tx,ty+6);mctx.lineTo(tx-5,ty-5);mctx.lineTo(tx+5,ty-5);}
    mctx.closePath();mctx.fill();
  }
  mctx.strokeStyle="#444";mctx.lineWidth=1;mctx.beginPath();
  mctx.moveTo(x,WY+WH/2);mctx.lineTo(x+WW,WY+WH/2);mctx.stroke();
}
function drawArr(cx,cy,dx,dy,len,color){
  if(len<3)return;var ex=cx+dx*len,ey=cy+dy*len;
  mctx.strokeStyle=color;mctx.fillStyle=color;mctx.lineWidth=3;
  mctx.beginPath();mctx.moveTo(cx,cy);mctx.lineTo(ex,ey);mctx.stroke();
  var a=Math.atan2(dy,dx);mctx.beginPath();mctx.moveTo(ex,ey);
  mctx.lineTo(ex-Math.cos(a-.4)*10,ey-Math.sin(a-.4)*10);
  mctx.lineTo(ex-Math.cos(a+.4)*10,ey-Math.sin(a+.4)*10);
  mctx.closePath();mctx.fill();
}
function drawMotor(d){
  mctx.clearRect(0,0,W,H);
  var ls=d.left_speed,ld=d.left_dir,rs=d.right_speed,rd=d.right_dir,st=d.state;
  mctx.fillStyle=sc(st);mctx.font="bold 14px monospace";mctx.textAlign="center";
  mctx.fillText(st,CX,18);
  mctx.fillStyle="#3a3a3a";mctx.strokeStyle="#777";mctx.lineWidth=2;
  mctx.beginPath();
  if(mctx.roundRect)mctx.roundRect(CX-RW/2,CY-RH/2,RW,RH,8);
  else mctx.rect(CX-RW/2,CY-RH/2,RW,RH);
  mctx.fill();mctx.stroke();
  mctx.fillStyle="#0f0";mctx.fillRect(CX-8,CY-RH/2-5,16,5);
  mctx.fillStyle="#0a0";mctx.font="9px monospace";mctx.textAlign="center";
  mctx.fillText("FRONT",CX,CY-RH/2-8);
  dw(LX,ls,ld);dw(RX,rs,rd);
  mctx.fillStyle="#aaa";mctx.font="10px monospace";mctx.textAlign="left";
  mctx.fillText("L",CX-RW/2-4,CY+4);mctx.textAlign="right";mctx.fillText("R",CX+RW/2+4,CY+4);
  mctx.fillStyle="#ccc";mctx.font="11px monospace";mctx.textAlign="center";
  mctx.fillText("L:"+ls,LX+WW/2,WY+WH+15);mctx.fillText("R:"+rs,RX+WW/2,WY+WH+15);
  function dl(dir){return dir===1?"FWD":dir===2?"BWD":"---";}
  mctx.fillStyle=wc(ld,ls);mctx.font="9px monospace";mctx.fillText(dl(ld),LX+WW/2,WY+WH+27);
  mctx.fillStyle=wc(rd,rs);mctx.fillText(dl(rd),RX+WW/2,WY+WH+27);
  var lv=(ld===1?1:ld===2?-1:0)*ls/255,rv=(rd===1?1:rd===2?-1:0)*rs/255;
  var lin=(lv+rv)/2,ang=(rv-lv)/2,mag=Math.sqrt(lin*lin+ang*ang);
  if(mag>0.02)drawArr(CX,CY,ang/mag,-lin/mag,30*Math.min(mag,1),"#ffee00");
}
drawMotor({left_speed:0,left_dir:0,right_speed:0,right_dir:0,state:"STOP"});
setInterval(function(){
  fetch("/motor").then(function(r){if(r.ok)return r.json();}).then(function(d){if(d)drawMotor(d);}).catch(function(){});
},100);

// ── SLAM / LiDAR 캔버스 ──────────────────────────────────────────────────
var sc2=document.getElementById("slam-canvas"),sctx=sc2.getContext("2d");
var sW=sc2.width,sH=sc2.height;
var sInfo=document.getElementById("slam-info");
var mapImg=new Image();
var SD=null;  // 최신 slam_state 데이터 캐시

// ── 데이터 fetch + 맵 이미지 갱신 ────────────────────────────────────────
// slam_state JSON에 map_b64 필드가 있을 때만 맵 이미지를 갱신 (2초 주기)
// 별도의 /slam_map 이미지 요청 없이 단일 JSON fetch로 처리
function fetchSD(){
  fetch("/slam_state")
    .then(function(r){return r.ok?r.json():null;})
    .then(function(d){
      if(!d)return;
      SD=d;
      if(d.map_b64){
        var img=new Image();
        img.onload=function(){mapImg=img;};
        img.src="data:image/jpeg;base64,"+d.map_b64;
      }
    })
    .catch(function(){});
}
setInterval(fetchSD,200);fetchSD();

// ── 좌표 변환 헬퍼 ───────────────────────────────────────────────────────
function m2c(mx,my,mi,ox,oy,s){
  var px=(mx-mi.crop_origin_x)/mi.crop_w_m*mi.img_w;
  var py=mi.img_h-(my-mi.crop_origin_y)/mi.crop_h_m*mi.img_h;
  return {x:ox+px*s,y:oy+py*s};
}
function drawDot(x,y,conf,depth,dv){
  var col=conf>0.25?"#ff3366":conf>0.15?"#ff8800":"#ffcc00";
  var lbl=conf>0.25?"CAM":conf>0.15?"LiDAR":"LAST";
  sctx.fillStyle=col;sctx.beginPath();sctx.arc(x,y,7,0,Math.PI*2);sctx.fill();
  sctx.strokeStyle=col;sctx.lineWidth=1.5;sctx.beginPath();sctx.arc(x,y,7,0,Math.PI*2);sctx.stroke();
  sctx.fillStyle="#fff";sctx.font="bold 9px monospace";sctx.textAlign="center";
  sctx.fillText(lbl,x,y-11);
  if(dv&&depth>0)sctx.fillText(depth.toFixed(1)+"m",x,y+19);
}

// ── 렌더 루프 (순수 동기, 항상 실행) ────────────────────────────────────
function renderSlam(){
  sctx.clearRect(0,0,sW,sH);
  sctx.fillStyle="#161616";sctx.fillRect(0,0,sW,sH);

  var d=SD;
  if(!d){
    sctx.fillStyle="#aaa";sctx.font="13px monospace";sctx.textAlign="center";
    sctx.fillText("연결 중...",sW/2,sH/2);
    return;
  }

  var hasMap=!!(d.map_info&&d.map_info.available&&mapImg.complete&&mapImg.naturalWidth>0);
  var hasScan=!!(d.scan&&d.scan.length>0);

  if(!hasMap&&!hasScan){
    sctx.fillStyle="#aaa";sctx.font="13px monospace";sctx.textAlign="center";
    sctx.fillText("SLAM / LiDAR 대기중...",sW/2,sH/2-10);
    sctx.fillStyle="#666";sctx.font="11px monospace";
    var mi=d.map_info;
    sctx.fillText("scan:"+(d.scan?d.scan.length:0)+"pts  map:"+(mi&&mi.available?"OK":"없음"),sW/2,sH/2+12);
    return;
  }

  if(hasMap){
    // ── SLAM 맵 모드 ────────────────────────────────────────────────
    var mi=d.map_info;
    var s=Math.min(sW/mi.img_w,sH/mi.img_h);
    var ox=(sW-mi.img_w*s)/2,oy=(sH-mi.img_h*s)/2;
    sctx.drawImage(mapImg,ox,oy,mi.img_w*s,mi.img_h*s);

    // 라이다 스캔 포인트 (로봇 기준 → 맵 좌표 변환)
    if(d.scan&&d.scan.length>0&&d.robot&&d.robot.valid){
      var rx=d.robot.x,ry=d.robot.y,ryaw=d.robot.yaw;
      var cosY=Math.cos(ryaw),sinY=Math.sin(ryaw);
      sctx.fillStyle="rgba(0,220,80,0.7)";
      for(var i=0;i<d.scan.length;i++){
        var sx=d.scan[i][0],sy=d.scan[i][1];
        var mx=rx+sx*cosY-sy*sinY,my2=ry+sx*sinY+sy*cosY;
        var cp=m2c(mx,my2,mi,ox,oy,s);
        sctx.fillRect(cp.x-1.5,cp.y-1.5,3,3);
      }
    }

    // 궤적
    if(d.trajectory&&d.trajectory.length>1){
      sctx.strokeStyle="rgba(255,100,200,0.4)";sctx.lineWidth=2;sctx.beginPath();
      for(var i=0;i<d.trajectory.length;i++){
        var p=m2c(d.trajectory[i][0],d.trajectory[i][1],mi,ox,oy,s);
        i===0?sctx.moveTo(p.x,p.y):sctx.lineTo(p.x,p.y);
      }sctx.stroke();
    }
    // 예측 위치
    if(d.prediction){
      var pp=m2c(d.prediction.x,d.prediction.y,mi,ox,oy,s);
      sctx.strokeStyle="#ffaa00";sctx.lineWidth=2;sctx.setLineDash([5,5]);
      sctx.beginPath();sctx.arc(pp.x,pp.y,12,0,Math.PI*2);sctx.stroke();
      sctx.setLineDash([]);
      sctx.fillStyle="#ffaa00";sctx.font="bold 10px monospace";sctx.textAlign="center";
      sctx.fillText("PRED",pp.x,pp.y-16);
    }
    // 객체
    if(d.objects){
      for(var i=0;i<d.objects.length;i++){
        var o=d.objects[i];if(!o.map_valid)continue;
        var op=m2c(o.map_x,o.map_y,mi,ox,oy,s);
        drawDot(op.x,op.y,o.confidence,o.depth,o.depth_valid);
      }
    }
    // 장소 마커
    if(d.places){
      for(var i=0;i<d.places.length;i++){
        var pl=d.places[i];
        var pp2=m2c(pl.x,pl.y,mi,ox,oy,s);
        sctx.fillStyle="#aa66ff";sctx.beginPath();sctx.arc(pp2.x,pp2.y,6,0,Math.PI*2);sctx.fill();
        sctx.strokeStyle="#cc99ff";sctx.lineWidth=1.5;sctx.beginPath();sctx.arc(pp2.x,pp2.y,6,0,Math.PI*2);sctx.stroke();
        sctx.fillStyle="#cc99ff";sctx.font="bold 9px monospace";sctx.textAlign="center";
        sctx.fillText(pl.name,pp2.x,pp2.y-10);
      }
    }
    // 로봇
    if(d.robot&&d.robot.valid){
      var rp=m2c(d.robot.x,d.robot.y,mi,ox,oy,s);
      sctx.save();sctx.translate(rp.x,rp.y);sctx.rotate(-d.robot.yaw);
      sctx.fillStyle="#00ccff";sctx.beginPath();
      sctx.moveTo(10,0);sctx.lineTo(-7,-6);sctx.lineTo(-7,6);
      sctx.closePath();sctx.fill();
      sctx.strokeStyle="#00eeff";sctx.lineWidth=1.5;sctx.stroke();
      sctx.restore();
    }

  } else {
    // ── LiDAR 레이더 모드 ───────────────────────────────────────────
    var cx=sW/2,cy=sH/2,ppm=Math.min(sW,sH)/12;
    // 거리 링
    for(var rv=2;rv<=6;rv+=2){
      sctx.strokeStyle="rgba(255,255,255,0.08)";sctx.lineWidth=1;
      sctx.beginPath();sctx.arc(cx,cy,rv*ppm,0,Math.PI*2);sctx.stroke();
      sctx.fillStyle="#555";sctx.font="9px monospace";sctx.textAlign="center";
      sctx.fillText(rv+"m",cx,cy-rv*ppm+10);
    }
    // 십자선
    sctx.strokeStyle="rgba(255,255,255,0.06)";sctx.lineWidth=1;
    sctx.beginPath();sctx.moveTo(cx,0);sctx.lineTo(cx,sH);sctx.stroke();
    sctx.beginPath();sctx.moveTo(0,cy);sctx.lineTo(sW,cy);sctx.stroke();
    // 스캔 포인트
    sctx.fillStyle="#00cc55";
    for(var i=0;i<d.scan.length;i++){
      var spx=cx+d.scan[i][1]*ppm,spy=cy+d.scan[i][0]*ppm;
      sctx.fillRect(spx-1.5,spy-1.5,3,3);
    }
    // 객체
    if(d.objects){
      for(var i=0;i<d.objects.length;i++){
        var o=d.objects[i];if(!o.depth_valid)continue;
        var ang=o.angle||0;
        drawDot(cx-o.depth*Math.sin(ang)*ppm,cy-o.depth*Math.cos(ang)*ppm,
                o.confidence,o.depth,true);
      }
    }
    // 장소 마커 (레이더 상대좌표)
    if(d.places&&d.robot&&d.robot.valid){
      var yaw=d.robot.yaw,rx=d.robot.x,ry=d.robot.y;
      for(var i=0;i<d.places.length;i++){
        var pl=d.places[i];
        var dx=pl.x-rx,dy=pl.y-ry;
        var lx=dx*Math.cos(-yaw)-dy*Math.sin(-yaw);
        var ly=dx*Math.sin(-yaw)+dy*Math.cos(-yaw);
        var ppx=cx-ly*ppm,ppy=cy-lx*ppm;
        sctx.fillStyle="#aa66ff";sctx.beginPath();sctx.arc(ppx,ppy,6,0,Math.PI*2);sctx.fill();
        sctx.strokeStyle="#cc99ff";sctx.lineWidth=1.5;sctx.beginPath();sctx.arc(ppx,ppy,6,0,Math.PI*2);sctx.stroke();
        sctx.fillStyle="#cc99ff";sctx.font="bold 9px monospace";sctx.textAlign="center";
        sctx.fillText(pl.name,ppx,ppy-10);
      }
    }
    // 로봇 (레이더 중앙)
    sctx.save();sctx.translate(cx,cy);
    sctx.fillStyle="#00ccff";sctx.beginPath();
    sctx.moveTo(0,-13);sctx.lineTo(-8,8);sctx.lineTo(8,8);
    sctx.closePath();sctx.fill();
    sctx.strokeStyle="#00eeff";sctx.lineWidth=1.5;sctx.stroke();
    sctx.restore();
    sctx.fillStyle="#444";sctx.font="10px monospace";sctx.textAlign="center";
    sctx.fillText("LiDAR Radar · SLAM 맵 대기중",cx,sH-6);
  }

  // ── 텍스트 정보 ─────────────────────────────────────────────────────
  var info="";
  if(d.robot&&d.robot.valid){
    info+="Robot: <span>("+d.robot.x.toFixed(2)+","+d.robot.y.toFixed(2)+")</span> "+
          "yaw:<span>"+(d.robot.yaw*180/Math.PI).toFixed(1)+"&deg;</span><br>";
  }else{
    info+="Robot: <span style='color:#888'>TF 없음</span><br>";
  }
  if(d.places&&d.places.length>0&&d.robot&&d.robot.valid){
    var rx=d.robot.x,ry=d.robot.y,nd=Infinity,nn="";
    for(var i=0;i<d.places.length;i++){
      var dd=Math.sqrt(Math.pow(d.places[i].x-rx,2)+Math.pow(d.places[i].y-ry,2));
      if(dd<nd){nd=dd;nn=d.places[i].name;}
    }
    info+="위치: <span style='color:#aa66ff'>"+nn+"</span> <span>("+nd.toFixed(1)+"m)</span><br>";
    renderPlaceList(d.places);
  }
  if(d.objects&&d.objects.length>0){
    var o=d.objects[0];
    var mode=o.confidence>0.25?"CAM":o.confidence>0.15?"LiDAR":"LAST";
    var oc=o.confidence>0.25?"#ff3366":o.confidence>0.15?"#ff8800":"#ffcc00";
    info+="Object[<span style='color:"+oc+"'>"+mode+"</span>]: ";
    if(o.depth_valid)info+="<span>"+o.depth.toFixed(2)+"m</span> ";
    if(o.map_valid)info+="map<span>("+o.map_x.toFixed(2)+","+o.map_y.toFixed(2)+")</span>";
    info+="<br>";
  }
  if(d.scan)info+="Scan:<span>"+d.scan.length+"pts</span> ";
  if(d.trajectory&&d.trajectory.length)info+="Traj:<span>"+d.trajectory.length+"pts</span>";
  sInfo.innerHTML=info;
}
setInterval(renderSlam,200);

// ── 로봇 로그 ────────────────────────────────────────────────────────────
var logBox=document.getElementById("log-box");
var logLen=0;
var logColors={FOLLOW:"#00ccff",MQTT:"#cc88ff",MOTOR:"#ffcc00",DETECT:"#ff8800",SLAM:"#00ff88",PLACE:"#aa66ff",INFO:"#aaa"};
function fetchLog(){
  fetch("/log").then(function(r){return r.ok?r.json():null;}).then(function(entries){
    if(!entries||entries.length===logLen)return;
    logLen=entries.length;
    var html="";
    for(var i=0;i<entries.length;i++){
      var e=entries[i];
      var col=logColors[e.l]||"#aaa";
      html+="<div class='le'>"+
            "<span class='lt'>"+e.t+"</span>"+
            "<span class='ll ll-"+e.l+"' style='color:"+col+"'>"+e.l+"</span>"+
            "<span class='lm'>"+e.m+"</span></div>";
    }
    logBox.innerHTML=html;
    logBox.scrollTop=logBox.scrollHeight;
  }).catch(function(){});
}
setInterval(fetchLog,1000);fetchLog();

// ── 장소 인덱싱 기능 ──────────────────────────────────────────────────────
function renderPlaceList(places){
  var el=document.getElementById("place-list");
  if(!el)return;
  var html="";
  for(var i=0;i<places.length;i++){
    var p=places[i];
    html+="<div class='pi'><span class='pname'>"+p.name+"</span>"+
          "<span style='color:#666'>("+p.x.toFixed(2)+","+p.y.toFixed(2)+")</span>"+
          "<span class='pdel' onclick='deletePlace(\\""+p.name+"\\")'>✕</span></div>";
  }
  el.innerHTML=html||"<span style='color:#555'>저장된 장소 없음</span>";
}

function addPlace(){
  var name=document.getElementById("place-name").value.trim();
  if(!name){alert("장소 이름을 입력하세요");return;}
  fetch("/add_place",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({name:name})})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){
        document.getElementById("place-name").value="";
        alert("저장: "+d.place.name+" ("+d.place.x+", "+d.place.y+")");
      } else {
        alert("실패: "+(d.error||"알 수 없는 오류"));
      }
    }).catch(function(){alert("서버 오류");});
}

function deletePlace(name){
  if(!confirm(name+" 삭제?"))return;
  fetch("/delete_place",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({name:name})})
    .then(function(r){return r.json();})
    .then(function(d){if(!d.ok)alert("삭제 실패");})
    .catch(function(){});
}

function saveMap(){
  var btn=document.getElementById("btn-savemap");
  btn.textContent="저장 중...";btn.disabled=true;
  fetch("/save_map",{method:"POST"})
    .then(function(r){return r.json();})
    .then(function(d){
      btn.textContent="💾 맵 저장";btn.disabled=false;
      if(d.ok)alert("맵 저장 완료\\n경로: "+d.path);
      else alert("맵 저장 실패\\n"+d.error);
    }).catch(function(){btn.textContent="💾 맵 저장";btn.disabled=false;alert("서버 오류");});
}
</script>
</body>
</html>'''


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
