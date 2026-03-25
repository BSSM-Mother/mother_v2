import json
import math
import threading
import time
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from motherv2_interfaces.msg import DetectionArray, MotorCommand, ObjectEstimateArray
from std_msgs.msg import Bool


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
        else:
            self._index()

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
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(body)


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

        # ── 구독: SLAM ──────────────────────────────────────────────────────
        # /map 은 hector_mapping이 RELIABLE로 퍼블리시 (latch)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 1)
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
        self.server = ThreadingHTTPServer(('0.0.0.0', port), MJPEGHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f'Web server started on port {port}')

    # ── 기존 콜백 ─────────────────────────────────────────────────────────

    def follow_callback(self, msg: Bool):
        with MJPEGHandler.status_lock:
            MJPEGHandler.status_state['follow_enabled'] = bool(msg.data)

    def detection_callback(self, msg: DetectionArray):
        boxes = []
        for det in msg.detections:
            boxes.append((det.x, det.y, det.w, det.h, det.confidence, det.class_id))
        with self._det_lock:
            self._det_boxes = boxes
            self._det_time = time.time()
        with MJPEGHandler.status_lock:
            MJPEGHandler.status_state['detecting'] = len(boxes) > 0

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
            MJPEGHandler.motor_state = data

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
            self._occupancy_grid = msg

    def _estimates_callback(self, msg: ObjectEstimateArray):
        objs = []
        for est in msg.estimates:
            obj = {
                'class_id': est.class_id,
                'confidence': round(est.confidence, 2),
                'depth': round(est.depth, 3),
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
        # 미지: #222222, 빈 공간: #3a3a44, 점유: #00ff66
        img = np.zeros((ch, cw, 3), dtype=np.uint8)
        unknown = cropped < 0
        free = cropped == 0
        occupied = cropped > 0

        img[unknown] = (34, 34, 34)
        img[free] = (68, 58, 58)

        # 점유도에 따라 녹색 그라데이션
        occ_vals = cropped[occupied].astype(np.float32) / 100.0
        img[occupied, 0] = 0                                    # B
        img[occupied, 1] = (100 + 155 * occ_vals).astype(np.uint8)  # G
        img[occupied, 2] = 0                                    # R

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
        """SLAM 상태 JSON 업데이트 (로봇 포즈, 객체, 예측, 궤적)."""
        with self._robot_pose_lock:
            rp = self._robot_pose

        with self._slam_obj_lock:
            objs = list(self._slam_objects)
            pred = self._slam_prediction

        traj = list(self._slam_trajectory)
        map_info = getattr(self, '_map_render_info', {'available': False})

        state = {
            'map_info': map_info,
            'robot': {
                'x': round(rp[0], 3) if rp else 0,
                'y': round(rp[1], 3) if rp else 0,
                'yaw': round(rp[2], 3) if rp else 0,
                'valid': rp is not None,
            },
            'objects': objs,
            'prediction': pred,
            'trajectory': [[round(x, 3), round(y, 3)] for x, y in traj[-100:]],
        }

        with MJPEGHandler.slam_state_lock:
            MJPEGHandler.slam_state = state

    def destroy_node(self):
        self.server.shutdown()
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
#slam-canvas{background:#222;border:1px solid #444}
.slam-info{font-size:.75em;color:#888;margin-top:8px;text-align:left;line-height:1.6}
.slam-info span{color:#0f0}
.legend{display:flex;gap:12px;justify-content:center;font-size:.65em;color:#aaa;margin-top:6px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:3px;vertical-align:middle}
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
<div class="panel" id="slam-panel" style="display:none">
<h2>SLAM Map</h2>
<canvas id="slam-canvas" width="420" height="420"></canvas>
<div class="legend">
  <div><i style="background:#3a3a44"></i>Free</div>
  <div><i style="background:#00ff66"></i>Wall</div>
  <div><i style="background:#00ccff"></i>Robot</div>
  <div><i style="background:#ff3366"></i>Object</div>
  <div><i style="background:#ffaa00;border-radius:0;border:1px dashed #ffaa00"></i>Prediction</div>
</div>
<div class="slam-info" id="slam-info"></div>
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

// ── SLAM 캔버스 ──────────────────────────────────────────────────────────
var sc2=document.getElementById("slam-canvas"),sctx=sc2.getContext("2d");
var sW=sc2.width,sH=sc2.height;
var sPanel=document.getElementById("slam-panel");
var sInfo=document.getElementById("slam-info");
var mapImg=new Image();
var mapLoaded=false;
var slamActive=false;

// 맵 이미지 로드 (3초 주기)
function loadMap(){
  var img=new Image();
  img.onload=function(){mapImg=img;mapLoaded=true;if(!slamActive){sPanel.style.display="block";slamActive=true;}};
  img.onerror=function(){};
  img.src="/slam_map?t="+Date.now();
}
setInterval(loadMap,3000);
loadMap();

// 맵 좌표 → 캔버스 픽셀 변환
function mapToCanvas(mx,my,mi){
  if(!mi||!mi.available)return null;
  // 맵 좌표 → 크롭 이미지 픽셀
  var px=(mx-mi.crop_origin_x)/mi.crop_w_m*mi.img_w;
  var py=mi.img_h-(my-mi.crop_origin_y)/mi.crop_h_m*mi.img_h;
  // 이미지 픽셀 → 캔버스 픽셀
  var sx=sW/mi.img_w, sy=sH/mi.img_h;
  var s=Math.min(sx,sy);
  var offX=(sW-mi.img_w*s)/2, offY=(sH-mi.img_h*s)/2;
  return {x:offX+px*s, y:offY+py*s};
}

// SLAM 상태 폴링 + 렌더 (200ms)
function drawSlam(){
  fetch("/slam_state").then(function(r){if(r.ok)return r.json();}).then(function(d){
    if(!d||!d.map_info||!d.map_info.available)return;
    if(!slamActive){sPanel.style.display="block";slamActive=true;}

    var mi=d.map_info;
    sctx.clearRect(0,0,sW,sH);
    sctx.fillStyle="#222";sctx.fillRect(0,0,sW,sH);

    // 맵 이미지 그리기 (aspect ratio 유지)
    if(mapLoaded){
      var sx=sW/mi.img_w, sy=sH/mi.img_h;
      var s=Math.min(sx,sy);
      var offX=(sW-mi.img_w*s)/2, offY=(sH-mi.img_h*s)/2;
      sctx.drawImage(mapImg,offX,offY,mi.img_w*s,mi.img_h*s);
    }

    // 궤적 라인
    if(d.trajectory&&d.trajectory.length>1){
      sctx.strokeStyle="rgba(255,100,200,0.4)";sctx.lineWidth=2;
      sctx.beginPath();
      for(var i=0;i<d.trajectory.length;i++){
        var p=mapToCanvas(d.trajectory[i][0],d.trajectory[i][1],mi);
        if(!p)continue;
        if(i===0)sctx.moveTo(p.x,p.y);else sctx.lineTo(p.x,p.y);
      }
      sctx.stroke();
    }

    // 예측 위치 (점선 원)
    if(d.prediction){
      var pp=mapToCanvas(d.prediction.x,d.prediction.y,mi);
      if(pp){
        sctx.strokeStyle="#ffaa00";sctx.lineWidth=2;
        sctx.setLineDash([5,5]);
        sctx.beginPath();sctx.arc(pp.x,pp.y,12,0,Math.PI*2);sctx.stroke();
        sctx.setLineDash([]);
        sctx.fillStyle="#ffaa00";sctx.font="bold 10px monospace";sctx.textAlign="center";
        sctx.fillText("PRED",pp.x,pp.y-16);
      }
    }

    // 객체 위치
    if(d.objects){
      for(var i=0;i<d.objects.length;i++){
        var o=d.objects[i];
        if(!o.map_valid)continue;
        var op=mapToCanvas(o.map_x,o.map_y,mi);
        if(!op)continue;
        sctx.fillStyle="#ff3366";
        sctx.beginPath();sctx.arc(op.x,op.y,7,0,Math.PI*2);sctx.fill();
        sctx.strokeStyle="#ff6699";sctx.lineWidth=2;
        sctx.beginPath();sctx.arc(op.x,op.y,7,0,Math.PI*2);sctx.stroke();
        sctx.fillStyle="#fff";sctx.font="bold 9px monospace";sctx.textAlign="center";
        sctx.fillText(o.depth_valid?o.depth.toFixed(1)+"m":"?",op.x,op.y-11);
      }
    }

    // 로봇 위치 (삼각형)
    if(d.robot&&d.robot.valid){
      var rp=mapToCanvas(d.robot.x,d.robot.y,mi);
      if(rp){
        var yaw=-d.robot.yaw; // 캔버스 y축 반전 보정
        var sz=10;
        sctx.save();sctx.translate(rp.x,rp.y);sctx.rotate(yaw);
        sctx.fillStyle="#00ccff";sctx.beginPath();
        sctx.moveTo(sz,0);sctx.lineTo(-sz*0.7,-sz*0.6);sctx.lineTo(-sz*0.7,sz*0.6);
        sctx.closePath();sctx.fill();
        sctx.strokeStyle="#00eeff";sctx.lineWidth=1.5;sctx.stroke();
        sctx.restore();
      }
    }

    // 정보 텍스트
    var info="";
    if(d.robot&&d.robot.valid){
      info+="Robot: <span>("+d.robot.x.toFixed(2)+", "+d.robot.y.toFixed(2)+")</span> yaw=<span>"+
        (d.robot.yaw*180/Math.PI).toFixed(1)+"&deg;</span><br>";
    }else{
      info+="Robot: <span style='color:#f66'>TF not available</span><br>";
    }
    if(d.objects&&d.objects.length>0){
      var o=d.objects[0];
      info+="Object: ";
      if(o.depth_valid)info+="depth=<span>"+o.depth.toFixed(2)+"m</span> ";
      if(o.map_valid)info+="map=<span>("+o.map_x.toFixed(2)+", "+o.map_y.toFixed(2)+")</span>";
      else info+="<span style='color:#f66'>no map</span>";
      info+="<br>";
    }
    if(d.prediction)info+="Prediction: <span>("+d.prediction.x.toFixed(2)+", "+d.prediction.y.toFixed(2)+")</span><br>";
    if(d.trajectory)info+="Trajectory: <span>"+d.trajectory.length+" pts</span>";
    sInfo.innerHTML=info;

  }).catch(function(){});
}
setInterval(drawSlam,200);
drawSlam();
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
