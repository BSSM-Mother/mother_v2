import json
import threading
import time
from http.server import HTTPServer, ThreadingHTTPServer, BaseHTTPRequestHandler

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from motherv2_interfaces.msg import DetectionArray, MotorCommand


class MJPEGHandler(BaseHTTPRequestHandler):
    latest_frame = None
    frame_lock = threading.Lock()
    motor_state = {'left_speed': 0, 'left_dir': 0, 'right_speed': 0, 'right_dir': 0, 'state': 'STOP'}
    motor_lock = threading.Lock()

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/stream':
            self._stream()
        elif self.path == '/snapshot':
            self._snapshot()
        elif self.path == '/motor':
            self._motor_json()
        else:
            self._index()

    def _index(self):
        html = b'''<!DOCTYPE html>
<html>
<head>
<title>MotherV2 Vision</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{background:#1a1a1a;color:#fff;font-family:monospace;text-align:center;margin:20px}
h1{color:#0f0;margin-bottom:16px}
#stream-img{max-width:100%;border:2px solid #444;border-radius:8px}
#motor-panel{margin-top:24px;display:inline-block}
#motor-panel h2{color:#aaa;font-size:.85em;margin:0 0 8px;text-transform:uppercase;letter-spacing:2px}
#robot-canvas{border:2px solid #444;border-radius:8px;background:#111;display:block;margin:0 auto}
</style>
</head>
<body>
<h1>MotherV2 - Person Following Robot</h1>
<img id="stream-img" src="/stream" alt="Detection Stream">
<p>Detection stream with bounding boxes</p>
<div id="motor-panel">
<h2>Drive Status</h2>
<canvas id="robot-canvas" width="300" height="230"></canvas>
</div>
<script>
var canvas=document.getElementById("robot-canvas");
var ctx=canvas.getContext("2d");
var W=canvas.width,H=canvas.height;
var CX=W/2,CY=118;
var RW=60,RH=100;
var WW=14,WH=90;
var WY=CY-WH/2;
var LX=CX-RW/2-WW-5;
var RX=CX+RW/2+5;

function wheelColor(dir,speed){
  if(!speed||!dir)return"#2a2a2a";
  return dir===1?"#00bb44":"#cc3300";
}
function stateColor(s){
  var m={STOP:"#666",FORWARD:"#00cc44",BACKWARD:"#ff6600",
         ROTATE_R:"#00aaff",ROTATE_L:"#00aaff",ARC:"#ffcc00",SEARCH:"#ff00ff"};
  return m[s]||"#ffffff";
}
function drawWheel(x,speed,dir){
  var fh=Math.round(speed/255*WH);
  ctx.fillStyle="#1c1c1c";ctx.strokeStyle="#555";ctx.lineWidth=1;
  ctx.fillRect(x,WY,WW,WH);ctx.strokeRect(x,WY,WW,WH);
  if(speed>0&&dir>0){
    ctx.fillStyle=wheelColor(dir,speed);
    if(dir===1)ctx.fillRect(x+1,WY+WH-fh,WW-2,fh);
    else ctx.fillRect(x+1,WY,WW-2,fh);
    var tx=x+WW/2;
    if(dir===1){
      var ty=WY+WH/4;
      ctx.beginPath();ctx.moveTo(tx,ty-6);ctx.lineTo(tx-5,ty+5);ctx.lineTo(tx+5,ty+5);
    } else {
      var ty=WY+3*WH/4;
      ctx.beginPath();ctx.moveTo(tx,ty+6);ctx.lineTo(tx-5,ty-5);ctx.lineTo(tx+5,ty-5);
    }
    ctx.closePath();ctx.fill();
  }
  ctx.strokeStyle="#444";ctx.lineWidth=1;ctx.beginPath();
  ctx.moveTo(x,WY+WH/2);ctx.lineTo(x+WW,WY+WH/2);ctx.stroke();
}
function drawArrow(cx,cy,dx,dy,len,color){
  if(len<3)return;
  var ex=cx+dx*len,ey=cy+dy*len;
  ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=3;
  ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(ex,ey);ctx.stroke();
  var a=Math.atan2(dy,dx);
  ctx.beginPath();
  ctx.moveTo(ex,ey);
  ctx.lineTo(ex-Math.cos(a-.4)*10,ey-Math.sin(a-.4)*10);
  ctx.lineTo(ex-Math.cos(a+.4)*10,ey-Math.sin(a+.4)*10);
  ctx.closePath();ctx.fill();
}
function draw(d){
  ctx.clearRect(0,0,W,H);
  var ls=d.left_speed,ld=d.left_dir,rs=d.right_speed,rd=d.right_dir,st=d.state;

  ctx.fillStyle=stateColor(st);ctx.font="bold 14px monospace";ctx.textAlign="center";
  ctx.fillText(st,CX,18);

  ctx.fillStyle="#3a3a3a";ctx.strokeStyle="#777";ctx.lineWidth=2;
  ctx.beginPath();
  if(ctx.roundRect)ctx.roundRect(CX-RW/2,CY-RH/2,RW,RH,8);
  else ctx.rect(CX-RW/2,CY-RH/2,RW,RH);
  ctx.fill();ctx.stroke();

  ctx.fillStyle="#0f0";ctx.fillRect(CX-8,CY-RH/2-5,16,5);
  ctx.fillStyle="#0a0";ctx.font="9px monospace";ctx.textAlign="center";
  ctx.fillText("FRONT",CX,CY-RH/2-8);

  drawWheel(LX,ls,ld);
  drawWheel(RX,rs,rd);

  ctx.fillStyle="#aaa";ctx.font="10px monospace";ctx.textAlign="left";
  ctx.fillText("L",CX-RW/2-4,CY+4);
  ctx.textAlign="right";
  ctx.fillText("R",CX+RW/2+4,CY+4);

  ctx.fillStyle="#ccc";ctx.font="11px monospace";ctx.textAlign="center";
  ctx.fillText("L:"+ls,LX+WW/2,WY+WH+15);
  ctx.fillText("R:"+rs,RX+WW/2,WY+WH+15);

  function dl(dir){return dir===1?"FWD":dir===2?"BWD":"---";}
  ctx.fillStyle=wheelColor(ld,ls);ctx.font="9px monospace";
  ctx.fillText(dl(ld),LX+WW/2,WY+WH+27);
  ctx.fillStyle=wheelColor(rd,rs);
  ctx.fillText(dl(rd),RX+WW/2,WY+WH+27);

  var lv=(ld===1?1:ld===2?-1:0)*ls/255;
  var rv=(rd===1?1:rd===2?-1:0)*rs/255;
  var lin=(lv+rv)/2,ang=(rv-lv)/2;
  var mag=Math.sqrt(lin*lin+ang*ang);
  if(mag>0.02)drawArrow(CX,CY,ang/mag,-lin/mag,30*Math.min(mag,1),"#ffee00");
}

draw({left_speed:0,left_dir:0,right_speed:0,right_dir:0,state:"STOP"});
setInterval(function(){
  fetch("/motor").then(function(r){if(r.ok)return r.json();}).then(function(d){if(d)draw(d);}).catch(function(){});
},100);
</script>
</body>
</html>'''
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
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
                time.sleep(0.033)  # ~30fps
        except (BrokenPipeError, ConnectionResetError):
            pass


class WebNode(Node):
    def __init__(self):
        super().__init__('web_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('stream_width', 640)
        port = int(self.get_parameter('port').value)
        self.stream_width = int(self.get_parameter('stream_width').value)

        self.bridge = CvBridge()

        # 최신 detection box 상태
        self._det_lock = threading.Lock()
        self._det_boxes = []   # list of (x,y,w,h,conf,cls_id)
        self._det_time = 0.0
        self._det_timeout = 1.0  # 1초 이상 detection 없으면 박스 지움

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 카메라 원본 구독 → 30fps 스트림
        self.cam_sub = self.create_subscription(
            Image, '/motherv2/image_raw', self.camera_callback, qos
        )
        # detection 결과 구독 → bbox 오버레이용
        self.det_sub = self.create_subscription(
            DetectionArray, '/motherv2/detections', self.detection_callback, 1
        )
        # 모터 커맨드 구독 → 구동 시각화용
        self.motor_sub = self.create_subscription(
            MotorCommand, '/motherv2/cmd_motor', self.motor_callback, 1
        )

        self.server = ThreadingHTTPServer(('0.0.0.0', port), MJPEGHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f'Web server started on port {port}')

    def detection_callback(self, msg: DetectionArray):
        boxes = []
        for det in msg.detections:
            boxes.append((det.x, det.y, det.w, det.h, det.confidence, det.class_id))
        with self._det_lock:
            self._det_boxes = boxes
            self._det_time = time.time()

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

        # 리사이즈
        h, w = frame.shape[:2]
        if w > self.stream_width:
            scale = self.stream_width / w
            frame = cv2.resize(frame, (self.stream_width, int(h * scale)))

        # detection 박스 오버레이
        with self._det_lock:
            boxes = self._det_boxes if (time.time() - self._det_time) < self._det_timeout else []

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

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


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
