import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from motherv2_interfaces.msg import Detection, DetectionArray

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

COCO_NAMES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
]
NAME_TO_ID = {name: i for i, name in enumerate(COCO_NAMES)}


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('debug_class', -1)
        self.declare_parameter('stream_width', 640)
        self.declare_parameter('detection_interval', 0.5)

        model_path = str(self.get_parameter('model_path').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.debug_class = int(self.get_parameter('debug_class').value)
        self.stream_width = int(self.get_parameter('stream_width').value)
        self.detection_interval = float(self.get_parameter('detection_interval').value)

        self.target_names = {'bottle'}
        if self.debug_class >= 0 and self.debug_class < len(COCO_NAMES):
            debug_name = COCO_NAMES[self.debug_class]
            self.target_names.add(debug_name)
            self.get_logger().info(f'Debug class: {self.debug_class} ({debug_name})')

        if not model_path:
            self.get_logger().error('model_path is required')
            raise RuntimeError('No model path')

        self.get_logger().info(f'Loading MediaPipe model: {model_path}')
        options = vision.ObjectDetectorOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            score_threshold=self.conf_threshold,
            max_results=10,
        )
        self.detector = vision.ObjectDetector.create_from_options(options)
        self.get_logger().info('MediaPipe EfficientDet-Lite0 loaded')

        self.bridge = CvBridge()

        # ── 공유 상태 (lock으로 보호) ──────────────────────────
        self._lock = threading.Lock()
        self._tracker = None          # OpenCV MOSSE
        self._tracked_box = None      # (x,y,w,h)
        self._tracked_cls = None
        self._is_tracking = False
        # detection 스레드 → callback: bbox만 전달 (프레임 X)
        self._pending_init = False
        self._pending_init_box = None
        self._pending_init_cls = None
        # ───────────────────────────────────────────────────────

        # detection 스레드용
        self._det_lock = threading.Lock()
        self._det_frame = None
        self._det_stamp = None
        self._last_det_req_time = 0.0

        self._running = True
        self._det_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self._det_thread.start()

        self.det_pub = self.create_publisher(DetectionArray, '/motherv2/detections', 1)
        self.img_pub = self.create_publisher(Image, '/motherv2/detection_image', 1)

        cam_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            Image, '/motherv2/image_raw', self.image_callback, cam_qos
        )

        self._trk_fps_counter = 0
        self._det_fps_counter = 0
        self._fps_time = time.time()

    def image_callback(self, msg: Image):
        """매 프레임 호출 (~30fps). 트래킹은 여기서 직접 실행."""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        stamp = msg.header.stamp
        now = time.time()

        # detection 스레드에 프레임 제공 (interval 체크)
        if now - self._last_det_req_time >= self.detection_interval:
            with self._det_lock:
                self._det_frame = frame.copy()
                self._det_stamp = stamp
            self._last_det_req_time = now

        results = []  # (x,y,w,h,conf,cls_id,is_tracked)

        with self._lock:
            # pending init: bbox는 detection에서, 프레임은 현재 것으로 초기화
            if self._pending_init:
                self._do_init_tracker(frame, self._pending_init_box, self._pending_init_cls)
                self._pending_init = False

            # 트래킹 실행
            if self._is_tracking and self._tracker is not None:
                success, bbox = self._tracker.update(frame)
                if success:
                    x, y, w, h = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])
                    self._tracked_box = (x, y, w, h)
                    results = [(x, y, w, h, 1.0, self._tracked_cls, True)]
                    self._trk_fps_counter += 1
                else:
                    self._tracker = None
                    self._is_tracking = False

        # FPS 로그
        elapsed = now - self._fps_time
        if elapsed >= 3.0:
            det_fps = self._det_fps_counter / elapsed
            trk_fps = self._trk_fps_counter / elapsed
            self.get_logger().info(
                f'Detection: {det_fps:.1f}fps  Tracking: {trk_fps:.1f}fps'
            )
            self._det_fps_counter = 0
            self._trk_fps_counter = 0
            self._fps_time = now

        self._publish(frame, stamp, results)

    def _do_init_tracker(self, frame, box, cls_id):
        """락 안에서 호출 - 트래커 초기화"""
        x, y, w, h = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        fh, fw = frame.shape[:2]
        x = max(0, min(x, fw - 1))
        y = max(0, min(y, fh - 1))
        w = max(1, min(w, fw - x))
        h = max(1, min(h, fh - y))
        if w < 5 or h < 5:
            return
        tracker = cv2.legacy.TrackerMOSSE_create()
        tracker.init(frame, (x, y, w, h))
        self._tracker = tracker
        self._tracked_box = (float(x), float(y), float(w), float(h))
        self._tracked_cls = cls_id
        self._is_tracking = True

    def _detection_loop(self):
        """백그라운드 스레드 - MediaPipe 추론만 담당"""
        while self._running:
            with self._det_lock:
                frame = self._det_frame
                self._det_frame = None

            if frame is None:
                time.sleep(0.01)
                continue

            # MediaPipe 추론 - 640x360으로 축소 후 처리 (속도 개선)
            fh, fw = frame.shape[:2]
            det_w, det_h = 640, 360
            if fw > det_w:
                small = cv2.resize(frame, (det_w, det_h))
                scale_x = fw / det_w
                scale_y = fh / det_h
            else:
                small = frame
                scale_x = scale_y = 1.0
            rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            result = self.detector.detect(mp_image)

            detections = []
            for det in result.detections:
                cat = det.categories[0]
                if cat.category_name not in self.target_names:
                    continue
                bb = det.bounding_box
                cls_id = NAME_TO_ID.get(cat.category_name, -1)
                detections.append((
                    float(bb.origin_x) * scale_x, float(bb.origin_y) * scale_y,
                    float(bb.width) * scale_x, float(bb.height) * scale_y,
                    float(cat.score), cls_id,
                ))

            self._det_fps_counter += 1

            if detections:
                best = max(detections, key=lambda d: d[4])
                # bbox만 등록 - 프레임은 callback의 현재 프레임 사용
                with self._lock:
                    self._pending_init = True
                    self._pending_init_box = best[:4]
                    self._pending_init_cls = best[5]
            # detections 없을 때는 트래커를 건드리지 않음 (MOSSE가 계속 추적)

    def _publish(self, frame, stamp, results):
        det_msgs = []
        annotated = frame.copy()

        for x, y, w, h, conf, cls_id, is_tracked in results:
            det = Detection()
            det.x, det.y, det.w, det.h = float(x), float(y), float(w), float(h)
            det.confidence = float(conf)
            det.class_id = int(cls_id) if cls_id is not None else 0
            det_msgs.append(det)

            label = COCO_NAMES[cls_id] if cls_id is not None and 0 <= cls_id < len(COCO_NAMES) else '?'
            color = (255, 128, 0) if is_tracked else (0, 255, 0)
            tag = 'TRK' if is_tracked else 'DET'
            ix, iy, iw, ih = int(x), int(y), int(w), int(h)
            cv2.rectangle(annotated, (ix, iy), (ix + iw, iy + ih), color, 2)
            cv2.putText(
                annotated, f'[{tag}] {label} {conf:.2f}',
                (ix, max(iy - 5, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
            )

        status = 'TRACKING' if self._is_tracking else 'SEARCHING...'
        cv2.putText(annotated, status, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        det_msg = DetectionArray()
        det_msg.header.stamp = stamp if stamp else self.get_clock().now().to_msg()
        det_msg.header.frame_id = 'camera'
        det_msg.detections = det_msgs
        self.det_pub.publish(det_msg)

        fh, fw = annotated.shape[:2]
        if fw > self.stream_width:
            scale = self.stream_width / fw
            stream_img = cv2.resize(annotated, (self.stream_width, int(fh * scale)))
        else:
            stream_img = annotated
        img_msg = self.bridge.cv2_to_imgmsg(stream_img, encoding='bgr8')
        img_msg.header = det_msg.header
        self.img_pub.publish(img_msg)

    def destroy_node(self):
        self._running = False
        self._det_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
