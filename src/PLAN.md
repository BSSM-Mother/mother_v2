# MotherV2 - Person Following Robot Project Plan

## Hardware Specification
- **SBC**: Raspberry Pi 4B (4GB RAM, 4-core Cortex-A72)
- **Camera**: IMX219 (CSI, mounted with CSI port facing up → needs 180° flip)
- **MCU**: STM32F103C8T6 via `/dev/ttyAMA0`
- **Drive**: 2-wheel differential drive
- **Wheel diameter**: 43mm, **Wheelbase**: 70mm
- **Chassis**: Circular, 140mm diameter
- **Environment**: 200mm block corridor maze

## Serial Protocol (STM32)
- Format: `rspeed,lspeed,rdir,ldir\n`
- dir: 0=stop, 1=forward, 2=reverse
- `s\n` = emergency stop (both wheels, preferred when both stop)
- Speed: 0~255 (dead zone at low values → needs min-speed compensation)
- Gradual stop (inertia) → needs early braking compensation

## Software Stack
- ROS2 Jazzy, Python 3.12
- OpenCV 4.11, ONNX Runtime 1.24
- cv_bridge, image_transport
- rpicam-vid (v0.7.0), python3-libcamera (v0.2.0)

---

## Package Architecture

### 1. motherv2_camera
Camera acquisition node using `camera_ros` package (christianrauch/camera_ros).
- Publishes: `/motherv2/image_raw` (sensor_msgs/Image)
- Uses libcamera backend → proper ISP processing for IMX219
- Parameters: orientation (180° for upside-down mount), width, height

### 2. motherv2_detection
Person detection using YOLOv8n ONNX (320x320 input).
- Subscribes: `/motherv2/image_raw`
- Publishes: `/motherv2/detections` (custom msg: bounding boxes, confidence, class)
- Publishes: `/motherv2/detection_image` (annotated image for web view)
- Async detection thread to maintain throughput
- Only filters "person" class (COCO class 0)
- Target: ~10-15fps detection on RPi4 with ONNX Runtime

### 3. motherv2_follower
Person following logic with PID control.
- Subscribes: `/motherv2/detections`
- Publishes: `/motherv2/cmd_motor` (custom msg: left/right speed + direction)
- **Priority 1**: Detect person → rotate toward person (angular PID based on bbox center x)
- **Priority 2**: Maintain distance (distance PID based on bbox height/width)
- **Distance estimation**: bbox height as proxy (larger = closer)
- Target bbox height for "ideal distance" (tunable parameter)
- Dead zone: if person near center and at right distance → stop
- Speed limits for 200mm corridor environment

### 4. motherv2_serial
STM32 serial communication node.
- Subscribes: `/motherv2/cmd_motor`
- Handles: dead zone compensation (min_speed offset)
- Handles: graceful shutdown → sends `s` on SIGINT/SIGTERM
- Serial: 115200 baud, 8N1 (configurable)
- Rate limiting to avoid flooding STM32

### 5. motherv2_web
Web visualization server (stdlib MJPEG stream).
- Subscribes: `/motherv2/detection_image`
- Serves MJPEG stream on HTTP port 8080
- Lightweight: single thread, no rosbridge overhead
- Shows: annotated camera feed with bounding boxes, distance info

### 6. motherv2_bringup
Launch files and convenience scripts.
- Launch file: starts all nodes with parameters
- `build.sh`: colcon build + source
- `run.sh`: launch execution
- `test_vision.sh`: camera + detection + web only (no motor)

### 7. motherv2_interfaces
Custom message definitions.
- `Detection.msg`: bbox (x, y, w, h), confidence, class_id
- `DetectionArray.msg`: header, Detection[]
- `MotorCommand.msg`: left_speed, right_speed, left_dir, right_dir

---

## Implementation Phases

### Phase 1: Infrastructure [Priority: HIGH]
- [x] Create PLAN.md
- [x] Create motherv2_interfaces package (custom messages)
- [x] Create motherv2_serial package (motor control node)
- [x] Create build.sh, run.sh, test_vision.sh scripts

### Phase 2: Camera [Priority: HIGH]
- [x] Clone and build camera_ros (christianrauch/camera_ros)
- [x] Configure camera_ros for IMX219 (orientation=180, BGR output)
- [x] Remap camera_ros topics to /motherv2/image_raw
- ~~Custom OpenCV V4L2 node~~ (failed: CSI needs libcamera ISP pipeline)
- ~~Custom libcamera Python node~~ (failed: API v0.2.0 missing features)
- ~~rpicam-vid pipe~~ (worked but pipe→topic latency issues)

### Phase 3: Detection [Priority: HIGHEST]
- [x] Create motherv2_detection package
- [x] Download/prepare YOLOv8n ONNX model (320x320, 13MB)
- [x] Implement async detection pipeline
- [ ] Test person detection at target FPS

### Phase 4: Following [Priority: HIGH]
- [x] Create motherv2_follower package
- [x] Implement angular PID (rotate toward person)
- [x] Implement distance PID (maintain distance)
- [x] Motor dead zone compensation in serial node
- [x] Graceful shutdown (send 's' on Ctrl+C)

### Phase 5: Visualization [Priority: MEDIUM]
- [x] Create motherv2_web package
- [x] MJPEG streaming server (stdlib http.server)
- [x] Overlay detection info on stream
- [ ] Verify web stream works end-to-end

### Phase 6: Integration [Priority: HIGH]
- [x] Update launch files for camera_ros
- [x] Full build successful (7 packages)
- [ ] End-to-end testing

### Phase 7: Fallback/Enhancement [Priority: LOW]
- [x] Document BLE RSSI-based tracking fallback (ESP32)
- [x] Document ultrasonic/LiDAR distance supplement
- [ ] Consider multi-Pi detection offloading architecture

---

## Key Design Decisions

### Detection Model: YOLOv8n ONNX
- Smallest YOLO variant, good accuracy/speed tradeoff
- ONNX Runtime already installed, no extra deps
- 320x320 input → ~100-200ms inference on RPi4 CPU
- Async pipeline: camera feeds at 30fps, detection processes latest frame
- Person-only filtering reduces post-processing overhead

### Camera: camera_ros (libcamera backend)
- CSI cameras (IMX219) require ISP pipeline → V4L2 direct access gives raw Bayer only
- camera_ros uses libcamera natively, outputs proper RGB/BGR through ISP
- orientation parameter handles 180° flip in hardware
- Tried alternatives that failed:
  - OpenCV V4L2: raw Bayer (8UC1), no color
  - libcamera Python API v0.2.0: missing request_completed, buffer assertion errors
  - rpicam-vid pipe: works but topic publishing had latency issues

### Web Visualization: stdlib MJPEG
- No Flask/rosbridge dependency needed
- stdlib http.server with MJPEG multipart stream
- Works in any browser with `<img src="http://robot:8080/stream">`

### Motor Dead Zone Compensation
- Configurable `min_speed` parameter (default: 80, tunable)
- If commanded_speed > 0: actual = min_speed + (commanded / 255) * (255 - min_speed)
- Ensures motor always receives enough power to actually move

### Corridor Navigation
- Max speed limited for 200mm corridors (robot is 140mm → 30mm clearance each side)
- Slow rotation speed to avoid wall collisions
- Conservative following distance

### Fallback Tracking (Future)
- **BLE RSSI**: ESP32 beacon on person, ESP32 receiver on robot
  - Triangulation with 2+ receivers or signal strength for rough direction
  - Cost: ~$5 per ESP32
- **Ultrasonic**: HC-SR04 for distance verification, supplement camera depth estimate
- **LiDAR**: TFMini for precise distance at ~$15
- These supplement camera, not replace it

---

## Troubleshooting Log

### Camera Issues
1. **OpenCV V4L2** → CSI /dev/video0 is unicam raw Bayer (8UC1), not usable
2. **libcamera Python v0.2.0** → `request_completed` signal missing, `prepareIsp()` assertion fail
3. **rpicam-vid pipe** → YUV420 pipe works but frame delivery to ROS topics unreliable
4. **Solution**: Use `camera_ros` package which properly integrates libcamera with ROS2

### Parameter Types
- ROS2 launch passes params via YAML → native types (int/float/bool)
- `declare_parameter` must use native type defaults, not strings
