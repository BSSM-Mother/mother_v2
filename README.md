# MotherV2 — Object Following Robot

Raspberry Pi 5 + ROS2 Jazzy 기반의 자율 추적 로봇.
카메라로 대상(기본: 페트병)을 인식하고 PID 제어로 따라다닌다.

---

## 시스템 구성

```
[IMX219 카메라]
      |
      | /motherv2/image_raw  (sensor_msgs/Image, BEST_EFFORT, depth=1)
      v
[camera_node]  ── camera_ros 패키지, libcamera 백엔드
      |
      +──────────────────────────────> [web_node]  → :8080 MJPEG 스트림
      |
      v
[detection_node]
  - 백그라운드 스레드: MediaPipe EfficientDet-Lite0 추론 (0.5초 간격)
  - 메인 콜백: OpenCV MOSSE 트래커 (~30fps)
      |
      | /motherv2/detections  (DetectionArray)
      v
[follower_node]  ── PID 제어, 20Hz
      |
      | /motherv2/cmd_motor  (MotorCommand)
      v
[serial_node]  ── UART → STM32
      |
      v
[모터 L/R]
```

---

## 노드별 동작 원리

### camera_node (camera_ros)
- libcamera 백엔드로 IMX219 카메라를 직접 제어
- `BGR888` 포맷으로 `/motherv2/image_raw` 퍼블리시
- 기본 해상도: 640×360 @ 30fps

### detection_node
- **핵심 구조**: Detection + Tracking 분리
  - 추론(MediaPipe): 별도 스레드에서 0.5초(=2fps)마다 실행 — CPU 부하 제한
  - 트래킹(MOSSE): 메인 콜백에서 매 프레임(~30fps) 실행 — 응답성 유지
- 탐지 대상: `bottle` (COCO class 39). `debug_class` 파라미터로 추가 가능
- 추론 시 내부적으로 640×360으로 다운스케일 후 처리
- 탐지 결과가 있으면 MOSSE 트래커를 해당 bbox로 재초기화
- 탐지가 없어도 트래커는 계속 추적 유지

### follower_node
- Angular PID: 화면 중앙에 대상을 유지 (좌우 steering)
- Distance PID: bbox 높이 비율로 거리 제어 (target_bbox_ratio 기본 0.90)
- 우선순위: angular error > 0.25이면 회전만 하고 전진 안 함
- 후진: bbox가 목표보다 `backward_threshold`(5%) 이상 클 때만 허용
- STOP 커맨드는 중복 전송 안 함 (PWM 버저 소음 방지)

### serial_node
- STM32 UART 프로토콜: `right_speed,left_speed,right_dir,left_dir\n`
- 정지 명령: `s\n`
- Dead zone 보상: 입력 1~255를 `min_speed`~`max_speed`로 선형 매핑
- 안전 타이머: 0.5초간 명령 없으면 자동 정지

### web_node
- `:8080/` — 모터 상태 시각화 대시보드 (Canvas)
- `:8080/stream` — MJPEG 스트림 (~30fps)
- `:8080/snapshot` — 단일 JPEG 캡처

---

## 빌드 & 실행

```bash
# 빌드
./build.sh

# 실행
./run.sh

# 파라미터 오버라이드 예시
./run.sh camera_orientation:=2 conf_threshold:=0.4 debug_class:=0
```

웹 대시보드: `http://<로봇 IP>:8080`

---

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `camera_device` | `0` | 카메라 인덱스 |
| `camera_width/height` | `640/360` | 해상도 |
| `camera_orientation` | `0` | 회전 (2 = 180도 뒤집기) |
| `serial_port` | `/dev/ttyAMA0` | STM32 UART 포트 |
| `conf_threshold` | `0.35` | 탐지 신뢰도 임계값 |
| `target_bbox_ratio` | `0.90` | 목표 거리 (bbox 높이 / 화면 높이) |
| `min_speed` | `130` | 최소 모터 PWM (dead zone 보상) |
| `max_speed` | `150` | 최대 모터 PWM |
| `debug_class` | `-1` | 추가 탐지 클래스 ID (-1 = 비활성) |

---

## ROS2 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/motherv2/image_raw` | `sensor_msgs/Image` | 카메라 원본 (BGR8) |
| `/motherv2/detections` | `DetectionArray` | 탐지/트래킹 결과 |
| `/motherv2/detection_image` | `sensor_msgs/Image` | bbox 오버레이 이미지 |
| `/motherv2/cmd_motor` | `MotorCommand` | 모터 제어 명령 |

---

## 트러블슈팅

### FPS가 안 나온다

**증상**: detection_node 로그에서 Tracking fps가 낮거나 카메라가 끊기는 느낌

**원인 1 — QoS 불일치**
카메라는 `BEST_EFFORT` QoS로 퍼블리시한다. 구독 측도 반드시 `BEST_EFFORT`로 맞춰야 한다. `RELIABLE`로 구독하면 프레임이 쌓이거나 드롭된다.

**원인 2 — 카메라 해상도**
해상도가 높을수록 YUV→BGR 변환과 메모리 복사 오버헤드가 커진다.
640×360이 성능/화질 균형상 최적이었다. 그 이상은 fps 저하 발생.

**원인 3 — MediaPipe 추론 주기**
`detection_interval` 파라미터(기본 0.5초)로 추론 빈도를 제한하고 있다.
이것을 낮추면(예: 0.2초) 탐지 응답은 빨라지지만 CPU 온도/쓰로틀링 유발 가능.

**원인 4 — Raspberry Pi CPU 쓰로틀링**
장시간 실행 시 발열로 CPU 클럭이 낮아진다. 방열판/팬 확인:
```bash
vcgencmd get_throttled   # 0x0이면 정상
vcgencmd measure_temp
```

---

### 카메라가 아예 안 열린다

**증상**: `camera_ros` 노드가 바로 죽거나 이미지가 안 옴

- `/boot/firmware/config.txt`에 `camera_auto_detect=1` 또는 `dtoverlay=imx219` 확인
- `rpicam-hello` 명령으로 카메라 인식 여부 먼저 확인
- 다른 프로세스가 카메라를 점유 중인지 확인 (`fuser /dev/video*`)

---

### 모터가 안 움직인다 / 한쪽만 돈다

**증상**: follower_node 로그는 정상인데 로봇이 안 움직임

- `serial_node` 로그에서 `Serial opened` 메시지 확인
- UART 포트 확인: `ls /dev/ttyAMA*` — 기본값은 `/dev/ttyAMA0`
- `raspi-config`에서 Serial Port 하드웨어 활성화 여부 확인 (login shell은 비활성화)
- STM32 펌웨어의 프로토콜 포맷이 `right_speed,left_speed,right_dir,left_dir\n` 맞는지 확인
- `min_speed`가 너무 높으면 모터가 항상 켜져 있어 정지 불가. 너무 낮으면 출발 자체가 안 됨 — 실제 dead zone에 맞게 조정 필요

---

### 로봇이 대상을 계속 놓친다 (SEARCHING 상태)

- `conf_threshold`를 낮춰 본다 (0.35 → 0.25)
- `debug_class:=0`으로 사람도 함께 탐지해 테스트
- 조명이 어두우면 탐지율 급감 — 충분한 조명 확보
- MOSSE 트래커는 빠른 움직임이나 완전한 가림(occlusion)에 약함. 이 경우 탐지 interval을 줄여야 복구가 빨라짐

---

### Ctrl+C 후에도 모터가 계속 돈다

- `serial_node`의 SIGINT 핸들러가 `s\n`을 전송한다
- 만약 serial_node가 먼저 죽으면 안전 타이머(0.5초)가 작동하지 않음
- 강제 종료 시: STM32 리셋 버튼 또는 전원 차단

---

## 패키지 구조

```
src/
├── motherv2_interfaces/   # 커스텀 메시지 (Detection, DetectionArray, MotorCommand)
├── motherv2_camera/       # rpicam-vid 기반 카메라 노드 (대안 구현)
├── motherv2_detection/    # MediaPipe + MOSSE 탐지/추적 노드
│   └── models/
│       ├── efficientdet_lite0.tflite   # 기본 사용 모델
│       └── yolov8n.onnx               # 미사용 (실험용)
├── motherv2_follower/     # PID 추적 제어 노드
├── motherv2_serial/       # STM32 UART 통신 노드
├── motherv2_web/          # MJPEG 스트림 + 모터 대시보드
├── motherv2_bringup/      # 런치 파일
└── camera_ros/            # libcamera ROS2 래퍼 (외부 패키지)
```
