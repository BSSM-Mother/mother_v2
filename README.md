# MotherV2 — Object Following Robot

Raspberry Pi 4B + ROS2 Jazzy 기반의 자율 추적 로봇.  
LiDAR + 카메라로 대상(기본: 사람)을 인식·추적하며, SLAM 맵 기반 동적 객체 탐지 및 자율 탐색을 지원한다.

---

## 시스템 구성

```
[IMX219 카메라]         [RPLIDAR A1/A2]
      |                        |
      | /motherv2/image_raw    | /scan
      v                        v
[detection_node]         [slam_toolbox]──→ /map (OccupancyGrid)
      |                        |              |
      | /motherv2/detections   | map TF       |
      v                        v              |
[follower_node]←──────[slam_localization_node]
      |                        |
      | /motherv2/cmd_motor    | /motherv2/object_estimates
      v                        |
[serial_node]──→ STM32         v
                         [web_node] :8080
                                |
                         [explore_node] (자율 탐색)
```

---

## 노드별 동작 원리

### camera_node (camera_ros)
- libcamera 백엔드로 IMX219 카메라를 직접 제어
- `BGR888` 포맷으로 `/motherv2/image_raw` 퍼블리시
- 기본 해상도: 640×360 @ 30fps

### detection_node
- **핵심 구조**: Detection + Tracking 분리
  - 추론(MediaPipe): 별도 스레드에서 0.5초마다 실행 — CPU 부하 제한
  - 트래킹(MOSSE): 메인 콜백에서 매 프레임(~30fps) 실행 — 응답성 유지
- 탐지 대상: 사람 (COCO class 0). `target_class_id` 파라미터로 변경 가능

### slam_localization_node
- LiDAR 깊이와 카메라 탐지 결과를 융합해 객체의 SLAM 맵 좌표를 추정
- **3단계 추적**
  1. 카메라 + LiDAR: bbox 중심 방향으로 LiDAR 거리 측정 → 맵 좌표 변환
  2. LiDAR only (카메라 FOV 이탈 시): 정적 맵과 현재 스캔을 비교해 동적 객체(사람) 탐지
  3. Dead-reckoning: 객체 완전 소실 시 마지막 속도벡터로 2초간 위치 예측
- **SLAM 모드**
  - `mapping`: slam_toolbox로 맵 생성 중. LiDAR 방향 기반 추정
  - `tracking`: 저장된 맵 스냅샷과 현재 스캔의 차분으로 동적 객체 추정 (더 정확)

### explore_node
- `/motherv2/slam_mode = "explore"` 수신 시 자동 활성화
- Nav2 없이 LiDAR + OccupancyGrid만으로 frontier(미탐색 경계) 기반 자율 맵핑
- 상태 머신: `idle → rotating → driving → idle` (반복)
  - frontier 소진 시 "tracking" 모드로 자동 전환
  - 정체(8초/12cm 미만 이동) 감지 시 후진 + 우회전 복구
- 용도: 방 하나 크기의 환경을 처음 탐색할 때 사용

### follower_node
- Angular PID: 화면 중앙에 대상을 유지 (좌우 steering)
- Distance PID: LiDAR 깊이 기반 거리 제어 (`target_depth_m` 기본 0.8m)
- `use_slam_depth=true`이면 LiDAR 깊이 직접 사용, 아니면 bbox 높이 비율로 추정

### serial_node
- STM32 UART 프로토콜: `right_speed,left_speed,right_dir,left_dir\n`
- Dead zone 보상: 입력을 `min_speed`~`max_speed`로 선형 매핑
- 안전 타이머: 0.5초간 명령 없으면 자동 정지

### web_node
- `:8080/` — 웹 대시보드 (라이브 카메라 + SLAM 맵 + 로그)
- `:8080/stream` — MJPEG 스트림

---

## 빌드 & 실행

```bash
# 빌드
./build.sh

# 실행 (SLAM 포함)
./run.sh

# SLAM 없이 실행 (카메라+탐지만)
./run.sh use_slam:=false

# 파라미터 오버라이드 예시
./run.sh camera_orientation:=2 lidar_port:=/dev/ttyAMA2

# API 서버 연동 (API_URL 환경변수 설정 시 api_node 자동 실행)
API_URL=http://your-server/api ./run.sh
```

웹 대시보드: `http://<로봇 IP>:8080`

---

## 웹 대시보드 사용 가이드

### 화면 구성

```
┌────────────────────────────────────────────────────────┐
│  [카메라 스트림]           [SLAM 맵]                    │
│                                                        │
│  • 실시간 영상              • 점유 격자 맵 (회색/흰색/검정) │
│  • 탐지 bbox 오버레이       • 로봇 위치 (청록색 화살표)    │
│  • 추적 대상 표시           • 객체 추정 위치 (주황색 원)   │
│                             • LiDAR 스캔 포인트 (녹색)   │
│                             • 인덱싱된 장소 (노란색 레이블)│
├──────────────────────────┬─────────────────────────────┤
│  [모터 상태]              │  [로봇 로그]                 │
│  방향/속도 표시           │  이벤트 시간순 목록           │
├──────────────────────────┴─────────────────────────────┤
│  [제어 버튼]                                            │
│  📂 맵 불러오기  💾 맵 저장                              │
│  [MAPPING] [추적 모드] [매핑 모드] [자동 탐색]           │
│  + 현재 위치 장소 이름 저장                              │
└────────────────────────────────────────────────────────┘
```

### 기능별 사용법

#### 1. 처음 사용 — 맵 생성

1. 로봇을 방에 배치하고 `./run.sh` 실행
2. 웹 대시보드 접속 → **매핑 모드** 버튼 확인 (기본값)
3. 방법 A — 수동: 로봇을 직접 밀거나 조종하며 방 전체를 돌기
4. 방법 B — 자동: **자동 탐색** 버튼 클릭 → 로봇이 스스로 frontier 탐색 시작
   - 탐색 완료(frontier 없음) 시 자동으로 추적 모드 전환됨
5. 맵이 충분히 생성되면 **💾 맵 저장** 클릭 → `~/maps/slam_map.*` 파일로 저장

#### 2. 매일 사용 — 추적 시작

1. **로봇을 맵 생성 당시와 같은 위치에 배치** (초기 위치 중요!)
2. `./run.sh` 실행
3. **📂 맵 불러오기** 클릭 → 저장된 맵 로드 + 추적 모드 자동 전환
4. 로봇이 대상을 자동으로 따라다니기 시작

> **주의**: slam_toolbox는 로봇 초기 위치를 맵의 원점으로 가정한다. 매 실행 시 동일한 시작점에서 켜야 로컬라이제이션이 정확하다.

#### 3. 장소 인덱싱

- 로봇이 특정 위치에 있을 때 하단 입력창에 이름 입력 후 **추가** 클릭
- SLAM 맵에 노란색 레이블로 표시됨
- 장소 정보는 `~/.ros/motherv2/places.json`에 저장

#### 4. SLAM 모드 전환

| 버튼 | 모드 | 설명 |
|------|------|------|
| 매핑 모드 | `mapping` | slam_toolbox로 맵 생성 중. LiDAR 방향 기반 객체 추정 |
| 추적 모드 | `tracking` | 저장된 맵과 현재 스캔 차분으로 동적 객체 탐지 |
| 자동 탐색 | `explore` | frontier 기반 자율 맵핑 시작 |

---

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `camera_device` | `0` | 카메라 인덱스 |
| `camera_width/height` | `640/360` | 해상도 |
| `camera_orientation` | `0` | 회전 (2 = 180도) |
| `serial_port` | `/dev/ttyAMA0` | STM32 UART 포트 |
| `lidar_port` | `/dev/ttyAMA2` | RPLIDAR 시리얼 포트 |
| `use_slam` | `true` | SLAM 활성화 여부 |
| `conf_threshold` | `0.35` | 탐지 신뢰도 임계값 |
| `target_depth_m` | `0.8` | LiDAR 기반 목표 추적 거리 (m) |
| `camera_hfov_deg` | `62.2` | IMX219 수평 화각 (도) |
| `lidar_angle_offset_rad` | `0.0` | 카메라-LiDAR 수평 각도 오프셋 |
| `min_speed` | `130` | 최소 모터 PWM |
| `max_speed` | `150` | 최대 모터 PWM |
| `mqtt_broker` | `broker.emqx.io` | MQTT 브로커 주소 |

---

## ROS2 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/motherv2/image_raw` | `sensor_msgs/Image` | 카메라 원본 (BGR8) |
| `/motherv2/detections` | `DetectionArray` | 탐지/트래킹 결과 |
| `/motherv2/detection_image` | `sensor_msgs/Image` | bbox 오버레이 이미지 |
| `/motherv2/cmd_motor` | `MotorCommand` | 모터 제어 명령 |
| `/motherv2/object_estimates` | `ObjectEstimateArray` | SLAM 좌표 객체 위치 추정 |
| `/motherv2/slam_mode` | `std_msgs/String` | SLAM 모드 (`mapping`/`tracking`/`explore`) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 스캔 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 점유 격자 맵 |

---

## 패키지 구조

```
src/
├── motherv2_interfaces/     # 커스텀 메시지 (Detection, DetectionArray, MotorCommand, ObjectEstimateArray)
├── motherv2_detection/      # MediaPipe + MOSSE 탐지/추적 노드
│   └── models/
│       └── efficientdet_lite0.tflite
├── motherv2_follower/       # LiDAR 기반 PID 추적 제어 노드
├── motherv2_serial/         # STM32 UART 통신 노드
├── motherv2_slam/           # SLAM 연동 노드
│   ├── slam_localization_node.py   # LiDAR+카메라 융합, 동적 객체 탐지
│   └── explore_node.py             # frontier 기반 자율 탐색
├── motherv2_web/            # 웹 대시보드 + HTTP API
│   ├── web_node.py                 # MJPEG 스트림 + 대시보드 + SLAM 시각화
│   └── api_node.py                 # 외부 API 폴링 (API_URL 설정 시)
├── motherv2_mqtt/           # MQTT 퍼블리시 노드 (ESP32 릴레이 제어)
├── motherv2_bringup/        # 런치 파일 + 설정
│   ├── launch/motherv2.launch.py   # 전체 시스템 런치
│   └── launch/slam.launch.py       # SLAM 서브 런치
└── echo_lidar/              # RPLIDAR ROS2 드라이버
```

---

## 트러블슈팅

### SLAM 맵이 웹에서 안 보인다

- slam_toolbox가 `/map` 토픽을 TRANSIENT_LOCAL QoS로 퍼블리시한다
- 구독 측(web_node)도 반드시 TRANSIENT_LOCAL이어야 연결됨
- `ros2 topic info /map --verbose`로 QoS 확인

### 맵 불러오기 후 위치가 틀리다

- 로봇 초기 위치가 맵 생성 당시와 다를 때 발생
- 해결: 로봇을 맵 생성 시작 지점(보통 방 출입구)으로 이동 후 시스템 재시작

### LiDAR가 인식 안 된다

```bash
ls /dev/ttyAMA*       # 포트 확인
sudo chmod 666 /dev/ttyAMA2   # 권한 부여
# 또는 dialout 그룹에 사용자 추가
sudo usermod -aG dialout robot
```

### 자동 탐색이 제자리 회전만 한다

- frontier 클러스터 최소 크기(`MIN_CLUSTER=5`)에 미달 — 맵이 너무 작거나 LiDAR 노이즈
- `ARRIVE_DIST`(0.6m)를 늘리거나 `MIN_CLUSTER`를 낮춰 볼 것

### 모터가 안 움직인다 / 한쪽만 돈다

- `serial_node` 로그에서 `Serial opened` 확인
- UART 포트 확인: `ls /dev/ttyAMA*` — 기본값 `/dev/ttyAMA0`
- STM32 프로토콜 포맷 확인: `right_speed,left_speed,right_dir,left_dir\n`

### FPS가 안 나온다

- Raspberry Pi 발열 확인: `vcgencmd get_throttled`  (`0x0` = 정상)
- 해상도를 낮추거나 detection interval을 늘린다
- 카메라 QoS는 반드시 BEST_EFFORT로 구독해야 한다

### Ctrl+C 후 모터가 계속 돈다

- `serial_node` SIGINT 핸들러가 정지 명령을 전송하지 못한 경우
- 강제 정지: STM32 리셋 버튼 또는 전원 차단
