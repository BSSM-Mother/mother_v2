#!/bin/bash
# slam_test.sh — SLAM 단독 테스트 스크립트
# 실행 노드: 라이다 + 카메라 + 객체감지 + SLAM + 웹 대시보드
#
# 사용법:
#   ./slam_test.sh                          # 기본 실행
#   ./slam_test.sh lidar_port:=/dev/ttyUSB1 # 라이다 포트 지정
#   ./slam_test.sh web_port:=9090           # 웹 포트 변경

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

# ── 색상 출력 ────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[slam_test]${NC} $*"; }
warn()  { echo -e "${YELLOW}[slam_test]${NC} $*"; }
error() { echo -e "${RED}[slam_test]${NC} $*" >&2; }

# ── ROS2 환경 ────────────────────────────────────────────────────────────────
if [ ! -f "$ROS_SETUP" ]; then
    error "ROS2 setup 파일을 찾을 수 없습니다: $ROS_SETUP"
    exit 1
fi
source "$ROS_SETUP"

# ── 인자 파싱 ────────────────────────────────────────────────────────────────
LAUNCH_ARGS=()

for arg in "$@"; do
    LAUNCH_ARGS+=("$arg")
done

# ── 워크스페이스 소스 ────────────────────────────────────────────────────────
INSTALL_SETUP="$WS_DIR/install/setup.bash"
if [ ! -f "$INSTALL_SETUP" ]; then
    error "install/setup.bash 없음 — 빌드 필요"
    exit 1
fi
source "$INSTALL_SETUP"

# ── 라이다 포트 확인 ─────────────────────────────────────────────────────────
LIDAR_PORT="/dev/ttyAMA2"
for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == lidar_port:=* ]]; then
        LIDAR_PORT="${arg#lidar_port:=}"
    fi
done

if [ ! -e "$LIDAR_PORT" ]; then
    warn "라이다 포트 '$LIDAR_PORT' 없음 — 라이다 없이 계속합니다"
fi

# ── 웹 포트 출력 ─────────────────────────────────────────────────────────────
WEB_PORT="8080"
for arg in "${LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == web_port:=* ]]; then
        WEB_PORT="${arg#web_port:=}"
    fi
done

info "SLAM 테스트 시작"
info "웹 대시보드: http://$(hostname -I | awk '{print $1}'):${WEB_PORT}"
info "라이다 포트: ${LIDAR_PORT}"
info "종료: Ctrl+C"
echo ""

# ── 런치 ────────────────────────────────────────────────────────────────────
exec ros2 launch motherv2_bringup slam_test.launch.py "${LAUNCH_ARGS[@]}"
