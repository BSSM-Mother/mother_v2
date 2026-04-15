#!/bin/bash
set -e

cd "$(dirname "$0")"

# API 연동 없이 무조건 follow 모드로 강제 실행
unset API_URL

source /opt/ros/jazzy/setup.bash
source install/setup.bash

IP=$(hostname -I | awk '{print $1}')
echo "=== MotherV2 System Test (Follow 강제 활성화) ==="
echo "API 없이 실행 — follow_default=True 강제 적용"
echo "Web stream: http://${IP}:8080"
echo ""
echo "Options (예시):"
echo "  debug_class:=39        -> bottle 추적"
echo "  debug_class:=0         -> person 추적 (기본)"
echo "  conf_threshold:=0.3    -> lower = 더 많은 감지"
echo "  max_speed:=150         -> 최대 속도 조정"
echo "  target_bbox_ratio:=0.8 -> 유지 거리 조정 (클수록 가까이)"
echo ""
echo "Press Ctrl+C to stop (motors will be safely stopped)"
echo ""

ros2 launch motherv2_bringup motherv2.launch.py "$@"
