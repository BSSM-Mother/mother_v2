#!/bin/bash
set -e

cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

IP=$(hostname -I | awk '{print $1}')
echo "=== MotherV2 Vision Test ==="
echo "Camera + Detection + Web only (no motor)"
echo "Web stream: http://${IP}:8080"
echo ""
echo "Options:"
echo "  debug_class:=32   -> sports ball (ping pong, soccer, tennis...)"
echo "  debug_class:=39   -> bottle"
echo "  debug_class:=41   -> cup"
echo "  debug_class:=56   -> chair"
echo "  debug_class:=67   -> cell phone"
echo "  conf_threshold:=0.3 -> lower = more detections"
echo ""
echo "Press Ctrl+C to stop"
echo ""

ros2 launch motherv2_bringup vision_test.launch.py "$@"
