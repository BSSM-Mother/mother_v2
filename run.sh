#!/bin/bash
set -e

cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=== MotherV2 Launching ==="
echo "Web stream: http://$(hostname -I | awk '{print $1}'):8080"
echo "Press Ctrl+C to stop (motors will be safely stopped)"
echo ""

ros2 launch motherv2_bringup motherv2.launch.py "$@"
