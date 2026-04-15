#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "=== MotherV2 Build ==="
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --parallel-workers 2 \
  --packages-select \
    echo_lidar \
    motherv2_interfaces \
    motherv2_camera \
    motherv2_detection \
    motherv2_follower \
    motherv2_serial \
    motherv2_web \
    motherv2_mqtt \
    motherv2_slam \
    motherv2_bringup

source install/setup.bash
echo "=== Build Complete ==="
echo "Run: source install/setup.bash"
