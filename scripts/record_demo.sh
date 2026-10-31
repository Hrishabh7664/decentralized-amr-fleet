#!/bin/bash
# record_demo.sh - Record ROS 2 bags of fleet coordination, odometry, and conflict topics
set -e

DURATION="${1:-30}"
OUTPUT_DIR="results/bags/demo_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo "=========================================================="
echo " Recording ROS 2 Bag Demo for ${DURATION} seconds"
echo " Output directory: ${OUTPUT_DIR}"
echo "=========================================================="

docker compose exec simulation bash -c "
  source /opt/ros/humble/setup.bash &&
  source /root/ros2_ws/install/setup.bash &&
  timeout ${DURATION} ros2 bag record -o /root/ros2_ws/${OUTPUT_DIR} \
    /clock \
    /tf \
    /tf_static \
    /fleet/active_robots \
    /fleet/conflicts \
    /fleet/bids \
    /robot_1/odom \
    /robot_1/cmd_vel \
    /robot_2/odom \
    /robot_2/cmd_vel \
    /robot_3/odom \
    /robot_3/cmd_vel
" || true

echo "Recording completed! Bag saved to ${OUTPUT_DIR}"
