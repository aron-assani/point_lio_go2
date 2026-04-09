#!/bin/bash
# Convenient wrapper script for parameter optimization

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROSBAG_PATH="${1:?Error: Please provide rosbag path as argument}"
CONFIG_PATH="${2:-/root/ros2_ws/src/point_lio/config/utlidar.yaml}"

echo "=========================================="
echo "Point-LIO Parameter Optimizer"
echo "=========================================="
echo "Rosbag: $ROSBAG_PATH"
echo "Config: $CONFIG_PATH"
echo ""

# Check if files exist
if [ ! -f "$ROSBAG_PATH" ]; then
    echo "✗ Error: Rosbag file not found: $ROSBAG_PATH"
    exit 1
fi

if [ ! -f "$CONFIG_PATH" ]; then
    echo "✗ Error: Config file not found: $CONFIG_PATH"
    exit 1
fi

# Source ROS setup
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Run optimizer with proper error handling
trap 'echo "Optimization interrupted"; exit 1' INT TERM

python3 "$SCRIPT_DIR/parameter_optimizer.py" "$ROSBAG_PATH" "$CONFIG_PATH"
