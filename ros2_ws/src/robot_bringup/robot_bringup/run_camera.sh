#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

# Activate virtual environment
source /home/prukubt/Agriculture_Robot/ros2_ws/venv/bin/activate

# Optional: log Python path
echo "Using Python: $(which python)"
echo "Arguments: $@"

# Run Flask camera stream with all arguments passed from ROS2 launch
exec python -m robot_bringup.camera_stream "$@"
