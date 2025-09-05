#!/bin/bash
# Activate venv
source /home/prukubt/Agriculture_Robot/ros2_ws/venv/bin/activate

# Run Flask camera stream with all arguments passed from ROS2 launch
exec python -m robot_bringup.camera_stream "$@"
