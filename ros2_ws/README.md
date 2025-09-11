# 1) สตาร์ททั้งหมด (agent serial + EKF + monitor)

ros2 launch robot_bringup bringup.launch.py serial_port:=/dev/ttyUSB0

# (หรือ) 2) สตาร์ท agent อย่างเดียว (UDP)

ros2 launch robot_bringup micro_ros_agent.launch.py transport:=udp udp_port:=8888

# จากนั้นรัน EKF แยก:

ros2 run robot_localization ekf_node --ros-args --params-file \
 $(ros2 pkg prefix robot_bringup)/share/robot_bringup/config/ekf.yaml

# ตรวจ topic

ros2 topic hz /wheel/odom
ros2 topic hz /imu/data
ros2 run rviz2 rviz2 # เปิดดู /odometry/filtered และ TF
