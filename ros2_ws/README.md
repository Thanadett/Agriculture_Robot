## ROS 2 Topics (Agriculture_Robot)

| Topic          | Direction   | Type                         | Rate   | Description                                        |
|----------------|-------------|------------------------------|--------|----------------------------------------------------|
| `/cmd_vel`     | Subscribed  | `geometry_msgs/msg/Twist`    | ~50 Hz | คำสั่งขับเคลื่อน (linear.x, angular.z)ส่งจาก Pi→ ESP32 |
| `/wheel_ticks` | Published   | `std_msgs/msg/Int32MultiArray` | ~50 Hz | ค่า encoder ticks ของล้อ [lf, lr, rf, rr]         |
| `/yaw_deg`     | Published   | `std_msgs/msg/Float32`       | ~100 Hz| มุม yaw จาก IMU (องศา)                           |
| `/imu/data`    | Published   | `sensor_msgs/msg/Imu`        | ~100 Hz| ข้อมูล IMU (orientation, angular vel., accel) *optional* |
| `/odom`        | Published   | `nav_msgs/msg/Odometry`      | ~10 Hz | Odometry รวม encoder + yaw                        |
| `/tf`          | Published   | `tf2_msgs/msg/TFMessage`     | ~10 Hz | Transform: `odom → base_link` (+ `base_link → imu_link`) |