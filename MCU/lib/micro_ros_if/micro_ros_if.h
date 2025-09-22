#pragma once
#include <Arduino.h>
#include "VelocityController.h"
#include "Encoder.h"
#include "IMU.h"

extern "C" {
  #include <rcl/rcl.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>
  #include <std_msgs/msg/int32_multi_array.h>
  #include <std_msgs/msg/float32.h>
  #include <geometry_msgs/msg/twist.h>
}

class MicroRosIF {
public:
  bool init();
  void spin_once(float now, float dt_control, float dt_pub_ticks, float dt_pub_yaw);

  // Hook from main to run control + publications
  void setModules(VelocityController* vc, Encoder* enc, IMU* imu){ vc_=vc; enc_=enc; imu_=imu; }

private:
  static void cmdvel_cb(const void* msgin, void* arg);

  // ROS core
  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rcl_timer_t timer_dummy_; // optional (not used)
  rclc_executor_t exec_;

  rcl_subscription_t sub_cmdvel_;
  geometry_msgs__msg__Twist cmdvel_msg_;

  rcl_publisher_t pub_ticks_;
  std_msgs__msg__Int32MultiArray ticks_msg_;

  rcl_publisher_t pub_yaw_;
  std_msgs__msg__Float32 yaw_msg_;

  VelocityController* vc_ = nullptr;
  Encoder* enc_ = nullptr;
  IMU* imu_ = nullptr;
};
