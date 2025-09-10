#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

class ImuPublisher {
public:
  // พารามิเตอร์หลัก
  struct Params {
    int   sda=21, scl=22;
    float loop_hz=100.0f;
    const char* frame_id="imu_link";
    // filter & calib
    float cf_alpha=0.96f;  // complementary (0.90–0.98)
    int   calib_samples=800;
    mpu6050_accel_range_t acc_range = MPU6050_RANGE_4_G;
    mpu6050_gyro_range_t  gyr_range = MPU6050_RANGE_500_DEG;
    mpu6050_bandwidth_t   lpf_bw    = MPU6050_BAND_21_HZ;
  };

  bool  init(const Params& p,
             rcl_node_t* node,
             rclc_support_t* support,
             rclc_executor_t* executor);
  void  spinOnce(); // เรียกใน loop (executor จะเรียก timer ให้เอง)

private:
  // micro-ROS
  rcl_publisher_t imu_pub_;
  rcl_timer_t     timer_;
  sensor_msgs__msg__Imu imu_msg_;

  // state
  Params params_;
  Adafruit_MPU6050 mpu_;
  bool imu_ready_=false;

  float roll_=0.0f, pitch_=0.0f, yaw_=0.0f;
  float gbx_=0, gby_=0, gbz_=0;

  // helpers
  static int64_t now_nanos();
  static void set_cov_diag(float cov[9], float rx, float ry, float rz);
  static void euler_to_quat(float r,float p,float y,float &qx,float &qy,float &qz,float &qw);

  bool imu_setup_and_calibrate_();
  static void timer_cb_(rcl_timer_t* timer, int64_t last_call_time);
};
