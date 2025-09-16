#pragma once // For MPU6050
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include "config.h"

class ImuPublisher
{
public:
  struct Params
  {
    int sda = I2C_SDA_PIN, scl = I2C_SCL_PIN;
    float loop_hz = IMU_HZ;
    const char *frame_id = IMU_FRAME;
    float cf_alpha = CF_ALPHA;
    int calib_samples = IMU_CALIB_SAMPLES;
    mpu6050_accel_range_t acc_range = MPU6050_RANGE_4_G;
    mpu6050_gyro_range_t gyr_range = MPU6050_RANGE_500_DEG;
    mpu6050_bandwidth_t lpf_bw = MPU6050_BAND_21_HZ;
  };

  bool init(const Params &p,
            rcl_node_t *node,
            rclc_support_t *support,
            rclc_executor_t *executor);
  void spinOnce();

  // optional: getter yaw-rate ถ้าจะใช้ใน Body PID
  float getYawRateRad() const { return gz_; }

private:
  rcl_publisher_t imu_pub_;
  rcl_timer_t timer_;
  sensor_msgs__msg__Imu imu_msg_;

  Params params_;
  Adafruit_MPU6050 mpu_;
  bool imu_ready_ = false;

  float roll_ = 0.0f, pitch_ = 0.0f, yaw_ = 0.0f;
  float gbx_ = 0, gby_ = 0, gbz_ = 0;
  float gz_ = 0.0f; // rad/s

  static int64_t now_nanos();
  static void set_cov_diag(double cov[9], double rx, double ry, double rz);
  static void euler_to_quat(float r, float p, float y, float &qx, float &qy, float &qz, float &qw);

  bool imu_setup_and_calibrate_();
  static void timer_cb_(rcl_timer_t *timer, int64_t last_call_time);
};
