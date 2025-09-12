#include "imu_node.h"
#include <math.h>
#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h> // --- PATCH: ใช้ default2

static ImuPublisher *g_self = nullptr;

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif

// ==================== Utilities ====================
int64_t ImuPublisher::now_nanos() { return (int64_t)micros() * 1000LL; } // ใช้ micros() → ns (fallback)

// ตั้งค่าคูณแนวทแยงสำหรับ covariance (double[9])
void ImuPublisher::set_cov_diag(double cov[9], double rx, double ry, double rz)
{
  for (int i = 0; i < 9; i++)
    cov[i] = 0.0;
  cov[0] = rx; // xx
  cov[4] = ry; // yy
  cov[8] = rz; // zz
}

void ImuPublisher::euler_to_quat(float r, float p, float y, float &qx, float &qy, float &qz, float &qw)
{
  float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f);
  float cp = cosf(p * 0.5f), sp = sinf(p * 0.5f);
  float cy = cosf(y * 0.5f), sy = sinf(y * 0.5f);

  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}

// ==================== IMU Setup & Calibration ====================
bool ImuPublisher::imu_setup_and_calibrate_()
{
  if (!mpu_.begin())
    return false;

  mpu_.setAccelerometerRange(params_.acc_range);
  mpu_.setGyroRange(params_.gyr_range);
  mpu_.setFilterBandwidth(params_.lpf_bw);

  delay(50);

  gbx_ = gby_ = gbz_ = 0.0f;

  for (int i = 0; i < params_.calib_samples; i++)
  {
    sensors_event_t a, g, t;
    mpu_.getEvent(&a, &g, &t);
    gbx_ += g.gyro.x;
    gby_ += g.gyro.y;
    gbz_ += g.gyro.z;
    delay((int)(1000.0f / params_.loop_hz));
  }

  gbx_ /= params_.calib_samples;
  gby_ /= params_.calib_samples;
  gbz_ /= params_.calib_samples;

  return true;
}

// ==================== Timer Callback ====================
void ImuPublisher::timer_cb_(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/)
{
  if (!g_self || !g_self->imu_ready_)
    return;

  auto &self = *g_self;
  float DT = 1.0f / self.params_.loop_hz;

  sensors_event_t a, g, t;
  self.mpu_.getEvent(&a, &g, &t);

  // --- linear acceleration ---
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // --- gyro (bias corrected) ---
  float gx = g.gyro.x - self.gbx_;
  float gy = g.gyro.y - self.gby_;
  float gz = g.gyro.z - self.gbz_;
  self.gz_ = gz;

  // --- Roll / Pitch จาก accel ---
  float roll_acc = atan2f(ay, az);
  float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

  // --- Roll / Pitch / Yaw จาก gyro ---
  float roll_gyro = self.roll_ + gx * DT;
  float pitch_gyro = self.pitch_ + gy * DT;
  float yaw_gyro = self.yaw_ + gz * DT;

  // --- Complementary Filter ---
  float a_cf = self.params_.cf_alpha;
  self.roll_ = a_cf * roll_gyro + (1.0f - a_cf) * roll_acc;
  self.pitch_ = a_cf * pitch_gyro + (1.0f - a_cf) * pitch_acc;
  self.yaw_ = yaw_gyro;

  if (self.yaw_ > M_PI)
    self.yaw_ -= 2.0f * M_PI;
  if (self.yaw_ < -M_PI)
    self.yaw_ += 2.0f * M_PI;

  // --- ROS2 IMU message ---
  int64_t tn = now_nanos();
  self.imu_msg_.header.stamp.sec = (int32_t)(tn / 1000000000LL);
  self.imu_msg_.header.stamp.nanosec = (uint32_t)(tn % 1000000000LL);

  float qx, qy, qz, qw;
  euler_to_quat(self.roll_, self.pitch_, self.yaw_, qx, qy, qz, qw);
  self.imu_msg_.orientation.x = qx;
  self.imu_msg_.orientation.y = qy;
  self.imu_msg_.orientation.z = qz;
  self.imu_msg_.orientation.w = qw;

  self.imu_msg_.angular_velocity.x = gx;
  self.imu_msg_.angular_velocity.y = gy;
  self.imu_msg_.angular_velocity.z = gz;

  self.imu_msg_.linear_acceleration.x = ax;
  self.imu_msg_.linear_acceleration.y = ay;
  self.imu_msg_.linear_acceleration.z = az;

  // --- PATCH: เก็บผลลัพธ์ publish กัน warning และตรวจเช็คได้ภายหลัง
  rcl_ret_t pub_ret = rcl_publish(&self.imu_pub_, &self.imu_msg_, nullptr);
  (void)pub_ret;
}

// ==================== Initialization ====================
bool ImuPublisher::init(const Params &p,
                        rcl_node_t *node,
                        rclc_support_t *support,
                        rclc_executor_t *executor)
{
  params_ = p;
  g_self = this;

  // --- I2C ---
  Wire.begin(params_.sda, params_.scl);
  Wire.setClock(400000);

  imu_ready_ = imu_setup_and_calibrate_();

  // --- Init message ให้ครบถ้วนก่อนใช้งาน
  sensor_msgs__msg__Imu__init(&imu_msg_);

  // --- ROS2 publisher ---
  rcl_ret_t ret = rclc_publisher_init_default(
      &imu_pub_, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data");
  if (ret != RCL_RET_OK)
    return false;

  // --- Header frame id ---
  // imu_msg_.header.frame_id ถูก __init แล้ว สามารถ assign ได้เลย
  if (!rosidl_runtime_c__String__assign(&imu_msg_.header.frame_id, params_.frame_id))
  {
    return false;
  }

  // --- Covariance (double) ---
  auto sq = [](double x)
  { return x * x; };
  double ori_cov_rp = sq(2.0 * DEG_TO_RAD); // roll/pitch ~ 2 deg
  double ori_cov_y = sq(6.0 * DEG_TO_RAD);  // yaw ~ 6 deg (จาก gyro-only)
  double ang_cov = sq(2.5);                 // rad/s
  double lin_cov = sq(0.4);                 // m/s^2

  set_cov_diag(imu_msg_.orientation_covariance, ori_cov_rp, ori_cov_rp, ori_cov_y);
  set_cov_diag(imu_msg_.angular_velocity_covariance, ang_cov, ang_cov, ang_cov);
  set_cov_diag(imu_msg_.linear_acceleration_covariance, lin_cov, lin_cov, lin_cov);

  // --- Timer (ใช้ default2 + autostart=true) ---
  const uint64_t period_ns = RCL_MS_TO_NS((int)(1000.0f / params_.loop_hz));
  ret = rclc_timer_init_default2(
      &timer_, support,
      period_ns,
      ImuPublisher::timer_cb_,
      true /* autostart */);
  if (ret != RCL_RET_OK)
    return false;

  // --- Add timer เข้า executor ---
  ret = rclc_executor_add_timer(executor, &timer_);
  if (ret != RCL_RET_OK)
    return false;

  return imu_ready_;
}

void ImuPublisher::spinOnce() {}
