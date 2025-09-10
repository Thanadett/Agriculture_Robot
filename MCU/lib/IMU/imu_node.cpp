#include "imu_node.h"
#include <math.h>

static ImuPublisher* g_self = nullptr; // ใช้เรียกภายใน callback

int64_t ImuPublisher::now_nanos() {
  return (int64_t)micros() * 1000LL;
}

void ImuPublisher::set_cov_diag(float cov[9], float rx, float ry, float rz){
  for (int i=0;i<9;i++) cov[i]=0.0f;
  cov[0]=rx; cov[4]=ry; cov[8]=rz;
}

void ImuPublisher::euler_to_quat(float r,float p,float y,float &qx,float &qy,float &qz,float &qw){
  float cr=cosf(r*0.5f), sr=sinf(r*0.5f);
  float cp=cosf(p*0.5f), sp=sinf(p*0.5f);
  float cy=cosf(y*0.5f), sy=sinf(y*0.5f);
  qw = cr*cp*cy + sr*sp*sy;
  qx = sr*cp*cy - cr*sp*sy;
  qy = cr*sp*cy + sr*cp*sy;
  qz = cr*cp*sy - sr*sp*cy;
}

bool ImuPublisher::imu_setup_and_calibrate_(){
  if(!mpu_.begin()) return false;

  mpu_.setAccelerometerRange(params_.acc_range);
  mpu_.setGyroRange(params_.gyr_range);
  mpu_.setFilterBandwidth(params_.lpf_bw);

  // คาลิเบรตไจโร (อยู่นิ่งตอนบูต)
  delay(50);
  gbx_=gby_=gbz_=0.0f;
  for(int i=0;i<params_.calib_samples;i++){
    sensors_event_t a,g,t; mpu_.getEvent(&a,&g,&t);
    gbx_ += g.gyro.x; gby_ += g.gyro.y; gbz_ += g.gyro.z;
    delay((int)(1000.0f/params_.loop_hz));
  }
  gbx_/=params_.calib_samples; gby_/=params_.calib_samples; gbz_/=params_.calib_samples;
  return true;
}

void ImuPublisher::timer_cb_(rcl_timer_t* timer, int64_t){
  (void)timer;
  if (!g_self || !g_self->imu_ready_) return;

  auto& self = *g_self;
  float DT = 1.0f / self.params_.loop_hz;

  sensors_event_t a,g,t;
  self.mpu_.getEvent(&a,&g,&t);

  // raw (Adafruit: accel in m/s^2, gyro in rad/s)
  float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
  float gx=g.gyro.x - self.gbx_, gy=g.gyro.y - self.gby_, gz=g.gyro.z - self.gbz_;

  // Complementary filter
  float roll_acc  = atan2f(ay, az);
  float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

  float roll_gyro  = self.roll_  + gx*DT;
  float pitch_gyro = self.pitch_ + gy*DT;
  float yaw_gyro   = self.yaw_   + gz*DT;

  float a = self.params_.cf_alpha;
  self.roll_  = a*roll_gyro  + (1.0f-a)*roll_acc;
  self.pitch_ = a*pitch_gyro + (1.0f-a)*pitch_acc;
  self.yaw_   = yaw_gyro;
  if (self.yaw_ >  M_PI) self.yaw_ -= 2.0f*M_PI;
  if (self.yaw_ < -M_PI) self.yaw_ += 2.0f*M_PI;

  // header stamp
  int64_t tn = now_nanos();
  self.imu_msg_.header.stamp.sec     = (int32_t)(tn / 1000000000LL);
  self.imu_msg_.header.stamp.nanosec = (uint32_t)(tn % 1000000000LL);

  // orientation
  float qx,qy,qz,qw;
  euler_to_quat(self.roll_, self.pitch_, self.yaw_, qx,qy,qz,qw);
  self.imu_msg_.orientation.x = qx;
  self.imu_msg_.orientation.y = qy;
  self.imu_msg_.orientation.z = qz;
  self.imu_msg_.orientation.w = qw;

  // angular velocity / linear acceleration
  self.imu_msg_.angular_velocity.x = gx;
  self.imu_msg_.angular_velocity.y = gy;
  self.imu_msg_.angular_velocity.z = gz;
  self.imu_msg_.linear_acceleration.x = ax; // รวม g ไว้ ให้ EKF ถอดทีหลัง
  self.imu_msg_.linear_acceleration.y = ay;
  self.imu_msg_.linear_acceleration.z = az;

  rcl_publish(&self.imu_pub_, &self.imu_msg_, NULL);
}

bool ImuPublisher::init(const Params& p,
                        rcl_node_t* node,
                        rclc_support_t* support,
                        rclc_executor_t* executor){
  params_ = p;
  g_self = this;

  Wire.begin(params_.sda, params_.scl);
  Wire.setClock(400000);

  imu_ready_ = imu_setup_and_calibrate_();

  // Publisher
  rclc_publisher_init_default(
    &imu_pub_, node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data");

  // msg frame_id (จัดการ capacity/size)
  imu_msg_.header.frame_id.data = (char*)params_.frame_id;
  imu_msg_.header.frame_id.size = strlen(params_.frame_id);
  imu_msg_.header.frame_id.capacity = imu_msg_.header.frame_id.size + 1;

  // covariance คร่าวๆ (จูนได้)
  float ori_cov_rp = sq(2.0f * DEG_TO_RAD);
  float ori_cov_y  = sq(6.0f * DEG_TO_RAD);
  float ang_cov    = sq(2.5f * DEG_TO_RAD);
  float lin_cov    = sq(0.4f);
  set_cov_diag(imu_msg_.orientation_covariance, ori_cov_rp, ori_cov_rp, ori_cov_y);
  set_cov_diag(imu_msg_.angular_velocity_covariance, ang_cov, ang_cov, ang_cov);
  set_cov_diag(imu_msg_.linear_acceleration_covariance, lin_cov, lin_cov, lin_cov);

  // Timer @ loop_hz
  rclc_timer_init_default(
    &timer_, support,
    RCL_MS_TO_NS((int)(1000.0f/params_.loop_hz)),
    timer_cb_);

  rclc_executor_add_timer(executor, &timer_);
  return imu_ready_;
}

void ImuPublisher::spinOnce(){
  // ไม่มีอะไรเพิ่ม—ใช้ executor เรียก timer_cb_
}
