#ifdef Node1
#include <Arduino.h>

// ---- micro-ROS ----
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ---- ROS 2 msgs ----
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <rosidl_runtime_c/string_functions.h>

// ---- Robot modules ----
#include "encoder/encoder_read.h"   // full/half-quad + filter
#include "imu/imu_node.h"           // MPU6050 + complementary filter
#include "motorDrive.h"             // โค้ด PWM 4 ล้อของคุณ (VW/P/PW4/ESTOP)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ========================= พารามิเตอร์หลัก =========================
static constexpr float ODOM_HZ      = 50.0f;   // ความถี่ odom (Hz)
static constexpr float IMU_HZ       = 100.0f;  // ความถี่ IMU (Hz)
static constexpr char  ODOM_FRAME[] = "odom";
static constexpr char  BASE_FRAME[] = "base_link";

// ========================= micro-ROS objects =========================
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;

// pubs/subs/timers
rcl_publisher_t odom_pub;
rcl_subscription_t cmdvel_sub;
rcl_subscription_t estop_sub;
rcl_timer_t     odom_timer;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmdvel_msg;
std_msgs__msg__Bool estop_msg;

// ========================= โมดูลฮาร์ดแวร์ =========================
QuadEncoderReader enc;
ImuPublisher      imu;

// ========================= สถานะ odom ภายใน =========================
static float x_ = 0.0f, y_ = 0.0f, yaw_ = 0.0f;

// ========================= จาก motorDrive.h (ตัวแปร global) =========================
extern uint32_t last_cmd_ms;
extern bool     estop;

// ========================= ฟังก์ชันช่วย =========================
static inline int64_t now_nanos() {
  return (int64_t)micros() * 1000LL;
}

static inline void body_twist_from_wheels(const QuadEncoderReader& e, float& v, float& wz){
  e.bodyTwistFromWheels(v, wz);
}

// ========================= Callback: /cmd_vel =========================
void cmdvel_cb(const void* msgin){
  auto msg = (const geometry_msgs__msg__Twist*)msgin;
  float V = (float)msg->linear.x;     // m/s
  float W = (float)msg->angular.z;    // rad/s

  // สั่งผ่าน motorDrive (map V,W -> PWM 4 ล้อ)
  cmdVW_to_targets(V, W);
  last_cmd_ms = millis(); // ป้องกัน watchdog ภายใน motorDrive
}

// ========================= Callback: /estop =========================
void estop_cb(const void* msgin){
  const std_msgs__msg__Bool* m = (const std_msgs__msg__Bool*)msgin;
  estop = m->data;
}

// ========================= Timer: publish /wheel/odom =========================
void odom_timer_cb(rcl_timer_t* /*timer*/, int64_t /*last*/){
  enc.update();

  // คำนวณ v,w จากล้อ (ค่าที่ผ่านฟิลเตอร์แล้ว)
  float v, w;
  body_twist_from_wheels(enc, v, w);

  // บูรณาการตำแหน่งแบบ simple Euler (2D)
  const float dt = 1.0f / ODOM_HZ;
  x_   += v * cosf(yaw_) * dt;
  y_   += v * sinf(yaw_) * dt;
  yaw_ += w * dt;
  if (yaw_ >  M_PI) yaw_ -= 2.0f*M_PI;
  if (yaw_ < -M_PI) yaw_ += 2.0f*M_PI;

  // เตรียม Odometry msg
  const int64_t t = now_nanos();
  odom_msg.header.stamp.sec     = (int32_t)(t / 1000000000LL);
  odom_msg.header.stamp.nanosec = (uint32_t)(t % 1000000000LL);

  // pose
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0f;
  const float cy=cosf(yaw_*0.5f), sy=sinf(yaw_*0.5f);
  odom_msg.pose.pose.orientation.x=0;
  odom_msg.pose.pose.orientation.y=0;
  odom_msg.pose.pose.orientation.z=sy;
  odom_msg.pose.pose.orientation.w=cy;

  // twist
  odom_msg.twist.twist.linear.x  = v;
  odom_msg.twist.twist.angular.z = w;

  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// ========================= setup() =========================
void setup(){
  // ---- Serial & micro-ROS transport
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // ---- เริ่มระบบขับเคลื่อน 4 ล้อ (จาก motorDrive.h/.cpp เดิมของคุณ)
  motorDrive_begin();  // ปรับพิน/ทิศล้อใน motorDrive.h ให้ตรงฮาร์ดแวร์จริง

  // ---- Encoder
  enc.setQuadMode(QUAD_FULL); // ใช้ full-quad (ละเอียด)
  enc.setWheelRadius(0.0635f); // ปรับตามล้อจริง
  enc.setTrack(0.30f);         // ระยะฐานล้อซ้าย-ขวา (m)
  enc.setPPR(ENCODER_PPR_MOTOR * REDUCTION_RATIO * 4.0f); // 11*270*4=11880
  enc.setInvert(false, true, false, true); // ตัวอย่าง: ล้อขวากลับทิศ
  enc.setVelFilterButter2(6.0f, ODOM_HZ);  // ฟิลเตอร์ความเร็ว fc=6Hz @50Hz
  enc.begin(true);

  // ---- micro-ROS init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot_node", "", &support);

  // ---- Odometry publisher
  rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "wheel/odom");

  // ตั้งค่า frame IDs (ทำครั้งเดียว)
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id,  ODOM_FRAME);
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id,   BASE_FRAME);

  // ตั้ง covariance คร่าวๆ (ไปจูนภาคสนามได้)
  for (int i=0;i<36;i++){ odom_msg.pose.covariance[i]=0; odom_msg.twist.covariance[i]=0; }
  odom_msg.pose.covariance[0]  = 0.05f*0.05f;   // x
  odom_msg.pose.covariance[7]  = 0.05f*0.05f;   // y
  odom_msg.pose.covariance[35] = (2.0f * M_PI/180.0f)*(2.0f * M_PI/180.0f); // yaw
  odom_msg.twist.covariance[0] = 0.10f*0.10f;   // vx
  odom_msg.twist.covariance[35]= (3.0f * M_PI/180.0f)*(3.0f * M_PI/180.0f); // wz

  // ---- cmd_vel subscriber (จาก Pi/ROS2)
  rclc_subscription_init_default(
    &cmdvel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"  // standard
  );

  // ---- estop subscriber (optional)
  rclc_subscription_init_default(
    &estop_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "estop"
  );

  // ---- Odom timer
  rclc_timer_init_default(
    &odom_timer, &support,
    RCL_MS_TO_NS((int)(1000.0f/ODOM_HZ)),
    odom_timer_cb);

  // ---- IMU Publisher (Adafruit MPU6050 + complementary filter)
  ImuPublisher::Params ip;
  ip.sda=21; ip.scl=22; ip.loop_hz=IMU_HZ;
  ip.frame_id="imu_link"; ip.cf_alpha=0.96f; ip.calib_samples=800;
  imu.init(ip, &node, &support, &executor);

  // ---- Executor (timer + subs + imu timer)
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_subscription(&executor, &cmdvel_sub, &cmdvel_msg, &cmdvel_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &estop_sub,   &estop_msg,   &estop_cb,   ON_NEW_DATA);
}

// ========================= loop() =========================
void loop(){
  // อ่านคำสั่งจาก Serial (VW / P / PW4 / ESTOP) — เผื่อสั่งจากเทอร์มินัล
  motorDrive_handleSerialOnce();

  // อัปเดต watchdog + ramp + เขียน PWM ไปมอเตอร์
  motorDrive_update();

  // ประมวลผล micro-ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
#endif








// #ifdef Node1
// #include <Arduino.h>
// #include "motorDrive.h"


// void setup() {
//   Serial.begin(115200);
//   delay(200);

//   // เริ่มระบบขับเคลื่อน (คงโค้ด motorDrive เดิม)
//   motorDrive_begin();

// }

// void loop() {
//   // 1) รับคำสั่งจาก Serial (VW, P, PW4, ESTOP) และตั้งเป้าหมาย PWM
//   motorDrive_handleSerialOnce();

//   // 3) ทำ watchdog + ส่ง PWM ไปมอเตอร์
//   motorDrive_update();

//     // 4) ส่งฟีดแบ็กออก Serial เป็นช่วงๆ
//   static uint32_t last = 0;
//   const uint32_t FEEDBACK_INTERVAL_MS = 500;  // <- ปรับตรงนี้ได้ (เช่น 100, 200, 500)
//   if (millis() - last >= FEEDBACK_INTERVAL_MS) {
//     last = millis();
//   }


//   delay(2);
// }
// #endif