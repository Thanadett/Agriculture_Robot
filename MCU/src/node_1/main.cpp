#ifdef Node1
#include <Arduino.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS msgs
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <rosidl_runtime_c/string_functions.h>

// โมดูลเรา
#include "config.h"
#include "encoder_read.h"
#include "imu_node.h"
#include "motorDrive.h"
#if USE_BODY_PID_ESP32
#include "pid.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------- micro-ROS objects ----------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t odom_pub;
rcl_subscription_t cmdvel_sub;
rcl_subscription_t estop_sub;
rcl_timer_t control_timer;
rcl_timer_t odom_timer;

geometry_msgs__msg__Twist cmdvel_msg;
std_msgs__msg__Bool estop_msg;
nav_msgs__msg__Odometry odom_msg;

// --------- modules ----------
QuadEncoderReader enc;
ImuPublisher imu;

#if USE_BODY_PID_ESP32
BodyPID body(&enc);
#endif

// --------- states ----------
static volatile float v_cmd = 0.0f, w_cmd = 0.0f;
static float x_ = 0.0f, y_ = 0.0f, yaw_ = 0.0f;
static float v_last = 0.0f, w_last = 0.0f;
extern bool estop; // จาก motorDrive.h

static inline int64_t now_nanos() { return (int64_t)micros() * 1000LL; }

// ----- Serial commands (optional) -----
#if CMD_SERIAL_ENABLE
static bool startsWith_(const String &s, const char *p)
{
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}
static bool parseFloatAfterEquals_(const String &s, const char *key, float &out)
{
  int idx = s.indexOf(key);
  if (idx < 0)
    return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end]))
    end++;
  out = s.substring(idx, end).toFloat();
  return true;
}
static void handleLine_(String line)
{
  line.trim();
  if (line.isEmpty())
    return;
  if (startsWith_(line, "ESTOP"))
  {
    int sp = line.indexOf(' '), val = 1;
    if (sp >= 0)
    {
      String tail = line.substring(sp + 1);
      tail.trim();
      val = tail.toInt();
    }
    estop = (val != 0);
    return;
  }
  if (startsWith_(line, "VW"))
  {
    float V = 0, W = 0;
    bool okV = parseFloatAfterEquals_(line, "V=", V);
    bool okW = parseFloatAfterEquals_(line, "W=", W);
    if (okV && okW)
    {
      v_cmd = V;
      w_cmd = W;
    }
    return;
  }
}
static void serial_poll_()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n')
    {
      if (rx_line.length() > 0)
      {
        handleLine_(rx_line);
        rx_line = "";
      }
    }
    else
    {
      if (rx_line.length() < 240)
        rx_line += c;
    }
  }
}
#endif

// --------- ROS callbacks ----------
static void cmdvel_cb(const void *msgin)
{
  auto msg = (const geometry_msgs__msg__Twist *)msgin;
  v_cmd = (float)msg->linear.x;
  w_cmd = (float)msg->angular.z;
}
static void estop_cb(const void *msgin)
{
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  estop = m->data;
}

// --------- timers ----------
static void control_timer_cb(rcl_timer_t *, int64_t)
{
  const float dt = 1.0f / CONTROL_HZ;

  enc.update();

#if USE_BODY_PID_ESP32
  const float gz_imu = 0.0f; // ถ้าจะใช้ IMU yaw-rate: setImuWeight>0 แล้วอ่านจาก imu.getYawRateRad()
  body.step(v_cmd, w_cmd, gz_imu, dt);
#else
  cmdVW_to_targets(v_cmd, w_cmd);
#endif

  motorDrive_update();

  enc.bodyTwistFromWheels(v_last, w_last);

  x_ += v_last * cosf(yaw_) * dt;
  y_ += v_last * sinf(yaw_) * dt;
  yaw_ += w_last * dt;
  if (yaw_ > M_PI)
    yaw_ -= 2.0f * M_PI;
  if (yaw_ < -M_PI)
    yaw_ += 2.0f * M_PI;
}

static void odom_timer_cb(rcl_timer_t *, int64_t)
{
  const int64_t t = now_nanos();
  odom_msg.header.stamp.sec = (int32_t)(t / 1000000000LL);
  odom_msg.header.stamp.nanosec = (uint32_t)(t % 1000000000LL);

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0f;
  const float cy = cosf(yaw_ * 0.5f), sy = sinf(yaw_ * 0.5f);
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = sy;
  odom_msg.pose.pose.orientation.w = cy;

  odom_msg.twist.twist.linear.x = v_last;
  odom_msg.twist.twist.angular.z = w_last;

  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// --------- setup / loop ----------
void setup()
{
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // motorDrive เดิม (ปรับให้มัน include "config.h")
  motorDrive_begin();

  // Encoder
  enc.setQuadMode(ENCODER_QUAD_MODE_FULL ? QUAD_FULL : QUAD_HALF);
  enc.setWheelRadius(WHEEL_RADIUS_M);
  enc.setTrack(TRACK_M);
  enc.setPPR(ENCODER_PPR_OUTPUT_DEFAULT);
  enc.setInvert(ENC_INV_FL == -1, ENC_INV_FR == -1, ENC_INV_RL == -1, ENC_INV_RR == -1);
  if (VEL_FILTER_MODE == 1)
    enc.setVelFilterEMA(EMA_ALPHA);
  else if (VEL_FILTER_MODE == 2)
    enc.setVelFilterButter2(BW2_FC_HZ, BW2_FS_HZ);
  enc.begin(true);

  // micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot_node", "", &support);

  rclc_publisher_init_default(
      &odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "wheel/odom");

  // frame IDs ตั้งครั้งเดียว
  odom_msg.header.frame_id = (rosidl_runtime_c__String){0};
  odom_msg.child_frame_id = (rosidl_runtime_c__String){0};
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, ODOM_FRAME);
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, BASE_FRAME);

  // covariance ตั้งคร่าวๆ
  for (int i = 0; i < 36; i++)
  {
    odom_msg.pose.covariance[i] = 0;
    odom_msg.twist.covariance[i] = 0;
  }
  odom_msg.pose.covariance[0] = 0.05f * 0.05f;                                     // x
  odom_msg.pose.covariance[7] = 0.05f * 0.05f;                                     // y
  odom_msg.pose.covariance[35] = (2.0f * M_PI / 180.0f) * (2.0f * M_PI / 180.0f);  // yaw
  odom_msg.twist.covariance[0] = 0.10f * 0.10f;                                    // vx
  odom_msg.twist.covariance[35] = (3.0f * M_PI / 180.0f) * (3.0f * M_PI / 180.0f); // wz

  // subs
  rclc_subscription_init_default(
      &cmdvel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel");
  rclc_subscription_init_default(
      &estop_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "estop");

  // IMU
  ImuPublisher::Params ip;
  ip.sda = I2C_SDA_PIN;
  ip.scl = I2C_SCL_PIN;
  ip.loop_hz = IMU_HZ;
  ip.frame_id = IMU_FRAME;
  ip.cf_alpha = CF_ALPHA;
  ip.calib_samples = IMU_CALIB_SAMPLES;
  imu.init(ip, &node, &support, &executor);

#if USE_BODY_PID_ESP32
  body.setGainsLinear(BODY_KP_V, BODY_KI_V, BODY_KD_V);
  body.setGainsAngular(BODY_KP_W, BODY_KI_W, BODY_KD_W);
  body.setLimits(BODY_V_MAX, BODY_W_MAX, BODY_AV_MAX, BODY_AW_MAX);
  body.setImuWeight(BODY_IMU_WEIGHT);
  body.setIClamp(BODY_I_V_ABS, BODY_I_W_ABS);
#endif

  // timers
  rclc_timer_init_default(
      &control_timer, &support,
      RCL_MS_TO_NS((int)(1000.0f / CONTROL_HZ)),
      control_timer_cb);
  rclc_timer_init_default(
      &odom_timer, &support,
      RCL_MS_TO_NS((int)(1000.0f / ODOM_HZ)),
      odom_timer_cb);

  // executor
  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_timer(&executor, &control_timer);
  rclc_executor_add_timer(&executor, &odom_timer);
  rclc_executor_add_subscription(&executor, &cmdvel_sub, &cmdvel_msg, &cmdvel_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &estop_sub, &estop_msg, &estop_cb, ON_NEW_DATA);

  Serial.println(
#if USE_BODY_PID_ESP32
      "READY: Mode A (Body PID on ESP32) | CONTROL@200Hz, ODOM@50Hz | Serial: 'VW V=<m/s> W=<rad/s>'"
#else
      "READY: Mode B (Pass-through /cmd_vel) | CONTROL@200Hz, ODOM@50Hz | Serial: 'VW V=<m/s> W=<rad/s>'"
#endif
  );
}

void loop()
{
#if CMD_SERIAL_ENABLE
  serial_poll_();
#endif
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
}
<<<<<<< HEAD
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
=======
>>>>>>> 221dcac2a572fb90c4496d49932c8aed48063a51
