#ifdef Node1
/*************************************************************
 * main.cpp  (ESP32 / Arduino / PlatformIO)
 * Integrates: QuadEncoderReader + ImuPublisher + BodyPID + motorDrive
 * - micro-ROS subscriber: /drive_cmd (std_msgs/Float32MultiArray [v,w])
 * - Deterministic loop @ CONTROL_HZ
 *************************************************************/

#include <Arduino.h>
#include "config.h"               // uses your macros for pins/rates/frames

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "encoder_read.h"
#include "imu_node.h"
#include "motorDrive.h"
#include "pid.h"

// =================== Rates / gains (override-safe) ===================
#ifndef CONTROL_HZ
#define CONTROL_HZ 200.0f           // default if not set in config.h
#endif

// If you prefer to rely on pid.h BODY_* defaults, comment these two lines.
static constexpr float KP_V = 0.6f, KI_V = 12.0f, KD_V = 0.0f;
static constexpr float KP_W = 0.6f, KI_W = 12.0f, KD_W = 0.0f;
static constexpr float I_V_ABS = 200.0f, I_W_ABS = 200.0f;

// =================== Modules ===================
QuadEncoderReader enc;           // pins/filters come from config.h
ImuPublisher      imu;
BodyPID           body_pid(&enc);

// =================== micro-ROS ===================
rcl_allocator_t    g_alloc;
rclc_support_t     g_support;
rcl_node_t         g_node;
rcl_subscription_t g_sub;
rclc_executor_t    g_exec;

// Pre-allocated sub message buffer
static std_msgs__msg__Float32MultiArray g_sub_msg;
static float g_sub_buf[4];       // we use [0]=v, [1]=w

// Minimal command state (written in callback)
static volatile float    g_cmd_v = 0.0f;
static volatile float    g_cmd_w = 0.0f;
static volatile uint32_t g_cmd_stamp_ms = 0;

// =================== Subscriber callback ===================
static void drive_cmd_cb(const void* msgin)
{
  const auto* m = (const std_msgs__msg__Float32MultiArray*)msgin;
  if (m->data.data && m->data.size >= 2) {
    g_cmd_v = m->data.data[0];     // linear m/s
    g_cmd_w = m->data.data[1];     // yaw rad/s
    g_cmd_stamp_ms = millis();
  }
}

// =================== Setup ===================
void setup()
{
  Serial.begin(115200);
  delay(30);

  // micro-ROS over USB Serial (change to Serial1 if you want)
  set_microros_serial_transports(Serial);

  // --- Init micro-ROS objects ---
  g_alloc = rcl_get_default_allocator();
  rclc_support_init(&g_support, 0, NULL, &g_alloc);
  rclc_node_init_default(&g_node, "esp32_body_node", "", &g_support);

  rclc_subscription_init_default(
      &g_sub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/drive_cmd");

  rclc_executor_init(&g_exec, &g_support.context, 1, &g_alloc);
  // Pre-allocate subscriber msg storage (no heap in callback)
  g_sub_msg.data.data = g_sub_buf;
  g_sub_msg.data.capacity = (uint32_t)(sizeof(g_sub_buf)/sizeof(g_sub_buf[0]));
  g_sub_msg.data.size = 0;
  rclc_executor_add_subscription(&g_exec, &g_sub, &g_sub_msg, &drive_cmd_cb, ON_NEW_DATA);

  // --- Init hardware modules ---
  enc.begin(true);                 // internal pullups for encoders (per config.h)
  motorDrive_begin();              // sets up PWM channels & outputs
  {
    ImuPublisher::Params ip;       // uses defaults from config.h (pins/rates/frame)
    imu.init(ip, &g_node, &g_support, &g_exec);   // also creates a ROS timer for IMU publish
  }

  // --- PID wiring ---
  body_pid.reset();
  body_pid.setGainsLinear(KP_V, KI_V, KD_V);
  body_pid.setGainsAngular(KP_W, KI_W, KD_W);
  body_pid.setIClamp(I_V_ABS, I_W_ABS);
  // If you prefer the BODY_* limits from config.h (pid.h), omit setLimits().
  // body_pid.setLimits(BODY_V_MAX, BODY_W_MAX, BODY_AV_MAX, BODY_AW_MAX);

  g_cmd_stamp_ms = millis();
  Serial.println("[setup] ESP32 body control ready.");
}

// =================== Loop ===================
void loop()
{
  // Keep ROS responsive without blocking
  rclc_executor_spin_some(&g_exec, 1000);   // 1 ms budget

  // Deterministic control period
  static uint32_t last_us = micros();
  const uint32_t period_us = (uint32_t)(1000000.0f / CONTROL_HZ);
  const uint32_t now_us = micros();
  const uint32_t dt_us = now_us - last_us;
  if (dt_us < period_us) {
    delayMicroseconds(50);
    return;
  }
  last_us = now_us;
  const float dt = dt_us * 1e-6f;

  // 1) Encoder integration (velocity estimation & filters per config.h)
  enc.update();

  // 2) Command freshness (simple watchdog)
  const uint32_t now_ms = millis();
  const bool fresh = (now_ms - g_cmd_stamp_ms) < 300;  // 300 ms timeout
  const float v_ref = fresh ? g_cmd_v : 0.0f;
  const float w_ref = fresh ? g_cmd_w : 0.0f;

  // 3) Run body-level PID (uses enc -> bodyTwist, IMU yaw rate)
  body_pid.step(v_ref, w_ref, imu.getYawRateRad(), dt);

  // 4) Drive motors (slew limiting & PWM write)
  motorDrive_update();

  // 5) Low-rate debug (safe to enable at small PRINT_HZ)
  #ifdef MOTOR_CTRL_PRINT_HZ
  static uint32_t div = 0;
  const uint32_t every = (uint32_t)(CONTROL_HZ / (float)MOTOR_CTRL_PRINT_HZ);
  if (++div >= (every ? every : 1U)) {
    div = 0;
    float v_enc=0, w_enc=0;
    enc.bodyTwistFromWheels(v_enc, w_enc);
    Serial.printf("v_ref=%.3f w_ref=%.3f | v=%.3f w=%.3f | gz=%.3f\n",
                  v_ref, w_ref, v_enc, w_enc, imu.getYawRateRad());
  }
  #endif
}
#endif