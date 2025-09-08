// lib/Encoder/encoder_ros_pub.cpp
#include <Arduino.h>
#include <micro_ros_platformio.h>   // ให้ได้ set_microros_serial_transports(Serial)
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "encoder_read.h"

// --- external encoder instance (defined in main.cpp) ---
extern DualEncoderReader enc;

// --- micro-ROS objects ---
static rcl_allocator_t      g_alloc;
static rclc_support_t       g_support;
static rcl_node_t           g_node;
static rcl_publisher_t      g_pub_js;
static rcl_publisher_t      g_pub_total;
static rcl_timer_t          g_timer;
static rclc_executor_t      g_exec;

// --- messages / buffers (static, no malloc in callbacks) ---
static sensor_msgs__msg__JointState     g_msg_js;
static std_msgs__msg__Float32MultiArray g_msg_total;

static double g_pos[2];
static double g_vel[2];
static float  g_total[2];

static void timer_cb(rcl_timer_t * timer, int64_t)
{
  (void)timer;

  // snapshot from encoder
  const float posR = enc.positionRearRad();
  const float posF = enc.positionFrontRad();
  const float velR = enc.velocityRearRad();
  const float velF = enc.velocityFrontRad();
  const float tR   = enc.totalDistanceRearM();
  const float tF   = enc.totalDistanceFrontM();

  // JointState: keep name[] empty to avoid rosidl string init on MCU
  g_msg_js.name.data = nullptr;
  g_msg_js.name.size = 0;
  g_msg_js.name.capacity = 0;

  g_pos[0] = (double)posR; g_pos[1] = (double)posF;
  g_vel[0] = (double)velR; g_vel[1] = (double)velF;

  g_msg_js.position.data = g_pos;
  g_msg_js.position.size = 2;
  g_msg_js.position.capacity = 2;

  g_msg_js.velocity.data = g_vel;
  g_msg_js.velocity.size = 2;
  g_msg_js.velocity.capacity = 2;

  g_msg_js.effort.data = nullptr;
  g_msg_js.effort.size = 0;
  g_msg_js.effort.capacity = 0;

  // publish and explicitly consume return value (silence -Wunused-result)
  volatile rcl_ret_t rc1 = rcl_publish(&g_pub_js, &g_msg_js, nullptr);
  (void)rc1;

  // totals
  g_total[0] = tR;
  g_total[1] = tF;

  g_msg_total.data.data = g_total;
  g_msg_total.data.size = 2;
  g_msg_total.data.capacity = 2;

  volatile rcl_ret_t rc2 = rcl_publish(&g_pub_total, &g_msg_total, nullptr);
  (void)rc2;
}

void enc_microros_begin_serial()
{
  // IMPORTANT: ต้องเรียก Serial.begin(115200) ใน main.cpp ก่อนฟังก์ชันนี้
  // ผูก micro-ROS กับพอร์ต Serial ปัจจุบัน
  set_microros_serial_transports(Serial);

  g_alloc = rcl_get_default_allocator();
  rclc_support_init(&g_support, 0, NULL, &g_alloc);

  rclc_node_init_default(&g_node, "esp32_encoder_node", "", &g_support);

  rclc_publisher_init_default(
    &g_pub_js, &g_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/enc/joint_states"
  );

  rclc_publisher_init_default(
    &g_pub_total, &g_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/enc/total"
  );

  const unsigned period_ms = 100; // 10 Hz
  rclc_timer_init_default(&g_timer, &g_support, RCL_MS_TO_NS(period_ms), timer_cb);

  rclc_executor_init(&g_exec, &g_support.context, 1, &g_alloc);
  rclc_executor_add_timer(&g_exec, &g_timer);
}

void enc_microros_spin_some()
{
  rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(2));
}
