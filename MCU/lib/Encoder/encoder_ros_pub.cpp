#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "encoder_read.h"

// Global instance (defined in main.cpp)
extern QuadEncoderReader enc4;

static rcl_allocator_t      g_alloc;
static rclc_support_t       g_support;
static rcl_node_t           g_node;
static rcl_publisher_t      g_pub_js;
static rcl_publisher_t      g_pub_total;
static rcl_timer_t          g_timer;
static rclc_executor_t      g_exec;

static sensor_msgs__msg__JointState     g_msg_js;
static std_msgs__msg__Float32MultiArray g_msg_total;

static double g_pos[4];
static double g_vel[4];
static float  g_total[4];

static uint32_t pub_fail_js = 0, pub_fail_total = 0;

static void timer_cb(rcl_timer_t * timer, int64_t) {
  (void)timer;

  g_pos[W_FL] = (double)enc4.positionRad(W_FL);
  g_pos[W_FR] = (double)enc4.positionRad(W_FR);
  g_pos[W_RL] = (double)enc4.positionRad(W_RL);
  g_pos[W_RR] = (double)enc4.positionRad(W_RR);

  g_vel[W_FL] = (double)enc4.velocityRadPerSec(W_FL);
  g_vel[W_FR] = (double)enc4.velocityRadPerSec(W_FR);
  g_vel[W_RL] = (double)enc4.velocityRadPerSec(W_RL);
  g_vel[W_RR] = (double)enc4.velocityRadPerSec(W_RR);

  g_total[W_FL] = enc4.totalDistanceM(W_FL);
  g_total[W_FR] = enc4.totalDistanceM(W_FR);
  g_total[W_RL] = enc4.totalDistanceM(W_RL);
  g_total[W_RR] = enc4.totalDistanceM(W_RR);

  // JointState without names (avoid string allocations on MCU)
  g_msg_js.name.data = nullptr; g_msg_js.name.size = 0; g_msg_js.name.capacity = 0;

  g_msg_js.position.data = g_pos; g_msg_js.position.size = 4; g_msg_js.position.capacity = 4;
  g_msg_js.velocity.data = g_vel; g_msg_js.velocity.size = 4; g_msg_js.velocity.capacity = 4;
  g_msg_js.effort.data   = nullptr; g_msg_js.effort.size = 0; g_msg_js.effort.capacity = 0;

//   (void) rcl_publish(&g_pub_js, &g_msg_js, nullptr);

  g_msg_total.data.data = g_total; g_msg_total.data.size = 4; g_msg_total.data.capacity = 4;
//   (void) rcl_publish(&g_pub_total, &g_msg_total, nullptr);
  
  rcl_ret_t rc1 = rcl_publish(&g_pub_js, &g_msg_js, nullptr);
  if (rc1 != RCL_RET_OK) { ++pub_fail_js; }
 
  rcl_ret_t rc2 = rcl_publish(&g_pub_total, &g_msg_total, nullptr);
  if (rc2 != RCL_RET_OK) { ++pub_fail_total; }

}

void enc_microros_begin_serial() {
  // ต้องมี Serial.begin(...) ใน setup() ก่อนเรียกฟังก์ชันนี้
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

void enc_microros_spin_some() {
  rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(2));
}
