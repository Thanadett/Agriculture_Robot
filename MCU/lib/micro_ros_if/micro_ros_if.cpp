#include "micro_ros_if.h"
#include "config.h"

// Define YAW_PUB_HZ if not defined in config.h
#ifndef YAW_PUB_HZ
#define YAW_PUB_HZ 100
#endif

// Define WHEEL_PUB_HZ if not defined in config.h
#ifndef WHEEL_PUB_HZ
#define WHEEL_PUB_HZ 50
#endif

extern "C"
{
#include <rmw_microros/rmw_microros.h>
}

static MicroRosIF *g_self = nullptr;

bool MicroRosIF::init()
{
  g_self = this;

// ---- transport ----
#ifdef USE_UDP
  if (USE_UDP)
  {
    // Set via menuconfig or hardcode agent IP/port:
    // rmw_uros_set_custom_transport(...)
  }
  else
  {
    // Serial transport; set by platformio upload port, uros serial init handled by microros-arduino
  }
#else
  // Serial transport; set by platformio upload port, uros serial init handled by microros-arduino
#endif

  allocator_ = rcl_get_default_allocator();

  // support
  rclc_support_init(&support_, 0, NULL, &allocator_);

  // node
  rclc_node_init_default(&node_, "esp32_base", "", &support_);

  // QoS: cmd_vel reliable depth 10
  rcl_subscription_options_t sub_ops = rcl_subscription_get_default_options();
  sub_ops.qos = rmw_qos_profile_default;
  sub_ops.qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  sub_ops.qos.depth = 10;
  rclc_subscription_init(&sub_cmdvel_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                         "/cmd_vel", &sub_ops.qos);

  // Publishers best_effort depth 10
  rcl_publisher_options_t pub_ops = rcl_publisher_get_default_options();
  pub_ops.qos = rmw_qos_profile_sensor_data; // best effort
  pub_ops.qos.depth = 10;

  rclc_publisher_init(&pub_ticks_, &node_,
                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/wheel_ticks", &pub_ops.qos);
  rclc_publisher_init(&pub_yaw_, &node_,
                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/yaw_deg", &pub_ops.qos);

  // executor (single sub)
  rclc_executor_init(&exec_, &support_.context, 1, &allocator_);
  rclc_executor_add_subscription_with_context(&exec_, &sub_cmdvel_, &cmdvel_msg_, &MicroRosIF::cmdvel_cb, NULL, ON_NEW_DATA);

  // Init messages
  std_msgs__msg__Int32MultiArray__init(&ticks_msg_);
  ticks_msg_.data.capacity = 4;
  ticks_msg_.data.size = 4;
  ticks_msg_.data.data = (int32_t *)malloc(4 * sizeof(int32_t));
  memset(ticks_msg_.data.data, 0, 4 * sizeof(int32_t));

  return true;
}

void MicroRosIF::cmdvel_cb(const void *msgin, void * /*arg*/)
{
  auto *msg = (const geometry_msgs__msg__Twist *)msgin;
  // Saturation will be handled inside controller too
  if (g_self && g_self->vc_)
  {
    g_self->vc_->setCmdVel((float)msg->linear.x, (float)msg->angular.z);
  }
}

void MicroRosIF::spin_once(float now, float dt_control, float dt_pub_ticks, float dt_pub_yaw)
{
  // Run executor to pump /cmd_vel
  rclc_executor_spin_some(&exec_, RCL_MS_TO_NS(1));

  // Control loop ticked externally at CONTROL_HZ (main.cpp)

  // Publish ticks @50Hz
  static float t_last_ticks = 0;
  if (now - t_last_ticks >= (1.0f / WHEEL_PUB_HZ))
  {
    t_last_ticks = now;
    ticks_msg_.data.data[0] = enc_->readTicks(Wheel::FL);
    ticks_msg_.data.data[1] = enc_->readTicks(Wheel::RL);
    ticks_msg_.data.data[2] = enc_->readTicks(Wheel::FR);
    ticks_msg_.data.data[3] = enc_->readTicks(Wheel::RR);
    if (rcl_publish(&pub_ticks_, &ticks_msg_, NULL) != RCL_RET_OK)
      ;
  }

  // Publish yaw @100Hz
  static float t_last_yaw = 0;
  if (now - t_last_yaw >= (1.0f / YAW_PUB_HZ))
  {
    t_last_yaw = now;
    yaw_msg_.data = imu_->getYawDeg();
    if (rcl_publish(&pub_yaw_, &yaw_msg_, NULL) != RCL_RET_OK)
      ;
  }
}
