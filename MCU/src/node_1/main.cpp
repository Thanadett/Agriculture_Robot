#ifdef Node1

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#include "config.h"
#include "encoder_read.h"
#include "imu_node.h"
#include "motor_driver.h"
#include "velocity_controller.h"
#include "pid_controller.h"
#include "logger.h"

// ================== Global Variables ==================
// ROS2 Infrastructure
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Publishers
rcl_publisher_t wheel_ticks_pub;
rcl_publisher_t yaw_deg_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;

// Subscribers
rcl_subscription_t cmd_vel_sub;

// Messages
std_msgs__msg__Int32MultiArray wheel_ticks_msg;
std_msgs__msg__Float32 yaw_deg_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// Control System Components
EncoderReader encoder_reader;
IMU imu_node;
MotorDriver motor_driver;
VelocityController velocity_controller;
PID pid_v(BODY_KP_V, BODY_KI_V, BODY_KD_V, BODY_I_V_ABS);
PID pid_w(BODY_KP_W, BODY_KI_W, BODY_KD_W, BODY_I_W_ABS);
Logger logger;

// Control Variables
float target_v = 0.0f; // Linear velocity (m/s)
float target_w = 0.0f; // Angular velocity (rad/s)
float current_v = 0.0f;
float current_w = 0.0f;

// Odometry
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

// Timing
unsigned long last_control_time = 0;
unsigned long last_odom_time = 0;
unsigned long last_imu_time = 0;
unsigned long last_wheel_ticks_time = 0;

// ================== ROS2 Callback Functions ==================
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Apply velocity limits
  target_v = constrain(msg->linear.x, -BODY_V_MAX, BODY_V_MAX);
  target_w = constrain(msg->angular.z, -BODY_W_MAX, BODY_W_MAX);

  logger.info("CMD_VEL: v=%.3f w=%.3f", target_v, target_w);
}

// ================== ROS2 Setup Functions ==================
bool setup_ros2()
{
  // Initialize micro-ROS
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();

  // Initialize support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
  {
    logger.error("Failed to initialize support");
    return false;
  }

  // Create node
  if (rclc_node_init_default(&node, "esp32_node1", "", &support) != RCL_RET_OK)
  {
    logger.error("Failed to initialize node");
    return false;
  }

  // Create publishers
  if (rclc_publisher_init_default(&wheel_ticks_pub, &node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/wheel_ticks") != RCL_RET_OK)
  {
    logger.error("Failed to create wheel_ticks publisher");
    return false;
  }

  if (rclc_publisher_init_default(&yaw_deg_pub, &node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/yaw_deg") != RCL_RET_OK)
  {
    logger.error("Failed to create yaw_deg publisher");
    return false;
  }

  if (rclc_publisher_init_default(&odom_pub, &node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom") != RCL_RET_OK)
  {
    logger.error("Failed to create odom publisher");
    return false;
  }

  if (rclc_publisher_init_default(&imu_pub, &node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu") != RCL_RET_OK)
  {
    logger.error("Failed to create imu publisher");
    return false;
  }

  // Create subscriber
  if (rclc_subscription_init_default(&cmd_vel_sub, &node,
                                     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK)
  {
    logger.error("Failed to create cmd_vel subscription");
    return false;
  }

  // Initialize executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
  {
    logger.error("Failed to initialize executor");
    return false;
  }

  // Add subscription to executor
  if (rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                     &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK)
  {
    logger.error("Failed to add subscription to executor");
    return false;
  }

  // Initialize messages
  wheel_ticks_msg.data.capacity = 4;
  wheel_ticks_msg.data.size = 4;
  wheel_ticks_msg.data.data = (int32_t *)malloc(4 * sizeof(int32_t));

  logger.info("ROS2 setup completed successfully");
  return true;
}

// ================== Control Loop Functions ==================
void update_odometry()
{
  static float prev_left_pos = 0.0f;
  static float prev_right_pos = 0.0f;

  // Get wheel positions (average of front and rear)
  float left_pos = (encoder_reader.getPositionRad(0) + encoder_reader.getPositionRad(1)) / 2.0f;
  float right_pos = (encoder_reader.getPositionRad(2) + encoder_reader.getPositionRad(3)) / 2.0f;

  // Calculate delta positions
  float dl = (left_pos - prev_left_pos) * WHEEL_RADIUS_M;
  float dr = (right_pos - prev_right_pos) * WHEEL_RADIUS_M;

  // Update previous positions
  prev_left_pos = left_pos;
  prev_right_pos = right_pos;

  // Calculate velocities and update odometry
  float dc = (dl + dr) / 2.0f;              // Center distance
  float dtheta = (dr - dl) / TRACK_WIDTH_M; // Change in angle

  current_v = dc / CONTROL_DT;
  current_w = dtheta / CONTROL_DT;

  // Update pose
  odom_x += dc * cos(odom_theta + dtheta / 2.0f);
  odom_y += dc * sin(odom_theta + dtheta / 2.0f);
  odom_theta += dtheta;

  // Normalize theta
  while (odom_theta > M_PI)
    odom_theta -= 2.0f * M_PI;
  while (odom_theta < -M_PI)
    odom_theta += 2.0f * M_PI;
}

void control_loop()
{
  unsigned long current_time = millis();

  if (current_time - last_control_time >= (unsigned long)(1000.0f / CONTROL_HZ))
  {
    // Update encoder readings
    encoder_reader.update();

    // Update odometry
    update_odometry();

    // Calculate control errors
    float error_v = target_v - current_v;
    float error_w = target_w - current_w;

    // PID control
    float cmd_v = pid_v.update(error_v, CONTROL_DT);
    float cmd_w = pid_w.update(error_w, CONTROL_DT);

    // Convert to wheel velocities
    float left_vel = cmd_v - (cmd_w * TRACK_WIDTH_M / 2.0f);
    float right_vel = cmd_v + (cmd_w * TRACK_WIDTH_M / 2.0f);

    // Update velocity controller
    velocity_controller.setTargetVelocity(0, left_vel / WHEEL_RADIUS_M);  // FL
    velocity_controller.setTargetVelocity(1, left_vel / WHEEL_RADIUS_M);  // RL
    velocity_controller.setTargetVelocity(2, right_vel / WHEEL_RADIUS_M); // FR
    velocity_controller.setTargetVelocity(3, right_vel / WHEEL_RADIUS_M); // RR

    velocity_controller.update(CONTROL_DT);

    last_control_time = current_time;
  }
}

void publish_wheel_ticks()
{
  unsigned long current_time = millis();

  if (current_time - last_wheel_ticks_time >= (unsigned long)(1000.0f / ODOM_HZ))
  {
    // Update tick counts
    wheel_ticks_msg.data.data[0] = encoder_reader.getTicks(0); // FL
    wheel_ticks_msg.data.data[1] = encoder_reader.getTicks(1); // RL
    wheel_ticks_msg.data.data[2] = encoder_reader.getTicks(2); // FR
    wheel_ticks_msg.data.data[3] = encoder_reader.getTicks(3); // RR

    // Publish
    rcl_ret_t ret = rcl_publish(&wheel_ticks_pub, &wheel_ticks_msg, NULL);
    if (ret != RCL_RET_OK)
    {
      logger.error("Failed to publish wheel_ticks");
    }

    last_wheel_ticks_time = current_time;
  }
}

void publish_yaw_deg()
{
  unsigned long current_time = millis();

  if (current_time - last_imu_time >= (unsigned long)(1000.0f / IMU_HZ))
  {
    imu_node.update(CONTROL_DT);
    yaw_deg_msg.data = imu_node.getYawDeg();
    rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (rc != RCL_RET_OK)
    {
      logger.error("Failed to publish imu (%d)", (int)rc);
    }

    // Also publish full IMU data
    imu_node.fillIMUMessage(imu_msg);
    rcl_publish(&imu_pub, &imu_msg, NULL);

    last_imu_time = current_time;
  }
}

void publish_odometry()
{
  unsigned long current_time = millis();

  if (current_time - last_odom_time >= (unsigned long)(1000.0f / ODOM_HZ))
  {
    // Fill odometry message
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    float cy = cos(odom_theta * 0.5f);
    float sy = sin(odom_theta * 0.5f);

    odom_msg.pose.pose.orientation.w = cy;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;

    odom_msg.twist.twist.linear.x = current_v;
    odom_msg.twist.twist.angular.z = current_w;

    // Set frame IDs
    strcpy(odom_msg.header.frame_id.data, "odom");
    strcpy(odom_msg.child_frame_id.data, "base_link");
    odom_msg.header.stamp.sec = current_time / 1000;
    odom_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;

    rcl_ret_t ret = rcl_publish(&odom_pub, &odom_msg, NULL);
    if (ret != RCL_RET_OK)
    {
      logger.error("Failed to publish odometry");
    }

    last_odom_time = current_time;
  }
}

// ================== Main Setup and Loop ==================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  logger.info("Initializing ESP32 Node1...");

  // Initialize hardware components
  if (!encoder_reader.init())
  {
    logger.error("Failed to initialize encoder reader");
    return;
  }

  if (!imu_node.init())
  {
    logger.error("Failed to initialize IMU");
    return;
  }

  if (!motor_driver.init())
  {
    logger.error("Failed to initialize motor driver");
    return;
  }

  if (!velocity_controller.init(&encoder_reader, &motor_driver))
  {
    logger.error("Failed to initialize velocity controller");
    return;
  }

  // Setup ROS2
  delay(2000); // Wait for micro-ROS agent
  if (!setup_ros2())
  {
    logger.error("Failed to setup ROS2");
    return;
  }

  logger.info("ESP32 Node1 initialization complete");
}

void loop()
{
  // Spin ROS2 executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  // Run control loop
  control_loop();

  // Publish topics
  publish_wheel_ticks();
  publish_yaw_deg();
  publish_odometry();

  // Small delay to prevent watchdog issues
  delay(1);
}

#endif // Node1