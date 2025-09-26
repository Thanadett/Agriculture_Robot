#ifdef Node1

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"
#include "motor_driver.h"
#include "encoder_read.h"

// ============================ Safety Macros ============================
#define RCCHECK(fn)                                                              \
    {                                                                            \
        rcl_ret_t rc_ = (fn);                                                    \
        if (rc_ != RCL_RET_OK)                                                   \
        {                                                                        \
            Serial.printf("[RCL] Error %d at %s:%d\n", rc_, __FILE__, __LINE__); \
            rclErrorLoop();                                                      \
        }                                                                        \
    }
#define RCSOFTCHECK(fn)        \
    {                          \
        rcl_ret_t rc_ = (fn);  \
        if (rc_ != RCL_RET_OK) \
        {                      \
            /* keep running */ \
        }                      \
    }

// Helper for periodic execution using uxr_millis()
#define EXECUTE_EVERY_N_MS(MS, X)     \
    do                                \
    {                                 \
        static uint32_t _ts = 0;      \
        uint32_t _now = uxr_millis(); \
        if (_now - _ts >= (MS))       \
        {                             \
            {                         \
                X;                    \
            }                         \
            _ts = _now;               \
        }                             \
    } while (0)

// ============================ micro-ROS State ==========================
enum AgentState
{
    WAITING_AGENT = 0,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
static AgentState g_state = WAITING_AGENT;

// Core objects
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer_ctrl;

// Pub/Sub
static rcl_publisher_t pub_ticks;      // /wheel_ticks
static rcl_publisher_t pub_heartbeat;  // /robot_heartbeat
static rcl_subscription_t sub_cmd_vel; // /cmd_vel

// Messages (pre-allocated)
static std_msgs__msg__Int32MultiArray msg_ticks;
static std_msgs__msg__String msg_hb;
static geometry_msgs__msg__Twist msg_cmd_vel;

// Time sync with agent
static unsigned long long time_offset_ms = 0;
static uint32_t last_time_sync_ms = 0;

// ============================ Robot Modules ============================
static QuadEncoderReader enc;
extern uint32_t last_cmd_ms; // from motor_drive.cpp

// ============================ Forward Decls ============================
static void rclErrorLoop();
static bool createEntities();
static bool destroyEntities();
static void syncTime();

static void on_cmd_vel(const void *msgin);
static void on_timer(rcl_timer_t *timer, int64_t last_call_time);

// ============================ Setup ============================
void setup()
{
    Serial.begin(115200);
    delay(100);

    // --------- Transport selection ---------
    // Default: Serial transport
    // If you want Wi-Fi: define MICROROS_WIFI and set credentials in your build flags
#ifdef MICROROS_WIFI
// Example:
// IPAddress agent_ip(192,168,1,50);
// set_microros_wifi_transports("SSID", "PASSWORD", agent_ip, 8888);
#error "Define your Wi-Fi transports here or remove MICROROS_WIFI"
#else
    set_microros_serial_transports(Serial);
#endif

    // --------- Hardware init ---------
    motorDrive_begin(); // LEDC channels + outputs to 0
    enc.begin(true);    // enable internal weak pullups
    // Map encoder inversion from macros (+1/-1) to bools (true = invert)
    enc.setInvert(ENC_INV_FL < 0, ENC_INV_FR < 0, ENC_INV_RL < 0, ENC_INV_RR < 0);
    enc.setWheelRadius(ENC_WHEEL_RADIUS);
    enc.setPPR(ENCODER_PPR_OUTPUT_DEFAULT); // 11 * 270 * 2 = 5940 (ตาม config.h)

    // First sync right away
    syncTime();
    last_time_sync_ms = millis();
}

// ============================ Main loop ============================
void loop()
{
    // Periodic time sync with agent (10 s)
    if (millis() - last_time_sync_ms > 10000U)
    {
        syncTime();
        last_time_sync_ms = millis();
    }

    // Basic micro-ROS agent state machine
    switch (g_state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, {
            g_state = (RMW_RET_OK == rmw_uros_ping_agent(500, 4)) ? AGENT_AVAILABLE : WAITING_AGENT;
        });
        break;

    case AGENT_AVAILABLE:
        if (createEntities())
        {
            g_state = AGENT_CONNECTED;
        }
        else
        {
            destroyEntities();
            g_state = WAITING_AGENT;
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(300, {
            if (RMW_RET_OK != rmw_uros_ping_agent(300, 3))
            {
                g_state = AGENT_DISCONNECTED;
            }
        });
        if (g_state == AGENT_CONNECTED)
        {
            // run executor non-blocking
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
        }
        break;

    case AGENT_DISCONNECTED:
        // Safety: stop motors
        cmdVW_to_targets(0.f, 0.f);
        motorDrive_update(); // write zero with slew
        destroyEntities();
        g_state = WAITING_AGENT;
        break;
    }

    // Local serial command helper (VW / P / PW4 / ESTOP)
    motorDrive_handleSerialOnce();
}

// ============================ Callbacks ============================

// /cmd_vel (Twist): linear.x = V [m/s], angular.z = W [rad/s]
static void on_cmd_vel(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
    float V = (float)m->linear.x;
    float W = (float)m->angular.z;

    // Update target wheels via kinematics mapper
    cmdVW_to_targets(V, W);
    last_cmd_ms = millis(); // refresh watchdog timestamp
}

// Timer @ ~20ms: update encoders + motors + publish /wheel_ticks
static void on_timer(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/)
{
    // 1) Update encoder snapshot
    enc.update();

    // 2) Drive outputs (watchdog/decay handled inside motorDrive_update)
    motorDrive_update();

    // 3) Publish /wheel_ticks as [FL, FR, RL, RR] (int32)
    if (msg_ticks.data.size >= 4)
    {
        msg_ticks.data.data[0] = (int32_t)enc.counts(W_FL);
        msg_ticks.data.data[1] = (int32_t)enc.counts(W_FR);
        msg_ticks.data.data[2] = (int32_t)enc.counts(W_RL);
        msg_ticks.data.data[3] = (int32_t)enc.counts(W_RR);
        RCSOFTCHECK(rcl_publish(&pub_ticks, &msg_ticks, NULL));
    }

    // 4) Lightweight heartbeat every ~200 ms
    static uint32_t hb_ts = 0;
    uint32_t now = millis();
    if (now - hb_ts >= 200U)
    {
        const char *hb = "OK";
        msg_hb.data.data = (char *)hb;
        msg_hb.data.size = strlen(hb);
        msg_hb.data.capacity = msg_hb.data.size + 1;
        RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_hb, NULL));
        hb_ts = now;
    }
}

// ============================ micro-ROS create/destroy ============================
static bool createEntities()
{
    allocator = rcl_get_default_allocator();

    // Init with Domain ID = 10 (match your ROS2 graph)
    rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_opts, allocator) != RCL_RET_OK)
        return false;
    (void)rcl_init_options_set_domain_id(&init_opts, 69);

    if (rclc_support_init_with_options(&support, 0, NULL, &init_opts, &allocator) != RCL_RET_OK)
    {
        rcl_init_options_fini(&init_opts);
        return false;
    }
    rcl_init_options_fini(&init_opts);

    // Node
    RCCHECK(rclc_node_init_default(&node, "esp32_base", "", &support));

    // Publisher: /wheel_ticks
    RCCHECK(rclc_publisher_init_default(
        &pub_ticks,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/wheel_ticks"));
    // Allocate Int32MultiArray data (size=4)
    rosidl_runtime_c__int32__Sequence__init(&msg_ticks.data, 4);
    msg_ticks.data.data[0] = 0;
    msg_ticks.data.data[1] = 0;
    msg_ticks.data.data[2] = 0;
    msg_ticks.data.data[3] = 0;

    // Publisher: /robot_heartbeat (String)
    RCCHECK(rclc_publisher_init_default(
        &pub_heartbeat,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/robot_heartbeat"));
    // Pre-assign buffer (we point to const "OK" at publish time)

    // Subscriber: /cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &sub_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // Timer: 20 ms control
    const uint32_t CTRL_MS = 20;
    RCCHECK(rclc_timer_init_default(
        &timer_ctrl, &support, RCL_MS_TO_NS(CTRL_MS), on_timer));

    // Executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_ctrl));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &sub_cmd_vel, &msg_cmd_vel, on_cmd_vel, ON_NEW_DATA));

    // Initial time sync
    syncTime();

    Serial.println("[INIT] micro-ROS entities created");
    return true;
}

static bool destroyEntities()
{
    // Finish entities in reverse order
    rcl_subscription_fini(&sub_cmd_vel, &node);
    rcl_publisher_fini(&pub_heartbeat, &node);
    rcl_publisher_fini(&pub_ticks, &node);

    // Free dynamic sequences
    if (msg_ticks.data.data)
    {
        rosidl_runtime_c__int32__Sequence__fini(&msg_ticks.data);
    }

    rcl_timer_fini(&timer_ctrl);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    Serial.println("[CLEAN] micro-ROS entities destroyed");
    return true;
}

// ============================ Time sync helpers ============================
static void syncTime()
{
    // Try to sync; do not hard-fail
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long now_ms = millis();
    unsigned long long agent_ms = rmw_uros_epoch_millis(); // epoch from agent
    time_offset_ms = (agent_ms > 0ULL) ? (agent_ms - now_ms) : 0ULL;
}

static void rclErrorLoop()
{
    // Hard reset on fatal RCL errors (simple recovery on MCU)
    Serial.println("[RCL] Fatal error -> restart");
    delay(100);
    ESP.restart();
}

// ==========================================================================
// End of file
// ==========================================================================

#endif // Node1
