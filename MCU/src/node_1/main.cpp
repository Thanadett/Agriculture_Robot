#ifdef Node1

#include <Arduino.h>
#include <math.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"
#include "motor_driver.h"
#include "encoder_read.h"

// ===== IMU + KF + PID =====
#include "imu.h"
#include "kalman_yaw.h"
#include "pid_rate.h"

// ---------------- Build-time config ----------------
#ifndef ROS_DOMAIN_ID_MCU
#define ROS_DOMAIN_ID_MCU 69
#endif

#ifndef USE_INNER_PID
#define USE_INNER_PID 0
#endif

// ใช้ "relative topics" ตาม best-practice ของ micro-ROS
#define TOPIC_WHEEL_TICKS "wheel_ticks"
#define TOPIC_HEARTBEAT "robot_heartbeat"
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_YAW_RATE "yaw_rate"
#define TOPIC_YAW_KF "yaw_kf"
#define TOPIC_GYRO_BIAS "gyro_bias"

#define TOPIC_JOY_RESET "joy_reset"

// ============================ Conditional Debug Macros ============================
// ปิด debug print เมื่อเชื่อมต่อ Agent เพื่อไม่ให้ทับ Serial ของ micro-ROS
#define DEBUG_ENABLED (g_state != AGENT_CONNECTED)

#define DEBUG_PRINT(x)       \
    do                       \
    {                        \
        if (DEBUG_ENABLED)   \
            Serial.print(x); \
    } while (0)
#define DEBUG_PRINTLN(x)       \
    do                         \
    {                          \
        if (DEBUG_ENABLED)     \
            Serial.println(x); \
    } while (0)
#define DEBUG_PRINTF(...)               \
    do                                  \
    {                                   \
        if (DEBUG_ENABLED)              \
            Serial.printf(__VA_ARGS__); \
    } while (0)

// ============================ Safety Macros ============================
#define RCCHECK(fn)                                                             \
    {                                                                           \
        rcl_ret_t rc_ = (fn);                                                   \
        if (rc_ != RCL_RET_OK)                                                  \
        {                                                                       \
            DEBUG_PRINTF("[RCL] Error %d at %s:%d\n", rc_, __FILE__, __LINE__); \
            rclErrorLoop();                                                     \
        }                                                                       \
    }
#define RCSOFTCHECK(fn)                                                              \
    {                                                                                \
        rcl_ret_t rc_ = (fn);                                                        \
        if (rc_ != RCL_RET_OK)                                                       \
        {                                                                            \
            DEBUG_PRINTF("[RCL] Soft error %d at %s:%d\n", rc_, __FILE__, __LINE__); \
        }                                                                            \
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
static AgentState g_last_logged_state = (AgentState)(-1);

// Core objects
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_timer_t timer_ctrl;

// Pub/Sub
static rcl_publisher_t pub_ticks;
static rcl_publisher_t pub_heartbeat;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_joy_reset;
static std_msgs__msg__Bool msg_joy_reset;

// Debug pubs
static rcl_publisher_t pub_yaw_rate;
static rcl_publisher_t pub_yaw_kf;
static rcl_publisher_t pub_gyro_bias;

// Messages
static std_msgs__msg__Float32MultiArray msg_ticks;
static std_msgs__msg__String msg_hb;
static geometry_msgs__msg__Twist msg_cmd_vel;

static std_msgs__msg__Float32 msg_yaw_rate;
static std_msgs__msg__Float32 msg_yaw_kf;
static std_msgs__msg__Float32 msg_gyro_bias;

// Time sync with agent
static unsigned long long time_offset_ms = 0;
static uint32_t last_time_sync_ms = 0;

// ============================ Robot Modules ============================
static QuadEncoderReader enc;
extern uint32_t last_cmd_ms;

// ============================ IMU/KF/PID and fast-loop state ============================
static IMU_MPU6050 g_imu;
static KalmanYaw g_kf;
static PIDRate g_pid_wz(0.8f, 0.0f, 0.0f, -1.0f, 1.0f, -0.3f, 0.3f, 10.0f);

static volatile float g_V_cmd = 0.0f;
static volatile float g_W_cmd = 0.0f;

static int32_t prev_counts[W_COUNT] = {0, 0, 0, 0};
static float yaw_enc_unwrapped = 0.0f;
static uint32_t last_fast_us = 0;

#ifndef TRACK_W_M
#ifdef WHEEL_SEP
#define TRACK_W_M WHEEL_SEP
#else
#define TRACK_W_M 0.365f
#endif
#endif

#ifndef WHEEL_RADIUS
#define WHEEL_RADIUS 0.0635f
#endif

#ifndef ENCODER_PPR_OUTPUT_DEFAULT
#define ENCODER_PPR_OUTPUT_DEFAULT 5940.0f
#endif

static inline float distance_per_tick()
{
    return (2.0f * PI * WHEEL_RADIUS) / (float)ENCODER_PPR_OUTPUT_DEFAULT;
}
static inline float round2(float x)
{
    return roundf(x * 100.0f) / 100.0f; // ปัดทศนิยม 2 ตำแหน่ง
}
static inline float m_to_cm_2f(float m)
{
    return round2(m * 100.0f); // m → cm แล้วปัด .2f
}

// ============================ Forward Decls ============================
static void rclErrorLoop();
static bool createEntities();
static bool destroyEntities();
static void syncTime();

static void on_cmd_vel(const void *msgin);
static void on_timer(rcl_timer_t *timer, int64_t last_call_time);
static void on_joy_reset(const void *msgin);

static void on_joy_reset(const void *msgin)
{
    const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
    static bool prev = false; // จำสถานะก่อนหน้า
    bool now = m->data;

    if (now && !prev)
    {
        enc.reset();
        DEBUG_PRINTLN("[RESET] wheel distance cleared by joystick");
    }
    prev = now;
}

static inline void fast_loop_200hz()
{
    const uint32_t now_us = micros();
    if (last_fast_us == 0)
    {
        last_fast_us = now_us;
        return;
    }
    uint32_t dt_us = now_us - last_fast_us;
    if (dt_us < 5000U)
        return;
    last_fast_us = now_us;

    const float dt = dt_us * 1e-6f;

    // 1) IMU -> world yaw-rate
    g_imu.update();
    const float wz_world = g_imu.yaw_rate_world_rad;

    // 2) Encoder yaw
    int32_t cFL = (int32_t)enc.counts(W_FL);
    int32_t cFR = (int32_t)enc.counts(W_FR);
    int32_t cRL = (int32_t)enc.counts(W_RL);
    int32_t cRR = (int32_t)enc.counts(W_RR);

    const int32_t dFL = cFL - prev_counts[W_FL];
    const int32_t dFR = cFR - prev_counts[W_FR];
    const int32_t dRL = cRL - prev_counts[W_RL];
    const int32_t dRR = cRR - prev_counts[W_RR];

    prev_counts[W_FL] = cFL;
    prev_counts[W_FR] = cFR;
    prev_counts[W_RL] = cRL;
    prev_counts[W_RR] = cRR;

    const float d_per_tick = distance_per_tick();
    const float dsL = 0.5f * ((float)dFL + (float)dRL) * d_per_tick;
    const float dsR = 0.5f * ((float)dFR + (float)dRR) * d_per_tick;
    const float dtheta = (dsR - dsL) / (float)TRACK_W_M;
    yaw_enc_unwrapped += dtheta;

    // 3) Kalman
    g_kf.step(wz_world, yaw_enc_unwrapped, dt);
    const float yaw_kf = g_kf.yaw();
    const float bias = g_kf.bias();

#if USE_INNER_PID
    // 4) Inner PID rate
    const float u = g_pid_wz.step(g_W_cmd, wz_world, dt);
    const float W_eff = g_W_cmd + u;
    cmdVW_to_targets(g_V_cmd, W_eff);
#endif

    // 5) Debug publish ~25 Hz
    static uint32_t dbg_last_ms = 0;
    const uint32_t now_ms = millis();
    if (g_state == AGENT_CONNECTED && (now_ms - dbg_last_ms) >= 40U)
    {
        msg_yaw_rate.data = wz_world;
        msg_yaw_kf.data = yaw_kf;
        msg_gyro_bias.data = bias;
        RCSOFTCHECK(rcl_publish(&pub_yaw_rate, &msg_yaw_rate, NULL));
        RCSOFTCHECK(rcl_publish(&pub_yaw_kf, &msg_yaw_kf, NULL));
        RCSOFTCHECK(rcl_publish(&pub_gyro_bias, &msg_gyro_bias, NULL));
        dbg_last_ms = now_ms;
    }
}

// ============================ Setup ============================
void setup()
{
    Serial.begin(115200);
    delay(100);
    DEBUG_PRINTLN("\n[BOOT] ESP32 starting...");

#ifndef MICROROS_WIFI
    set_microros_serial_transports(Serial);
    DEBUG_PRINTLN("[MICROROS] Serial transport set");
#else
#error "Wi-Fi transport not configured in this example"
#endif

    // Hardware init
    motorDrive_begin();
    enc.begin(true);
    enc.setInvert(ENC_INV_FL < 0, ENC_INV_FR < 0, ENC_INV_RL < 0, ENC_INV_RR < 0);
    enc.setWheelRadius(ENC_WHEEL_RADIUS);
    enc.setPPR(ENCODER_PPR_OUTPUT_DEFAULT);

    // Seed counts for fast loop
    prev_counts[W_FL] = (int32_t)enc.counts(W_FL);
    prev_counts[W_FR] = (int32_t)enc.counts(W_FR);
    prev_counts[W_RL] = (int32_t)enc.counts(W_RL);
    prev_counts[W_RR] = (int32_t)enc.counts(W_RR);

    if (!g_imu.begin())
        DEBUG_PRINTLN("[IMU] init FAILED (check wiring)");
    else
        DEBUG_PRINTLN("[IMU] init OK");

    g_kf.setNoise(1e-5f, 1e-6f, 1e-3f);
    g_kf.init(0.0f, 0.0f);

#if USE_INNER_PID
    g_pid_wz.setGains(0.6f, 0.0f, 0.02f);
    g_pid_wz.setIClamp(-0.3f, 0.3f);
    g_pid_wz.setOutputClamp(-0.6f, 0.6f);
    g_pid_wz.setDLpf(10.0f);
#endif

    syncTime();
    last_time_sync_ms = millis();

    DEBUG_PRINTF("[INFO] Domain: %d, Topics: %s, %s, %s, %s, %s, %s\n",
                 ROS_DOMAIN_ID_MCU,
                 TOPIC_WHEEL_TICKS, TOPIC_HEARTBEAT, TOPIC_CMD_VEL,
                 TOPIC_YAW_RATE, TOPIC_YAW_KF, TOPIC_GYRO_BIAS);
}

// ============================ Main loop ============================
void loop()
{
    // Fast loop
    fast_loop_200hz();

    // Periodic time sync with agent (10 s)
    if (millis() - last_time_sync_ms > 10000U)
    {
        syncTime();
        last_time_sync_ms = millis();
    }

    // State machine
    switch (g_state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, {
            rmw_ret_t pr = rmw_uros_ping_agent(500, 4);
            if (pr == RMW_RET_OK)
            {
                g_state = AGENT_AVAILABLE;
                DEBUG_PRINTLN("[PING] agent available");
            }
            else
            {
                DEBUG_PRINTLN("[PING] no agent");
            }
        });
        break;

    case AGENT_AVAILABLE:
        DEBUG_PRINTLN("[STATE] AGENT_AVAILABLE -> creating entities...");
        if (createEntities())
        {
            g_state = AGENT_CONNECTED;
            DEBUG_PRINTLN("[STATE] AGENT_CONNECTED");
        }
        else
        {
            DEBUG_PRINTLN("[ERR] createEntities failed, retry...");
            destroyEntities();
            g_state = WAITING_AGENT;
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(300, {
            if (RMW_RET_OK != rmw_uros_ping_agent(300, 3))
            {
                DEBUG_PRINTLN("[WARN] agent lost -> disconnect");
                g_state = AGENT_DISCONNECTED;
            }
        });
        if (g_state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
        }
        break;

    case AGENT_DISCONNECTED:
        cmdVW_to_targets(0.f, 0.f);
        motorDrive_update();
        destroyEntities();
        g_state = WAITING_AGENT;
        DEBUG_PRINTLN("[STATE] DISCONNECTED -> WAITING_AGENT");
        break;
    }

    // Local serial command helper (ปิดเมื่อเชื่อม Agent)
    if (g_state != AGENT_CONNECTED)
    {
        motorDrive_handleSerialOnce();
    }
}

// ============================ Callbacks ============================
static void on_cmd_vel(const void *msgin)
{
    const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
    float V = (float)m->linear.x;
    float W = (float)m->angular.z;

    g_V_cmd = V;
    g_W_cmd = W;

    cmdVW_to_targets(V, W);
    last_cmd_ms = millis();
}

static void on_timer(rcl_timer_t * /*timer*/, int64_t /*last_call_time*/)
{
    enc.update();
    motorDrive_update();

    if (msg_ticks.data.size >= 4)
    {
        // ใช้ "ระยะสะสมต่อวงล้อ" จาก QuadEncoderReader (หน่วย m)
        // แล้วแปลง → cm และปัด .2f ก่อนส่งออก
        msg_ticks.data.data[0] = m_to_cm_2f(enc.totalDistanceM(W_FL));
        msg_ticks.data.data[1] = m_to_cm_2f(enc.totalDistanceM(W_FR));
        msg_ticks.data.data[2] = m_to_cm_2f(enc.totalDistanceM(W_RL));
        msg_ticks.data.data[3] = m_to_cm_2f(enc.totalDistanceM(W_RR));

        RCSOFTCHECK(rcl_publish(&pub_ticks, &msg_ticks, NULL));
    }

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

    rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&init_opts, allocator) != RCL_RET_OK)
        return false;

    rcl_ret_t drc = rcl_init_options_set_domain_id(&init_opts, ROS_DOMAIN_ID_MCU);
    if (drc != RCL_RET_OK)
    {
        DEBUG_PRINTF("[ERR] set_domain_id rc=%d\n", drc);
    }
    else
    {
        DEBUG_PRINTF("[OK ] set_domain_id=%d\n", ROS_DOMAIN_ID_MCU);
    }

    if (rclc_support_init_with_options(&support, 0, NULL, &init_opts, &allocator) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] rclc_support_init_with_options");
        rcl_init_options_fini(&init_opts);
        return false;
    }
    rcl_init_options_fini(&init_opts);

    // Node
    if (rclc_node_init_default(&node, "esp32_base", "", &support) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] node init");
        return false;
    }
    DEBUG_PRINTLN("[INIT] node esp32_base created");

    // Publishers
    if (rclc_publisher_init_default(&pub_ticks, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                    TOPIC_WHEEL_TICKS) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] pub wheel_ticks");
        return false;
    }
    rosidl_runtime_c__float32__Sequence__init(&msg_ticks.data, 4);
    msg_ticks.data.data[0] = 0;
    msg_ticks.data.data[1] = 0;
    msg_ticks.data.data[2] = 0;
    msg_ticks.data.data[3] = 0;
    DEBUG_PRINTLN("[INIT] pub wheel_ticks created");

    if (rclc_publisher_init_default(&pub_heartbeat, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                    TOPIC_HEARTBEAT) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] pub heartbeat");
        return false;
    }
    DEBUG_PRINTLN("[INIT] pub robot_heartbeat created");

    if (rclc_publisher_init_default(&pub_yaw_rate, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                    TOPIC_YAW_RATE) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] pub yaw_rate");
        return false;
    }
    DEBUG_PRINTLN("[INIT] pub yaw_rate created");

    if (rclc_publisher_init_default(&pub_yaw_kf, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                    TOPIC_YAW_KF) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] pub yaw_kf");
        return false;
    }
    DEBUG_PRINTLN("[INIT] pub yaw_kf created");

    if (rclc_publisher_init_default(&pub_gyro_bias, &node,
                                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                    TOPIC_GYRO_BIAS) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] pub gyro_bias");
        return false;
    }
    DEBUG_PRINTLN("[INIT] pub gyro_bias created");

    // Subscriber
    if (rclc_subscription_init_default(&sub_cmd_vel, &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                                       TOPIC_CMD_VEL) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] sub cmd_vel");
        return false;
    }
    DEBUG_PRINTLN("[INIT] sub cmd_vel created");

    // หลังจากสร้าง sub_cmd_vel เสร็จ ให้เพิ่ม:
    if (rclc_subscription_init_default(&sub_joy_reset, &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                       TOPIC_JOY_RESET) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] sub joy_reset");
        return false;
    }
    DEBUG_PRINTLN("[INIT] sub joy_reset created");

    // Timer
    const uint32_t CTRL_MS = 20;
    if (rclc_timer_init_default(&timer_ctrl, &support, RCL_MS_TO_NS(CTRL_MS), on_timer) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] timer init");
        return false;
    }
    DEBUG_PRINTLN("[INIT] timer 20ms created");

    // Executor
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK)
    {
        DEBUG_PRINTLN("[ERR] executor init");
        return false;
    }
    rclc_executor_add_timer(&executor, &timer_ctrl);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, on_cmd_vel, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_joy_reset,
                                   &msg_joy_reset, on_joy_reset, ON_NEW_DATA);
    DEBUG_PRINTLN("[INIT] executor ready");

    // Initial time sync
    syncTime();

    DEBUG_PRINTLN("[INIT] micro-ROS entities created");
    return true;
}

static bool destroyEntities()
{
    rcl_subscription_fini(&sub_cmd_vel, &node);
    rcl_subscription_fini(&sub_joy_reset, &node);

    rcl_publisher_fini(&pub_gyro_bias, &node);
    rcl_publisher_fini(&pub_yaw_kf, &node);
    rcl_publisher_fini(&pub_yaw_rate, &node);
    rcl_publisher_fini(&pub_heartbeat, &node);
    rcl_publisher_fini(&pub_ticks, &node);

    if (msg_ticks.data.data)
        rosidl_runtime_c__float__Sequence__fini(&msg_ticks.data);

    rcl_timer_fini(&timer_ctrl);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    DEBUG_PRINTLN("[CLEAN] micro-ROS entities destroyed");
    return true;
}

// ============================ Time sync helpers ============================
static void syncTime()
{
    RCSOFTCHECK(rmw_uros_sync_session(10));
    unsigned long now_ms = millis();
    unsigned long long agent_ms = rmw_uros_epoch_millis();
    time_offset_ms = (agent_ms > 0ULL) ? (agent_ms - now_ms) : 0ULL;
    DEBUG_PRINTF("[TIME] agent_ms=%llu, mcu_ms=%lu, offset=%llu\n",
                 agent_ms, now_ms, time_offset_ms);
}

static void rclErrorLoop()
{
    DEBUG_PRINTLN("[RCL] Fatal error -> restart");
    delay(200);
    ESP.restart();
}

#endif // Node1