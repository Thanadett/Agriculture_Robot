#pragma once
#include <Arduino.h>

// ---------------- Pins ----------------
#define LF_IN1 14
#define LF_IN2 12
#define LR_IN1 27
#define LR_IN2 26

#define RF_IN1 33   //not used
#define RF_IN2 25   //not used
#define RR_IN1 4    //not used
#define RR_IN2 0    //not used

// ---------------- LEDC PWM ----------------
static constexpr int PWM_FREQ_HZ = 20000; // 20kHz เงียบ
static constexpr int PWM_RES_BITS = 8;    // 10-bit (0..1023)
static constexpr int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1;

// 8 ช่องไม่ซ้ำกัน
static constexpr int CH_LF_IN1 = 0;
static constexpr int CH_LF_IN2 = 1;
static constexpr int CH_LR_IN1 = 2;
static constexpr int CH_LR_IN2 = 3;
static constexpr int CH_RF_IN1 = 4;
static constexpr int CH_RF_IN2 = 5;
static constexpr int CH_RR_IN1 = 6;
static constexpr int CH_RR_IN2 = 7;

// ---------------- Robot Params ----------------
static constexpr float WHEEL_SEP = 0.20f;    // m
static constexpr float WHEEL_RADIUS = 0.05f; // m

static constexpr float MAX_LINEAR_UNITS = 255.0f;
static constexpr float OMEGA_WHEEL_FULL = (MAX_LINEAR_UNITS > 1e-6f)
                                              ? (MAX_LINEAR_UNITS / WHEEL_RADIUS) // = 255 / R
                                              : 1.0f;                             // กันหารศูนย์

static constexpr bool INVERT_LF = false;
static constexpr bool INVERT_LR = false;
static constexpr bool INVERT_RF = false;
static constexpr bool INVERT_RR = false;

// Safety / smoothness
static constexpr uint32_t CMD_TIMEOUT_MS = 2000; // ms
static constexpr float RAMP_STEP = 0.05f;
static constexpr float IDLE_DECAY = 0.85f;

// ---------------- Globals (defined in .cpp) ----------------
extern String rx_line;
extern bool estop;
extern uint32_t last_cmd_ms;

// เป้าหมาย PWM (normalized -1..1)
extern float tgt_LF, tgt_LR, tgt_RF, tgt_RR;
// เอาต์พุต PWM หลัง slew
extern float out_LF, out_LR, out_RF, out_RR;

// ---------------- Public API ----------------
// เรียกครั้งเดียวใน setup() (ตั้งค่า PWM, พิมพ์เมนู)
void motorDrive_begin();

// เรียกบ่อยๆ ใน loop() เพื่ออ่านคำสั่งจาก Serial (VW/P/PW4/ESTOP)
void motorDrive_handleSerialOnce();

// เรียกบ่อยๆ ใน loop() เพื่อทำ watchdog + slew + เขียน PWM
void motorDrive_update();

// เผื่ออยากเรียกเองจากที่อื่น (เช่น callback micro-ROS) ให้ map V,W -> เป้าหมายล้อ
void cmdVW_to_targets(float V_mps, float W_radps);
