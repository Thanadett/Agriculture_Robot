#pragma once
#include <Arduino.h>
#include "config.h"

// ---------------- Robot Params ----------------
static constexpr float WHEEL_SEP = 0.30f;          // m
static constexpr float WHEEL_RADIUS = 0.0635f;     // m
static constexpr float MAX_OMEGA_FOR_FULL = 20.0f; // rad/s -> 100% PWM

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