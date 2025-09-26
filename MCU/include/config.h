#pragma once
#include <Arduino.h>

// ---------------- Robot Params ----------------
static constexpr float WHEEL_SEP = 0.365f;         // m
static constexpr float WHEEL_RADIUS = 0.0635f;     // m
static constexpr float MAX_OMEGA_FOR_FULL = 20.0f; // rad/s -> 100% PWM

static constexpr bool INVERT_LF = false;
static constexpr bool INVERT_LR = false;
static constexpr bool INVERT_RF = false;
static constexpr bool INVERT_RR = false;

//-------- Motor Driver Pins ----------------
#define LF_IN1 32 // Left  Front IN1
#define LF_IN2 23 // Left  Front IN2

#define LR_IN1 27 // Left  Rear  IN1
#define LR_IN2 26 // Left  Rear  IN2

#define RF_IN1 12 // Right Front IN1
#define RF_IN2 14 // Right Front IN2

#define RR_IN1 25 // Right Rear  IN1
#define RR_IN2 33 // Right Rear  IN2

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

// Safety / smoothness
static constexpr uint32_t CMD_TIMEOUT_MS = 2000; // ms
static constexpr float RAMP_STEP = 0.05f;
static constexpr float IDLE_DECAY = 0.85f;

/* -------- Encoder Configuration --------
 * JGB37-520 + เกียร์ 270:1, PPR มอเตอร์ = 11 (ก่อนเกียร์)
 * โค้ดหลักคูณ x4 จาก quad decoding
 */
#define ENCODER_PPR_MOTOR 11.0f
#define REDUCTION_RATIO 270.0f   // gearbox ratio
#define QUAD_MULTIPLIER 2.0f     // half-quad
#define ENC_WHEEL_RADIUS 0.0635f // m (D=0.127 m ~ 5")

/* Encoder Pins (GPIO) — ปัจจุบันใช้ตามที่ระบุ
 * หมายเหตุ: GPIO0/2/15 เป็น boot-strap pins อาจกวนการบูตได้ ถ้าใช้จริงแล้วเจออาการแปลก ๆ
 * แนะนำย้ายไปพิน input-only (34/35/36/39) หรือพินทั่วไปที่ปลอดภัย
 */
// Front Left
#define ENC_FL_A 16
#define ENC_FL_B 17

// Rear Left
#define ENC_RL_A 18
#define ENC_RL_B 19

// Front Right
#define ENC_FR_A 13
#define ENC_FR_B 4 // strap (GPIO4) — ใช้ได้ถ้าเอ็นโค้ดเดอร์ไม่ดึง LOW ตอนบูต

// Rear Right
#define ENC_RR_A 5 // strap (GPIO5)
#define ENC_RR_B 2 // strap (GPIO2)  *ถ้าเสี่ยงบูต ให้สลับกับ 15 แล้วทดสอบ*

// Encoder direction correction (+1 / -1)
#define ENC_INV_FL (+1)
#define ENC_INV_FR (-1)
#define ENC_INV_RL (+1)
#define ENC_INV_RR (-1)

/* -------- IMU (MPU6050) -------- */
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define CF_ALPHA 0.96f        // (reserved) complementary filter alpha
#define IMU_CALIB_SAMPLES 800 // Gyro Z bias calibration samples