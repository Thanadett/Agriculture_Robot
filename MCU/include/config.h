#pragma once
#include <Arduino.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================== โหมดระบบ (เลือก A/B) ==================
// 1 = PID (v,ω) บน ESP32, 0 = ไม่ทำ PID (ผ่าน /cmd_vel ตรงเข้ามอเตอร์)
#define USE_PID_ESP32 1

// ================== ความถี่ลูป/หัวข้อ ROS ==================
#define CONTROL_HZ 200.0f // ลูปควบคุมหลัก (อ่าน encoder + PID/pass-through + motorDrive_update)
#define ODOM_HZ 50.0f     // publish wheel/odom
#define IMU_HZ 100.0f     // publish imu/data

#define ODOM_FRAME "odom"
#define BASE_FRAME "base_link"
#define IMU_FRAME "imu_link"

// ================== พารามิเตอร์ตัวรถ ==================
#define WHEEL_RADIUS_M 0.0635f                                // m
#define TRACK_M 0.30f                                         // ฐานล้อซ้าย-ขวา (m)
#define MOTOR_MAX_RPM 37.0f                                   // rpm ที่ปลายล้อ (สุดรอบ)
#define OMEGA_MAX_RAD_S (MOTOR_MAX_RPM * 2.0f * M_PI / 60.0f) // ≈ 3.875

// ================== Encoder (ESP32Encoder) ==================
// พัลส์/รอบฝั่งมอเตอร์ * อัตราทด * โหมดควอด (2=half, 4=full)
#define ENCODER_PPR_MOTOR 11.0f
#define REDUCTION_RATIO 270.0f
#define ENCODER_QUAD_MODE_FULL 1 // 1=full-quad, 0=half-quad
#if ENCODER_QUAD_MODE_FULL
#define QUAD_MULT_DEFAULT 4.0f
#else
#define QUAD_MULT_DEFAULT 2.0f
#endif
#define ENCODER_PPR_OUTPUT_DEFAULT (ENCODER_PPR_MOTOR * REDUCTION_RATIO * QUAD_MULT_DEFAULT)

// Encoder pin (A,B)
#define ENC_FL_A 35
#define ENC_FL_B 32
#define ENC_RL_A 33
#define ENC_RL_B 25
#define ENC_FR_A 19
#define ENC_FR_B 18
#define ENC_RR_A 17
#define ENC_RR_B 16

// ทิศอ่าน encoder (+1=ปกติ, -1=กลับทิศ)
#define ENC_INV_FL (+1)
#define ENC_INV_FR (-1)
#define ENC_INV_RL (+1)
#define ENC_INV_RR (-1)

// ฟิลเตอร์ความเร็วล้อ: 0=None, 1=EMA, 2=Butterworth 2nd
#define VEL_FILTER_MODE 2
#define EMA_ALPHA 0.35f
#define BW2_FC_HZ 6.0f       // cutoff (Hz)
#define BW2_FS_HZ CONTROL_HZ // sample rate

// ================== IMU (MPU6050 on I2C) ==================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define CF_ALPHA 0.96f
#define IMU_CALIB_SAMPLES 800

// ================== มอเตอร์ ==================
// ขามอเตอร์ซ้ายหน้า/หลัง
#define LF_IN1 14
#define LF_IN2 12
#define LR_IN1 27
#define LR_IN2 26
// ขามอเตอร์ขวาหน้า/หลัง
#define RF_IN1 25
#define RF_IN2 33
#define RR_IN1 4
#define RR_IN2 0

// LEDC ช่อง (0..7) และพารามิเตอร์ PWM
static constexpr int PWM_FREQ_HZ = 20000; // 20kHz เงียบ
static constexpr int PWM_RES_BITS = 8;    // 8-bit (0..255)
static constexpr int PWM_MAX_DUTY = (1 << PWM_RES_BITS) - 1;

static constexpr int CH_LF_IN1 = 0;
static constexpr int CH_LF_IN2 = 1;
static constexpr int CH_LR_IN1 = 2;
static constexpr int CH_LR_IN2 = 3;
static constexpr int CH_RF_IN1 = 4;
static constexpr int CH_RF_IN2 = 5;
static constexpr int CH_RR_IN1 = 6;
static constexpr int CH_RR_IN2 = 7;

// โพลาริตี้มอเตอร์ (+1 ตรง, -1 กลับ) — ให้สอดคล้องกับ ENC_INV_* เสมอ
#define MOTOR_SIGN_LF (+1.0f)
#define MOTOR_SIGN_LR (+1.0f)
#define MOTOR_SIGN_RF (-1.0f)
#define MOTOR_SIGN_RR (-1.0f)

// ================== PID ค่าเริ่มต้น ==================
#define BODY_KP_V 1.5f
#define BODY_KI_V 0.8f
#define BODY_KD_V 0.0f
#define BODY_KP_W 2.0f
#define BODY_KI_W 1.0f
#define BODY_KD_W 0.0f

#define BODY_V_MAX 0.24f  // m/s
#define BODY_W_MAX 1.50f  // rad/s
#define BODY_AV_MAX 2.40f // m/s^2
#define BODY_AW_MAX 14.0f // rad/s^2

#define BODY_I_V_ABS 0.15f
#define BODY_I_W_ABS 0.80f
#define BODY_IMU_WEIGHT 0.0f // 0=ใช้ encoder ล้วน, 0..1 ผสม IMU yaw-rate

// ================== อื่นๆ ==================
#define CMD_SERIAL_ENABLE 1 // พิมพ์ "VW V= W=" และ "ESTOP 0|1" ทาง Serial ได้
