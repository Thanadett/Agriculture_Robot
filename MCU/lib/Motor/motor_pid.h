#pragma once
#include <Arduino.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// โหมดขับมอเตอร์
enum MotorDriveMode : uint8_t {
  DRIVE_PWM_DIR = 0,   // ขับด้วย PWM + DIR (1 PWM, 1 DIR)
  DRIVE_PWM_IN1IN2 = 1 // ขับด้วย IN1/IN2 (dual-PWM)
};

// คลาสควบคุมความเร็วล้อ 1 ตัว (rad/s) ด้วย PI + Feedforward
class MotorPID {
public:
  struct PinsPWMDir {
    int pwm;   // ขา PWM (LEDC)
    int dir;   // ขา DIR (digital)
  };
  struct PinsIn1In2 {
    int in1_pwm; // ขา IN1 (LEDC)
    int in2_pwm; // ขา IN2 (LEDC)
  };

  // ตั้งค่าหลักตอนสร้าง
  MotorPID(MotorDriveMode mode,
           uint8_t ledc_ch1,           // ช่อง LEDC ช่องหลัก
           uint8_t ledc_ch2 = 0xFF,    // ช่อง LEDC ช่องที่สอง (เฉพาะ dual-PWM)
           uint32_t pwm_freq_hz = 20000,
           uint8_t pwm_resolution_bits = 10, // 10-bit => 0..1023
           float omega_max_rad_s = 4.0f);    // ความเร็วเชิงมุมสูงสุดของล้อ (ของคุณ ≈3.875 rad/s)

  // bind pins
  void attach(const PinsPWMDir& p);
  void attach(const PinsIn1In2& p);

  // เปิด/ปิดการควบคุม (disable จะหยุดมอเตอร์ทันที)
  void setEnabled(bool en);
  bool enabled() const { return enabled_; }

  // กำหนด gains และตัวช่วย
  void setPI(float kp, float ki);
  void setFeedforward(float kv_pwm_per_rad, float ka_pwm_per_rad2 = 0.0f); // u_ff = kv*w_ref + ka*dw/dt
  void setOutputLimits(int pwm_max, int min_pwm_deadzone = 0);  // clamp และ deadzone offset
  void setSlewRate(float d_omega_ref_max); // rad/s per update (จำกัด setpoint เคลื่อนที่เร็วเกิน)
  void setBatteryComp(float v_nominal);    // เปิดคอมเพนเสตแรงดันแบต (ตั้ง V_nom)
  void updateBatteryVoltage(float v_batt); // ใส่ค่าแบตล่าสุดเป็นระยะ ๆ

  // watchdog: ถ้าไม่ได้เรียก setTarget() เกิน timeout_ms จะหยุดมอเตอร์
  void setWatchdog(uint32_t timeout_ms);

  // ตั้งเป้าหมายความเร็ว (rad/s) — เรียกทุกรอบคุม (เช่น 200–500 Hz)
  void setTarget(float omega_ref);

  // อัปเดตควบคุม (ใส่ความเร็วจริง rad/s + dt วินาที) — จะสั่ง PWM ออกขา
  void update(float omega_meas, float dt);

  // หยุด/เบรก
  void stop();

  // อ่านสถานะ
  float target() const { return omega_ref_; }
  float error()  const { return err_; }
  int   lastPWM() const { return last_pwm_out_; }

private:
  void ledcWriteSigned_(int pwm_signed);
  int  applyDeadzone_(int pwm_abs) const;

private:
  // โหมด/พิน/LEDC
  MotorDriveMode mode_;
  bool pins_attached_ = false;

  PinsPWMDir  pins1_{-1, -1};
  PinsIn1In2  pins2_{-1, -1};
  uint8_t ch1_, ch2_;
  uint32_t pwm_freq_;
  uint8_t  pwm_res_bits_;
  int pwm_max_;           // max เลข PWM (เช่น 1023)
  int deadzone_min_pwm_;  // offset แรงเสียดทาน (ถ้า 0 คือไม่ใช้)

  // สถานะ
  bool enabled_ = true;
  float omega_max_;       // สำหรับ clamp ref
  float omega_ref_ = 0.0f;
  float omega_ref_prev_ = 0.0f;
  float err_ = 0.0f;
  float integ_ = 0.0f;
  float kp_ = 6.6f, ki_ = 3.3f; // ตั้งต้นตามที่คำนวณให้ไว้
  float kv_ff_ = 65.8f;         // PWM/(rad/s) จาก 255/3.875 → ปรับตามความละเอียดจริง
  float ka_ff_ = 0.0f;          // เสริมความเร่ง หากอยาก
  float slew_domega_max_ = 999.0f; // ไม่จำกัดโดยดีฟอลต์

  // Battery compensation
  bool  batt_comp_enable_ = false;
  float v_nom_ = 12.4f;
  float v_batt_ = 12.4f;

  // Anti-windup
  float integ_min_ = -80.0f, integ_max_ = 80.0f;

  // Watchdog
  uint32_t wd_timeout_ms_ = 200;
  uint32_t wd_last_cmd_ms_ = 0;

  // Output record
  int last_pwm_out_ = 0;
};
