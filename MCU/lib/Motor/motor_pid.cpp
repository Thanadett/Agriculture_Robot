#include "motor_pid.h"

static inline float clampf_(float x, float a, float b) {
  return (x < a) ? a : (x > b) ? b : x;
}
static inline int clampsi_(int x, int a, int b) {
  return (x < a) ? a : (x > b) ? b : x;
}
static inline float sgnf_(float x) {
  return (x >= 0.0f) ? 1.0f : -1.0f;
}

MotorPID::MotorPID(MotorDriveMode mode,
                   uint8_t ledc_ch1,
                   uint8_t ledc_ch2,
                   uint32_t pwm_freq_hz,
                   uint8_t pwm_resolution_bits,
                   float omega_max_rad_s)
: mode_(mode),
  ch1_(ledc_ch1),
  ch2_(ledc_ch2),
  pwm_freq_(pwm_freq_hz),
  pwm_res_bits_(pwm_resolution_bits),
  omega_max_(omega_max_rad_s)
{
  pwm_max_ = (1 << pwm_res_bits_) - 1; // เช่น 10-bit → 1023
  deadzone_min_pwm_ = 0;
}

void MotorPID::attach(const PinsPWMDir& p) {
  pins1_ = p;
  // PWM
  ledcSetup(ch1_, pwm_freq_, pwm_res_bits_);
  ledcAttachPin(pins1_.pwm, ch1_);
  // DIR
  pinMode(pins1_.dir, OUTPUT);
  digitalWrite(pins1_.dir, LOW);
  pins_attached_ = true;
}

void MotorPID::attach(const PinsIn1In2& p) {
  pins2_ = p;
  // IN1
  ledcSetup(ch1_, pwm_freq_, pwm_res_bits_);
  ledcAttachPin(pins2_.in1_pwm, ch1_);
  // IN2
  if (ch2_ == 0xFF) { // ต้องมี channel ที่สอง
    // ป้องกันใช้ผิด
    while (true) { delay(1000); }
  }
  ledcSetup(ch2_, pwm_freq_, pwm_res_bits_);
  ledcAttachPin(pins2_.in2_pwm, ch2_);
  pins_attached_ = true;
}

void MotorPID::setEnabled(bool en) {
  enabled_ = en;
  if (!en) stop();
}

void MotorPID::setPI(float kp, float ki) {
  kp_ = kp; ki_ = ki;
}

void MotorPID::setFeedforward(float kv_pwm_per_rad, float ka_pwm_per_rad2) {
  kv_ff_ = kv_pwm_per_rad;
  ka_ff_ = ka_pwm_per_rad2;
}

void MotorPID::setOutputLimits(int pwm_max, int min_pwm_deadzone) {
  pwm_max_ = pwm_max;
  deadzone_min_pwm_ = min_pwm_deadzone;
}

void MotorPID::setSlewRate(float d_omega_ref_max) {
  slew_domega_max_ = (d_omega_ref_max > 0.0f) ? d_omega_ref_max : 999.0f;
}

void MotorPID::setBatteryComp(float v_nominal) {
  batt_comp_enable_ = true;
  v_nom_ = v_nominal;
}

void MotorPID::updateBatteryVoltage(float v_batt) {
  v_batt_ = (v_batt > 0.1f) ? v_batt : v_nom_;
}

void MotorPID::setWatchdog(uint32_t timeout_ms) {
  wd_timeout_ms_ = timeout_ms;
}

void MotorPID::setTarget(float omega_ref) {
  wd_last_cmd_ms_ = millis();
  // จำกัด setpoint
  omega_ref = clampf_(omega_ref, -omega_max_, omega_max_);
  // slew-rate limit
  float d = omega_ref - omega_ref_;
  float max_step = slew_domega_max_;
  if (fabsf(d) > max_step) {
    omega_ref_ += sgnf_(d) * max_step;
  } else {
    omega_ref_ = omega_ref;
  }
}

void MotorPID::update(float omega_meas, float dt) {
  if (!pins_attached_) return;

  // Watchdog
  if (wd_timeout_ms_ > 0 && (millis() - wd_last_cmd_ms_) > wd_timeout_ms_) {
    stop();
    return;
  }
  if (!enabled_) {
    stop();
    return;
  }

  // Error
  err_ = omega_ref_ - omega_meas;

  // Feedforward (PWM units)
  float domega_ref = (omega_ref_ - omega_ref_prev_) / (dt > 1e-4f ? dt : 1e-4f);
  float u_ff = kv_ff_ * omega_ref_ + ka_ff_ * domega_ref;

  // PI
  float u_p = kp_ * err_;
  // conditional integration (anti-windup): อินทิเกรตเมื่อไม่อิ่ม หรือ error ช่วยคลายอิ่ม
  float u_pre = u_ff + u_p + integ_;
  bool saturated = (fabsf(u_pre) >= (float)pwm_max_);
  if (!saturated || (u_pre * err_) < 0.0f) {
    integ_ += ki_ * err_ * dt;
    integ_ = clampf_(integ_, integ_min_, integ_max_);
  }
  float u = u_ff + u_p + integ_;

  // Battery compensation
  if (batt_comp_enable_) {
    float scale = (v_batt_ > 1e-3f) ? (v_nom_ / v_batt_) : 1.0f;
    u *= scale;
  }

  // Clamp and apply deadzone
  int u_pwm = (int)roundf(u);
  u_pwm = clampsi_(u_pwm, -pwm_max_, pwm_max_);

  // ถ้า u_pwm เป็น 0 ให้หยุดจริง ๆ
  if (u_pwm == 0) {
    last_pwm_out_ = 0;
    ledcWriteSigned_(0);
    omega_ref_prev_ = omega_ref_;
    return;
  }

  // ใส่ deadzone offset
  int abs_u = abs(u_pwm);
  abs_u = applyDeadzone_(abs_u);
  u_pwm = (u_pwm > 0) ? abs_u : -abs_u;

  // ส่งออก
  ledcWriteSigned_(u_pwm);
  last_pwm_out_ = u_pwm;

  // เก็บสำหรับรอบต่อไป
  omega_ref_prev_ = omega_ref_;
}

void MotorPID::stop() {
  integ_ = 0.0f;
  last_pwm_out_ = 0;
  ledcWriteSigned_(0);
}

int MotorPID::applyDeadzone_(int pwm_abs) const {
  if (deadzone_min_pwm_ <= 0) return pwm_abs;
  if (pwm_abs == 0) return 0;
  int out = deadzone_min_pwm_ + pwm_abs;
  if (out > pwm_max_) out = pwm_max_;
  return out;
}

void MotorPID::ledcWriteSigned_(int pwm_signed) {
  pwm_signed = clampsi_(pwm_signed, -pwm_max_, pwm_max_);

  if (mode_ == DRIVE_PWM_DIR) {
    // ตั้งทิศ
    if (pwm_signed >= 0) digitalWrite(pins1_.dir, HIGH);
    else                 digitalWrite(pins1_.dir, LOW);

    int duty = abs(pwm_signed);
    ledcWrite(ch1_, duty);

  } else { // DRIVE_PWM_IN1IN2
    int duty = abs(pwm_signed);
    if (pwm_signed >= 0) {
      // เดินหน้า: IN1 = PWM, IN2 = 0
      ledcWrite(ch1_, duty);
      ledcWrite(ch2_, 0);
    } else {
      // ถอย: IN1 = 0, IN2 = PWM
      ledcWrite(ch1_, 0);
      ledcWrite(ch2_, duty);
    }
  }
}
