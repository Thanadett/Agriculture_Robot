#pragma once
#include <Arduino.h>
#include "config.h"
#include "PID.h"
#include "Encoder.h"
#include "MotorDriver.h"
#include "Wheel.h"

class VelocityController
{
public:
  void begin(Encoder *enc, MotorDriver *mot);
  // Set base cmd (v, w) after limiting jerk/acc bounds
  void setCmdVel(float v_mps, float w_rps);
  // Run at CONTROL_HZ; outputs PWM to motors
  void update(float dt);

  // For logging/diagnostics
  float v_sp() const { return v_sp_; }
  float w_sp() const { return w_sp_; }
  float vL_meas() const { return vL_meas_; }
  float vR_meas() const { return vR_meas_; }
  float pwmL() const { return pwmL_; }
  float pwmR() const { return pwmR_; }

private:
  Encoder *enc_ = nullptr;
  MotorDriver *mot_ = nullptr;

  PID pidL_, pidR_;

  float v_cmd_ = 0, w_cmd_ = 0; // commanded by /cmd_vel (after saturation)
  float v_sp_ = 0, w_sp_ = 0;   // shaped by accel limits
  float vL_sp_ = 0, vR_sp_ = 0;
  float vL_meas_ = 0, vR_meas_ = 0;
  float pwmL_ = 0, pwmR_ = 0;

  // Helper: convert ticks/sec -> m/s
  static inline float ticksToMps(float ticks_per_sec)
  {
    float rev_per_sec = ticks_per_sec / TICKS_PER_REV;
    float wheel_rad_s = rev_per_sec * 2.0f * PI; // rad/s
    return wheel_rad_s * WHEEL_RADIUS_M;         // v = ωR
  }
};
