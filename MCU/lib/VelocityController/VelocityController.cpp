#include "VelocityController.h"
#include "Wheel.h"

void VelocityController::begin(Encoder *e, MotorDriver *m)
{
  enc_ = e;
  mot_ = m;
  pidL_.setGains(WH_KP, WH_KI, WH_KD, WH_I_ABS);
  pidR_.setGains(WH_KP, WH_KI, WH_KD, WH_I_ABS);
}

void VelocityController::setCmdVel(float v_mps, float w_rps)
{
  // Saturate to limits
  v_cmd_ = constrain(v_mps, -V_MAX_MPS, V_MAX_MPS);
  w_cmd_ = constrain(w_rps, -W_MAX_RPS, W_MAX_RPS);
}

void VelocityController::update(float dt)
{
  if (!enc_ || !mot_)
    return;

  // 1) Acc-limited shaping on v_sp_, w_sp_
  float dv_max = ACC_V_MAX * dt;
  float dw_max = ACC_W_MAX * dt;
  v_sp_ += constrain(v_cmd_ - v_sp_, -dv_max, dv_max);
  w_sp_ += constrain(w_cmd_ - w_sp_, -dw_max, dw_max);

  // 2) Differential kinematics → left/right wheel linear speeds
  // vL = v - (w * track/2), vR = v + (w * track/2)
  vL_sp_ = v_sp_ - 0.5f * w_sp_ * TRACK_WIDTH_M;
  vR_sp_ = v_sp_ + 0.5f * w_sp_ * TRACK_WIDTH_M;

  // 3) Measure wheel speeds from encoder delta ticks
  float ticksL = float(enc_->deltaTicks(Wheel::FL) + enc_->deltaTicks(Wheel::RL)) * 0.5f;
  float ticksR = float(enc_->deltaTicks(Wheel::FR) + enc_->deltaTicks(Wheel::RR)) * 0.5f;
  float tpsL = ticksL / dt;
  float tpsR = ticksR / dt;
  vL_meas_ = ticksToMps(tpsL);
  vR_meas_ = ticksToMps(tpsR);

  // 4) PID per side → PWM in m/s domain → scale to [-1..1] by V_MAX_MPS
  float uL = pidL_.compute(vL_sp_, vL_meas_, dt);
  float uR = pidR_.compute(vR_sp_, vR_meas_, dt);
  pwmL_ = constrain(uL / V_MAX_MPS, -1.f, 1.f);
  pwmR_ = constrain(uR / V_MAX_MPS, -1.f, 1.f);

  // 5) Drive motors (map to wheels with correct sign)
  mot_->setPWM(Wheel::FL, pwmL_);
  mot_->setPWM(Wheel::RL, pwmL_);
  mot_->setPWM(Wheel::FR, pwmR_);
  mot_->setPWM(Wheel::RR, pwmR_);
}
