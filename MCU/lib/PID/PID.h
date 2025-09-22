#pragma once
#include <Arduino.h>

class PID {
public:
  void setGains(float kp, float ki, float kd, float i_abs);
  void reset();
  // Returns control output; anti-windup via integral clamping
  float compute(float sp, float meas, float dt);

private:
  float kp_=0, ki_=0, kd_=0, i_abs_=0;
  float integ_=0, prev_err_=0;
  bool  first_=true;
};
