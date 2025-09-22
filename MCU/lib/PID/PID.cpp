#include "PID.h"
void PID::setGains(float kp,float ki,float kd,float i_abs){ kp_=kp; ki_=ki; kd_=kd; i_abs_=i_abs; }
void PID::reset(){ integ_=0; prev_err_=0; first_=true; }

float PID::compute(float sp, float meas, float dt){
  float err = sp - meas;
  if(first_){ prev_err_ = err; first_ = false; }
  integ_ += err * dt;
  if(i_abs_ > 0) integ_ = constrain(integ_, -i_abs_, i_abs_);
  float deriv = (err - prev_err_) / max(dt, 1e-6f);
  prev_err_ = err;
  return kp_*err + ki_*integ_ + kd_*deriv;
}
