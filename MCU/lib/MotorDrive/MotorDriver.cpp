#include "MotorDriver.h"

static inline void setupChan(int chan, int pin)
{
  ledcSetup(chan, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(pin, chan);
  ledcWrite(chan, 0);
}

void MotorDriver::begin()
{
  // Configure all channels/pins
  pinMode(FL.inL, OUTPUT);
  pinMode(FL.inR, OUTPUT);
  pinMode(RL.inL, OUTPUT);
  pinMode(RL.inR, OUTPUT);
  pinMode(FR.inL, OUTPUT);
  pinMode(FR.inR, OUTPUT);
  pinMode(RR.inL, OUTPUT);
  pinMode(RR.inR, OUTPUT);
  setupChan(FL.chanL, FL.inL);
  setupChan(FL.chanR, FL.inR);
  setupChan(RL.chanL, RL.inL);
  setupChan(RL.chanR, RL.inR);
  setupChan(FR.chanL, FR.inL);
  setupChan(FR.chanR, FR.inR);
  setupChan(RR.chanL, RR.inL);
  setupChan(RR.chanR, RR.inR);
}

void MotorDriver::enable(bool en)
{
  enabled_ = en;
  if (!en)
  {
    ledcWrite(FL.chanL, 0);
    ledcWrite(FL.chanR, 0);
    ledcWrite(RL.chanL, 0);
    ledcWrite(RL.chanR, 0);
    ledcWrite(FR.chanL, 0);
    ledcWrite(FR.chanR, 0);
    ledcWrite(RR.chanL, 0);
    ledcWrite(RR.chanR, 0);
  }
}

void MotorDriver::applyPWM(const WheelPins &wp, float pwm01)
{
  // signed command; positive drives IN_L, negative drives IN_R (per polarity doc)
  pwm01 = constrain(pwm01, -1.0f, 1.0f);
  int duty = int(fabs(pwm01) * PWM_MAX);
  if (!enabled_)
    duty = 0;

  if (pwm01 * wp.dir_sign >= 0)
  {
    ledcWrite(wp.chanL, duty);
    ledcWrite(wp.chanR, 0);
  }
  else
  {
    ledcWrite(wp.chanL, 0);
    ledcWrite(wp.chanR, duty);
  }
}

void MotorDriver::setPWM(Wheel w, float pwm01)
{
  switch (w)
  {
  case Wheel::FL:
    applyPWM(FL, pwm01);
    break;
  case Wheel::RL:
    applyPWM(RL, pwm01);
    break;
  case Wheel::FR:
    applyPWM(FR, pwm01);
    break;
  case Wheel::RR:
    applyPWM(RR, pwm01);
    break;
  }
}
