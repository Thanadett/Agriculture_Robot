#include <Arduino.h>
#include <math.h>

#include "servo.h"

// ===== Utility functions =====


UnifiedServo::UnifiedServo(ServoKind kind, int gpio, ServoProfile profile)
: kind_(kind), pin_(gpio), p_(profile) {}

bool UnifiedServo::begin() {
  s_.setPeriodHertz(p_.us_hz);              // keep ~50 Hz for standard RC servos
  return s_.attach(pin_, p_.us_min, p_.us_max);  // clamp range & reduce jitter
}

// -------- Common / shared controls --------

void UnifiedServo::setPulseUs(int us) {
  us = constrain(us, p_.us_min, p_.us_max);
  s_.writeMicroseconds(us);
}

void UnifiedServo::goCenterOrStop() {
  setPulseUs(p_.us_center);
}

void UnifiedServo::nudgeCenterUs(int delta) {
  p_.us_center = constrain(p_.us_center + delta, p_.us_min, p_.us_max);
}

bool UnifiedServo::attached(){
  return s_.attached();
}

void UnifiedServo::detach() {
  s_.detach();
}

// -------- Type-specific helpers --------

void UnifiedServo::setAngleDeg(int deg) {
  if (kind_ != ServoKind::Positional180) return;  // ignore on 360°
  deg = constrain(deg, 0, 180);
  const int us = map(deg, 0, 180, p_.us_min, p_.us_max);
  setPulseUs(us);
}

void UnifiedServo::setSpeedPercent(int spd) {
  if (kind_ != ServoKind::Continuous360) return;  // ignore on 180°
  spd = constrain(spd, -100, 100);

  int us = p_.us_center;
  if (spd > 0) {
    us = map(spd, 0, 100, p_.us_center, p_.us_max);   // forward
  } else if (spd < 0) {
    us = map(spd, -100, 0, p_.us_min, p_.us_center);  // reverse
  }
  setPulseUs(us);
}


