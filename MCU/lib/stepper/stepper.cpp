// src/StepperDriver.cpp
#include "stepper.h"

/* ===== Constructor ===== */
Stepper::Stepper(int pinPUL, int pinDIR, int pinENA, bool activeLow)
: pinPUL_(pinPUL), pinDIR_(pinDIR), pinENA_(pinENA), activeLow_(activeLow), currentDirCW_(true) {}

/* ===== Init ===== */
void Stepper::begin() {
  pinMode(pinPUL_, OUTPUT);
  pinMode(pinDIR_, OUTPUT);
  pinMode(pinENA_, OUTPUT);

  // Default idle levels:
  // For active LOW inputs, idle HIGH on PUL keeps pulses "off".
  // For active HIGH inputs, idle LOW on PUL keeps pulses "off".
  digitalWrite(pinPUL_, activeLow_ ? HIGH : LOW);

  // Default direction
  dirLevel_(true);

  // Keep driver disabled on boot for safety
  enaInactive_();
}

/* ===== Enable/Disable ===== */
void Stepper::enable() {
  enaActive_();
}

void Stepper::disable() {
  enaInactive_();
}

/* ===== Direction ===== */
void Stepper::setDirection(bool cw) {
  // Optional global invert (from Config.h)
  bool dir = INVERT_DIRECTION ? !cw : cw;
  currentDirCW_ = dir;
  dirLevel_(dir);
}

/* ===== One Step Pulse (blocking) ===== */
void Stepper::stepOne() {
  // Generate one pulse with required width; polarity depends on wiring.
  if (activeLow_) {
    // Active LOW: pulse LOW then back HIGH
    digitalWrite(pinPUL_, LOW);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(pinPUL_, HIGH);
  } else {
    // Active HIGH: pulse HIGH then back LOW
    digitalWrite(pinPUL_, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(pinPUL_, LOW);
  }
}

/* ===== Move N steps (blocking) ===== */
void Stepper::moveSteps(long steps, unsigned long interval_us) {
  if (steps == 0) return;

  bool forward = (steps > 0);
  setDirection(forward);

  long count = labs(steps);
  // Ensure interval is not too small vs pulse width
  unsigned long minInterval = (STEP_PULSE_US * 2);
  if (interval_us < minInterval) interval_us = minInterval;

  for (long i = 0; i < count; ++i) {
    stepOne();
    // Wait remaining interval (already spent STEP_PULSE_US inside stepOne)
    delayMicroseconds(interval_us - STEP_PULSE_US);
  }
}

/* ===== Move by revolutions (blocking) ===== */
void Stepper::moveRevolutions(float rev, unsigned long interval_us) {
  long usteps = revToMicrosteps(rev);
  moveSteps(usteps, interval_us);
}

/* ===== Move by degrees (blocking) ===== */
void Stepper::moveDegrees(float deg, unsigned long interval_us) {
  long usteps = degToMicrosteps(deg);
  moveSteps(usteps, interval_us);
}

/* ===== Utilities ===== */
long Stepper::revToMicrosteps(float rev) {
  const long stepsPerRev = MOTOR_STEPS_PER_REV * MICROSTEP;
  return (long) llround(rev * (double)stepsPerRev);
}

long Stepper::degToMicrosteps(float deg) {
  const long stepsPerRev = MOTOR_STEPS_PER_REV * MICROSTEP;
  return (long) llround((deg / 360.0) * (double)stepsPerRev);
}

/* ===== Private Helpers ===== */
inline void Stepper::pulse_() {
  stepOne();
}

inline void Stepper::enaActive_() {
  // ENA "active" often means pulling ENA- LOW (if + is tied to +5V).
  // For activeLow_=true, drive LOW to activate; else HIGH.
  digitalWrite(pinENA_, activeLow_ ? LOW : HIGH);
}

inline void Stepper::enaInactive_() {
  digitalWrite(pinENA_, activeLow_ ? HIGH : LOW);
}

inline void Stepper::dirLevel_(bool cw) {
  // TB6600 samples DIR before pulse edges; keep a small setup time if you change DIR on the fly.
  digitalWrite(pinDIR_, (cw ^ activeLow_) ? HIGH : LOW);
  // Note: If motion direction looks inverted, flip INVERT_DIRECTION in Config.h
}
