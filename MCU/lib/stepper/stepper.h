#pragma once
#include <Arduino.h>

/**
 * @brief Minimal TB6600 stepper driver helper (blocking moves, simple timing).
 *        - Works with "active LOW" or "active HIGH" pulse logic (see TB6600_ACTIVE_LOW).
 *        - Provides enable/disable, setDirection, stepOne, moveSteps, moveRev/deg/mm (if you know lead).
 *        - Designed for clarity; extend with acceleration profiles if needed.
 */

// ---------- Hardware Pins (ESP32) ----------
#define PIN_PUL   25  // TB6600 PUL- (or PUL if single-ended)
#define PIN_DIR   26  // TB6600 DIR- 
#define PIN_ENA   27  // TB6600 ENA-

// ---------- TB6600 Input Polarity ----------
// Many TB6600 boards are used with PUL+/DIR+/ENA+ tied to +5V and the MCU sinks on the "-" pins.
// In that wiring, the inputs are "active LOW" (a LOW on PUL- creates a pulse).
// Set this to true if you wire + pins to +5V and drive the - pins from the MCU (recommended).
// Set to false if you drive + pins directly from GPIO (active HIGH pulses).
#define TB6600_ACTIVE_LOW  true

// ---------- Step Signal Timings (microseconds) ----------
// Check your TB6600 datasheet; typical min high/low time is ~5us–10us.
#define STEP_PULSE_US    8     // pulse width
#define STEP_INTERVAL_US 500   // base interval between steps at "nominal speed" (lower = faster)

// ---------- Motor Parameters ----------
// Steps per revolution of motor (full-step). NEMA17 common: 200 steps/rev.
#define MOTOR_STEPS_PER_REV 200

// Microstepping (matches TB6600 DIP switches). For example: 1, 2, 4, 8, 16, 32
#define MICROSTEP 16

// Optional: direction invert if your wiring makes motion reversed
#define INVERT_DIRECTION false

class Stepper {
public:
  Stepper(int pinPUL, int pinDIR, int pinENA, bool activeLow);

  void begin();                 // pinModes + default states
  void enable();                // ENA active
  void disable();               // ENA inactive

  void setDirection(bool cw);   // cw=true -> clockwise (or "forward")
  void stepOne();               // one pulse (blocking)

  /**
   * @brief Move a given number of steps at a base interval (us).
   * @param steps     Signed step count (negative = reverse)
   * @param interval  Base interval between steps in microseconds (>= STEP_PULSE_US*2)
   */
  void moveSteps(long steps, unsigned long interval_us);

  /**
   * @brief Move by revolutions (rev). Positive = forward, negative = reverse.
   */
  void moveRevolutions(float rev, unsigned long interval_us);

  /**
   * @brief Move by degrees of motor shaft (0..360, or negative).
   */
  void moveDegrees(float deg, unsigned long interval_us);

  /**
   * @brief Convert revolutions/degree to microsteps (utility).
   */
  static long revToMicrosteps(float rev);
  static long degToMicrosteps(float deg);

private:
  int pinPUL_, pinDIR_, pinENA_;
  bool activeLow_;
  bool currentDirCW_;

  inline void pulse_();         // low/high or high/low depending on polarity
  inline void enaActive_();     // drive ENA active per polarity
  inline void enaInactive_();   // drive ENA inactive per polarity
  inline void dirLevel_(bool cw);
};
