#pragma once
#include <Arduino.h>

//DIR+ => GPIO
//PUL+ => GPIO
//ENA+ => GND
//ENA- => GND
//DIR- => GND
//PUL- => GND

//  Driver TB6600 (STEP/DIR)
constexpr int PIN_STEP  = 18;   //  STEP
constexpr int PIN_DIR   = 19;   //  DIR
constexpr int STEP_DELAY_US = 800; // หน่วงไมโครวินาทีต่อหนึ่งสเต็ป (ความเร็ว)

struct StepperProfile {
  int step_pin;
  int dir_pin;
  int step_delay_us;
};

// ตัวควบคุม Stepper
class UnifiedStepper {
public:
  UnifiedStepper(int stepPin, int dirPin, int stepDelayUs);
  bool begin();
  void stepCW(unsigned steps);      // หมุนตามเข็ม (Clockwise)
  void stepCCW(unsigned steps);     // หมุนทวนเข็ม (Counter-Clockwise)
  void rotateContinuous(bool cw);   // หมุนต่อเนื่อง (ต้องเรียกซ้ำใน loop)
  void stop();

  bool isContinuous() const { return continuous_; }
  bool directionCW() const  { return dirCW_; }

private:
  StepperProfile p_;
  bool continuous_ = false;
  bool dirCW_      = true;
  unsigned lastMicros_ = 0;
};

extern UnifiedStepper Nema17; 

// -------------------- ปุ่มควบคุม --------------------
using STP_handler_step = void(*)(bool down, UnifiedStepper& stepper);

struct STP_handlers_step {
  STP_handler_step onArrowUp   = nullptr; //  ทวนเข็ม
  STP_handler_step onArrowDown = nullptr; //  ตามเข็ม
};

// ตั้ง handler
void stepper_set_handlers(const STP_handlers_step& h);

// helper: case-insensitive startsWith
static inline bool _startsWith_ST(const String &s, const char *p);
// ประมวลผลสตริง "STP UP=DOWN DOWN=UP" (คล้าย servo)
bool stepper_handle_line(const String& line, UnifiedStepper& stepper);

// poll serial
void stepper_serial_poll(UnifiedStepper& stepper);

// callback
void onStpUp(bool down,   UnifiedStepper& stepper);
void onStpDown(bool down, UnifiedStepper& stepper);

// เรียกทุก loop เพื่อทำให้การหมุนต่อเนื่องยังเดินสเต็ป (ถ้ากำลัง continuous)
void stepper_tick(UnifiedStepper& stepper);

