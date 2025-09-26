#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

// ================== Wiring Mode ==================
// Active-Low (แนะนำ): PUL+/DIR+/ENA+ -> 3.3V, PUL-/DIR-/ENA- -> GPIO
//   - LOW = Enable, HIGH = Disable
//   - ตั้ง ENABLE_INVERT = true
//
// Active-High: PUL+/DIR+/ENA+ -> GPIO, PUL-/DIR-/ENA- -> GND
//   - HIGH = Enable, LOW = Disable
//   - ตั้ง ENABLE_INVERT = false
//
// เปลี่ยนค่าตรงนี้อย่างเดียวพอ
#ifndef ENABLE_INVERT
#define ENABLE_INVERT true // true = Active-Low, false = Active-High
#endif

#ifndef DIRECTION_INVERT
#define DIRECTION_INVERT false // true ถ้าทิศทางกลับด้าน
#endif

#ifndef STEP_INVERT
#define STEP_INVERT false // ส่วนใหญ่ไม่ต้องกลับขั้น
#endif

struct STP_Params
{
  int step_pin;
  int dir_pin;
  int ena_pin;
  int step_delay_us; // ค่าคุม speed/accel เบื้องต้น (ยิ่งน้อย = เร็ว)
};

class UnifiedStepper
{
public:
  UnifiedStepper(int stepPin, int dirPin, int enaPin, int stepDelayUs);

  // ต้องเรียกใน setup()
  bool begin();

  // โหมด "ตำแหน่ง" (ไม่บล็อก) ใช้ร่วมกับ tick()
  void moveSteps(long steps); // + ตามเข็ม, - ทวนเข็ม

  // โหมด "ความเร็วคงที่"
  void rotateContinuous(bool cw); // true=CW, false=CCW
  void stop();                    // หยุดทุกโหมด

  // ต้องเรียกใน loop เสมอ
  void tick();

  // ปรับค่าความเร็ว/เร่ง (steps/s, steps/s^2)
  void setMaxSpeed(float sps);
  void setAcceleration(float sps2);

  // ยูทิลิตี้เล็กน้อย
  long currentPosition();
  void setCurrentPosition(long pos);

private:
  // ใช้เฉพาะภายใน (เผื่อกรณีอยากบังคับเอง)
  inline void ena_enable_logic();  // เปิด (ตาม ENABLE_INVERT)
  inline void ena_disable_logic(); // ปิด (ตาม ENABLE_INVERT)

  STP_Params p_;
  AccelStepper stepper_;
  bool continuous_ = false;
};

// ---------------------- Optional: Serial Command Parser ----------------------
// โปรโตคอลเรียบง่าย: "STP C_Up=DOWN" / "STP C_Up=UP" / "STP C_Dn=DOWN" / "STP C_Dn=UP"
// ใช้กับปุ่มขึ้น/ลง เพื่อหมุน CCW/CW ขณะกดค้าง และปล่อยแล้วหยุด
// ใช้: stepper_handle_line(line, myStepper);
bool stepper_handle_line(const String &raw, UnifiedStepper &stepper);
