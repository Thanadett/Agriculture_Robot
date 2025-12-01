#pragma once
#include <Arduino.h>
#include <config.h>
#include <ESP32Servo.h>
#include <math.h>


// ===== Hardware pins =====

// #define PIN_SERVO_TD8120MG 4
// #define PIN_SERVO_360 0 //MG996R 360 continuous rotation servo
// #define PIN_SERVO_180 2 //MG996R 180 standard servo
// PWM pins of ESP32 
constexpr int PIN__TD8120MG = 19;
constexpr int PIN_MG996R_360 = 17; //MG996R 360 continuous rotation servo
constexpr int PIN_MG996R = 27; //MG996R 180 standard servo
constexpr int PIN_SG90_180 = 14; //SG90 180 standard servo

const int min_p_width = 500; // the shortest pulse sent to a servo
const int max_p_width = 2500; // the longest pulse sent to a servo
const int default_p_width = 1500; // default pulse width
const int hz = 50; // Analog servos run at ~50 Hz updates

// --- NEW: analog trigger state (0..100) ---
static int g_LT_pct = 0;  // last LT percent (0..100)
static int g_RT_pct = 0;  // last RT percent (0..100)



struct ServoProfile {
  int us_min;
  int us_max;
  int us_center;    // for 180°: mechanical mid; for 360°: neutral stop
  int us_hz;
};

// decompose servo behaviors.
enum class ServoKind { Positional180, Continuous360 };

// ===== Utility functions =====

// แปลงองศา (0-180) เป็นไมโครวินาที ตามพารามิเตอร์โปรไฟล์
int angleDegToUs(int deg, const ServoProfile& p);

// แปลงเปอร์เซ็นต์ความเร็ว (-100 ถึง 100) เป็นไมโครวินาที ตามพารามิเตอร์โปรไฟล์
int speedPercentToUs(int spd, const ServoProfile& p);

// Unified interface used by both 180° and 360° servos.
class UnifiedServo {
public:
  // Construct with type, GPIO pin, and profile.
  UnifiedServo(ServoKind kind, int gpio, ServoProfile profile);

  // Must be called once before use. Returns false if attach fails.
  bool begin();

  // -------- Common / shared controls --------
  // Direct microsecond command (works for both 180° & 360°).
  void setPulseUs(int us);

  // Go to center (180°) or stop (360°).
  void goCenterOrStop();

  // Adjust center/neutral (useful for trimming 360° stop drift).
  void nudgeCenterUs(int delta);

  bool attached();
  void detach();

  // -------- Type-specific helpers --------
  // 180° only: set absolute angle [0..180] degrees.
  void setAngleDeg(int deg);

  // 360° only: set speed percentage [-100..+100]; neg=reverse, pos=forward.
  void setSpeedPercent(int spd);

  // Diagnostics
  ServoKind kind() const { return kind_; }
  int pin() const { return pin_; }

private:
  Servo      s_; //s_ => servo object
  ServoKind  kind_;
  int        pin_;
  ServoProfile p_;//p_ => pulse profile
};
// servo.h (ส่วนประกาศ)
  extern UnifiedServo TD8120MG;    // 360°
  extern UnifiedServo MG996R_360;  // 360°
  extern UnifiedServo MG996R;      // 180°
  extern UnifiedServo SG90_180;        // 180°
//----------------------------- button ----------------------------------------

typedef void (*BtnBoolFn)(bool /*down*/, UnifiedServo &);
typedef void (*BtnAnalogFn)(int /*0..100*/, UnifiedServo &);

// --- NEW: latched state for analog triggers ---
enum class ActiveSide { NONE, LT, RT };
static ActiveSide g_active_side = ActiveSide::NONE;
static int g_last_angle_deg = 0;  // เริ่มไว้กลาง (ปรับได้ตามต้องการ)
static const int kMinActivePct = 5;   // ≤3 ถือว่า "ปล่อย/สัญญาณรบกวน" -> เพิกเฉย
static const int kSnapHighPct = 97;   // ≥97 snap เป็น 100 (ตัวเลือก)


// callback: down = true/false, servo = target instance
using BTN_handler = void(*)(bool down, UnifiedServo& servo);

struct BTN_handlers {
  BtnBoolFn   onA = nullptr;
  BtnBoolFn   onB = nullptr;
  BtnBoolFn   onX = nullptr;
  BtnBoolFn   onY = nullptr;
  BtnBoolFn   onL = nullptr;
  BtnBoolFn   onR = nullptr;

  // --- NEW: สำหรับ LT/RT แบบแอนะล็อก ---
  BtnAnalogFn onLT = nullptr;   // รับค่า 0..100
  BtnAnalogFn onRT = nullptr;   // รับค่า 0..100
};

// helper: case-insensitive startsWith
static inline bool _startsWith(const String &s, const char *p);
// helper: parse token after "key="
static inline bool _parseTokenAfterEquals(const String &s, const char *key, String &out);

// set handler in setup()
void button_set_handlers(const BTN_handlers& h);

// manage 1 received line; turns true when done (prefix "BTN")
bool button_handle_line(const String& line);

// poll from Serial 
void button_serial_poll();

// ------------- button ctrl -------------
void onBtnA(bool down, UnifiedServo& servo);
void onBtnB(bool down, UnifiedServo& servo);
void onBtnX(bool down, UnifiedServo& servo);
void onBtnY(bool down, UnifiedServo& servo); 
void onBtnL(bool down, UnifiedServo& servo);
void onBtnR(bool down, UnifiedServo& servo);

// NEW prototypes
void onBtnLT(int percent, UnifiedServo& servo);
void onBtnRT(int percent, UnifiedServo& servo);
