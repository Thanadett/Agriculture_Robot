#include "stepper.h"

// ======================== UnifiedStepper ========================
UnifiedStepper::UnifiedStepper(int stepPin, int dirPin, int enaPin, int stepDelayUs)
    : p_{stepPin, dirPin, enaPin, stepDelayUs},
      stepper_(AccelStepper::DRIVER, stepPin, dirPin)
{
}

bool UnifiedStepper::begin()
{
  pinMode(p_.step_pin, OUTPUT);
  pinMode(p_.dir_pin, OUTPUT);
  pinMode(p_.ena_pin, OUTPUT);

  digitalWrite(p_.step_pin, LOW);
  digitalWrite(p_.dir_pin, LOW);

  // บอก AccelStepper ว่ามี ENA pin
  stepper_.setEnablePin(p_.ena_pin);

  // *** ลำดับอาร์กิวเมนต์ AccelStepper: (directionInvert, stepInvert, enableInvert) ***
  stepper_.setPinsInverted(DIRECTION_INVERT, STEP_INVERT, ENABLE_INVERT);

  // Pulse width ของ TB6600 ปลอดภัยตั้งไว้ 10us
  stepper_.setMinPulseWidth(10);

  // ค่าตั้งต้นความเร็ว/เร่ง (คำนวณหยาบจาก stepDelayUs)
  const float default_speed_sps = 1.e6f / (0.75f * (float)p_.step_delay_us);
  const float default_accel_sps2 = default_speed_sps * 2.0f;

  stepper_.setMaxSpeed(default_speed_sps * 0.8f); // กันสั่น/กระตุก
  stepper_.setAcceleration(default_accel_sps2);

  // เปิดเอาต์พุต (จะเขียนระดับถูกต้องตาม ENABLE_INVERT ให้เอง)
  stepper_.enableOutputs();
  stepper_.setCurrentPosition(0);
  return true;
}

void UnifiedStepper::moveSteps(long steps)
{
  continuous_ = false;
  stepper_.move(steps); // ไม่บล็อก → ไปเรื่อย ๆ ใน tick()
}

void UnifiedStepper::rotateContinuous(bool cw)
{
  continuous_ = true;
  // ใช้ค่า maxSpeed ที่ตั้งไว้
  const float sps = stepper_.maxSpeed();
  stepper_.setSpeed(cw ? +sps : -sps); // runSpeed() จะใช้ค่านี้
}

void UnifiedStepper::stop()
{
  continuous_ = false;
  stepper_.stop();      // สำหรับโหมด run()
  stepper_.setSpeed(0); // สำหรับโหมด runSpeed()
}

void UnifiedStepper::tick()
{
  if (continuous_)
    stepper_.runSpeed();
  else
    stepper_.run();
}

void UnifiedStepper::setMaxSpeed(float sps)
{
  stepper_.setMaxSpeed(sps);
}

void UnifiedStepper::setAcceleration(float sps2)
{
  stepper_.setAcceleration(sps2);
}

long UnifiedStepper::currentPosition()
{
  return stepper_.currentPosition();
}

void UnifiedStepper::setCurrentPosition(long pos)
{
  stepper_.setCurrentPosition(pos);
}

// ถ้าอยากคุม ENA เอง (ปกติไม่จำเป็นเพราะใช้ enableOutputs()/disableOutputs() แล้ว)
inline void UnifiedStepper::ena_enable_logic()
{
#if ENABLE_INVERT
  digitalWrite(p_.ena_pin, LOW);
#else
  digitalWrite(p_.ena_pin, HIGH);
#endif
}
inline void UnifiedStepper::ena_disable_logic()
{
#if ENABLE_INVERT
  digitalWrite(p_.ena_pin, HIGH);
#else
  digitalWrite(p_.ena_pin, LOW);
#endif
}

// ================== Serial Command Parser (Optional) ==================
static bool _startsWithNoCase(const String &s, const char *p)
{
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}
static inline bool _parseTokenAfterEquals(const String &s, const char *key, String &out)
{
  int idx = s.indexOf(key);
  if (idx < 0)
    return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end]))
    end++;
  out = s.substring(idx, end);
  out.trim();
  return out.length() > 0;
}

bool stepper_handle_line(const String &raw, UnifiedStepper &stepper)
{
  String line = raw;
  line.trim();
  if (line.isEmpty())
    return false;
  if (!_startsWithNoCase(line, "STP"))
    return false;

  // ตัวอย่างคำสั่ง: "STP C_Up=DOWN" / "STP C_Up=UP" / "STP C_Dn=DOWN" / "STP C_Dn=UP"
  String tokUp, tokDn;
  bool hasUp = _parseTokenAfterEquals(line, "C_Up=", tokUp);
  bool hasDown = _parseTokenAfterEquals(line, "C_Dn=", tokDn);

  auto toDown = [](const String &t) -> bool
  {
    return t.equalsIgnoreCase("DOWN") || t == "1";
  };

  static bool prevUp = false, prevDn = false;

  if (hasUp)
  {
    bool nowUp = toDown(tokUp);
    if (nowUp != prevUp)
    {
      if (nowUp)
      { // กดปุ่ม "ขึ้น" → ทวนเข็ม
        stepper.rotateContinuous(false);
      }
      else
      { // ปล่อย → หยุด
        stepper.stop();
      }
      prevUp = nowUp;
    }
  }

  if (hasDown)
  {
    bool nowDn = toDown(tokDn);
    if (nowDn != prevDn)
    {
      if (nowDn)
      { // กดปุ่ม "ลง" → ตามเข็ม
        stepper.rotateContinuous(true);
      }
      else
      { // ปล่อย → หยุด
        stepper.stop();
      }
      prevDn = nowDn;
    }
  }

  return true;
}
