#include <Arduino.h>
#include <math.h>

#include "servo.h"

// ===== Utility functions =====
UnifiedServo::UnifiedServo(ServoKind kind, int gpio, ServoProfile profile)
: kind_(kind), pin_(gpio), p_(profile) {}


// ===== Pulse profiles =====
//stop, max, min, Hz
ServoProfile TD8120MG_pf  {min_p_width, max_p_width, default_p_width, hz};
ServoProfile MG996R_pf    {min_p_width, max_p_width, default_p_width, hz};
ServoProfile MG996R_360_pf{min_p_width, max_p_width, default_p_width, hz};

 // Instances
UnifiedServo TD8120MG   (ServoKind::Continuous360,  PIN__TD8120MG, TD8120MG_pf);
UnifiedServo MG996R     (ServoKind::Positional180,  PIN_MG996R, MG996R_pf);
UnifiedServo MG996R_360 (ServoKind::Continuous360,  PIN_MG996R_360, MG996R_360_pf);


int channel_ = -1;
bool UnifiedServo::begin() {
  s_.setPeriodHertz(p_.us_hz);              // keep ~50 Hz for standard RC servos
  int channel_ = s_.attach(pin_, p_.us_min, p_.us_max);  // clamp range & reduce jitter
  return channel_;                        //channel => 0 - 15 , = 0 if fail
}
// -------- Common / shared controls --------

// Direct microsecond command (works for both 180° & 360°).
void UnifiedServo::setPulseUs(int us) {
  us = constrain(us, p_.us_min, p_.us_max);
  s_.writeMicroseconds(us);
}

// Go to center (180°) or stop (360°).
void UnifiedServo::goCenterOrStop() {
  setPulseUs(p_.us_center);
}

// Adjust center/neutral (useful for trimming 360° stop drift).
void UnifiedServo::nudgeCenterUs(int delta) {
  p_.us_center = constrain(p_.us_center + delta, p_.us_min, p_.us_max);
}

// bool UnifiedServo::attached(){
//   return s_.attached();
// }

// void UnifiedServo::detach() {
//   s_.detach();
// }

// -------- Type-specific helpers --------
// 180° only: set absolute angle [0..180] degrees.
void UnifiedServo::setAngleDeg(int deg) {
  if (kind_ != ServoKind::Positional180) return;  // ignore on 360°
  deg = constrain(deg, 0, 180);
  const int us = map(deg, 0, 180, p_.us_min, p_.us_max);
  setPulseUs(us);
}

// 360° only: set speed percentage [-100..+100]; neg=reverse, pos=forward.
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

// ---------- button receiver control ---------------

static BTN_handlers g_handlers;
static String g_rx_line;


void button_set_handlers(const BTN_handlers& h) {
  g_handlers = h;
}

// helper: case-insensitive startsWith
static inline bool _startsWith(const String &s, const char *p) {
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}

// helper: case-insensitive startsWith
static inline bool _parseTokenAfterEquals(const String &s, const char *key, String &out) {
  int idx = s.indexOf(key);
  if (idx < 0) return false;
  idx += strlen(key);
  int end = idx;
  while (end < (int)s.length() && !isWhitespace(s[end])) end++;
  out = s.substring(idx, end);
  out.trim();
  return out.length() > 0;
}

// handle a line like "BTN A=DOWN B=UP X=DOWN"
bool button_handle_line(const String& raw) {
  String line = raw;
  line.trim();
  if (line.isEmpty()) return false;
  if (!_startsWith(line, "BTN")) return false;
  // DEBUG: ดูบรรทัดที่เข้ามา
  Serial.printf("[ESP32] RX: %s\n", line.c_str());

  // รองรับ A=DOWN/UP หรือ A=1/0
  String tokA, tokB, tokX, tokY;
  bool hasA = _parseTokenAfterEquals(line, "A=", tokA);
  bool hasB = _parseTokenAfterEquals(line, "B=", tokB);
  bool hasX = _parseTokenAfterEquals(line, "X=", tokX);
  bool haxY = _parseTokenAfterEquals(line, "Y=", tokY);

  auto toDown = [](const String& t)->bool {
    return t.equalsIgnoreCase("DOWN") || t == "1";
  };

  //TD8120MG => testing | change to MG996R_360 for final
  if (hasA && g_handlers.onA) g_handlers.onA(toDown(tokA), MG996R_360); //360 | feed
  if (hasB && g_handlers.onB) g_handlers.onB(toDown(tokB), TD8120MG); //360
  if (hasX && g_handlers.onX) g_handlers.onX(toDown(tokX), MG996R); //180
  if (haxY && g_handlers.onY) g_handlers.onY(toDown(tokY), MG996R_360); //360 | feed

  // (จะพิมพ์ ACK ก็ได้)
  // Serial.printf("ACK %s\n", line.c_str());
  return true;
}

// poll from Serial
void button_serial_poll() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (g_rx_line.length() > 0) {
        button_handle_line(g_rx_line);
        g_rx_line = "";
        
      }
    } else {
      if (g_rx_line.length() < 240) g_rx_line += c;
    }
  }
}


// ---------- button control ---------------
void onBtnX(bool down, UnifiedServo& servoType) {
  if (servoType.kind() == ServoKind::Positional180) {
    if(down){
      Serial.println("To 180");
      servoType.setAngleDeg(180);
    }else {
      Serial.println("To 0");
      servoType.setAngleDeg(0);
    }
  }
}
void onBtnB(bool down,  UnifiedServo& servoType) {
  if (servoType.kind() == ServoKind::Continuous360) {
    if (down) {
      Serial.println("forward");
      servoType.setSpeedPercent(+50); // forward
    } else {
      Serial.println("stop");
      servoType.goCenterOrStop();     // stop
    }
  }
}
void onBtnY(bool down,  UnifiedServo& servoType) {
  if (servoType.kind() == ServoKind::Continuous360) {
    if (down) {
      Serial.println("forward");
      servoType.setSpeedPercent(+50); // forward
      delay(1500);
      Serial.println("Deployed 1 sapling");
    } else {
      Serial.println("stop");
      servoType.goCenterOrStop();     // stop
    }
  }
}

void onBtnA(bool down,  UnifiedServo& servoType) {
  if (servoType.kind() == ServoKind::Continuous360) {
    if (down) {
      Serial.println("reverse");
      servoType.setSpeedPercent(-50); // reverse
      delay(1500);
      Serial.println("reverse 1 sapling");
    } else {
      Serial.println("stop");
      servoType.goCenterOrStop();     // stop
    }
  }
}

