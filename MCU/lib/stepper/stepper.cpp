#include <Arduino.h>
#include "stepper.h"

UnifiedStepper::UnifiedStepper(int stepPin, int dirPin, int enaPin, int stepDelayUs)
: p_{stepPin, dirPin, enaPin, stepDelayUs} {}


UnifiedStepper Nema17(PIN_STEP, PIN_DIR, PIN_ENABLE, STEP_DELAY_US);


bool UnifiedStepper::begin() {
  pinMode(p_.step_pin, OUTPUT);
  pinMode(p_.dir_pin, OUTPUT);
  digitalWrite(p_.step_pin, LOW);
  digitalWrite(p_.ena_Pin, LOW);

  //Enable pin (active-low)
  pinMode(p_.ena_Pin, OUTPUT);
  digitalWrite(p_.ena_Pin, HIGH); // HIGH = Disable
  return true;
}


void UnifiedStepper::stepCW(unsigned steps) {
  // >>> ADD: Enable driver (active-low)
  digitalWrite(p_.ena_Pin, LOW);

  digitalWrite(p_.dir_pin, HIGH);
  for (unsigned i = 0; i < steps; i++) {
    digitalWrite(p_.step_pin, HIGH);
    delayMicroseconds(p_.step_delay_us);
    digitalWrite(p_.step_pin, LOW);
    delayMicroseconds(p_.step_delay_us);
  }

  digitalWrite(p_.ena_Pin, HIGH);
}


void UnifiedStepper::stepCCW(unsigned steps) {
  digitalWrite(p_.ena_Pin, LOW);

  digitalWrite(p_.dir_pin, LOW);
  for (unsigned i = 0; i < steps; i++) {
    digitalWrite(p_.step_pin, HIGH);
    delayMicroseconds(p_.step_delay_us);
    digitalWrite(p_.step_pin, LOW);
    delayMicroseconds(p_.step_delay_us);
  }

  digitalWrite(p_.ena_Pin, HIGH);
}


void UnifiedStepper::rotateContinuous(bool cw) {
  //Ensure driver is enabled while continuous running
  digitalWrite(p_.ena_Pin, LOW); // active-low enable
  continuous_ = true;
  dirCW_ = cw;
  unsigned now = micros();
  if (now - lastMicros_ >= (unsigned)p_.step_delay_us*2) {
    lastMicros_ = now;
    digitalWrite(p_.dir_pin, cw ? HIGH : LOW);
    digitalWrite(p_.step_pin, !digitalRead(p_.step_pin));
  }
}

void UnifiedStepper::stop() {
  continuous_ = false;
  digitalWrite(p_.step_pin, LOW);
  //Disable driver
  digitalWrite(PIN_ENABLE, HIGH); // active-low -> HIGH = Disable
}

// ----------------- ปุ่ม -----------------
static STP_handlers_step g_handlers_step;
static String g_rx_line_step;

void stepper_set_handlers(const STP_handlers_step& h) { g_handlers_step = h; }

static inline bool _parseTokenAfterEquals(const String &s,const char *key,String &out){
  int idx=s.indexOf(key); if(idx<0) return false;
  idx+=strlen(key);
  int end=idx; while(end<(int)s.length() && !isWhitespace(s[end])) end++;
  out=s.substring(idx,end); out.trim();
  return out.length()>0;
}
// helper: case-insensitive startsWith
static inline bool _startsWith_ST(const String &s, const char *p) {
  size_t n = strlen(p);
  return s.length() >= (int)n && s.substring(0, n).equalsIgnoreCase(p);
}

bool stepper_handle_line(const String& raw, UnifiedStepper& stepper) {
  String line = raw; line.trim();
  line.trim();
  if (line.isEmpty()) return false;
  if (!_startsWith_ST(line, "STP")) return false;

  Serial.printf("[ESP32] RX: %s\n", line.c_str());
  String tokUp, tokDown;
  bool hasUp   = _parseTokenAfterEquals(line,"C_Up=",tokUp);
  bool hasDown = _parseTokenAfterEquals(line,"C_Dn=",tokDown);

  auto toDown = [](const String& t)->bool {
    return t.equalsIgnoreCase("DOWN") || t=="1";
  };

  if (hasUp   && g_handlers_step.onArrowUp)
    g_handlers_step.onArrowUp(toDown(tokUp), stepper);
  if (hasDown && g_handlers_step.onArrowDown)
    g_handlers_step.onArrowDown(toDown(tokDown), stepper);
  return true;
}


void stepper_tick(UnifiedStepper& stepper) {
  if (stepper.isContinuous()) {
    stepper.rotateContinuous(stepper.directionCW());
  }
}

// void stepper_serial_poll(UnifiedStepper& stepper) {
//   while (Serial.available() > 0) {
//     char c = (char)Serial.read();
//     if (c=='\r' || c=='\n') {
//       if (g_rx_line_step.length() > 0) {
//         stepper_handle_line(g_rx_line_step, stepper);
//         g_rx_line_step="";
//       }
//     } else if (g_rx_line_step.length() < 200) g_rx_line_step += c;
//   }

//   // หากกำลังหมุนต่อเนื่อง ให้เรียก step ต่อ
//   if (stepper.isContinuous())
//     stepper.rotateContinuous(stepper.directionCW());
// }

// callbacks
void onStpUp(bool down, UnifiedStepper& stepper) {
  if (down) {
    Serial.println("Stepper CCW start");
    stepper.rotateContinuous(false); // ทวนเข็ม
  } else {
    Serial.println("Stepper stop");
    stepper.stop();
  }
}

void onStpDown(bool down, UnifiedStepper& stepper) {
  if (down) {
    Serial.println("Stepper CW start");
    stepper.rotateContinuous(true); // ตามเข็ม
  } else {
    Serial.println("Stepper stop");
    stepper.stop();
  }
}
