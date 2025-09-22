#include <Arduino.h>
#include "stepper.h"

UnifiedStepper::UnifiedStepper(int stepPin, int dirPin, int stepDelayUs)
: p_{stepPin, dirPin, stepDelayUs} {}


UnifiedStepper Nema17(PIN_STEP, PIN_DIR, STEP_DELAY_US);


bool UnifiedStepper::begin() {
  pinMode(p_.step_pin, OUTPUT);
  pinMode(p_.dir_pin, OUTPUT);
  digitalWrite(p_.step_pin, LOW);
  digitalWrite(p_.dir_pin, LOW);
  return true;
}

void UnifiedStepper::stepCW(unsigned steps) {
  digitalWrite(p_.dir_pin, HIGH);
  for (unsigned i = 0; i < steps; i++) {
    digitalWrite(p_.step_pin, HIGH);
    delayMicroseconds(p_.step_delay_us);
    digitalWrite(p_.step_pin, LOW);
    delayMicroseconds(p_.step_delay_us);
  }
}

void UnifiedStepper::stepCCW(unsigned steps) {
  digitalWrite(p_.dir_pin, LOW);
  for (unsigned i = 0; i < steps; i++) {
    digitalWrite(p_.step_pin, HIGH);
    delayMicroseconds(p_.step_delay_us);
    digitalWrite(p_.step_pin, LOW);
    delayMicroseconds(p_.step_delay_us);
  }
}

void UnifiedStepper::rotateContinuous(bool cw) {
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

bool stepper_handle_line(const String& raw, UnifiedStepper& stepper) {
  String line = raw; line.trim();
  if (!line.startsWith("STP")) return false;
  String tokUp, tokDown;
  bool hasUp   = _parseTokenAfterEquals(line,"UP=",tokUp);
  bool hasDown = _parseTokenAfterEquals(line,"DOWN=",tokDown);

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
