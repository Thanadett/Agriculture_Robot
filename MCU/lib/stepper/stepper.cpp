#include <Arduino.h>
#include "stepper.h"

//accel stepper
UnifiedStepper::UnifiedStepper(int stepPin, int dirPin, int enaPin, int stepDelayUs)
: p_{stepPin, dirPin, enaPin, stepDelayUs} {}


UnifiedStepper Nema17(PIN_STEP, PIN_DIR, PIN_ENABLE, STEP_DELAY_US);


bool UnifiedStepper::begin() {
  pinMode(p_.step_pin, OUTPUT);
  pinMode(p_.dir_pin, OUTPUT);
  pinMode(p_.ena_Pin, OUTPUT);

  digitalWrite(p_.step_pin, HIGH); // idle high
  digitalWrite(p_.dir_pin,  LOW);
  digitalWrite(p_.ena_Pin,  HIGH);
  ledcInit(0, p_.step_pin, ledc_bits_, 1000); // channel 0, start 1kHz
  return true;
}

//manual step CW
// void UnifiedStepper::stepCW(unsigned steps) {
//   digitalWrite(p_.ena_Pin, LOW);
//   delayMicroseconds(200);  // wait for enable to take effect
//   digitalWrite(p_.dir_pin, HIGH);
//   delayMicroseconds(20); // wait for direction to take effect  
//   for (unsigned i = 0; i < steps; i++) {
//     digitalWrite(p_.step_pin, LOW);
//     delayMicroseconds(p_.step_delay_us);
//     digitalWrite(p_.step_pin, HIGH);
//     delayMicroseconds(p_.step_delay_us);
//   }

//   digitalWrite(p_.ena_Pin, HIGH);
// }

// //manual step CCW
// void UnifiedStepper::stepCCW(unsigned steps) {
//   digitalWrite(p_.ena_Pin, LOW);
//   delayMicroseconds(200);
//   digitalWrite(p_.dir_pin, LOW);
//   delayMicroseconds(20);
//   for (unsigned i = 0; i < steps; i++) {
//     digitalWrite(p_.step_pin, LOW);
//     delayMicroseconds(p_.step_delay_us);
//     digitalWrite(p_.step_pin, HIGH);
//     delayMicroseconds(p_.step_delay_us);
//   }

//   digitalWrite(p_.ena_Pin, HIGH);
// }


// void UnifiedStepper::rotateContinuous(bool cw) {
//   //Ensure driver is enabled while continuous running
//   digitalWrite(p_.ena_Pin, LOW); // active-low enable
//   continuous_ = true;
//   dirCW_ = cw;
//   if(dirCW_ == true){
//         digitalWrite(p_.dir_pin, HIGH);
//       }
//   if(dirCW_ == false){
//       digitalWrite(p_.dir_pin, LOW);
//     }

//   unsigned now = micros();
//   if (now - lastMicros_ >= (unsigned)p_.step_delay_us*2) {
//     lastMicros_ = now;
  
//     digitalWrite(p_.step_pin, LOW);
//     delayMicroseconds(p_.step_delay_us);
//     digitalWrite(p_.step_pin, HIGH);
//   }
// }
// void UnifiedStepper::rotateCon_CW() {
//   //Ensure driver is enabled while continuous running
//   digitalWrite(p_.ena_Pin, LOW); // active-low enable
//   continuous_ = true;
    
//   unsigned now = micros();
//   if (now - lastMicros_ >= (unsigned)p_.step_delay_us*2) {
//     lastMicros_ = now;
//     digitalWrite(p_.dir_pin, HIGH);
//     digitalWrite(p_.step_pin, LOW);
//     delayMicroseconds(p_.step_delay_us);
//     digitalWrite(p_.step_pin, HIGH);
//   }
// }

// void UnifiedStepper::rotateCon_CCW() {
//   //Ensure driver is enabled while continuous running
//   digitalWrite(p_.ena_Pin, LOW); // active-low enable
//   continuous_ = true;
    
//   unsigned now = micros();
//   if (now - lastMicros_ >= (unsigned)p_.step_delay_us*2) {
//     lastMicros_ = now;
//     digitalWrite(p_.dir_pin, LOW);
//     digitalWrite(p_.step_pin, LOW);
//     delayMicroseconds(p_.step_delay_us);
//     digitalWrite(p_.step_pin, HIGH);
//   }
// }

//=================== LEDC CTRL ==================================================
int channel = 0;
int resolution = 10;   // 10-bit
int freq = 2000;   
void ledcInit(int channel, int step_pin, int resolution_bits, int start_freq) {
  ledcSetup(channel, start_freq, resolution_bits);
  ledcAttachPin(step_pin, channel);

  int maxDuty = (1 << resolution_bits) - 1;
  // idle: HIGH (duty = max)
  ledcWrite(channel, maxDuty);
}

//create pulse, hz, duty 50%
void ledcStart(int channel, int resolution_bits, float freq_hz) {
  if (freq_hz <= 0) freq_hz = 1000.0f;
  ledcSetup(channel, freq_hz, resolution_bits);

  int maxDuty = (1 << resolution_bits) - 1;
  int duty50 = maxDuty / 2;
  ledcWrite(channel, duty50);
}

// stop pulse: stay HIGH 
void ledcStop(int channel, int resolution_bits, bool idle_high = true) {
  int maxDuty = (1 << resolution_bits) - 1;
  ledcWrite(channel, idle_high ? maxDuty : 0);
}
// ---------------- Manual step  ----------------
void UnifiedStepper::stepCW(unsigned steps) {
  //ENA on (active-low)
  digitalWrite(p_.ena_Pin, LOW);
  delayMicroseconds(200);

  digitalWrite(p_.dir_pin, HIGH);
  delayMicroseconds(10);

  // create pulse N time
  for (unsigned i = 0; i < steps; i++) {
    digitalWrite(p_.step_pin, LOW);
    delayMicroseconds(p_.step_delay_us);
    digitalWrite(p_.step_pin, HIGH);
    delayMicroseconds(p_.step_delay_us);
  }

  // ปิด ENA ถ้าไม่ต้องการถือแรง (ถ้าจะวิ่งต่อ เอา HIGH ออก)
  digitalWrite(p_.ena_Pin, HIGH);
}

void UnifiedStepper::stepCCW(unsigned steps) {
  digitalWrite(p_.ena_Pin, LOW);
  delayMicroseconds(200);

  digitalWrite(p_.dir_pin, LOW);
  delayMicroseconds(10);

  for (unsigned i = 0; i < steps; i++) {
    digitalWrite(p_.step_pin, LOW);
    delayMicroseconds(p_.step_delay_us);
    digitalWrite(p_.step_pin, HIGH);
    delayMicroseconds(p_.step_delay_us);
  }

  digitalWrite(p_.ena_Pin, HIGH);
}

// ---------------- Continuous (LEDC PWM) ----------------
void UnifiedStepper::startContinuous(bool cw, float freq_hz) {
  digitalWrite(p_.ena_Pin, LOW);
  delayMicroseconds(100);
  // select ccw, cw
  digitalWrite(p_.dir_pin, cw ? HIGH : LOW);
  delayMicroseconds(10);
  // start PWM at STEP (50% duty)
  ledcStart(0, ledc_bits_, freq_hz);

  continuous_ = true;
  dirCW_ = cw;
}

void UnifiedStepper::stop_ledc() {
  continuous_ = false;
  // หยุด PWM แล้วค้าง HIGH เป็น idle
  ledcStop(0, ledc_bits_, true);
  // ปิดไดรเวอร์ (active-low)
  digitalWrite(p_.ena_Pin, HIGH);
}
//=================== LEDC CTRL (END) ==================================================


// void UnifiedStepper::stop() {
//   continuous_ = false;
//   digitalWrite(p_.step_pin, LOW);
//   //Disable driver
//   digitalWrite(p_.ena_Pin, HIGH); 
// }

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


// void stepper_tick(UnifiedStepper& stepper) {
//   if (stepper.isContinuous()) {
//     stepper.rotateContinuous(stepper.directionCW());
//   }
// }

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
    // stepper.rotateContinuous(false);
    // stepper.rotateCon_CCW();
    stepper.startContinuous(false, 2000);
    delay(100);
  } else {
    Serial.println("Stepper stop");
    stepper.stop_ledc();
  }
}

void onStpDown(bool down, UnifiedStepper& stepper) {
  if (down) {
    Serial.println("Stepper CW start");
    // stepper.rotateContinuous(true);
    // stepper.rotateCon_CW();
    stepper.startContinuous(true, 2000);
    delay(100);
  } else {
    Serial.println("Stepper stop");
    stepper.stop_ledc();
  }
}
