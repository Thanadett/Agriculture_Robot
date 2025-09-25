#include <Arduino.h>
#include "stepper.h"

// Helper for ENA pin
// Assuming active-low enable
#ifndef ENABLE_ACTIVE_LOW
#define ENABLE_ACTIVE_LOW 1
#endif

inline void UnifiedStepper::ena_enable() {
#if ENABLE_ACTIVE_LOW
  digitalWrite(p_.ena_Pin, LOW);
#else
  digitalWrite(p_.ena_Pin, HIGH);
#endif
}
inline void UnifiedStepper::ena_disable() {
#if ENABLE_ACTIVE_LOW
  digitalWrite(p_.ena_Pin, HIGH);
#else
  digitalWrite(p_.ena_Pin, LOW);
#endif
}

//accel stepper
UnifiedStepper::UnifiedStepper(int stepPin, int dirPin, int enaPin, int stepDelayUs)
: p_{stepPin, dirPin, enaPin, stepDelayUs},
  stepper_(AccelStepper::DRIVER, stepPin, dirPin)  
  {//1e6 = 1 second in microseconds
    default_speed_sps_  = 1.e6f / (0.75f * (float)stepDelayUs);  
    default_accel_sps2_ = default_speed_sps_ * 2.5f;   
  }


UnifiedStepper Nema17(PIN_STEP, PIN_DIR, PIN_ENABLE, STEP_DELAY_US);


bool UnifiedStepper::begin() {
  pinMode(p_.step_pin, OUTPUT);
  pinMode(p_.dir_pin, OUTPUT);
  pinMode(p_.ena_Pin, OUTPUT);

  digitalWrite(p_.step_pin, LOW); // idle 
  digitalWrite(p_.dir_pin,  LOW);
  // digitalWrite(p_.ena_Pin,  HIGH);
  stepper_.setPinsInverted(
    false /*step*/, 
    true /*dir*/, 
    false /*enable*/); 
  stepper_.enableOutputs();                                 

  stepper_.setMinPulseWidth(6); // microseconds (us) 
  stepper_.setMaxSpeed(default_speed_sps_);      
  stepper_.setAcceleration(default_accel_sps2_); 
  stepper_.setCurrentPosition(0); 

  return true;
}

// manual step CW
void UnifiedStepper::stepCW(unsigned steps) {
  // ena_enable();
  delayMicroseconds(100);
  // digitalWrite(p_.dir_pin, HIGH);
  stepper_.moveTo(stepper_.currentPosition() + (long)steps);
  stepper_.runToPosition();                      
  // ena_disable();
}

//manual step CCW
void UnifiedStepper::stepCCW(unsigned steps) {
  // ena_enable();
  delayMicroseconds(100);
  // digitalWrite(p_.dir_pin, LOW);
  stepper_.moveTo(stepper_.currentPosition() - (long)steps);
  stepper_.runToPosition();                        // ★
  // ena_disable();
}

void UnifiedStepper::rotateContinuous(bool cw) {
  //  ena_enable();
  continuous_ = true;
  delayMicroseconds(50);

  dirCW_ = cw;
  float sps = default_speed_sps_;

  stepper_.setMaxSpeed(sps);
  if(dirCW_ == true){
    stepper_.setSpeed(+sps);
  }
  else if (dirCW_ == false){
    stepper_.setSpeed(-sps);
  } 
}


void UnifiedStepper::stop() {
  continuous_ = false;
  stepper_.stop();          // stop when used  run(), accel
  stepper_.setSpeed(0);     // stop runSpeed() if used
  // ena_disable();
}

//call in loop
void UnifiedStepper::tick() {                     
  if (continuous_) stepper_.runSpeed();  // constant vel no accel
  else             stepper_.run();      // accel to target pos
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


static bool prevUp = false, prevDn = false; 
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

  // if (hasUp   && g_handlers_step.onArrowUp)
  //   g_handlers_step.onArrowUp(toDown(tokUp), stepper);
  // if (hasDown && g_handlers_step.onArrowDown)
  //   g_handlers_step.onArrowDown(toDown(tokDown), stepper);
  bool up = hasUp && toDown(tokUp);
  bool dn = hasDown && toDown(tokDown);

  // separate logic: ( both pressed , both released -> stop )
  // if (up && !dn) {
  //   if (g_handlers_step.onArrowUp)   g_handlers_step.onArrowUp(true,  stepper);
  // } else if (dn && !up) {
  //   if (g_handlers_step.onArrowDown) g_handlers_step.onArrowDown(true, stepper);
  // } else {
  //   stepper.stop();
  // }

  // ส่งเหตุการณ์เป็นรายปุ่ม: DOWN => true, UP => false
  if (hasUp) {
    bool nowUp = toDown(tokUp);                 // true เมื่อ C_Up=DOWN, false เมื่อ C_Up=UP
    if (nowUp != prevUp && g_handlers_step.onArrowUp)
      g_handlers_step.onArrowUp(nowUp, stepper);  // onStpUp(true/false)
    prevUp = nowUp;
  }

  if (hasDown) {
    bool nowDn = toDown(tokDown);               // true เมื่อ C_Dn=DOWN, false เมื่อ C_Dn=UP
    if (nowDn != prevDn && g_handlers_step.onArrowDown)
      g_handlers_step.onArrowDown(nowDn, stepper); // onStpDown(true/false)
    prevDn = nowDn;
  }
  return true;
}



static bool isUpRunning = false;
static bool isDownRunning = false;
// callbacks
void onStpUp(bool down, UnifiedStepper& stepper) {
  // if(!down) return; //stop only when up released

  if (down) {
    Serial.println("Stepper CCW start");
    delayMicroseconds(50);
    stepper.rotateContinuous(false);
    delayMicroseconds(50);
    // if (!isUpRunning) {
    //   Serial.println("Stepper CCW start");
    //   stepper.rotateContinuous(false);
    //   isUpRunning = true;
    //   isDownRunning = false; 
    // }
    // else {
    // Serial.println("Stepper stop");
    // stepper.stop();
    // isUpRunning = false;
    // }
  } else{
      delayMicroseconds(100);
      stepper.stop();
  }
}

void onStpDown(bool down, UnifiedStepper& stepper) {
  // if(!down) stepper.stop();
  
  if (down) {
    Serial.println("Stepper CW start");
    delayMicroseconds(50);
    stepper.rotateContinuous(true);
    delayMicroseconds(50);
    // if (!isDownRunning) {
    //   Serial.println("Stepper CW start");
    //   stepper.rotateContinuous(true);
    //   isDownRunning = true;
    //   isUpRunning = false; 
    // }
    // else {
    //   Serial.println("Stepper stop");
    //   stepper.stop();
    //   isDownRunning = false;
    // }
  } else{
      delayMicroseconds(100);
      stepper.stop();
  }
}
