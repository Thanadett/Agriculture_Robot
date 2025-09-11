#include <Arduino.h>
#include "servo.h"


void setup() {
  Serial.begin(115200);
  delay(1000);


    TD8120MG.setPeriodHertz(50); // Set refresh period to 50Hz
    TD8120MG.attach(PIN_MG996R_360, min_p_width, max_p_width); // Attach the servo on pin

  // Instances
  UnifiedServo servo360(ServoKind::Positional180,  PIN__TD8120MG, TD8120MG);
  UnifiedServo servo360(ServoKind::Continuous360,  PIN_MG996R_360, MG996R_360);
  UnifiedServo servo180(ServoKind::Positional180,  PIN_MG996R, MG996R);

  // Shared action (works for both types):
  // - 180°: move to mechanical center (~90°)
  // - 360°: stop (neutral)
  TD8120MG.goCenterOrStop();
  MG996R_360.goCenterOrStop();
  MG996R.goCenterOrStop();
}



void loop() {
  // ===== 180° demo: absolute angles =====
  MG996R.setAngleDeg(0);    delay(600);
  MG996R.setAngleDeg(90);   delay(600);
  MG996R.setAngleDeg(180);  delay(600);
  MG996R.setAngleDeg(90);   delay(600);

  // ===== 360° demo: speed/direction =====
  TD8120MG.setSpeedPercent(+50); delay(1500);
  TD8120MG.goCenterOrStop();     delay(800);
  TD8120MG.setSpeedPercent(-30); delay(1500);
  TD8120MG.goCenterOrStop();     delay(1200);

  // ===== Shared command example: direct microseconds =====
  MG996R.setPulseUs(1500); // center for 180°
  TD8120MG.setPulseUs(1500); // stop for 360°
  delay(800);
}
