#ifdef Node2
#include <Arduino.h>
#include "servo.h"


void setup() {
  Serial.begin(115200);
  delay(1000);
  BTN_handlers handlers;
  handlers.onA = onBtnA;
  handlers.onB = onBtnB;
  handlers.onX = onBtnX;
  //callbacks for button A/B/X
  button_set_handlers(handlers); 
  

  TD8120MG.begin();// Attach the servo on pin , set Hz and min/max pulse width
  MG996R_360.begin();
  MG996R.begin();

  //======================== debug info ========================
  int ch_1 = TD8120MG.begin();
  int ch_2 = MG996R_360.begin();
  int ch_3 = MG996R.begin();
  if (ch_3 >= 0)
    Serial.printf("MG996R attached on channel %d (pin=%d)\n", ch_3, MG996R.pin());
  else
    Serial.printf("MG996R attach FAILED (pin=%d)\n", MG996R.pin());
  if (ch_2 >= 0)
    Serial.printf("MG996R_360 attached on channel %d (pin=%d)\n", ch_2, MG996R_360.pin());
  else
    Serial.printf("MG996R_360 attach FAILED (pin=%d)\n", MG996R_360.pin());
  if (ch_1 >= 0)
    Serial.printf("TD8120MG attached on channel %d (pin=%d)\n", ch_2, TD8120MG.pin());
  else
    Serial.printf("TD8120MG attach FAILED (pin=%d)\n", TD8120MG.pin());
  // ======================== end debug info ========================
  // Shared action (works for both types):
  // - 180°: move to mechanical center (~90°)
  // - 360°: stop (neutral)
  TD8120MG.goCenterOrStop();
  MG996R_360.goCenterOrStop();
  MG996R.setAngleDeg(0); // start at 0 degree
}



void loop() {
  // ===== 180° demo: absolute angles =====
  // MG996R.setAngleDeg(0);    
  // delay(600);


  // ===== 360° demo: speed/direction =====
  // TD8120MG.setSpeedPercent(+50); delay(1500);
  // TD8120MG.goCenterOrStop();     delay(800);
  // TD8120MG.setSpeedPercent(-30); delay(1500);
  // TD8120MG.goCenterOrStop();     delay(1200);

  // ===== Shared command example: direct microseconds =====
  // MG996R.setPulseUs(1500); // center for 180°
  // TD8120MG.setPulseUs(1500); // stop for 360°
  // delay(800);
  button_serial_poll();
  delay(10);
}
#endif
