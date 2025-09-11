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
