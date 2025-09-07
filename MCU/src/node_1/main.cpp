#ifdef Node1
#include <Arduino.h>
#include "motorDrive.h"


void setup() {
  Serial.begin(115200);
  delay(200);

  // เริ่มระบบขับเคลื่อน (คงโค้ด motorDrive เดิม)
  motorDrive_begin();

}

void loop() {
  // 1) รับคำสั่งจาก Serial (VW, P, PW4, ESTOP) และตั้งเป้าหมาย PWM
  motorDrive_handleSerialOnce();

  // 3) ทำ watchdog + ส่ง PWM ไปมอเตอร์
  motorDrive_update();

    // 4) ส่งฟีดแบ็กออก Serial เป็นช่วงๆ
  static uint32_t last = 0;
  const uint32_t FEEDBACK_INTERVAL_MS = 500;  // <- ปรับตรงนี้ได้ (เช่น 100, 200, 500)
  if (millis() - last >= FEEDBACK_INTERVAL_MS) {
    last = millis();
  }


  delay(2);
}
#endif