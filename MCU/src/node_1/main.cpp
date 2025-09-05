#ifdef Node1
#include <Arduino.h>
#include "motorDrive.h"
#include "encoder_reader.h"

// ใช้ค่า default ที่ประกาศไว้ใน encoder_reader.h
DualEncoderReader enc;  // (rearA=ENC_R_A, rearB=ENC_R_B, frontA=ENC_F_A, frontB=ENC_F_B, PPR=ENCODER_PPR_OUTPUT_DEFAULT)

void setup() {
  Serial.begin(115200);
  delay(200);

  motorDrive_begin();

  enc.begin(/*enable_internal_pullups=*/true);
  // enc.setInvert(true, false);          // ถ้าทิศกลับค่อยเปิดใช้
  // enc.setPPR(…); enc.setWheelRadius(…); // ถ้าอยาก override ค่า default
}

void loop() {
  motorDrive_handleSerialOnce();  // รับคำสั่ง VW/P/PW4/ESTOP
  enc.update();                   // อัปเดต encoder
  motorDrive_update();            // watchdog + PWM

  static uint32_t last = 0;
  const uint32_t FEEDBACK_INTERVAL_MS = 500;  // ปรับความถี่พิมพ์ได้
  if (millis() - last >= FEEDBACK_INTERVAL_MS) {
    last = millis();
    enc.printFB(Serial);   // FB_ENC
    enc.printFB2(Serial);  // FB_ENC2
  }
  delay(2);
}

#endif
