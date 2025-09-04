#ifdef Node1
#include <Arduino.h>
#include "motorDrive.h"
#include "encoder_reader.h"

// --- ปรับพิน encoder ให้ตรงกับการต่อจริง ---
#define ENC_L_A 16
#define ENC_L_B 17
#define ENC_R_A 5
#define ENC_R_B 18

// --- พารามิเตอร์สำหรับ encoder ---
static constexpr float PPR_OUTPUT       = 1320.0f;   // PPR ของ "เพลาขาออก" (คาลิเบรตจริง)
static constexpr float ENC_WHEEL_RADIUS = 0.0635f;   // รัศมีล้อจริง 0.127/2 = 0.0635 m (5 นิ้ว)
// เกณฑ์ “นับว่าเกิดการหมุน” หน่วยเป็นพัลส์ (1 = เปลี่ยน 1 พัลส์ก็พิมพ์)
static constexpr long  MIN_COUNTS_DELTA = 1;

// สร้างอ็อบเจ็กต์อ่าน encoder
DualEncoderReader enc(ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B, PPR_OUTPUT);

// สถานะล่าสุดเพื่อเช็คการเปลี่ยน
static long s_lastCountL = 0;
static long s_lastCountR = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  // เริ่มระบบขับเคลื่อน (คงโค้ด motorDrive เดิม)
  motorDrive_begin();

  // เริ่มอ่าน encoder
  enc.begin(/*enable_internal_pullups=*/true);
  enc.setWheelRadius(ENC_WHEEL_RADIUS);  // ใช้ค่ารัศมีล้อจริง

  // อ่านค่าเริ่มต้นไว้เป็น baseline
  s_lastCountL = enc.countsLeft();
  s_lastCountR = enc.countsRight();

  // (ถ้าทิศทางกลับข้าง ให้เปิดบรรทัดด้านล่าง)
  // enc.setInvert(true, false);
}

void loop() {
  // 1) รับคำสั่งจาก Serial (VW, P, PW4, ESTOP) และตั้งเป้าหมาย PWM
  motorDrive_handleSerialOnce();

  // 2) อัปเดตค่า encoder → pos/vel
  enc.update();

  // 3) ทำ watchdog + ส่ง PWM ไปมอเตอร์
  motorDrive_update();

    // 4) ส่งฟีดแบ็กออก Serial เป็นช่วงๆ
  static uint32_t last = 0;
  const uint32_t FEEDBACK_INTERVAL_MS = 500;  // <- ปรับตรงนี้ได้ (เช่น 100, 200, 500)
  if (millis() - last >= FEEDBACK_INTERVAL_MS) {
    last = millis();

    enc.printFB(Serial);   // FB_ENC VL/VR/PL/PR
    enc.printFB2(Serial);  // FB_ENC2 C/RL/RR
  }


  delay(2);
}
#endif