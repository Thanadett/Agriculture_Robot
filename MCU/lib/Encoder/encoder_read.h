#pragma once
// ===== encoder_reader.h =====
// อ่าน Quadrature Encoder สองล้อ (หลัง/หน้า) ด้วยไลบรารี ESP32Encoder
// - position [rad], velocity [rad/s]
// - ปรับ PPR (pulses per revolution "เพลาขาออก") ได้
// - invert ทิศทางหลัง/หน้าได้
// - รองรับคำนวณเส้นรอบวง (2πR) และ "ระยะที่เหลือถึงรอบถัดไป"
// - ส่งฟีดแบ็กสองแบบ: FB_ENC (rad, rad/s) และ FB_ENC2 (ระยะทาง)

#include <Arduino.h>
#include <ESP32Encoder.h>

//yellow wire = A, green wire = B
#define ENC_R_A 16   // R => rear A
#define ENC_R_B 17   // R => rear B
#define ENC_F_A 5    // F => front A
#define ENC_F_B 18   // F => front B

// สร้างอ็อบเจ็กต์อ่าน encoder
#define ENCODER_PPR_MOTOR 11.0f  // pulses/รอบ ที่แกนมอเตอร์ (จาก datasheet)
#define REDUCTION_RATIO   270.0f // อัตราทดเกียร์ (จากรุ่นที่ใช้)
#define QUAD_MULTIPLIER   2.0f   // 2.0 = half-quad, 4.0 = full-quad
#define ENC_WHEEL_RADIUS  0.0635f // m (ล้อเส้นผ่านศูนย์กลาง 0.127 m)

// คำนวน PPR ที่ “เพลาขาออก” + โหมดนับ เพื่อนำไปแปลงเป็นเรเดียน/ส. และระยะทาง
static constexpr float ENCODER_PPR_OUTPUT_DEFAULT =
    ENCODER_PPR_MOTOR * REDUCTION_RATIO * QUAD_MULTIPLIER;

class DualEncoderReader {
public:
  // ระบุขา A/B ของ หลัง และ หน้า และ PPR ของ "เพลาขาออก"
  // มีค่า default เท่ากับที่กำหนดด้านบน ทำให้เรียกได้โดยไม่ต้องส่งอาร์กิวเมนต์
  DualEncoderReader(int rearA = ENC_R_A, int rearB = ENC_R_B,
                    int frontA = ENC_F_A, int frontB = ENC_F_B,
                    float pulses_per_rev_output = ENCODER_PPR_OUTPUT_DEFAULT);

  // เรียกครั้งเดียวใน setup()
  void begin(bool enable_internal_pullups = true);

  // เรียกซ้ำใน loop() ทุกคาบ (20–50 ms)
  void update();

  // ===== Getters =====
  // ตำแหน่งเชิงมุม (rad)
  float positionRearRad()  const { return posR_rad_; }
  float positionFrontRad() const { return posF_rad_; }

  // ความเร็วเชิงมุม (rad/s)
  float velocityRearRad()  const { return velR_rad_s_; }
  float velocityFrontRad() const { return velF_rad_s_; }

  // จำนวนพัลส์สะสม (ตั้งแต่เริ่ม) 
  long countsRear()  { return encR_.getCount(); }
  long countsFront() { return encF_.getCount(); }

  // รีเซตซอฟต์ (สะสม/เวลา/มุม/สปีด)
  void reset();

  // ตั้ง/อ่าน PPR (เพลาขาออก)
  void setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }

  // กลับทิศทางการนับ (แยก หลัง/หน้า)
  void setInvert(bool invert_rear, bool invert_front) {
    invR_ = invert_rear  ? -1 : 1;
    invF_ = invert_front ? -1 : 1;
  }

  // ---------- ส่วนของ "ระยะทาง/รอบล้อ" ----------
  // ตั้ง/อ่าน รัศมีล้อ (เมตร) เพื่อให้คำนวณระยะทางได้
  void  setWheelRadius(float radius_m) { wheel_radius_m_ = radius_m; }
  float wheelRadius() const            { return wheel_radius_m_; }

  // ระยะที่วิ่งไปแล้วของแต่ละล้อ (เมตร) = R * theta  (ของ “สถานะปัจจุบัน”)
  float distanceRearM()  const { return wheel_radius_m_ * posR_rad_; }
  float distanceFrontM() const { return wheel_radius_m_ * posF_rad_; }

  // ระยะรวมสะสมทั้งหมดตั้งแต่เริ่ม (TOTAL) — ไม่รีเซ็ตเองในแต่ละรอบ
  float totalDistanceRearM()  const { return total_dist_R_m_; }
  float totalDistanceFrontM() const { return total_dist_F_m_; }

  // เส้นรอบวงล้อ (เมตร) = 2πR
  float circumferenceM() const;

  // ระยะ "ที่เหลือ" จนครบรอบถัดไป (เมตร)
  float remainingToNextRevRearM()  const;
  float remainingToNextRevFrontM() const;

  // ---------- Serial feedback ----------
  void printFB(Stream& s) const;
  void printFB2(Stream& s) const;

private:
  // พิน
  int rA_, rB_, fA_, fB_;

  // ไลบรารี ESP32Encoder
  ESP32Encoder encR_;
  ESP32Encoder encF_;

  // ค่าคาลิเบรต
  float ppr_out_ = ENCODER_PPR_OUTPUT_DEFAULT;

  // ทิศ (1 หรือ -1)
  int invR_ = 1;
  int invF_ = 1;

  // รัศมีล้อ (เมตร) — จำเป็นต่อการคำนวณระยะ
  float wheel_radius_m_ = ENC_WHEEL_RADIUS; // ค่าเริ่มต้น (แก้ตามล้อจริงด้วย setWheelRadius)

  // สถานะภายใน
  long     lastR_ = 0, lastF_ = 0;    // ค่าพัลส์ล่าสุด (ไว้คำนวณ delta)
  uint32_t last_ts_ms_ = 0;           // เวลาอัปเดตครั้งก่อน (ms)

  // ค่าที่คำนวณล่าสุด
  float posR_rad_   = 0.f, posF_rad_   = 0.f;
  float velR_rad_s_ = 0.f, velF_rad_s_ = 0.f;

  // ตัวนับจำนวนรอบ (อิงจาก count/PPR ปัจจุบัน)
  long revRear_  = 0;
  long revFront_ = 0;

  // ระยะทางสะสมรวมทั้งหมด (TOTAL) — เพิ่มขึ้นตาม |Δcount| * (C/PPR)
  float total_dist_R_m_ = 0.f;
  float total_dist_F_m_ = 0.f;
};
