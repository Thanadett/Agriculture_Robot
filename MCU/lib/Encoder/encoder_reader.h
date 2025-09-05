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

class DualEncoderReader {
public:
  // ระบุขา A/B ของ หลัง และ หน้า และ PPR ของ "เพลาขาออก"
  DualEncoderReader(int rearA, int rearB, int frontA, int frontB,
                    float pulses_per_rev_output);

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
  // → เอา const ออกเพราะ ESP32Encoder::getCount() ไม่รองรับ const
  long countsRear()  { return encR_.getCount(); }
  long countsFront() { return encF_.getCount(); }

  // รีเซตซอฟต์ (สะสม/เวลา/มุม/สปีด)
  void reset();

  // ตั้ง/อ่าน PPR (เพลาขาออก)
  void setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }

  // กลับทิศทางการนับ (แยก หลัง/หน้า)
  void setInvert(bool invert_left, bool invert_right) {
    invR_ = invert_left  ? -1 : 1;
    invF_ = invert_right ? -1 : 1;
  }

  // ---------- ส่วนของ "ระยะทาง/รอบล้อ" ----------
  // ตั้ง/อ่าน รัศมีล้อ (เมตร) เพื่อให้คำนวณระยะทางได้
  void  setWheelRadius(float radius_m) { wheel_radius_m_ = radius_m; }
  float wheelRadius() const            { return wheel_radius_m_; }

  // ระยะที่วิ่งไปแล้วของแต่ละล้อ (เมตร) = R * theta
  float distanceRearM()  const { return wheel_radius_m_ * posR_rad_; }
  float distanceFrontM() const { return wheel_radius_m_ * posF_rad_; }

  // เส้นรอบวงล้อ (เมตร) = 2πR
  float circumferenceM() const;

  // ระยะ "ที่เหลือ" จนครบรอบถัดไป (เมตร)
  float remainingToNextRevRearM()  const;
  float remainingToNextRevFrontM() const;

  // ---------- Serial feedback ----------
  // แบบเดิม: FB_ENC VL=<rad/s> VR=<rad/s> PL=<rad> PR=<rad>
  void printFB(Stream& s) const;

  // แบบใหม่: FB_ENC2 C=<m> RL=<m> RR=<m>
  //   C = เส้นรอบวงล้อ (เมตร)
  //   RL/RR = ระยะที่เหลือถึง "ครบหนึ่งรอบถัดไป" (เมตร)
  void printFB2(Stream& s) const;

private:
  // พิน
  int rA_, rB_, fA_, fB_;

  // ไลบรารี ESP32Encoder
  ESP32Encoder encR_;
  ESP32Encoder encF_;

  // ค่าคาลิเบรต
  float ppr_out_; //ppr => pulses per revolution (ของเพลาขาออก)

  // ทิศ (1 หรือ -1)
  int invR_ = 1;
  int invF_ = 1;

  // รัศมีล้อ (เมตร) — จำเป็นต่อการคำนวณระยะ
  float wheel_radius_m_ = 0.05f; // ค่าเริ่มต้น 5 ซม. (แก้ตามล้อจริงด้วย setWheelRadius)

  // สถานะภายใน
  long     lastR_ = 0, lastF_ = 0;    // ค่าพัลส์ล่าสุด
  long     accumR_ = 0, accumF_ = 0;  // สะสมพัลส์
  uint32_t last_ts_ms_ = 0;           // เวลาอัปเดตครั้งก่อน (ms)

  // ค่าที่คำนวณล่าสุด
  float posR_rad_   = 0.f, posF_rad_   = 0.f;
  float velR_rad_s_ = 0.f, velF_rad_s_ = 0.f;

  
};
