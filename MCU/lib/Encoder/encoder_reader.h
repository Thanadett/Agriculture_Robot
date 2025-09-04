#pragma once
// ===== encoder_reader.h =====
// อ่าน Quadrature Encoder สองล้อ (ซ้าย/ขวา) ด้วยไลบรารี ESP32Encoder
// - position [rad], velocity [rad/s]
// - ปรับ PPR (pulses per revolution "เพลาขาออก") ได้
// - invert ทิศทางซ้าย/ขวาได้
// - รองรับคำนวณเส้นรอบวง (2πR) และ "ระยะที่เหลือถึงรอบถัดไป"
// - ส่งฟีดแบ็กสองแบบ: FB_ENC (rad, rad/s) และ FB_ENC2 (ระยะทาง)

#include <Arduino.h>
#include <ESP32Encoder.h>

class DualEncoderReader {
public:
  // ระบุขา A/B ของซ้ายและขวา และ PPR ของ "เพลาขาออก"
  DualEncoderReader(int leftA, int leftB, int rightA, int rightB,
                    float pulses_per_rev_output);

  // เรียกครั้งเดียวใน setup()
  void begin(bool enable_internal_pullups = true);

  // เรียกซ้ำใน loop() ทุกคาบ (20–50 ms)
  void update();

  // ===== Getters =====
  // ตำแหน่งเชิงมุม (rad)
  float positionLeftRad()  const { return posL_rad_; }
  float positionRightRad() const { return posR_rad_; }

  // ความเร็วเชิงมุม (rad/s)
  float velocityLeftRad()  const { return velL_rad_s_; }
  float velocityRightRad() const { return velR_rad_s_; }

  // จำนวนพัลส์สะสม (ตั้งแต่เริ่ม) 
  // → เอา const ออกเพราะ ESP32Encoder::getCount() ไม่รองรับ const
  long countsLeft()  { return encL_.getCount(); }
  long countsRight() { return encR_.getCount(); }

  // รีเซตซอฟต์ (สะสม/เวลา/มุม/สปีด)
  void reset();

  // ตั้ง/อ่าน PPR (เพลาขาออก)
  void setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }

  // กลับทิศทางการนับ (แยกซ้าย/ขวา)
  void setInvert(bool invert_left, bool invert_right) {
    invL_ = invert_left  ? -1 : 1;
    invR_ = invert_right ? -1 : 1;
  }

  // ---------- ส่วนของ "ระยะทาง/รอบล้อ" ----------
  // ตั้ง/อ่าน รัศมีล้อ (เมตร) เพื่อให้คำนวณระยะทางได้
  void  setWheelRadius(float radius_m) { wheel_radius_m_ = radius_m; }
  float wheelRadius() const            { return wheel_radius_m_; }

  // ระยะที่วิ่งไปแล้วของแต่ละล้อ (เมตร) = R * theta
  float distanceLeftM()  const { return wheel_radius_m_ * posL_rad_; }
  float distanceRightM() const { return wheel_radius_m_ * posR_rad_; }

  // เส้นรอบวงล้อ (เมตร) = 2πR
  float circumferenceM() const;

  // ระยะ "ที่เหลือ" จนครบรอบถัดไป (เมตร)
  float remainingToNextRevLeftM()  const;
  float remainingToNextRevRightM() const;

  // ---------- Serial feedback ----------
  // แบบเดิม: FB_ENC VL=<rad/s> VR=<rad/s> PL=<rad> PR=<rad>
  void printFB(Stream& s) const;

  // แบบใหม่: FB_ENC2 C=<m> RL=<m> RR=<m>
  //   C = เส้นรอบวงล้อ (เมตร)
  //   RL/RR = ระยะที่เหลือถึง "ครบหนึ่งรอบถัดไป" (เมตร)
  void printFB2(Stream& s) const;

private:
  // พิน
  int lA_, lB_, rA_, rB_;

  // ไลบรารี ESP32Encoder
  ESP32Encoder encL_;
  ESP32Encoder encR_;

  // ค่าคาลิเบรต
  float ppr_out_;

  // ทิศ (1 หรือ -1)
  int invL_ = 1;
  int invR_ = 1;

  // รัศมีล้อ (เมตร) — จำเป็นต่อการคำนวณระยะ
  float wheel_radius_m_ = 0.05f; // ค่าเริ่มต้น 5 ซม. (แก้ตามล้อจริงด้วย setWheelRadius)

  // สถานะภายใน
  long     lastL_ = 0, lastR_ = 0;    // ค่าพัลส์ล่าสุด
  long     accumL_ = 0, accumR_ = 0;  // สะสมพัลส์
  uint32_t last_ts_ms_ = 0;           // เวลาอัปเดตครั้งก่อน (ms)

  // ค่าที่คำนวณล่าสุด
  float posL_rad_   = 0.f, posR_rad_   = 0.f;
  float velL_rad_s_ = 0.f, velR_rad_s_ = 0.f;

  
};
