// encoder_reader.cpp
#include "encoder_reader.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// NOTE: ปรับชื่อพารามิเตอร์ให้ตรงกับ .h: rearA/rearB, frontA/frontB
DualEncoderReader::DualEncoderReader(int rearA, int rearB, int frontA, int frontB,
                                     float pulses_per_rev_output)
: rA_(rearA), rB_(rearB), fA_(frontA), fB_(frontB),
  ppr_out_(pulses_per_rev_output) {}

void DualEncoderReader::begin(bool enable_internal_pullups) {
  ESP32Encoder::useInternalWeakPullResistors = enable_internal_pullups ? UP : DOWN;

  // แนบ encoder ตามด้าน "หลัง/หน้า" ให้ตรงกับ .h
  encR_.attachHalfQuad(rA_, rB_); //Half-Quadrature mode => x2 counts per pulse
  encF_.attachHalfQuad(fA_, fB_); //reset count to zero
  encR_.clearCount();
  encF_.clearCount();

  // ค่าเริ่มต้นล่าสุด
  lastR_ = encR_.getCount(); //เก็บค่า pulse ล่าสุด
  lastF_ = encF_.getCount();
  //เซ็ตค่าเริ่มต้นของ ตำแหน่งเชิงมุม (rad) และ ความเร็วเชิงมุม (rad/s) เป็นศูนย์
  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;
  last_ts_ms_ = millis();
}

// รีเซตซอฟต์ (สะสม/เวลา/มุม/สปีด)
void DualEncoderReader::reset() {
  encR_.clearCount();
  encF_.clearCount();
  lastR_ = 0;
  lastF_ = 0;
  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;
  last_ts_ms_ = millis();
}

void DualEncoderReader::update() {
  // อ่าน count ตามด้าน "หลัง/หน้า" และคูณด้วยทิศ (inv => -1, 1)
  //curR, curF => จำนวนพัลส์ปัจจุบันที่อ่านได้จาก encoder หลัง/หน้า
  long curR = encR_.getCount() * invR_;
  long curF = encF_.getCount() * invF_;

  //dR, dF => ความเปลี่ยนแปลงของ pulse ในรอบนี้ (ใช้หาความเร็ว)
  long dR = curR - lastR_;
  long dF = curF - lastF_;
  lastR_ = curR;
  lastF_ = curF;

  //dt → ช่วงเวลาที่ผ่านไป (วินาที) นับจากการอัปเดตครั้งก่อน
  //ถ้า dt ≤ 0 ให้บังคับเป็น 0.001 s กันหารศูนย์
  uint32_t now = millis();
  float dt = (now - last_ts_ms_) / 1000.0f;
  if (dt <= 0.f) dt = 1e-3f;
  last_ts_ms_ = now;

  const float ppr = (ppr_out_ <= 0.f ? 1.f : ppr_out_);

  // ตำแหน่งเชิงมุม (rad) = (count / PPR) * 2π
  posR_rad_ = ((float)curR / ppr) * 2.0f * (float)M_PI;
  posF_rad_ = ((float)curF / ppr) * 2.0f * (float)M_PI;

  // ความเร็วเชิงมุม (rad/s)
  float revR_dt = ((float)dR / ppr) / dt;
  float revF_dt = ((float)dF / ppr) / dt;
  velR_rad_s_ = revR_dt * 2.0f * (float)M_PI;
  velF_rad_s_ = revF_dt * 2.0f * (float)M_PI;
}

void DualEncoderReader::printFB(Stream& s) const {
  // แบบใหม่: FB_ENC แสดงความเร็วเชิงมุม (rad/s) และ "ระยะทางรวม" (เมตร)
  // ระยะทางรวม = รัศมีล้อ (m) * มุมที่หมุนไปแล้ว (rad)
  float distL = wheel_radius_m_ * posR_rad_;
  float distR = wheel_radius_m_ * posF_rad_;
  s.printf("FB_ENC Vel_R=%.6f Vel_F=%.6f Dist_R=%.6f Dist_F=%.6f\n",
           velR_rad_s_, velF_rad_s_, distL, distR);
}

//Calculate circumference of the wheel
float DualEncoderReader::circumferenceM() const {
  return 2.0f * (float)M_PI * wheel_radius_m_;
}

//Calculate remaining distance to the next full revolution for the right wheel
//C = circumference of the wheel
float DualEncoderReader::remainingToNextRevRearM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posR_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}

//Calculate remaining distance to the next full revolution for the right wheel
//C = circumference of the wheel
float DualEncoderReader::remainingToNextRevFrontM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posF_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}

void DualEncoderReader::printFB2(Stream& s) const {
  // FB_ENC2 แสดงเส้นรอบวงและระยะที่เหลือถึงรอบถัดไป
  s.printf("FB_ENC2 C=%.6f DistLeft_R=%.6f DistLeft_F=%.6f\n",
           circumferenceM(),
           remainingToNextRevRearM(),
           remainingToNextRevFrontM());
}
