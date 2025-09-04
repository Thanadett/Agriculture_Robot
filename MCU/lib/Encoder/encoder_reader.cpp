#include "encoder_reader.h"

DualEncoderReader::DualEncoderReader(int leftA, int leftB, int rightA, int rightB,
                                     float pulses_per_rev_output)
: lA_(leftA), lB_(leftB), rA_(rightA), rB_(rightB),
  ppr_out_(pulses_per_rev_output) {}

void DualEncoderReader::begin(bool enable_internal_pullups) {
  ESP32Encoder::useInternalWeakPullResistors = enable_internal_pullups ? UP : DOWN;

  encL_.attachHalfQuad(lA_, lB_);
  encR_.attachHalfQuad(rA_, rB_);
  encL_.clearCount();
  encR_.clearCount();

  lastL_ = encL_.getCount();
  lastR_ = encR_.getCount();
  posL_rad_ = posR_rad_ = 0.f;
  velL_rad_s_ = velR_rad_s_ = 0.f;
  last_ts_ms_ = millis();
}

void DualEncoderReader::reset() {
  encL_.clearCount();
  encR_.clearCount();
  lastL_ = 0;
  lastR_ = 0;
  posL_rad_ = posR_rad_ = 0.f;
  velL_rad_s_ = velR_rad_s_ = 0.f;
  last_ts_ms_ = millis();
}

void DualEncoderReader::update() {
  long curL = encL_.getCount() * invL_;
  long curR = encR_.getCount() * invR_;

  long dL = curL - lastL_;
  long dR = curR - lastR_;
  lastL_ = curL;
  lastR_ = curR;

  uint32_t now = millis();
  float dt = (now - last_ts_ms_) / 1000.0f;
  if (dt <= 0.f) dt = 1e-3f;
  last_ts_ms_ = now;

  const float ppr = (ppr_out_ <= 0.f ? 1.f : ppr_out_);

  posL_rad_ = ((float)curL / ppr) * 2.0f * (float)M_PI;
  posR_rad_ = ((float)curR / ppr) * 2.0f * (float)M_PI;

  float revL_dt = ((float)dL / ppr) / dt;
  float revR_dt = ((float)dR / ppr) / dt;
  velL_rad_s_ = revL_dt * 2.0f * (float)M_PI;
  velR_rad_s_ = revR_dt * 2.0f * (float)M_PI;
}

void DualEncoderReader::printFB(Stream& s) const {
  // แบบใหม่: FB_ENC แสดงความเร็วเชิงมุม (rad/s) และ "ระยะทางรวม" (เมตร)
  // ระยะทางรวม = รัศมีล้อ (m) * มุมที่หมุนไปแล้ว (rad)
  float distL = wheel_radius_m_ * posL_rad_;
  float distR = wheel_radius_m_ * posR_rad_;
  s.printf("FB_ENC Vel_L=%.6f Vel_R=%.6f Dist_L=%.6f Dist_R=%.6f\n",
           velL_rad_s_, velR_rad_s_, distL, distR);
}


float DualEncoderReader::circumferenceM() const {
  return 2.0f * (float)M_PI * wheel_radius_m_;
}

float DualEncoderReader::remainingToNextRevLeftM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posL_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}

float DualEncoderReader::remainingToNextRevRightM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posR_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}

void DualEncoderReader::printFB2(Stream& s) const {
  // FB_ENC2 แสดงเส้นรอบวงและระยะที่เหลือถึงรอบถัดไป
  s.printf("FB_ENC2 C=%.6f RL=%.6f RR=%.6f\n",
           circumferenceM(),
           remainingToNextRevLeftM(),
           remainingToNextRevRightM());
}
