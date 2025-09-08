#include "encoder_read.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

DualEncoderReader::DualEncoderReader(int rearA, int rearB, int frontA, int frontB,
                                     float pulses_per_rev_output)
: rA_(rearA), rB_(rearB), fA_(frontA), fB_(frontB),
  ppr_out_(pulses_per_rev_output) {}

void DualEncoderReader::begin(bool enable_internal_pullups) {
  ESP32Encoder::useInternalWeakPullResistors = enable_internal_pullups ? UP : DOWN;

  encR_.attachHalfQuad(rA_, rB_);
  encF_.attachHalfQuad(fA_, fB_);
  encR_.clearCount();
  encF_.clearCount();

  lastR_ = encR_.getCount();
  lastF_ = encF_.getCount();

  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;

  total_dist_R_m_ = 0.f;
  total_dist_F_m_ = 0.f;

  last_ts_ms_ = millis();
}

void DualEncoderReader::reset() {
  encR_.clearCount();
  encF_.clearCount();
  lastR_ = 0; lastF_ = 0;
  posR_rad_ = posF_rad_ = 0.f;
  velR_rad_s_ = velF_rad_s_ = 0.f;
  total_dist_R_m_ = 0.f;
  total_dist_F_m_ = 0.f;
  last_ts_ms_ = millis();
}

void DualEncoderReader::update() {
  long curR = encR_.getCount() * invR_;
  long curF = encF_.getCount() * invF_;

  long dR = curR - lastR_;
  long dF = curF - lastF_;
  lastR_  = curR;
  lastF_  = curF;

  uint32_t now = millis();
  float dt = (now - last_ts_ms_) / 1000.0f;
  if (dt <= 0.f) dt = 1e-3f;
  last_ts_ms_ = now;

  const float ppr = (ppr_out_ <= 0.f ? 1.f : ppr_out_);
  const float two_pi = 2.0f * (float)M_PI;
  const float C = circumferenceM();

  total_dist_R_m_ += fabsf(((float)dR / ppr) * C);
  total_dist_F_m_ += fabsf(((float)dF / ppr) * C);

  posR_rad_ = ((float)curR / ppr) * two_pi;
  posF_rad_ = ((float)curF / ppr) * two_pi;

  float revR_dt = ((float)dR / ppr) / dt;
  float revF_dt = ((float)dF / ppr) / dt;
  velR_rad_s_ = revR_dt * two_pi;
  velF_rad_s_ = revF_dt * two_pi;
}

float DualEncoderReader::circumferenceM() const {
  return 2.0f * (float)M_PI * wheel_radius_m_;
}

float DualEncoderReader::remainingToNextRevRearM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posR_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}

float DualEncoderReader::remainingToNextRevFrontM() const {
  if (wheel_radius_m_ <= 0.f) return 0.f;
  const float C = circumferenceM();
  const float rev = posF_rad_ / (2.0f * (float)M_PI);
  const float rev_next = ceilf(rev);
  float remain_rev = rev_next - rev;
  if (remain_rev < 1e-6f) remain_rev = 0.f;
  return remain_rev * C;
}
