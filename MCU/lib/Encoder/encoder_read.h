#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config.h"

enum WheelIndex : uint8_t
{
  W_FL = 0,
  W_FR = 1,
  W_RL = 2,
  W_RR = 3,
  W_COUNT = 4
};
enum QuadMode : uint8_t
{
  QUAD_HALF = 0,
  QUAD_FULL = 1
};
enum VelFilter : uint8_t
{
  VF_NONE = 0,
  VF_EMA = 1,
  VF_BUTTER2 = 2
};

class QuadEncoderReader
{
public:
  QuadEncoderReader(
      int flA = ENC_FL_A, int flB = ENC_FL_B,
      int frA = ENC_FR_A, int frB = ENC_FR_B,
      int rlA = ENC_RL_A, int rlB = ENC_RL_B,
      int rrA = ENC_RR_A, int rrB = ENC_RR_B,
      float pulses_per_rev_output = ENCODER_PPR_OUTPUT_DEFAULT);

  void begin(bool enable_internal_pullups = true);
  void update();
  void reset();

  // getters
  float positionRad(WheelIndex w) const { return pos_rad_[w]; }
  float velocityRadPerSec(WheelIndex w) const { return vel_filt_rad_s_[w]; }
  float velocityRawRadPerSec(WheelIndex w) const { return vel_raw_rad_s_[w]; }
  long counts(WheelIndex w) { return enc_[w].getCount(); }
  float totalDistanceM(WheelIndex w) const { return total_dist_m_[w]; }

  // geometry / params
  void setWheelRadius(float r_m) { wheel_radius_m_ = r_m; }
  float wheelRadius() const { return wheel_radius_m_; }
  float circumferenceM() const { return 2.0f * (float)M_PI * wheel_radius_m_; }

  void setTrack(float track_m) { track_m_ = track_m; }
  float track() const { return track_m_; }

  void setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }
  void setQuadMode(QuadMode m) { quad_mode_ = m; }

  void setInvert(bool invFL, bool invFR, bool invRL, bool invRR)
  {
    inv_[W_FL] = invFL ? -1 : 1;
    inv_[W_FR] = invFR ? -1 : 1;
    inv_[W_RL] = invRL ? -1 : 1;
    inv_[W_RR] = invRR ? -1 : 1;
  }

  // filters
  void setVelFilterNone() { vfilter_ = VF_NONE; }
  void setVelFilterEMA(float alpha);
  void setVelFilterButter2(float fc, float fs);

  // helper
  void bodyTwistFromWheels(float &v_m_s, float &wz_rad_s) const;

private:
  void attachEncoder(ESP32Encoder &e, int pinA, int pinB);

  int pins_[W_COUNT][2];
  ESP32Encoder enc_[W_COUNT];

  float ppr_out_ = ENCODER_PPR_OUTPUT_DEFAULT;
  int inv_[W_COUNT] = {ENC_INV_FL, ENC_INV_FR, ENC_INV_RL, ENC_INV_RR};
  float wheel_radius_m_ = WHEEL_RADIUS_M;
  float track_m_ = TRACK_M;
  QuadMode quad_mode_ = ENCODER_QUAD_MODE_FULL ? QUAD_FULL : QUAD_HALF;

  long last_counts_[W_COUNT] = {0, 0, 0, 0};
  uint32_t last_ts_ms_ = 0;

  float pos_rad_[W_COUNT] = {0, 0, 0, 0};
  float vel_raw_rad_s_[W_COUNT] = {0, 0, 0, 0};
  float vel_filt_rad_s_[W_COUNT] = {0, 0, 0, 0};
  float total_dist_m_[W_COUNT] = {0, 0, 0, 0};

  VelFilter vfilter_ = (VEL_FILTER_MODE == 1) ? VF_EMA : ((VEL_FILTER_MODE == 2) ? VF_BUTTER2 : VF_NONE);
  float ema_alpha_ = EMA_ALPHA;

  // 2nd order LPF (bilinear) coeffs
  float bw_b0_ = 1, bw_b1_ = 0, bw_b2_ = 0, bw_a1_ = 0, bw_a2_ = 0;
  float x1_[W_COUNT] = {0}, x2_[W_COUNT] = {0}, y1_[W_COUNT] = {0}, y2_[W_COUNT] = {0};
};
