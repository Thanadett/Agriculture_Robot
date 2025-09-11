#include "encoder_read.h"
#include <math.h>

QuadEncoderReader::QuadEncoderReader(
    int flA, int flB, int frA, int frB, int rlA, int rlB, int rrA, int rrB,
    float pulses_per_rev_output)
    : ppr_out_(pulses_per_rev_output)
{
  pins_[W_FL][0] = flA;
  pins_[W_FL][1] = flB;
  pins_[W_FR][0] = frA;
  pins_[W_FR][1] = frB;
  pins_[W_RL][0] = rlA;
  pins_[W_RL][1] = rlB;
  pins_[W_RR][0] = rrA;
  pins_[W_RR][1] = rrB;
}

void QuadEncoderReader::attachEncoder(ESP32Encoder &e, int pinA, int pinB)
{
  if (quad_mode_ == QUAD_FULL)
    e.attachFullQuad(pinA, pinB);
  else
    e.attachHalfQuad(pinA, pinB);
}

void QuadEncoderReader::begin(bool enable_internal_pullups)
{
  ESP32Encoder::useInternalWeakPullResistors = enable_internal_pullups ? UP : DOWN;

  attachEncoder(enc_[W_FL], pins_[W_FL][0], pins_[W_FL][1]);
  attachEncoder(enc_[W_FR], pins_[W_FR][0], pins_[W_FR][1]);
  attachEncoder(enc_[W_RL], pins_[W_RL][0], pins_[W_RL][1]);
  attachEncoder(enc_[W_RR], pins_[W_RR][0], pins_[W_RR][1]);

  for (int i = 0; i < W_COUNT; ++i)
  {
    enc_[i].clearCount();
    last_counts_[i] = enc_[i].getCount();
    pos_rad_[i] = vel_raw_rad_s_[i] = vel_filt_rad_s_[i] = 0.0f;
    total_dist_m_[i] = 0.0f;
    x1_[i] = x2_[i] = y1_[i] = y2_[i] = 0.0f;
  }
  last_ts_ms_ = millis();

  // default filter config
  if (vfilter_ == VF_BUTTER2)
    setVelFilterButter2(BW2_FC_HZ, BW2_FS_HZ);
}

void QuadEncoderReader::reset()
{
  for (int i = 0; i < W_COUNT; ++i)
  {
    enc_[i].clearCount();
    last_counts_[i] = 0;
    pos_rad_[i] = vel_raw_rad_s_[i] = vel_filt_rad_s_[i] = 0.0f;
    total_dist_m_[i] = 0.0f;
    x1_[i] = x2_[i] = y1_[i] = y2_[i] = 0.0f;
  }
  last_ts_ms_ = millis();
}

void QuadEncoderReader::setVelFilterEMA(float alpha)
{
  vfilter_ = VF_EMA;
  if (alpha < 0.01f)
    alpha = 0.01f;
  if (alpha > 0.99f)
    alpha = 0.99f;
  ema_alpha_ = alpha;
}

void QuadEncoderReader::setVelFilterButter2(float fc, float fs)
{
  vfilter_ = VF_BUTTER2;
  float K = tanf((float)M_PI * fc / fs);
  float K2 = K * K;
  float norm = 1.0f / (1.0f + (float)M_SQRT2 * K + K2);
  bw_b0_ = K2 * norm;
  bw_b1_ = 2.0f * bw_b0_;
  bw_b2_ = bw_b0_;
  bw_a1_ = 2.0f * (K2 - 1.0f) * norm;
  bw_a2_ = (1.0f - (float)M_SQRT2 * K + K2) * norm;
}

void QuadEncoderReader::update()
{
  uint32_t now = millis();
  float dt = (now - last_ts_ms_) / 1000.0f;
  if (dt <= 0)
    dt = 1e-3f;
  last_ts_ms_ = now;

  const float ppr = (ppr_out_ <= 0.0f) ? 1.0f : ppr_out_;
  const float two_pi = 2.0f * (float)M_PI;
  const float C = circumferenceM();

  for (int i = 0; i < W_COUNT; ++i)
  {
    long cur = enc_[i].getCount() * inv_[i];
    long d = cur - last_counts_[i];
    last_counts_[i] = cur;

    total_dist_m_[i] += fabsf(((float)d / ppr) * C);
    pos_rad_[i] = ((float)cur / ppr) * two_pi;

    float rev_dt = ((float)d / ppr) / dt;
    float v_raw = rev_dt * two_pi;
    vel_raw_rad_s_[i] = v_raw;

    float v_out = v_raw;
    if (vfilter_ == VF_EMA)
    {
      v_out = ema_alpha_ * v_raw + (1.0f - ema_alpha_) * vel_filt_rad_s_[i];
    }
    else if (vfilter_ == VF_BUTTER2)
    {
      float x0 = v_raw;
      float y0 = bw_b0_ * x0 + bw_b1_ * x1_[i] + bw_b2_ * x2_[i] - bw_a1_ * y1_[i] - bw_a2_ * y2_[i];
      x2_[i] = x1_[i];
      x1_[i] = x0;
      y2_[i] = y1_[i];
      y1_[i] = y0;
      v_out = y0;
    }
    vel_filt_rad_s_[i] = v_out;
  }
}

void QuadEncoderReader::bodyTwistFromWheels(float &v_m_s, float &wz_rad_s) const
{
  float wL = 0.5f * (vel_filt_rad_s_[W_FL] + vel_filt_rad_s_[W_RL]);
  float wR = 0.5f * (vel_filt_rad_s_[W_FR] + vel_filt_rad_s_[W_RR]);
  float r = wheel_radius_m_;
  float b = (track_m_ > 1e-6f) ? track_m_ : 0.30f;

  v_m_s = 0.5f * r * (wR + wL);
  wz_rad_s = (r / b) * (wR - wL);
}
