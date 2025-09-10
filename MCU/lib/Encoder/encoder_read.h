#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Wheel index order: FL, FR, RL, RR ---
enum WheelIndex : uint8_t { W_FL = 0, W_FR = 1, W_RL = 2, W_RR = 3, W_COUNT = 4 };

// --- Quadrature attach mode ---
enum QuadMode : uint8_t { QUAD_HALF = 0, QUAD_FULL = 1 };

// --- Velocity filter type ---
enum VelFilter : uint8_t { VF_NONE = 0, VF_EMA = 1, VF_BUTTER2 = 2 };

// --- Default encoder pins ---
#define ENC_FL_A 35 // yellow
#define ENC_FL_B 32 // green
#define ENC_RL_A 33
#define ENC_RL_B 25
#define ENC_FR_A 19
#define ENC_FR_B 18
#define ENC_RR_A 17
#define ENC_RR_B 16

// --- Encoder spec defaults ---
#define ENCODER_PPR_MOTOR 11.0f
#define REDUCTION_RATIO   270.0f     // gearbox ratio
#define QUAD_MULTIPLIER_DEFAULT 2.0f // half-quad = ×2
#define ENC_WHEEL_RADIUS  0.0635f    // m (D=0.127 m ~ 5")

static constexpr float ENCODER_PPR_OUTPUT_DEFAULT =
    ENCODER_PPR_MOTOR * REDUCTION_RATIO * QUAD_MULTIPLIER_DEFAULT;

// ===== 4-wheel quadrature encoder reader =====
class QuadEncoderReader {
public:
  QuadEncoderReader(
      int flA = ENC_FL_A, int flB = ENC_FL_B,
      int frA = ENC_FR_A, int frB = ENC_FR_B,
      int rlA = ENC_RL_A, int rlB = ENC_RL_B,
      int rrA = ENC_RR_A, int rrB = ENC_RR_B,
      float pulses_per_rev_output = ENCODER_PPR_OUTPUT_DEFAULT)
  : ppr_out_(pulses_per_rev_output) {
    pins_[W_FL][0] = flA; pins_[W_FL][1] = flB;
    pins_[W_FR][0] = frA; pins_[W_FR][1] = frB;
    pins_[W_RL][0] = rlA; pins_[W_RL][1] = rlB;
    pins_[W_RR][0] = rrA; pins_[W_RR][1] = rrB;
  }

  // ---- lifecycle ----
  void begin(bool enable_internal_pullups = true);
  void update();
  void reset();

  // ---- getters ----
  float positionRad(WheelIndex w)       const { return pos_rad_[w]; }           // [rad]
  float velocityRadPerSec(WheelIndex w) const { return vel_filt_rad_s_[w]; }    // filtered [rad/s]
  float velocityRawRadPerSec(WheelIndex w) const { return vel_raw_rad_s_[w]; }  // raw [rad/s]
  long  counts(WheelIndex w)                  { return enc_[w].getCount(); }
  float totalDistanceM(WheelIndex w)    const { return total_dist_m_[w]; }       // accumulated |m|

  // ---- wheel geometry / kinematics ----
  void  setWheelRadius(float r_m) { wheel_radius_m_ = r_m; }
  float wheelRadius() const       { return wheel_radius_m_; }
  float circumferenceM() const    { return 2.0f * (float)M_PI * wheel_radius_m_; }

  void  setTrack(float track_m)   { track_m_ = track_m; } // ระยะล้อซ้าย-ขวา (ฐานราง)
  float track() const             { return track_m_; }

  // ---- PPR & quad mode ----
  void  setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }

  void setQuadMode(QuadMode m) { quad_mode_ = m; } // QUAD_HALF (×2) / QUAD_FULL (×4)

  // ---- invert direction per wheel ----
  void setInvert(bool invFL, bool invFR, bool invRL, bool invRR) {
    inv_[W_FL] = invFL ? -1 : 1;
    inv_[W_FR] = invFR ? -1 : 1;
    inv_[W_RL] = invRL ? -1 : 1;
    inv_[W_RR] = invRR ? -1 : 1;
  }

  // ---- velocity filters ----
  void setVelFilterNone() { vfilter_ = VF_NONE; }
  void setVelFilterEMA(float alpha) { vfilter_ = VF_EMA; ema_alpha_ = constrain(alpha, 0.01f, 0.99f); }
  // Butterworth 2nd-order: ตั้งค่า cutoff (Hz) และ sample rate (Hz)
  void setVelFilterButter2(float fc, float fs);

  // ---- helper: average side speeds & body twist ----
  // คืนค่า (v, w) ของตัวถัง 2D จากความเร็วล้อ (เฉลี่ยซ้าย/ขวา)
  void bodyTwistFromWheels(float& v_linear_m_s, float& w_yaw_rad_s) const;

private:
  void attachEncoder(ESP32Encoder& e, int pinA, int pinB);

private:
  int pins_[W_COUNT][2];
  ESP32Encoder enc_[W_COUNT];

  float ppr_out_ = ENCODER_PPR_OUTPUT_DEFAULT;
  int   inv_[W_COUNT] = {1,1,1,1};
  float wheel_radius_m_ = ENC_WHEEL_RADIUS;
  float track_m_ = 0.30f;  // <<== ตั้งให้ตรงกับหุ่น (ระยะล้อซ้าย-ขวา)

  QuadMode quad_mode_ = QUAD_HALF;

  long     last_counts_[W_COUNT] = {0,0,0,0};
  uint32_t last_ts_ms_ = 0;

  float pos_rad_[W_COUNT]        = {0,0,0,0};
  float vel_raw_rad_s_[W_COUNT]  = {0,0,0,0};
  float vel_filt_rad_s_[W_COUNT] = {0,0,0,0};
  float total_dist_m_[W_COUNT]   = {0,0,0,0};

  // --- filters ---
  VelFilter vfilter_ = VF_NONE;

  // EMA
  float ema_alpha_ = 0.35f;

  // Butterworth (biquad, direct form I): y[n] = b0 x[n] + b1 x[n-1] + b2 x[n-2] - a1 y[n-1] - a2 y[n-2]
  float bw_b0_=1, bw_b1_=0, bw_b2_=0, bw_a1_=0, bw_a2_=0;
  // per-wheel states
  float x1_[W_COUNT] = {0,0,0,0}, x2_[W_COUNT] = {0,0,0,0};
  float y1_[W_COUNT] = {0,0,0,0}, y2_[W_COUNT] = {0,0,0,0};
};
