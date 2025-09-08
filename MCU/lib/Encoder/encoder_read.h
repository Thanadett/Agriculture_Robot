#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

#define ENC_R_A 16
#define ENC_R_B 17
#define ENC_F_A 5
#define ENC_F_B 18

#define ENCODER_PPR_MOTOR 11.0f
#define REDUCTION_RATIO   270.0f
#define QUAD_MULTIPLIER   2.0f
#define ENC_WHEEL_RADIUS  0.0635f

static constexpr float ENCODER_PPR_OUTPUT_DEFAULT =
    ENCODER_PPR_MOTOR * REDUCTION_RATIO * QUAD_MULTIPLIER;

class DualEncoderReader {
public:
  DualEncoderReader(int rearA = ENC_R_A, int rearB = ENC_R_B,
                    int frontA = ENC_F_A, int frontB = ENC_F_B,
                    float pulses_per_rev_output = ENCODER_PPR_OUTPUT_DEFAULT);

  void begin(bool enable_internal_pullups = true);
  void update();
  void reset();

  // --- getters ---
  float positionRearRad()  const { return posR_rad_; }
  float positionFrontRad() const { return posF_rad_; }
  float velocityRearRad()  const { return velR_rad_s_; }
  float velocityFrontRad() const { return velF_rad_s_; }
  long  countsRear()              { return encR_.getCount(); }
  long  countsFront()             { return encF_.getCount(); }

  void  setPPR(float pulses_per_rev_output) { ppr_out_ = pulses_per_rev_output; }
  float getPPR() const { return ppr_out_; }

  void setInvert(bool invert_rear, bool invert_front) {
    invR_ = invert_rear  ? -1 : 1;
    invF_ = invert_front ? -1 : 1;
  }

  void  setWheelRadius(float radius_m) { wheel_radius_m_ = radius_m; }
  float wheelRadius() const            { return wheel_radius_m_; }

  float distanceRearM()  const { return wheel_radius_m_ * posR_rad_; }
  float distanceFrontM() const { return wheel_radius_m_ * posF_rad_; }

  float totalDistanceRearM()  const { return total_dist_R_m_; }
  float totalDistanceFrontM() const { return total_dist_F_m_; }

  float circumferenceM() const;
  float remainingToNextRevRearM()  const;
  float remainingToNextRevFrontM() const;

private:
  int rA_, rB_, fA_, fB_;
  ESP32Encoder encR_, encF_;
  float ppr_out_ = ENCODER_PPR_OUTPUT_DEFAULT;
  int   invR_ = 1, invF_ = 1;

  float wheel_radius_m_ = ENC_WHEEL_RADIUS;

  long     lastR_ = 0, lastF_ = 0;
  uint32_t last_ts_ms_ = 0;

  float posR_rad_ = 0.f, posF_rad_ = 0.f;
  float velR_rad_s_ = 0.f, velF_rad_s_ = 0.f;

  long revRear_ = 0, revFront_ = 0;

  float total_dist_R_m_ = 0.f;
  float total_dist_F_m_ = 0.f;
};


