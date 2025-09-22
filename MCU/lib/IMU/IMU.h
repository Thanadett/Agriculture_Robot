#pragma once
#include <Arduino.h>

class IMU {
public:
  bool begin();
  void update(float dt);
  float getYawDeg() const;

private:
  // Complementary filter yaw (degrees). Start encoder-only mode by returning 0 if fusion weight is 0.
  float yaw_deg_ = 0.0f;
  bool  ok_ = false;

  // Raw/estimation states (placeholders for MPU6050 driver)
  float gyro_z_dps_ = 0.0f;
  float acc_roll_deg_ = 0.0f, acc_pitch_deg_ = 0.0f; // not used for yaw; kept for future
};
