#include "IMU.h"
#include "config.h"
#include <Wire.h>

// NOTE: For production, integrate a proven MPU6050 lib (e.g., Jeff Rowberg) and apply DLPF settings.

bool IMU::begin(){
  // Define I2C_SDA and I2C_SCL if not defined elsewhere, or use default pins:
  Wire.begin(); // Uses default I2C pins for your board
  // Init MPU6050 here (omitted for brevity). Calibrate gyro bias using IMU_CALIB_SAMPLES.
  ok_ = true;
  yaw_deg_ = 0.0f;
  return ok_;
}

void IMU::update(float dt){
  if(!ok_) return;
  // Read gyro Z, integrate. Placeholder:
  // gyro_z_dps_ = readGyroZ();  // TODO: replace with real driver read
  // yaw_deg_ += gyro_z_dps_ * dt;

  // If COMP_ALPHA < 1, fuse with yaw-from-mag or external ref; for now, encoder-only start => keep yaw_deg_ as is.
}

float IMU::getYawDeg() const { return yaw_deg_; }
