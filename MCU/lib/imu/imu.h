#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <MadgwickAHRS.h>
#include <Adafruit_Sensor.h>

class IMU_MPU6050
{
public:
    // ======== User Config ========
    static constexpr float SAMPLE_HZ = 200.0f;
    static constexpr uint32_t CALIB_MS = 3000;     // stationary calibration time
    static constexpr float BETA = 0.05f;           // Madgwick beta for 200 Hz
    static constexpr float GYRO_STILL_THR = 0.02f; // rad/s
    static constexpr float ACC_NORM_THR_G = 0.10f;
    static constexpr float BIAS_ALPHA = 0.0015f;

    // ======== Data Outputs ========
    float roll_deg{0}, pitch_deg{0}, yaw_deg{0};
    float yaw_rate_body_rad{0};  // r (corrected)
    float yaw_rate_world_rad{0}; // ψ̇ (corrected for roll/pitch)

    IMU_MPU6050();
    bool begin();
    void update();
    void printDebug(uint32_t print_ms = 100);

private:
    Adafruit_MPU6050 mpu_;
    Madgwick filter_;

    // static bias from boot calibration
    float gyro_bx_{0}, gyro_by_{0}, gyro_bz_{0};
    float accel_bx_{0}, accel_by_{0}, accel_bz_{0};

    // online bias estimate for gz
    float gzb_est_{0};

    // time keeping
    uint32_t last_us_{0};
    uint32_t last_print_{0};
    const uint32_t period_us_ = static_cast<uint32_t>(1e6f / SAMPLE_HZ);

    void calibrate(uint32_t ms);
};
