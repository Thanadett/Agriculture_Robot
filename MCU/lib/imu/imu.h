#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../logger/logger.h"

// MPU6050 complementary filter (yaw = gyro integration w/ bias removal)
class IMU6050CF
{
public:
    bool begin(int sda, int scl, int calib_samples)
    {
        Wire.begin(sda, scl);
        if (!mpu.begin())
        {
            logger.error("MPU6050 not found");
            return false;
        }
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        calibrateGyro(calib_samples);
        last_ms_ = millis();
        return true;
    }

    // Returns yaw (deg). Roll/pitch computed but not returned here.
    float update()
    {
        sensors_event_t a, g, t;
        mpu.getEvent(&a, &g, &t);
        uint32_t now = millis();
        float dt = (now - last_ms_) * 1e-3f;
        if (dt <= 0)
            dt = 1e-3f;
        last_ms_ = now;

        // Gyro bias compensated
        float gz = (g.gyro.z * RAD_TO_DEG) - gyro_bias_z_deg_s_;
        yaw_deg_ += gz * dt; // ไม่มี magnetometer → มี drift ได้
        // keep in [-180,180]
        if (yaw_deg_ > 180)
            yaw_deg_ -= 360;
        else if (yaw_deg_ < -180)
            yaw_deg_ += 360;
        return yaw_deg_;
    }

    float yaw_deg() const { return yaw_deg_; }

private:
    Adafruit_MPU6050 mpu;
    uint32_t last_ms_ = 0;
    float yaw_deg_ = 0;
    float gyro_bias_z_deg_s_ = 0;

    void calibrateGyro(int n)
    {
        logger.info("Calibrating gyro: %d samples", n);
        float sum = 0;
        for (int i = 0; i < n; i++)
        {
            sensors_event_t a, g, t;
            mpu.getEvent(&a, &g, &t);
            sum += g.gyro.z * RAD_TO_DEG;
            delay(2);
        }
        gyro_bias_z_deg_s_ = sum / n;
        logger.info("Gyro Z bias = %.3f deg/s", gyro_bias_z_deg_s_);
    }
};