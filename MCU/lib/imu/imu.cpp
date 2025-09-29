#include "imu.h"
#include <math.h>

static inline float deg2rad(float d) { return d * PI / 180.0f; }

IMU_MPU6050::IMU_MPU6050() {}

bool IMU_MPU6050::begin()
{
    if (!mpu_.begin())
        return false;

    // Configure ranges and bandwidth (recommended defaults)
    mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu_.setGyroRange(MPU6050_RANGE_1000_DEG);
    mpu_.setFilterBandwidth(MPU6050_BAND_44_HZ);
    mpu_.setSampleRateDivisor(4); // 1kHz/(1+4)=200Hz
    Wire.setClock(400000);

    calibrate(CALIB_MS);

    filter_.begin(SAMPLE_HZ);
    // if Madgwick library supports setBeta:
    // filter_.setBeta(BETA);

    // Seed orientation with gravity only
    sensors_event_t a, g, t;
    mpu_.getEvent(&a, &g, &t);
    const float ax_g = (a.acceleration.x - accel_bx_) / 9.80665f;
    const float ay_g = (a.acceleration.y - accel_by_) / 9.80665f;
    const float az_g = (a.acceleration.z - accel_bz_) / 9.80665f;
    filter_.updateIMU(0, 0, 0, ax_g, ay_g, az_g);

    last_us_ = micros();
    return true;
}

void IMU_MPU6050::calibrate(uint32_t ms)
{
    Serial.println(F("[CAL] Keep vehicle flat & still..."));
    sensors_event_t a, g, t;
    double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    const uint32_t t0 = millis();
    uint32_t n = 0;

    while ((millis() - t0) < ms)
    {
        mpu_.getEvent(&a, &g, &t);
        ax += a.acceleration.x;
        ay += a.acceleration.y;
        az += a.acceleration.z;
        gx += g.gyro.x;
        gy += g.gyro.y;
        gz += g.gyro.z;
        n++;
        delay(2);
    }
    if (n == 0)
        n = 1;
    ax /= n;
    ay /= n;
    az /= n;
    gx /= n;
    gy /= n;
    gz /= n;

    accel_bx_ = ax;
    accel_by_ = ay;
    accel_bz_ = az - 9.80665f; // remove gravity

    gyro_bx_ = gx;
    gyro_by_ = gy;
    gyro_bz_ = gz;

    Serial.printf("[CAL] accel_bias [m/s^2]= %.3f %.3f %.3f\n",
                  accel_bx_, accel_by_, accel_bz_);
    Serial.printf("[CAL] gyro_bias  [rad/s] = %.5f %.5f %.5f\n",
                  gyro_bx_, gyro_by_, gyro_bz_);
    Serial.println(F("[CAL] Done"));
}

void IMU_MPU6050::update()
{
    uint32_t now_us = micros();
    if (now_us - last_us_ < period_us_)
        return;
    while (now_us - last_us_ >= period_us_)
        last_us_ += period_us_;

    sensors_event_t a, g, t;
    mpu_.getEvent(&a, &g, &t);

    // Remove static bias
    float gx = g.gyro.x - gyro_bx_;
    float gy = g.gyro.y - gyro_by_;
    float gz = g.gyro.z - gyro_bz_;

    float ax_g = (a.acceleration.x - accel_bx_) / 9.80665f;
    float ay_g = (a.acceleration.y - accel_by_) / 9.80665f;
    float az_g = (a.acceleration.z - accel_bz_) / 9.80665f;

    // Online bias learning (Z)
    const float a_norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    const bool still_acc = fabsf(a_norm - 1.0f) <= ACC_NORM_THR_G;
    const bool still_gyro = (fabsf(gx) <= GYRO_STILL_THR) &&
                            (fabsf(gy) <= GYRO_STILL_THR) &&
                            (fabsf(gz) <= GYRO_STILL_THR);
    if (still_acc && still_gyro)
    {
        gzb_est_ = (1.0f - BIAS_ALPHA) * gzb_est_ + BIAS_ALPHA * gz;
    }
    const float gz_corr = gz - gzb_est_;

    // Madgwick update
    filter_.updateIMU(gx, gy, gz_corr, ax_g, ay_g, az_g);

    // Euler angles
    roll_deg = filter_.getRoll();
    pitch_deg = filter_.getPitch();
    yaw_deg = filter_.getYaw();

    // Body rates
    yaw_rate_body_rad = gz_corr;

    // World yaw rate (Euler kinematics)
    const float phi = deg2rad(roll_deg);
    const float th = deg2rad(pitch_deg);
    float cth = cosf(th);
    if (fabsf(cth) < 1e-3f)
        cth = (cth >= 0 ? 1e-3f : -1e-3f);
    yaw_rate_world_rad =
        (sinf(phi) / cth) * gy +
        (cosf(phi) / cth) * gz_corr;
}

void IMU_MPU6050::printDebug(uint32_t print_ms)
{
    if (millis() - last_print_ < print_ms)
        return;
    last_print_ = millis();
    Serial.printf("Pitch: % .2f  Roll: % .2f  Yaw: % .2f\n",
                  pitch_deg, roll_deg, yaw_deg);
    Serial.printf("YawRate(body):  % .3f deg/s | YawRate(world): % .3f deg/s\n\n",
                  yaw_rate_body_rad * 180.0f / PI,
                  yaw_rate_world_rad * 180.0f / PI);
}
