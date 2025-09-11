#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "encoder_read.h"
#include "motorDrive.h"

class BodyPID
{
public:
    explicit BodyPID(QuadEncoderReader *enc);

    void setGainsLinear(float kp, float ki, float kd = 0.0f);
    void setGainsAngular(float kp, float ki, float kd = 0.0f);
    void setLimits(float v_max, float w_max, float a_v_max, float a_w_max);
    void setImuWeight(float imu_weight);
    void setFeedforward(float kv = 1.0f, float kw = 1.0f);
    void setIClamp(float i_v_abs, float i_w_abs);
    void reset();

    // เรียกทุก dt วินาที, ส่ง out เข้า motorDrive ให้เอง
    void step(float v_ref, float w_ref, float gz_imu, float dt);

    float v_out() const { return v_out_; }
    float w_out() const { return w_out_; }
    float i_v() const { return i_v_; }
    float i_w() const { return i_w_; }

private:
    QuadEncoderReader *enc_ = nullptr;

    float kp_v_ = BODY_KP_V, ki_v_ = BODY_KI_V, kd_v_ = BODY_KD_V;
    float kp_w_ = BODY_KP_W, ki_w_ = BODY_KI_W, kd_w_ = BODY_KD_W;

    float kv_ff_ = 1.0f, kw_ff_ = 1.0f;

    float i_v_ = 0, i_w_ = 0;
    float i_v_min_ = -BODY_I_V_ABS, i_v_max_ = BODY_I_V_ABS;
    float i_w_min_ = -BODY_I_W_ABS, i_w_max_ = BODY_I_W_ABS;

    float v_max_ = BODY_V_MAX;
    float w_max_ = BODY_W_MAX;
    float a_v_max_ = BODY_AV_MAX;
    float a_w_max_ = BODY_AW_MAX;

    float imu_w_ = BODY_IMU_WEIGHT;

    float prev_e_v_ = 0, prev_e_w_ = 0;
    float v_out_ = 0, w_out_ = 0;
};
