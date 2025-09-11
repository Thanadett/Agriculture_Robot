#include "pid.h"

static inline float clamp_(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi ? hi : x);
}
static inline float clampAbs_(float x, float a)
{
    if (a <= 0)
        return 0;
    if (x > a)
        return a;
    if (x < -a)
        return -a;
    return x;
}

BodyPID::BodyPID(QuadEncoderReader *enc) : enc_(enc) {}

void BodyPID::setGainsLinear(float kp, float ki, float kd)
{
    kp_v_ = kp;
    ki_v_ = ki;
    kd_v_ = kd;
}
void BodyPID::setGainsAngular(float kp, float ki, float kd)
{
    kp_w_ = kp;
    ki_w_ = ki;
    kd_w_ = kd;
}
void BodyPID::setLimits(float v_max, float w_max, float a_v_max, float a_w_max)
{
    v_max_ = fabsf(v_max);
    w_max_ = fabsf(w_max);
    a_v_max_ = fabsf(a_v_max);
    a_w_max_ = fabsf(a_w_max);
}
void BodyPID::setImuWeight(float imu_weight)
{
    if (imu_weight < 0)
        imu_weight = 0;
    if (imu_weight > 1)
        imu_weight = 1;
    imu_w_ = imu_weight;
}
void BodyPID::setFeedforward(float kv, float kw)
{
    kv_ff_ = kv;
    kw_ff_ = kw;
}
void BodyPID::setIClamp(float i_v_abs, float i_w_abs)
{
    i_v_min_ = -fabsf(i_v_abs);
    i_v_max_ = fabsf(i_v_abs);
    i_w_min_ = -fabsf(i_w_abs);
    i_w_max_ = fabsf(i_w_abs);
}
void BodyPID::reset()
{
    i_v_ = 0;
    i_w_ = 0;
    prev_e_v_ = 0;
    prev_e_w_ = 0;
    v_out_ = 0;
    w_out_ = 0;
}

void BodyPID::step(float v_ref, float w_ref, float gz_imu, float dt)
{
    if (!enc_ || dt <= 0)
        return;

    float v_enc = 0, w_enc = 0;
    enc_->bodyTwistFromWheels(v_enc, w_enc);

    const float w_meas = (1.0f - imu_w_) * w_enc + imu_w_ * gz_imu;
    const float v_meas = v_enc;

    const float e_v = v_ref - v_meas;
    const float e_w = w_ref - w_meas;

    const float dv = (e_v - prev_e_v_) / dt;
    float u_v_cand = kv_ff_ * v_ref + kp_v_ * e_v + i_v_ + kd_v_ * dv;
    const bool sat_v = (fabsf(u_v_cand) > (v_max_ + 1e-6f));
    if (!sat_v || (u_v_cand * e_v) < 0.0f)
    {
        i_v_ += ki_v_ * e_v * dt;
        i_v_ = clamp_(i_v_, i_v_min_, i_v_max_);
    }
    float u_v = kv_ff_ * v_ref + kp_v_ * e_v + i_v_ + kd_v_ * dv;

    const float dw = (e_w - prev_e_w_) / dt;
    float u_w_cand = kw_ff_ * w_ref + kp_w_ * e_w + i_w_ + kd_w_ * dw;
    const bool sat_w = (fabsf(u_w_cand) > (w_max_ + 1e-6f));
    if (!sat_w || (u_w_cand * e_w) < 0.0f)
    {
        i_w_ += ki_w_ * e_w * dt;
        i_w_ = clamp_(i_w_, i_w_min_, i_w_max_);
    }
    float u_w = kw_ff_ * w_ref + kp_w_ * e_w + i_w_ + kd_w_ * dw;

    const float dv_cmd = clampAbs_(u_v - v_out_, a_v_max_ * dt);
    const float dw_cmd = clampAbs_(u_w - w_out_, a_w_max_ * dt);
    v_out_ += dv_cmd;
    w_out_ += dw_cmd;

    v_out_ = clamp_(v_out_, -v_max_, v_max_);
    w_out_ = clamp_(w_out_, -w_max_, w_max_);

    cmdVW_to_targets(v_out_, w_out_);

    prev_e_v_ = e_v;
    prev_e_w_ = e_w;
}
