#pragma once
#include <Arduino.h>
#include "config.h"
#include "Wheel.h"

class MotorDriver
{
public:
    void begin();
    // pwm in [-1.0, 1.0], signed by desired wheel direction; internally mapped to H-bridge pins.
    void setPWM(Wheel w, float pwm01);
    void enable(bool en);

private:
    void applyPWM(const WheelPins &wp, float pwm01);
    bool enabled_ = true;
};
