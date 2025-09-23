#pragma once

#include <Arduino.h>
#include "config.h" // for WheelPins, PWM constants
#include <cstdint>

/**
 * @class MotorDriver
 * @brief High-level motor driver for four-wheel robot.
 *
 * Provides per-wheel speed commands, emergency stop, and test routines.
 */
class MotorDriver
{
public:
    MotorDriver();
    ~MotorDriver();

    /**
     * @brief Initialize LEDC channels and stop all motors.
     * @return true if initialization succeeds.
     */
    bool init();

    /**
     * @brief Immediately set a normalized speed [-1.0,1.0] for a wheel.
     * @param wheel_index 0=FL, 1=RL, 2=FR, 3=RR
     * @param speed normalized duty (-1=full reverse, +1=full forward)
     */
    void setMotorSpeed(int wheel_index, float speed);

    /**
     * @brief Set normalized speeds for all 4 wheels at once.
     */
    void setAllMotorSpeeds(float fl, float rl, float fr, float rr);

    /**
     * @brief Set a raw PWM duty [0..PWM_MAX] with sign taken from direction.
     * Converts to normalized range internally.
     */
    void setMotorPWM(int wheel_index, int pwm_value);

    /**
     * @brief Stop all motors (safe normal stop).
     */
    void stop();

    /**
     * @brief Hard emergency stop: force PWM to 0 immediately.
     */
    void emergencyStop();

    /**
     * @brief Get current normalized speed command of a wheel.
     */
    float getCurrentSpeed(int wheel_index) const;

    /**
     * @brief Print current driver state and wheel PWM values.
     */
    void printStatus() const;

    /**
     * @brief Run a simple test sequence for all motors (forward/reverse).
     * Blocks while running.
     */
    void testMotors();

private:
    /**
     * @brief Configure all LEDC channels and attach pins.
     */
    void setupLEDC();

    /**
     * @brief Low-level PWM setter with sign handling and clamping.
     * Called by setMotorSpeed().
     */
    void setPWM(int wheel_index, float pwm_value);

private:
    bool is_initialized;
    float current_pwm[4]; ///< Last commanded normalized duty for each motor
};
