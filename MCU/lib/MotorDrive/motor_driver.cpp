#include "motor_driver.h"
#include "logger.h"

extern Logger logger;

// Motor pin configurations using WheelPins from config.h
const WheelPins MOTOR_PINS[4] = {FL, RL, FR, RR};

MotorDriver::MotorDriver()
{
  is_initialized = false;
  for (int i = 0; i < 4; i++)
  {
    current_pwm[i] = 0.0f;
  }
}

MotorDriver::~MotorDriver()
{
  stop();
}

bool MotorDriver::init()
{
  logger.info("Initializing Motor Driver...");

  setupLEDC();

  // Initialize all motors to stop
  stop();

  is_initialized = true;
  logger.info("Motor Driver initialized successfully");

  return true;
}

void MotorDriver::setupLEDC()
{
  logger.info("Setting up LEDC PWM channels...");

  for (int i = 0; i < 4; i++)
  {
    const WheelPins &pins = MOTOR_PINS[i];

    // Setup LEDC channels for each motor
    ledcSetup(pins.chanL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(pins.chanR, PWM_FREQ_HZ, PWM_RES_BITS);

    // Attach pins to LEDC channels
    ledcAttachPin(pins.inL, pins.chanL);
    ledcAttachPin(pins.inR, pins.chanR);

    // Initialize to 0 (stopped)
    ledcWrite(pins.chanL, 0);
    ledcWrite(pins.chanR, 0);

    logger.info("Motor %d: IN1=%d (Ch%d), IN2=%d (Ch%d), Sign=%d",
                i, pins.inL, pins.chanL, pins.inR, pins.chanR, pins.dir_sign);
  }
}

void MotorDriver::setPWM(int wheel_index, float pwm_value)
{
  if (wheel_index < 0 || wheel_index >= 4)
    return;

  const WheelPins &pins = MOTOR_PINS[wheel_index];

  // Apply motor direction sign
  pwm_value *= pins.dir_sign;

  // Constrain PWM value
  pwm_value = constrain(pwm_value, -1.0f, 1.0f);

  // Convert to PWM range (0-255)
  int pwm_int = (int)(pwm_value * PWM_MAX);

  if (pwm_int >= 0)
  {
    // Forward direction
    ledcWrite(pins.chanL, pwm_int);
    ledcWrite(pins.chanR, 0);
  }
  else
  {
    // Reverse direction
    ledcWrite(pins.chanL, 0);
    ledcWrite(pins.chanR, -pwm_int);
  }

  current_pwm[wheel_index] = pwm_value;
}

void MotorDriver::setMotorSpeed(int wheel_index, float speed)
{
  if (!is_initialized)
  {
    logger.error("Motor driver not initialized");
    return;
  }

  // Constrain speed to [-1.0, 1.0]
  speed = constrain(speed, -1.0f, 1.0f);

  setPWM(wheel_index, speed);
}

void MotorDriver::setAllMotorSpeeds(float fl, float rl, float fr, float rr)
{
  setMotorSpeed(0, fl); // Front Left
  setMotorSpeed(1, rl); // Rear Left
  setMotorSpeed(2, fr); // Front Right
  setMotorSpeed(3, rr); // Rear Right
}

void MotorDriver::setMotorPWM(int wheel_index, int pwm_value)
{
  if (!is_initialized)
  {
    logger.error("Motor driver not initialized");
    return;
  }

  // Convert PWM value to speed (-1.0 to 1.0)
  float speed = (float)pwm_value / PWM_MAX;
  setMotorSpeed(wheel_index, speed);
}

void MotorDriver::stop()
{
  for (int i = 0; i < 4; i++)
  {
    setPWM(i, 0.0f);
  }
  logger.info("All motors stopped");
}

void MotorDriver::emergencyStop()
{
  // Immediately stop all motors
  for (int i = 0; i < 4; i++)
  {
    const WheelPins &pins = MOTOR_PINS[i];
    ledcWrite(pins.chanL, 0);
    ledcWrite(pins.chanR, 0);
    current_pwm[i] = 0.0f;
  }
  logger.warn("EMERGENCY STOP activated");
}

float MotorDriver::getCurrentSpeed(int wheel_index) const
{
  if (wheel_index < 0 || wheel_index >= 4)
    return 0.0f;
  return current_pwm[wheel_index];
}

void MotorDriver::printStatus() const
{
  logger.info("Motor Driver Status:");
  logger.info("  Initialized: %s", is_initialized ? "YES" : "NO");
  for (int i = 0; i < 4; i++)
  {
    logger.info("  Motor %d: PWM=%.3f (%.1f%%)", i, current_pwm[i], current_pwm[i] * 100.0f);
  }
}

void MotorDriver::testMotors()
{
  if (!is_initialized)
  {
    logger.error("Cannot test motors - driver not initialized");
    return;
  }

  logger.info("Starting motor test sequence...");

  const float test_speed = 0.3f;  // 30% power
  const int test_duration = 2000; // 2 seconds per motor

  // Test each motor individually
  for (int i = 0; i < 4; i++)
  {
    const char *motor_names[] = {"Front Left", "Rear Left", "Front Right", "Rear Right"};

    logger.info("Testing %s motor (forward)...", motor_names[i]);
    setMotorSpeed(i, test_speed);
    delay(test_duration);

    logger.info("Testing %s motor (reverse)...", motor_names[i]);
    setMotorSpeed(i, -test_speed);
    delay(test_duration);

    setMotorSpeed(i, 0.0f); // Stop motor
    delay(500);
  }

  // Test all motors together
  logger.info("Testing all motors forward...");
  setAllMotorSpeeds(test_speed, test_speed, test_speed, test_speed);
  delay(test_duration);

  logger.info("Testing all motors reverse...");
  setAllMotorSpeeds(-test_speed, -test_speed, -test_speed, -test_speed);
  delay(test_duration);

  // Stop all motors
  stop();
  logger.info("Motor test sequence completed");
}