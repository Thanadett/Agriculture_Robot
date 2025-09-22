#ifdef Node1
#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "IMU.h"
#include "PID.h"
#include "VelocityController.h"
#include "Logger.h"
#include "micro_ros_if.h"

MotorDriver motor;
Encoder encoder;
IMU imu;
VelocityController vctrl;
Logger logger;
MicroRosIF rosif;

static inline float now_s() { return micros() * 1e-6f; }

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("[BOOT] ESP32 MCU starting...");

  motor.begin();
  encoder.begin();
  imu.begin();
  vctrl.begin(&encoder, &motor);
  logger.begin(true); // Enable logging by default

  rosif.setModules(&vctrl, &encoder, &imu);
  rosif.init();

  Serial.println("[BOOT] Ready.");
}

void loop()
{
  static float t_prev = now_s();
  static float t_ctrl_acc = 0, t_pub_ticks_acc = 0, t_pub_yaw_acc = 0;

  float t_now = now_s();
  float dt = t_now - t_prev;
  if (dt <= 0)
    dt = 1.0f / CONTROL_HZ;
  t_prev = t_now;

  // accumulate timers
  t_ctrl_acc += dt;
  t_pub_ticks_acc += dt;
  t_pub_yaw_acc += dt;

  // executor + pubs handled inside spin_once
  rosif.spin_once(t_now, t_ctrl_acc, t_pub_ticks_acc, t_pub_yaw_acc);

  // 200 Hz control
  if (t_ctrl_acc >= (1.0f / CONTROL_HZ))
  {
    float step = t_ctrl_acc;
    t_ctrl_acc = 0;

    imu.update(step);
    vctrl.update(step);

    logger.logCSV(t_now, vctrl.v_sp(), vctrl.w_sp(), vctrl.vL_meas(), vctrl.vR_meas(),
                  vctrl.pwmL(), vctrl.pwmR(), imu.getYawDeg());
  }
}
#endif