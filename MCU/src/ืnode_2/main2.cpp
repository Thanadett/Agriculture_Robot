#ifdef Node2
#include <Arduino.h>
#include "encoder_read.h"

// from encoder_ros_pub.cpp
void enc_microros_begin_serial();
void enc_microros_spin_some();

// Global encoder
DualEncoderReader enc;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Init encoder
  enc.begin(true);
  enc.setInvert(false, false);    // ปรับตามการเดินสายจริง
  enc.setWheelRadius(0.0635f);    // ปรับตามล้อจริง ถ้าไม่ใช่ 63.5 mm

  // Start micro-ROS (Serial transport)
  enc_microros_begin_serial();
}

void loop() {
  enc.update();              // refresh encoder math
  enc_microros_spin_some();  // publish to ROS 2
  delay(2);
}
#endif
