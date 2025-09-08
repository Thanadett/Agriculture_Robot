#ifdef Node2
// # เทอร์มินัล A
// ros2 topic echo /enc/joint_states
// # เทอร์มินัล B
// ros2 topic echo /enc/total

//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp_node2 -v6
#include <Arduino.h>
#include "encoder_read.h"

// from encoder_ros_pub.cpp
void enc_microros_begin_serial();
void enc_microros_spin_some();

// Global 4-wheel encoder
QuadEncoderReader enc4;

void setup() {
  Serial.begin(115200);
  delay(200);

  enc4.begin(true);
  enc4.setInvert(false, false, false, false); // ปรับตามการเดินสายจริง
  enc4.setWheelRadius(0.0635f);               // ปรับตามล้อจริง
  // enc4.setPPR(<your_PPR_output_shaft_if_diff>);

  enc_microros_begin_serial();
}

void loop() {
  enc4.update();
  enc_microros_spin_some();
  delay(2);
}
#endif