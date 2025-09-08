#ifdef Node2
// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp_node2 
// ros2 topic echo /enc/joint_states
// ros2 topic echo /enc/total

#include <Arduino.h>
#include "encoder_read.h"
#include "encoder_ros_pub.h" 

// ★ เพิ่ม 2 บรรทัดนี้
#include <micro_ros_platformio.h>
extern "C" { 
  #include <rmw_microros/rmw_microros.h> 
}

// from encoder_ros_pub.cpp
//void enc_microros_begin_serial();
//void enc_microros_spin_some();

// Global 4-wheel encoder
QuadEncoderReader enc4;

void setup() {
  Serial.begin(115200);
  delay(200);

  // ★★ ตั้ง transport ให้ micro-ROS ใช้ Serial ตัวนี้
  set_microros_serial_transports(Serial);

  // ★★ รอ agent ให้พร้อมก่อนเริ่ม init node/publisher (100ms x 30 ≈ 3s)
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 30)) {
    delay(100);
  }

  enc4.begin(true);
  enc4.setInvert(false, false, false, false); // ปรับตามการเดินสายจริง
  enc4.setWheelRadius(0.0635f);               // ปรับตามล้อจริง
  // enc4.setPPR(<PPR_output_shaft_if_diff>);

  enc_microros_begin_serial();  // ← จากไฟล์ encoder_ros_pub.cpp
}

void loop() {
  enc4.update();
  enc_microros_spin_some();     // ← ให้ executor spin ส่ง/poll callbacks
  delay(2);
}
#endif
