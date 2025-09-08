//ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp_node2 --ros-domain-id 69 


#ifdef Node2
#include <Arduino.h>
#include "encoder_read.h"
#include "encoder_ros_pub.h"

QuadEncoderReader enc4;

void setup() {
  Serial.begin(115200);
  delay(200);

  enc4.begin(true);
  enc4.setInvert(false,false,false,false);
  enc4.setWheelRadius(0.0635f);

  enc_agent_watchdog_begin();
  while (!enc_agent_check_and_reconnect()) { delay(200); }
  
}

void loop() {
  enc4.update();

  // ถ้า agent หลุด ฟังก์ชันนี้จะพยายามเชื่อมใหม่ให้เอง
  bool connected = enc_agent_check_and_reconnect();

  if (connected) {
    enc_microros_spin_some();
  }

  delay(2);
}
#endif
