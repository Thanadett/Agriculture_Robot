#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Joystick axis -> STP button style
- Read /joy (sensor_msgs/Joy)
- Use one axis (default: index 7) as a virtual button
- Emit STP lines to std_msgs/String topic
  * axis >= +threshold -> STP DOWN=DOWN
  * axis <= -threshold -> STP UP=DOWN
  * return to neutral  -> STP DOWN=UP  and STP UP=UP
ESP32 side (stepper_handle_line) will interpret direction.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class JoystickStepperButton(Node):
    def __init__(self):
        super().__init__('joystick_stepper_button')

        # ---------- Parameters ----------
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('stp_cmd_topic', '/step_cmd')
        self.declare_parameter('axis_index', 7)     # Xbox D-pad vertical = 7
        self.declare_parameter('threshold', 0.5)    # deadzone threshold
        self.declare_parameter('debounce_ms', 20)   # small debounce

        p = lambda k: self.get_parameter(k).value
        self.joy_topic   = str(p('joy_topic'))
        self.stp_topic   = str(p('stp_cmd_topic'))
        self.axis_index  = int(p('axis_index'))
        self.threshold   = float(p('threshold'))
        self.debounce_ms = int(p('debounce_ms'))


        # ---------- Pub/Sub ----------
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_stp = self.create_publisher(String, self.stp_topic, qos_profile=qos_cmd)
        self.sub_joy = self.create_subscription(Joy, self.joy_topic, self.cb_joy,
                                                qos_profile=qos_profile_sensor_data)

        # ---------- State ----------
        self.prev_up = 0
        self.prev_down = 0
        self.last_change_ms = 0.0

        self.get_logger().info(
            f'JoystickStepperButton started: joy="{self.joy_topic}" axis={self.axis_index} '
            f'-> STP topic="{self.stp_topic}"'
        )

    def _emit(self, text: str):
        self.pub_stp.publish(String(data=text))

    def _now_ms(self) -> float:
        return time.monotonic() * 1000.0

    def cb_joy(self, msg: Joy):
        now = self._now_ms()
        if now - self.last_change_ms < self.debounce_ms:
            return

        val = 0.0
        if 0 <= self.axis_index < len(msg.axes):
            val = float(msg.axes[self.axis_index])

        # กด UP ถ้าแกนเป็น -1 (ดันขึ้น) / กด DOWN ถ้า +1 (ดันลง)
        up = 1 if val <= -self.threshold else 0
        down = 1 if val >= self.threshold else 0

        if up != self.prev_up:
            self._emit(f'STP C_Up={"DOWN" if up else "UP"}')
            self.prev_up = up
            self.last_change_ms = now

        if down != self.prev_down:
            self._emit(f'STP C_Dn={"DOWN" if down else "UP"}')
            self.prev_down = down
            self.last_change_ms = now


def main():
    rclpy.init()
    node = JoystickStepperButton()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
