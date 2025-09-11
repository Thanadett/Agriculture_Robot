#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoystickButtons(Node):
    def __init__(self):
        super().__init__('joystick_buttons')

        # ---- Parameters ----
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('servo_cmd_topic', '/servo_cmd')
        self.declare_parameter('btn_a', 0)  # Xbox: A=0
        self.declare_parameter('btn_b', 1)  # Xbox: B=1
        self.declare_parameter('btn_x', 2)  # Xbox: X=2
        self.declare_parameter('debounce_ms', 20)  # กันสั่นเล็กน้อย

        p = lambda k: self.get_parameter(k).get_parameter_value()
        joy_topic = p('joy_topic').string_value
        servo_topic = p('servo_cmd_topic').string_value
        self.idx_a = int(p('btn_a').integer_value)
        self.idx_b = int(p('btn_b').integer_value)
        self.idx_x = int(p('btn_x').integer_value)
        self.debounce_ms = int(p('debounce_ms').integer_value)

        # ---- Pub/Sub ----
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KeepLast, depth=10)
        self.pub_servo = self.create_publisher(String, servo_topic, qos_profile=qos)
        self.sub_joy = self.create_subscription(Joy, joy_topic, self.cb_joy,
                                                qos_profile=qos_profile_sensor_data)

        # ---- State ----
        self.prev_a = 0
        self.prev_b = 0
        self.prev_x = 0
        self.last_change_ms = 0.0

        self.get_logger().info(f'Buttons node started: joy={joy_topic} -> servo_cmd={servo_topic}')

    @staticmethod
    def _btn(msg, idx):
        return 1 if (0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1) else 0

    def _emit(self, text: str):
        self.pub_servo.publish(String(data=text))
        # self.get_logger().info(f'Emit: {text}')

    def cb_joy(self, msg: Joy):
        now_ms = time.monotonic() * 1000.0
        if now_ms - self.last_change_ms < self.debounce_ms:
            return

        a = self._btn(msg, self.idx_a)
        b = self._btn(msg, self.idx_b)
        x = self._btn(msg, self.idx_x)

        if a != self.prev_a:
            self._emit(f'BTN A={"DOWN" if a else "UP"}')
            self.prev_a = a
            self.last_change_ms = now_ms

        if b != self.prev_b:
            self._emit(f'BTN B={"DOWN" if b else "UP"}')
            self.prev_b = b
            self.last_change_ms = now_ms

        if x != self.prev_x:
            self._emit(f'BTN X={"DOWN" if x else "UP"}')
            self.prev_x = x
            self.last_change_ms = now_ms


def main():
    rclpy.init()
    node = JoystickButtons()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
