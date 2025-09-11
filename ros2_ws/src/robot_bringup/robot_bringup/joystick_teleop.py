#!/usr/bin/env python3
# ROS2 Joystick → /cmd_vel (RB = instant max; A = emergency stop)
# Left stick Y  -> linear.x (เดินหน้า/ถอย)
# Right stick X -> angular.z (เลี้ยวซ้าย/ขวา)

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def deadzone_map(x: float, dz: float) -> float:
    if abs(x) < dz:
        return 0.0
    y = (abs(x) - dz) / (1.0 - dz)
    y = max(0.0, min(1.0, y))
    return math.copysign(y, x)


def expo(x: float, k: float) -> float:
    # k ∈ [0..1] (0 = linear, 1 = โค้งมาก)
    return (1.0 - k) * x + k * (x ** 3)


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop_stick')

        # ---------- Parameters ----------
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # two-stick mapping
        # throttle (เดินหน้า/ถอย)
        self.declare_parameter('axis_left_y', 1)
        # steering (เลี้ยวซ้าย/ขวา)
        self.declare_parameter('axis_right_x', 3)
        self.declare_parameter('invert_left_y', 1.0)    # 1 หรือ -1
        self.declare_parameter('invert_right_x', 1.0)   # 1 หรือ -1

        # buttons  (RB = turbo, A = emergency stop)
        self.declare_parameter('btn_turbo', 5)          # RB
        self.declare_parameter('btn_emergency_stop', 4) # LB

        # tuning
        self.declare_parameter('max_linear', 255.0)       # m/s
        self.declare_parameter('max_angular', 255.0)     # rad/s
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('expo_linear', 3.0)
        self.declare_parameter('expo_angular', 2.5)

        self.declare_parameter('ramp_rate_linear', 250.0) 
        self.declare_parameter('ramp_rate_angular', 250.0)
        self.declare_parameter('joy_timeout_ms', 800)

        #---------------- Servo buttons ----------------
        # self.declare_parameter('btn_a', 0) # A
        # self.declare_parameter('btn_b', 1) # B
        # self.declare_parameter('btn_x', 2) # X

        # ---------- Read params ----------
        def p(k): return self.get_parameter(k).get_parameter_value()
        joy_topic = p('joy_topic').string_value
        cmd_vel_topic = p('cmd_vel_topic').string_value

        self.ax_ly = int(p('axis_left_y').integer_value)
        self.ax_rx = int(p('axis_right_x').integer_value)
        self.inv_ly = float(p('invert_left_y').double_value)
        self.inv_rx = float(p('invert_right_x').double_value)

        self.btn_turbo = int(p('btn_turbo').integer_value)
        self.btn_estop = int(p('btn_emergency_stop').integer_value)

        self.max_lin = float(p('max_linear').double_value)
        self.max_ang = float(p('max_angular').double_value)
        self.deadzone = float(p('deadzone').double_value)
        self.exp_lin = float(p('expo_linear').double_value)
        self.exp_ang = float(p('expo_angular').double_value)
        self.ramp_lin = float(p('ramp_rate_linear').double_value)
        self.ramp_ang = float(p('ramp_rate_angular').double_value)
        self.joy_to_ms = int(p('joy_timeout_ms').integer_value)

    
        # self.btn_a = int(p('btn_a').integer_value)
        # self.btn_b = int(p('btn_b').integer_value)
        # self.btn_x = int(p('btn_x').integer_value)


        # ---------- Pub/Sub ----------
        cmd_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_cmd = self.create_publisher(
            Twist, cmd_vel_topic, qos_profile=cmd_qos)
        self.sub_joy = self.create_subscription(
            Joy, joy_topic, self.cb_joy, qos_profile=qos_profile_sensor_data)
        # servo ctrl publisher
        # self.pub_servo = self.create_publisher(String, '/servo_cmd', qos_profile=cmd_qos)


        # ---------- State ----------
        self.last_joy_time = 0.0
        self.turbo_active = False
        self._btn_last_estop = False

        self.cur_v = 0.0
        self.cur_w = 0.0
        self.v_target = 0.0
        self.w_target = 0.0
        self.t_last = time.monotonic()

        # ---------- button state ----------
        # self._prev_btn_a = 0 #previous state
        # self._prev_btn_b = 0
        # self._prev_btn_x = 0


        # ส่งที่ ~50 Hz
        self.timer = self.create_timer(0.02, self.tick)

        self.get_logger().info(
            f'Joystick started: joy={joy_topic} -> cmd={cmd_vel_topic} | '
        )

    # -------- helpers --------
    @staticmethod
    def _btn(msg, idx):
        return 1 if (0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1) else 0

    @staticmethod
    def _axis(msg, idx):
        try:
            return float(msg.axes[idx])
        except Exception:
            return 0.0

    # -------- callbacks --------
    def cb_joy(self, msg: Joy):
        now = time.monotonic()
        self.last_joy_time = now

        # ---------- publish button events ----------
        btn_a = self._btn(msg, self.btn_a)
        btn_b = self._btn(msg, self.btn_b)
        btn_x = self._btn(msg, self.btn_x)

        # A: press/release
        if btn_a == 1 and self._prev_btn_a == 0:
            self.pub_button.publish(String(data='BTN A=DOWN'))
        elif btn_a == 0 and self._prev_btn_a == 1:
            self.pub_button.publish(String(data='BTN A=UP'))
        # B: press/release
        if btn_b == 1 and self._prev_btn_b == 0:
            self.pub_button.publish(String(data='BTN B=DOWN'))
        elif btn_b == 0 and self._prev_btn_b == 1:
            self.pub_button.publish(String(data='BTN B=UP'))
        # X: press/release
        if btn_x == 1 and self._prev_btn_x == 0:
            self.pub_button.publish(String(data='BTN X=DOWN'))
        elif btn_x == 0 and self._prev_btn_x == 1:
            self.pub_button.publish(String(data='BTN X=UP'))

        # update previous states
        self._prev_btn_a = btn_a
        self._prev_btn_b = btn_b
        self._prev_btn_x = btn_x

        if self._btn(msg, self.btn_estop):
            self.v_target = 0.0
            self.w_target = 0.0
            self.turbo_active = False
            self._btn_last_estop = True
            return
        else:
            self._btn_last_estop = False

        # อ่านแกน
        ly = self._axis(msg, self.ax_ly) * self.inv_ly     # throttle
        rx = self._axis(msg, self.ax_rx) * self.inv_rx     # steering

        # deadzone + expo
        lin_in = expo(deadzone_map(ly, self.deadzone), self.exp_lin)   # -1..1
        ang_in = expo(deadzone_map(rx, self.deadzone), self.exp_ang)   # -1..1

        # ปกติ: สเกลตามแกน (แปรผันถึงค่าสูงสุด)
        v_norm = lin_in * self.max_lin
        w_norm = ang_in * self.max_ang

        # RB = instant max 
        self.turbo_active = bool(self._btn(msg, self.btn_turbo))
        if self.turbo_active:
            # ถ้าแกนเป็น 0 -> คงที่ 0
            self.v_target = (math.copysign(self.max_lin, lin_in)
                             if abs(lin_in) > 0.0 else 0.0)
            self.w_target = (math.copysign(self.max_ang, ang_in)
                             if abs(ang_in) > 0.0 else 0.0)
        else:
            self.v_target = v_norm
            self.w_target = w_norm

    # -------- main loop --------
    def tick(self):
        now = time.monotonic()
        dt = now - self.t_last
        self.t_last = now

        # ถ้าไม่มีสัญญาณจอยนานเกิน -> หยุด
        if (now - self.last_joy_time) * 1000.0 > self.joy_to_ms:
            self.v_target = 0.0
            self.w_target = 0.0
            self.turbo_active = False

        if self._btn_last_estop: 
            self.cur_v = 0.0
            self.cur_w = 0.0
        elif self.turbo_active:
            self.cur_v = self.v_target
            self.cur_w = self.w_target

        else:
            max_step_v = max(1e-6, self.ramp_lin * dt)
            max_step_w = max(1e-6, self.ramp_ang * dt)
            dv = max(-max_step_v, min(max_step_v, self.v_target - self.cur_v))
            dw = max(-max_step_w, min(max_step_w, self.w_target - self.cur_w))
            self.cur_v += dv
            self.cur_w += dw

        # publish
        msg = Twist()
        msg.linear.x = self.cur_v
        msg.angular.z = self.cur_w
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
