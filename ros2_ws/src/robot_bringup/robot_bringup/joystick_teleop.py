#!/usr/bin/env python3
# ROS2 Joystick → /cmd_vel  (safe, smooth, param-driven)
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def apply_deadzone(x: float, dz: float) -> float:
    if abs(x) < dz:
        return 0.0
    s = (abs(x) - dz) / (1.0 - dz)
    s = max(0.0, min(1.0, s))
    return math.copysign(s, x)


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # Parameters
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('max_linear', 1.0)      # m/s
        self.declare_parameter('max_angular', 2.5)     # rad/s
        self.declare_parameter('turbo_multiplier', 1.4)

        self.declare_parameter('axis_linear', 1)       # left stick Y
        self.declare_parameter('axis_angular', 0)      # left stick X
        self.declare_parameter('invert_linear', 1.0)   # 1 or -1
        self.declare_parameter('invert_angular', 1.0)  # 1 or -1

        self.declare_parameter('require_enable_button', False)
        self.declare_parameter('enable_button', 4)     # LB
        self.declare_parameter('turbo_button', 5)      # RB
        self.declare_parameter('estop_button', -1)     # disabled by default

        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('ramp_rate', 10.0)       # [1/s]
        self.declare_parameter('joy_timeout_ms', 2000)

        joy_topic = self.get_parameter(
            'joy_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.turbo_mul = float(self.get_parameter('turbo_multiplier').value)

        self.axis_lin = int(self.get_parameter('axis_linear').value)
        self.axis_ang = int(self.get_parameter('axis_angular').value)
        self.inv_lin = float(self.get_parameter('invert_linear').value)
        self.inv_ang = float(self.get_parameter('invert_angular').value)

        self.req_enable = bool(self.get_parameter(
            'require_enable_button').value)
        self.btn_enable = int(self.get_parameter('enable_button').value)
        self.btn_turbo = int(self.get_parameter('turbo_button').value)
        self.btn_estop = int(self.get_parameter('estop_button').value)

        self.deadzone = float(self.get_parameter('deadzone').value)
        self.ramp_rate = float(self.get_parameter('ramp_rate').value)
        self.joy_timeout = int(self.get_parameter('joy_timeout_ms').value)

        # Publishers / Subscribers
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_cmd = self.create_publisher(
            Twist, cmd_vel_topic, qos_profile=cmd_qos)
        self.sub_joy = self.create_subscription(
            Joy, joy_topic, self.cb_joy, qos_profile=qos_profile_sensor_data)

        # State
        self.last_joy_time = 0.0
        self.enabled = False
        self.turbo = False
        self.estop = False

        self.cur_v = 0.0
        self.cur_w = 0.0
        self.v_target = 0.0
        self.w_target = 0.0
        self.t_last = time.monotonic()

        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        self.get_logger().info(
            f'Joystick teleop started: joy={joy_topic} -> cmd={cmd_vel_topic}')

    def cb_joy(self, msg: Joy):
        now = time.monotonic()
        self.last_joy_time = now

        def safe_axis(axes, idx):
            try:
                return float(axes[idx])
            except Exception:
                return 0.0

        lx = safe_axis(msg.axes, self.axis_ang)  # turn
        ly = safe_axis(msg.axes, self.axis_lin)  # forward/back

        ang_in = apply_deadzone(lx, self.deadzone) * self.inv_ang
        lin_in = apply_deadzone(ly, self.deadzone) * self.inv_lin

        def btn(i):
            if i < 0 or i >= len(msg.buttons):
                return 0
            return int(msg.buttons[i])

        enable_pressed = btn(self.btn_enable) if self.btn_enable >= 0 else 1
        turbo_pressed = btn(self.btn_turbo) if self.btn_turbo >= 0 else 0
        estop_pressed = btn(self.btn_estop) if self.btn_estop >= 0 else 0

        self.enabled = (enable_pressed == 1) if self.req_enable else True
        self.turbo = (turbo_pressed == 1)

        if estop_pressed:
            self.estop = True

        if self.enabled and not self.estop:
            lin_scale = self.max_linear * \
                (self.turbo_mul if self.turbo else 1.0)
            ang_scale = self.max_angular * \
                (self.turbo_mul if self.turbo else 1.0)
            self.v_target = float(lin_in * lin_scale)
            self.w_target = float(ang_in * ang_scale)
        else:
            self.v_target = 0.0
            self.w_target = 0.0

    def tick(self):
        now = time.monotonic()
        dt = now - self.t_last
        self.t_last = now

        if (now - self.last_joy_time) * 1000.0 > self.joy_timeout:
            self.v_target = 0.0
            self.w_target = 0.0

        max_step = max(1e-6, self.ramp_rate * dt)
        dv = max(-max_step, min(max_step, self.v_target - self.cur_v))
        dw = max(-max_step, min(max_step, self.w_target - self.cur_w))
        self.cur_v += dv
        self.cur_w += dw

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
