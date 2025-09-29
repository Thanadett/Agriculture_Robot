#!/usr/bin/env python3
# wheel_ticks_from_cmdvel.py
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist


class WheelTicksSim(Node):
    def __init__(self):
        super().__init__('wheel_ticks_sim')

        # ---- Parameters (ปรับได้จาก --ros-args -p ...) ----
        self.declare_parameter('wheel_radius_m', 0.0635)
        self.declare_parameter('track_width_m', 0.365)
        self.declare_parameter('ppr_out', 5940.0)
        self.declare_parameter('rate_hz', 50.0)
        # ชื่อเดียวกับ MCU ได้ ถ้าปิด MCU
        self.declare_parameter('ticks_topic', '/wheel_ticks')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('noise_ticks_std', 0.0)

        self.R = float(self.get_parameter('wheel_radius_m').value)
        self.W = float(self.get_parameter('track_width_m').value)
        self.PPR = float(self.get_parameter('ppr_out').value)
        self.HZ = float(self.get_parameter('rate_hz').value)
        self.ticks_topic = self.get_parameter(
            'ticks_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter(
            'cmd_vel_topic').get_parameter_value().string_value
        self.noise_std = float(self.get_parameter('noise_ticks_std').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub_cmd = self.create_subscription(
            Twist, self.cmd_topic, self.cb_cmd, qos)
        self.pub = self.create_publisher(
            Int32MultiArray, self.ticks_topic, qos)

        self.v_cmd, self.w_cmd = 0.0, 0.0
        self.last = self.get_clock().now()

        # cumulative ticks: [FL, FR, RL, RR]
        self.t = [0, 0, 0, 0]
        # fractional accumulators for precise rounding
        self.accL = 0.0
        self.accR = 0.0

        self.circ = 2.0 * math.pi * self.R
        self.rev_per_m = 1.0 / self.circ

        self.timer = self.create_timer(1.0 / self.HZ, self.step)
        self.get_logger().info(
            f"Sim /wheel_ticks @ {self.HZ:.1f} Hz  (R={self.R:.3f}, W={self.W:.3f}, PPR={self.PPR:.1f})"
        )

    def cb_cmd(self, m: Twist):
        self.v_cmd = float(m.linear.x)
        self.w_cmd = float(m.angular.z)

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-6
        self.last = now

        # diff-drive kinematics
        vL = self.v_cmd - 0.5 * self.w_cmd * self.W
        vR = self.v_cmd + 0.5 * self.w_cmd * self.W

        dsL = vL * dt
        dsR = vR * dt

        # meters -> revolutions -> ticks (float)
        dticks_L = dsL * self.rev_per_m * self.PPR
        dticks_R = dsR * self.rev_per_m * self.PPR

        if self.noise_std > 0.0:
            import random
            dticks_L += random.gauss(0.0, self.noise_std)
            dticks_R += random.gauss(0.0, self.noise_std)

        # accumulate to ints (ซ้ายใช้ร่วม FL/RL, ขวา FR/RR)
        self.accL += dticks_L
        self.accR += dticks_R
        incL = int(self.accL)
        self.accL -= incL
        incR = int(self.accR)
        self.accR -= incR

        self.t[0] += incL  # FL
        self.t[2] += incR  # FR
        self.t[1] += incL  # RL
        self.t[3] += incR  # RR

        msg = Int32MultiArray()
        msg.data = self.t
        self.pub.publish(msg)


def main():
    rclpy.init()
    n = WheelTicksSim()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
