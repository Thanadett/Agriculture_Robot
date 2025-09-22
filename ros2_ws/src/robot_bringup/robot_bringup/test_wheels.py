#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

TICKS_PER_REV = 11880
WHEEL_R = 0.0635  # m
RATE_HZ = 50.0

v_mps = 0.10
rev_per_s = v_mps / (2.0 * math.pi * WHEEL_R)
ticks_per_s = rev_per_s * TICKS_PER_REV            # ≈ 2970 ticks/s
ticks_per_step = int(round(ticks_per_s / RATE_HZ)) # ≈ 59 ticks/step

class TestWheels(Node):
    def __init__(self):
        super().__init__('test_wheels')
        self.pub = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)
        self.t = [0, 0, 0, 0]  # FL, RL, FR, RR
        self.timer = self.create_timer(1.0 / RATE_HZ, self.step)
        self.get_logger().info(f'Publishing wheel ticks @ {RATE_HZ} Hz, +{ticks_per_step}/step (straight)')

    def step(self):
        # วิ่งตรง: ซ้ายและขวาเพิ่มเท่ากัน
        self.t[0] += ticks_per_step  # FL
        self.t[1] += ticks_per_step  # RL
        self.t[2] += ticks_per_step  # FR
        self.t[3] += ticks_per_step  # RR
        msg = Int32MultiArray()
        msg.data = self.t
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TestWheels()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
