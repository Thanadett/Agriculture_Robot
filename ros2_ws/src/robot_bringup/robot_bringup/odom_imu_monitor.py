#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time

class OdomImuMonitor(Node):
    def __init__(self):
        super().__init__('odom_imu_monitor')
        self.odom_topic = self.declare_parameter('odom_topic', '/wheel/odom').get_parameter_value().string_value
        self.imu_topic  = self.declare_parameter('imu_topic',  '/imu/data').get_parameter_value().string_value
        self.warn_rate_odom = self.declare_parameter('warn_rate_odom', 30.0).get_parameter_value().double_value
        self.warn_rate_imu  = self.declare_parameter('warn_rate_imu',  80.0).get_parameter_value().double_value

        self.t_last_odom = None
        self.t_last_imu  = None
        self.odom_count = 0
        self.imu_count  = 0

        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 10)
        self.sub_imu  = self.create_subscription(Imu, self.imu_topic, self.cb_imu, 10)

        self.timer = self.create_timer(2.0, self.tick)
        self.get_logger().info(f"Monitor subscribing: odom={self.odom_topic}, imu={self.imu_topic}")

    def cb_odom(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t_last_odom is not None:
            dt = now - self.t_last_odom
            if dt > 0:
                rate = 1.0 / dt
                if rate < self.warn_rate_odom:
                    self.get_logger().warn(f"ODOM rate low: {rate:.1f} Hz (<{self.warn_rate_odom})")
        self.t_last_odom = now
        self.odom_count += 1

    def cb_imu(self, msg: Imu):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t_last_imu is not None:
            dt = now - self.t_last_imu
            if dt > 0:
                rate = 1.0 / dt
                if rate < self.warn_rate_imu:
                    self.get_logger().warn(f"IMU rate low: {rate:.1f} Hz (<{self.warn_rate_imu})")
        self.t_last_imu = now
        self.imu_count += 1

    def tick(self):
        # รายงานสั้นๆ ทุก 2 วินาที
        od = 'n/a' if self.t_last_odom is None else f"{self.odom_count} msgs"
        im = 'n/a' if self.t_last_imu  is None else f"{self.imu_count} msgs"
        self.get_logger().info(f"[2s] odom: {od} | imu: {im}")
        self.odom_count = 0
        self.imu_count  = 0

def main():
    rclpy.init()
    node = OdomImuMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
