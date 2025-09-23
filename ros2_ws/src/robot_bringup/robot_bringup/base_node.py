#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class BaseNode(Node):
    def __init__(self):
        super().__init__('base_controller')

        # params
        self.wheel_radius = self.declare_parameter('wheel_radius_m', 0.0635).value
        self.track_width  = self.declare_parameter('track_width_m', 0.30).value
        self.ticks_per_rev= self.declare_parameter('ticks_per_rev', 11880).value
        self.frame_odom   = self.declare_parameter('frame_odom', 'odom').value
        self.frame_base   = self.declare_parameter('frame_base', 'base_link').value

        # state
        self.prev_ticks = None
        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_time = self.get_clock().now()
        self.yaw_rad = None

        # subs
        self.create_subscription(Int32MultiArray, '/wheel_ticks', self.on_ticks, 10)
        self.create_subscription(Float32, '/yaw_deg', self.on_yaw, 10)

        # pubs
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tfbr = TransformBroadcaster(self)

    def on_yaw(self, msg: Float32):
        self.yaw_rad = math.radians(msg.data)

    def on_ticks(self, msg: Int32MultiArray):
        if len(msg.data) < 4:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0: return

        if self.prev_ticks is None:
            self.prev_ticks = msg.data
            return

        d = [msg.data[i] - self.prev_ticks[i] for i in range(4)]
        self.prev_ticks = msg.data

        dL = 0.5 * (d[0] + d[1])
        dR = 0.5 * (d[2] + d[3])

        def ticks_to_m(ticks):
            rev = ticks / self.ticks_per_rev
            return rev * 2.0 * math.pi * self.wheel_radius

        sL = ticks_to_m(dL)
        sR = ticks_to_m(dR)

        s = 0.5 * (sL + sR)
        dth = (sR - sL) / self.track_width

        th = self.yaw_rad if self.yaw_rad is not None else (self.th + dth)
        self.th = th
        self.x += s * math.cos(th)
        self.y += s * math.sin(th)

        vx = s / dt
        wz = dth / dt

        # publish odom
        od = Odometry()
        od.header.stamp = now.to_msg()
        od.header.frame_id = self.frame_odom
        od.child_frame_id  = self.frame_base
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y

        qz = math.sin(th/2.0)
        qw = math.cos(th/2.0)
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        od.twist.twist.linear.x = vx
        od.twist.twist.angular.z = wz
        self.odom_pub.publish(od)

        # publish TF
        t = TransformStamped()
        t.header = od.header
        t.child_frame_id = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tfbr.sendTransform(t)

def main():
    rclpy.init()
    node = BaseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
