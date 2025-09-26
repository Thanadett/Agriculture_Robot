#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time

try:
    from tf2_ros import TransformBroadcaster
    HAS_TF = True
except Exception:
    HAS_TF = False

class BaseNode(Node):
    def __init__(self):
        super().__init__('base_controller')

        # ---------------- Parameters ----------------
        self.declare_parameter('ticks_topic', '/wheel_ticks')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_radius_m', 0.0635) # รัศมีล้อ (เมตร)
        self.declare_parameter('track_width_m', 0.365)    # ระยะล้อซ้าย-ขวา (เมตร)
        self.declare_parameter('ppr_out', 5940.0)          # พัลส์/รอบล้อ (ต้องตรงกับ ESP32)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.ticks_topic = self.get_parameter('ticks_topic').get_parameter_value().string_value
        self.odom_topic  = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.R  = float(self.get_parameter('wheel_radius_m').value)
        self.W  = float(self.get_parameter('track_width_m').value)
        self.PPR = float(self.get_parameter('ppr_out').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.publish_tf = bool(self.get_parameter('publish_tf').value) and HAS_TF

        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---------------- I/O ----------------
        self.sub_ticks = self.create_subscription(
            Int32MultiArray, self.ticks_topic, self.cb_ticks, qos
        )
        self.pub_odom = self.create_publisher(Odometry, self.odom_topic, qos)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None

        # ---------------- State ----------------
        self.last_ticks = None     # [FL, FR, RL, RR]
        self.last_time = None      # rclpy Time
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Precompute circumference factor
        self.circ = 2.0 * math.pi * self.R
        self.rev_per_count = 1.0 / max(self.PPR, 1.0)  # กันหารด้วยศูนย์

        self.get_logger().info(
            f'base_controller up: sub={self.ticks_topic}, pub={self.odom_topic}, '
            f'R={self.R:.3f} m, W={self.W:.3f} m, PPR={self.PPR:.1f}, TF={bool(self.tf)}'
        )

    def cb_ticks(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) != 4:
            self.get_logger().warn(f'/wheel_ticks length={len(data)} (expected 4); ignored')
            return

        now = self.get_clock().now()

        # First message: just latch
        if self.last_ticks is None or self.last_time is None:
            self.last_ticks = data
            self.last_time = now
            return

        # dt (seconds)
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-6

        # delta counts per wheel
        dFL = data[0] - self.last_ticks[0]
        dFR = data[1] - self.last_ticks[1]
        dRL = data[2] - self.last_ticks[2]
        dRR = data[3] - self.last_ticks[3]

        # convert counts -> distance per wheel (meters)
        # distance = delta_rev * circumference = (delta_count / PPR) * 2πR
        sFL = dFL * self.rev_per_count * self.circ
        sFR = dFR * self.rev_per_count * self.circ
        sRL = dRL * self.rev_per_count * self.circ
        sRR = dRR * self.rev_per_count * self.circ

        # left/right average (diff-drive from 4 wheels)
        sL = 0.5 * (sFL + sRL)
        sR = 0.5 * (sFR + sRR)

        # body-frame integration
        ds = 0.5 * (sR + sL)
        dth = (sR - sL) / max(self.W, 1e-6)

        # integrate pose (runge-kutta 1/2 step)
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th = self.normalize_angle(self.th + dth)

        # velocities
        vx = ds / dt
        vth = dth / dt

        # publish odometry
        odom = Odometry()
        odom.header.stamp = self.to_time_msg(now)
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # pose
        qz, qw = math.sin(self.th * 0.5), math.cos(self.th * 0.5)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # (optional) simple covariances
        odom.pose.covariance = [
            1e-3, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e6, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e6, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 1e-2
        ]

        # twist
        odom.twist.twist.linear.x  = float(vx)
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(vth)
        odom.twist.covariance = [
            1e-2, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  1e-2, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e6, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e6, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 5e-2
        ]

        self.pub_odom.publish(odom)

        # optional TF
        if self.tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf.sendTransform(t)

        # latch
        self.last_ticks = data
        self.last_time = now

    # -------- helpers --------
    @staticmethod
    def normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def to_time_msg(t) -> Time:
        # rclpy Time -> builtin_interfaces/Time
        return Time(sec=int(t.nanoseconds * 1e-9), nanosec=int(t.nanoseconds % 1e9))


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()