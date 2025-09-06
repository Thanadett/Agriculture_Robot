#!/usr/bin/env python3
# camera_hud.py - Subscribe image, overlay HUD (FPS/Latency, center marker,
# virtual joystick, speed indicator) with runtime parameters (ROS2 Jazzy)

import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node, SetParametersResult
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge


def clamp(v, a, b): return a if v < a else b if v > b else v


class CameraHUD(Node):
    def __init__(self):
        super().__init__('camera_hud')

        # ---------------- Parameters (dynamic) ----------------
        # Topics
        # ใช้ compressed เพื่อลดแบนด์วิธ
        self.declare_parameter('image_topic', '/image_raw/compressed')
        self.declare_parameter('joy_topic',   '/joy')
        self.declare_parameter('cmd_topic',   '/cmd_vel')

        # Overlays on/off
        self.declare_parameter('show_fps',        True)
        self.declare_parameter('show_latency',    True)
        self.declare_parameter('show_center',     True)
        self.declare_parameter('show_joy',        True)
        self.declare_parameter('show_speed',      True)

        # Overlay styling
        self.declare_parameter('font_scale',      0.6)
        self.declare_parameter('thickness',       2)
        # background box alpha
        self.declare_parameter('alpha',           0.25)
        self.declare_parameter('hud_color_bgr',   [0, 255, 255])  # yellow
        self.declare_parameter('center_color_bgr', [0, 255, 0])    # green
        self.declare_parameter('joy_color_bgr',   [255, 0, 0])    # blue
        self.declare_parameter('speed_color_bgr', [0, 0, 255])    # red

        # HUD config
        self.declare_parameter('center_size_px',  25)
        # 0..1 of min(width,height)/2
        self.declare_parameter('joy_scale',       0.9)
        self.declare_parameter('speed_bar_len',   150)
        # publish/log every N sec
        self.declare_parameter('metrics_period_s', 1.0)
        self.declare_parameter('latency_cap_ms',  500)     # clamp display

        # Runtime set
        def p(k): return self.get_parameter(k).value
        self.top_image = p('image_topic')
        self.top_joy = p('joy_topic')
        self.top_cmd = p('cmd_topic')

        # ---------------- Subscriptions ----------------
        # image: ใช้ QoS sensor_data (best effort) ลดหน่วง/ไม่ค้างคิว
        self.bridge = CvBridge()
        img_qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)

        # รองรับทั้ง raw และ compressed (cv_bridge จัดการให้โดยใช้ toCvCopy/np.frombuffer ถ้าจำเป็น)
        self.sub_img = self.create_subscription(
            Image, self.top_image, self.cb_image, img_qos)

        # joy & cmd_vel เพื่อ overlay
        self.last_joy = None
        self.sub_joy = self.create_subscription(
            Joy, self.top_joy, self.cb_joy, 10)

        self.last_cmd = Twist()
        self.sub_cmd = self.create_subscription(
            Twist, self.top_cmd, self.cb_cmd, 10)

        # metrics publisher (ข้อความอ่านง่าย)
        self.pub_metrics = self.create_publisher(
            String, '/camera_hud/metrics', 10)

        # ---------------- FPS & Latency ----------------
        self.last_stamp_ns = None
        self.frame_count = 0
        self.fps = 0.0
        self.latency_ms = 0.0
        self.t_last = time.monotonic()
        self.t_last_metrics = time.monotonic()

        # window (ถ้ารันบน Pi แล้วดูผ่าน Foxglove ก็ไม่ต้องแสดงหน้าต่างนี้)
        self.window_name = 'Camera HUD (local preview)'
        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)  # เปิดถ้าต้องการพรีวิวบนเครื่องเดียวกัน

        # dynamic param callback
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f'CameraHUD started. image={self.top_image} joy={self.top_joy} cmd={self.top_cmd}')

    # -------- parameter change --------
    def _on_param_change(self, params):
        for p in params:
            if p.name == 'image_topic' and p.value != self.top_image:
                self.get_logger().info(f'switch image topic -> {p.value}')
                # NOTE: rclpy ยังไม่สะดวกต่อการ re-subscribe runtime ใน 1 node;
                # ใช้ค่าใหม่ครั้งหน้าตอนรีสตาร์ทจะเสถียรกว่า
        return SetParametersResult(successful=True)

    # -------- callbacks --------
    def cb_joy(self, msg: Joy):
        self.last_joy = msg

    def cb_cmd(self, msg: Twist):
        self.last_cmd = msg

    def cb_image(self, msg: Image):
        now = time.monotonic()

        # FPS (measured on receive)
        self.frame_count += 1
        dt_report = now - self.t_last
        if dt_report >= 1.0:
            self.fps = self.frame_count / dt_report
            self.frame_count = 0
            self.t_last = now

        # Latency (header stamp vs now) — อาจเป็น 0 ถ้าต้นทางไม่ตั้ง stamp
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.latency_ms = clamp((time.time() - t_img) * 1000.0,
                                    0.0, float(self.get_parameter('latency_cap_ms').value))
        else:
            self.latency_ms = 0.0

        # ------------- convert image -------------
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge convert failed: {e}')
            return

        h, w = frame.shape[:2]
        overlay = frame.copy()

        # ------------- overlays -------------
        font_scale = float(self.get_parameter('font_scale').value)
        thickness = int(self.get_parameter('thickness').value)
        alpha = float(self.get_parameter('alpha').value)

        color_hud = tuple(int(c)
                          for c in self.get_parameter('hud_color_bgr').value)
        color_ctr = tuple(int(c)
                          for c in self.get_parameter('center_color_bgr').value)
        color_joy = tuple(int(c)
                          for c in self.get_parameter('joy_color_bgr').value)
        color_spd = tuple(int(c)
                          for c in self.get_parameter('speed_color_bgr').value)

        if bool(self.get_parameter('show_center').value):
            s = int(self.get_parameter('center_size_px').value)
            cx, cy = w//2, h//2
            cv2.line(overlay, (cx-s, cy), (cx+s, cy), color_ctr, 2)
            cv2.line(overlay, (cx, cy-s), (cx, cy+s), color_ctr, 2)

        if bool(self.get_parameter('show_joy').value) and self.last_joy:
            # virtual joystick: วาด 2 วง (ซ้าย = axes[0,1], ขวา = axes[3,4] เป็นดีฟอลต์ตามจอย Xbox)
            rad = int(min(w, h) * 0.25 *
                      float(self.get_parameter('joy_scale').value))
            pad = int(rad * 0.2)
            # left stick center
            lc = (pad + rad, h - (pad + rad))
            rc = (w - (pad + rad), h - (pad + rad))
            cv2.circle(overlay, lc, rad, color_joy, 2)
            cv2.circle(overlay, rc, rad, color_joy, 2)

            def stick_xy(ax_x, ax_y):
                x = clamp(self._axis(self.last_joy, ax_x), -1.0, 1.0)
                y = -clamp(self._axis(self.last_joy, ax_y), -
                           1.0, 1.0)  # ยกแกน Y ขึ้นบน
                return (int(x*rad), int(y*rad))

            # left stick: 0=x,1=y ; right stick: 3=x,4=y (แก้ได้ภายหลังถ้าจอยต่าง)
            lx, ly = stick_xy(0, 1)
            rx, ry = stick_xy(3, 4)
            cv2.circle(overlay, (lc[0]+lx, lc[1]+ly), 6, color_joy, -1)
            cv2.circle(overlay, (rc[0]+rx, rc[1]+ry), 6, color_joy, -1)

        if bool(self.get_parameter('show_speed').value):
            v = float(self.last_cmd.linear.x)
            w_ang = float(self.last_cmd.angular.z)
            # bars
            bar_len = int(self.get_parameter('speed_bar_len').value)
            cx = 20
            basey = 30

            def draw_bar(y, val, vmax, label, col):
                val = clamp(val, -vmax, vmax)
                m = (val / vmax) if vmax > 1e-6 else 0.0
                x0, x1 = cx, cx + int(bar_len * abs(m))
                if m >= 0:
                    cv2.rectangle(overlay, (cx, y-8), (x1, y+8), col, -1)
                else:
                    cv2.rectangle(overlay, (cx+int(bar_len*m),
                                  y-8), (cx, y+8), col, -1)
                cv2.putText(overlay, f'{label}: {val:+.2f}', (cx+bar_len+10, y+5),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, col, thickness, cv2.LINE_AA)

            # สเกล bar 1.0 (ปรับรันไทม์ได้ถ้าต้องการ)
            draw_bar(basey, v,   1.0, 'v (m/s)', color_spd)
            draw_bar(basey+28, w_ang, 2.5, 'w (rad/s)', color_spd)

        if bool(self.get_parameter('show_fps').value) or bool(self.get_parameter('show_latency').value):
            lines = []
            if bool(self.get_parameter('show_fps').value):
                lines.append(f'FPS: {self.fps:.1f}')
            if bool(self.get_parameter('show_latency').value):
                lines.append(f'Latency: {self.latency_ms:.0f} ms')
            txt = ' | '.join(lines)
            (tw, th), _ = cv2.getTextSize(
                txt, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            box = np.zeros((th+12, tw+12, 3), dtype=np.uint8)
            box[:] = color_hud
            cv2.addWeighted(box, alpha, box, 0, 0, box)
            overlay[10:10+box.shape[0], 10:10+box.shape[1]] = cv2.addWeighted(
                overlay[10:10+box.shape[0], 10:10+box.shape[1]], 1.0, box, alpha, 0.0)
            cv2.putText(overlay, txt, (16, 16+th),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)

        # blend overlays
        cv2.addWeighted(overlay, 1.0, frame, 0.0, 0.0, frame)

        # publish metrics periodically
        if (now - self.t_last_metrics) >= float(self.get_parameter('metrics_period_s').value):
            self.t_last_metrics = now
            s = String()
            s.data = f'fps={self.fps:.1f}, latency_ms={self.latency_ms:.0f}'
            self.pub_metrics.publish(s)
            self.get_logger().info(s.data)

        # optional local preview:
        # cv2.imshow(self.window_name, frame); cv2.waitKey(1)

    @staticmethod
    def _axis(msg, idx):
        try:
            return float(msg.axes[idx])
        except Exception:
            return 0.0


def main():
    rclpy.init()
    node = CameraHUD()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
