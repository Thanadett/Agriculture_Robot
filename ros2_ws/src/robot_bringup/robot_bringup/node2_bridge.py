#!/usr/bin/env python3
import time
import rclpy, serial, threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class Node2Bridge(Node):
    def __init__(self):
        super().__init__('node2_bridge')

        # ---- parameters (same style) ----
        self.declare_parameter('port', '/dev/esp32_node2')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('servo_topic', '/servo_cmd')
        self.declare_parameter('step_topic',  '/step_cmd')

        self.declare_parameter('open_retry_sec', 1.0)  # retry เปิดพอร์ตทุก ๆ 1s
        self.declare_parameter('verbose', False)       # log ตอนส่ง

        p = lambda k: self.get_parameter(k).value
        self.port    = str(p('port'))
        self.baud    = int(p('baud'))
        self.retry   = float(p('open_retry_sec'))
        self.verbose = bool(p('verbose'))
        servo_topic  = str(p('servo_topic'))
        step_topic   = str(p('step_topic'))

        self.ser = None
        self._lock = threading.Lock()
        self._open_serial_with_retry()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # ใช้ handler เดียวกันทั้งสองท็อปปิก
        self.sub_servo = self.create_subscription(String, servo_topic, self.cb, qos)
        self.sub_step  = self.create_subscription(String, step_topic,  self.cb, qos)

        # ตั้ง timer คอยเช็ค/เปิดใหม่ ถ้าหลุดระหว่างทาง
        self.create_timer(1.0, self._tick_check_serial)

        self.get_logger().info(
            f'Node2Bridge ready: port={self.port} baud={self.baud} '
            f'servo_topic={servo_topic} step_topic={step_topic}'
        )

    # ---------- serial helpers ----------
    def _open_serial_with_retry(self):
        while rclpy.ok():
            try:
                self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.02)
                self.get_logger().info(f'Opened serial: {self.port} @ {self.baud}')
                return
            except Exception as e:
                self.get_logger().warning(f'Cannot open {self.port}: {e}. Retry in {self.retry:.1f}s')
                time.sleep(self.retry)

    def _tick_check_serial(self):
        if self.ser is None or not self.ser.is_open:
            try:
                self.ser = serial.Serial(port=self.port, baudrate=self.baud, timeout=0.02)
                self.get_logger().info(f'Re-opened serial: {self.port}')
            except Exception:
                # เงียบไว้ ให้ลองรอบต่อไป
                pass

    # ---------- unified callback for both topics ----------
    def cb(self, msg: String):
        line = (msg.data or '').strip()
        if not line:
            return
        if not line.endswith('\n'):
            line += '\n'

        with self._lock:
            if self.ser is None or not self.ser.is_open:
                self.get_logger().warning(f'Serial not open. Drop: {line.strip()}')
                return
            try:
                self.ser.write(line.encode('ascii'))
                if self.verbose:
                    self.get_logger().info(f'Sent: {line.strip()}')
            except Exception as e:
                self.get_logger().error(f'Write failed: {e}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None  # ให้ timer ไปเปิดใหม่

def main():
    rclpy.init()
    node = Node2Bridge()
    try:
        rclpy.spin(node)
    finally:
        if node.ser is not None:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
