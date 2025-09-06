#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_linear', 255.0)
        self.declare_parameter('max_angular', 255.0)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
            self.get_logger().info(f'Opened serial: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Cannot open serial {port}: {e}')
            raise

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)

        self.stop_evt = threading.Event()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def cb_cmd(self, msg: Twist):
        vmax = max(0.0, float(self.get_parameter('max_linear').value))
        wmax = max(0.0, float(self.get_parameter('max_angular').value))

        v = max(min(msg.linear.x, vmax), -vmax)
        w = max(min(msg.angular.z, wmax), -wmax)

        line = f'VW V={v:.3f} W={w:.3f}\n'
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def _read_loop(self):
        buf = b''
        while not self.stop_evt.is_set():
            try:
                data = self.ser.read(128)
                if data:
                    buf += data
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        s = line.decode('utf-8', errors='ignore').strip()
                        if s:
                            self.get_logger().info(f'ESP32: {s}')
            except Exception:
                pass

    def destroy_node(self):
        self.stop_evt.set()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
