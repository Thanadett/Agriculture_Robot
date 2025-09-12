#!/usr/bin/env python3
import rclpy, serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class ServoBridge(Node):
    def __init__(self):
        super().__init__('servo_bridge')

        # parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', '/servo_cmd')

        p = lambda k: self.get_parameter(k).get_parameter_value()
        port = p('port').string_value
        baud = int(p('baud').integer_value) if p('baud').type == 2 else int(p('baud').double_value)
        topic = p('topic').string_value

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
        self.get_logger().info(f'Opened serial: {port} @ {baud}')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.sub = self.create_subscription(String, topic, self.cb, qos)

    def cb(self, msg: String):
        line = msg.data.strip()
        if not line:
            return
        # บังคับรูปแบบที่ ESP32 ต้องการมี \n
        if not line.endswith('\n'):
            line += '\n'
        try:
            self.ser.write(line.encode('ascii'))
            # self.get_logger().info(f'Sent: {line.strip()}')
        except Exception as e:
            self.get_logger().error(f'Write failed: {e}')

def main():
    rclpy.init()
    node = ServoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
