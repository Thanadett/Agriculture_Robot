import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import serial
import threading

class EncodeBridge(Node):
    def __init__(self):
        super().__init__('encode_bridge')
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)


        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02)
            self.get_logger().info(f'Opened serial: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Cannot open serial {port}: {e}')
            raise
        
        self.sub = self.create_subscription(JointState, '/enc/joint_states', self.cb_cmd, 10)
        self.sub = self.create_subscription(Float32MultiArray, '/enc/total', self.cb_cmd, 10)


        self.stop_evt = threading.Event()
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()


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
    node = EncodeBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
