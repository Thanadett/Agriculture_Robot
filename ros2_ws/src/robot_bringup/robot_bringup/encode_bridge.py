#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class EncSubscriber(Node):
    def __init__(self):
        super().__init__('enc_subscriber')

        # ----------------------------------
        self.declare_parameter('print_hz', 2.0)          # พิมพ์กี่ครั้งต่อวินาที (เช่น 2 Hz)
        self.declare_parameter('decimals', 4)            # ทศนิยมสำหรับแสดงผล
        self.declare_parameter('deadband', 1e-4)         # ค่าต่ำกว่า deadband จะปัดเป็น 0
        self.declare_parameter('wheel_radius', 0.0635)   # m (ถ้าต้องการคำนวณ m/s จาก rad/s)
        self.declare_parameter('units_pos', 'rad')       # 'rad' หรือ 'deg'
        self.declare_parameter('units_dist', 'm')        # 'm' หรือ 'mm'
        self.declare_parameter('enable_log', True)       # << สวิตช์ เปิด/ปิด log
        # -----------------------------------

        self.print_hz   = float(self.get_parameter('print_hz').value)
        self.decimals   = int(self.get_parameter('decimals').value)
        self.deadband   = float(self.get_parameter('deadband').value)
        self.radius     = float(self.get_parameter('wheel_radius').value)
        self.units_pos  = str(self.get_parameter('units_pos').value)   # rad/deg
        self.units_dist = str(self.get_parameter('units_dist').value)  # m/mm
        self.enable_log = bool(self.get_parameter('enable_log').value) # << ใช้งานสวิตช์

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.VOLATILE
        qos.history     = HistoryPolicy.KEEP_LAST

        # เก็บ “ค่าล่าสุด” จากทั้งสอง topic
        self._last_js = None            # JointState
        self._last_total = None         # Float32MultiArray
        self._last_js_time = 0.0
        self._last_total_time = 0.0

        self.create_subscription(JointState, '/enc/joint_states', self._on_js, qos)
        self.create_subscription(Float32MultiArray, '/enc/total', self._on_total, qos)

        # ตั้ง timer ให้ “พิมพ์สรุป” ตามจังหวะที่ต้องการ
        period = 1.0 / max(self.print_hz, 0.1)
        self.create_timer(period, self._print_once, callback_group=ReentrantCallbackGroup())

        # ทำให้สามารถเปลี่ยน parameter ระหว่างรันได้ (เช่น ros2 param set)
        self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info(
            f'enc_subscriber started (RELIABLE, QoS-depth=10, print_hz={self.print_hz}, enable_log={self.enable_log})'
        )

    # ------------------- Param dynamic update -------------------
    def _on_param_update(self, params):
        for p in params:
            if p.name == 'enable_log' and p.type_ in (Parameter.Type.BOOL, Parameter.Type.INTEGER):
                self.enable_log = bool(p.value)
            elif p.name == 'print_hz' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                #ปลี่ยน print_hz ระหว่างรันจะไม่มีผลกับ period ของ timer เดิม
                pass
            elif p.name == 'decimals' and p.type_ in (Parameter.Type.INTEGER,):
                self.decimals = int(p.value)
            elif p.name == 'deadband' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                self.deadband = float(p.value)
            elif p.name == 'wheel_radius' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                self.radius = float(p.value)
            elif p.name == 'units_pos' and p.type_ == Parameter.Type.STRING:
                self.units_pos = str(p.value)
            elif p.name == 'units_dist' and p.type_ == Parameter.Type.STRING:
                self.units_dist = str(p.value)
        return rclpy.parameter.SetParametersResult(successful=True)

    # ------------------- Callbacks -------------------
    def _on_js(self, msg: JointState):
        self._last_js = msg
        self._last_js_time = time.time()

    def _on_total(self, msg: Float32MultiArray):
        self._last_total = msg
        self._last_total_time = time.time()

    # ------------------- Timer print -------------------
    def _print_once(self):
        # ถ้ายังไม่เคยรับ ไม่ต้องพิมพ์
        if self._last_js is None and self._last_total is None:
            return
        # ถ้าปิดสวิตช์ ก็ไม่พิมพ์
        if not self.enable_log:
            return

        # เตรียมฟอร์แมตและหน่วย
        pos_scale = 180.0/3.141592653589793 if self.units_pos.lower() == 'deg' else 1.0
        pos_unit  = 'deg' if self.units_pos.lower() == 'deg' else 'rad'

        dist_scale = 1000.0 if self.units_dist.lower() == 'mm' else 1.0
        dist_unit  = 'mm' if self.units_dist.lower() == 'mm' else 'm'

        lines = []

        # รวมข้อมูล JointState (pos=rad, vel=rad/s)
        if self._last_js is not None:
            pos = list(self._last_js.position) if self._last_js.position else []
            vel = list(self._last_js.velocity) if self._last_js.velocity else []

            pos_fmt = self._fmt_list(pos, scale=pos_scale, unit=pos_unit)
            vel_fmt = self._fmt_list(vel, scale=1.0, unit='rad/s')

            # ความเร็วเชิงเส้นของล้อ (m/s):
            vel_mps = [v * self.radius for v in vel] if vel else []
            vel_mps_fmt = self._fmt_list(vel_mps, scale=1.0, unit='m/s')

            lines.append(f"JS pos[{pos_unit}]={pos_fmt}    |\n vel={vel_fmt} \n vel_lin={vel_mps_fmt}")

        # รวมข้อมูล TOTAL (ระยะทางแนวราบ)
        if self._last_total is not None:
            data = list(self._last_total.data) if self._last_total.data else []
            data_fmt = self._fmt_list(data, scale=dist_scale, unit=dist_unit)

            # layout
            dims = []
            for d in self._last_total.layout.dim:
                dims.append(f"{d.label}:size={d.size},stride={d.stride}")
            dims_txt = "[" + ", ".join(dims) + "]" if dims else "[]"

            lines.append(f"\nTOTAL dist[{dist_unit}]={data_fmt} | layout={dims_txt} | offset={self._last_total.layout.data_offset}")

        # พิมพ์ 1 รอบต่อ timer
        if lines:
            self.get_logger().info(" | ".join(lines))

    # ตัดเลขกวนตาด้วย deadband + ฟอร์แมตทศนิยม
    def _fmt_list(self, xs, scale=1.0, unit=""):
        outs = []
        for x in xs:
            y = x * scale
            if abs(y) < self.deadband:
                y = 0.0
            outs.append(f"{y:.{self.decimals}f}")
        suffix = f" {unit}" if unit else ""
        return "[" + ", ".join(outs) + "]" + suffix


def main():
    rclpy.init()
    node = EncSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
