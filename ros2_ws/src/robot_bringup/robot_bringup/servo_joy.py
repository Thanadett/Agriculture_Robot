#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int32

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def map_raw_to_percent(raw: float, released_raw: float, pressed_raw: float) -> int:
    # แปลงค่าดิบของแกนให้เป็น 0..100 โดย 0=ปล่อย, 100=กดสุด
    # handle กรณี released_raw == pressed_raw
    if abs(pressed_raw - released_raw) < 1e-6:
        return 0
    t = (raw - released_raw) / (pressed_raw - released_raw)
    pct = int(round(clamp(t * 100.0, 0.0, 100.0)))
    return pct

class JoystickButtons(Node):
    def __init__(self):
        super().__init__('joystick_buttons')

        # ---- Parameters ----
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('servo_cmd_topic', '/servo_cmd')
        self.declare_parameter('btn_a', 0)  # Xbox: A=0
        self.declare_parameter('btn_b', 1)  # Xbox: B=1
        # self.declare_parameter('btn_x', 2)  # Xbox: X=2
        self.declare_parameter('btn_y', 3)  # Xbox: Y=3

        self.declare_parameter('axis_lt', 2)   # LT
        self.declare_parameter('axis_rt', 5)   # RT
        # ออปชัน ส่งค่า numeric เพิ่มเติม
        self.declare_parameter('publish_numeric', True)
        self.declare_parameter('change_threshold_pct', 2)  # emit when changed >= 2%

          # Raw range mapping (ค่าแกนตอน "ปล่อย" และ "กดสุด")
        # Xbox บน Linux: ปล่อย ≈ +1.0, กดสุด ≈ -1.0
        self.declare_parameter('lt_released_raw', 1.0)
        self.declare_parameter('lt_pressed_raw', -1.0)
        self.declare_parameter('rt_released_raw', 1.0)
        self.declare_parameter('rt_pressed_raw', -1.0)
        self.declare_parameter('ema_alpha', 0.2)   # EMA smoothing factor for triggers
        
        self.declare_parameter('axis_index', 6)     # Xbox D-pad horizontal = 6
        self.declare_parameter('threshold', 0.5)    # deadzone threshold
        self.declare_parameter('debounce_ms', 20) # small debounce


        p = lambda k: self.get_parameter(k).get_parameter_value()
        joy_topic = p('joy_topic').string_value
        servo_topic = p('servo_cmd_topic').string_value
        self.idx_a = int(p('btn_a').integer_value)
        self.idx_b = int(p('btn_b').integer_value)
        # self.idx_x = int(p('btn_x').integer_value)
        self.idx_y = int(p('btn_y').integer_value)

        self.idx_lt = int(p('axis_lt').integer_value)
        self.idx_rt = int(p('axis_rt').integer_value)

        self.lt_rel = float(p('lt_released_raw').double_value)
        self.lt_prs = float(p('lt_pressed_raw').double_value)
        self.rt_rel = float(p('rt_released_raw').double_value)
        self.rt_prs = float(p('rt_pressed_raw').double_value)

        self.axis_index  = int(p('axis_index').integer_value)
        self.threshold   = float(p('threshold').double_value)
        self.debounce_ms = int(p('debounce_ms').integer_value)
        self.ema_alpha   = float(p('ema_alpha').double_value)
        self.change_threshold_pct = int(p('change_threshold_pct').integer_value)
        self.publish_numeric = p('publish_numeric').bool_value

        # ---- Pub/Sub ----
        qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, 
        depth=10
        )
        self.pub_servo = self.create_publisher(String, servo_topic, qos_profile=qos)
        self.sub_joy = self.create_subscription(Joy, joy_topic, self.cb_joy,
                                                qos_profile=qos_profile_sensor_data)

        if self.publish_numeric:
            self.pub_lt_pct = self.create_publisher(Int32, '/lt_percent', qos_profile=qos)
            self.pub_rt_pct = self.create_publisher(Int32, '/rt_percent', qos_profile=qos)
        else:
            self.pub_lt_pct = None
            self.pub_rt_pct = None

    
        # ---- State ----
        self.prev_a = 0
        self.prev_b = 0
        # self.prev_x = 0
        self.prev_y = 0

        self.last_sent_lt = 0
        self.last_sent_rt = 0

        #D - pad
        self.prev_left = 0
        self.prev_right = 0
        self.last_change_ms = 0.0

        self.get_logger().info(f'Buttons node started: joy={joy_topic} -> servo_cmd={servo_topic}')

    @staticmethod
    def _btn(msg, idx):
        return 1 if (0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1) else 0

    def _emit(self, text: str):
        self.pub_servo.publish(String(data=text))
        # self.get_logger().info(f'Emit: {text}')

    def _emit_numeric(self, lt_pct: int, rt_pct: int):
        if self.pub_lt_pct:
            self.pub_lt_pct.publish(Int32(data=int(lt_pct)))
        if self.pub_rt_pct:
            self.pub_rt_pct.publish(Int32(data=int(rt_pct)))

    def _ema(self, prev_val: float, new_val: float) -> float:
        a = clamp(self.ema_alpha, 0.0, 1.0)
        if a <= 0.0:
            return new_val
        return (1.0 - a) * prev_val + a * new_val

    def cb_joy(self, msg: Joy):
        now_ms = time.monotonic() * 1000.0
        if now_ms - self.last_change_ms < self.debounce_ms:
            return

        a = self._btn(msg, self.idx_a)
        b = self._btn(msg, self.idx_b)
        # x = self._btn(msg, self.idx_x)
        y = self._btn(msg, self.idx_y)

        if a != self.prev_a:
            self._emit(f'BTN A={"DOWN" if a else "UP"}')
            self.prev_a = a
            self.last_change_ms = now_ms

        if b != self.prev_b:
            self._emit(f'BTN B={"DOWN" if b else "UP"}')
            self.prev_b = b
            self.last_change_ms = now_ms

        # if x != self.prev_x:
        #     self._emit(f'BTN X={"DOWN" if x else "UP"}')
        #     self.prev_x = x
        #     self.last_change_ms = now_ms

        if y != self.prev_y:
            self._emit(f'BTN Y={"DOWN" if y else "UP"}')
            self.prev_y = y
            self.last_change_ms = now_ms

        # D-pad left/right
        val = 0.0
        if 0 <= self.axis_index < len(msg.axes):
            val = float(msg.axes[self.axis_index])

        # กด UP ถ้าแกนเป็น 1 (ดันขึ้น) / กด DOWN ถ้า -1 (ดันลง)
        left = 1 if val <= -self.threshold else 0
        right = 1 if val >= self.threshold else 0

        if left != self.prev_left:
            self._emit(f'BTN L={"DOWN" if left else "UP"}')
            self.prev_left = left
            self.last_change_ms = now_ms

        if right != self.prev_right:
            self._emit(f'BTN R={"DOWN" if right else "UP"}')
            self.prev_right = right
            self.last_change_ms = now_ms


        # --- LT/RT analog → percent 0..100 ---
        lt_raw = msg.axes[self.idx_lt] if 0 <= self.idx_lt < len(msg.axes) else self.lt_rel
        rt_raw = msg.axes[self.idx_rt] if 0 <= self.idx_rt < len(msg.axes) else self.rt_rel

        lt_pct_new = map_raw_to_percent(lt_raw, self.lt_rel, self.lt_prs)
        rt_pct_new = map_raw_to_percent(rt_raw, self.rt_rel, self.rt_prs)

        # EMA กันกระพือ
        lt_pct_f = int(round(self._ema(self.lt_pct, lt_pct_new)))
        rt_pct_f = int(round(self._ema(self.rt_pct, rt_pct_new)))

        self.lt_pct = lt_pct_f
        self.rt_pct = rt_pct_f
        # ส่งออกเมื่อเปลี่ยนเกิน threshold
        changed = False
        if abs(self.lt_pct - self.last_sent_lt) >= self.change_threshold_pct:
            self._emit(f'BTN LT={self.lt_pct}')
            self.last_sent_lt = self.lt_pct
            changed = True

        if abs(self.rt_pct - self.last_sent_rt) >= self.change_threshold_pct:
            self._emit(f'BTN RT={self.rt_pct}')
            self.last_sent_rt = self.rt_pct
            changed = True

        # ส่งแบบตัวเลข (ถ้าเปิดใช้งาน)
        if changed and self.publish_numeric:
            self._emit_numeric(self.lt_pct, self.rt_pct)

        if changed:
            self.last_change_ms = now_ms




def main():
    rclpy.init()
    node = JoystickButtons()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
