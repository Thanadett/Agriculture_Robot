#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import threading
import time
import logging
import cv2
import numpy as np
from flask import Flask, Response, render_template, jsonify

# ---------------- Logging ----------------
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("camera-stream")

# ---------------- Flask ----------------
app = Flask(__name__)

# ---------------- Globals for Camera 1 ----------------
latest_jpeg_cam1 = None
jpeg_lock_cam1 = threading.Lock()
new_frame_event_cam1 = threading.Event()
latest_bgr_cam1 = None
bgr_lock_cam1 = threading.Lock()
fps_ema_cam1 = 0.0
last_tick_cam1 = time.monotonic()

# ---------------- Globals for Camera 2 ----------------
latest_jpeg_cam2 = None
jpeg_lock_cam2 = threading.Lock()
new_frame_event_cam2 = threading.Event()
latest_bgr_cam2 = None
bgr_lock_cam2 = threading.Lock()
fps_ema_cam2 = 0.0
last_tick_cam2 = time.monotonic()

# ---------------- Config ----------------
CONFIG = {
    'device1': 0,
    'device2': 1,
    'width1': 800,
    'height1': 600,
    'width2': 640,
    'height2': 480,
    'fps': 30,
    'port': 5000,
    'host': '0.0.0.0',
    'quality': 75,
    'flip': False,
    'rotate': 0,
    'show_fps': True
}

# ---------------- Guides ----------------
GUIDES = {
    'cam1': {
        'mode': 'percent',
        'left': 0.33, 'right': 0.67,
        'angle_deg': 0.0,
        'y_top': 0.0, 'y_bottom': 1.0,
        'color': (0, 255, 255),
        'thickness': 2, 'style': 'solid',
        'dash_len': 18, 'gap_len': 12,
        'alpha': 1.0,
        'top': None, 'bottom': None,
        'x_left': 0.0, 'x_right': 1.0,
        'h_angle_deg': 0.0,
    },
    'cam2': {
        'mode': 'percent',
        'left': 0.35, 'right': 0.70,
        'angle_deg': 0.0,
        'y_top': 0.0, 'y_bottom': 1.0,
        'color': (255, 255, 0),
        'thickness': 2, 'style': 'solid',
        'dash_len': 18, 'gap_len': 12,
        'alpha': 1.0,
        'top': None, 'bottom': None,
        'x_left': 0.0, 'x_right': 1.0,
        'h_angle_deg': 0.0,
    },
    'show_guides': True
}

# ---------------- ROS live data ----------------
ros_data = {
    'cmd_vel': {
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    },
    # wheel_ticks: 4 ล้อ + ค่าเฉลี่ย/รวมเพื่ออ้างอิง
    'wheel_ticks': {
        'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0,
        'left': 0.0, 'right': 0.0, 'total': 0.0
    },
    'yaw_rate': 0.0,              # rad/s (Float32)
    'robot_heartbeat': ""         # String (เช่น "OK" หรือ "1.0 Hz")
}
ros_lock = threading.Lock()

# ---------------- Try TurboJPEG ----------------
try:
    from turbojpeg import TurboJPEG, TJPF_BGR, TJSAMP_420
    _jpeg = TurboJPEG()

    def encode_jpeg(img_bgr, quality=75):
        return _jpeg.encode(img_bgr, quality=quality,
                            pixel_format=TJPF_BGR, jpeg_subsample=TJSAMP_420)
    log.info("TurboJPEG enabled")
except Exception:
    _jpeg = None

    def encode_jpeg(img_bgr, quality=75):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality),
                  int(cv2.IMWRITE_JPEG_OPTIMIZE), 1]
        ok, buf = cv2.imencode(".jpg", img_bgr, params)
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes()
    log.info("TurboJPEG not available, falling back to cv2.imencode")

# ---------------- Guide draw utils ----------------
def _alpha_blend_line(dst, p1, p2, color, thickness, alpha,
                      style='solid', dash_len=16, gap_len=12):
    if alpha <= 0:
        return
    overlay = dst.copy()
    if style == 'solid':
        cv2.line(overlay, p1, p2, color, thickness, cv2.LINE_AA)
    else:
        x1, y1 = p1; x2, y2 = p2
        dx, dy = x2 - x1, y2 - y1
        length = float(np.hypot(dx, dy))
        if length < 1e-6:
            return
        vx, vy = dx / length, dy / length
        if style == 'dotted':
            dash = max(1, int(thickness * 1.25))
            gap = max(2, int(thickness * 1.25 * 2))
            step = dash + gap
            t = 0.0
            while t <= length:
                cx = int(x1 + vx * t); cy = int(y1 + vy * t)
                cv2.circle(overlay, (cx, cy), max(1, thickness // 2),
                           color, -1, cv2.LINE_AA)
                t += step
        else:  # dashed
            dash = max(2, int(dash_len)); gap = max(2, int(gap_len))
            t = 0.0; on = True
            while t < length:
                seg = min(dash if on else gap, length - t)
                if on:
                    xA = int(x1 + vx * t); yA = int(y1 + vy * t)
                    xB = int(x1 + vx * (t + seg)); yB = int(y1 + vy * (t + seg))
                    cv2.line(overlay, (xA, yA), (xB, yB),
                             color, thickness, cv2.LINE_AA)
                t += seg; on = not on
    cv2.addWeighted(overlay, alpha, dst, 1.0 - alpha, 0, dst)

def _line_endpoints_at_x(x_pos, angle_deg, y_top, y_bottom, w, h):
    y0 = int(y_top * h); y1 = int(y_bottom * h)
    yc = h / 2.0; theta = np.deg2rad(angle_deg)
    if abs(theta) < 1e-6:
        return (int(x_pos), y0), (int(x_pos), y1)
    dy = (y1 - y0)
    vx = np.sin(theta) * dy; vy = np.cos(theta) * dy
    ym = (y0 + y1) / 2.0; p_mid = np.array([x_pos, yc + (ym - yc)])
    p1 = (int(p_mid[0] - vx / 2.0), int(p_mid[1] - vy / 2.0))
    p2 = (int(p_mid[0] + vx / 2.0), int(p_mid[1] + vy / 2.0))
    return p1, p2

def _line_endpoints_at_y(y_pos, angle_deg, x_left, x_right, w, h):
    x0 = int(max(0, min(w - 1, x_left)))
    x1 = int(max(0, min(w - 1, x_right)))
    xc = w / 2.0; theta = np.deg2rad(angle_deg)
    if abs(theta) < 1e-6:
        return (x0, int(y_pos)), (x1, int(y_pos))
    dx = (x1 - x0)
    vx = np.cos(theta) * dx; vy = np.sin(theta) * dx
    xm = (x0 + x1) / 2.0; p_mid = np.array([xc + (xm - xc), y_pos])
    p1 = (int(p_mid[0] - vx / 2.0), int(p_mid[1] - vy / 2.0))
    p2 = (int(p_mid[0] + vx / 2.0), int(p_mid[1] + vy / 2.0))
    return p1, p2

def _as_pos(val, size, mode):
    if val is None:
        return None
    return float(val) * size if mode == 'percent' else float(val)

def draw_pot_guides(frame_bgr, cam_key):
    if not GUIDES.get('show_guides', False):
        return
    g = GUIDES[cam_key]
    h, w = frame_bgr.shape[:2]
    mode = g.get('mode', 'percent')

    # vertical
    xL = _as_pos(g.get('left'),  w, mode) if g.get('left')  is not None else None
    xR = _as_pos(g.get('right'), w, mode) if g.get('right') is not None else None
    y_top = float(g.get('y_top', 0.0)); y_bottom = float(g.get('y_bottom', 1.0))
    angle_v = float(g.get('angle_deg', 0.0))
    color_v = tuple(int(c) for c in g.get('color', (0,255,255)))
    thick_v = int(g.get('thickness', 2)); alpha_v = float(g.get('alpha', 1.0))
    style_v = g.get('style', 'solid')
    dash_v = int(g.get('dash_len', 16)); gap_v = int(g.get('gap_len', 12))
    if xL is not None:
        p1L, p2L = _line_endpoints_at_x(xL, angle_v, y_top, y_bottom, w, h)
        _alpha_blend_line(frame_bgr, p1L, p2L, color_v, thick_v, alpha_v, style_v, dash_v, gap_v)
    if xR is not None:
        p1R, p2R = _line_endpoints_at_x(xR, angle_v, y_top, y_bottom, w, h)
        _alpha_blend_line(frame_bgr, p1R, p2R, color_v, thick_v, alpha_v, style_v, dash_v, gap_v)

    # horizontal
    top_val = g.get('top', None); bottom_val = g.get('bottom', None)
    if top_val is not None or bottom_val is not None:
        yT = _as_pos(top_val, h, mode) if top_val is not None else None
        yB = _as_pos(bottom_val, h, mode) if bottom_val is not None else None
        x_left_val  = g.get('x_left',  0.0 if mode == 'percent' else 0)
        x_right_val = g.get('x_right', 1.0 if mode == 'percent' else (w-1))
        x_left_px  = _as_pos(x_left_val,  w, mode)
        x_right_px = _as_pos(x_right_val, w, mode)
        angle_h = float(g.get('h_angle_deg', 0.0))
        color_h = tuple(int(c) for c in g.get('h_color', g.get('color', (0,255,255))))
        thick_h = int(g.get('h_thickness', g.get('thickness', 2)))
        alpha_h = float(g.get('h_alpha', g.get('alpha', 1.0)))
        style_h = g.get('h_style', g.get('style', 'solid'))
        dash_h = int(g.get('h_dash_len', g.get('dash_len', 16)))
        gap_h  = int(g.get('h_gap_len',  g.get('gap_len', 12)))
        if yT is not None:
            q1, q2 = _line_endpoints_at_y(yT, angle_h, x_left_px, x_right_px, w, h)
            _alpha_blend_line(frame_bgr, q1, q2, color_h, thick_h, alpha_h, style_h, dash_h, gap_h)
        if yB is not None:
            q1, q2 = _line_endpoints_at_y(yB, angle_h, x_left_px, x_right_px, w, h)
            _alpha_blend_line(frame_bgr, q1, q2, color_h, thick_h, alpha_h, style_h, dash_h, gap_h)

# ---------------- Overlay helpers ----------------
def draw_overlay_inplace(frame_bgr, fps_ema, camera_name, show_fps=True):
    cam_key = 'cam1' if camera_name.upper() == 'CAM1' else 'cam2'
    draw_pot_guides(frame_bgr, cam_key)
    if show_fps:
        text = f"{camera_name} FPS: {fps_ema:.1f}"
        font = cv2.FONT_HERSHEY_SIMPLEX; scale = 0.6; thick = 2
        size = cv2.getTextSize(text, font, scale, thick)[0]
        cv2.rectangle(frame_bgr, (12, 12),
                      (12 + size[0] + 12, 12 + size[1] + 12), (26, 26, 26), -1)
        cv2.rectangle(frame_bgr, (12, 12),
                      (12 + size[0] + 12, 12 + size[1] + 12), (127, 255, 16), 1)
        cv2.putText(frame_bgr, text, (18, 12 + size[1] + 3), font, scale,
                    (127, 255, 16), thick, cv2.LINE_AA)

# ---------------- ROS2 Subscribers ----------------
_ros_ok = False
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32, Float32MultiArray, String
    _ros_ok = True
except Exception as e:
    log.error(f"rclpy not available; cannot subscribe to ROS topics. ({e})")

def _ticks_from_array(msg):
    """รองรับความยาว >=4 (4 ล้อ) หรือ >=2"""
    try:
        arr = list(msg.data)
        if len(arr) >= 4:
            fl, fr, rl, rr = map(float, arr[:4])
            left = 0.5 * (fl + rl)
            right = 0.5 * (fr + rr)
            total = float(0.25 * (np.sum(arr[:4])))
            return fl, fr, rl, rr, left, right, total
        elif len(arr) >= 2:
            fl, fr = float(arr[0]), float(arr[1])
            rl, rr = 0.0, 0.0
            left = fl
            right = fr
            total = fl + fr
            return fl, fr, rl, rr, left, right, total
        elif len(arr) == 1:
            v = float(arr[0])
            return v, v, 0.0, 0.0, v, v, 2.0 * v
    except Exception:
        pass
    return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('camera_telemetry_bridge')
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, '/wheel_ticks', self._on_ticks, 10)
        self.create_subscription(Float32, '/yaw_rate', self._on_yaw_rate, 10)
        self.create_subscription(String, '/robot_heartbeat', self._on_heartbeat, 10)

    def _on_cmd_vel(self, msg: Twist):
        with ros_lock:
            ros_data['cmd_vel']['linear']['x'] = float(msg.linear.x)
            ros_data['cmd_vel']['linear']['y'] = float(msg.linear.y)
            ros_data['cmd_vel']['linear']['z'] = float(msg.linear.z)
            ros_data['cmd_vel']['angular']['x'] = float(msg.angular.x)
            ros_data['cmd_vel']['angular']['y'] = float(msg.angular.y)
            ros_data['cmd_vel']['angular']['z'] = float(msg.angular.z)

    def _on_ticks(self, msg: Float32MultiArray):
        fl, fr, rl, rr, left, right, total = _ticks_from_array(msg)
        with ros_lock:
            w = ros_data['wheel_ticks']
            w['fl'], w['fr'], w['rl'], w['rr'] = fl, fr, rl, rr
            w['left'], w['right'], w['total'] = left, right, total

    def _on_yaw_rate(self, msg: Float32):
        with ros_lock:
            ros_data['yaw_rate'] = float(msg.data)

    def _on_heartbeat(self, msg: String):
        with ros_lock:
            ros_data['robot_heartbeat'] = str(msg.data)

def start_ros_subscribers_in_thread():
    if not _ros_ok:
        return None
    rclpy.init(args=None)
    node = TelemetryNode()

    def _spin():
        try:
            rclpy.spin(node)
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    log.info("ROS subscribers started (spin in background thread)")
    return node

# ---------------- Capture Thread ----------------
def capture_loop(camera_id, device_id):
    if camera_id == 1:
        global fps_ema_cam1, last_tick_cam1
        jpeg_lock = jpeg_lock_cam1; bgr_lock = bgr_lock_cam1
        new_frame_event = new_frame_event_cam1
        camera_name = "CAM1"; width = CONFIG['width1']; height = CONFIG['height1']
    else:
        global fps_ema_cam2, last_tick_cam2
        jpeg_lock = jpeg_lock_cam2; bgr_lock = bgr_lock_cam2
        new_frame_event = new_frame_event_cam2
        camera_name = "CAM2"; width = CONFIG['width2']; height = CONFIG['height2']

    fps_target = CONFIG['fps']; flip = CONFIG['flip']; rotate = CONFIG['rotate']
    show_fps = CONFIG['show_fps']; jpeg_quality = CONFIG['quality']

    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    cap = None
    for backend in [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_ANY]:
        try:
            cap = cv2.VideoCapture(device_id, backend)
            if cap.isOpened():
                log.info(f"{camera_name} opened with backend: {backend}")
                break
        except Exception as e:
            log.warning(f"Open {camera_name} failed for backend {backend}: {e}")

    if cap is None or not cap.isOpened():
        log.error(f"Cannot open {camera_name} device {device_id}")
        dummy_frame = np.zeros((height, width, 3), dtype=np.uint8)
        dummy_frame.fill(26)
        cv2.putText(dummy_frame, f"NO {camera_name} DETECTED", (width//4, height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (232, 232, 232), 2)
        cv2.putText(dummy_frame, f"Device: {device_id} | {width}x{height}@{fps_target}fps",
                    (width//4, height//2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (127, 255, 16), 2)
        while True:
            frame_copy = dummy_frame.copy()
            draw_overlay_inplace(frame_copy, 0, camera_name, show_fps=show_fps)
            with bgr_lock:
                if camera_id == 1:
                    globals()['latest_bgr_cam1'] = frame_copy.copy()
                else:
                    globals()['latest_bgr_cam2'] = frame_copy.copy()
            try:
                jpeg_bytes = encode_jpeg(frame_copy, quality=jpeg_quality)
                with jpeg_lock:
                    if camera_id == 1:
                        globals()['latest_jpeg_cam1'] = jpeg_bytes
                        new_frame_event_cam1.set(); new_frame_event_cam1.clear()
                    else:
                        globals()['latest_jpeg_cam2'] = jpeg_bytes
                        new_frame_event_cam2.set(); new_frame_event_cam2.clear()
            except Exception as e:
                log.error(f"{camera_name} JPEG encode failed: {e}")
            time.sleep(1.0/fps_target)
    else:
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS, fps_target)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            try:
                cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                cap.set(cv2.CAP_PROP_EXPOSURE, -6)
            except Exception:
                pass
        except Exception as e:
            log.warning(f"{camera_name} some properties could not be set: {e}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS) or fps_target
    log.info(f"{camera_name} settings: {width}x{height}@{fps_target}fps | actual: {actual_w}x{actual_h}@{actual_fps:.1f}")

    target_frame_time = 1.0 / fps_target
    retry = 0; max_retry = 5

    while True:
        tick0 = time.monotonic()
        ret, frame = cap.read()
        if not ret:
            retry += 1
            log.warning(f"{camera_name} read() failed {retry}/{max_retry}")
            if retry >= max_retry:
                log.error(f"Too many {camera_name} read() failures, attempting to reopen...")
                cap.release(); time.sleep(1.0)
                cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
                if not cap.isOpened():
                    log.error(f"Failed to reopen {camera_name}")
                    time.sleep(2); continue
                retry = 0
            time.sleep(0.05); continue

        retry = 0
        if frame.shape[:2] != (height, width):
            frame = cv2.resize(frame, (width, height))
        if flip:
            frame = cv2.flip(frame, 1)
        if rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        now = time.monotonic()
        if camera_id == 1:
            dt = now - last_tick_cam1
            if dt > 0:
                inst = 1.0 / dt
                alpha = 0.1 if abs(inst - fps_ema_cam1) < 5 else 0.2
                fps_ema_cam1 = inst if fps_ema_cam1 == 0 else (alpha * inst + (1-alpha) * fps_ema_cam1)
            last_tick_cam1 = now; fps_current = fps_ema_cam1
        else:
            dt = now - last_tick_cam2
            if dt > 0:
                inst = 1.0 / dt
                alpha = 0.1 if abs(inst - fps_ema_cam2) < 5 else 0.2
                fps_ema_cam2 = inst if fps_ema_cam2 == 0 else (alpha * inst + (1-alpha) * fps_ema_cam2)
            last_tick_cam2 = now; fps_current = fps_ema_cam2

        draw_overlay_inplace(frame, fps_current, camera_name, show_fps=show_fps)

        with (bgr_lock_cam1 if camera_id == 1 else bgr_lock_cam2):
            if camera_id == 1:
                globals()['latest_bgr_cam1'] = frame.copy()
            else:
                globals()['latest_bgr_cam2'] = frame.copy()

        try:
            jpeg_bytes = encode_jpeg(frame, quality=CONFIG['quality'])
        except Exception as e:
            log.error(f"{camera_name} JPEG encode failed: {e}")
            time.sleep(0.001); continue

        with (jpeg_lock_cam1 if camera_id == 1 else jpeg_lock_cam2):
            if camera_id == 1:
                globals()['latest_jpeg_cam1'] = jpeg_bytes
                new_frame_event_cam1.set(); new_frame_event_cam1.clear()
            else:
                globals()['latest_jpeg_cam2'] = jpeg_bytes
                new_frame_event_cam2.set(); new_frame_event_cam2.clear()

        elapsed = time.monotonic() - tick0
        sleep_time = target_frame_time - elapsed
        if sleep_time > 0.001:
            time.sleep(sleep_time)

# ---------------- MJPEG stream generators ----------------
def mjpeg_generator_cam1():
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        if not new_frame_event_cam1.wait(timeout=2.0):
            continue
        with jpeg_lock_cam1:
            buf = latest_jpeg_cam1
        if buf is None:
            time.sleep(0.01); continue
        yield boundary + buf + b'\r\n'

def mjpeg_generator_cam2():
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        if not new_frame_event_cam2.wait(timeout=2.0):
            continue
        with jpeg_lock_cam2:
            buf = latest_jpeg_cam2
        if buf is None:
            time.sleep(0.01); continue
        yield boundary + buf + b'\r\n'

# ---------------- Routes ----------------
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video1")
def video1():
    return Response(
        mjpeg_generator_cam1(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control':'no-cache, no-store, must-revalidate',
                 'Pragma':'no-cache','Expires':'0',
                 'Access-Control-Allow-Origin':'*'}
    )

@app.route("/video2")
def video2():
    return Response(
        mjpeg_generator_cam2(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control':'no-cache, no-store, must-revalidate',
                 'Pragma':'no-cache','Expires':'0',
                 'Access-Control-Allow-Origin':'*'}
    )

@app.route("/api/telemetry")
def api_telemetry():
    global fps_ema_cam1, fps_ema_cam2, CONFIG, ros_data
    with ros_lock:
        current = {k: v for k, v in ros_data.items()}
    return jsonify({
        "timestamp": time.time(),
        "cameras": {
            "cam1": {
                "fps": round(fps_ema_cam1, 1),
                "resolution": {"width": CONFIG['width1'], "height": CONFIG['height1']},
                "status": "active" if latest_bgr_cam1 is not None else "inactive",
                "device": CONFIG['device1']
            },
            "cam2": {
                "fps": round(fps_ema_cam2, 1),
                "resolution": {"width": CONFIG['width2'], "height": CONFIG['height2']},
                "status": "active" if latest_bgr_cam2 is not None else "inactive",
                "device": CONFIG['device2']
            }
        },
        "target_fps": CONFIG['fps'],
        "configured_resolution": {
            "cam1": {"width": CONFIG['width1'], "height": CONFIG['height1']},
            "cam2": {"width": CONFIG['width2'], "height": CONFIG['height2']}
        },
        "overlays": {
            "guides": {
                "enabled": GUIDES.get('show_guides', False),
                "cam1": {k: v for k, v in GUIDES['cam1'].items() if k != 'color'},
                "cam2": {k: v for k, v in GUIDES['cam2'].items() if k != 'color'}
            }
        },
        # ส่งเฉพาะคีย์ที่ใช้จริง
        "ros_topics": current,
        "system": {"uptime": time.time(), "ui_theme": "dark", "quality": "optimized"}
    })

# ---------------- Helpers ----------------
def _parse_bgr(s):
    try:
        b, g, r = [int(x.strip()) for x in s.split(',')]
        return (b, g, r)
    except Exception:
        return None

def _parse_style(s):
    s = (s or '').strip().lower()
    return s if s in ('solid', 'dashed', 'dotted') else 'solid'

def _parse_mode(s):
    s = (s or '').strip().lower()
    return s if s in ('percent', 'pixel') else 'percent'

# ---------------- Main ----------------
def main():
    global CONFIG, GUIDES

    parser = argparse.ArgumentParser(description="Robot Camera Stream")
    # Cameras
    parser.add_argument('--device1', type=int, required=True, help='Camera 1 device index')
    parser.add_argument('--device2', type=int, required=True, help='Camera 2 device index')
    parser.add_argument('--width1', type=int, required=True, help='Camera 1 width')
    parser.add_argument('--height1', type=int, required=True, help='Camera 1 height')
    parser.add_argument('--width2', type=int, required=True, help='Camera 2 width')
    parser.add_argument('--height2', type=int, required=True, help='Camera 2 height')
    parser.add_argument('--fps', type=int, required=True, help='Camera FPS')
    parser.add_argument('--flip', type=int, default=0, help='Flip camera horizontally (0/1)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270], help='Rotate camera')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--quality', type=int, default=75, help='JPEG quality (1-100)')
    parser.add_argument('--show-fps', type=int, default=0, help='Show FPS overlay (0/1)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Server host')

    # Guides
    parser.add_argument('--show-guides', type=int, default=1, help='Show guide lines (0/1)')
    parser.add_argument('--c1-mode', type=str); parser.add_argument('--c2-mode', type=str)
    parser.add_argument('--c1-left', type=float); parser.add_argument('--c1-right', type=float)
    parser.add_argument('--c2-left', type=float); parser.add_argument('--c2-right', type=float)
    parser.add_argument('--c1-angle', type=float); parser.add_argument('--c2-angle', type=float)
    parser.add_argument('--c1-y-top', type=float); parser.add_argument('--c1-y-bottom', type=float)
    parser.add_argument('--c2-y-top', type=float); parser.add_argument('--c2-y-bottom', type=float)
    parser.add_argument('--c1-style', type=str); parser.add_argument('--c2-style', type=str)
    parser.add_argument('--c1-color', type=str); parser.add_argument('--c2-color', type=str)
    parser.add_argument('--c1-thickness', type=int); parser.add_argument('--c2-thickness', type=int)
    parser.add_argument('--c1-alpha', type=float); parser.add_argument('--c2-alpha', type=float)
    parser.add_argument('--c1-dash', type=int); parser.add_argument('--c1-gap', type=int)
    parser.add_argument('--c2-dash', type=int); parser.add_argument('--c2-gap', type=int)
    parser.add_argument('--c1-top', type=float); parser.add_argument('--c1-bottom', type=float)
    parser.add_argument('--c2-top', type=float); parser.add_argument('--c2-bottom', type=float)
    parser.add_argument('--c1-x-left', type=float); parser.add_argument('--c1-x-right', type=float)
    parser.add_argument('--c2-x-left', type=float); parser.add_argument('--c2-x-right', type=float)
    parser.add_argument('--c1-h-angle', type=float); parser.add_argument('--c2-h-angle', type=float)
    parser.add_argument('--c1-h-style', type=str); parser.add_argument('--c2-h-style', type=str)
    parser.add_argument('--c1-h-color', type=str); parser.add_argument('--c2-h-color', type=str)
    parser.add_argument('--c1-h-thickness', type=int); parser.add_argument('--c2-h-thickness', type=int)
    parser.add_argument('--c1-h-alpha', type=float); parser.add_argument('--c2-h-alpha', type=float)
    parser.add_argument('--c1-h-dash', type=int); parser.add_argument('--c1-h-gap', type=int)
    parser.add_argument('--c2-h-dash', type=int); parser.add_argument('--c2-h-gap', type=int)

    args, _unknown = parser.parse_known_args()

    CONFIG.update({
        'device1': args.device1, 'device2': args.device2,
        'width1': args.width1, 'height1': args.height1,
        'width2': args.width2, 'height2': args.height2,
        'fps': args.fps, 'port': args.port, 'host': args.host,
        'quality': args.quality, 'flip': bool(args.flip),
        'rotate': args.rotate, 'show_fps': bool(args.show_fps)
    })

    GUIDES['show_guides'] = bool(args.show_guides)
    if args.c1_mode: GUIDES['cam1']['mode'] = _parse_mode(args.c1_mode)
    if args.c2_mode: GUIDES['cam2']['mode'] = _parse_mode(args.c2_mode)
    if args.c1_left is not None:  GUIDES['cam1']['left']  = args.c1_left
    if args.c1_right is not None: GUIDES['cam1']['right'] = args.c1_right
    if args.c2_left is not None:  GUIDES['cam2']['left']  = args.c2_left
    if args.c2_right is not None: GUIDES['cam2']['right'] = args.c2_right
    if args.c1_angle is not None: GUIDES['cam1']['angle_deg'] = float(args.c1_angle)
    if args.c2_angle is not None: GUIDES['cam2']['angle_deg'] = float(args.c2_angle)
    if args.c1_y_top is not None: GUIDES['cam1']['y_top'] = float(args.c1_y_top)
    if args.c1_y_bottom is not None: GUIDES['cam1']['y_bottom'] = float(args.c1_y_bottom)
    if args.c2_y_top is not None: GUIDES['cam2']['y_top'] = float(args.c2_y_top)
    if args.c2_y_bottom is not None: GUIDES['cam2']['y_bottom'] = float(args.c2_y_bottom)
    if args.c1_style: GUIDES['cam1']['style'] = _parse_style(args.c1_style)
    if args.c2_style: GUIDES['cam2']['style'] = _parse_style(args.c2_style)
    c = _parse_bgr(args.c1_color) if args.c1_color else None
    if c: GUIDES['cam1']['color'] = c
    c = _parse_bgr(args.c2_color) if args.c2_color else None
    if c: GUIDES['cam2']['color'] = c
    if args.c1_thickness is not None: GUIDES['cam1']['thickness'] = int(args.c1_thickness)
    if args.c2_thickness is not None: GUIDES['cam2']['thickness'] = int(args.c2_thickness)
    if args.c1_alpha is not None: GUIDES['cam1']['alpha'] = max(0.0, min(1.0, float(args.c1_alpha)))
    if args.c2_alpha is not None: GUIDES['cam2']['alpha'] = max(0.0, min(1.0, float(args.c2_alpha)))
    if args.c1_dash is not None: GUIDES['cam1']['dash_len'] = int(args.c1_dash)
    if args.c1_gap is not None:  GUIDES['cam1']['gap_len']  = int(args.c1_gap)
    if args.c2_dash is not None: GUIDES['cam2']['dash_len'] = int(args.c2_dash)
    if args.c2_gap is not None:  GUIDES['cam2']['gap_len']  = int(args.c2_gap)
    if args.c1_top is not None:     GUIDES['cam1']['top'] = args.c1_top
    if args.c1_bottom is not None:  GUIDES['cam1']['bottom'] = args.c1_bottom
    if args.c2_top is not None:     GUIDES['cam2']['top'] = args.c2_top
    if args.c2_bottom is not None:  GUIDES['cam2']['bottom'] = args.c2_bottom
    if args.c1_x_left is not None:  GUIDES['cam1']['x_left']  = args.c1_x_left
    if args.c1_x_right is not None: GUIDES['cam1']['x_right'] = args.c1_x_right
    if args.c2_x_left is not None:  GUIDES['cam2']['x_left']  = args.c2_x_left
    if args.c2_x_right is not None: GUIDES['cam2']['x_right'] = args.c2_x_right
    if args.c1_h_angle is not None: GUIDES['cam1']['h_angle_deg'] = float(args.c1_h_angle)
    if args.c2_h_angle is not None: GUIDES['cam2']['h_angle_deg'] = float(args.c2_h_angle)
    if args.c1_h_style: GUIDES['cam1']['h_style'] = _parse_style(args.c1_h_style)
    if args.c2_h_style: GUIDES['cam2']['h_style'] = _parse_style(args.c2_h_style)
    c = _parse_bgr(args.c1_h_color) if args.c1_h_color else None
    if c: GUIDES['cam1']['h_color'] = c
    c = _parse_bgr(args.c2_h_color) if args.c2_h_color else None
    if c: GUIDES['cam2']['h_color'] = c
    if args.c1_h_thickness is not None: GUIDES['cam1']['h_thickness'] = int(args.c1_h_thickness)
    if args.c2_h_thickness is not None: GUIDES['cam2']['h_thickness'] = int(args.c2_h_thickness)
    if args.c1_h_alpha is not None: GUIDES['cam1']['h_alpha'] = max(0.0, min(1.0, float(args.c1_h_alpha)))
    if args.c2_h_alpha is not None: GUIDES['cam2']['h_alpha'] = max(0.0, min(1.0, float(args.c2_h_alpha)))
    if args.c1_h_dash is not None: GUIDES['cam1']['h_dash_len'] = int(args.c1_h_dash)
    if args.c1_h_gap is not None:  GUIDES['cam1']['h_gap_len']  = int(args.c1_h_gap)
    if args.c2_h_dash is not None: GUIDES['cam2']['h_dash_len'] = int(args.c2_h_dash)
    if args.c2_h_gap is not None:  GUIDES['cam2']['h_gap_len']  = int(args.c2_h_gap)

    log.info("Starting Robot Camera Stream System")
    log.info("=" * 60)
    log.info(f"Server: http://{CONFIG['host']}:{CONFIG['port']}")
    log.info(f"Camera 1: {CONFIG['width1']}×{CONFIG['height1']}@{CONFIG['fps']}fps (device {CONFIG['device1']})")
    log.info(f"Camera 2: {CONFIG['width2']}×{CONFIG['height2']}@{CONFIG['fps']}fps (device {CONFIG['device2']})")
    log.info(f"Quality: {CONFIG['quality']}% | Features: Guides={'ON' if GUIDES['show_guides'] else 'OFF'}")
    log.info(f"Guides CAM1: {GUIDES['cam1']}")
    log.info(f"Guides CAM2: {GUIDES['cam2']}")
    log.info("=" * 60)

    # เริ่ม ROS subscribers
    start_ros_subscribers_in_thread()

    # Start camera capture threads
    cam1_thread = threading.Thread(target=capture_loop, args=(1, CONFIG['device1']), daemon=True); cam1_thread.start()
    cam2_thread = threading.Thread(target=capture_loop, args=(2, CONFIG['device2']), daemon=True); cam2_thread.start()

    # Wait a bit for cameras
    log.info("Initializing camera system...")
    time.sleep(2)

    try:
        app.run(host=CONFIG['host'], port=CONFIG['port'], threaded=True, debug=False)
    except KeyboardInterrupt:
        log.info("\nSystem shutdown by user")
    except Exception as e:
        log.error(f"Server error: {e}")

if __name__ == "__main__":
    main()
