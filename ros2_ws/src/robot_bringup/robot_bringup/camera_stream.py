#!/usr/bin/env python3
# Low-latency MJPEG camera with aim assist overlays (crosshair, corridor, heading ladder, steering cue)
# Units for drive overlay are "bits" (-255..+255) to match your teleop/serial bridge.

import argparse
import time
import threading
from dataclasses import dataclass, asdict
from typing import Tuple

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request

# ---------------- Flask ----------------
app = Flask(__name__)

# ---------------- Config ----------------
@dataclass
class StreamConfig:
    device: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    flip: int = 1                 # 1=mirror (selfie), 0=no flip
    rotate_deg: int = 0           # 0/90/180/270 optional
    jpeg_quality: int = 75        # 60..85 แนะนำ

    # Overlays
    show_fps: bool = True
    show_debug: bool = False
    show_crosshair: bool = True
    show_corridor: bool = True
    show_heading: bool = False
    show_speed: bool = True       # v/w "bits" overlay

    # Corridor (รางแปลง): สัดส่วนความกว้างจากกึ่งกลางภาพ (0..0.49)
    corridor_ratio: float = 0.35
    # Heading ladder: จำนวนเส้น
    heading_lines: int = 6

    color_text: Tuple[int, int, int] = (255, 255, 255)
    color_cross: Tuple[int, int, int] = (0, 255, 255)
    color_corridor: Tuple[int, int, int] = (0, 255, 0)
    color_heading: Tuple[int, int, int] = (255, 200, 0)
    color_steer: Tuple[int, int, int] = (50, 180, 255)

CONFIG = StreamConfig()
CONFIG_LOCK = threading.Lock()

# ---------------- Drive Command (bits) ----------------
@dataclass
class DriveCmd:
    linear: float = 0.0   # -255..+255
    angular: float = 0.0  # -255..+255
    ts: float = 0.0

DRIVE = DriveCmd()
DRIVE_LOCK = threading.Lock()

# ---------------- Stats ----------------
STATS_LOCK = threading.Lock()
stats = {
    "capture_fps": 0.0,
    "clients": 0,
    "frames_encoded": 0,
    "last_encode_ms": 0.0,
    "dropped": 0,
    "start_ts": time.time(),
}

# ---------------- Capture (latest-frame strategy) ----------------
latest_frame = None
latest_lock = threading.Lock()

def open_capture(device_index=0, w=640, h=480, fps=30):
    # ใช้ V4L2 + MJPG เพื่อลด latency / ลดภาระ decode
    cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
    try:
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    except Exception:
        pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS,          fps)
    # ลดค้างเฟรม (บางไดรเวอร์ไม่รองรับ แต่ไม่เป็นไร)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    return cap

def _maybe_rotate(frame, deg: int):
    if deg == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame

def capture_loop():
    global latest_frame
    frames = 0
    t0 = time.time()

    with CONFIG_LOCK:
        dev, w, h, f, flip, rot = CONFIG.device, CONFIG.width, CONFIG.height, CONFIG.fps, CONFIG.flip, CONFIG.rotate_deg
    cap = open_capture(dev, w, h, f)
    if not cap.isOpened():
        print(f"[camera] Cannot open device {dev}")
        time.sleep(2)

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            with STATS_LOCK:
                stats["dropped"] += 1
            time.sleep(0.002)
            continue

        if flip:
            frame = cv2.flip(frame, 1)
        if rot:
            frame = _maybe_rotate(frame, rot)

        with latest_lock:
            latest_frame = frame

        frames += 1
        now = time.time()
        if now - t0 >= 1.0:
            with STATS_LOCK:
                stats["capture_fps"] = frames / (now - t0)
            frames = 0
            t0 = now

threading.Thread(target=capture_loop, daemon=True).start()

# ---------------- Overlays (aim assist) ----------------
def put_text(img, text, org, color=(255, 255, 255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

def draw_crosshair(img, color=(0, 255, 255)):
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2
    ln = max(10, min(w, h) // 18)
    thick = max(1, min(w, h) // 200)
    cv2.line(img, (cx - ln, cy), (cx + ln, cy), color, thick)
    cv2.line(img, (cx, cy - ln), (cx, cy + ln), color, thick)
    cv2.circle(img, (cx, cy), thick * 2 + 1, color, -1)

def draw_corridor(img, ratio=0.35, color=(0, 255, 0)):
    # วาด "รางแปลง" สองเส้นแนวตั้งเพื่อช่วยเล็งให้อยู่กึ่งกลางแนวแปลง
    ratio = max(0.05, min(0.49, float(ratio)))
    h, w = img.shape[:2]
    cx = w // 2
    left_x = int(cx - w * ratio)
    right_x = int(cx + w * ratio)
    thick = max(1, w // 300)
    cv2.line(img, (left_x, 0), (left_x, h), color, thick)
    cv2.line(img, (right_x, 0), (right_x, h), color, thick)
    # เส้นฐานด้านล่างเป็นเป้าเข้าโซน
    base_y = int(h * 0.85)
    cv2.line(img, (left_x, base_y), (right_x, base_y), color, thick)

def draw_heading_ladder(img, n=6, color=(255, 200, 0)):
    # เส้นแนวนอนสั้นๆ ไล่ขึ้น เพื่อช่วยกะความเอียง/แนวตรงของหัวรถกับสันร่อง
    n = max(2, min(12, int(n)))
    h, w = img.shape[:2]
    cx = w // 2
    gap = h // (n + 2)
    half = max(8, w // 16)
    thick = max(1, w // 300)
    for i in range(1, n + 1):
        y = i * gap
        cv2.line(img, (cx - half, y), (cx + half, y), color, thick)

def draw_steer_cue(img, angular_bits, color=(50, 180, 255)):
    # ลูกศรบอกทิศเลี้ยวตาม "bits" (-255..+255) ช่วยประคองพวงมาลัย
    h, w = img.shape[:2]
    cx, cy = w // 2, int(h * 0.75)
    norm = max(-255.0, min(255.0, float(angular_bits))) / 255.0
    length = int((w * 0.25) * abs(norm))
    thick = max(2, w // 180)
    if abs(norm) < 1e-3:
        cv2.circle(img, (cx, cy), max(6, w // 90), color, -1)
        return
    if norm > 0:
        # เลี้ยวขวา →
        cv2.arrowedLine(img, (cx - length, cy), (cx + length, cy), color, thick, tipLength=0.25)
    else:
        # เลี้ยวซ้าย ←
        cv2.arrowedLine(img, (cx + length, cy), (cx - length, cy), color, thick, tipLength=0.25)

def compose_overlay(frame):
    # ทำงานบน "สำเนา" เพื่อความเสถียร (ไม่รบกวน capture thread)
    with CONFIG_LOCK:
        cfg = StreamConfig(**asdict(CONFIG))
    with DRIVE_LOCK:
        drv = DriveCmd(**asdict(DRIVE))
    with STATS_LOCK:
        cap_fps = stats["capture_fps"]
        enc_ms = stats["last_encode_ms"]

    img = frame.copy()

    if cfg.show_corridor:
        draw_corridor(img, cfg.corridor_ratio, cfg.color_corridor)
    if cfg.show_crosshair:
        draw_crosshair(img, cfg.color_cross)
    if cfg.show_heading:
        draw_heading_ladder(img, cfg.heading_lines, cfg.color_heading)
    if cfg.show_speed:
        put_text(img, f"v={drv.linear:+.0f} bits | w={drv.angular:+.0f} bits", (10, 22), cfg.color_text)
        draw_steer_cue(img, drv.angular, cfg.color_steer)

    y = 40
    if cfg.show_fps:
        put_text(img, f"FPS: {cap_fps:.1f}", (10, y), cfg.color_text); y += 18
    if cfg.show_debug:
        put_text(img, f"ENC: {enc_ms:.1f} ms", (10, y), cfg.color_text); y += 18
        put_text(img, f"TS: {time.strftime('%H:%M:%S')}", (10, y), cfg.color_text)

    return img

# ---------------- MJPEG ----------------
def mjpeg_generator():
    with STATS_LOCK:
        stats["clients"] += 1
    try:
        while True:
            with latest_lock:
                frame = None if (latest_frame is None) else latest_frame.copy()
            if frame is None:
                # ไม่มีเฟรมล่าสุด → รอไม่นานเพื่อลด busy-wait
                time.sleep(0.003)
                continue

            composed = compose_overlay(frame)

            t0 = time.time()
            with CONFIG_LOCK:
                q = int(CONFIG.jpeg_quality)
            ok, buf = cv2.imencode(".jpg", composed, [
                int(cv2.IMWRITE_JPEG_QUALITY), q,
                int(cv2.IMWRITE_JPEG_PROGRESSIVE), 0,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 0
            ])
            if not ok:
                continue
            dt_ms = (time.time() - t0) * 1000.0
            with STATS_LOCK:
                stats["frames_encoded"] += 1
                stats["last_encode_ms"] = dt_ms

            # ป้องกัน proxy buffering
            yield (b'--frame\r\n'
                   b'Cache-Control: no-store\r\n'
                   b'X-Accel-Buffering: no\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
    finally:
        with STATS_LOCK:
            stats["clients"] = max(0, stats["clients"] - 1)

# ---------------- Routes ----------------
@app.route("/")
def index():
    return "<h2>Camera Stream</h2><img src='/video'>"

@app.route("/video")
def video():
    return Response(mjpeg_generator(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/health")
def health():
    return jsonify(ok=True, ts=time.time())

@app.route("/stats")
def get_stats():
    with CONFIG_LOCK:
        cfg = asdict(CONFIG)
    with STATS_LOCK:
        s = stats.copy()
    with DRIVE_LOCK:
        d = asdict(DRIVE)
    return jsonify(ok=True, config=cfg, stats=s, drive=d)

@app.route("/config", methods=["GET", "POST"])
def config_route():
    if request.method == "GET":
        with CONFIG_LOCK:
            return jsonify(asdict(CONFIG))
    data = request.get_json(silent=True) or {}
    changed = {}
    with CONFIG_LOCK:
        for k, v in data.items():
            if hasattr(CONFIG, k):
                # type-safe tweaks
                if k in ("device", "width", "height", "fps", "flip", "rotate_deg",
                         "jpeg_quality", "heading_lines"):
                    v = int(v)
                if k in ("corridor_ratio",):
                    v = float(v)
                setattr(CONFIG, k, v)
                changed[k] = v
    return jsonify(ok=True, changed=changed)

@app.route("/api/drive", methods=["POST"])
def api_drive():
    data = request.get_json(force=True, silent=True) or {}
    # หน่วยเป็น "bits" (-255..+255)
    lin = float(data.get("linear", 0.0))
    ang = float(data.get("angular", 0.0))
    with DRIVE_LOCK:
        DRIVE.linear = max(-255.0, min(255.0, lin))
        DRIVE.angular = max(-255.0, min(255.0, ang))
        DRIVE.ts = time.time()
    return jsonify(ok=True, drive=asdict(DRIVE))

# ---------------- Main ----------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--flip', type=int, default=1)
    parser.add_argument('--rotate', dest='rotate_deg', type=int, default=0,
                        help='Rotate image by 0/90/180/270 degrees')
    parser.add_argument('--quality', dest='jpeg_quality', type=int, default=75)
    args = parser.parse_args()

    with CONFIG_LOCK:
        CONFIG.device = args.device
        CONFIG.width = args.width
        CONFIG.height = args.height
        CONFIG.fps = args.fps
        CONFIG.flip = args.flip
        CONFIG.rotate_deg = args.rotate_deg
        CONFIG.jpeg_quality = args.jpeg_quality

    # threaded=True เพื่อให้ /video ส่งได้พร้อมกับ /config
    app.run(host="0.0.0.0", port=args.port, debug=False, threaded=True)

if __name__ == "__main__":
    main()