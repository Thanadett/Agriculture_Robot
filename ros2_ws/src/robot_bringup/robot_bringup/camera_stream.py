import os
import time
import threading
import collections
import math
import json
import platform
from dataclasses import dataclass, asdict
from typing import Optional, Tuple
from flask import Flask, render_template, Response, request, jsonify
import cv2
import numpy as np

app = Flask(__name__)

# ---------------- Config (dynamic) ----------------


@dataclass
class StreamConfig:
    device: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    flip: int = 1              # 1 = mirror, 0 = no flip
    show_fps: bool = True
    show_center: bool = True
    show_speed: bool = True
    show_debug: bool = True
    jpeg_quality: int = 80
    # สี overlay (B,G,R)
    color_text: Tuple[int, int, int] = (255, 255, 255)
    color_center: Tuple[int, int, int] = (0, 255, 255)


CONFIG = StreamConfig()
CONFIG_LOCK = threading.Lock()

# ---------------- Drive Command State ----------------


@dataclass
class DriveCmd:
    linear: float = 0.0   # m/s (commanded)
    angular: float = 0.0  # rad/s (commanded)
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

# ---------------- Capture Thread (single camera, multi-client) ----------------
latest_frame = None
latest_lock = threading.Lock()


def open_capture(device_index=0, w=640, h=480, fps=30):
    use_gst = os.getenv("USE_GST", "0") == "1"
    if use_gst:
        pipeline = (
            f"libcamerasrc ! video/x-raw,width={w},height={h},framerate={fps}/1 "
            "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true"
        )
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    else:
        cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def capture_loop():
    global latest_frame
    # วัด FPS capture
    t0 = time.time()
    frames = 0
    cap = None

    def reopen():
        nonlocal cap
        with CONFIG_LOCK:
            d, w, h, f = CONFIG.device, CONFIG.width, CONFIG.height, CONFIG.fps
        if cap:
            cap.release()
        cap = open_capture(d, w, h, f)

    reopen()
    if not cap or not cap.isOpened():
        print("Cannot open camera")
        time.sleep(2)

    while True:
        with CONFIG_LOCK:
            flip = CONFIG.flip
        if not cap or not cap.isOpened():
            reopen()
            time.sleep(0.5)
            continue
        ok, frame = cap.read()
        if not ok:
            with STATS_LOCK:
                stats["dropped"] += 1
            time.sleep(0.01)
            continue
        if flip:
            frame = cv2.flip(frame, 1)

        with latest_lock:
            latest_frame = frame

        frames += 1
        now = time.time()
        if now - t0 >= 1.0:
            with STATS_LOCK:
                stats["capture_fps"] = frames / (now - t0)
            t0 = now
            frames = 0


# Start capture thread
threading.Thread(target=capture_loop, daemon=True).start()

# ---------------- Overlay helpers ----------------


def draw_center_marker(img, color=(0, 255, 255)):
    h, w = img.shape[:2]
    cx, cy = w//2, h//2
    ln = max(10, min(w, h)//20)
    thickness = max(1, min(w, h)//200)
    cv2.line(img, (cx-ln, cy), (cx+ln, cy), color, thickness)
    cv2.line(img, (cx, cy-ln), (cx, cy+ln), color, thickness)
    cv2.circle(img, (cx, cy), thickness*2+1, color, -1)


def put_text(img, text, org, color=(255, 255, 255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 1, cv2.LINE_AA)


def compose_overlay(frame):
    # ใช้ค่าปัจจุบันแบบ thread-safe
    with CONFIG_LOCK:
        cfg = StreamConfig(**asdict(CONFIG))
    with DRIVE_LOCK:
        drv = DriveCmd(**asdict(DRIVE))
    with STATS_LOCK:
        cap_fps = stats["capture_fps"]
        enc_ms = stats["last_encode_ms"]

    img = frame.copy()

    # center marker
    if cfg.show_center:
        draw_center_marker(img, cfg.color_center)

    # debug text (FPS เซิร์ฟเวอร์, encode time)
    y = 20
    if cfg.show_fps or cfg.show_debug:
        if cfg.show_fps:
            put_text(img, f"FPS: {cap_fps:.1f}", (10, y), cfg.color_text)
            y += 18
        if cfg.show_debug:
            put_text(img, f"ENC: {enc_ms:.1f} ms", (10, y), cfg.color_text)
            y += 18
            put_text(img, f"TS: {time.strftime('%H:%M:%S')}",
                     (10, y), cfg.color_text)
            y += 18

    # speed indicator (commanded)
    if cfg.show_speed:
        put_text(
            img, f"v={drv.linear:+.2f} m/s | w={drv.angular:+.2f} rad/s", (10, y), (0, 255, 0))
        y += 18

    return img

# ---------------- MJPEG generator ----------------


def mjpeg_generator():
    with STATS_LOCK:
        stats["clients"] += 1
    try:
        # แถวต่อไปนี้จะใช้เฟรมล่าสุดเสมอ (shared-buffer)
        while True:
            with latest_lock:
                frame = latest_frame.copy() if latest_frame is not None else None
            if frame is None:
                time.sleep(0.01)
                continue

            # สร้าง overlay ตาม config
            composed = compose_overlay(frame)

            # เข้ารหัส JPEG
            t0 = time.time()
            with CONFIG_LOCK:
                q = CONFIG.jpeg_quality
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), q]
            ok, buf = cv2.imencode(".jpg", composed, encode_param)
            if not ok:
                continue
            jpg = buf.tobytes()
            dt = (time.time() - t0) * 1000.0
            with STATS_LOCK:
                stats["frames_encoded"] += 1
                stats["last_encode_ms"] = dt

            # multipart/x-mixed-replace
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
    except GeneratorExit:
        pass
    finally:
        with STATS_LOCK:
            stats["clients"] = max(0, stats["clients"] - 1)

# ---------------- Routes ----------------


@app.route("/")
def index():
    return render_template("template.html")


@app.route("/video")
def video():
    return Response(
        mjpeg_generator(),
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={'Cache-Control': 'no-store, no-cache, must-revalidate, max-age=0'}
    )


@app.route("/health")
def health():
    return jsonify(ok=True, ts=time.time())


@app.route("/stats")
def get_stats():
    # เพิ่ม CPU temp (เฉพาะ Linux/Pi ถ้ามี)
    cpu_temp = None
    if platform.system() == "Linux":
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                cpu_temp = int(f.read().strip()) / 1000.0
        except Exception:
            cpu_temp = None

    with CONFIG_LOCK:
        cfg = asdict(CONFIG)
    with STATS_LOCK:
        s = stats.copy()
    with DRIVE_LOCK:
        d = asdict(DRIVE)

    return jsonify(
        ok=True,
        config=cfg,
        stats=s,
        drive=d,
        cpu_temp=cpu_temp
    )


@app.route("/config", methods=["GET", "POST"])
def config():
    if request.method == "GET":
        with CONFIG_LOCK:
            return jsonify(asdict(CONFIG))
    data = request.get_json(force=True, silent=True) or {}
    changed = {}
    with CONFIG_LOCK:
        for k, v in data.items():
            if hasattr(CONFIG, k):
                setattr(CONFIG, k, v)
                changed[k] = v
    # ถ้ามีการเปลี่ยน W/H/FPS/DEVICE ให้รีโอเพนใน thread capture (จะจับจาก CONFIG อยู่แล้ว)
    return jsonify(ok=True, changed=changed)


@app.route("/api/drive", methods=["POST"])
def api_drive():
    """รับคำสั่งจาก virtual joystick: {linear: float, angular: float}"""
    data = request.get_json(force=True, silent=True) or {}
    lin = float(data.get("linear", 0.0))
    ang = float(data.get("angular", 0.0))
    with DRIVE_LOCK:
        DRIVE.linear = max(-1.0, min(1.0, lin))
        DRIVE.angular = max(-2.0, min(2.0, ang))
        DRIVE.ts = time.time()
    # TODO: ตรงนี้คุณสามารถต่อเข้ากับ ROS2 / GPIO ฯลฯ ได้
    return jsonify(ok=True, drive=asdict(DRIVE))


def main():
    app.run(host="0.0.0.0", port=5000, debug=False)


if __name__ == "__main__":
    main()
