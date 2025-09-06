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
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("lowlag-stream")

# ---------------- Flask ----------------
app = Flask(__name__)

# ---------------- Globals ----------------
latest_jpeg = None
jpeg_lock = threading.Lock()
new_frame_event = threading.Event()

latest_bgr = None
bgr_lock = threading.Lock()

show_grid = True
show_center_dot = True
grid_color = (100, 100, 100)    # BGR
center_dot_color = (0, 0, 255)  # BGR

fps_ema = 0.0
last_tick = time.monotonic()

_grid_cache = {"shape": None, "mask": None}

# ---------------- Try TurboJPEG ----------------
try:
    from turbojpeg import TurboJPEG, TJPF_BGR, TJSAMP_420
    _jpeg = TurboJPEG()
    def encode_jpeg(img_bgr, quality=85):
        return _jpeg.encode(img_bgr, quality=quality, pixel_format=TJPF_BGR, jpeg_subsample=TJSAMP_420)
    log.info("TurboJPEG enabled")
except Exception:
    _jpeg = None
    def encode_jpeg(img_bgr, quality=85):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality), int(cv2.IMWRITE_JPEG_OPTIMIZE), 1]
        ok, buf = cv2.imencode(".jpg", img_bgr, params)
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes()
    log.info("TurboJPEG not available, falling back to cv2.imencode")

# ---------------- Overlay helpers ----------------
def _ensure_grid_mask(shape):
    global _grid_cache
    if _grid_cache["shape"] == shape and _grid_cache["mask"] is not None:
        return _grid_cache["mask"]
    h, w = shape[:2]
    mask = np.zeros((h, w, 3), dtype=np.uint8)
    for i in (1, 2):
        x = (w * i) // 3
        cv2.line(mask, (x, 0), (x, h), grid_color, 1)
        y = (h * i) // 3
        cv2.line(mask, (0, y), (w, y), grid_color, 1)
    _grid_cache = {"shape": shape, "mask": mask}
    return mask

def draw_overlay_inplace(frame_bgr, show_fps=True):
    global fps_ema
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    # grid
    if show_grid:
        frame_bgr[:] = cv2.add(frame_bgr, _ensure_grid_mask(frame_bgr.shape))
    # center dot
    if show_center_dot:
        cv2.circle(frame_bgr, (cx, cy), 4, center_dot_color, -1, cv2.LINE_AA)
    # fps
    if show_fps:
        text = f"FPS: {fps_ema:.1f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thick = 2
        size = cv2.getTextSize(text, font, scale, thick)[0]
        cv2.rectangle(frame_bgr, (10, 10), (10 + size[0] + 10, 10 + size[1] + 10), (0, 0, 0), -1)
        cv2.putText(frame_bgr, text, (15, 10 + size[1] + 0), font, scale, (0, 255, 0), thick, cv2.LINE_AA)
    # coords
    coord_text = f"Center: ({cx},{cy})"
    csize = cv2.getTextSize(coord_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
    cv2.rectangle(frame_bgr, (w - csize[0] - 15, 10), (w - 5, 10 + csize[1] + 8), (0, 0, 0), -1)
    cv2.putText(frame_bgr, coord_text, (w - csize[0] - 10, 10 + csize[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

# ---------------- Capture Thread ----------------
def capture_loop(device=0, width=800, height=600, fps=30, flip=True, rotate=0, show_fps=True, jpeg_quality=85):
    global latest_jpeg, latest_bgr, fps_ema, last_tick
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    cap = None
    for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
        try:
            cap = cv2.VideoCapture(device, backend)
            if cap.isOpened():
                log.info(f"Camera opened with backend: {backend}")
                break
        except Exception as e:
            log.warning(f"Open camera failed for backend {backend}: {e}")

    if cap is None or not cap.isOpened():
        log.error(f"Cannot open camera device {device}")
        return

    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    except Exception as e:
        log.warning(f"Some camera properties could not be set: {e}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS) or fps
    log.info(f"Camera requested: {width}x{height}@{fps} | actual: {actual_w}x{actual_h}@{actual_fps:.1f}")

    target_frame_time = 1.0 / max(1.0, min(actual_fps, float(fps)))
    retry = 0
    max_retry = 5

    while True:
        tick0 = time.monotonic()
        ret, frame = cap.read()
        if not ret:
            retry += 1
            log.warning(f"read() failed {retry}/{max_retry}")
            if retry >= max_retry:
                log.error("Too many read() failures, reopening camera ...")
                cap.release()
                time.sleep(0.5)
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                retry = 0
            time.sleep(0.02)
            continue

        if flip:
            frame = cv2.flip(frame, 1)
        if rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        draw_overlay_inplace(frame, show_fps=show_fps)

        now = time.monotonic()
        dt = now - last_tick
        if dt > 0:
            inst = 1.0 / dt
            fps_ema = inst if fps_ema == 0 else (0.2 * inst + 0.8 * fps_ema)
        last_tick = now

        with bgr_lock:
            latest_bgr = frame

        try:
            jpeg_bytes = encode_jpeg(frame, quality=jpeg_quality)
        except Exception as e:
            log.error(f"JPEG encode failed: {e}")
            time.sleep(0.01)
            continue

        with jpeg_lock:
            latest_jpeg = jpeg_bytes
            new_frame_event.set()
            new_frame_event.clear()

        elapsed = time.monotonic() - tick0
        sleep_time = target_frame_time - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

# ---------------- MJPEG stream ----------------
def mjpeg_generator():
    boundary = b'--frame\\r\\nContent-Type: image/jpeg\\r\\n\\r\\n'
    while True:
        if not new_frame_event.wait(timeout=1.0):
            pass
        with jpeg_lock:
            buf = latest_jpeg
        if buf is None:
            time.sleep(0.01)
            continue
        yield boundary + buf + b'\\r\\n'

# ---------------- Routes ----------------
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video")
def video():
    return Response(mjpeg_generator(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/toggle/<overlay_type>")
def toggle_overlay(overlay_type):
    global show_grid, show_center_dot
    if overlay_type == 'grid':
        show_grid = not show_grid
        return f"Grid {'enabled' if show_grid else 'disabled'}"
    elif overlay_type == 'center':
        show_center_dot = not show_center_dot
        return f"Center dot {'enabled' if show_center_dot else 'disabled'}"
    return "Invalid overlay type"

# --- Stub API for future telemetry (encoders/IMU) ---
@app.route("/api/telemetry")
def api_telemetry():
    return jsonify({
        "t": time.time(),
        "encoders": {"fl": 0, "fr": 0, "rl": 0, "rr": 0},
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "notes": "stub data; replace with real sensors later"
    })

# ---------------- Main ----------------
def main():
    parser = argparse.ArgumentParser(description="Low-latency MJPEG camera stream with overlay (templated UI)")
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--width', type=int, default=800)
    parser.add_argument('--height', type=int, default=600)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--flip', type=int, default=1)
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270])
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--quality', type=int, default=85)
    parser.add_argument('--show-fps', type=int, default=1)
    args = parser.parse_args()

    log.info("Starting camera system (templated UI)")
    log.info(f"Server port: {args.port}")
    log.info(f"Camera settings: {args.width}x{args.height} @ {args.fps}fps | flip={bool(args.flip)} rotate={args.rotate}")

    th_cap = threading.Thread(
        target=capture_loop,
        kwargs=dict(
            device=args.device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            flip=bool(args.flip),
            rotate=args.rotate,
            show_fps=bool(args.show_fps),
            jpeg_quality=args.quality,
        ),
        daemon=True
    )
    th_cap.start()

    for _ in range(100):
        with jpeg_lock:
            if latest_jpeg is not None:
                break
        time.sleep(0.02)
    log.info("Camera system ready!")

    try:
        app.run(host="0.0.0.0", port=args.port, threaded=True, debug=False)
    except KeyboardInterrupt:
        log.info("System shutdown by user")
    except Exception as e:
        log.error(f"Server error: {e}")

if __name__ == "__main__":
    main()