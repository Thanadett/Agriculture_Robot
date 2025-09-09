#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
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

show_grid = False  
show_center_dot = True  
grid_color = (100, 100, 100)    # BGR (not used)
center_dot_color = (0, 0, 255)  # BGR - green center dot

fps_ema = 0.0
last_tick = time.monotonic()

# Global variables to store configuration from command line
CONFIG = {
    'device': 0,
    'width': 800,
    'height': 600,
    'fps': 30,
    'port': 5000,
    'host': '0.0.0.0',
    'quality': 75,
    'flip': False,
    'rotate': 0,
    'show_fps': True
}

# ---------------- Try TurboJPEG ----------------
try:
    from turbojpeg import TurboJPEG, TJPF_BGR, TJSAMP_420
    _jpeg = TurboJPEG()
    def encode_jpeg(img_bgr, quality=75):
        return _jpeg.encode(img_bgr, quality=quality, pixel_format=TJPF_BGR, jpeg_subsample=TJSAMP_420)
    log.info("TurboJPEG enabled")
except Exception:
    _jpeg = None
    def encode_jpeg(img_bgr, quality=75):
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality), int(cv2.IMWRITE_JPEG_OPTIMIZE), 1]
        ok, buf = cv2.imencode(".jpg", img_bgr, params)
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes()
    log.info("TurboJPEG not available, falling back to cv2.imencode")

# ---------------- Overlay helpers ----------------
def draw_overlay_inplace(frame_bgr, show_fps=True):
    """Draw overlay on frame using configuration from CONFIG global"""
    global fps_ema
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    
    if show_center_dot:
        cv2.circle(frame_bgr, (cx, cy), 5, center_dot_color, -1, cv2.LINE_AA)
        cv2.circle(frame_bgr, (cx, cy), 7, (255, 255, 255), 1, cv2.LINE_AA)
    
    # Clean FPS overlay
    if show_fps:
        text = f"FPS: {fps_ema:.1f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thick = 2
        size = cv2.getTextSize(text, font, scale, thick)[0]
        # Dark background
        cv2.rectangle(frame_bgr, (12, 12), (12 + size[0] + 12, 12 + size[1] + 12), (26, 26, 26), -1)
        cv2.rectangle(frame_bgr, (12, 12), (12 + size[0] + 12, 12 + size[1] + 12), (127, 255, 16), 1)
        cv2.putText(frame_bgr, text, (18, 12 + size[1] + 3), font, scale, (127, 255, 16), thick, cv2.LINE_AA)
    
# ---------------- Capture Thread ----------------
def capture_loop():
    """Camera capture loop using configuration from CONFIG global"""
    global latest_jpeg, latest_bgr, fps_ema, last_tick, CONFIG
    
    # Initialize frame counting variables
    frame_count = 0
    fps_check_interval = 30  # Check FPS every 30 frames
    start_time = time.monotonic()
    
    # Get configuration values
    device = CONFIG['device']
    width = CONFIG['width']
    height = CONFIG['height']
    fps = CONFIG['fps']
    flip = CONFIG['flip']
    rotate = CONFIG['rotate']
    show_fps = CONFIG['show_fps']
    jpeg_quality = CONFIG['quality']
    
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    cap = None
    # Try different backends
    for backend in [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_ANY]:
        try:
            cap = cv2.VideoCapture(device, backend)
            if cap.isOpened():
                log.info(f"Camera opened with backend: {backend}")
                break
        except Exception as e:
            log.warning(f"Open camera failed for backend {backend}: {e}")

    if cap is None or not cap.isOpened():
        log.error(f"Cannot open camera device {device}")
        # Create dummy frame for testing with configured dimensions
        dummy_frame = np.zeros((height, width, 3), dtype=np.uint8)
        # Dark background 
        dummy_frame.fill(26)
        cv2.putText(dummy_frame, "NO CAMERA DETECTED", (width//4, height//2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (232, 232, 232), 2)
        cv2.putText(dummy_frame, f"Device: {device} | {width}x{height}@{fps}fps", (width//4, height//2 + 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (127, 255, 16), 2)
        
        while True:
            frame_copy = dummy_frame.copy()
            draw_overlay_inplace(frame_copy, show_fps=show_fps)
            
            with bgr_lock:
                latest_bgr = frame_copy.copy()
                
            try:
                jpeg_bytes = encode_jpeg(frame_copy, quality=jpeg_quality)
                with jpeg_lock:
                    latest_jpeg = jpeg_bytes
                    new_frame_event.set()
                    new_frame_event.clear()
            except Exception as e:
                log.error(f"JPEG encode failed: {e}")
            time.sleep(1.0/fps)
        return

    try:
        # Set camera properties using configured values
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Additional settings for better quality
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # Lower exposure for less motion blur
        except:
            pass
            
    except Exception as e:
        log.warning(f"Some camera properties could not be set: {e}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS) or fps
    log.info(f"Camera settings: {width}x{height}@{fps}fps | actual: {actual_w}x{actual_h}@{actual_fps:.1f}")

    target_frame_time = 1.0 / fps
    retry = 0
    max_retry = 5

    while True:
        tick0 = time.monotonic()
        ret, frame = cap.read()
        
        if not ret:
            retry += 1
            log.warning(f"Camera read() failed {retry}/{max_retry}")
            if retry >= max_retry:
                log.error("Too many read() failures, attempting to reopen camera...")
                cap.release()
                time.sleep(1.0)
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                if not cap.isOpened():
                    log.error("Failed to reopen camera")
                    time.sleep(2)
                    continue
                retry = 0
            time.sleep(0.05)
            continue
        
        retry = 0  # Reset retry counter on successful read
        frame_count += 1  # Increment frame counter
        
        # Performance monitoring every N frames
        if frame_count % fps_check_interval == 0:
            elapsed_time = time.monotonic() - start_time
            measured_fps = frame_count / elapsed_time
            log.debug(f"Performance check: {measured_fps:.1f} FPS over {frame_count} frames")
            
            # Reset counters periodically to avoid overflow
            if frame_count >= 300:
                frame_count = 0
                start_time = time.monotonic()
        
        # Ensure frame is the correct resolution
        if frame.shape[:2] != (height, width):
            frame = cv2.resize(frame, (width, height))
        
        # Apply transformations
        if flip:
            frame = cv2.flip(frame, 1)
        if rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        draw_overlay_inplace(frame, show_fps=show_fps)

        # Calculate EMA FPS more smoothly
        now = time.monotonic()
        dt = now - last_tick
        if dt > 0:
            inst = 1.0 / dt
            # Use different smoothing factor based on FPS stability
            alpha = 0.1 if abs(inst - fps_ema) < 5 else 0.2
            fps_ema = inst if fps_ema == 0 else (alpha * inst + (1-alpha) * fps_ema)
        last_tick = now

        # Store latest BGR frame
        with bgr_lock:
            latest_bgr = frame.copy()

        # Encode to JPEG with error handling
        try:
            jpeg_bytes = encode_jpeg(frame, quality=jpeg_quality)
        except Exception as e:
            log.error(f"JPEG encode failed: {e}")
            time.sleep(0.001)
            continue

        # Store latest JPEG
        with jpeg_lock:
            latest_jpeg = jpeg_bytes
            new_frame_event.set()
            new_frame_event.clear()

        # Adaptive frame rate limiting
        elapsed = time.monotonic() - tick0
        sleep_time = target_frame_time - elapsed
        
        # Only sleep if we have significant time left
        if sleep_time > 0.001:
            time.sleep(sleep_time)
        elif sleep_time < -0.01:
            # If we're running significantly behind, log it
            log.debug(f"Frame processing taking too long: {elapsed:.3f}s (target: {target_frame_time:.3f}s)")

# ---------------- MJPEG stream ----------------
def mjpeg_generator():
    """Generate MJPEG stream"""
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        # Wait for new frame with timeout
        if not new_frame_event.wait(timeout=2.0):
            continue
            
        with jpeg_lock:
            buf = latest_jpeg
            
        if buf is None:
            time.sleep(0.01)
            continue
            
        yield boundary + buf + b'\r\n'

# ---------------- Routes ----------------
@app.route("/")
def index():
    """Main page - serve the static HTML file"""
    return render_template("index.html")

@app.route("/video")
def video():
    """Video stream route"""
    return Response(
        mjpeg_generator(), 
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
            'Expires': '0',
            'Access-Control-Allow-Origin': '*'
        }
    )

@app.route("/api/telemetry")
def api_telemetry():
    """Telemetry API using dynamic configuration"""
    global latest_bgr, fps_ema, CONFIG
    
    # Use configured resolution
    h, w = CONFIG['height'], CONFIG['width']
    cx, cy = w//2, h//2
    
    with bgr_lock:
        if latest_bgr is not None:
            h, w = latest_bgr.shape[:2]
            cx, cy = w//2, h//2
    
    return jsonify({
        "timestamp": time.time(),
        "fps": round(fps_ema, 1),
        "resolution": {"width": w, "height": h},
        "target_fps": CONFIG['fps'],
        "configured_resolution": {"width": CONFIG['width'], "height": CONFIG['height']},
        "device": CONFIG['device'],
        "overlays": {
            "grid": False, 
            "center_dot": True
        },
        "camera_status": "active" if latest_bgr is not None else "inactive",
        "system": {
            "uptime": time.time(),
            "ui_theme": "dark",
            "quality": "optimized"
        },
        "encoders": {"fl": 0, "fr": 0, "rl": 0, "rr": 0},
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    })

# ---------------- Main ----------------
def main():
    global CONFIG
    
    parser = argparse.ArgumentParser(description="Robot Camera Stream")
    parser.add_argument('--device', type=int, help='Camera device index')
    parser.add_argument('--width', type=int, help='Camera width')
    parser.add_argument('--height', type=int, help='Camera height')
    parser.add_argument('--fps', type=int, help='Camera FPS')
    parser.add_argument('--flip', type=int, default=0, help='Flip camera horizontally (0/1)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270], help='Rotate camera')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--quality', type=int, default=75,help='JPEG quality (1-100)')
    parser.add_argument('--show-fps', type=int, default=1, help='Show FPS overlay (0/1)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Server host')
    args = parser.parse_args()

    # Validate required arguments from launch file
    if args.device is None:
        log.error("--device is required from launch file")
        return
    if args.width is None:
        log.error("--width is required from launch file")
        return
    if args.height is None:
        log.error("--height is required from launch file")
        return
    if args.fps is None:
        log.error("--fps is required from launch file")
        return

    # Store all configuration in global CONFIG
    CONFIG.update({
        'device': args.device,
        'width': args.width,
        'height': args.height,
        'fps': args.fps,
        'port': args.port,
        'host': args.host,
        'quality': args.quality,
        'flip': bool(args.flip),
        'rotate': args.rotate,
        'show_fps': bool(args.show_fps)
    })

    log.info("Starting Robot Camera Stream System")
    log.info("=" * 50)
    log.info(f"Server: http://{CONFIG['host']}:{CONFIG['port']}")
    log.info(f"Camera: {CONFIG['width']}×{CONFIG['height']}@{CONFIG['fps']}fps (device {CONFIG['device']})")
    log.info(f"Quality: {CONFIG['quality']}% | Features: Center dot overlay")
    log.info(f"Options: flip={CONFIG['flip']}, rotate={CONFIG['rotate']}°")
    log.info("Configuration from launch file successfully loaded")
    log.info("=" * 50)

    # Start camera capture thread - no parameters needed, uses global CONFIG
    th_cap = threading.Thread(
        target=capture_loop,
        daemon=True
    )
    th_cap.start()

    # Wait for camera to initialize
    log.info("Initializing camera system...")
    initialization_success = False
    
    for i in range(200):  # Wait up to 10 seconds
        with jpeg_lock:
            if latest_jpeg is not None:
                initialization_success = True
                break
        time.sleep(0.05)
        if i % 20 == 0:
            log.info(f"Still waiting for camera... ({i//20 + 1}/10)")
    
    if initialization_success:
        log.info("Camera system ready!")
        log.info(f"Using configuration: {CONFIG['width']}x{CONFIG['height']}@{CONFIG['fps']}fps on device {CONFIG['device']}")
    else:
        log.warning("Camera not detected within timeout, but starting server anyway...")
        log.warning("Server will show dummy feed until camera becomes available")
    

    try:
        app.run(host=CONFIG['host'], port=CONFIG['port'], threaded=True, debug=False)
    except KeyboardInterrupt:
        log.info("\nSystem shutdown by user")
    except Exception as e:
        log.error(f"Server error: {e}")

if __name__ == "__main__":
    main()
    parser.add_argument