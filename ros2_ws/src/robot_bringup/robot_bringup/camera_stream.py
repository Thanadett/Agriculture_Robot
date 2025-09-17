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

# Common settings
show_grid = False  
show_center_dot = True  
grid_color = (100, 100, 100)    # BGR (not used)
center_dot_color = (0, 0, 255)  # BGR - red center dot

# Global variables to store configuration from command line
CONFIG = {
    'device1': 0,
    'device2': 1,
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

# ROS-like data simulation (replace with actual ROS subscriber)
ros_data = {
    'cmd_vel': {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
    'imu': {'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}, 'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
    'battery': {'voltage': 12.0, 'percentage': 85},
    'odometry': {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'velocity': {'linear': 0.0, 'angular': 0.0}}
}
ros_lock = threading.Lock()

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
def draw_overlay_inplace(frame_bgr, fps_ema, camera_name, show_fps=True):
    """Draw overlay on frame"""
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    
    if show_center_dot:
        cv2.circle(frame_bgr, (cx, cy), 5, center_dot_color, -1, cv2.LINE_AA)
        cv2.circle(frame_bgr, (cx, cy), 7, (255, 255, 255), 1, cv2.LINE_AA)
    
    # Clean FPS overlay with camera name
    if show_fps:
        text = f"{camera_name} FPS: {fps_ema:.1f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.6
        thick = 2
        size = cv2.getTextSize(text, font, scale, thick)[0]
        # Dark background
        cv2.rectangle(frame_bgr, (12, 12), (12 + size[0] + 12, 12 + size[1] + 12), (26, 26, 26), -1)
        cv2.rectangle(frame_bgr, (12, 12), (12 + size[0] + 12, 12 + size[1] + 12), (127, 255, 16), 1)
        cv2.putText(frame_bgr, text, (18, 12 + size[1] + 3), font, scale, (127, 255, 16), thick, cv2.LINE_AA)

# ---------------- ROS Data Simulator ----------------
def ros_data_simulator():
    """Simulate ROS topic data updates"""
    global ros_data
    
    while True:
        with ros_lock:
            # Simulate cmd_vel changes
            ros_data['cmd_vel']['linear']['x'] = np.sin(time.time() * 0.5) * 2.0
            ros_data['cmd_vel']['angular']['z'] = np.cos(time.time() * 0.3) * 1.5
            
            # Simulate IMU data
            ros_data['imu']['angular_velocity']['z'] = np.sin(time.time() * 0.2) * 0.5
            
            # Simulate battery discharge
            ros_data['battery']['percentage'] = max(20, 100 - (time.time() % 100))
            ros_data['battery']['voltage'] = 10.0 + ros_data['battery']['percentage'] * 0.04
            
            # Simulate odometry
            ros_data['odometry']['position']['x'] += ros_data['cmd_vel']['linear']['x'] * 0.1
            ros_data['odometry']['position']['y'] += ros_data['cmd_vel']['linear']['y'] * 0.1
            ros_data['odometry']['velocity']['linear'] = abs(ros_data['cmd_vel']['linear']['x'])
            ros_data['odometry']['velocity']['angular'] = abs(ros_data['cmd_vel']['angular']['z'])
            
        time.sleep(0.1)

# ---------------- Capture Thread ----------------
def capture_loop(camera_id, device_id):
    """Camera capture loop for specific camera"""
    if camera_id == 1:
        global latest_jpeg_cam1, latest_bgr_cam1, fps_ema_cam1, last_tick_cam1
        latest_jpeg = latest_jpeg_cam1
        latest_bgr = latest_bgr_cam1
        fps_ema = fps_ema_cam1
        last_tick = last_tick_cam1
        jpeg_lock = jpeg_lock_cam1
        bgr_lock = bgr_lock_cam1
        new_frame_event = new_frame_event_cam1
        camera_name = "CAM1"
    else:
        global latest_jpeg_cam2, latest_bgr_cam2, fps_ema_cam2, last_tick_cam2
        latest_jpeg = latest_jpeg_cam2
        latest_bgr = latest_bgr_cam2
        fps_ema = fps_ema_cam2
        last_tick = last_tick_cam2
        jpeg_lock = jpeg_lock_cam2
        bgr_lock = bgr_lock_cam2
        new_frame_event = new_frame_event_cam2
        camera_name = "CAM2"
    
    # Initialize frame counting variables
    frame_count = 0
    fps_check_interval = 30
    start_time = time.monotonic()
    
    # Get configuration values
    width = CONFIG['width']
    height = CONFIG['height']
    fps_target = CONFIG['fps']
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
            cap = cv2.VideoCapture(device_id, backend)
            if cap.isOpened():
                log.info(f"{camera_name} opened with backend: {backend}")
                break
        except Exception as e:
            log.warning(f"Open {camera_name} failed for backend {backend}: {e}")

    if cap is None or not cap.isOpened():
        log.error(f"Cannot open {camera_name} device {device_id}")
        # Create dummy frame
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
                        new_frame_event_cam1.set()
                        new_frame_event_cam1.clear()
                    else:
                        globals()['latest_jpeg_cam2'] = jpeg_bytes
                        new_frame_event_cam2.set()
                        new_frame_event_cam2.clear()
            except Exception as e:
                log.error(f"{camera_name} JPEG encode failed: {e}")
            time.sleep(1.0/fps_target)
        return

    try:
        # Set camera properties
        cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap1.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap1.set(cv2.CAP_PROP_FPS, fps_target)
        cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap2.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap2.set(cv2.CAP_PROP_FPS, fps_target)
        cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Additional settings
        try:
            cap1.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cap1.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap1.set(cv2.CAP_PROP_EXPOSURE, -6)

            cap2.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cap2.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap2.set(cv2.CAP_PROP_EXPOSURE, -6)
        except:
            pass
            
    except Exception as e:
        log.warning(f"{camera_name} some properties could not be set: {e}")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS) or fps_target
    log.info(f"{camera_name} settings: {width}x{height}@{fps_target}fps | actual: {actual_w}x{actual_h}@{actual_fps:.1f}")

    target_frame_time = 1.0 / fps_target
    retry = 0
    max_retry = 5

    while True:
        tick0 = time.monotonic()
        ret, frame = cap.read()
        
        if not ret:
            retry += 1
            log.warning(f"{camera_name} read() failed {retry}/{max_retry}")
            if retry >= max_retry:
                log.error(f"Too many {camera_name} read() failures, attempting to reopen...")
                cap.release()
                time.sleep(1.0)
                cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
                if not cap.isOpened():
                    log.error(f"Failed to reopen {camera_name}")
                    time.sleep(2)
                    continue
                retry = 0
            time.sleep(0.05)
            continue
        
        retry = 0
        frame_count += 1
        
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

        # Calculate FPS
        now = time.monotonic()
        if camera_id == 1:
            dt = now - last_tick_cam1
            if dt > 0:
                inst = 1.0 / dt
                alpha = 0.1 if abs(inst - fps_ema_cam1) < 5 else 0.2
                fps_ema_cam1 = inst if fps_ema_cam1 == 0 else (alpha * inst + (1-alpha) * fps_ema_cam1)
            last_tick_cam1 = now
            fps_current = fps_ema_cam1
        else:
            dt = now - last_tick_cam2
            if dt > 0:
                inst = 1.0 / dt
                alpha = 0.1 if abs(inst - fps_ema_cam2) < 5 else 0.2
                fps_ema_cam2 = inst if fps_ema_cam2 == 0 else (alpha * inst + (1-alpha) * fps_ema_cam2)
            last_tick_cam2 = now
            fps_current = fps_ema_cam2

        draw_overlay_inplace(frame, fps_current, camera_name, show_fps=show_fps)

        # Store latest BGR frame
        with bgr_lock:
            if camera_id == 1:
                globals()['latest_bgr_cam1'] = frame.copy()
            else:
                globals()['latest_bgr_cam2'] = frame.copy()

        # Encode to JPEG
        try:
            jpeg_bytes = encode_jpeg(frame, quality=jpeg_quality)
        except Exception as e:
            log.error(f"{camera_name} JPEG encode failed: {e}")
            time.sleep(0.001)
            continue

        # Store latest JPEG
        with jpeg_lock:
            if camera_id == 1:
                globals()['latest_jpeg_cam1'] = jpeg_bytes
                new_frame_event_cam1.set()
                new_frame_event_cam1.clear()
            else:
                globals()['latest_jpeg_cam2'] = jpeg_bytes
                new_frame_event_cam2.set()
                new_frame_event_cam2.clear()

        # Frame rate limiting
        elapsed = time.monotonic() - tick0
        sleep_time = target_frame_time - elapsed
        
        if sleep_time > 0.001:
            time.sleep(sleep_time)

# ---------------- MJPEG stream generators ----------------
def mjpeg_generator_cam1():
    """Generate MJPEG stream for camera 1"""
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        if not new_frame_event_cam1.wait(timeout=2.0):
            continue
            
        with jpeg_lock_cam1:
            buf = latest_jpeg_cam1
            
        if buf is None:
            time.sleep(0.01)
            continue
            
        yield boundary + buf + b'\r\n'

def mjpeg_generator_cam2():
    """Generate MJPEG stream for camera 2"""
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
        if not new_frame_event_cam2.wait(timeout=2.0):
            continue
            
        with jpeg_lock_cam2:
            buf = latest_jpeg_cam2
            
        if buf is None:
            time.sleep(0.01)
            continue
            
        yield boundary + buf + b'\r\n'

# ---------------- Routes ----------------
@app.route("/")
def index():
    """Main page"""
    return render_template("index.html")

@app.route("/video1")
def video1():
    """Video stream route for camera 1"""
    return Response(
        mjpeg_generator_cam1(), 
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
            'Expires': '0',
            'Access-Control-Allow-Origin': '*'
        }
    )

@app.route("/video2")
def video2():
    """Video stream route for camera 2"""
    return Response(
        mjpeg_generator_cam2(), 
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
    """Enhanced telemetry API with dual camera and ROS data"""
    global latest_bgr_cam1, latest_bgr_cam2, fps_ema_cam1, fps_ema_cam2, CONFIG, ros_data
    
    h, w = CONFIG['height'], CONFIG['width']
    
    with ros_lock:
        current_ros_data = ros_data.copy()
    
    return jsonify({
        "timestamp": time.time(),
        "cameras": {
            "cam1": {
                "fps": round(fps_ema_cam1, 1),
                "resolution": {"width": w, "height": h},
                "status": "active" if latest_bgr_cam1 is not None else "inactive",
                "device": CONFIG['device1']
            },
            "cam2": {
                "fps": round(fps_ema_cam2, 1),
                "resolution": {"width": w, "height": h},
                "status": "active" if latest_bgr_cam2 is not None else "inactive", 
                "device": CONFIG['device2']
            }
        },
        "target_fps": CONFIG['fps'],
        "configured_resolution": {"width": CONFIG['width'], "height": CONFIG['height']},
        "overlays": {"grid": False, "center_dot": True},
        "ros_topics": current_ros_data,
        "system": {
            "uptime": time.time(),
            "ui_theme": "dark",
            "quality": "optimized"
        }
    })

# ---------------- Main ----------------
def main():
    global CONFIG
    
    parser = argparse.ArgumentParser(description="Robot Camera Stream")
    parser.add_argument('--device1', type=int, help='Camera 1 device index')
    parser.add_argument('--device2', type=int, help='Camera 2 device index')
    parser.add_argument('--width1', type=int, help='Camera width1')
    parser.add_argument('--height1', type=int, help='Camera height1')
    parser.add_argument('--width2', type=int, help='Camera width2')
    parser.add_argument('--height2', type=int, help='Camera height2')
    parser.add_argument('--fps', type=int, help='Camera FPS')
    parser.add_argument('--flip', type=int, default=0, help='Flip camera horizontally (0/1)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270], help='Rotate camera')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--quality', type=int, default=75, help='JPEG quality (1-100)')
    parser.add_argument('--show-fps', type=int, default=1, help='Show FPS overlay (0/1)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Server host')

    args, unknown = parser.parse_known_args()

    # Validate required arguments
    if args.device1 is None:
        log.error("--device1 is required")
        return
    if args.device2 is None:
        log.error("--device2 is required") 
        return
    if args.width1 is None:
        log.error("--width1 is required")
        return
    if args.height1 is None:
        log.error("--height1 is required")
    if args.width2 is None:
        log.error("--width2 is required")
        return
    if args.height2 is None:
        log.error("--height2 is required")
        return
    if args.fps is None:
        log.error("--fps is required")
        return

    # Store configuration
    CONFIG.update({
        'device1': args.device1,
        'device2': args.device2,
        'width1': args.width1,
        'height1': args.height1,
        'width2': args.width2,
        'height2': args.height2,
        'fps': args.fps,
        'port': args.port,
        'host': args.host,
        'quality': args.quality,
        'flip': bool(args.flip),
        'rotate': args.rotate,
        'show_fps': bool(args.show_fps)
    })

    log.info("Starting Dual Robot Camera Stream System")
    log.info("=" * 60)
    log.info(f"Server: http://{CONFIG['host']}:{CONFIG['port']}")
    log.info(f"Camera 1: {CONFIG['width']}×{CONFIG['height']}@{CONFIG['fps']}fps (device {CONFIG['device1']})")
    log.info(f"Camera 2: {CONFIG['width']}×{CONFIG['height']}@{CONFIG['fps']}fps (device {CONFIG['device2']})")
    log.info(f"Quality: {CONFIG['quality']}% | Features: Center dot overlay, ROS data")
    log.info(f"Options: flip={CONFIG['flip']}, rotate={CONFIG['rotate']}°")
    log.info("=" * 60)

    # Start ROS data simulator
    ros_thread = threading.Thread(target=ros_data_simulator, daemon=True)
    ros_thread.start()

    # Start camera capture threads
    cam1_thread = threading.Thread(target=capture_loop, args=(1, CONFIG['device1']), daemon=True)
    cam1_thread.start()
    
    cam2_thread = threading.Thread(target=capture_loop, args=(2, CONFIG['device2']), daemon=True)
    cam2_thread.start()

    # Wait for cameras to initialize
    log.info("Initializing dual camera system...")
    time.sleep(2)

    cam1_ready = False
    cam2_ready = False
    
    for i in range(100):
        with jpeg_lock_cam1:
            if latest_jpeg_cam1 is not None:
                cam1_ready = True
        with jpeg_lock_cam2:
            if latest_jpeg_cam2 is not None:
                cam2_ready = True
                
        if cam1_ready and cam2_ready:
            break
        time.sleep(0.1)
        if i % 20 == 0:
            log.info(f"Waiting for cameras... CAM1: {'✓' if cam1_ready else '✗'}, CAM2: {'✓' if cam2_ready else '✗'}")
    
    log.info(f"Camera system status: CAM1: {'Ready' if cam1_ready else 'Failed'}, CAM2: {'Ready' if cam2_ready else 'Failed'}")

    try:
        app.run(host=CONFIG['host'], port=CONFIG['port'], threaded=True, debug=False)
    except KeyboardInterrupt:
        log.info("\nSystem shutdown by user")
    except Exception as e:
        log.error(f"Server error: {e}")

if __name__ == "__main__":
    main()