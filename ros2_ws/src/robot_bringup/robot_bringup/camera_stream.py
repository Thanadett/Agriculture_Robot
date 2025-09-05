#!/usr/bin/env python3
import argparse
import threading
import time
import cv2
import numpy as np
from flask import Flask, Response
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Global variables
latest_frame = None
frame_lock = threading.Lock()
fps_counter = 0
fps_display = 0
fps_lock = threading.Lock()
last_fps_time = time.time()

# Targeting system variables
show_grid = True
show_center_dot = True
grid_color = (100, 100, 100)  # Gray
center_dot_color = (0, 0, 255)  # Red

def fps_calculator():
    """Calculate and update FPS display"""
    global fps_counter, fps_display, last_fps_time
    
    while True:
        time.sleep(1.0)
        with fps_lock:
            current_time = time.time()
            time_diff = current_time - last_fps_time
            fps_display = fps_counter / time_diff if time_diff > 0 else 0
            fps_counter = 0
            last_fps_time = current_time

def draw_targeting_overlay(frame):
    """Draw targeting overlay on frame"""
    global show_grid, show_center_dot
    
    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2
    
    if show_grid:
        for i in range(1, 3):
            x = width * i // 3
            cv2.line(frame, (x, 0), (x, height), grid_color, 1)
        for i in range(1, 3):
            y = height * i // 3
            cv2.line(frame, (0, y), (width, y), grid_color, 1)
    
    if show_center_dot:
        cv2.circle(frame, (center_x, center_y), 4, center_dot_color, -1, cv2.LINE_AA)
    
    return frame

def capture_loop(device=0, width=800, height=600, fps=30, flip=True, rotate=0, show_fps=True):
    global latest_frame, fps_counter
    cap = None
    for backend in [cv2.CAP_V4L2, cv2.CAP_ANY]:
        try:
            cap = cv2.VideoCapture(device, backend)
            if cap.isOpened():
                logger.info(f"Camera opened successfully with backend: {backend}")
                break
        except Exception as e:
            logger.warning(f"Failed to open camera with backend {backend}: {e}")
    
    if cap is None or not cap.isOpened():
        logger.error(f"Cannot open camera device {device}")
        return
    
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        logger.info(f"Camera configured - Requested: {width}x{height}@{fps}fps")
        logger.info(f"Camera actual: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}@{cap.get(cv2.CAP_PROP_FPS):.1f}fps")
    except Exception as e:
        logger.warning(f"Some camera properties could not be set: {e}")
    
    try:
        camera_fps = cap.get(cv2.CAP_PROP_FPS)
        target_frame_time = 1.0 / min(camera_fps, fps) if camera_fps > 0 else 1.0 / fps
    except:
        target_frame_time = 1.0 / fps
    
    last_frame_time = time.time()
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    font_thickness = 2
    retry_count = 0
    max_retries = 5
    
    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                retry_count += 1
                logger.warning(f"Failed to read frame, retry {retry_count}/{max_retries}")
                if retry_count >= max_retries:
                    logger.error("Too many failed frame reads, restarting capture...")
                    cap.release()
                    time.sleep(1)
                    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                    retry_count = 0
                time.sleep(0.1)
                continue
            retry_count = 0
            
            if flip:
                frame = cv2.flip(frame, 1)
            if rotate == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif rotate == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif rotate == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            frame = draw_targeting_overlay(frame)
            
            if show_fps:
                with fps_lock:
                    fps_text = f"FPS: {fps_display:.1f}"
                text_size = cv2.getTextSize(fps_text, font, font_scale, font_thickness)[0]
                cv2.rectangle(frame, (10,10), (text_size[0]+20, text_size[1]+20), (0,0,0), -1)
                cv2.putText(frame, fps_text, (15,30), font, font_scale, (0,255,0), font_thickness)
            
            height, width = frame.shape[:2]
            coord_text = f"Center: ({width//2},{height//2})"
            coord_size = cv2.getTextSize(coord_text, font, 0.4, 1)[0]
            cv2.rectangle(frame, (width-coord_size[0]-20,10), (width-5,coord_size[1]+20), (0,0,0), -1)
            cv2.putText(frame, coord_text, (width-coord_size[0]-15,25), font, 0.4, (255,255,255), 1)
            
            with frame_lock:
                latest_frame = frame.copy()
            
            with fps_lock:
                fps_counter += 1
            
            elapsed = time.time() - last_frame_time
            sleep_time = target_frame_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_frame_time = time.time()
            
        except Exception as e:
            logger.error(f"Error in capture loop: {e}")
            time.sleep(0.1)
    cap.release()

def mjpeg_stream(jpeg_quality=85):
    global latest_frame
    while True:
        try:
            with frame_lock:
                if latest_frame is None:
                    time.sleep(0.01)
                    continue
                frame = latest_frame.copy()
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality, int(cv2.IMWRITE_JPEG_OPTIMIZE), 1]
            ret, buffer = cv2.imencode('.jpg', frame, encode_params)
            if not ret:
                logger.warning("Failed to encode frame")
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        except Exception as e:
            logger.error(f"Error in MJPEG stream: {e}")
            time.sleep(0.1)

@app.route('/video')
def video():
    return Response(mjpeg_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/toggle/<overlay_type>')
def toggle_overlay(overlay_type):
    global show_grid, show_center_dot
    if overlay_type == 'grid':
        show_grid = not show_grid
        status = "enabled" if show_grid else "disabled"
        return f"Grid {status}"
    elif overlay_type == 'center':
        show_center_dot = not show_center_dot
        status = "enabled" if show_center_dot else "disabled"
        return f"Center dot {status}"
    return "Invalid overlay type"

@app.route('/')
def index():
    return '''
<!DOCTYPE html>
<html>
<head>
<title> STREAM ACTIVE </title>
<style>
html,body{height:100%;margin:0;font-family:'Segoe UI',sans-serif;background:#d3d3d3;color:#121212;display:flex;flex-direction:column;align-items:center;}
h1{color:#007f00;text-shadow:0 0 8px #007f00;margin:15px 0;}
.camera-frame{border:2px solid #444;border-radius:10px;max-width:90vw;max-height:65vh;box-shadow:0 0 15px rgba(0,0,0,0.3);}
.controls{margin-top:15px;padding:10px 15px;background:rgba(255,255,255,0.8);border-radius:8px;border:1px solid #888;}
button{background:#eee;color:#121212;border:1px solid #888;padding:6px 12px;margin:0 4px;border-radius:4px;cursor:pointer;transition:0.2s;}
button:hover{background:#bbb;color:#000;}
.info{margin-top:12px;padding:8px 15px;background:rgba(255,255,255,0.7);border-radius:6px;border:1px solid #aaa;font-size:0.9em;color:#121212;}
.status-bar{position:fixed;bottom:10px;right:10px;background:rgba(200,200,200,0.8);padding:6px 12px;border-radius:4px;border:1px solid #888;color:#121212;font-size:0.85em;display:flex;gap:15px;}
</style>
<script>
function toggleOverlay(t){fetch('/toggle/'+t).then(r=>r.text()).then(d=>{document.getElementById('status').innerText=d});}
setInterval(()=>{document.getElementById('timestamp').innerText=new Date().toLocaleTimeString();},1000);
</script>
</head>
<body>
<h1> STREAM ACTIVE </h1>
<img src="/video" class="camera-frame">
<div class="controls">
<button onclick="toggleOverlay('grid')">Toggle Grid</button>
<button onclick="toggleOverlay('center')">Toggle Center Dot</button>
</div>
<div class="info">
<strong>Status:</strong> <span id="status">All systems operational</span>
</div>
<div class="status-bar">
<span><span id="timestamp"></span></span>
<span>Stream: LIVE</span>
</div>
</body>
</html>
'''

def main():
    parser = argparse.ArgumentParser(description='Camera Stream with Targeting System')
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--width', type=int, default=800)
    parser.add_argument('--height', type=int, default=600)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--flip', type=int, default=1)
    parser.add_argument('--rotate', type=int, default=0, choices=[0,90,180,270])
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--quality', type=int, default=85)
    parser.add_argument('--show-fps', type=int, default=1)
    
    args = parser.parse_args()
    
    logger.info(" Starting Camera System")
    logger.info(f" Server port: {args.port}")
    logger.info(f" Camera settings: {args.width}x{args.height} @ {args.fps}fps")
    
    fps_thread = threading.Thread(target=fps_calculator, daemon=True)
    fps_thread.start()
    
    capture_thread = threading.Thread(
        target=capture_loop,
        kwargs={'device': args.device, 'width': args.width, 'height': args.height,
                'fps': args.fps, 'flip': bool(args.flip), 'rotate': args.rotate,
                'show_fps': bool(args.show_fps)},
        daemon=True
    )
    capture_thread.start()
    
    time.sleep(2)
    logger.info("camera system ready!")
    
    try:
        app.run(host='0.0.0.0', port=args.port, threaded=True, debug=False)
    except KeyboardInterrupt:
        logger.info(" System shutdown by user")
    except Exception as e:
        logger.error(f" Server error: {e}")

if __name__ == "__main__":
    main()
