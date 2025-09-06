#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import argparse
import threading
import time
import logging
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

# ---------------- Logging ----------------
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("lowlag-stream")

# ---------------- Flask ----------------
app = Flask(__name__)

# ---------------- HTML Template (same as before) ----------------
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Camera Stream</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body { 
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #1a1a1a;
            color: #e8e8e8;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            padding: 20px;
            line-height: 1.6;
        }
        
        h1 { 
            margin-bottom: 30px; 
            color: orange;
            text-align: center;
            font-size: 3rem;
            font-weight: 600;
            letter-spacing: -0.025em;
            text-shadow: 1px 1px 6px rgba(0,0,0,0.7);
        }
        
        #video-container { 
            position: relative; 
            display: flex;
            justify-content: center;
            align-items: center;
            margin: 20px 0;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 10px 40px rgba(0,0,0,0.4);
            background: #2a2a2a;
            padding: 8px;
        }
        
        #video-stream { 
            width: {{ width }}px; 
            height: {{ height }}px; 
            border-radius: 8px;
            display: block;
            background: #000;
            border: 1px solid #3a3a3a;
        }
        
        #overlay { 
            position: absolute; 
            top: 18px; 
            left: 18px; 
            color: #10a37f; 
            background: rgba(26, 26, 26, 0.9);
            backdrop-filter: blur(8px);
            padding: 10px 14px; 
            font-family: 'SF Mono', 'Monaco', 'Cascadia Code', 'Roboto Mono', monospace; 
            font-size: 13px;
            border-radius: 8px;
            border: 1px solid rgba(58, 58, 58, 0.6);
            font-weight: 500;
        }
        
        .info-panel {
            background: rgba(42, 42, 42, 0.8);
            backdrop-filter: blur(12px);
            padding: 24px;
            margin: 16px 0;
            border-radius: 12px;
            border: 1px solid rgba(58, 58, 58, 0.3);
            min-width: 420px;
            text-align: left;
            box-shadow: 0 4px 20px rgba(0,0,0,0.15);
        }
        
        .info-panel h2 {
            margin-bottom: 16px;
            color: #f0f0f0;
            font-size: 1rem;
            font-weight: 600;
            letter-spacing: -0.025em;
        }
        
        #timestamp {
            color: #b0b0b0;
            font-size: 14px;
            font-family: 'SF Mono', 'Monaco', 'Cascadia Code', 'Roboto Mono', monospace;
            font-weight: 400;
        }
        
        .status-indicator {
            display: inline-block;
            width: 8px;
            height: 8px;
            border-radius: 50%;
            margin-right: 10px;
            animation: pulse 2s infinite;
        }
        
        .status-connected {
            background-color: #10a37f;
            box-shadow: 0 0 8px rgba(16, 163, 127, 0.5);
        }
        
        .status-disconnected {
            background-color: #ff4444;
            box-shadow: 0 0 8px rgba(255, 68, 68, 0.5);
        }
        
        .status-loading {
            background-color: #ffa500;
            box-shadow: 0 0 8px rgba(255, 165, 0, 0.5);
        }
        
        @keyframes pulse {
            0% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.7; transform: scale(1.1); }
            100% { opacity: 1; transform: scale(1); }
        }
        
        .info-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 8px 0;
            border-bottom: 1px solid rgba(58, 58, 58, 0.2);
        }
        
        .info-row:last-child {
            border-bottom: none;
        }
        
        .info-label {
            color: #b0b0b0;
            font-size: 14px;
            font-weight: 400;
        }
        
        .info-value {
            color: #e8e8e8;
            font-size: 14px;
            font-weight: 500;
            font-family: 'SF Mono', 'Monaco', 'Cascadia Code', 'Roboto Mono', monospace;
        }
        
        @media (max-width: 900px) {
            #video-stream {
                width: 90vw;
                max-width: {{ width }}px;
                height: auto;
                aspect-ratio: {{ width }}/{{ height }};
            }
            
            .info-panel {
                min-width: 90vw;
                max-width: 500px;
            }
            
            h1 {
                font-size: 1.5rem;
            }
        }
        
        @media (max-width: 600px) {
            body {
                padding: 16px;
            }
            
            #overlay {
                font-size: 12px;
                padding: 8px 12px;
            }
            
            h1 {
                font-size: 1.3rem;
            }
            
            .info-panel {
                padding: 20px;
            }
        }
    </style>
</head>
<body>
    <h1>Robot Camera Stream</h1>
    
    <div id="video-container">
        <img id="video-stream" src="/video" alt="Camera Stream">
        <div id="overlay">
            <span class="status-indicator status-connected" id="status-dot"></span>
            FPS: <span id="fps">0</span> 
        </div>
    </div>
    
    <div class="info-panel">
        <h2>System Information</h2>
        <div class="info-row">
            <span class="info-label">Timestamp:</span>
            <span class="info-value" id="timestamp">Loading...</span>
        </div>
        <div class="info-row">
            <span class="info-label">Stream Status:</span>
            <span class="info-value" id="stream-status">Connecting...</span>
        </div>
        <div class="info-row">
            <span class="info-label">Resolution:</span>
            <span class="info-value">{{ width }}×{{ height }} @ {{ fps }}fps</span>
        </div>
    </div>

    <script>
        const $ = (sel) => document.querySelector(sel);
        
        let streamConnected = false;
        let telemetryErrors = 0;
        
        function updateTimestamp() {
            try {
                const now = new Date();
                const options = {
                    year: 'numeric',
                    month: '2-digit',
                    day: '2-digit',
                    hour: '2-digit',
                    minute: '2-digit',
                    second: '2-digit',
                    hour12: false
                };
                $("#timestamp").innerText = now.toLocaleString('en-GB', options);
            } catch (e) {
                $("#timestamp").innerText = new Date().toLocaleString();
            }
        }
        
        setInterval(updateTimestamp, 1000);
        updateTimestamp();
        
        function updateStatusIndicator(connected) {
            const dot = $("#status-dot");
            const status = $("#stream-status");
            
            if (connected) {
                dot.className = "status-indicator status-connected";
                status.innerText = "Connected";
            } else {
                dot.className = "status-indicator status-disconnected";
                status.innerText = "Disconnected";
            }
        }
        
        async function fetchTelemetry() {
            try {
                const controller = new AbortController();
                const timeoutId = setTimeout(() => controller.abort(), 5000);
                
                const r = await fetch("/api/telemetry", {
                    signal: controller.signal
                });
                
                clearTimeout(timeoutId);
                
                if (!r.ok) throw new Error(`HTTP ${r.status}`);
                
                const data = await r.json();
                
                $("#fps").textContent = data.fps?.toFixed(1) || "0";
                
                telemetryErrors = 0;
                
                if (!streamConnected) {
                    streamConnected = true;
                    updateStatusIndicator(true);
                }
                
            } catch (e) {
                telemetryErrors++;
                
                if (streamConnected) {
                    streamConnected = false;
                    updateStatusIndicator(false);
                }
                
                if (telemetryErrors > 5) {
                    await new Promise(resolve => setTimeout(resolve, 2000));
                }
            }
        }
        
        const telemetryInterval = setInterval(fetchTelemetry, 500);
        fetchTelemetry();
        
        const videoElement = $("#video-stream");
        
        videoElement.addEventListener("load", function() {
            console.log("Video stream loaded successfully");
            streamConnected = true;
            updateStatusIndicator(true);
        });
        
        videoElement.addEventListener("error", function() {
            console.log("Video stream error, attempting to reload...");
            streamConnected = false;
            updateStatusIndicator(false);
            
            setTimeout(() => {
                const timestamp = new Date().getTime();
                this.src = "/video?" + timestamp;
            }, 2000);
        });
        
        videoElement.addEventListener("abort", function() {
            console.log("Video stream aborted");
            streamConnected = false;
            updateStatusIndicator(false);
        });
        
        setInterval(() => {
            if (!streamConnected && telemetryErrors > 3) {
                console.log("Attempting to reconnect video stream...");
                const timestamp = new Date().getTime();
                videoElement.src = "/video?" + timestamp;
            }
        }, 10000);
        
        document.addEventListener("visibilitychange", function() {
            if (document.hidden) {
                console.log("Page hidden, reducing update frequency");
                clearInterval(telemetryInterval);
            } else {
                console.log("Page visible, resuming normal updates");
                fetchTelemetry();
                setInterval(fetchTelemetry, 500);
            }
        });
        
        console.log("Robot Camera Stream Interface Loaded");
        
        $("#status-dot").className = "status-indicator status-loading";
        $("#stream-status").innerText = "Connecting...";
        
        let frameCount = 0;
        setInterval(() => {
            frameCount++;
            if (frameCount % 120 === 0) {
                console.log(`Performance: ${frameCount} telemetry updates, Stream: ${streamConnected ? 'Connected' : 'Disconnected'}`);
            }
        }, 500);
    </script>
</body>
</html>
"""

# ---------------- Globals ----------------
latest_jpeg = None
jpeg_lock = threading.Lock()
new_frame_event = threading.Event()

latest_bgr = None
bgr_lock = threading.Lock()

show_center_dot = True  
center_dot_color = (0, 255, 0)  # Green center dot

fps_ema = 0.0
last_tick = time.monotonic()

# Global variables to store configuration from command line
CONFIG = {
    'device': 0,
    'width': 640,  # Lower default resolution for better performance
    'height': 480,
    'fps': 30,
    'port': 5000,
    'host': '0.0.0.0',
    'quality': 85,  # Slightly lower quality for speed
    'flip': True,
    'rotate': 0,
    'show_fps': True
}

# ---------------- Try TurboJPEG ----------------
try:
    from turbojpeg import TurboJPEG, TJPF_BGR, TJSAMP_420
    _jpeg = TurboJPEG()
    def encode_jpeg(img_bgr, quality=85):
        return _jpeg.encode(img_bgr, quality=quality, pixel_format=TJPF_BGR, jpeg_subsample=TJSAMP_420)
    log.info("TurboJPEG enabled for optimal performance")
except Exception:
    _jpeg = None
    def encode_jpeg(img_bgr, quality=85):
        # Optimized CV2 encoding parameters
        params = [
            int(cv2.IMWRITE_JPEG_QUALITY), int(quality),
            int(cv2.IMWRITE_JPEG_OPTIMIZE), 1,
            int(cv2.IMWRITE_JPEG_PROGRESSIVE), 1
        ]
        ok, buf = cv2.imencode(".jpg", img_bgr, params)
        if not ok:
            raise RuntimeError("cv2.imencode failed")
        return buf.tobytes()
    log.info("Using optimized cv2.imencode")

# ---------------- Overlay helpers ----------------
def draw_overlay_inplace(frame_bgr, show_fps=True):
    """Minimal overlay drawing for maximum performance"""
    global fps_ema
    
    if show_center_dot:
        h, w = frame_bgr.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.circle(frame_bgr, (cx, cy), 3, center_dot_color, -1)
    
    if show_fps and fps_ema > 0:
        text = f"FPS: {fps_ema:.1f}"
        cv2.putText(frame_bgr, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# ---------------- Optimized Capture Thread ----------------
def capture_loop():
    """Highly optimized camera capture loop"""
    global latest_jpeg, latest_bgr, fps_ema, last_tick, CONFIG
    
    # Get configuration values
    device = CONFIG['device']
    width = CONFIG['width']
    height = CONFIG['height']
    fps = CONFIG['fps']
    flip = CONFIG['flip']
    rotate = CONFIG['rotate']
    show_fps = CONFIG['show_fps']
    jpeg_quality = CONFIG['quality']
    
    # Set OpenCV threading for optimal performance
    cv2.setUseOptimized(True)
    cv2.setNumThreads(2)  # Use 2 threads for better performance
    
    log.info(f"Attempting to open camera device {device}")
    
    # Try opening camera with optimal backend
    cap = None
    for backend_name, backend in [("V4L2", cv2.CAP_V4L2), ("GSTREAMER", cv2.CAP_GSTREAMER), ("ANY", cv2.CAP_ANY)]:
        try:
            cap = cv2.VideoCapture(device, backend)
            if cap.isOpened():
                log.info(f"Camera opened successfully with {backend_name} backend")
                break
            else:
                cap.release()
        except Exception as e:
            log.warning(f"Failed to open camera with {backend_name}: {e}")
    
    if cap is None or not cap.isOpened():
        log.error(f"Cannot open camera device {device}")
        return
    
    # Force MJPEG format for maximum performance (from your v4l2 output)
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    
    # Set optimal camera properties
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimal buffer for low latency
    
    # Performance optimizations
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # Don't convert to RGB
    
    # Disable auto-adjustments for consistent performance
    try:
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual exposure
    except:
        log.warning("Could not set manual focus/exposure")
    
    # Verify actual settings
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    
    # Convert fourcc back to string
    fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
    
    log.info(f"Camera configured: {actual_w}x{actual_h}@{actual_fps:.1f}fps, format: {fourcc_str}")
    
    # Performance tracking
    frame_count = 0
    last_fps_check = time.monotonic()
    target_frame_time = 1.0 / fps
    
    # Main capture loop
    while True:
        loop_start = time.monotonic()
        
        # Read frame
        ret, frame = cap.read()
        if not ret:
            log.warning("Failed to read frame, attempting to recover...")
            cap.release()
            time.sleep(0.1)
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not cap.isOpened():
                log.error("Failed to reopen camera")
                break
            continue
        
        frame_count += 1
        
        # Apply transformations only if needed
        if flip:
            frame = cv2.flip(frame, 1)
        
        if rotate != 0:
            if rotate == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif rotate == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif rotate == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        
        # Resize if needed (should not be needed with proper camera config)
        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
        
        # Add minimal overlay
        draw_overlay_inplace(frame, show_fps=show_fps)
        
        # Calculate FPS more efficiently
        now = time.monotonic()
        dt = now - last_tick
        if dt > 0:
            inst_fps = 1.0 / dt
            # Faster EMA update
            alpha = 0.2
            fps_ema = inst_fps if fps_ema == 0 else (alpha * inst_fps + (1-alpha) * fps_ema)
        last_tick = now
        
        # Store frame for API access (non-blocking)
        try:
            if bgr_lock.acquire(blocking=False):
                latest_bgr = frame.copy()
                bgr_lock.release()
        except:
            pass
        
        # Encode JPEG
        try:
            jpeg_bytes = encode_jpeg(frame, quality=jpeg_quality)
        except Exception as e:
            log.error(f"JPEG encode failed: {e}")
            continue
        
        # Store JPEG (non-blocking)
        try:
            if jpeg_lock.acquire(blocking=False):
                latest_jpeg = jpeg_bytes
                new_frame_event.set()
                new_frame_event.clear()
                jpeg_lock.release()
        except:
            pass
        
        # Performance monitoring
        if frame_count % 60 == 0:  # Every 60 frames
            elapsed = now - last_fps_check
            measured_fps = 60.0 / elapsed if elapsed > 0 else 0
            log.info(f"Performance: {measured_fps:.1f} FPS (target: {fps}, measured FPS: {fps_ema:.1f})")
            last_fps_check = now
        
        # Minimal frame rate control
        elapsed = time.monotonic() - loop_start
        if elapsed < target_frame_time:
            sleep_time = target_frame_time - elapsed
            if sleep_time > 0.001:  # Only sleep if significant time left
                time.sleep(sleep_time)
    
    cap.release()

# ---------------- Optimized MJPEG stream ----------------
def mjpeg_generator():
    """Optimized MJPEG stream generator"""
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
    
    while True:
        # Wait for new frame
        new_frame_event.wait(timeout=1.0)
        
        with jpeg_lock:
            buf = latest_jpeg
            
        if buf is None:
            time.sleep(0.001)
            continue
        
        # Optimized response format
        yield boundary + str(len(buf)).encode() + b'\r\n\r\n' + buf + b'\r\n'

# ---------------- Routes ----------------
@app.route("/")
def index():
    """Main page with dynamic configuration"""
    global CONFIG
    template = HTML_TEMPLATE.replace("{{ width }}", str(CONFIG['width']))
    template = template.replace("{{ height }}", str(CONFIG['height']))
    template = template.replace("{{ fps }}", str(CONFIG['fps']))
    return render_template_string(template)

@app.route("/video")
def video():
    """Optimized video stream route"""
    return Response(
        mjpeg_generator(), 
        mimetype='multipart/x-mixed-replace; boundary=frame',
        headers={
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
            'Expires': '0',
            'Access-Control-Allow-Origin': '*',
            'Connection': 'close'
        }
    )

@app.route("/api/telemetry")
def api_telemetry():
    """Fast telemetry API"""
    global latest_bgr, fps_ema, CONFIG
    
    h, w = CONFIG['height'], CONFIG['width']
    
    # Fast response without heavy processing
    return jsonify({
        "timestamp": time.time(),
        "fps": round(fps_ema, 1),
        "resolution": {"width": w, "height": h},
        "target_fps": CONFIG['fps'],
        "device": CONFIG['device'],
        "camera_status": "active" if latest_bgr is not None else "inactive",
        "performance": "optimized"
    })

# ---------------- Main ----------------
def main():
    global CONFIG
    
    parser = argparse.ArgumentParser(description="Optimized Robot Camera Stream")
    parser.add_argument('--device', type=int, default=0, help='Camera device index')
    parser.add_argument('--width', type=int, default=640, help='Camera width')
    parser.add_argument('--height', type=int, default=480, help='Camera height')
    parser.add_argument('--fps', type=int, default=30, help='Camera FPS')
    parser.add_argument('--flip', type=int, default=1, help='Flip camera horizontally (0/1)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270], help='Rotate camera')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--quality', type=int, default=85, help='JPEG quality (1-100)')
    parser.add_argument('--show-fps', type=int, default=1, help='Show FPS overlay (0/1)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Server host')
    args = parser.parse_args()

    # Update configuration
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

    log.info("Starting OPTIMIZED Robot Camera Stream System")
    log.info("=" * 50)
    log.info(f"Server: http://{CONFIG['host']}:{CONFIG['port']}")
    log.info(f"Camera: {CONFIG['width']}×{CONFIG['height']}@{CONFIG['fps']}fps (device {CONFIG['device']})")
    log.info(f"Quality: {CONFIG['quality']}% | Optimizations: MJPEG direct, minimal overlay")
    log.info(f"Options: flip={CONFIG['flip']}, rotate={CONFIG['rotate']}°")
    log.info("=" * 50)

    # Start optimized camera capture thread
    th_cap = threading.Thread(target=capture_loop, daemon=True)
    th_cap.start()

    # Wait for camera initialization
    log.info("Initializing optimized camera system...")
    for i in range(100):  # 5 second timeout
        with jpeg_lock:
            if latest_jpeg is not None:
                log.info("Camera system ready with optimizations!")
                break
        time.sleep(0.05)
        if i % 20 == 0:
            log.info(f"Waiting for camera... ({i//20 + 1}/5)")
    
    try:
        # Use threaded Flask for better performance
        app.run(host=CONFIG['host'], port=CONFIG['port'], threaded=True, debug=False)
    except KeyboardInterrupt:
        log.info("\nSystem shutdown by user")
    except Exception as e:
        log.error(f"Server error: {e}")

if __name__ == "__main__":
    main()