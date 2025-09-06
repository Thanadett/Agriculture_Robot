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

# ---------------- HTML Template ----------------
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
        
        #telemetry { 
            background: rgba(16, 16, 16, 0.9);
            color: #10a37f;
            padding: 16px; 
            font-family: 'SF Mono', 'Monaco', 'Cascadia Code', 'Roboto Mono', monospace; 
            max-height: 300px; 
            overflow-y: auto; 
            border-radius: 8px;
            font-size: 12px;
            line-height: 1.5;
            white-space: pre-wrap;
            border: 1px solid rgba(58, 58, 58, 0.3);
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
        
        /* Scrollbar styling */
        #telemetry::-webkit-scrollbar {
            width: 6px;
        }
        
        #telemetry::-webkit-scrollbar-track {
            background: rgba(26, 26, 26, 0.5);
            border-radius: 3px;
        }
        
        #telemetry::-webkit-scrollbar-thumb {
            background: rgba(16, 163, 127, 0.6);
            border-radius: 3px;
        }
        
        #telemetry::-webkit-scrollbar-thumb:hover {
            background: rgba(16, 163, 127, 0.8);
        }
        
        /* Firefox scrollbar */
        #telemetry {
            scrollbar-width: thin;
            scrollbar-color: rgba(16, 163, 127, 0.6) rgba(26, 26, 26, 0.5);
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
        // Utility functions
        const $ = (sel) => document.querySelector(sel);
        
        // Global state
        let streamConnected = false;
        let telemetryErrors = 0;
        
        // Update timestamp every second
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
        
        // Start timestamp updates
        setInterval(updateTimestamp, 1000);
        updateTimestamp();
        
        // Update status indicator
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
        
        // Telemetry fetching
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
                
                // Update overlay data
                $("#fps").textContent = data.fps?.toFixed(1) || "0";
                $("#cx").textContent = data.center?.x || "0";
                $("#cy").textContent = data.center?.y || "0";
                
                // Update telemetry display
                $("#telemetry").textContent = JSON.stringify(data, null, 2);
                
                // Reset error counter on success
                telemetryErrors = 0;
                
                // Update connection status
                if (!streamConnected) {
                    streamConnected = true;
                    updateStatusIndicator(true);
                }
                
            } catch (e) {
                telemetryErrors++;
                
                let errorMsg = `Telemetry error (${telemetryErrors}): `;
                
                if (e.name === 'AbortError') {
                    errorMsg += "Request timeout - server may be busy";
                } else {
                    errorMsg += e.message;
                }
                
                $("#telemetry").textContent = errorMsg;
                
                // Update connection status
                if (streamConnected) {
                    streamConnected = false;
                    updateStatusIndicator(false);
                }
                
                // If too many errors, slow down polling
                if (telemetryErrors > 5) {
                    await new Promise(resolve => setTimeout(resolve, 2000));
                }
            }
        }
        
        // Start telemetry polling
        const telemetryInterval = setInterval(fetchTelemetry, 500);
        fetchTelemetry();
        
        // Video stream handling
        const videoElement = $("#video-stream");
        const streamStatusEl = $("#stream-status");
        
        videoElement.addEventListener("load", function() {
            console.log("Video stream loaded successfully");
            streamConnected = true;
            updateStatusIndicator(true);
        });
        
        videoElement.addEventListener("error", function() {
            console.log("Video stream error, attempting to reload...");
            streamConnected = false;
            updateStatusIndicator(false);
            
            // Wait a bit before retrying
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
        
        // Check stream health periodically
        setInterval(() => {
            if (!streamConnected && telemetryErrors > 3) {
                console.log("Attempting to reconnect video stream...");
                const timestamp = new Date().getTime();
                videoElement.src = "/video?" + timestamp;
            }
        }, 10000);
        
        // Page visibility handling
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
        
        // Initialize
        console.log("Robot Camera Stream Interface Loaded");
        console.log("Clean dark mode UI - Dynamic resolution support");
        
        // Set initial status
        $("#status-dot").className = "status-indicator status-loading";
        $("#stream-status").innerText = "Connecting...";
        
        // Performance monitoring
        let frameCount = 0;
        setInterval(() => {
            frameCount++;
            if (frameCount % 120 === 0) { // Every 120 intervals (60 seconds)
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
    'quality': 92,
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
    """Main page with dynamic configuration"""
    global CONFIG
    # Inject camera configuration into template
    template = HTML_TEMPLATE.replace("{{ width }}", str(CONFIG['width']))
    template = template.replace("{{ height }}", str(CONFIG['height']))
    template = template.replace("{{ fps }}", str(CONFIG['fps']))
    return render_template_string(template)

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
    parser.add_argument('--device', type=int, help='Camera device index (required from launch file)')
    parser.add_argument('--width', type=int, help='Camera width (required from launch file)')
    parser.add_argument('--height', type=int, help='Camera height (required from launch file)')
    parser.add_argument('--fps', type=int, help='Camera FPS (required from launch file)')
    parser.add_argument('--flip', type=int, default=1, help='Flip camera horizontally (0/1)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270], help='Rotate camera')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--quality', type=int, default=92, help='JPEG quality (1-100)')
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
    log.info(f"Theme: Claude.ai Dark Mode")
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