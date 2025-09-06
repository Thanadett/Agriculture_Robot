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
            font-family: Arial, sans-serif; 
            background: #2c2c2c; 
            color: #ffffff;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }
        
        h1 { 
            margin-bottom: 20px; 
            color: #ffffff;
            text-align: center;
            font-size: 2rem;
            text-shadow: 0 2px 4px rgba(0,0,0,0.5);
        }
        
        #video-container { 
            position: relative; 
            display: flex;
            justify-content: center;
            align-items: center;
            margin: 20px 0;
            border-radius: 10px;
            overflow: hidden;
            box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        }
        
        #video-stream { 
            width: 800px; 
            height: 600px; 
            border: 3px solid #444; 
            display: block;
            background: #000;
            border-radius: 8px;
        }
        
        #overlay { 
            position: absolute; 
            top: 15px; 
            left: 15px; 
            color: #00ff00; 
            background: rgba(0,0,0,0.8); 
            padding: 8px 12px; 
            font-family: 'Courier New', monospace; 
            font-size: 16px;
            border-radius: 6px;
            border: 1px solid #00ff00;
            text-shadow: 0 0 5px #00ff00;
        }
        
        .info-panel {
            background: rgba(255,255,255,0.1);
            backdrop-filter: blur(10px);
            padding: 20px;
            margin: 15px 0;
            border-radius: 10px;
            border: 1px solid rgba(255,255,255,0.2);
            min-width: 400px;
            text-align: center;
        }
        
        .info-panel h2 {
            margin-bottom: 15px;
            color: #00ff00;
            font-size: 1.2rem;
        }
        
        #telemetry { 
            background: rgba(0,0,0,0.8); 
            color: #00ff00; 
            padding: 15px; 
            font-family: 'Courier New', monospace; 
            max-height: 250px; 
            overflow-y: auto; 
            border-radius: 8px;
            font-size: 12px;
            line-height: 1.5;
            white-space: pre-wrap;
            border: 1px solid #00ff00;
            text-shadow: 0 0 3px #00ff00;
        }
        
        #timestamp {
            color: #cccccc;
            font-size: 16px;
            font-family: 'Courier New', monospace;
        }
        
        .status-indicator {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 8px;
            animation: pulse 2s infinite;
        }
        
        .status-connected {
            background-color: #00ff00;
            box-shadow: 0 0 10px #00ff00;
        }
        
        .status-disconnected {
            background-color: #ff0000;
            box-shadow: 0 0 10px #ff0000;
        }
        
        .status-loading {
            background-color: #ffff00;
            box-shadow: 0 0 10px #ffff00;
        }
        
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
        
        @media (max-width: 900px) {
            #video-stream {
                width: 90vw;
                max-width: 800px;
                height: auto;
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
                padding: 10px;
            }
            
            #overlay {
                font-size: 14px;
                padding: 6px 10px;
            }
            
            h1 {
                font-size: 1.2rem;
            }
        }
        
        /* Scrollbar styling for webkit browsers */
        #telemetry::-webkit-scrollbar {
            width: 6px;
        }
        
        #telemetry::-webkit-scrollbar-track {
            background: rgba(0,0,0,0.3);
            border-radius: 3px;
        }
        
        #telemetry::-webkit-scrollbar-thumb {
            background: #00ff00;
            border-radius: 3px;
        }
        
        #telemetry::-webkit-scrollbar-thumb:hover {
            background: #00cc00;
        }
    </style>
</head>
<body>
    <h1> Robot Camera Stream</h1>
    
    <div id="video-container">
        <img id="video-stream" src="/video" alt="Camera Stream">
        <div id="overlay">
            <span class="status-indicator status-connected" id="status-dot"></span>
            FPS: <span id="fps">0</span> | 
            Center: (<span id="cx">0</span>, <span id="cy">0</span>)
        </div>
    </div>
    
    <div class="info-panel">
        <h2> System Information</h2>
        <div> Timestamp: <span id="timestamp">Loading...</span></div>
        <div> Stream Status: <span id="stream-status">Connecting...</span></div>
    </div>
    
    <div class="info-panel">
        <h2> Live Telemetry</h2>
        <pre id="telemetry">Loading telemetry data...</pre>
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
        console.log("Minimal UI - Center dot overlay enabled by default");
        
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

# Fixed settings - only center dot, no grid, no toggles
show_grid = False  # Always disabled
show_center_dot = True  # Always enabled
grid_color = (100, 100, 100)    # BGR (not used)
center_dot_color = (0, 0, 255)  # BGR - red center dot

fps_ema = 0.0
last_tick = time.monotonic()

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
    global fps_ema
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2
    
    # Only center dot - no grid
    if show_center_dot:
        # Draw a larger, more visible center dot
        cv2.circle(frame_bgr, (cx, cy), 6, center_dot_color, -1, cv2.LINE_AA)
        cv2.circle(frame_bgr, (cx, cy), 8, (255, 255, 255), 2, cv2.LINE_AA)  # White outline
    
    # fps overlay
    if show_fps:
        text = f"FPS: {fps_ema:.1f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.7
        thick = 2
        size = cv2.getTextSize(text, font, scale, thick)[0]
        # Black background with green text
        cv2.rectangle(frame_bgr, (10, 10), (10 + size[0] + 15, 10 + size[1] + 15), (0, 0, 0), -1)
        cv2.rectangle(frame_bgr, (10, 10), (10 + size[0] + 15, 10 + size[1] + 15), (0, 255, 0), 1)
        cv2.putText(frame_bgr, text, (17, 10 + size[1] + 5), font, scale, (0, 255, 0), thick, cv2.LINE_AA)
    
    # coordinates display
    coord_text = f"Center: ({cx},{cy})"
    csize = cv2.getTextSize(coord_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.rectangle(frame_bgr, (w - csize[0] - 20, 10), (w - 5, 10 + csize[1] + 12), (0, 0, 0), -1)
    cv2.rectangle(frame_bgr, (w - csize[0] - 20, 10), (w - 5, 10 + csize[1] + 12), (255, 255, 255), 1)
    cv2.putText(frame_bgr, coord_text, (w - csize[0] - 15, 10 + csize[1] + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

# ---------------- Capture Thread ----------------
def capture_loop(device=0, width=800, height=600, fps=30, flip=True, rotate=0, show_fps=True, jpeg_quality=90):
    global latest_jpeg, latest_bgr, fps_ema, last_tick
    
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
        # Create dummy frame for testing
        dummy_frame = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(dummy_frame, "NO CAMERA DETECTED", (width//4, height//2), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        cv2.putText(dummy_frame, f"Device: {device}", (width//4, height//2 + 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
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
        # Set camera properties
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Additional settings for better quality and lower latency
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
    log.info(f"Camera settings: {width}x{height}@{fps} | actual: {actual_w}x{actual_h}@{actual_fps:.1f}")

    target_frame_time = 1.0 / max(1.0, min(actual_fps, float(fps)))
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
        
        # Apply transformations
        if flip:
            frame = cv2.flip(frame, 1)
        if rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Draw overlays (only center dot)
        draw_overlay_inplace(frame, show_fps=show_fps)

        # Calculate FPS
        now = time.monotonic()
        dt = now - last_tick
        if dt > 0:
            inst = 1.0 / dt
            fps_ema = inst if fps_ema == 0 else (0.15 * inst + 0.85 * fps_ema)
        last_tick = now

        # Store latest BGR frame
        with bgr_lock:
            latest_bgr = frame.copy()

        # Encode to JPEG
        try:
            jpeg_bytes = encode_jpeg(frame, quality=jpeg_quality)
        except Exception as e:
            log.error(f"JPEG encode failed: {e}")
            time.sleep(0.01)
            continue

        # Store latest JPEG
        with jpeg_lock:
            latest_jpeg = jpeg_bytes
            new_frame_event.set()
            new_frame_event.clear()

        # Frame rate limiting
        elapsed = time.monotonic() - tick0
        sleep_time = target_frame_time - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

# ---------------- MJPEG stream ----------------
def mjpeg_generator():
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
    return render_template_string(HTML_TEMPLATE)

@app.route("/video")
def video():
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
    global latest_bgr, fps_ema
    h, w = 0, 0
    cx, cy = 0, 0
    
    with bgr_lock:
        if latest_bgr is not None:
            h, w = latest_bgr.shape[:2]
            cx, cy = w//2, h//2
    
    return jsonify({
        "timestamp": time.time(),
        "fps": round(fps_ema, 1),
        "center": {"x": cx, "y": cy},
        "resolution": {"width": w, "height": h},
        "overlays": {
            "grid": False,  # Always disabled
            "center_dot": True  # Always enabled
        },
        "camera_status": "active" if latest_bgr is not None else "inactive",
        "system": {
            "uptime": time.time(),
            "memory_usage": "N/A",
            "cpu_usage": "N/A"
        },
        "encoders": {"fl": 0, "fr": 0, "rl": 0, "rr": 0},
        "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "notes": "Center dot overlay active - minimal UI mode"
    })

# ---------------- Main ----------------
def main():
    parser = argparse.ArgumentParser(description="Robot Camera Stream - Minimal UI with center dot only")
    parser.add_argument('--device', type=int, default=0, help='Camera device index')
    parser.add_argument('--width', type=int, default=800, help='Camera width')
    parser.add_argument('--height', type=int, default=600, help='Camera height')
    parser.add_argument('--fps', type=int, default=30, help='Camera FPS')
    parser.add_argument('--flip', type=int, default=1, help='Flip camera horizontally (0/1)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270], help='Rotate camera')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--quality', type=int, default=90, help='JPEG quality (1-100)')
    parser.add_argument('--show-fps', type=int, default=1, help='Show FPS overlay (0/1)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Server host')
    args = parser.parse_args()

    log.info(" Starting Robot Camera Stream System")
    log.info("=" * 50)
    log.info(f" Server: http://{args.host}:{args.port}")
    log.info(f" Camera: {args.width}x{args.height}@{args.fps}fps (device {args.device})")
    log.info(f" Features: Center dot overlay, {args.quality}% quality")
    log.info(f" Options: flip={bool(args.flip)}, rotate={args.rotate}°")
    log.info("=" * 50)

    # Start camera capture thread
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

    # Wait for camera to initialize
    log.info("Initializing camera system...")
    for i in range(150):  # Wait up to 7.5 seconds
        with jpeg_lock:
            if latest_jpeg is not None:
                break
        time.sleep(0.05)
        if i % 20 == 0:
            log.info(f"   Still waiting... ({i//20 + 1}/8)")
    
    if latest_jpeg is not None:
        log.info(" Camera system ready!")
    else:
        log.warning("!  Camera not detected, running with dummy feed")
    

    try:
        app.run(host=args.host, port=args.port, threaded=True, debug=False)
    except KeyboardInterrupt:
        log.info("\n System shutdown by user")
    except Exception as e:
        log.error(f" Server error: {e}")

if __name__ == "__main__":
    main()