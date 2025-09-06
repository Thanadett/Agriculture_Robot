#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import argparse
import threading
import time
import logging
import json
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify, request

# ---------------- Logging ----------------
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("lowlag-stream")

# ---------------- Flask ----------------
app = Flask(__name__)

# ---------------- HTML Template ----------------
HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Camera Stream</title>
    <style>
        * { margin:0; padding:0; box-sizing:border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background:#1a1a1a; color:#e8e8e8; min-height:100vh;
            display:flex; flex-direction:column; align-items:center; padding:20px; line-height:1.6;
        }
        h1 { margin-bottom: 16px; color: orange; text-align:center; font-weight:600; letter-spacing:-0.02em; }
        #video-container { position:relative; display:flex; justify-content:center; align-items:center; margin: 10px 0 16px;
            border-radius:12px; overflow:hidden; box-shadow:0 10px 40px rgba(0,0,0,.4); background:#2a2a2a; padding:8px; }
        #video-stream { width: min(90vw, 900px); height: auto; border-radius:8px; background:#000; border:1px solid #3a3a3a; }
        #overlay { position:absolute; top:18px; left:18px; color:#10a37f; background:rgba(26,26,26,.9);
            backdrop-filter: blur(8px); padding:8px 12px; font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
            font-size:12px; border-radius:8px; border:1px solid rgba(58,58,58,.6); }
        .status-indicator { display:inline-block; width:8px; height:8px; border-radius:50%; margin-right:8px; animation:pulse 2s infinite; }
        .status-connected { background:#10a37f; box-shadow:0 0 8px rgba(16,163,127,.5); }
        .status-disconnected { background:#ff4444; box-shadow:0 0 8px rgba(255,68,68,.5); }
        .status-loading { background:#ffa500; box-shadow:0 0 8px rgba(255,165,0,.5); }
        @keyframes pulse { 0%{opacity:1;transform:scale(1)} 50%{opacity:.7;transform:scale(1.1)} 100%{opacity:1;transform:scale(1)} }
        .info-panel { background:rgba(42,42,42,.8); backdrop-filter: blur(12px); padding:16px; margin:10px 0; border-radius:12px;
            border:1px solid rgba(58,58,58,.3); min-width:min(90vw, 640px); text-align:left; box-shadow:0 4px 20px rgba(0,0,0,.15); }
        .info-row { display:flex; justify-content:space-between; align-items:center; padding:8px 0; border-bottom:1px solid rgba(58,58,58,.2); }
        .info-row:last-child { border-bottom:none; }
        .info-label { color:#b0b0b0; font-size:14px; font-weight:400; }
        .info-value { color:#e8e8e8; font-size:14px; font-weight:500; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
        #telemetry { background: #101010; color:#10a37f; padding:12px; border-radius:8px; font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
            font-size:12px; line-height:1.5; white-space: pre; border:1px solid rgba(58,58,58,.3); max-height:300px; overflow:auto; }
        #telemetry::-webkit-scrollbar{width:6px} #telemetry::-webkit-scrollbar-thumb{background:rgba(16,163,127,.6); border-radius:3px}
        button { background:#10a37f; color:#111; border:none; padding:8px 12px; border-radius:8px; cursor:pointer; font-weight:600; }
        .btn-row { display:flex; gap:8px; flex-wrap:wrap; margin: 6px 0 10px; }
        .btn-secondary { background:#3a3a3a; color:#e8e8e8; }
    </style>
</head>
<body>
    <h1>Robot Camera Stream</h1>

    <div id="video-container">
        <img id="video-stream" src="/video" alt="Camera Stream">
        <div id="overlay">
            <span class="status-indicator status-loading" id="status-dot"></span>
            <span id="overlay-text">Connecting…</span>
        </div>
    </div>

    <div class="info-panel">
        <div class="info-row"><span class="info-label">Timestamp:</span><span class="info-value" id="timestamp">Loading…</span></div>
        <div class="info-row"><span class="info-label">Stream Status:</span><span class="info-value" id="stream-status">Connecting…</span></div>
        <div class="info-row"><span class="info-label">Resolution:</span><span class="info-value" id="res">-</span></div>
        <div class="info-row"><span class="info-label">Target FPS:</span><span class="info-value" id="tfps">-</span></div>
        <div class="info-row"><span class="info-label">Measured FPS:</span><span class="info-value" id="fps">0</span></div>
        <div class="info-row"><span class="info-label">Center (cx, cy):</span><span class="info-value"><span id="cx">0</span>, <span id="cy">0</span></span></div>
        <div class="btn-row">
            <button id="toggle-center" class="btn-secondary">Toggle center dot</button>
            <a href="/snapshot.jpg" target="_blank"><button>Snapshot</button></a>
            <button id="reload">Reload stream</button>
        </div>
    </div>

    <div class="info-panel">
        <div style="margin-bottom:6px; font-weight:600;">Telemetry</div>
        <div id="telemetry">Loading…</div>
    </div>

<script>
const $ = (sel) => document.querySelector(sel);
let streamConnected = false;
let telemetryErrors = 0;
let telemetryIntervalId = null;

// timestamp
function updateTimestamp(){
  const now = new Date();
  $("#timestamp").innerText = now.toLocaleString('en-GB',{year:'numeric',month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
}
setInterval(updateTimestamp, 1000); updateTimestamp();

function updateStatusIndicator(connected){
  const dot = $("#status-dot");
  const status = $("#stream-status");
  dot.className = "status-indicator " + (connected ? "status-connected" : "status-disconnected");
  status.innerText = connected ? "Connected" : "Disconnected";
  $("#overlay-text").innerText = connected ? "Live" : "Reconnecting…";
}

async function fetchTelemetry(){
  try{
    const controller = new AbortController();
    const t = setTimeout(() => controller.abort(), 5000);
    const r = await fetch("/api/telemetry", {signal: controller.signal});
    clearTimeout(t);
    if(!r.ok) throw new Error("HTTP " + r.status);
    const d = await r.json();
    $("#fps").textContent = (d.fps ?? 0).toFixed(1);
    $("#res").textContent = `${d.resolution.width}×${d.resolution.height} (actual: ${d.actual.width}×${d.actual.height})`;
    $("#tfps").textContent = d.target_fps;
    $("#cx").textContent = d.center.x;
    $("#cy").textContent = d.center.y;
    $("#telemetry").textContent = JSON.stringify(d, null, 2);

    telemetryErrors = 0;
    if(!streamConnected){ streamConnected = true; updateStatusIndicator(true); }
  }catch(e){
    telemetryErrors++;
    $("#telemetry").textContent = `Telemetry error (${telemetryErrors}): ${e.name==="AbortError"?"timeout":e.message}`;
    if(streamConnected){ streamConnected = false; updateStatusIndicator(false); }
    if(telemetryErrors > 5){ await new Promise(res=>setTimeout(res, 2000)); }
  }
}
function startTelemetry(){ if(!telemetryIntervalId){ telemetryIntervalId = setInterval(fetchTelemetry, 500); } }
function stopTelemetry(){ if(telemetryIntervalId){ clearInterval(telemetryIntervalId); telemetryIntervalId=null; } }
startTelemetry(); fetchTelemetry();

// video events
const video = $("#video-stream");
video.addEventListener("load",()=>{ streamConnected = true; updateStatusIndicator(true); });
video.addEventListener("error",()=>{ streamConnected = false; updateStatusIndicator(false);
    setTimeout(()=>{ video.src = "/video?ts="+Date.now(); }, 1500);
});

// page visibility
document.addEventListener("visibilitychange",()=>{
  if(document.hidden){ stopTelemetry(); }
  else{ startTelemetry(); fetchTelemetry(); }
});

// overlay toggle
$("#toggle-center").addEventListener("click", async ()=>{
  try{
    const cur = await (await fetch('/api/overlays')).json();
    const next = { center_dot: !cur.center_dot, grid: cur.grid };
    await fetch('/api/overlays', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(next)});
    fetchTelemetry();
  }catch(e){}
});
$("#reload").addEventListener("click", ()=>{ video.src = "/video?ts="+Date.now(); });

$("#status-dot").className="status-indicator status-loading";
$("#stream-status").innerText="Connecting…";
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

# overlay states (runtime configurable)
show_grid = False
show_center_dot = True
grid_color = (80, 80, 80)
center_dot_color = (0, 255, 0)

fps_ema = 0.0
last_frame_time = 0.0  # updated when a new frame is produced

# device/capture settings (will be set from args)
ARGS = None
ACTUAL_W = 0
ACTUAL_H = 0
ACTUAL_FPS = 0.0

# encoder/IMU placeholders (external can POST to update)
encoders_latest = {"fl":0,"fr":0,"rl":0,"rr":0}
imu_latest = {"roll":0.0,"pitch":0.0,"yaw":0.0}

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
def draw_overlay_inplace(frame_bgr, fps_value: float, draw_grid: bool, draw_center: bool):
    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2

    # grid
    if draw_grid:
        step = 40
        for x in range(step, w, step):
            cv2.line(frame_bgr, (x, 0), (x, h), grid_color, 1, cv2.LINE_AA)
        for y in range(step, h, step):
            cv2.line(frame_bgr, (0, y), (w, y), grid_color, 1, cv2.LINE_AA)

    if draw_center:
        cv2.circle(frame_bgr, (cx, cy), 5, center_dot_color, -1, cv2.LINE_AA)
        cv2.circle(frame_bgr, (cx, cy), 7, (255, 255, 255), 1, cv2.LINE_AA)

    # FPS badge
    text = f"FPS: {fps_value:.1f}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.6
    thick = 2
    size = cv2.getTextSize(text, font, scale, thick)[0]
    cv2.rectangle(frame_bgr, (12, 12), (12 + size[0] + 12, 12 + size[1] + 12), (26, 26, 26), -1)
    cv2.rectangle(frame_bgr, (12, 12), (12 + size[0] + 12, 12 + size[1] + 12), (127, 255, 16), 1)
    cv2.putText(frame_bgr, text, (18, 12 + size[1] + 3), font, scale, (127, 255, 16), thick, cv2.LINE_AA)

# ---------------- Capture helpers ----------------
def _open_capture(device_arg, backend):
    cap = None
    try:
        cap = cv2.VideoCapture(device_arg, backend)
        if cap.isOpened():
            return cap
    except Exception as e:
        log.warning(f"Open camera failed for backend {backend}: {e}")
    if cap is not None:
        cap.release()
    return None

def _parse_device(dev):
    # allow int index or path string
    if isinstance(dev, int):
        return dev
    try:
        # numeric string -> int
        return int(str(dev))
    except:
        return str(dev)

# ---------------- Capture Thread ----------------
def capture_loop(device, width, height, fps, flip=True, rotate=0, show_fps=True, jpeg_quality=90, enable_adaptive_q=True):
    global latest_jpeg, latest_bgr, fps_ema, last_frame_time, ACTUAL_W, ACTUAL_H, ACTUAL_FPS

    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    dev = _parse_device(device)

    # Try different backends (prefer GStreamer, then V4L2, then ANY)
    backends = [cv2.CAP_GSTREAMER, cv2.CAP_V4L2, cv2.CAP_ANY]
    cap = None
    for backend in backends:
        cap = _open_capture(dev, backend)
        if cap:
            log.info(f"Camera opened with backend: {backend}")
            break

    if cap is None:
        log.error(f"Cannot open camera device {device}")
        # Dummy frame loop
        dummy = np.zeros((height, width, 3), dtype=np.uint8); dummy.fill(26)
        cv2.putText(dummy, "NO CAMERA DETECTED", (max(10, width//4-60), height//2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (232,232,232), 2, cv2.LINE_AA)
        while True:
            frame = dummy.copy()
            # emulate fps counter
            now = time.monotonic()
            dt = now - last_frame_time if last_frame_time else 0
            inst = (1.0/dt) if (dt>0) else 0.0
            fps_ema = inst if fps_ema == 0.0 else (0.15*inst + 0.85*fps_ema)
            last_frame_time = now

            draw_overlay_inplace(frame, fps_ema if show_fps else 0.0, False, True)
            with bgr_lock:
                latest_bgr = frame.copy()
            try:
                jpg = encode_jpeg(frame, quality=jpeg_quality)
                with jpeg_lock:
                    latest_jpeg = jpg
                    new_frame_event.set(); new_frame_event.clear()
            except Exception as e:
                log.error(f"JPEG encode failed: {e}")
            time.sleep(1.0/max(1,fps))
        # never returns
    else:
        # Try to set properties according to args
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except: pass
        try: cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        except: pass
        try: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        except: pass
        try: cap.set(cv2.CAP_PROP_FPS, int(fps))
        except: pass
        try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except: pass
        # Try disable AF/AE if supported (keeps latency stable)
        for (prop,val) in [(cv2.CAP_PROP_AUTOFOCUS, 0), (cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)]:
            try: cap.set(prop, val)
            except: pass

    # Read actuals
    ACTUAL_W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or width
    ACTUAL_H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or height
    ACTUAL_FPS = float(cap.get(cv2.CAP_PROP_FPS) or fps)
    log.info(f"Requested: {width}x{height}@{fps} | Actual: {ACTUAL_W}x{ACTUAL_H}@{ACTUAL_FPS:.1f}")

    target_frame_time = 1.0 / max(1, fps)
    retry = 0
    max_retry = 5
    q_now = int(jpeg_quality)

    while True:
        tick0 = time.monotonic()
        ret, frame = cap.read()
        if not ret:
            retry += 1
            log.warning(f"Camera read() failed {retry}/{max_retry}")
            if retry >= max_retry:
                log.error("Too many read() failures, attempting to reopen…")
                cap.release()
                time.sleep(1.0)
                # try alternative backend on reopen
                for backend in backends[1:]+backends[:1]:
                    cap = _open_capture(dev, backend)
                    if cap: break
                if cap is None:
                    log.error("Failed to reopen camera")
                    time.sleep(2)
                    continue
                retry = 0
            time.sleep(0.01)
            continue
        retry = 0

        # Resize only if needed (keeps compute low if camera outputs correct size)
        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, (width, height))

        # Transform
        if flip:
            frame = cv2.flip(frame, 1)
        if rotate == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotate == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif rotate == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # FPS calc first (to paint the latest)
        now = time.monotonic()
        dt = now - last_frame_time if last_frame_time else 0
        inst = (1.0/dt) if (dt>0) else 0.0
        fps_ema_val = inst if fps_ema == 0.0 else (0.15*inst + 0.85*fps_ema)

        # draw overlay
        draw_overlay_inplace(frame, fps_ema_val if show_fps else 0.0, show_grid, show_center_dot)

        # store bgr & encode
        with bgr_lock:
            latest_bgr = frame.copy()

        # adaptive JPEG quality (keeps latency under load)
        elapsed_pre = time.monotonic() - tick0
        if enable_adaptive_q:
            if elapsed_pre > target_frame_time * 1.2 and q_now > 70:
                q_now -= 2
            elif elapsed_pre < target_frame_time * 0.7 and q_now < jpeg_quality:
                q_now += 1

        try:
            jpg = encode_jpeg(frame, quality=q_now)
        except Exception as e:
            log.error(f"JPEG encode failed: {e}")
            time.sleep(0.005)
            continue

        with jpeg_lock:
            latest_jpeg = jpg
            new_frame_event.set(); new_frame_event.clear()

        # finalize timing
        last_frame_time = now
        fps_ema = fps_ema_val

        # frame pacing
        elapsed = time.monotonic() - tick0
        sleep_time = target_frame_time - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

# ---------------- MJPEG stream ----------------
def mjpeg_generator():
    boundary = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    while True:
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

@app.route("/snapshot.jpg")
def snapshot():
    with jpeg_lock:
        buf = latest_jpeg
    if buf is None:
        return ("NO FRAME", 503)
    return (buf, 200, {"Content-Type": "image/jpeg", "Cache-Control": "no-store"})

@app.route("/healthz")
def healthz():
    alive = (latest_bgr is not None) and (time.monotonic() - last_frame_time < 2.0)
    return ("OK", 200) if alive else ("STALE", 503)

@app.route("/api/overlays", methods=["GET","POST"])
def api_overlays():
    global show_grid, show_center_dot
    if request.method == "POST":
        try:
            data = request.get_json(force=True) or {}
            if "grid" in data: show_grid = bool(data["grid"])
            if "center_dot" in data: show_center_dot = bool(data["center_dot"])
        except Exception as e:
            return jsonify({"error": str(e)}), 400
    return jsonify({"grid": show_grid, "center_dot": show_center_dot})

@app.route("/api/encoders", methods=["GET","POST"])
def api_encoders():
    global encoders_latest
    if request.method == "POST":
        try:
            data = request.get_json(force=True) or {}
            encoders_latest.update({k:int(v) for k,v in data.items() if k in encoders_latest})
        except Exception as e:
            return jsonify({"error": str(e)}), 400
    return jsonify(encoders_latest)

@app.route("/api/imu", methods=["GET","POST"])
def api_imu():
    global imu_latest
    if request.method == "POST":
        try:
            data = request.get_json(force=True) or {}
            for k in ("roll","pitch","yaw"):
                if k in data: imu_latest[k] = float(data[k])
        except Exception as e:
            return jsonify({"error": str(e)}), 400
    return jsonify(imu_latest)

@app.route("/api/telemetry")
def api_telemetry():
    global fps_ema, latest_bgr
    with bgr_lock:
        if latest_bgr is not None:
            h, w = latest_bgr.shape[:2]
        else:
            h, w = ARGS.height, ARGS.width
    cx, cy = w//2, h//2
    return jsonify({
        "timestamp": time.time(),
        "fps": round(float(fps_ema), 1),
        "resolution": {"width": int(ARGS.width), "height": int(ARGS.height)},
        "actual": {"width": int(ACTUAL_W or ARGS.width), "height": int(ACTUAL_H or ARGS.height), "fps": float(ACTUAL_FPS or ARGS.fps)},
        "target_fps": int(ARGS.fps),
        "center": {"x": int(cx), "y": int(cy)},
        "overlays": {"grid": bool(show_grid), "center_dot": bool(show_center_dot)},
        "camera_status": "active" if latest_bgr is not None else "inactive",
        "encoders": encoders_latest,
        "imu": imu_latest,
        "system": {"uptime": time.monotonic(), "ui_theme": "dark", "quality": int(ARGS.quality)},
    })

# ---------------- Main ----------------
def main():
    global ARGS
    parser = argparse.ArgumentParser(description="Robot Camera Stream")
    parser.add_argument('--device', type=str, default='0', help='Camera device index or path (e.g., 0 or /dev/video0)')
    parser.add_argument('--width', type=int, default=800, help='Camera width')
    parser.add_argument('--height', type=int, default=600, help='Camera height')
    parser.add_argument('--fps', type=int, default=30, help='Camera FPS')
    parser.add_argument('--flip', type=int, default=1, help='Flip camera horizontally (0/1)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270], help='Rotate camera')
    parser.add_argument('--port', type=int, default=5000, help='Server port')
    parser.add_argument('--quality', type=int, default=90, help='JPEG quality (1-100)')
    parser.add_argument('--show-fps', type=int, default=1, help='Show FPS overlay (0/1)')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Server host')
    parser.add_argument('--no-adaptive-quality', action='store_true', help='Disable adaptive JPEG quality control')
    ARGS = parser.parse_args()

    log.info("Starting Robot Camera Stream System")
    log.info("=" * 60)
    log.info(f"Server: http://{ARGS.host}:{ARGS.port}")
    log.info(f"Camera request: {ARGS.width}×{ARGS.height}@{ARGS.fps} (device {ARGS.device})")
    log.info(f"Quality: {ARGS.quality}% | Options: flip={bool(ARGS.flip)}, rotate={ARGS.rotate}°")
    log.info("=" * 60)

    th_cap = threading.Thread(
        target=capture_loop,
        kwargs=dict(
            device=ARGS.device,
            width=ARGS.width,
            height=ARGS.height,
            fps=ARGS.fps,
            flip=bool(ARGS.flip),
            rotate=ARGS.rotate,
            show_fps=bool(ARGS.show_fps),
            jpeg_quality=ARGS.quality,
            enable_adaptive_q=not ARGS.no_adaptive_quality,
        ),
        daemon=True
    )
    th_cap.start()

    # Wait for first frame (up to 7.5s)
    log.info("Initializing camera system…")
    for i in range(150):
        with jpeg_lock:
            if latest_jpeg is not None:
                break
        time.sleep(0.05)
        if i % 20 == 0:
            log.info(f"Still waiting… ({i//20 + 1}/8)")

    if latest_jpeg is not None:
        log.info("Camera system ready!")
    else:
        log.warning("Camera not detected, running with dummy feed")

    try:
        # threaded=True keeps MJPEG chunks responsive
        app.run(host=ARGS.host, port=ARGS.port, threaded=True, debug=False)
    except KeyboardInterrupt:
        log.info("\nSystem shutdown by user")
    except Exception as e:
        log.error(f"Server error: {e}")

if __name__ == "__main__":
    main()