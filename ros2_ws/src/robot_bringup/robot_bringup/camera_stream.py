import os
import time
import threading
from dataclasses import dataclass, asdict
from typing import Tuple
from flask import Flask, Response, request, jsonify
import cv2

app = Flask(__name__)

# ---------------- Config ----------------
@dataclass
class StreamConfig:
    device: int = 0
    width: int = 640
    height: int = 480
    fps: int = 30
    flip: int = 1
    show_fps: bool = True
    show_center: bool = True
    show_speed: bool = True
    show_debug: bool = True
    jpeg_quality: int = 80
    color_text: Tuple[int,int,int] = (255,255,255)
    color_center: Tuple[int,int,int] = (0,255,255)

CONFIG = StreamConfig()
CONFIG_LOCK = threading.Lock()

# ---------------- Drive Command ----------------
@dataclass
class DriveCmd:
    linear: float = 0.0
    angular: float = 0.0
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

# ---------------- Capture ----------------
latest_frame = None
latest_lock = threading.Lock()

def open_capture(device_index=0, w=640, h=480, fps=30):
    cap = cv2.VideoCapture(device_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def capture_loop():
    global latest_frame
    t0 = time.time()
    frames = 0
    cap = open_capture(CONFIG.device, CONFIG.width, CONFIG.height, CONFIG.fps)
    if not cap.isOpened():
        print("Cannot open camera")
        time.sleep(2)

    while True:
        with CONFIG_LOCK:
            flip = CONFIG.flip
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

threading.Thread(target=capture_loop, daemon=True).start()

# ---------------- Overlay ----------------
def draw_center_marker(img, color=(0,255,255)):
    h,w = img.shape[:2]
    cx,cy = w//2, h//2
    ln = max(10, min(w,h)//20)
    thick = max(1, min(w,h)//200)
    cv2.line(img,(cx-ln,cy),(cx+ln,cy),color,thick)
    cv2.line(img,(cx,cy-ln),(cx,cy+ln),color,thick)
    cv2.circle(img,(cx,cy),thick*2+1,color,-1)

def put_text(img,text,org,color=(255,255,255)):
    cv2.putText(img,text,org,cv2.FONT_HERSHEY_SIMPLEX,0.5,color,1,cv2.LINE_AA)

def compose_overlay(frame):
    with CONFIG_LOCK:
        cfg = StreamConfig(**asdict(CONFIG))
    with DRIVE_LOCK:
        drv = DriveCmd(**asdict(DRIVE))
    with STATS_LOCK:
        cap_fps = stats["capture_fps"]
        enc_ms = stats["last_encode_ms"]
    img = frame.copy()
    if cfg.show_center:
        draw_center_marker(img,cfg.color_center)
    y = 20
    if cfg.show_fps:
        put_text(img,f"FPS: {cap_fps:.1f}",(10,y),cfg.color_text)
        y+=18
    if cfg.show_debug:
        put_text(img,f"ENC: {enc_ms:.1f} ms",(10,y),cfg.color_text)
        y+=18
        put_text(img,f"TS: {time.strftime('%H:%M:%S')}",(10,y),cfg.color_text)
        y+=18
    if cfg.show_speed:
        put_text(img,f"v={drv.linear:+.2f} m/s | w={drv.angular:+.2f} rad/s",(10,y),(0,255,0))
    return img

# ---------------- MJPEG ----------------
def mjpeg_generator():
    with STATS_LOCK:
        stats["clients"] += 1
    try:
        while True:
            with latest_lock:
                frame = latest_frame.copy() if latest_frame is not None else None
            if frame is None:
                time.sleep(0.01)
                continue
            composed = compose_overlay(frame)
            t0 = time.time()
            with CONFIG_LOCK:
                q = CONFIG.jpeg_quality
            ok, buf = cv2.imencode(".jpg",composed,[int(cv2.IMWRITE_JPEG_QUALITY),q])
            if not ok:
                continue
            jpg = buf.tobytes()
            dt = (time.time()-t0)*1000.0
            with STATS_LOCK:
                stats["frames_encoded"] += 1
                stats["last_encode_ms"] = dt
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpg+b'\r\n')
    finally:
        with STATS_LOCK:
            stats["clients"] = max(0, stats["clients"]-1)

# ---------------- Routes ----------------
@app.route("/")
def index():
    return "<h2>Camera Stream Running</h2><img src='/video'>"

@app.route("/video")
def video():
    return Response(mjpeg_generator(),mimetype='multipart/x-mixed-replace; boundary=frame')

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
    return jsonify(ok=True,config=cfg,stats=s,drive=d)

@app.route("/config",methods=["GET","POST"])
def config_route():
    if request.method=="GET":
        with CONFIG_LOCK:
            return jsonify(asdict(CONFIG))
    data = request.get_json(force=True,silent=True) or {}
    changed={}
    with CONFIG_LOCK:
        for k,v in data.items():
            if hasattr(CONFIG,k):
                setattr(CONFIG,k,v)
                changed[k]=v
    return jsonify(ok=True,changed=changed)

@app.route("/api/drive",methods=["POST"])
def api_drive():
    data = request.get_json(force=True,silent=True) or {}
    lin = float(data.get("linear",0.0))
    ang = float(data.get("angular",0.0))
    with DRIVE_LOCK:
        DRIVE.linear = max(-1.0,min(1.0,lin))
        DRIVE.angular = max(-2.0,min(2.0,ang))
        DRIVE.ts = time.time()
    return jsonify(ok=True,drive=asdict(DRIVE))

# ---------------- Main ----------------
def main():
    app.run(host="0.0.0.0",port=5000,debug=False,threaded=True)

if __name__=="__main__":
    main()
