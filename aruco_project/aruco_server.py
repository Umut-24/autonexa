import cv2
import cv2.aruco as aruco
import numpy as np
from collections import deque
from flask import Flask, Response, render_template_string, request, stream_with_context
import threading
import time
import json
import os

# ================================
#           FLASK AYARLARI
# ================================
app = Flask(__name__)

# ================================
#           SETTINGS
# ================================
MARKER_SIZE = 10.0  # cm
CAMERA_INDEX = 0

DISTANCE_SCALE = None
CALIBRATED = False
TARGET_ID = 0
NUM_MARKERS = 15

# Server control
RUNNING = True

# Telemetry shared between video generator and SSE
telemetry = {
    'target_id': TARGET_ID,
    'distance_cm': None,
    'bearing': None,
    'tx_cm': None,
    'ty_cm': None,
    'frame_time': None,
}
telemetry_lock = threading.Lock()

# Mini map area
MINIMAP_WORLD_SIZE = 600
MINIMAP = 350
MINIMAP_SCALE = MINIMAP / MINIMAP_WORLD_SIZE
MAX_TRAIL_LENGTH = 200

# Global Video Capture
cap = None

# ================================
#         TRAIL SYSTEM
# ================================
class NavigationTrail:
    def __init__(self, max_length=200):
        self.positions = deque(maxlen=max_length)

    def add_position(self, x, y):       
        self.positions.append((x, y))

    def clear(self):
        self.positions.clear()

trail = NavigationTrail(MAX_TRAIL_LENGTH)

# ================================
#     YARDIMCI FONKSIYONLAR
# ================================
def get_dummy_calibration_matrix(width, height):
    focal_length = width
    cx = width / 2
    cy = height / 2
    cam_mtx = np.array([[focal_length, 0, cx], [0, focal_length, cy], [0, 0, 1]], dtype=np.float32)
    dist = np.zeros((4, 1))
    return cam_mtx, dist

def putTextNice(img, text, pos, color, scale=0.5, thickness=1):
    cv2.putText(img, text, (pos[0] + 2, pos[1] + 2), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

def bezier_curve(p0, p1, p2, n=30):
    pts = []
    for t in np.linspace(0, 1, n):
        x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t ** 2 * p2[0]
        y = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t ** 2 * p2[1]
        pts.append([int(x), int(y)])
    return np.array(pts, np.int32)

def get_command(angle, distance_cm):
    a = abs(angle)
    if distance_cm < 2: return "Target reached."
    if a < 0.5: return f"Go straight {distance_cm:.1f} cm."
    if a < 5: return f"Turn slightly {'L' if angle < 0 else 'R'} {int(a)} deg."
    if a < 15: return f"Turn {'L' if angle < 0 else 'R'} {int(a)} deg."
    return f"Turn SHARP {'L' if angle < 0 else 'R'} {int(a)} deg."

def draw_text_block(frame, lines):
    width = 480
    height = 50 * len(lines) + 30
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (10 + width, 10 + height), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)
    y_offset = 50
    for text, color in lines:
        putTextNice(frame, text, (20, y_offset), color, 1.0, 2)
        y_offset += 50

def draw_minimap(frame, trail, tx_cm, ty_cm, bearing):
    h, w = frame.shape[:2]
    map_x = w - MINIMAP - 25
    map_y = 25
    minimap = np.zeros((MINIMAP, MINIMAP, 3), dtype=np.uint8)
    minimap[:] = (25, 25, 27)
    center = (MINIMAP // 2, MINIMAP // 2)

    def w2m(x, y):
        return (int(center[0] + x * MINIMAP_SCALE), int(center[1] - y * MINIMAP_SCALE))

    # Grid ve Arayüz
    cv2.circle(minimap, center, 6, (0, 0, 255), -1)
    if tx_cm is not None and ty_cm is not None:
        tx, ty = w2m(tx_cm, ty_cm)
        cv2.line(minimap, center, (tx, ty), (0, 255, 100), 2, cv2.LINE_AA)
        cv2.circle(minimap, (tx, ty), 7, (0, 255, 255), -1)

    putTextNice(minimap, "AutoNexa Map", (10, 20), (255, 255, 255), 0.5, 1)

    # Frame üzerine çizim
    frame[map_y:map_y + MINIMAP, map_x:map_x + MINIMAP] = minimap
    cv2.rectangle(frame, (map_x, map_y), (map_x + MINIMAP, map_y + MINIMAP), (0, 255, 255), 3)

# ================================
#        GÖRÜNTÜ İŞLEME LOOP
# ================================
def generate_frames():
    global cap, DISTANCE_SCALE, CALIBRATED, TARGET_ID, trail
    
    # Kamera başlatma (Sadece bir kez)
    if cap is None:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) # Performans için düşürdüm
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()
    
    # Sabit kalibrasyon değeri (Input yerine hardcoded yapıldı server modu için)
    KNOWN_DISTANCE_CM = 50.0 

    while RUNNING:
        success, frame = cap.read()
        if not success:
            break
        
        h, w, _ = frame.shape
        cam_mtx, dist = get_dummy_calibration_matrix(w, h)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

        target_x_cm, target_y_cm, distance_cm, bearing = None, None, 0.0, 0.0
        cx, car_y = w // 2, h - 20

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids, (100, 100, 100))
            for i in range(len(ids)):
                if ids[i][0] == TARGET_ID:
                    obj_points = np.array([
                        [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
                        [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
                        [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
                        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
                    ], dtype=np.float32)

                    _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], cam_mtx, dist)
                    dist_raw = np.linalg.norm(tvec)

                    if not CALIBRATED:
                        # Check if a calibration value was provided via the web UI
                        with telemetry_lock:
                            known = telemetry.get('_cal_known', KNOWN_DISTANCE_CM)
                        try:
                            DISTANCE_SCALE = float(known) / dist_raw
                            CALIBRATED = True
                        except Exception:
                            DISTANCE_SCALE = KNOWN_DISTANCE_CM / dist_raw
                            CALIBRATED = True
                    
                    if CALIBRATED:
                        distance_cm = dist_raw * DISTANCE_SCALE
                        target_x_cm = tvec[0][0] * DISTANCE_SCALE
                        target_y_cm = tvec[2][0] * DISTANCE_SCALE
                        bearing = np.degrees(np.arctan2(tvec[0][0], tvec[2][0]))

                        # Görselleştirmeler
                        c = corners[i][0]
                        mx, my = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
                        p0, p2 = (cx, car_y), (mx, my)
                        p1 = (cx + (mx - cx) * 0.3, (car_y + my) / 2)
                        cv2.polylines(frame, [bezier_curve(p0, p1, p2)], False, (0, 255, 0), 5)
                        cv2.rectangle(frame, (int(c[0][0]), int(c[0][1])), (int(c[2][0]), int(c[2][1])), (0, 255, 0), 2)

        # UI Çizimi
        lines = [(f"TARGET ID: {TARGET_ID}", (200, 200, 200))]
        if target_x_cm is not None:
            cmd_col = (0, 255, 0) if abs(bearing) < 5 else (0, 0, 255)
            lines.append((f"Dist: {distance_cm:.1f} cm", (255, 255, 0)))
            lines.append((get_command(bearing, distance_cm), cmd_col))
        
        draw_text_block(frame, lines)
        draw_minimap(frame, trail, target_x_cm, target_y_cm, bearing)

        # Frame'i encode et
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Update telemetry for clients
        with telemetry_lock:
            telemetry['target_id'] = TARGET_ID
            telemetry['distance_cm'] = None if target_x_cm is None else float(distance_cm)
            telemetry['bearing'] = None if target_x_cm is None else float(bearing)
            telemetry['tx_cm'] = None if target_x_cm is None else float(target_x_cm)
            telemetry['ty_cm'] = None if target_x_cm is None else float(target_y_cm)
            telemetry['frame_time'] = time.time()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
        # Minimal mobile/web app served inline
        template = '''
<!doctype html>
<html>
<head>
    <meta name="viewport" content="width=device-width,initial-scale=1" />
    <title>AutoNexa Mobile</title>
    <style>
        body { font-family: Arial, Helvetica, sans-serif; background:#111; color:#eee; margin:0; padding:12px; }
        .row { display:flex; gap:8px; flex-wrap:wrap; }
        button { padding:10px 12px; border-radius:6px; border:none; background:#2a2a2a; color:#fff }
        .video { width:100%; max-width:640px; border:4px solid #222 }
        .telemetry { background:#0b0b0b; padding:8px; border-radius:6px; margin-top:8px }
        .id-grid { display:grid; grid-template-columns:repeat(8,1fr); gap:6px; margin-top:8px }
    </style>
</head>
<body>
    <h3>AutoNexa Mobile Control</h3>
    <div class="row">
        <img class="video" id="video" src="/video_feed" />
    </div>

    <div class="telemetry" id="telemetry">Connecting...</div>

    <div style="margin-top:8px">
        <div class="row">
            <button onclick="prevId()">Prev</button>
            <button onclick="nextId()">Next</button>
            <button onclick="quitServer()" style="background:#aa2222">Quit</button>
        </div>

        <div class="id-grid" id="idGrid"></div>

        <div style="margin-top:8px">
            <input id="calib" placeholder="Calibrate distance cm" style="padding:8px; width:160px" />
            <button onclick="calibrate()">Calibrate</button>
        </div>
    </div>

    <script>
        const telemetryEl = document.getElementById('telemetry');
        const idGrid = document.getElementById('idGrid');
        function makeIdButtons(){
            for(let i=0;i<16;i++){
                const b = document.createElement('button');
                b.textContent = i;
                b.onclick = ()=> setId(i);
                idGrid.appendChild(b);
            }
        }
        makeIdButtons();

        function setId(id){ fetch(`/set_id/${id}`).then(()=>console.log('set',id)); }
        function nextId(){ fetch('/next_id'); }
        function prevId(){ fetch('/prev_id'); }
        function quitServer(){ if(confirm('Quit server?')) fetch('/quit'); }
        function calibrate(){ const v = document.getElementById('calib').value; if(v) fetch(`/calibrate?distance=${encodeURIComponent(v)}`); }

        // SSE telemetry
        const es = new EventSource('/telemetry');
        es.onmessage = (e)=>{
            const d = JSON.parse(e.data);
            telemetryEl.innerHTML = `ID: ${d.target_id} <br> Dist: ${d.distance_cm ?? 'N/A'} cm <br> Ang: ${d.bearing ?? 'N/A'} deg <br> TX: ${d.tx_cm ?? '-'} TY: ${d.ty_cm ?? '-'} `;
        };
    </script>
</body>
</html>
'''

        return render_template_string(template)


@app.route('/telemetry')
def telemetry_stream():
        def event_stream():
                while RUNNING:
                        with telemetry_lock:
                                data = json.dumps(telemetry)
                        yield f"data: {data}\n\n"
                        time.sleep(0.1)

        return Response(stream_with_context(event_stream()), mimetype='text/event-stream')


@app.route('/state')
def state_json():
    with telemetry_lock:
        data = dict(telemetry)
    return Response(json.dumps(data), mimetype='application/json')


@app.route('/prev_id')
def prev_id():
        global TARGET_ID
        TARGET_ID = (TARGET_ID - 1 + NUM_MARKERS) % NUM_MARKERS
        return "OK"


@app.route('/set_id/<int:idx>')
def set_id(idx):
        global TARGET_ID
        if 0 <= idx < NUM_MARKERS:
                TARGET_ID = idx
                return "OK"
        return "INVALID", 400


@app.route('/calibrate')
def calibrate():
        global DISTANCE_SCALE, CALIBRATED
        d = request.args.get('distance', None)
        try:
                known = float(d)
        except Exception:
                return "BAD REQUEST", 400
        # Store a known distance to apply when a marker is detected
        with telemetry_lock:
                telemetry['_cal_known'] = known
        CALIBRATED = False
        return "OK"


@app.route('/quit')
def quit_route():
        global RUNNING
        RUNNING = False
        threading.Thread(target=lambda: (time.sleep(0.5), os._exit(0)), daemon=True).start()
        return "OK"

@app.route('/next_id')
def next_id():
    global TARGET_ID
    TARGET_ID = (TARGET_ID + 1) % NUM_MARKERS
    return "OK"

if __name__ == "__main__":
    # 0.0.0.0 tüm ağ arayüzlerine açar
    print("AutoNexa Server Başlatılıyor...")
    print("Lütfen PC'nin IP adresini not edin (örn: 192.168.1.X)")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
