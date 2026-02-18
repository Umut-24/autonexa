"""
Enhanced ArUco Server with Sensor Fusion Preparation for ROS2
Features:
- Smooth path trajectory planning
- Multi-marker waypoint sequencing
- Better calibration
- JSON endpoints for mobile app map + pose visualization
- Placeholder for future ROS2 integration
"""

import cv2
import cv2.aruco as aruco
import numpy as np
from collections import deque
from flask import Flask, Response, render_template_string, request, stream_with_context, jsonify, send_file
import threading
import time
import json
import os
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import io
from PIL import Image as PILImage

# ================================
#           FLASK SETUP
# ================================
app = Flask(__name__)

# ================================
#           SETTINGS
# ================================
MARKER_SIZE = 10.0  # cm
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# Calibration & Detection
DISTANCE_SCALE = None
CALIBRATED = False
TARGET_ID = 0
NUM_MARKERS = 16
KNOWN_DISTANCE_CM = 50.0

# Testbed dimensions
TESTBED_WIDTH_CM = 200  # 2m
TESTBED_HEIGHT_CM = 200
MAP_PIXEL_SCALE = 2  # 1 cm = 2 pixels

# Server control
RUNNING = True

# ================================
#     TELEMETRY & STATE
# ================================
telemetry = {
    'target_id': TARGET_ID,
    'distance_cm': None,
    'bearing': None,
    'tx_cm': None,
    'ty_cm': None,
    'frame_time': None,
}
telemetry_lock = threading.Lock()

# Robot pose (for map visualization)
robot_pose = {
    'x_cm': TESTBED_WIDTH_CM / 2,
    'y_cm': TESTBED_HEIGHT_CM / 2,
    'theta_deg': 0.0,
    'timestamp': time.time(),
}
robot_pose_lock = threading.Lock()

# Detected parking spots (markers)
parking_spots = {}  # {marker_id: {'x': cm, 'y': cm, 'bearing': deg, 'distance': cm}}
spots_lock = threading.Lock()

# Trail system
class NavigationTrail:
    def __init__(self, max_length=200):
        self.positions = deque(maxlen=max_length)
    
    def add_position(self, x, y):
        self.positions.append((x, y))
    
    def clear(self):
        self.positions.clear()
    
    def get_smooth_path(self, n_points=50):
        """Return Catmull-Rom spline through collected positions"""
        if len(self.positions) < 4:
            return list(self.positions)
        
        # Simple smoothing: take every nth point
        points = list(self.positions)
        return points[::max(1, len(points) // n_points)]

trail = NavigationTrail(MAX_TRAIL_LENGTH := 200)

# Video capture
cap = None

# ================================
#        HELPER FUNCTIONS
# ================================

def get_dummy_calibration_matrix(width, height):
    """Camera matrix estimation from image dimensions"""
    focal_length = width
    cx = width / 2
    cy = height / 2
    cam_mtx = np.array(
        [[focal_length, 0, cx], [0, focal_length, cy], [0, 0, 1]],
        dtype=np.float32
    )
    dist = np.zeros((4, 1))
    return cam_mtx, dist

def putTextNice(img, text, pos, color, scale=0.5, thickness=1):
    """Draw text with outline for readability"""
    cv2.putText(img, text, (pos[0] + 2, pos[1] + 2),
                cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

def bezier_curve(p0, p1, p2, n=30):
    """Quadratic Bezier curve for smooth paths"""
    pts = []
    for t in np.linspace(0, 1, n):
        x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t ** 2 * p2[0]
        y = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t ** 2 * p2[1]
        pts.append([int(x), int(y)])
    return np.array(pts, np.int32)

def get_command(angle_deg, distance_cm):
    """Generate guidance command for car"""
    a = abs(angle_deg)
    if distance_cm < 2:
        return "Target reached! Begin parking maneuver."
    if a < 0.5:
        return f"Go straight {distance_cm:.1f} cm."
    if a < 5:
        return f"Turn slightly {'LEFT' if angle_deg < 0 else 'RIGHT'} {int(a)} degrees."
    if a < 15:
        return f"Turn {'LEFT' if angle_deg < 0 else 'RIGHT'} {int(a)} degrees."
    return f"Turn SHARP {'LEFT' if angle_deg < 0 else 'RIGHT'} {int(a)} degrees."

def draw_text_block(frame, lines):
    """Draw semi-transparent text overlay"""
    width = 480
    height = 50 * len(lines) + 30
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (10 + width, 10 + height), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)
    y_offset = 50
    for text, color in lines:
        putTextNice(frame, text, (20, y_offset), color, 1.0, 2)
        y_offset += 50

def create_occupancy_grid_image(width_px, height_px):
    """Create a blank occupancy grid for map visualization"""
    grid = np.ones((height_px, width_px, 3), dtype=np.uint8) * 30  # Dark background
    return grid

def add_robot_to_grid(grid, x_cm, y_cm, theta_deg, testbed_w, testbed_h, scale):
    """Draw robot position on occupancy grid"""
    x_px = int(x_cm * scale)
    y_px = int(y_cm * scale)
    
    # Ensure within bounds
    x_px = max(0, min(x_px, grid.shape[1] - 1))
    y_px = max(0, min(y_px, grid.shape[0] - 1))
    
    # Draw robot as green circle
    cv2.circle(grid, (x_px, y_px), 8, (0, 255, 0), -1)
    
    # Draw heading arrow
    angle_rad = np.radians(theta_deg)
    end_x = int(x_px + 15 * np.cos(angle_rad))
    end_y = int(y_px - 15 * np.sin(angle_rad))
    cv2.arrowedLine(grid, (x_px, y_px), (end_x, end_y), (0, 255, 255), 2, tipLength=0.3)
    
    return grid

def add_parking_spots_to_grid(grid, spots_dict, scale):
    """Draw detected parking spots (ArUco markers) on grid"""
    for marker_id, spot_data in spots_dict.items():
        x_px = int(spot_data['x'] * scale)
        y_px = int(spot_data['y'] * scale)
        
        # Ensure within bounds
        x_px = max(0, min(x_px, grid.shape[1] - 1))
        y_px = max(0, min(y_px, grid.shape[0] - 1))
        
        # Draw as blue circle
        cv2.circle(grid, (x_px, y_px), 6, (255, 0, 0), -1)
        cv2.putText(grid, str(marker_id), (x_px - 8, y_px + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    return grid

def generate_occupancy_grid_png():
    """Generate occupancy grid as PNG for mobile app"""
    width_px = int(TESTBED_WIDTH_CM * MAP_PIXEL_SCALE)
    height_px = int(TESTBED_HEIGHT_CM * MAP_PIXEL_SCALE)
    
    grid = create_occupancy_grid_image(width_px, height_px)
    
    with robot_pose_lock:
        grid = add_robot_to_grid(grid, robot_pose['x_cm'], robot_pose['y_cm'],
                                 robot_pose['theta_deg'], TESTBED_WIDTH_CM, TESTBED_HEIGHT_CM,
                                 MAP_PIXEL_SCALE)
    
    with spots_lock:
        grid = add_parking_spots_to_grid(grid, parking_spots.copy(), MAP_PIXEL_SCALE)
    
    # Convert to PNG bytes
    is_success, buffer = cv2.imencode(".png", grid)
    return io.BytesIO(buffer.tobytes())

# ================================
#        VIDEO PROCESSING LOOP
# ================================

def generate_frames():
    """Main video frame generator with ArUco detection"""
    global cap, DISTANCE_SCALE, CALIBRATED, TARGET_ID, trail
    
    if cap is None:
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()
    
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
            
            # Process each detected marker
            for i in range(len(ids)):
                marker_id = ids[i][0]
                obj_points = np.array([
                    [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
                    [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
                    [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
                    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
                ], dtype=np.float32)

                _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], cam_mtx, dist)
                dist_raw = np.linalg.norm(tvec)

                # Calibration logic
                if not CALIBRATED:
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
                    marker_x_cm = tvec[0][0] * DISTANCE_SCALE
                    marker_y_cm = tvec[2][0] * DISTANCE_SCALE
                    bearing_deg = np.degrees(np.arctan2(tvec[0][0], tvec[2][0]))
                    
                    # Store this marker's position
                    with spots_lock:
                        parking_spots[marker_id] = {
                            'x': marker_x_cm,
                            'y': marker_y_cm,
                            'bearing': bearing_deg,
                            'distance': distance_cm,
                        }
                    
                    # If this is our target, highlight and update guidance
                    if marker_id == TARGET_ID:
                        target_x_cm = marker_x_cm
                        target_y_cm = marker_y_cm
                        distance_cm = distance_cm
                        bearing = bearing_deg
                        
                        # Bezier curve from car to target
                        c = corners[i][0]
                        mx, my = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
                        p0, p2 = (cx, car_y), (mx, my)
                        p1 = (cx + (mx - cx) * 0.3, (car_y + my) / 2)
                        cv2.polylines(frame, [bezier_curve(p0, p1, p2)], False, (0, 255, 0), 5)
                        cv2.rectangle(frame, (int(c[0][0]), int(c[0][1])),
                                    (int(c[2][0]), int(c[2][1])), (0, 255, 0), 2)

        # Update robot pose estimate (simple: use target marker as reference)
        if target_x_cm is not None and distance_cm < 300:  # Within reasonable range
            with robot_pose_lock:
                # Very simple odometry: use marker as anchor point
                # In production: use wheel encoders + IMU
                robot_pose['x_cm'] = TESTBED_WIDTH_CM / 2 - target_x_cm
                robot_pose['y_cm'] = TESTBED_HEIGHT_CM / 2 + distance_cm
                robot_pose['theta_deg'] = bearing
                robot_pose['timestamp'] = time.time()

        # UI Display
        lines = [(f"TARGET ID: {TARGET_ID}", (200, 200, 200))]
        if target_x_cm is not None:
            cmd_col = (0, 255, 0) if abs(bearing) < 5 else (0, 0, 255)
            lines.append((f"Dist: {distance_cm:.1f} cm", (255, 255, 0)))
            lines.append((get_command(bearing, distance_cm), cmd_col))
            lines.append((f"Spots detected: {len(parking_spots)}", (100, 200, 255)))
        
        draw_text_block(frame, lines)

        # Encode frame
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Update telemetry
        with telemetry_lock:
            telemetry['target_id'] = TARGET_ID
            telemetry['distance_cm'] = None if target_x_cm is None else float(distance_cm)
            telemetry['bearing'] = None if target_x_cm is None else float(bearing)
            telemetry['tx_cm'] = None if target_x_cm is None else float(target_x_cm)
            telemetry['ty_cm'] = None if target_x_cm is None else float(target_y_cm)
            telemetry['frame_time'] = time.time()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# ================================
#        FLASK ROUTES
# ================================

@app.route('/video_feed')
def video_feed():
    """MJPEG video stream with ArUco detection"""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/map_image')
def map_image():
    """Occupancy grid as PNG image"""
    try:
        png_bytes = generate_occupancy_grid_png()
        return send_file(png_bytes, mimetype='image/png')
    except Exception as e:
        return {"error": str(e)}, 500

@app.route('/robot_pose')
def get_robot_pose():
    """Current robot pose in testbed coordinates"""
    with robot_pose_lock:
        pose = dict(robot_pose)
    return jsonify({
        'x_cm': pose['x_cm'],
        'y_cm': pose['y_cm'],
        'theta_deg': pose['theta_deg'],
        'timestamp': pose['timestamp'],
    })

@app.route('/parking_spots')
def get_parking_spots():
    """All detected parking spots (ArUco markers)"""
    with spots_lock:
        spots = dict(parking_spots)
    
    result = []
    for marker_id, data in spots.items():
        result.append({
            'id': marker_id,
            'x_cm': data['x'],
            'y_cm': data['y'],
            'bearing_deg': data['bearing'],
            'distance_cm': data['distance'],
        })
    
    return jsonify(result)

@app.route('/state')
def state_json():
    """Telemetry state (for mobile app)"""
    with telemetry_lock:
        data = dict(telemetry)
    return Response(json.dumps(data), mimetype='application/json')

@app.route('/telemetry')
def telemetry_stream():
    """SSE stream of telemetry updates"""
    def event_stream():
        while RUNNING:
            with telemetry_lock:
                data = json.dumps(telemetry)
            yield f"data: {data}\n\n"
            time.sleep(0.1)
    return Response(stream_with_context(event_stream()), mimetype='text/event-stream')

@app.route('/set_id/<int:idx>')
def set_id(idx):
    global TARGET_ID
    if 0 <= idx < NUM_MARKERS:
        TARGET_ID = idx
        return "OK"
    return "INVALID", 400

@app.route('/prev_id')
def prev_id():
    global TARGET_ID
    TARGET_ID = (TARGET_ID - 1 + NUM_MARKERS) % NUM_MARKERS
    return "OK"

@app.route('/next_id')
def next_id():
    global TARGET_ID
    TARGET_ID = (TARGET_ID + 1) % NUM_MARKERS
    return "OK"

@app.route('/calibrate')
def calibrate():
    global DISTANCE_SCALE, CALIBRATED
    d = request.args.get('distance', None)
    try:
        known = float(d)
    except Exception:
        return "BAD REQUEST", 400
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

@app.route('/')
def index():
    """Lightweight web UI"""
    template = '''
<!doctype html>
<html>
<head>
    <meta name="viewport" content="width=device-width,initial-scale=1" />
    <title>AutoNexa Server</title>
    <style>
        body { font-family: Arial, sans-serif; background:#111; color:#eee; margin:0; padding:12px; }
        .container { max-width: 1200px; margin: 0 auto; }
        .row { display:flex; gap:12px; flex-wrap:wrap; margin-bottom: 12px; }
        button { padding:10px 12px; border-radius:6px; border:none; background:#2a2a2a; color:#fff; cursor:pointer; }
        button:hover { background:#3a3a3a; }
        .video { max-width:640px; border:4px solid #444; width:100%; }
        .map { max-width:400px; border:2px solid #444; width:100%; }
        .telemetry { background:#0b0b0b; padding:12px; border-radius:6px; font-size: 14px; }
        .id-grid { display:grid; grid-template-columns:repeat(8,1fr); gap:6px; margin-top:8px; }
        .id-grid button { padding: 8px; font-size: 12px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>AutoNexa Autonomous Parking System</h1>
        <div class="row">
            <div>
                <h3>Camera Feed</h3>
                <img class="video" id="video" src="/video_feed" />
            </div>
            <div>
                <h3>Map</h3>
                <img class="map" id="map" src="/map_image" />
            </div>
        </div>
        <div class="telemetry" id="telemetry">Connecting...</div>
        <div style="margin-top:12px">
            <div class="row">
                <button onclick="prevId()">◄ Prev</button>
                <button onclick="nextId()">Next ►</button>
                <button onclick="quitServer()" style="background:#aa2222">Quit</button>
            </div>
            <div class="id-grid" id="idGrid"></div>
            <div style="margin-top:12px">
                <input id="calib" placeholder="Calibration distance (cm)" style="padding:8px; width:160px" />
                <button onclick="calibrate()">Calibrate</button>
            </div>
        </div>
    </div>
    <script>
        const telemetryEl = document.getElementById('telemetry');
        const idGrid = document.getElementById('idGrid');
        const mapImg = document.getElementById('map');
        
        function makeIdButtons(){
            for(let i=0;i<16;i++){
                const b = document.createElement('button');
                b.textContent = i;
                b.onclick = ()=> setId(i);
                idGrid.appendChild(b);
            }
        }
        makeIdButtons();

        function setId(id){ fetch(`/set_id/${id}`); }
        function nextId(){ fetch('/next_id'); }
        function prevId(){ fetch('/prev_id'); }
        function quitServer(){ if(confirm('Quit server?')) fetch('/quit'); }
        function calibrate(){ const v = document.getElementById('calib').value; if(v) fetch(`/calibrate?distance=${encodeURIComponent(v)}`); }

        // SSE telemetry
        const es = new EventSource('/telemetry');
        es.onmessage = (e)=>{
            const d = JSON.parse(e.data);
            telemetryEl.innerHTML = `
                <strong>Target ID:</strong> ${d.target_id}<br/>
                <strong>Distance:</strong> ${d.distance_cm ?? 'N/A'} cm<br/>
                <strong>Bearing:</strong> ${d.bearing ?? 'N/A'} degrees<br/>
                <strong>Position:</strong> X=${d.tx_cm ?? '-'} cm, Y=${d.ty_cm ?? '-'} cm
            `;
        };
        
        // Refresh map every 500ms
        setInterval(() => {
            mapImg.src = '/map_image?' + new Date().getTime();
        }, 500);
    </script>
</body>
</html>
'''
    return render_template_string(template)

if __name__ == "__main__":
    print("=" * 60)
    print("AutoNexa Autonomous Parking System - Server")
    print("=" * 60)
    print("Starting server on http://0.0.0.0:5000")
    print("Camera feed: http://<PC_IP>:5000/video_feed")
    print("Map image: http://<PC_IP>:5000/map_image")
    print("Robot pose: http://<PC_IP>:5000/robot_pose")
    print("Parking spots: http://<PC_IP>:5000/parking_spots")
    print("=" * 60)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
