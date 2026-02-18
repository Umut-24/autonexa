import cv2
import cv2.aruco as aruco
import numpy as np
from collections import deque
import sys
import os

# ================================
#           SETTINGS
# ================================
MARKER_SIZE = 10.0  # cm
# Allow camera index/path to be set via command line or environment variable
if len(sys.argv) > 1:
    arg = sys.argv[1]
    if arg.isdigit():
        CAMERA_INDEX = int(arg)
    elif arg.startswith('/dev/video'):
        CAMERA_INDEX = arg  # Device path
    else:
        CAMERA_INDEX = 0
elif 'CAMERA_INDEX' in os.environ:
    env_val = os.environ['CAMERA_INDEX']
    if env_val.isdigit():
        CAMERA_INDEX = int(env_val)
    elif env_val.startswith('/dev/video'):
        CAMERA_INDEX = env_val
    else:
        CAMERA_INDEX = 0
else:
    CAMERA_INDEX = 0

DISTANCE_SCALE = None
CALIBRATED = False

TARGET_ID = 0      # Currently selected target
NUM_MARKERS = 15   # Support IDs 0 to 14

# Mini map area
MINIMAP_WORLD_SIZE = 600  # cm
MINIMAP = 350
MINIMAP_SCALE = MINIMAP / MINIMAP_WORLD_SIZE
MAX_TRAIL_LENGTH = 200


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


# ================================
#     DUMMY CALIBRATION MATRIX
# ================================
def get_dummy_calibration_matrix(width, height):
    focal_length = width
    cx = width / 2
    cy = height / 2

    cam_mtx = np.array([
        [focal_length, 0, cx],
        [0, focal_length, cy],
        [0, 0, 1]
    ], dtype=np.float32)

    dist = np.zeros((4, 1))
    return cam_mtx, dist


# ================================
#        BETTER TEXT DRAWING
# ================================
def putTextNice(img, text, pos, color, scale=0.5, thickness=1):
    cv2.putText(img, text, (pos[0] + 2, pos[1] + 2),
                cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, pos,
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


# ================================
#          BEZIER CURVE
# ================================
def bezier_curve(p0, p1, p2, n=30):
    pts = []
    for t in np.linspace(0, 1, n):
        x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t ** 2 * p2[0]
        y = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t ** 2 * p2[1]
        pts.append([int(x), int(y)])
    return np.array(pts, np.int32)


# ================================
#         COMMAND SYSTEM
# ================================
def get_command(angle, distance_cm):
    a = abs(angle)

    if distance_cm < 2:
        return "Target reached."
    if a < 0.5:
        return f"Go straight for {distance_cm:.1f} cm."
    if a < 5:
        return f"Turn slightly {'left' if angle < 0 else 'right'} by {int(a)} deg."
    if a < 15:
        return f"Turn {'left' if angle < 0 else 'right'} by {int(a)} deg."
    if a < 30:
        return f"Turn sharply {'left' if angle < 0 else 'right'} by {int(a)} deg."
    return f"Turn extremely {'left' if angle < 0 else 'right'} by {int(a)} deg."


# ================================
#         TEXT BLOCK UI
# ================================
def draw_text_block(frame, lines):
    width = 480
    height = 50 * len(lines) + 30

    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (10 + width, 10 + height),
                  (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    y_offset = 50
    for text, color in lines:
        putTextNice(frame, text, (20, y_offset), color, 1.0, 2)
        y_offset += 50


# ================================
#       MINI-MAP (UPDATED)
# ================================
def draw_minimap(frame, trail, tx_cm, ty_cm, bearing):
    h, w = frame.shape[:2]

    # Map Config
    WORLD_CM = 400
    MINIMAP = 350
    SCALE = MINIMAP / WORLD_CM

    # Map position (Top Right)
    map_x = w - MINIMAP - 25
    map_y = 25

    minimap = np.zeros((MINIMAP, MINIMAP, 3), dtype=np.uint8)
    minimap[:] = (25, 25, 27)

    center = (MINIMAP // 2, MINIMAP // 2)

    def w2m(x, y):
        return (
            int(center[0] + x * SCALE),
            int(center[1] - y * SCALE)
        )

    # ---------- FIXED GRID ----------
    grid_50 = max(1, round(50 * SCALE))
    grid_100 = max(1, round(100 * SCALE))

    for i in range(0, MINIMAP, grid_50):
        cv2.line(minimap, (i, 0), (i, MINIMAP), (42, 42, 45), 1)
        cv2.line(minimap, (0, i), (MINIMAP, i), (42, 42, 45), 1)

    for i in range(0, MINIMAP, grid_100):
        cv2.line(minimap, (i, 0), (i, MINIMAP), (75, 75, 80), 2)
        cv2.line(minimap, (0, i), (MINIMAP, i), (75, 75, 80), 2)

    # ---------- CAR DOT ----------
    cv2.circle(minimap, center, 6, (0, 0, 255), -1)

    # ---------- TARGET ----------
    if tx_cm is not None and ty_cm is not None:
        tx, ty = w2m(tx_cm, ty_cm)
        # Line to target
        cv2.line(minimap, center, (tx, ty), (0, 255, 100), 2, cv2.LINE_AA)
        # Target Dot
        cv2.circle(minimap, (tx, ty), 7, (0, 255, 255), -1)

    # ---------- CIRCLES ----------
    rad3 = int(300 * SCALE)
    rad1 = int(100 * SCALE)

    cv2.circle(minimap, center, rad3, (0, 160, 255), 2, cv2.LINE_AA)
    cv2.circle(minimap, center, rad1, (80, 120, 180), 1, cv2.LINE_AA)

    putTextNice(minimap, "1m", (center[0] + rad1 - 22, center[1] + 5),
                (120, 160, 200), 0.40, 1)

    # ---------- COMPASS ----------
    putTextNice(minimap, "N", (center[0] - 8, 22), (255, 255, 255), 0.55, 2)
    putTextNice(minimap, "S", (center[0] - 8, MINIMAP - 10), (255, 255, 255), 0.55, 2)
    putTextNice(minimap, "E", (MINIMAP - 25, center[1] + 6), (255, 255, 255), 0.55, 2)
    putTextNice(minimap, "W", (10, center[1] + 6), (255, 255, 255), 0.55, 2)

    # ---------- SCALE TEXT ----------
    scale_text = f"1 px = 0.5 m"
    putTextNice(minimap, scale_text, (10, MINIMAP - 12), (180, 220, 160), 0.45, 1)

    # ---------- DRAW ON FRAME ----------
    shadow = frame.copy()
    cv2.rectangle(shadow, (map_x - 4, map_y - 4),
                  (map_x + MINIMAP + 4, map_y + MINIMAP + 4),
                  (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(shadow, 0.35, frame, 0.65, 0)

    frame[map_y:map_y + MINIMAP, map_x:map_x + MINIMAP] = minimap

    cv2.rectangle(frame, (map_x, map_y),
                  (map_x + MINIMAP, map_y + MINIMAP),
                  (0, 255, 255), 3)


# ================================
#       CAMERA DETECTION
# ================================
def find_camera(camera_index=None):
    """Try to find a working camera device"""
    if camera_index is None:
        camera_index = CAMERA_INDEX
    
    # First, try the configured index/path
    if camera_index is not None:
        cap = cv2.VideoCapture(camera_index)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"Found camera at {camera_index}")
                return cap
            cap.release()
    
    # Try common indices
    print("Searching for camera...")
    for idx in range(10):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"Found camera at index {idx}")
                return cap
            cap.release()
    
    # Try device paths
    video_devices = []
    if os.path.exists('/dev'):
        for f in os.listdir('/dev'):
            if f.startswith('video') and f[5:].isdigit():
                video_devices.append(f'/dev/{f}')
    
    # Sort by device number
    video_devices.sort(key=lambda x: int(x.split('video')[1]))
    
    for dev_path in video_devices:
        try:
            cap = cv2.VideoCapture(dev_path)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    print(f"Found camera at {dev_path}")
                    return cap
                cap.release()
        except:
            continue
    
    # Try GStreamer pipeline for Raspberry Pi cameras (if available)
    try:
        # Common GStreamer pipeline for Raspberry Pi camera
        gst_pipeline = (
            "libcamerasrc ! "
            "video/x-raw,width=1920,height=1080,format=RGB ! "
            "videoconvert ! "
            "appsink"
        )
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print("Found camera using GStreamer (libcamera)")
                return cap
            cap.release()
    except:
        pass
    
    return None


# ================================
#            MAIN LOOP
# ================================
def main():
    global DISTANCE_SCALE, CALIBRATED, TARGET_ID

    print("Enter real distance (cm) for calibration:")
    try:
        KNOWN_DISTANCE_CM = float(input("> "))
    except ValueError:
        KNOWN_DISTANCE_CM = 50.0  # default fallback
        print("Invalid input, using 50.0 cm")

    print(f"Initial Target ID: {TARGET_ID}")
    
    # Try to find a working camera
    cap = find_camera()
    
    if cap is None:
        print("\nERROR: Could not find any working camera!")
        print("\nTroubleshooting:")
        print("1. Check if camera is connected")
        print("2. Check camera permissions: ls -l /dev/video*")
        print("3. Try adding your user to video group: sudo usermod -a -G video $USER")
        print("4. For Raspberry Pi, you may need to enable camera in raspi-config")
        print("5. Try specifying camera manually:")
        print("   python3 aruco.py 20")
        print("   python3 aruco.py /dev/video20")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # Standard 4x4 dictionary (supports IDs 0-49)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()

    cv2.namedWindow("AutoNexa Navigation Pro", cv2.WINDOW_NORMAL)

    trail = NavigationTrail(MAX_TRAIL_LENGTH)

    print("\nSystem Ready.")
    print("Controls:")
    print("  'n' -> Next Target ID")
    print("  'p' -> Previous Target ID")
    print("  'c' -> Clear Trail")
    print("  'q' -> Quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w, _ = frame.shape
        cam_mtx, dist = get_dummy_calibration_matrix(w, h)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)

        target_x_cm = None
        target_y_cm = None
        current_bearing = None
        distance_cm = 0.0
        bearing = 0.0

        cx = w // 2
        car_y = h - 20

        # Draw ALL detected markers faintly (gray) just for visualization
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids, (100, 100, 100))

            for i in range(len(ids)):
                current_id = ids[i][0]

                # If this is the TARGET we want to track
                if current_id == TARGET_ID:
                    # Solve PnP
                    obj_points = np.array([
                        [-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
                        [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
                        [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
                        [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
                    ], dtype=np.float32)

                    _, rvec, tvec = cv2.solvePnP(obj_points, corners[i], cam_mtx, dist)
                    dist_raw = np.linalg.norm(tvec)

                    # Calibration
                    if not CALIBRATED:
                        DISTANCE_SCALE = KNOWN_DISTANCE_CM / dist_raw
                        CALIBRATED = True
                        print("\n>> Calibration complete.")
                        print(f"DISTANCE_SCALE = {DISTANCE_SCALE:.4f}")

                    if CALIBRATED:
                        distance_cm = dist_raw * DISTANCE_SCALE
                        target_x_cm = tvec[0][0] * DISTANCE_SCALE
                        target_y_cm = tvec[2][0] * DISTANCE_SCALE

                    # Calculate Bearing
                    bearing = np.degrees(np.arctan2(tvec[0][0], tvec[2][0]))
                    current_bearing = bearing

                    # Visuals for TARGET (Bright Colors)
                    c = corners[i][0]
                    mx = int(np.mean(c[:, 0]))
                    my = int(np.mean(c[:, 1]))

                    # Color coding for angle
                    a = abs(bearing)
                    col = (0, 255, 0) if a < 5 else (0, 255, 255) if a < 20 else (0, 0, 255)

                    # Bezier Curve
                    p0 = (cx, car_y)
                    p2 = (mx, my)
                    p1 = (cx + (mx - cx) * 0.3, (car_y + my) / 2)
                    curve = bezier_curve(p0, p1, p2)
                    cv2.polylines(frame, [curve], False, col, 5)

                    # Highlight the target marker specifically
                    cv2.rectangle(frame, (int(c[0][0]), int(c[0][1])), 
                                  (int(c[2][0]), int(c[2][1])), (0, 255, 0), 2)

        # ---------------- UI UPDATES ----------------
        lines = []
        
        # Always show which ID we are tracking
        lines.append((f"TARGET ID: [ {TARGET_ID} ]", (200, 200, 200)))

        if target_x_cm is not None:
            command = get_command(bearing, distance_cm)
            a_val = abs(bearing)
            cmd_col = (0, 255, 0) if a_val < 5 else (0, 255, 255) if a_val < 20 else (0, 0, 255)

            lines.append((f"Dist: {distance_cm:.1f} cm", (255, 255, 0)))
            lines.append((f"Ang: {bearing:.1f} deg", (0, 255, 255)))
            lines.append((command, cmd_col))
        else:
            putTextNice(frame, f"Searching for ID {TARGET_ID}...",
                        (w // 2 - 250, h // 2), (150, 150, 150), 1.0, 2)

        draw_text_block(frame, lines)
        draw_minimap(frame, trail, target_x_cm, target_y_cm, current_bearing)

        cv2.imshow("AutoNexa Navigation Pro", frame)

        key = cv2.waitKey(1) & 0xFF

        # ===== INPUT CONTROLS =====
        if key == ord('n'):  # Next
            TARGET_ID = (TARGET_ID + 1) % NUM_MARKERS
            trail.clear()
            print(f">> Switched to ID {TARGET_ID}")

        elif key == ord('p'):  # Previous
            TARGET_ID = (TARGET_ID - 1 + NUM_MARKERS) % NUM_MARKERS
            trail.clear()
            print(f">> Switched to ID {TARGET_ID}")

        elif key == ord('c'):
            trail.clear()
            print("Trail cleared!")

        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']:
        print("Usage: python3 aruco.py [CAMERA_INDEX or /dev/videoX]")
        print("\nExamples:")
        print("  python3 aruco.py              # Try to auto-detect camera")
        print("  python3 aruco.py 0            # Use camera index 0")
        print("  python3 aruco.py /dev/video20 # Use specific device path")
        print("\nEnvironment variable:")
        print("  CAMERA_INDEX=20 python3 aruco.py")
        sys.exit(0)
    main()