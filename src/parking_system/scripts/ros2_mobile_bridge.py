#!/usr/bin/env python3
"""
ROS2-Flask Bridge Node for AutoNexa Mobile App.

Subscribes to ROS2 topics (/scan, /map, /amcl_pose, /odometry/filtered)
and serves data via Flask HTTP endpoints for the Flutter mobile app.

Endpoints:
  /api/scan      - Latest LIDAR scan as (x,y) points in map frame (JSON)
  /api/map       - Real SLAM occupancy grid (PNG)
  /api/map_info  - Map metadata: resolution, origin, dimensions (JSON)
  /api/pose      - Robot pose from AMCL/EKF (JSON)
  /api/status    - Combined status (JSON)
  /api/markers   - Currently visible ArUco markers (JSON)
  /api/nav_goal  - POST a Nav2 goal (JSON {x, y, yaw})
  /api/control   - POST joystick commands (JSON {x, y, e, speed_limit})
  /api/telemetry - GET Pico telemetry (motors, encoders, odom)
  /api/estop     - POST emergency stop
  /video_feed    - Camera MJPEG stream
"""

import threading
import time
import io
from math import cos, sin, isinf, isnan

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from flask import Flask, Response, jsonify, request, send_file

# Optional: ArUco detection
try:
    import cv2.aruco as aruco
    HAS_ARUCO = True
except ImportError:
    HAS_ARUCO = False


# =============================================================================
#  ROS2 Bridge Node
# =============================================================================

class MobileBridgeNode(Node):
    def __init__(self):
        super().__init__('mobile_bridge')

        # --- Stored data (thread-safe via locks) ---
        self._scan_lock = threading.Lock()
        self._scan_points = []       # List of [x, y] in map frame
        self._scan_stamp = 0.0

        self._map_lock = threading.Lock()
        self._map_png = None         # Cached PNG bytes
        self._map_info = {}          # {width, height, resolution, origin_x, origin_y}

        self._pose_lock = threading.Lock()
        self._pose = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0, 'stamp': 0.0}
        self._pose_source = 'none'   # 'amcl', 'odom', 'none'

        self._marker_lock = threading.Lock()
        self._markers = {}           # {id: {x_m, y_m, bearing_deg, distance_m}}

        # --- Control state (joystick -> /cmd_vel) ---
        self._control_lock = threading.Lock()
        self._last_control_time = 0.0
        self._control_watchdog_s = 0.5  # zero velocity if no command in 500ms
        self._estop_latched = False  # tracks whether Pico E-STOP was latched

        # --- Telemetry from Pico ---
        self._telemetry_lock = threading.Lock()
        self._pico_telemetry = {}    # Latest telemetry dict

        # --- QoS for map (transient local, reliable) ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # --- Subscriptions ---
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)

        # --- Nav2 goal publisher ---
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # --- Joystick control: publish to /cmd_vel ---
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pico_estop_client = self.create_client(SetBool, '/pico/estop')

        # --- Pico telemetry subscriptions ---
        self.create_subscription(
            JointState, '/pico/joint_feedback', self._joint_feedback_cb, 10)
        self.create_subscription(
            Odometry, '/pico/odom', self._pico_odom_cb, 10)

        # --- Watchdog timer: zero cmd_vel if no control command recently ---
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)

        # --- Camera (optional) ---
        self._camera_lock = threading.Lock()
        self._latest_frame = None
        self._camera_running = False
        self._start_camera()

        self.get_logger().info('MobileBridgeNode started')

    # ---- Callbacks ----

    def _scan_cb(self, msg: LaserScan):
        pose = self._get_pose()
        points = self._scan_to_points(msg, pose['x_m'], pose['y_m'], pose['yaw_rad'])
        with self._scan_lock:
            self._scan_points = points
            self._scan_stamp = time.time()

    def _map_cb(self, msg: OccupancyGrid):
        png_bytes = self._occupancy_grid_to_png(msg)
        info = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
        }
        with self._map_lock:
            self._map_png = png_bytes
            self._map_info = info

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        with self._pose_lock:
            self._pose = {
                'x_m': x, 'y_m': y, 'yaw_rad': yaw,
                'stamp': time.time(),
            }
            self._pose_source = 'amcl'

    def _odom_cb(self, msg: Odometry):
        # Only use odom if AMCL hasn't provided a pose recently
        with self._pose_lock:
            if self._pose_source == 'amcl' and (time.time() - self._pose['stamp']) < 2.0:
                return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        with self._pose_lock:
            self._pose = {
                'x_m': x, 'y_m': y, 'yaw_rad': yaw,
                'stamp': time.time(),
            }
            self._pose_source = 'odom'

    def _joint_feedback_cb(self, msg: JointState):
        data = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.velocity):
                data[f'{name}_vel'] = round(msg.velocity[i], 4)
            if i < len(msg.position):
                data[f'{name}_pos'] = round(msg.position[i], 4)
        with self._telemetry_lock:
            self._pico_telemetry.update(data)
            self._pico_telemetry['joint_stamp'] = time.time()

    def _pico_odom_cb(self, msg: Odometry):
        with self._telemetry_lock:
            self._pico_telemetry['odom_x'] = round(msg.pose.pose.position.x, 4)
            self._pico_telemetry['odom_y'] = round(msg.pose.pose.position.y, 4)
            self._pico_telemetry['odom_yaw'] = round(
                self._quat_to_yaw(msg.pose.pose.orientation), 4)
            self._pico_telemetry['odom_vx'] = round(msg.twist.twist.linear.x, 4)
            self._pico_telemetry['odom_wz'] = round(msg.twist.twist.angular.z, 4)
            self._pico_telemetry['odom_stamp'] = time.time()

    def _watchdog_cb(self):
        with self._control_lock:
            if self._last_control_time > 0 and \
               (time.time() - self._last_control_time) > self._control_watchdog_s:
                self._last_control_time = 0.0
                zero = Twist()
                self._cmd_vel_pub.publish(zero)

    def publish_control(self, x: float, y: float, e: int, speed_limit: float):
        """Convert joystick input to Twist and publish on /cmd_vel."""
        max_vx = 0.35
        max_wz = 0.8
        sl = max(0.1, min(1.0, speed_limit))

        twist = Twist()
        if e:
            # Emergency: zero velocity (twist stays at zero)
            pass
        else:
            # If Pico E-STOP was previously latched, clear it now
            if self._estop_latched:
                self._clear_pico_estop()
            twist.linear.x = y * max_vx * sl
            twist.angular.z = -x * max_wz * sl

        self._cmd_vel_pub.publish(twist)
        with self._control_lock:
            self._last_control_time = time.time()

    # ---- Helpers ----

    @staticmethod
    def _quat_to_yaw(q):
        """Extract yaw from quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        from math import atan2
        return atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _scan_to_points(scan_msg, robot_x, robot_y, robot_yaw):
        """Convert LaserScan to list of (x, y) points in map frame."""
        points = []
        angle = scan_msg.angle_min
        cos_y = cos(robot_yaw)
        sin_y = sin(robot_yaw)
        for r in scan_msg.ranges:
            if (not isinf(r) and not isnan(r)
                    and scan_msg.range_min < r < scan_msg.range_max):
                # Point in robot frame
                px = r * cos(angle)
                py = r * sin(angle)
                # Transform to map frame
                mx = robot_x + px * cos_y - py * sin_y
                my = robot_y + px * sin_y + py * cos_y
                points.append([round(mx, 4), round(my, 4)])
            angle += scan_msg.angle_increment
        return points

    @staticmethod
    def _occupancy_grid_to_png(map_msg):
        """Convert nav_msgs/OccupancyGrid to PNG bytes."""
        w = map_msg.info.width
        h = map_msg.info.height
        data = np.array(map_msg.data, dtype=np.int8).reshape((h, w))
        # -1 (unknown) -> gray, 0 (free) -> white, 100 (occupied) -> black
        img = np.full((h, w), 128, dtype=np.uint8)
        img[data == 0] = 255
        img[data == 100] = 0
        # Partial occupancy: linear map 1-99 -> 254-1
        mask = (data > 0) & (data < 100)
        img[mask] = (255 - (data[mask].astype(np.uint16) * 255 // 100)).astype(np.uint8)
        # Flip Y axis (ROS origin bottom-left, image origin top-left)
        img = np.flipud(img)
        _, buf = cv2.imencode('.png', img)
        return buf.tobytes()

    def _get_pose(self):
        with self._pose_lock:
            return dict(self._pose)

    def _start_camera(self):
        """Start camera capture in background thread."""
        def _capture_loop():
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                self.get_logger().warn('No camera found, /video_feed will be unavailable')
                return
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self._camera_running = True

            aruco_dict = None
            aruco_params = None
            if HAS_ARUCO:
                aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
                aruco_params = aruco.DetectorParameters()

            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                # ArUco detection overlay
                if HAS_ARUCO and aruco_dict is not None:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners, ids, _ = aruco.detectMarkers(
                        gray, aruco_dict, parameters=aruco_params)
                    if ids is not None:
                        aruco.drawDetectedMarkers(frame, corners, ids)
                        self._update_markers(corners, ids, frame.shape[1], frame.shape[0])

                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                with self._camera_lock:
                    self._latest_frame = buf.tobytes()

                time.sleep(0.033)  # ~30 fps cap

            cap.release()

        t = threading.Thread(target=_capture_loop, daemon=True)
        t.start()

    def _update_markers(self, corners, ids, frame_w, frame_h):
        """Update detected markers with bearing/distance from camera center."""
        cam_cx = frame_w / 2
        markers = {}
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            c = corners[i][0]
            mx = float(np.mean(c[:, 0]))
            my = float(np.mean(c[:, 1]))
            # Rough bearing from camera center
            bearing = float(np.degrees(np.arctan2(mx - cam_cx, frame_h)))
            # Rough size-based distance (larger marker = closer)
            side = float(np.linalg.norm(c[0] - c[1]))
            distance = 10.0 / (side / frame_w) if side > 0 else 0.0  # rough estimate
            markers[marker_id] = {
                'bearing_deg': round(bearing, 1),
                'distance_m': round(distance / 100.0, 2),
            }
        with self._marker_lock:
            self._markers = markers

    # ---- Public accessors for Flask ----

    def get_scan(self):
        with self._scan_lock:
            return self._scan_points.copy(), self._scan_stamp

    def get_map_png(self):
        with self._map_lock:
            return self._map_png

    def get_map_info(self):
        with self._map_lock:
            return dict(self._map_info) if self._map_info else None

    def get_pose(self):
        with self._pose_lock:
            return dict(self._pose), self._pose_source

    def get_markers(self):
        with self._marker_lock:
            return dict(self._markers)

    def get_camera_frame(self):
        with self._camera_lock:
            return self._latest_frame

    def get_telemetry(self):
        with self._telemetry_lock:
            return dict(self._pico_telemetry)

    def estop(self):
        """Publish zero velocity and request latched Pico E-STOP."""
        zero = Twist()
        self._cmd_vel_pub.publish(zero)
        with self._control_lock:
            self._last_control_time = 0.0

        self._estop_latched = True

        if not self._pico_estop_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning('/pico/estop service not available, zero cmd_vel sent only')
            return

        req = SetBool.Request()
        req.data = True
        future = self._pico_estop_client.call_async(req)

        def _estop_done(fut):
            try:
                res = fut.result()
                if res is not None and res.success:
                    self.get_logger().warning('Pico E-STOP latched successfully')
                else:
                    self.get_logger().error('Pico E-STOP request returned failure')
            except Exception as exc:
                self.get_logger().error(f'Pico E-STOP request failed: {exc}')

        future.add_done_callback(_estop_done)

    def clear_estop(self):
        """Clear the Pico E-STOP latch so motors can run again."""
        self._clear_pico_estop()

    def _clear_pico_estop(self):
        """Send E-STOP clear to Pico via service call."""
        if not self._estop_latched:
            return

        self._estop_latched = False

        if not self._pico_estop_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning('/pico/estop service not available for clear')
            return

        req = SetBool.Request()
        req.data = False
        future = self._pico_estop_client.call_async(req)

        def _clear_done(fut):
            try:
                res = fut.result()
                if res is not None and res.success:
                    self.get_logger().info('Pico E-STOP cleared')
                else:
                    self.get_logger().error('Pico E-STOP clear returned failure')
            except Exception as exc:
                self.get_logger().error(f'Pico E-STOP clear failed: {exc}')

        future.add_done_callback(_clear_done)

    def send_nav_goal(self, x, y, yaw):
        """Publish a Nav2 goal."""
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        from math import cos as mcos, sin as msin
        msg.pose.orientation.z = msin(float(yaw) / 2.0)
        msg.pose.orientation.w = mcos(float(yaw) / 2.0)
        self._goal_pub.publish(msg)
        self.get_logger().info(f'Published nav goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')


# =============================================================================
#  Flask HTTP Server
# =============================================================================

flask_app = Flask(__name__)
bridge_node: MobileBridgeNode = None  # Set after node init


@flask_app.route('/api/scan')
def api_scan():
    points, stamp = bridge_node.get_scan()
    return jsonify({
        'points': points,
        'stamp': stamp,
        'count': len(points),
    })


@flask_app.route('/api/map')
def api_map():
    png = bridge_node.get_map_png()
    if png is None:
        return jsonify({'error': 'No map available yet'}), 503
    return send_file(io.BytesIO(png), mimetype='image/png')


@flask_app.route('/api/map_info')
def api_map_info():
    info = bridge_node.get_map_info()
    if info is None:
        return jsonify({'error': 'No map available yet'}), 503
    return jsonify(info)


@flask_app.route('/api/pose')
def api_pose():
    pose, source = bridge_node.get_pose()
    pose['source'] = source
    return jsonify(pose)


@flask_app.route('/api/status')
def api_status():
    pose, source = bridge_node.get_pose()
    scan_points, scan_stamp = bridge_node.get_scan()
    map_info = bridge_node.get_map_info()
    markers = bridge_node.get_markers()
    return jsonify({
        'pose': {**pose, 'source': source},
        'scan': {
            'count': len(scan_points),
            'age_s': round(time.time() - scan_stamp, 2) if scan_stamp > 0 else None,
        },
        'map': map_info,
        'markers': {str(k): v for k, v in markers.items()},
    })


@flask_app.route('/api/markers')
def api_markers():
    markers = bridge_node.get_markers()
    return jsonify({str(k): v for k, v in markers.items()})


@flask_app.route('/api/control', methods=['POST'])
def api_control():
    data = request.get_json(silent=True)
    if not data:
        return jsonify({'error': 'JSON body required'}), 400
    x = float(data.get('x', 0))
    y = float(data.get('y', 0))
    e = int(data.get('e', 0))
    speed_limit = float(data.get('speed_limit', 1.0))
    bridge_node.publish_control(x, y, e, speed_limit)
    return jsonify({'status': 'ok'})


@flask_app.route('/api/telemetry')
def api_telemetry():
    return jsonify(bridge_node.get_telemetry())


@flask_app.route('/api/estop', methods=['POST'])
def api_estop():
    bridge_node.estop()
    return jsonify({'status': 'stopped'})


@flask_app.route('/api/estop_clear', methods=['POST'])
def api_estop_clear():
    bridge_node.clear_estop()
    return jsonify({'status': 'cleared'})


@flask_app.route('/api/nav_goal', methods=['POST'])
def api_nav_goal():
    data = request.get_json(silent=True)
    if not data or 'x' not in data or 'y' not in data:
        return jsonify({'error': 'JSON body with x, y required'}), 400
    yaw = data.get('yaw', 0.0)
    bridge_node.send_nav_goal(data['x'], data['y'], yaw)
    return jsonify({'status': 'ok'})


@flask_app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            frame = bridge_node.get_camera_frame()
            if frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@flask_app.route('/')
def index():
    return '''<!doctype html>
<html><head><title>AutoNexa ROS2 Bridge</title></head>
<body style="font-family:sans-serif;background:#111;color:#eee;padding:20px">
<h1>AutoNexa ROS2 Mobile Bridge</h1>
<p>Endpoints:</p>
<ul>
<li><a href="/api/status">/api/status</a> — combined status</li>
<li><a href="/api/scan">/api/scan</a> — LIDAR scan points</li>
<li><a href="/api/map">/api/map</a> — occupancy grid PNG</li>
<li><a href="/api/map_info">/api/map_info</a> — map metadata</li>
<li><a href="/api/pose">/api/pose</a> — robot pose</li>
<li><a href="/api/markers">/api/markers</a> — ArUco markers</li>
<li><a href="/api/telemetry">/api/telemetry</a> — Pico motor/encoder telemetry</li>
<li>POST /api/control — joystick control (JSON: x, y, e, speed_limit)</li>
<li>POST /api/estop — emergency stop (latching)</li>
<li>POST /api/estop_clear — clear emergency stop</li>
<li><a href="/video_feed">/video_feed</a> — camera MJPEG</li>
</ul>
</body></html>'''


# =============================================================================
#  Main
# =============================================================================

def main():
    global bridge_node

    rclpy.init()
    bridge_node = MobileBridgeNode()

    # Spin ROS2 in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge_node,), daemon=True)
    spin_thread.start()

    bridge_node.get_logger().info('Starting Flask server on http://0.0.0.0:5000')
    flask_app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
