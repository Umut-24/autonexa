#!/usr/bin/env python3
"""
ROS2-Flask Bridge Node for AutoNexa Mobile App.

Subscribes to ROS2 topics (/scan, /map, /amcl_pose, /odometry/filtered)
and serves data via Flask HTTP endpoints for the Flutter mobile app.

REST Endpoints:
  /api/scan        - Latest LIDAR scan as (x,y) points in map frame (JSON)
  /api/map         - Real SLAM occupancy grid (PNG)
  /api/map_info    - Map metadata: resolution, origin, dimensions (JSON)
  /api/map_version - Bumps on each /map update; clients ETag the PNG
  /api/plan        - Latest Nav2 planner output (JSON polyline)
  /api/goal        - Active goal pose (JSON x/y/yaw/active/stamp)
  /api/pose        - Robot pose from AMCL/EKF/odom (JSON)
  /api/status      - Combined status (JSON)
  /api/markers     - Currently visible ArUco markers (JSON)
  /api/nav_goal    - POST a Nav2 goal (JSON {x, y, yaw})
  /api/cancel_nav  - POST to cancel current Nav2 goal (stops autonomous motion)
  /api/control     - POST joystick commands (JSON {x, y, e, speed_limit})
  /api/telemetry   - GET Pico telemetry (motors, encoders, odom)
  /api/estop       - POST emergency stop (cancels Nav2 + Pico latch)
  /api/estop_clear - POST to clear the Pico E-STOP latch
  /api/mode        - GET/POST AUTO/MANUAL/ESTOP control state machine
  /api/nav_status  - Nav2 NavigateToPose action status string

WebSocket Endpoints:
  /ws/control      - Joystick stream (client -> bridge), JSON per message
  /ws/telemetry    - Server-pushed snapshots at 10 Hz: pose + telemetry +
                     mode + nav_status + goal

  /video_feed      - Camera MJPEG stream

Runtime Python deps (pip): flask, flask-sock, numpy, opencv-python.
"""

import json
import os
import subprocess
import threading
import time
import io
import zlib
from collections import deque
from math import cos, sin, isinf, isnan, pi as MATH_PI

# Static yaw rotation from base_link to laser_link, in radians.
# Mirrors the static TF in nav2_live_slam.launch.py — kept as a constant
# here because _scan_to_points() does its own math instead of going through
# tf2 (each scan callback would otherwise pay a TF lookup it doesn't need;
# the laser orientation is fixed by hardware).
# 2026-05-07: LiDAR mounted facing rearward → static TF set to π in yaw,
# this offset must match or the app's scan dots rotate off the walls.
LASER_LINK_YAW_OFFSET = MATH_PI

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import ParameterType
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

try:
    from nav2_msgs.srv import ClearEntireCostmap
except ImportError:
    ClearEntireCostmap = None
from flask import Flask, Response, jsonify, request, send_file
from flask_sock import Sock

try:
    import yaml
except ImportError:
    yaml = None

# Persistent app data (waypoints + runtime overrides). Lives outside the ROS
# workspace so it survives `colcon build --symlink-install` clobbering and is
# trivial to back up / hand-edit.
AUTONEXA_DATA_DIR = os.path.expanduser('~/.autonexa')
RUNTIME_OVERRIDES_PATH = os.path.join(AUTONEXA_DATA_DIR, 'runtime_overrides.yaml')
WAYPOINTS_PATH = os.path.join(AUTONEXA_DATA_DIR, 'waypoints.json')
MAPS_DIR = os.path.join(AUTONEXA_DATA_DIR, 'maps')

# Parameter-tuner whitelist — only these nodes can be reached from the app's
# generic /api/params endpoint. Keeps the surface small and predictable.
PARAM_TUNER_WHITELIST = (
    '/nav2_pico_bridge',
    '/controller_server',
    '/planner_server',
    '/velocity_smoother',
    '/global_costmap/global_costmap',
    '/local_costmap/local_costmap',
)

# Topics monitored by /api/health. Each tuple: (topic, expected_min_hz, label).
# Rates are deliberately conservative — set the floor to "if we drop below
# this, something is wrong", not the steady-state rate.
HEALTH_TOPICS = (
    ('/scan',                  5.0, 'LiDAR scan'),
    ('/map',                   0.05, 'SLAM map'),
    ('/odom',                  5.0, 'Wheel/scan odometry'),
    ('/cmd_vel_safe',          0.5, 'Safety-gated cmd_vel'),
    ('/pico/joint_feedback',   2.0, 'Pico joint feedback'),
    ('/plan',                  0.2, 'Nav2 plan'),
)

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
        self._map_version = 0        # Increments on each /map update; clients poll
                                     # /api/map_version to skip redundant PNG fetches.

        # --- Nav2 plan + current goal ---
        self._plan_lock = threading.Lock()
        self._plan_points = []       # Downsampled [[x, y], ...] in map frame
        self._plan_stamp = 0.0

        self._goal_lock = threading.Lock()
        self._current_goal = {       # Last goal sent (active until cancel)
            'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'active': False, 'stamp': 0.0,
        }

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

        # --- Control mode state machine: AUTO / MANUAL / ESTOP -------------
        # AUTO   — Nav2 owns /cmd_vel; joystick input is ignored.
        # MANUAL — joystick owns /cmd_vel; any active Nav2 goal is cancelled
        #          on entry so the BT navigator stops publishing.
        # ESTOP  — zero velocity + Pico hardware latch; both joystick and
        #          Nav2 are blocked.
        # The mode is the single source of truth — every cmd_vel publisher
        # in this bridge consults it before publishing.
        self._mode_lock = threading.Lock()
        self._mode = 'MANUAL'         # safe default — no autonomous motion at boot
        self._mode_stamp = time.time()

        # --- Nav2 action goal status -----------------------------------------
        # Latest status code from /navigate_to_pose/_action/status. Mapped to
        # human-readable strings before serializing to clients.
        self._nav_status_lock = threading.Lock()
        self._nav_status = 'IDLE'      # IDLE/PLANNING/EXECUTING/SUCCEEDED/CANCELED/ABORTED
        self._nav_status_stamp = 0.0

        # --- WebSocket subscriber registry ----------------------------------
        # Telemetry pusher fans out to every connected /ws/telemetry client.
        # Adds/removes are handled inside the per-connection handler.
        self._ws_lock = threading.Lock()
        self._ws_telemetry_clients = set()

        # --- Telemetry from Pico ---
        self._telemetry_lock = threading.Lock()
        self._pico_telemetry = {}    # Latest telemetry dict

        # --- Safety mode (Part A) ---
        # 'soft' (default): joystick -> /cmd_vel -> velocity_smoother ->
        #     collision_monitor -> /cmd_vel_safe -> Pico bridge. Walls block.
        # 'off': joystick -> /cmd_vel_manual -> Pico bridge directly. The
        #     user is in control. nav2_pico_bridge still applies its own
        #     vx/wz clamps + watchdog so a runaway is still bounded.
        # Only meaningful in MANUAL mode; AUTO always uses /cmd_vel_safe.
        self._safety_lock = threading.Lock()
        self._safety_mode = 'soft'

        # --- Map fingerprint (Part D) ---
        # Lets waypoints know if the SLAM map under them has been replaced
        # since they were saved (full SLAM restart -> new fingerprint).
        self._map_fingerprint = ''

        # --- Waypoints (Part D) ---
        self._waypoints_lock = threading.Lock()
        self._waypoints = []   # list[dict]; persisted to WAYPOINTS_PATH

        # --- Topic health stats (Part G3) ---
        # EWMA rate per monitored topic so /api/health can report green /
        # yellow / red without bringing in `ros2 topic hz` machinery.
        self._health_lock = threading.Lock()
        self._health_stats = {topic: {'last_t': 0.0, 'rate': 0.0}
                              for topic, _, _ in HEALTH_TOPICS}

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
        # Fuse either /odometry/filtered (EKF, when launched) or /odom
        # (laser_scan_matcher output — active in live-SLAM mode). First
        # callback to fire wins; AMCL takes priority when available.
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

        # Health-only subscription on /cmd_vel_safe so the diagnostics panel
        # can detect a stalled safety chain (smoother / collision_monitor) even
        # when the robot is idle. Cheap; a Twist is ~50 B.
        self.create_subscription(
            Twist, '/cmd_vel_safe', self._cmd_vel_safe_cb, 10)

        # Nav2 planner output — visualized as a polyline in the
        # mobile app. Depth 1; we replace fully on each new plan.
        self.create_subscription(
            Path, '/plan', self._plan_cb, 1)
        # Track goals issued from RViz too (so the marker is correct regardless
        # of who set the goal — app or RViz).
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_cb, 10)
        # NavigateToPose action status — gives us PLANNING/EXECUTING/SUCCEEDED/
        # ABORTED/CANCELED so the app can show what Nav2 is actually doing,
        # not just "we sent a goal".
        nav_status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._nav_status_cb, nav_status_qos)

        # --- Nav2 goal publisher ---
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # --- Nav2 cancel service client ---
        # The BT navigator exposes /navigate_to_pose/_action/cancel_goal.
        # An empty CancelGoal.Request cancels *all* active goals — which
        # is exactly the "stop autonomous motion" behavior the app wants.
        self._nav_cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        # --- Joystick control: publish to /cmd_vel ---
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Safety-bypass channel (Part A). nav2_pico_bridge subscribes to this
        # *and* /cmd_vel_safe; freshest message in the last 200 ms wins.
        self._cmd_vel_manual_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        # Pose reset / relocalize (Part G1). AMCL convention; SLAM Toolbox also
        # snaps its pose graph to /initialpose when running in mapping mode.
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._pico_estop_client = self.create_client(SetBool, '/pico/estop')

        # --- Costmap clear + SLAM restart clients (Part C) ---
        if ClearEntireCostmap is not None:
            self._global_costmap_clear = self.create_client(
                ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            self._local_costmap_clear = self.create_client(
                ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        else:
            self._global_costmap_clear = None
            self._local_costmap_clear = None
            self.get_logger().warning(
                'nav2_msgs missing — /api/clear_costmaps will be unavailable')
        self._slam_change_state = self.create_client(
            ChangeState, '/slam_toolbox/change_state')

        # --- Generic remote SetParameters cache (Parts B / E / G2) ---
        # Keyed by node name so we don't recreate clients on every call.
        self._param_clients_lock = threading.Lock()
        self._param_clients = {}   # node_name -> {set, get, list}

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

        try:
            os.makedirs(AUTONEXA_DATA_DIR, exist_ok=True)
        except OSError as exc:
            self.get_logger().warning(f'cannot create {AUTONEXA_DATA_DIR}: {exc}')
        self._load_waypoints()

        self.get_logger().info('MobileBridgeNode started')

    # ---- Callbacks ----

    def _scan_cb(self, msg: LaserScan):
        self._mark_health('/scan')
        pose = self._get_pose()
        points = self._scan_to_points(msg, pose['x_m'], pose['y_m'], pose['yaw_rad'])
        with self._scan_lock:
            self._scan_points = points
            self._scan_stamp = time.time()

    def _map_cb(self, msg: OccupancyGrid):
        self._mark_health('/map')
        png_bytes = self._occupancy_grid_to_png(msg)
        info = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
        }
        # Cheap fingerprint over header + first 4 KB of cells. SLAM-Toolbox
        # restarts always change width/height and reset the cell pattern, so
        # this catches "same physical area, fresh map" as well as relocations.
        try:
            sample = bytes(msg.data[:4096]) if msg.data else b''
        except (TypeError, ValueError):
            sample = b''
        crc = zlib.crc32(sample) & 0xffffffff
        fingerprint = (
            f"w={info['width']},h={info['height']},"
            f"res={info['resolution']:.4f},"
            f"origin={info['origin_x']:.3f},{info['origin_y']:.3f},"
            f"h={crc:08x}"
        )
        with self._map_lock:
            self._map_png = png_bytes
            self._map_info = info
            self._map_version += 1
            self._map_fingerprint = fingerprint

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
        self._mark_health('/odom')
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
        self._mark_health('/pico/joint_feedback')
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

    def _cmd_vel_safe_cb(self, _msg: Twist):
        """Health-only subscription so /api/health can show whether the
        safety chain is alive even when no goal/joystick is active."""
        self._mark_health('/cmd_vel_safe')

    def _plan_cb(self, msg: Path):
        """Cache the latest Nav2 plan as a downsampled list of [x, y]."""
        self._mark_health('/plan')
        # Cap at 100 waypoints to keep /api/plan responses ~1.5 KB even for
        # long plans; for longer plans we stride.
        max_pts = 100
        n = len(msg.poses)
        if n == 0:
            pts = []
        elif n <= max_pts:
            pts = [[round(p.pose.position.x, 4), round(p.pose.position.y, 4)]
                   for p in msg.poses]
        else:
            stride = max(1, n // max_pts)
            pts = [[round(msg.poses[i].pose.position.x, 4),
                    round(msg.poses[i].pose.position.y, 4)]
                   for i in range(0, n, stride)][:max_pts]
            # Always include the final pose so the polyline reaches the goal.
            last = msg.poses[-1].pose.position
            if pts and pts[-1] != [round(last.x, 4), round(last.y, 4)]:
                pts.append([round(last.x, 4), round(last.y, 4)])
        with self._plan_lock:
            self._plan_points = pts
            self._plan_stamp = time.time()

    def _nav_status_cb(self, msg: GoalStatusArray):
        """Track the latest /navigate_to_pose action status."""
        if not msg.status_list:
            return
        # The action server appends new status entries; the most recent one
        # reflects the current goal.
        latest = msg.status_list[-1]
        code_to_str = {
            0: 'UNKNOWN',
            1: 'PLANNING',     # ACCEPTED — server has the goal, planning
            2: 'EXECUTING',
            3: 'CANCELING',
            4: 'SUCCEEDED',
            5: 'CANCELED',
            6: 'ABORTED',
        }
        label = code_to_str.get(latest.status, 'UNKNOWN')
        with self._nav_status_lock:
            self._nav_status = label
            self._nav_status_stamp = time.time()
        # Once a goal terminates, mark the cached goal inactive so the map
        # overlay clears even if the user didn't explicitly cancel.
        if label in ('SUCCEEDED', 'CANCELED', 'ABORTED'):
            with self._goal_lock:
                self._current_goal['active'] = False
            with self._plan_lock:
                self._plan_points = []

    def _goal_pose_cb(self, msg: PoseStamped):
        """Track goals published on /goal_pose (from RViz or this bridge)."""
        yaw = self._quat_to_yaw(msg.pose.orientation)
        with self._goal_lock:
            self._current_goal = {
                'x': float(msg.pose.position.x),
                'y': float(msg.pose.position.y),
                'yaw': float(yaw),
                'active': True,
                'stamp': time.time(),
            }

    def _watchdog_cb(self):
        with self._control_lock:
            if self._last_control_time > 0 and \
               (time.time() - self._last_control_time) > self._control_watchdog_s:
                self._last_control_time = 0.0
                zero = Twist()
                self._cmd_vel_pub.publish(zero)

    def publish_control(self, x: float, y: float, e: int, speed_limit: float):
        """Convert joystick input to Twist and publish.

        Mode-gated: in AUTO/ESTOP we never publish joystick velocities so we
        don't fight Nav2 or the safety latch. In MANUAL the joystick wins.

        Safety-routed: in MANUAL mode, safety_mode='soft' (default) goes
        through the full Nav2 safety chain on /cmd_vel; safety_mode='off'
        publishes to /cmd_vel_manual which nav2_pico_bridge consumes
        directly, bypassing collision_monitor + velocity_smoother.
        """
        max_vx = 0.35
        max_wz = 0.8
        sl = max(0.1, min(1.0, speed_limit))

        with self._mode_lock:
            mode = self._mode

        # Treat any joystick input from a non-MANUAL mode as a no-op. The
        # 50 Hz watchdog will keep zeroing /cmd_vel on the bridge side, and
        # Nav2 (in AUTO) continues unmolested.
        if mode != 'MANUAL':
            return

        twist = Twist()
        if e:
            # Per-message emergency flag — same effect as ESTOP mode but
            # bounded to this single command. Twist stays at zero.
            pass
        else:
            # If Pico E-STOP was previously latched, clear it now
            if self._estop_latched:
                self._clear_pico_estop()
            twist.linear.x = y * max_vx * sl
            # Servo polarity: flipped 2026-05-07 so joystick X aligns with
            # physical wheel direction (push right -> wheels turn right).
            twist.angular.z = x * max_wz * sl

        with self._safety_lock:
            safety = self._safety_mode
        if safety == 'off':
            self._cmd_vel_manual_pub.publish(twist)
        else:
            self._cmd_vel_pub.publish(twist)
        with self._control_lock:
            self._last_control_time = time.time()

    def get_safety_mode(self) -> str:
        with self._safety_lock:
            return self._safety_mode

    def set_safety_mode(self, mode: str) -> bool:
        mode = (mode or '').lower()
        if mode not in ('soft', 'off'):
            return False
        with self._safety_lock:
            if self._safety_mode == mode:
                return True
            self._safety_mode = mode
        # On a soft -> off (or off -> soft) flip, push a zero on whichever
        # topic is being abandoned so the Pico bridge sees a clean stop on
        # that channel and doesn't latch the last value via its 200 ms
        # freshest-wins logic.
        zero = Twist()
        if mode == 'off':
            self._cmd_vel_pub.publish(zero)
        else:
            self._cmd_vel_manual_pub.publish(zero)
        self.get_logger().warning(f'Safety mode -> {mode}')
        return True

    # --- Health stats ---

    def _mark_health(self, topic: str) -> None:
        now = time.time()
        with self._health_lock:
            stat = self._health_stats.get(topic)
            if stat is None:
                return
            dt = now - stat['last_t'] if stat['last_t'] > 0 else 0.0
            if dt > 0:
                inst = 1.0 / dt
                # EWMA, alpha=0.2 — smooths bursts without lagging too far.
                stat['rate'] = 0.8 * stat['rate'] + 0.2 * inst if stat['rate'] > 0 else inst
            stat['last_t'] = now

    def get_health(self) -> list:
        now = time.time()
        out = []
        with self._health_lock:
            for topic, expected_hz, label in HEALTH_TOPICS:
                stat = self._health_stats[topic]
                age = now - stat['last_t'] if stat['last_t'] > 0 else None
                rate = round(stat['rate'], 2)
                # ok if we've seen anything in the last 3x expected period
                # and the rolling rate is at least 50% of expected.
                stale_window = max(1.0, 3.0 / expected_hz)
                ok = (age is not None and age < stale_window
                      and rate >= 0.5 * expected_hz)
                out.append({
                    'topic': topic,
                    'label': label,
                    'expected_hz': expected_hz,
                    'rate_hz': rate,
                    'last_age_s': round(age, 2) if age is not None else None,
                    'ok': ok,
                })
        return out

    # --- Waypoints ---

    def _load_waypoints(self) -> None:
        if not os.path.exists(WAYPOINTS_PATH):
            return
        try:
            with open(WAYPOINTS_PATH, 'r', encoding='utf-8') as fh:
                doc = json.load(fh)
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f'waypoints load failed: {exc}')
            return
        wps = doc.get('waypoints')
        if isinstance(wps, list):
            with self._waypoints_lock:
                self._waypoints = wps
            self.get_logger().info(f'loaded {len(wps)} waypoint(s) from {WAYPOINTS_PATH}')

    def _save_waypoints(self) -> None:
        try:
            os.makedirs(AUTONEXA_DATA_DIR, exist_ok=True)
        except OSError:
            pass
        with self._waypoints_lock:
            doc = {'schema_version': 1, 'waypoints': list(self._waypoints)}
        try:
            tmp = WAYPOINTS_PATH + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh, indent=2)
            os.replace(tmp, WAYPOINTS_PATH)
        except OSError as exc:
            self.get_logger().error(f'waypoints save failed: {exc}')

    def _current_fingerprint(self) -> str:
        with self._map_lock:
            return self._map_fingerprint

    def list_waypoints(self) -> list:
        fp = self._current_fingerprint()
        with self._waypoints_lock:
            wps = list(self._waypoints)
        out = []
        for wp in wps:
            entry = dict(wp)
            entry['stale'] = bool(fp) and entry.get('map_fingerprint') != fp
            out.append(entry)
        return out

    def upsert_waypoint(self, name: str, kind: str, pose: dict) -> dict:
        name = (name or '').strip()
        if not name:
            raise ValueError('name required')
        if kind not in ('park', 'summon', 'home', 'custom'):
            kind = 'custom'
        x = float(pose.get('x', 0.0))
        y = float(pose.get('y', 0.0))
        yaw = float(pose.get('yaw', 0.0))
        entry = {
            'name': name,
            'kind': kind,
            'pose': {'x': x, 'y': y, 'yaw': yaw},
            'map_fingerprint': self._current_fingerprint(),
            'created_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        }
        with self._waypoints_lock:
            self._waypoints = [w for w in self._waypoints if w.get('name') != name]
            self._waypoints.append(entry)
        self._save_waypoints()
        return entry

    def delete_waypoint(self, name: str) -> bool:
        with self._waypoints_lock:
            before = len(self._waypoints)
            self._waypoints = [w for w in self._waypoints if w.get('name') != name]
            removed = len(self._waypoints) < before
        if removed:
            self._save_waypoints()
        return removed

    def navigate_to_waypoint(self, name: str) -> bool:
        with self._waypoints_lock:
            wp = next((w for w in self._waypoints if w.get('name') == name), None)
        if wp is None:
            return False
        pose = wp.get('pose') or {}
        self.send_nav_goal(pose.get('x', 0.0), pose.get('y', 0.0), pose.get('yaw', 0.0))
        return True

    # --- Remote SetParameters / GetParameters / ListParameters ---

    def _param_client(self, node: str, kind: str):
        """Lazy-create and cache a service client for the named node.
        kind in {'set', 'get', 'list'}."""
        with self._param_clients_lock:
            entry = self._param_clients.setdefault(node, {})
            if kind in entry:
                return entry[kind]
            if kind == 'set':
                cli = self.create_client(SetParameters, f'{node}/set_parameters')
            elif kind == 'get':
                cli = self.create_client(GetParameters, f'{node}/get_parameters')
            elif kind == 'list':
                cli = self.create_client(ListParameters, f'{node}/list_parameters')
            else:
                raise ValueError(f'unknown param client kind: {kind}')
            entry[kind] = cli
            return cli

    @staticmethod
    def _to_param_msg(name: str, value):
        """Convert a Python value to an rcl_interfaces Parameter message via
        rclpy's Parameter helper (handles type marshaling)."""
        if isinstance(value, bool):
            p = Parameter(name, Parameter.Type.BOOL, value)
        elif isinstance(value, int):
            p = Parameter(name, Parameter.Type.INTEGER, value)
        elif isinstance(value, float):
            p = Parameter(name, Parameter.Type.DOUBLE, value)
        elif isinstance(value, str):
            p = Parameter(name, Parameter.Type.STRING, value)
        elif isinstance(value, list) and value and all(isinstance(v, float) for v in value):
            p = Parameter(name, Parameter.Type.DOUBLE_ARRAY, [float(v) for v in value])
        elif isinstance(value, list) and value and all(isinstance(v, int) for v in value):
            p = Parameter(name, Parameter.Type.INTEGER_ARRAY, [int(v) for v in value])
        else:
            raise ValueError(f'unsupported param value type for {name}: {type(value).__name__}')
        return p.to_parameter_msg()

    def set_remote_params(self, node: str, items: dict, timeout: float = 1.5) -> dict:
        """Synchronously call SetParameters on `node`. Returns
        {name: {ok, reason}}; per-param results."""
        cli = self._param_client(node, 'set')
        if not cli.wait_for_service(timeout_sec=timeout):
            return {k: {'ok': False, 'reason': f'{node}/set_parameters unavailable'}
                    for k in items}
        try:
            params = [self._to_param_msg(k, v) for k, v in items.items()]
        except ValueError as exc:
            return {k: {'ok': False, 'reason': str(exc)} for k in items}
        req = SetParameters.Request()
        req.parameters = params
        future = cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {k: {'ok': False, 'reason': 'timeout'} for k in items}
        res = future.result()
        out = {}
        for name, r in zip(items.keys(), res.results):
            out[name] = {'ok': bool(r.successful), 'reason': r.reason or ''}
        return out

    def list_remote_params(self, node: str, timeout: float = 1.5) -> list:
        cli = self._param_client(node, 'list')
        if not cli.wait_for_service(timeout_sec=timeout):
            return []
        req = ListParameters.Request()
        future = cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return []
        res = future.result()
        return list(res.result.names)

    def get_remote_params(self, node: str, names: list, timeout: float = 1.5) -> dict:
        cli = self._param_client(node, 'get')
        if not cli.wait_for_service(timeout_sec=timeout):
            return {}
        req = GetParameters.Request()
        req.names = list(names)
        future = cli.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {}
        res = future.result()
        out = {}
        for name, pv in zip(names, res.values):
            out[name] = self._param_value_to_python(pv)
        return out

    @staticmethod
    def _param_value_to_python(pv):
        t = pv.type
        if t == ParameterType.PARAMETER_BOOL:
            return bool(pv.bool_value)
        if t == ParameterType.PARAMETER_INTEGER:
            return int(pv.integer_value)
        if t == ParameterType.PARAMETER_DOUBLE:
            return float(pv.double_value)
        if t == ParameterType.PARAMETER_STRING:
            return str(pv.string_value)
        if t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(pv.integer_array_value)
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(pv.double_array_value)
        if t == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(pv.bool_array_value)
        if t == ParameterType.PARAMETER_STRING_ARRAY:
            return list(pv.string_array_value)
        return None

    # --- Runtime overrides YAML (Parts B / E / G2) ---

    def persist_runtime_overrides(self, node: str, items: dict) -> None:
        """Merge `items` under `node` in runtime_overrides.yaml. Surviving
        on-disk entries for that node are preserved if not overwritten."""
        if yaml is None:
            self.get_logger().warning(
                'PyYAML missing — runtime overrides not persisted to disk')
            return
        try:
            os.makedirs(AUTONEXA_DATA_DIR, exist_ok=True)
        except OSError as exc:
            self.get_logger().warning(f'cannot create {AUTONEXA_DATA_DIR}: {exc}')
            return
        doc = {}
        if os.path.exists(RUNTIME_OVERRIDES_PATH):
            try:
                with open(RUNTIME_OVERRIDES_PATH, 'r', encoding='utf-8') as fh:
                    doc = yaml.safe_load(fh) or {}
            except (OSError, yaml.YAMLError) as exc:
                self.get_logger().warning(
                    f'overrides read failed (will overwrite): {exc}')
                doc = {}
        node_key = node.lstrip('/')
        section = doc.get(node_key) or {}
        section.update(items)
        doc[node_key] = section
        try:
            tmp = RUNTIME_OVERRIDES_PATH + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                yaml.safe_dump(doc, fh, default_flow_style=False)
            os.replace(tmp, RUNTIME_OVERRIDES_PATH)
        except OSError as exc:
            self.get_logger().error(f'overrides write failed: {exc}')

    # --- Costmap clear + SLAM restart (Part C) ---

    def _call_clear_costmap(self, client, timeout: float = 1.0) -> dict:
        if client is None:
            return {'ok': False, 'reason': 'nav2_msgs not installed'}
        if not client.wait_for_service(timeout_sec=timeout):
            return {'ok': False, 'reason': f'{client.srv_name} unavailable'}
        req = ClearEntireCostmap.Request()
        future = client.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {'ok': False, 'reason': 'timeout'}
        return {'ok': True}

    def clear_costmaps(self) -> dict:
        global_res = self._call_clear_costmap(self._global_costmap_clear)
        local_res = self._call_clear_costmap(self._local_costmap_clear)
        return {'global': global_res, 'local': local_res}

    def _slam_change(self, transition_id: int, timeout: float = 2.0) -> dict:
        if not self._slam_change_state.wait_for_service(timeout_sec=timeout):
            return {'ok': False, 'reason': '/slam_toolbox/change_state unavailable'}
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self._slam_change_state.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {'ok': False, 'reason': 'timeout'}
        res = future.result()
        return {'ok': bool(res.success)}

    def restart_mapping(self) -> dict:
        """Cycle SLAM Toolbox through deactivate -> cleanup -> configure ->
        activate. Drops the current map and starts fresh. Costmaps cleared
        as well so the planner doesn't carry stale obstacles into the new
        map frame."""
        steps = []
        for tid, label in (
            (Transition.TRANSITION_DEACTIVATE, 'deactivate'),
            (Transition.TRANSITION_CLEANUP, 'cleanup'),
            (Transition.TRANSITION_CONFIGURE, 'configure'),
            (Transition.TRANSITION_ACTIVATE, 'activate'),
        ):
            r = self._slam_change(tid)
            steps.append({'step': label, **r})
            if not r.get('ok'):
                break
        # Drop costmap obstacles; their map frame is about to change anyway.
        cm = self.clear_costmaps()
        # Bump map_version so the app refetches a blank PNG immediately.
        with self._map_lock:
            self._map_version += 1
            self._map_png = None
        return {'steps': steps, 'costmaps': cm}

    # --- Pose reset / relocalize (Part G1) ---

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        from math import cos as mcos, sin as msin
        msg.pose.pose.orientation.z = msin(float(yaw) / 2.0)
        msg.pose.pose.orientation.w = mcos(float(yaw) / 2.0)
        # Loose covariance — don't fight whatever localizer is listening.
        # x, y: 0.25 m^2; yaw: ~0.07 rad^2 (~15 deg). Order is row-major 6x6.
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.0685
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)
        self.get_logger().info(
            f'initialpose published: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')

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
        """Convert LaserScan to list of (x, y) points in map frame.

        The scan rays are emitted in the laser_link frame. Since 2026-05-07
        laser_link is yaw-rotated by π relative to base_link (LiDAR mounted
        rearward). To get points in the map frame we rotate by the composed
        yaw (laser→base→map) before translating by the robot pose.
        Without this composition, app scan dots end up mirrored across the
        robot vs. the SLAM walls.
        """
        points = []
        angle = scan_msg.angle_min
        yaw_eff = robot_yaw + LASER_LINK_YAW_OFFSET
        cos_y = cos(yaw_eff)
        sin_y = sin(yaw_eff)
        for r in scan_msg.ranges:
            if (not isinf(r) and not isnan(r)
                    and scan_msg.range_min < r < scan_msg.range_max):
                # Point in laser_link frame
                px = r * cos(angle)
                py = r * sin(angle)
                # Rotate by yaw_eff (laser→base→map) and translate to robot pose
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

    def get_map_version(self):
        with self._map_lock:
            return self._map_version

    def get_plan(self):
        with self._plan_lock:
            return list(self._plan_points), self._plan_stamp

    def get_goal(self):
        with self._goal_lock:
            return dict(self._current_goal)

    def get_mode(self):
        with self._mode_lock:
            return self._mode

    def set_mode(self, new_mode: str) -> bool:
        """Transition the control mode. Returns True on a valid transition.

        Side effects on entry:
          MANUAL  — cancel any active Nav2 goal so the BT navigator stops
                    publishing /cmd_vel; clear the Pico E-STOP latch.
          AUTO    — clear the Pico E-STOP latch; do nothing to Nav2 (the
                    user is expected to set a goal next).
          ESTOP   — full estop() — cancel Nav2, zero /cmd_vel, latch Pico.
        """
        new_mode = (new_mode or '').upper()
        if new_mode not in ('AUTO', 'MANUAL', 'ESTOP'):
            return False
        with self._mode_lock:
            if self._mode == new_mode:
                return True
            self._mode = new_mode
            self._mode_stamp = time.time()
        self.get_logger().warning(f'Control mode -> {new_mode}')

        if new_mode == 'MANUAL':
            try:
                self.cancel_nav_goal()
            except Exception as exc:  # pragma: no cover
                self.get_logger().error(f'cancel_nav on mode change: {exc}')
            if self._estop_latched:
                self._clear_pico_estop()
        elif new_mode == 'AUTO':
            if self._estop_latched:
                self._clear_pico_estop()
        elif new_mode == 'ESTOP':
            self.estop()
        return True

    def get_nav_status(self):
        with self._nav_status_lock:
            return self._nav_status, self._nav_status_stamp

    def build_telemetry_snapshot(self) -> dict:
        """Single-shot snapshot bundling everything WS clients want at high
        rate: pose, telemetry, mode, nav status, current goal. Map / scan /
        plan stay on REST since they're either large (PNG) or change slowly."""
        pose, source = self.get_pose()
        nav_status, nav_stamp = self.get_nav_status()
        return {
            'pose': {**pose, 'source': source},
            'telemetry': self.get_telemetry(),
            'mode': self.get_mode(),
            'safety_mode': self.get_safety_mode(),
            'nav_status': nav_status,
            'nav_status_stamp': nav_stamp,
            'goal': self.get_goal(),
            'estop_latched': self._estop_latched,
            'map_fingerprint': self._current_fingerprint(),
            'stamp': time.time(),
        }

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
        """Publish zero velocity, cancel Nav2, and latch the Pico E-STOP.

        E-STOP must abort the active Nav2 goal at every instant — otherwise
        the BT navigator keeps re-publishing /cmd_vel after the latch clears
        and the robot resumes its plan unexpectedly. Cancel first, then zero
        velocity, then latch hardware.
        """
        # 1. Cancel any in-flight Nav2 goal (non-blocking; safe if none).
        try:
            self.cancel_nav_goal()
        except Exception as exc:  # pragma: no cover — defensive
            self.get_logger().error(f'cancel_nav_goal during estop failed: {exc}')

        # 2. Zero velocity (cancel_nav_goal already does this, but repeat in
        #    case the cancel path failed).
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

    def cancel_nav_goal(self):
        """Cancel any in-flight NavigateToPose goal and stop motion.

        Strategy:
          1. Publish a zero Twist on /cmd_vel so the controller chain
             halts immediately even if the cancel service round-trip
             is slow or unavailable.
          2. Ask the BT navigator to cancel all active goals via the
             action cancel service (graceful teardown of the plan).
        """
        # Immediate safety: zero velocity flows through the same safety
        # chain Nav2 uses (velocity_smoother -> collision_monitor ->
        # cmd_vel_to_pico_bridge). Guarantees wheels stop even if the
        # cancel service isn't live.
        zero = Twist()
        self._cmd_vel_pub.publish(zero)
        with self._control_lock:
            self._last_control_time = 0.0

        # Mark the cached goal as inactive so the app stops drawing the
        # goal marker / planner path immediately, regardless of whether the
        # action server's cancel ack arrives.
        with self._goal_lock:
            self._current_goal['active'] = False
        with self._plan_lock:
            self._plan_points = []
            self._plan_stamp = time.time()

        # Graceful cancel via action cancel service (non-blocking).
        if not self._nav_cancel_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning(
                'Nav2 cancel service not available — zero cmd_vel sent only')
            return

        # Empty CancelGoal.Request with zero UUID + zero stamp means
        # "cancel everything".
        req = CancelGoal.Request()
        future = self._nav_cancel_client.call_async(req)

        def _cancel_done(fut):
            try:
                res = fut.result()
                if res is None:
                    self.get_logger().error('Nav2 cancel returned None')
                    return
                # return_code: 0=NONE, 1=REJECTED, 2=UNKNOWN_GOAL_ID,
                # 3=GOAL_TERMINATED. 0 is success.
                self.get_logger().info(
                    f'Nav2 cancel: return_code={res.return_code}, '
                    f'cancelled={len(res.goals_canceling)}')
            except Exception as exc:
                self.get_logger().error(f'Nav2 cancel failed: {exc}')

        future.add_done_callback(_cancel_done)


# =============================================================================
#  Flask HTTP Server
# =============================================================================

flask_app = Flask(__name__)
sock = Sock(flask_app)
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
    info = dict(info)
    info['fingerprint'] = bridge_node._current_fingerprint()
    return jsonify(info)


@flask_app.route('/api/map_version')
def api_map_version():
    """Lightweight (~30 B) version counter — clients only refetch /api/map
    when this value changes. Lets the app poll cheaply at 0.5–1 Hz without
    burning bandwidth on the (slow-changing) PNG."""
    return jsonify({'v': bridge_node.get_map_version()})


@flask_app.route('/api/plan')
def api_plan():
    points, stamp = bridge_node.get_plan()
    return jsonify({
        'points': points,
        'stamp': stamp,
        'count': len(points),
    })


@flask_app.route('/api/goal')
def api_goal():
    return jsonify(bridge_node.get_goal())


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


@flask_app.route('/api/cancel_nav', methods=['POST'])
def api_cancel_nav():
    bridge_node.cancel_nav_goal()
    return jsonify({'status': 'cancelled'})


@flask_app.route('/api/mode', methods=['GET', 'POST'])
def api_mode():
    """Get or set the control mode (AUTO / MANUAL / ESTOP)."""
    if request.method == 'GET':
        return jsonify({'mode': bridge_node.get_mode()})
    data = request.get_json(silent=True) or {}
    requested = data.get('mode', '')
    if not bridge_node.set_mode(requested):
        return jsonify({'error': f'invalid mode: {requested!r}'}), 400
    return jsonify({'mode': bridge_node.get_mode()})


@flask_app.route('/api/nav_status')
def api_nav_status():
    label, stamp = bridge_node.get_nav_status()
    return jsonify({'status': label, 'stamp': stamp})


# ---------------------------------------------------------------------------
#  Part A — Manual safety bypass
# ---------------------------------------------------------------------------

@flask_app.route('/api/safety_mode', methods=['GET', 'POST'])
def api_safety_mode():
    if request.method == 'GET':
        return jsonify({'safety_mode': bridge_node.get_safety_mode()})
    data = request.get_json(silent=True) or {}
    requested = data.get('safety_mode', '')
    if not bridge_node.set_safety_mode(requested):
        return jsonify({'error': f'invalid safety_mode: {requested!r}'}), 400
    return jsonify({'safety_mode': bridge_node.get_safety_mode()})


# ---------------------------------------------------------------------------
#  Part B — Direction calibration (vx_polarity / servo_polarity)
# ---------------------------------------------------------------------------

@flask_app.route('/api/calibrate_direction', methods=['GET', 'POST'])
def api_calibrate_direction():
    """GET: report current bridge polarity values.
    POST: {vx_polarity?: ±1, servo_polarity?: ±1} — apply via SetParameters
    on /nav2_pico_bridge AND persist to runtime_overrides.yaml so the values
    survive a relaunch."""
    if request.method == 'GET':
        vals = bridge_node.get_remote_params(
            '/nav2_pico_bridge', ['vx_polarity', 'servo_polarity'])
        return jsonify(vals)
    data = request.get_json(silent=True) or {}
    items = {}
    for key in ('vx_polarity', 'servo_polarity'):
        if key in data:
            try:
                v = int(data[key])
            except (TypeError, ValueError):
                return jsonify({'error': f'{key} must be ±1'}), 400
            if v not in (-1, 1):
                return jsonify({'error': f'{key} must be ±1'}), 400
            items[key] = v
    if not items:
        return jsonify({'error': 'provide vx_polarity and/or servo_polarity'}), 400
    results = bridge_node.set_remote_params('/nav2_pico_bridge', items)
    persisted = {k: v for k, v in items.items() if results.get(k, {}).get('ok')}
    if persisted:
        bridge_node.persist_runtime_overrides('nav2_pico_bridge', persisted)
    return jsonify({'results': results, 'persisted': list(persisted.keys())})


# ---------------------------------------------------------------------------
#  Part C — Map / Nav2 reset
# ---------------------------------------------------------------------------

@flask_app.route('/api/clear_costmaps', methods=['POST'])
def api_clear_costmaps():
    return jsonify(bridge_node.clear_costmaps())


@flask_app.route('/api/restart_mapping', methods=['POST'])
def api_restart_mapping():
    return jsonify(bridge_node.restart_mapping())


# ---------------------------------------------------------------------------
#  Part D — Manual waypoints
# ---------------------------------------------------------------------------

@flask_app.route('/api/waypoints', methods=['GET', 'POST'])
def api_waypoints():
    if request.method == 'GET':
        return jsonify({'waypoints': bridge_node.list_waypoints(),
                        'fingerprint': bridge_node._current_fingerprint()})
    data = request.get_json(silent=True) or {}
    name = data.get('name', '')
    kind = data.get('kind', 'custom')
    pose = data.get('pose') or {}
    try:
        entry = bridge_node.upsert_waypoint(name, kind, pose)
    except ValueError as exc:
        return jsonify({'error': str(exc)}), 400
    return jsonify(entry)


@flask_app.route('/api/waypoints/<name>', methods=['DELETE'])
def api_waypoint_delete(name):
    if not bridge_node.delete_waypoint(name):
        return jsonify({'error': 'not found'}), 404
    return jsonify({'status': 'deleted', 'name': name})


@flask_app.route('/api/waypoints/<name>/navigate', methods=['POST'])
def api_waypoint_navigate(name):
    if not bridge_node.navigate_to_waypoint(name):
        return jsonify({'error': 'not found'}), 404
    return jsonify({'status': 'ok', 'name': name})


# ---------------------------------------------------------------------------
#  Phase 2 — Parking-plan API aliases over the map-frame waypoint DB
# ---------------------------------------------------------------------------

def _spot_from_waypoint(wp: dict) -> dict:
    pose = dict(wp.get('pose') or {})
    name = str(wp.get('name') or '')
    return {
        'id': name,
        'name': name,
        'kind': 'map_static',
        'stale': bool(wp.get('stale')),
        'map_fingerprint': wp.get('map_fingerprint', ''),
        # Phase 2 intentionally navigates to this staging pose only. Later
        # phases add final_pose / ArUco docking precision.
        'staging_pose': pose,
        'final_pose': pose,
        'created_at': wp.get('created_at', ''),
    }


@flask_app.route('/api/spots', methods=['GET', 'POST'])
def api_spots():
    """List or create map-frame parking spots.

    This is the parking_plan.md API surface, backed by the existing persistent
    waypoint store so the current app and the new SpotsTab can share data.
    """
    if request.method == 'GET':
        spots = [
            _spot_from_waypoint(wp)
            for wp in bridge_node.list_waypoints()
            if wp.get('kind') == 'park'
        ]
        return jsonify({
            'spots': spots,
            'fingerprint': bridge_node._current_fingerprint(),
        })

    data = request.get_json(silent=True) or {}
    spot_id = (data.get('id') or data.get('name') or '').strip()
    pose = data.get('staging_pose') or data.get('final_pose') or data.get('pose') or {}
    try:
        entry = bridge_node.upsert_waypoint(spot_id, 'park', pose)
    except (TypeError, ValueError) as exc:
        return jsonify({'error': str(exc)}), 400
    return jsonify(_spot_from_waypoint(entry))


@flask_app.route('/api/spots/<spot_id>', methods=['DELETE'])
def api_spot_delete(spot_id):
    if not bridge_node.delete_waypoint(spot_id):
        return jsonify({'error': 'not found'}), 404
    return jsonify({'status': 'deleted', 'id': spot_id})


@flask_app.route('/api/save_spot', methods=['POST'])
def api_save_spot():
    """Capture the current map pose as a static parking spot."""
    data = request.get_json(silent=True) or {}
    spot_id = (data.get('id') or data.get('name') or '').strip()
    if not spot_id:
        return jsonify({'error': 'id required'}), 400
    pose, source = bridge_node.get_pose()
    if source == 'none' or pose.get('stamp', 0.0) <= 0:
        return jsonify({'error': 'no robot pose available'}), 503
    entry = bridge_node.upsert_waypoint(
        spot_id,
        'park',
        {'x': pose['x_m'], 'y': pose['y_m'], 'yaw': pose['yaw_rad']},
    )
    return jsonify(_spot_from_waypoint(entry))


@flask_app.route('/api/park_at', methods=['POST'])
def api_park_at():
    """Phase 2 park behavior: NavigateToPose to the spot staging pose."""
    data = request.get_json(silent=True) or {}
    spot_id = (data.get('id') or data.get('name') or '').strip()
    if not spot_id:
        return jsonify({'error': 'id required'}), 400
    if not bridge_node.navigate_to_waypoint(spot_id):
        return jsonify({'error': 'not found'}), 404
    return jsonify({'status': 'ok', 'id': spot_id, 'mode': 'staging_only'})


@flask_app.route('/api/summon', methods=['POST'])
def api_summon():
    """Phase 2 summon behavior: NavigateToPose to a supplied pickup pose."""
    data = request.get_json(silent=True) or {}
    if 'id' in data or 'name' in data:
        spot_id = (data.get('id') or data.get('name') or '').strip()
        if not bridge_node.navigate_to_waypoint(spot_id):
            return jsonify({'error': 'not found'}), 404
        return jsonify({'status': 'ok', 'id': spot_id})
    if 'x' not in data or 'y' not in data:
        return jsonify({'error': 'x and y required'}), 400
    bridge_node.send_nav_goal(data['x'], data['y'], data.get('yaw', 0.0))
    return jsonify({'status': 'ok'})


@flask_app.route('/api/lock_map', methods=['POST'])
def api_lock_map():
    """Persist the current SLAM map with nav2_map_server's map_saver_cli."""
    os.makedirs(MAPS_DIR, exist_ok=True)
    prefix = os.path.join(MAPS_DIR, f"garage_{time.strftime('%Y%m%d_%H%M%S')}")
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix]
    try:
        proc = subprocess.run(
            cmd, check=False, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return jsonify({'error': str(exc), 'command': cmd}), 500
    if proc.returncode != 0:
        return jsonify({
            'error': 'map_saver_cli failed',
            'returncode': proc.returncode,
            'stdout': proc.stdout[-2000:],
            'stderr': proc.stderr[-2000:],
            'command': cmd,
        }), 500
    return jsonify({
        'status': 'ok',
        'map_prefix': prefix,
        'yaml': prefix + '.yaml',
        'pgm': prefix + '.pgm',
        'stdout': proc.stdout[-2000:],
    })


# ---------------------------------------------------------------------------
#  Part E — Nav2 max linear speed
# ---------------------------------------------------------------------------

@flask_app.route('/api/nav2_speed', methods=['GET', 'POST'])
def api_nav2_speed():
    """Live-tune Nav2's linear speed cap. Sets RPP
    FollowPath.desired_linear_vel and velocity_smoother max_velocity[0] in
    lockstep so the smoother doesn't override the controller. Persists to
    runtime_overrides.yaml under the respective node sections."""
    if request.method == 'GET':
        ctrl = bridge_node.get_remote_params(
            '/controller_server', ['FollowPath.desired_linear_vel'])
        sm = bridge_node.get_remote_params(
            '/velocity_smoother', ['max_velocity'])
        desired = ctrl.get('FollowPath.desired_linear_vel')
        return jsonify({
            'controller_desired_linear_vel': desired,
            # Backward-compatible field name for the current Flutter client.
            'controller_max_vel_x': desired,
            'smoother_max_velocity': sm.get('max_velocity'),
        })
    data = request.get_json(silent=True) or {}
    if 'max_vel_x' not in data:
        return jsonify({'error': 'max_vel_x required'}), 400
    try:
        target = float(data['max_vel_x'])
    except (TypeError, ValueError):
        return jsonify({'error': 'max_vel_x must be a number'}), 400
    if not 0.05 <= target <= 0.50:
        return jsonify({'error': 'max_vel_x must be in [0.05, 0.50]'}), 400
    ctrl = bridge_node.set_remote_params(
        '/controller_server', {'FollowPath.desired_linear_vel': target})
    # velocity_smoother expects a 3-vector [vx, vy, wz]; preserve current vy/wz.
    cur = bridge_node.get_remote_params('/velocity_smoother', ['max_velocity'])
    vec = list(cur.get('max_velocity') or [target, 0.0, 0.5])
    vec[0] = target
    sm = bridge_node.set_remote_params(
        '/velocity_smoother', {'max_velocity': [float(v) for v in vec]})
    bridge_node.persist_runtime_overrides(
        'controller_server', {'FollowPath.desired_linear_vel': target})
    bridge_node.persist_runtime_overrides(
        'velocity_smoother', {'max_velocity': [float(v) for v in vec]})
    return jsonify({'controller': ctrl, 'smoother': sm,
                    'controller_desired_linear_vel': target,
                    'controller_max_vel_x': target,
                    'smoother_max_velocity': vec})


# ---------------------------------------------------------------------------
#  Part G1 — Pose reset / relocalize
# ---------------------------------------------------------------------------

@flask_app.route('/api/relocalize', methods=['POST'])
def api_relocalize():
    data = request.get_json(silent=True) or {}
    if 'x' not in data or 'y' not in data:
        return jsonify({'error': 'x, y required'}), 400
    yaw = data.get('yaw', 0.0)
    bridge_node.publish_initial_pose(data['x'], data['y'], yaw)
    return jsonify({'status': 'ok'})


# ---------------------------------------------------------------------------
#  Part G2 — Live param tuner
# ---------------------------------------------------------------------------

@flask_app.route('/api/params', methods=['GET', 'POST'])
def api_params():
    if request.method == 'GET':
        node = request.args.get('node', '').strip()
        if node not in PARAM_TUNER_WHITELIST:
            return jsonify({'error': 'node not in whitelist',
                            'whitelist': list(PARAM_TUNER_WHITELIST)}), 400
        names = bridge_node.list_remote_params(node)
        if not names:
            return jsonify({'node': node, 'params': {}, 'names': []})
        # Cap at 200 names for the GetParameters round trip — bigger nodes
        # still report the full list under 'names' for client-side filtering.
        sample = names[:200]
        values = bridge_node.get_remote_params(node, sample)
        return jsonify({'node': node, 'names': names, 'params': values})
    data = request.get_json(silent=True) or {}
    node = data.get('node', '').strip()
    if node not in PARAM_TUNER_WHITELIST:
        return jsonify({'error': 'node not in whitelist',
                        'whitelist': list(PARAM_TUNER_WHITELIST)}), 400
    items = data.get('params') or {}
    if not isinstance(items, dict) or not items:
        return jsonify({'error': 'params object required'}), 400
    results = bridge_node.set_remote_params(node, items)
    bridge_node.persist_runtime_overrides(
        node.lstrip('/'),
        {k: v for k, v in items.items() if results.get(k, {}).get('ok')})
    return jsonify({'node': node, 'results': results})


# ---------------------------------------------------------------------------
#  Part G3 — Topic / node health
# ---------------------------------------------------------------------------

@flask_app.route('/api/health')
def api_health():
    return jsonify({'topics': bridge_node.get_health()})


# =============================================================================
#  WebSocket endpoints — joystick (high-rate) + telemetry push
# =============================================================================

@sock.route('/ws/control')
def ws_control(ws):
    """Bidirectional joystick stream. Each inbound JSON message
    {x, y, e, speed_limit} is fed straight into publish_control. We don't
    push anything back on this socket — it's input-only — but we keep the
    connection open as long as the client wants.

    The 50 Hz Pico watchdog and bridge-side 200 ms timeout mean that if
    the WebSocket dies mid-drive, motors zero on their own."""
    bridge_node.get_logger().info('WS /ws/control connected')
    try:
        while True:
            raw = ws.receive(timeout=5)
            if raw is None:
                # Idle keepalive — do nothing.
                continue
            try:
                data = json.loads(raw)
            except (TypeError, ValueError):
                continue
            x = float(data.get('x', 0))
            y = float(data.get('y', 0))
            e = int(data.get('e', 0))
            sl = float(data.get('speed_limit', 1.0))
            bridge_node.publish_control(x, y, e, sl)
    except Exception as exc:
        bridge_node.get_logger().info(f'WS /ws/control closed: {exc}')


@sock.route('/ws/telemetry')
def ws_telemetry(ws):
    """Server-push telemetry stream. We register the connection in the
    bridge node's pusher set; a background thread fans out snapshots at
    10 Hz to every connected client. When the client disconnects (any send
    raises), the thread evicts it from the set."""
    with bridge_node._ws_lock:
        bridge_node._ws_telemetry_clients.add(ws)
    bridge_node.get_logger().info(
        f'WS /ws/telemetry connected ({len(bridge_node._ws_telemetry_clients)} total)')
    # Send an immediate snapshot so the client doesn't have to wait for the
    # next push tick to populate its UI.
    try:
        ws.send(json.dumps(bridge_node.build_telemetry_snapshot()))
    except Exception:
        pass
    try:
        while True:
            # Block on receive so flask-sock keeps the socket alive. We
            # don't expect any inbound traffic, but this also lets us
            # detect client-initiated disconnects promptly.
            msg = ws.receive(timeout=10)
            if msg is None:
                continue
    except Exception:
        pass
    finally:
        with bridge_node._ws_lock:
            bridge_node._ws_telemetry_clients.discard(ws)
        bridge_node.get_logger().info('WS /ws/telemetry disconnected')


def _telemetry_pusher_loop():
    """Background thread: push a telemetry snapshot to every connected WS
    client at 10 Hz. Clients that fail to send are evicted on the next pass."""
    period = 0.1  # 10 Hz
    while True:
        time.sleep(period)
        if bridge_node is None:
            continue
        with bridge_node._ws_lock:
            clients = list(bridge_node._ws_telemetry_clients)
        if not clients:
            continue
        try:
            payload = json.dumps(bridge_node.build_telemetry_snapshot())
        except Exception:
            continue
        dead = []
        for ws in clients:
            try:
                ws.send(payload)
            except Exception:
                dead.append(ws)
        if dead:
            with bridge_node._ws_lock:
                for ws in dead:
                    bridge_node._ws_telemetry_clients.discard(ws)


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
<li>POST /api/nav_goal — send Nav2 goal (JSON: x, y, yaw)</li>
<li>POST /api/cancel_nav — cancel active Nav2 goal</li>
<li>GET/POST /api/mode — read or set AUTO/MANUAL/ESTOP</li>
<li>GET/POST /api/safety_mode — soft (default) | off (manual bypass)</li>
<li><a href="/api/nav_status">/api/nav_status</a> — Nav2 action status</li>
<li>GET/POST /api/calibrate_direction — vx_polarity / servo_polarity</li>
<li>POST /api/clear_costmaps · POST /api/restart_mapping</li>
<li>GET/POST/DELETE /api/waypoints — manual park/summon spots</li>
<li>POST /api/waypoints/&lt;name&gt;/navigate</li>
<li>GET/POST /api/spots · DELETE /api/spots/&lt;id&gt; — parking-plan static spots</li>
<li>POST /api/save_spot · POST /api/park_at · POST /api/summon</li>
<li>POST /api/lock_map — save current SLAM map under ~/.autonexa/maps</li>
<li>GET/POST /api/nav2_speed — live Nav2 target-speed slider</li>
<li>POST /api/relocalize — set robot pose (x,y,yaw)</li>
<li>GET/POST /api/params?node=&lt;name&gt; — live param tuner (whitelist only)</li>
<li><a href="/api/health">/api/health</a> — topic rates / staleness</li>
<li>WS /ws/control — joystick stream (input-only)</li>
<li>WS /ws/telemetry — server-pushed telemetry snapshots @ 10 Hz</li>
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

    # 10 Hz telemetry fan-out for /ws/telemetry subscribers
    pusher_thread = threading.Thread(target=_telemetry_pusher_loop, daemon=True)
    pusher_thread.start()

    bridge_node.get_logger().info('Starting Flask server on http://0.0.0.0:5000')
    flask_app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
