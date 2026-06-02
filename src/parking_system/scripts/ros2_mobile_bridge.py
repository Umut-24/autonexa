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
import queue
import re
import shutil
import signal
import subprocess
import threading
import time
import io
import uuid
import hashlib
from collections import deque
from math import cos, sin, isinf, isnan, hypot, ceil, pi as MATH_PI

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
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Empty
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
try:
    from slam_toolbox.srv import Reset as SlamReset
except ImportError:
    SlamReset = None
from flask import Flask, Response, jsonify, request, send_file, send_from_directory
from flask_sock import Sock

try:
    import yaml
except ImportError:
    yaml = None

try:
    from parking_system import build_urdf as _build_urdf
except Exception:  # pragma: no cover
    _build_urdf = None

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None

# Persistent app data (waypoints + runtime overrides). Lives outside the ROS
# workspace so it survives `colcon build --symlink-install` clobbering and is
# trivial to back up / hand-edit.
AUTONEXA_DATA_DIR = os.path.expanduser('~/.autonexa')
RUNTIME_OVERRIDES_PATH = os.path.join(AUTONEXA_DATA_DIR, 'runtime_overrides.yaml')
WAYPOINTS_PATH = os.path.join(AUTONEXA_DATA_DIR, 'waypoints.json')
MAPS_DIR = os.path.join(AUTONEXA_DATA_DIR, 'maps')
PLANNER_MODE_PATH = os.path.join(AUTONEXA_DATA_DIR, 'planner_mode.txt')
RELOCALIZE_AUTO_PATH = os.path.join(AUTONEXA_DATA_DIR, 'relocalize_auto.json')
# Launch-time flag for encoder->EKF odometry fusion. Read by the launch files
# (use_ekf arg default); written here. Toggling requires a relaunch.
USE_EKF_PATH = os.path.join(AUTONEXA_DATA_DIR, 'use_ekf.txt')


def _resolve_desktop_web_dir() -> str:
    """Find the static desktop console directory in install or source trees."""
    candidates = []
    if get_package_share_directory is not None:
        try:
            candidates.append(
                os.path.join(
                    get_package_share_directory('parking_system'),
                    'web_desktop',
                )
            )
        except Exception:
            pass
    candidates.append(
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), os.pardir, 'web_desktop')
        )
    )
    for path in candidates:
        if os.path.isdir(path):
            return path
    return candidates[-1]


DESKTOP_WEB_DIR = _resolve_desktop_web_dir()

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

# Only these nav2_pico_bridge params are physical CALIBRATION and may be
# persisted to runtime_overrides.yaml (the bridge replays just these on
# startup). Everything else on the bridge (speed/accel caps, creep gate,
# steer slew, manual window) is PC-config-authoritative: the app can change
# it live for the session, but it is NOT written to disk, so it can't
# silently override the PC config on the next launch. Mirror of
# nav2_pico_bridge.RUNTIME_CALIBRATION — keep the two in sync.
BRIDGE_CALIBRATION_KEYS = (
    'vx_polarity',
    'servo_polarity',
    'reverse_steer_polarity',
    'servo_center_us',
    'servo_us_min',
    'servo_us_max',
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

# Guarded desktop command console. The browser can request only these named
# profiles; no raw shell text is accepted, and every external command is run
# with shell=False after argument validation. Motion-producing commands are
# intentionally absent so all driving remains mode-gated through /ws/control.
COMMAND_TOPIC_RE = re.compile(r'^/[A-Za-z0-9_][A-Za-z0-9_/]*$')

COMMAND_PROFILES = {
    'ros_node_list': {
        'group': 'ROS graph',
        'label': 'ROS nodes',
        'description': 'List visible ROS nodes.',
        'argv': ['ros2', 'node', 'list'],
        'max_runtime_s': 8,
        'args': [],
    },
    'ros_topic_list': {
        'group': 'ROS graph',
        'label': 'ROS topics',
        'description': 'List visible ROS topics.',
        'argv': ['ros2', 'topic', 'list'],
        'max_runtime_s': 8,
        'args': [],
    },
    'ros_service_list': {
        'group': 'ROS graph',
        'label': 'ROS services',
        'description': 'List visible ROS services.',
        'argv': ['ros2', 'service', 'list'],
        'max_runtime_s': 8,
        'args': [],
    },
    'topic_info': {
        'group': 'Topic tools',
        'label': 'Topic info',
        'description': 'Show publishers, subscribers, and type for one topic.',
        'argv': ['ros2', 'topic', 'info', '{topic}'],
        'max_runtime_s': 8,
        'args': [
            {'name': 'topic', 'label': 'Topic', 'type': 'topic',
             'default': '/scan'},
        ],
    },
    'topic_hz': {
        'group': 'Topic tools',
        'label': 'Topic hz',
        'description': 'Measure a topic rate for a bounded time window.',
        'argv': ['ros2', 'topic', 'hz', '{topic}'],
        'max_runtime_s': 12,
        'runtime_arg': 'duration_s',
        'args': [
            {'name': 'topic', 'label': 'Topic', 'type': 'topic',
             'default': '/scan'},
            {'name': 'duration_s', 'label': 'Seconds', 'type': 'int',
             'min': 4, 'max': 30, 'default': 8},
        ],
    },
    'diagnose_scan_quality': {
        'group': 'Diagnostics',
        'label': 'Scan quality',
        'description': 'Run parking_system scan-quality diagnostics.',
        'argv': ['ros2', 'run', 'parking_system', 'diagnose_scan_quality.py'],
        'max_runtime_s': 20,
        'args': [],
    },
    'diagnose_localization': {
        'group': 'Diagnostics',
        'label': 'Localization',
        'description': 'Run parking_system localization diagnostics.',
        'argv': ['ros2', 'run', 'parking_system', 'diagnose_localization.py'],
        'max_runtime_s': 20,
        'args': [],
    },
    'diagnose_tf_tree': {
        'group': 'Diagnostics',
        'label': 'TF tree',
        'description': 'Run parking_system TF-tree diagnostics.',
        'argv': ['ros2', 'run', 'parking_system', 'diagnose_tf_tree.py'],
        'max_runtime_s': 20,
        'args': [],
    },
    'diagnose_control_chain': {
        'group': 'Diagnostics',
        'label': 'Control chain',
        'description': 'Run parking_system control-chain diagnostics.',
        'argv': ['ros2', 'run', 'parking_system', 'diagnose_control_chain.py'],
        'max_runtime_s': 30,
        'args': [],
    },
    'print_robot_position': {
        'group': 'Diagnostics',
        'label': 'Robot position',
        'description': 'Print the current map-frame robot pose.',
        'argv': ['ros2', 'run', 'parking_system', 'print_robot_position.py'],
        'max_runtime_s': 12,
        'args': [],
    },
    'ros_log_tail': {
        'group': 'Logs',
        'label': 'ROS latest log',
        'description': 'Tail the newest log file under ~/.ros/log/latest.',
        'kind': 'log_tail',
        'max_runtime_s': 5,
        'args': [
            {'name': 'lines', 'label': 'Lines', 'type': 'int',
             'min': 20, 'max': 500, 'default': 120},
        ],
    },
}


def _now_stamp() -> float:
    return round(time.time(), 3)


def _validate_command_arg(spec: dict, raw):
    arg_type = spec.get('type')
    name = spec.get('name', 'arg')
    if raw is None or raw == '':
        raw = spec.get('default')
    if arg_type == 'topic':
        value = str(raw or '').strip()
        if not COMMAND_TOPIC_RE.fullmatch(value):
            raise ValueError(f'{name} must be an absolute ROS topic name')
        return value
    if arg_type == 'int':
        try:
            value = int(raw)
        except (TypeError, ValueError):
            raise ValueError(f'{name} must be an integer')
        min_v = int(spec.get('min', value))
        max_v = int(spec.get('max', value))
        if value < min_v or value > max_v:
            raise ValueError(f'{name} must be in [{min_v}, {max_v}]')
        return value
    raise ValueError(f'unsupported arg type for {name}')


def _build_command(profile_id: str, args: dict):
    profile = COMMAND_PROFILES.get(profile_id)
    if profile is None:
        raise ValueError(f'unknown profile_id: {profile_id!r}')
    args = args or {}
    values = {}
    for spec in profile.get('args', []):
        name = spec['name']
        values[name] = _validate_command_arg(spec, args.get(name))
    runtime = float(profile.get('max_runtime_s', 10))
    runtime_arg = profile.get('runtime_arg')
    if runtime_arg in values:
        runtime = min(runtime, float(values[runtime_arg]) + 1.0)
    kind = profile.get('kind', 'external')
    if kind == 'log_tail':
        return profile, None, values, runtime
    argv = []
    for item in profile.get('argv', []):
        text = str(item)
        for key, value in values.items():
            text = text.replace('{' + key + '}', str(value))
        argv.append(text)
    if not argv:
        raise ValueError('profile has no command')
    return profile, argv, values, runtime


def _public_command_profiles():
    profiles = []
    for profile_id, profile in COMMAND_PROFILES.items():
        profiles.append({
            'id': profile_id,
            'group': profile.get('group', ''),
            'label': profile.get('label', profile_id),
            'description': profile.get('description', ''),
            'args': profile.get('args', []),
            'max_runtime_s': profile.get('max_runtime_s', 10),
            'kind': profile.get('kind', 'external'),
        })
    return profiles

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
        self._bridge_start_time = time.time()
        self.declare_parameter('active_map_yaml', '')
        try:
            _active_map_yaml_param = str(
                self.get_parameter('active_map_yaml').value or '').strip()
        except Exception:
            _active_map_yaml_param = ''

        # --- Stored data (thread-safe via locks) ---
        self._scan_lock = threading.Lock()
        self._scan_points = []       # List of [x, y] in map frame
        self._scan_stamp = 0.0

        self._map_lock = threading.Lock()
        self._map_png = None         # Cached PNG bytes
        self._map_info = {}          # {width, height, resolution, origin_x, origin_y}
        self._map_version = 0        # Increments on each /map update; clients poll
                                     # /api/map_version to skip redundant PNG fetches.
        # Raw occupancy grid as a (H, W) int8 numpy array (-1/0..100 per cell).
        # Used by _nudge_goal_from_walls to pre-shift goals away from static-map
        # walls. Kept under the same _map_lock as the PNG so updates are atomic.
        self._map_grid = None

        # --- Nav2 plan + current goal ---
        self._plan_lock = threading.Lock()
        self._plan_points = []       # Downsampled [[x, y], ...] in map frame
        self._plan_stamp = 0.0

        # --- Near-wall safety state (continuous monitor) -----------------
        # While a goal is active a background monitor watches the robot's
        # distance to the nearest LiDAR obstacle. Near a wall it engages a
        # full bypass (collision_monitor skipped + inflation relaxed) so
        # escape and final-park paths are not blocked; clear of walls it
        # restores collision_monitor + normal inflation. Hysteresis between
        # the enter/exit thresholds keeps it from flapping.
        self._near_wall_lock = threading.Lock()
        self._near_wall_engaged = False      # full bypass currently engaged?
        self._inflation_relaxed = False      # inflation at the escape value?
        # Default fallbacks match the current nav2_navigation_params.yaml
        # (global 0.22 / local 0.18); the live values are captured before
        # each relax so the exact pre-relax halo is put back.
        self._saved_inflation = {'global': 0.22, 'local': 0.18}
        self._inflation_escape_radius = 0.015
        self._near_wall_enter_m = 0.35       # engage when an obstacle is closer
        self._near_wall_exit_m = 0.55        # release when all obstacles farther

        # --- Planner mode + multi-point decompose-on-failure (item 2) ----
        # 'standard'  — single SMAC goal; on ABORT, one same-goal auto-retry.
        # 'multipoint'— on ABORT, stage the goal: drive to an intermediate
        #               open-space waypoint, then re-issue the final goal.
        self._planner_mode_lock = threading.Lock()
        self._planner_mode = self._load_planner_mode()
        self._mp_lock = threading.Lock()
        self._mp_final_goal = None           # {'x','y','yaw'} the user's goal
        self._mp_stage = 'idle'              # 'idle' | 'waypoint' | 'final'
        self._mp_decompose_count = 0
        self._mp_decompose_max = 2

        # --- AMCL periodic relocalize (localization mode) ----------------
        # Periodically forces an AMCL filter update against the latest scan
        # (request_nomotion_update) to counter the slow pose drift seen on
        # featureless walls. Localization-mode only — a silent no-op in
        # live-SLAM mode (no /amcl). Persisted across relaunches; default OFF.
        self._relocalize_auto_lock = threading.Lock()
        _ra = self._load_relocalize_auto()
        self._relocalize_auto_enabled = _ra['enabled']
        self._relocalize_auto_interval = _ra['interval_s']
        self._relocalize_last_t = 0.0
        self._relocalize_fut = None          # keep last call_async future alive

        self._goal_lock = threading.Lock()
        self._current_goal = {       # Last goal sent (active until cancel)
            'x': 0.0, 'y': 0.0, 'yaw': 0.0,
            'active': False, 'stamp': 0.0,
        }

        self._pose_lock = threading.Lock()
        self._pose = {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0, 'stamp': 0.0}
        self._pose_source = 'none'   # 'amcl', 'odom', 'none'

        # Map-frame pose comes from the map->base_link TF (AMCL/SLAM map->odom
        # composed with the scan-matcher/EKF odom->base_link). This is always
        # correct in the map frame and tracks continuously, even when AMCL is
        # idle (stationary) and stops republishing /amcl_pose. Raw /odom is an
        # odom-frame pose and is only a last-resort fallback for manual /
        # bridge-only mode where there is no map frame at all — see _odom_cb.
        self._map_frame = 'map'
        self._base_frame = 'base_link'
        self._last_tf_ok = 0.0       # wall-clock of the last good map->base_link
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

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

        # --- nav2_pico_bridge nav-bypass mirror ------------------------------
        # When Nav2 enters EXECUTING we flip /nav2_pico_bridge:nav_bypass_active
        # = True so the chassis follows /cmd_vel_smoothed (pre-collision_monitor)
        # and only an operator E-STOP halts it. Cleared on goal terminal
        # transitions. Mirror kept here purely for /api/status reporting; the
        # source of truth is the bridge parameter we set via SetParameters.
        self._nav_bypass_active = False
        # Track the previous status so _nav_status_cb fires SetParameters
        # only on actual EXECUTING entry / exit transitions, not every tick.
        self._prev_nav_status_for_bypass = 'IDLE'

        # --- Planner-failure auto-retry ------------------------------------
        # If NavigateToPose ABORTs within `_retry_window_s` of send_nav_goal,
        # automatically re-run the inflation-escape sequence and re-publish
        # the same goal. Capped at `_retry_max` attempts per user-issued goal
        # so a genuinely impossible goal doesn't loop forever.
        self._retry_lock = threading.Lock()
        self._last_goal_send = None   # {'x','y','yaw','stamp'} or None
        self._retry_count = 0
        self._retry_max = 2
        self._retry_window_s = 6.0

        # --- WebSocket subscriber registry ----------------------------------
        # Telemetry pusher fans out to every connected /ws/telemetry client.
        # Adds/removes are handled inside the per-connection handler.
        self._ws_lock = threading.Lock()
        self._ws_telemetry_clients = set()

        # --- Desktop guarded command console --------------------------------
        # Per-websocket handlers enforce one active process per browser tab.
        # The shared audit log is intentionally small and in-memory; it is
        # enough to see what was run from the console without creating another
        # persistent operator-data file.
        self._command_audit_lock = threading.Lock()
        self._command_audit = deque(maxlen=100)

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

        # --- Map packages + waypoints (Part D) ---
        # Live SLAM starts with a session id. Saved/AMCL maps replace it with
        # a stable map_id computed from the .yaml + referenced image, and the
        # sidecar manifest beside that map owns the active parking spots.
        self._waypoints_lock = threading.Lock()
        self._waypoints = []   # list[dict]; persisted to WAYPOINTS_PATH
        self._legacy_waypoints = []  # preserved central DB / diagnostics
        self._map_fingerprint = self._new_map_session_fingerprint()
        self._map_mode = 'live_slam'
        self._active_map_yaml = ''
        self._active_map_pgm = ''
        self._active_manifest_path = ''
        self._active_map_hash = ''
        self._active_map_metadata = {}
        self._active_map_created_at = ''
        self._pending_active_map_yaml = _active_map_yaml_param

        # --- Topic health stats (Part G3) ---
        # EWMA rate per monitored topic so /api/health can report green /
        # yellow / red without bringing in `ros2 topic hz` machinery.
        self._health_lock = threading.Lock()
        self._health_stats = {topic: {'last_t': 0.0, 'rate': 0.0}
                              for topic, _, _ in HEALTH_TOPICS}

        # --- Desktop screenshot stream (1 Hz) ---
        # Captures GNOME desktop via gnome-screenshot every ~1 s, downsamples
        # to ~720p, JPEG-encodes, caches bytes. Lets the app render a
        # low-rate "Desktop" tab so the user can see RViz / dev windows
        # without VNC. Wayland-friendly (gnome-screenshot uses portal API).
        self._desktop_lock = threading.Lock()
        self._desktop_jpeg = None     # JPEG bytes of latest shot
        self._desktop_stamp = 0.0
        self._desktop_version = 0
        self._desktop_warned = False  # log only once if capture is failing

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
        # Dedicated in-place map/pose-graph reset (keeps the node ACTIVE; does
        # not fight lifecycle_manager_slam the way a deactivate/cleanup cycle
        # does). Preferred path for restart_mapping().
        if SlamReset is not None:
            self._slam_reset = self.create_client(
                SlamReset, '/slam_toolbox/reset')
        else:
            self._slam_reset = None

        # --- AMCL no-motion update client + timer (Part E) ---
        # nav2_amcl advertises this Empty service globally as
        # /request_nomotion_update. In live-SLAM mode it simply never becomes
        # ready, so the periodic callback no-ops. The timer ticks at 1 Hz and
        # the callback enforces the configured interval itself.
        self._amcl_nomotion_client = self.create_client(
            Empty, '/request_nomotion_update')
        self._relocalize_timer = self.create_timer(1.0, self._relocalize_auto_cb)

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

        # --- Pose timer: serve map->base_link TF as the robot pose (~15 Hz) ---
        self._pose_tf_timer = self.create_timer(1.0 / 15.0, self._update_pose_from_tf)

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
        if self._pending_active_map_yaml:
            self._activate_saved_map_package(self._pending_active_map_yaml,
                                             load_spots=True)
        else:
            self._activate_live_session_waypoints()

        # Continuous near-wall safety monitor (items 3 + 4).
        self._near_wall_thread = threading.Thread(
            target=self._near_wall_monitor_loop, daemon=True,
            name='near-wall-monitor')
        self._near_wall_thread.start()

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
        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        with self._map_lock:
            self._map_png = png_bytes
            self._map_info = info
            self._map_grid = grid
            self._map_version += 1

    def _update_pose_from_tf(self):
        """Primary pose source: look up map->base_link and serve it as the
        robot pose. This is the true map-frame pose in every mode (AMCL,
        live-SLAM, EKF) and keeps tracking continuously between AMCL updates,
        so a stationary robot never reverts to the odom-frame (0,0,0) the way
        the old /amcl_pose-then-/odom fallback did. Runs at ~15 Hz off a timer;
        the lookup is non-blocking (latest available transform)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except TransformException:
            # No map frame yet (manual / bridge-only mode, or AMCL not up).
            # _odom_cb provides the fallback after _last_tf_ok goes stale.
            return
        t = tf.transform.translation
        yaw = self._quat_to_yaw(tf.transform.rotation)
        now = time.time()
        with self._pose_lock:
            self._pose = {'x_m': t.x, 'y_m': t.y, 'yaw_rad': yaw, 'stamp': now}
            self._pose_source = 'amcl'
            self._last_tf_ok = now

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
        # Raw /odom is an ODOM-frame pose. Only serve it when no map->base_link
        # TF has been available for 2 s — i.e. genuine manual / bridge-only
        # mode with no map frame. While AMCL/SLAM are up, _update_pose_from_tf
        # owns the pose (map frame); using odom here would serve an odom-frame
        # pose as if it were map-frame (the off-map (0,0,0) bug).
        with self._pose_lock:
            if (time.time() - self._last_tf_ok) < 2.0:
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
            prev = self._prev_nav_status_for_bypass
            self._nav_status = label
            self._nav_status_stamp = time.time()
            self._prev_nav_status_for_bypass = label
        # Nav-bypass is no longer flipped here. The old behaviour bypassed
        # collision_monitor for the *entire* EXECUTING phase, which left the
        # chassis with no obstacle braking during autonomous driving. The
        # near-wall monitor (_near_wall_monitor_loop) now owns nav-bypass:
        # collision_monitor stays authoritative in open space and is only
        # bypassed when the robot is genuinely close to a wall.
        # ABORTED soon after send usually means SMAC couldn't plan from a
        # costly start pose (typical wall-parked case). In 'standard' mode
        # fire one same-goal auto-retry; in 'multipoint' mode decompose:
        # stage the goal via an intermediate open-space waypoint and then
        # re-issue the final goal.
        if label == 'ABORTED':
            if self.get_planner_mode() == 'multipoint':
                self._maybe_decompose_goal()
            else:
                self._maybe_auto_retry_goal()

        # Multi-point staging: an intermediate waypoint just SUCCEEDED ->
        # resume the final goal. When the final goal itself succeeds the
        # stage is 'final', so this does not re-fire.
        if label == 'SUCCEEDED':
            with self._mp_lock:
                resume = (self._mp_stage == 'waypoint'
                          and self._mp_final_goal is not None)
                final = dict(self._mp_final_goal) if resume else None
                if resume:
                    self._mp_stage = 'final'
                else:
                    self._mp_stage = 'idle'
                    self._mp_final_goal = None
            if resume:
                threading.Thread(target=self._mp_resume_final,
                                 args=(final,), daemon=True,
                                 name='multipoint-final').start()

        # Once a goal terminates, mark the cached goal inactive so the map
        # overlay clears even if the user didn't explicitly cancel.
        if label in ('SUCCEEDED', 'CANCELED', 'ABORTED'):
            with self._goal_lock:
                self._current_goal['active'] = False
            with self._plan_lock:
                self._plan_points = []
        if label == 'CANCELED':
            with self._mp_lock:
                self._mp_stage = 'idle'
                self._mp_final_goal = None

    def _set_nav_bypass(self, active: bool) -> None:
        """Flip /nav2_pico_bridge:nav_bypass_active to the given value.
        Runs on a background thread because set_remote_params does a
        synchronous wait on the parameter service. Logs the outcome
        either way — this is a real safety mode flip and we want it
        visible in the bridge journal."""
        try:
            res = self.set_remote_params(
                '/nav2_pico_bridge', {'nav_bypass_active': bool(active)},
                timeout=2.0)
            r = res.get('nav_bypass_active', {})
            if r.get('ok'):
                self._nav_bypass_active = bool(active)
                self.get_logger().warning(
                    f"nav-bypass -> {'ON' if active else 'OFF'} "
                    f"(nav_status transition)")
            else:
                self.get_logger().error(
                    f"nav-bypass set to {active} REJECTED: "
                    f"{r.get('reason', '(no reason)')}")
        except Exception as exc:
            self.get_logger().error(f'_set_nav_bypass({active}) failed: {exc}')

    def _maybe_auto_retry_goal(self) -> None:
        """Re-escape + re-publish the last goal once, if we just aborted
        within the retry window. Runs on a background thread so the
        nav_status callback itself never blocks."""
        with self._retry_lock:
            last = self._last_goal_send
            if last is None:
                return
            age = time.time() - last.get('stamp', 0.0)
            if age > self._retry_window_s:
                return
            if self._retry_count >= self._retry_max:
                self.get_logger().warning(
                    f'auto-retry: cap reached ({self._retry_count}/'
                    f'{self._retry_max}), giving up on '
                    f'({last["x"]:.2f},{last["y"]:.2f})')
                return
            self._retry_count += 1
            attempt = self._retry_count
            goal = dict(last)

        def _retry():
            try:
                self.get_logger().info(
                    f'auto-retry {attempt}/{self._retry_max}: re-escaping '
                    f'and re-publishing ({goal["x"]:.2f},{goal["y"]:.2f})')
                try:
                    self._relax_inflation()
                except Exception as exc:
                    self.get_logger().warning(
                        f'auto-retry escape failed: {exc}')
                time.sleep(0.3)  # let cleared costmaps refresh

                from math import cos as mcos, sin as msin
                msg = PoseStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = float(goal['x'])
                msg.pose.position.y = float(goal['y'])
                msg.pose.orientation.z = msin(float(goal['yaw']) / 2.0)
                msg.pose.orientation.w = mcos(float(goal['yaw']) / 2.0)
                with self._goal_lock:
                    self._current_goal = {
                        'x': float(goal['x']), 'y': float(goal['y']),
                        'yaw': float(goal['yaw']),
                        'active': True, 'stamp': time.time(),
                    }
                # Refresh send-stamp so a later abort still falls inside
                # the retry window for any subsequent attempts.
                with self._retry_lock:
                    if self._last_goal_send is not None:
                        self._last_goal_send['stamp'] = time.time()
                self._goal_pub.publish(msg)
            except Exception as exc:
                self.get_logger().warning(f'auto-retry publish failed: {exc}')

        threading.Thread(target=_retry, daemon=True,
                         name='nav-auto-retry').start()

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
            # Wheel-direction-consistent joystick: push-right always = front
            # wheels physically right, regardless of forward/reverse (RC-car
            # feel). ROS convention: +angular.z = CCW = body rotates left,
            # joystick +X = right push, so wz_cmd = -x maps push-right to a
            # right command.
            #
            # NOTE: wz is NOT scaled by speed_limit (sl). The speed slider must
            # limit forward SPEED only, not steering authority — scaling wz by
            # sl shrank the steering range so a kept-low speed slider meant the
            # wheels never reached full lock ("tam dönmüyor"). Full stick =>
            # full steering at any speed setting.
            #
            # NO reverse pre-flip: the old `if y < 0: wz_cmd = -wz_cmd` flipped
            # the sign whenever the throttle axis dipped below zero, so a steady
            # "full left" with the stick grazing y≈0 made wz flip +/-, which the
            # bridge turned into a servo that slammed left<->right (the manual
            # jitter bug). RC-style steering is now produced directly in
            # nav2_pico_bridge for the manual source (stick -> wheel angle, no
            # speed/direction dependence), so a consistent wz sign here is both
            # correct and jitter-free. Nav2's /cmd_vel is unaffected (it does not
            # go through this path).
            twist.angular.z = -x * max_wz  # full steering authority, not sl-scaled * sl

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

    # --- Desktop screenshot ---

    def capture_desktop_once(self) -> bool:
        """Capture one frame of the GNOME desktop, downsample, JPEG-encode,
        and cache. Returns True on success, False on any failure (and warns
        once via the logger). Designed to be called at ~1 Hz from a worker
        thread; safe to call concurrently with HTTP fetches via the lock.

        Storage: gnome-screenshot writes to /dev/shm (RAM-backed tmpfs, not
        the SD card). The temp PNG is deleted immediately after we've
        decoded it into memory, so steady-state on-disk footprint is zero
        and RAM usage is just the cached JPEG bytes (~100 KB).
        """
        tmp_path = '/dev/shm/autonexa_desktop.png'

        # Inherit DISPLAY/XDG_RUNTIME_DIR from launch env; the bridge runs
        # as the same user that owns the GNOME session, which is what
        # gnome-screenshot needs to talk to the screenshot portal.
        env = dict(os.environ)
        env.setdefault('DISPLAY', ':0')
        try:
            uid = os.getuid()
            env.setdefault('XDG_RUNTIME_DIR', f'/run/user/{uid}')
        except OSError:
            pass

        # Pre-clean any leftover from a previous failed cycle so we never
        # accidentally serve a stale frame on a transient failure.
        try:
            os.unlink(tmp_path)
        except FileNotFoundError:
            pass
        except OSError:
            pass

        try:
            result = subprocess.run(
                ['gnome-screenshot', '-f', tmp_path],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                env=env, timeout=2.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            if not self._desktop_warned:
                self.get_logger().warning(
                    f'desktop capture disabled: gnome-screenshot {exc}')
                self._desktop_warned = True
            return False

        if result.returncode != 0 or not os.path.exists(tmp_path):
            if not self._desktop_warned:
                self.get_logger().warning(
                    f'gnome-screenshot rc={result.returncode} '
                    f'stderr={result.stderr.decode("utf-8", errors="replace")[:200]!r}')
                self._desktop_warned = True
            return False

        try:
            img = cv2.imread(tmp_path)
        finally:
            # Delete the temp PNG immediately — we have the bytes in `img`
            # now, so the file has served its purpose. Keeps /dev/shm at
            # zero between captures.
            try:
                os.unlink(tmp_path)
            except OSError:
                pass

        if img is None:
            return False

        # Downsample if wider than 1280 px so a 1080p source becomes ~720p.
        # Keeps JPEG payload around 80–200 KB at q=60.
        h, w = img.shape[:2]
        if w > 1280:
            new_w = 1280
            new_h = int(h * (new_w / w))
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if not ok:
            return False

        with self._desktop_lock:
            self._desktop_jpeg = buf.tobytes()
            self._desktop_stamp = time.time()
            self._desktop_version += 1
            # Reset warning latch on success so a recovery is logged.
            self._desktop_warned = False
        return True

    def get_desktop_jpeg(self):
        with self._desktop_lock:
            return self._desktop_jpeg, self._desktop_version, self._desktop_stamp

    # --- Desktop console / system status ---

    def get_command_profiles(self) -> dict:
        return {
            'profiles': _public_command_profiles(),
            'audit': self.get_command_audit(),
        }

    def get_command_audit(self) -> list:
        with self._command_audit_lock:
            return list(self._command_audit)

    def audit_command(self, entry: dict) -> None:
        safe = {
            'timestamp': _now_stamp(),
            'run_id': entry.get('run_id', ''),
            'profile_id': entry.get('profile_id', ''),
            'args': entry.get('args', {}),
            'return_code': entry.get('return_code'),
            'reason': entry.get('reason', ''),
        }
        with self._command_audit_lock:
            self._command_audit.appendleft(safe)

    def get_system_status(self) -> dict:
        mem = self._read_meminfo()
        disk = shutil.disk_usage(os.path.expanduser('~'))
        temp_c = self._read_cpu_temp_c()
        try:
            load = list(os.getloadavg())
        except (AttributeError, OSError):
            load = []
        try:
            node_count = len(self.get_node_names())
            ros_graph_ok = True
        except Exception:
            node_count = 0
            ros_graph_ok = False
        return {
            'bridge_uptime_s': round(time.time() - self._bridge_start_time, 1),
            'loadavg': load,
            'memory': mem,
            'disk_home': {
                'total': disk.total,
                'used': disk.used,
                'free': disk.free,
                'percent': round((disk.used / disk.total) * 100.0, 1)
                if disk.total else 0.0,
            },
            'temperature_c': temp_c,
            'ros_available': shutil.which('ros2') is not None,
            'ros_graph_ok': ros_graph_ok,
            'ros_node_count': node_count,
            'desktop_web_dir': DESKTOP_WEB_DIR,
            'camera_running': self._camera_running,
        }

    def _read_meminfo(self) -> dict:
        values = {}
        try:
            with open('/proc/meminfo', 'r', encoding='utf-8') as fh:
                for line in fh:
                    name, rest = line.split(':', 1)
                    parts = rest.strip().split()
                    if parts:
                        values[name] = int(parts[0]) * 1024
        except (OSError, ValueError):
            return {}
        total = values.get('MemTotal', 0)
        avail = values.get('MemAvailable', 0)
        used = max(0, total - avail)
        return {
            'total': total,
            'available': avail,
            'used': used,
            'percent': round((used / total) * 100.0, 1) if total else 0.0,
        }

    def _read_cpu_temp_c(self):
        for path in (
            '/sys/class/thermal/thermal_zone0/temp',
            '/sys/class/hwmon/hwmon0/temp1_input',
        ):
            try:
                with open(path, 'r', encoding='utf-8') as fh:
                    raw = fh.read().strip()
                if not raw:
                    continue
                value = float(raw)
                return round(value / 1000.0 if value > 200 else value, 1)
            except (OSError, ValueError):
                continue
        return None

    def tail_latest_ros_log(self, lines: int) -> dict:
        log_root = os.path.realpath(os.path.expanduser('~/.ros/log'))
        latest = os.path.realpath(os.path.join(log_root, 'latest'))
        prefix = log_root + os.sep
        if latest != log_root and not latest.startswith(prefix):
            return {'ok': False, 'error': 'latest log path escapes ~/.ros/log'}
        if not os.path.isdir(latest):
            return {'ok': False, 'error': '~/.ros/log/latest does not exist'}

        candidates = []
        for root, dirs, names in os.walk(latest):
            real_root = os.path.realpath(root)
            if real_root != log_root and not real_root.startswith(prefix):
                dirs[:] = []
                continue
            for name in names:
                path = os.path.realpath(os.path.join(root, name))
                if path != log_root and not path.startswith(prefix):
                    continue
                try:
                    if os.path.isfile(path):
                        candidates.append((os.path.getmtime(path), path))
                except OSError:
                    continue
        if not candidates:
            return {'ok': False, 'error': 'no log files under latest'}

        _, newest = max(candidates)
        tail = deque(maxlen=lines)
        try:
            with open(newest, 'r', encoding='utf-8', errors='replace') as fh:
                for line in fh:
                    tail.append(line)
        except OSError as exc:
            return {'ok': False, 'error': str(exc)}
        shown_path = newest.replace(os.path.expanduser('~'), '~', 1)
        return {
            'ok': True,
            'path': shown_path,
            'text': ''.join(tail),
        }

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

    @staticmethod
    def _utc_now() -> str:
        return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())

    @staticmethod
    def _abs_user_path(path: str) -> str:
        return os.path.abspath(os.path.expanduser(path or ''))

    @staticmethod
    def _manifest_path_for_yaml(yaml_path: str) -> str:
        base, _ = os.path.splitext(yaml_path)
        return base + '.autonexa.json'

    @staticmethod
    def _sha256_file(path: str) -> str:
        h = hashlib.sha256()
        with open(path, 'rb') as fh:
            for chunk in iter(lambda: fh.read(1024 * 1024), b''):
                h.update(chunk)
        return h.hexdigest()

    @staticmethod
    def _parse_map_yaml(yaml_path: str) -> dict:
        with open(yaml_path, 'r', encoding='utf-8') as fh:
            text = fh.read()
        if yaml is not None:
            doc = yaml.safe_load(text) or {}
            if isinstance(doc, dict):
                return doc
        # Tiny fallback for nav2 map YAMLs if PyYAML is unavailable.
        doc = {}
        for line in text.splitlines():
            line = line.split('#', 1)[0].strip()
            if ':' not in line:
                continue
            key, val = line.split(':', 1)
            key = key.strip()
            val = val.strip().strip('"').strip("'")
            if key == 'origin':
                nums = re.findall(r'-?\d+(?:\.\d+)?', val)
                doc[key] = [float(n) for n in nums[:3]]
            elif key in ('resolution', 'occupied_thresh', 'free_thresh'):
                try:
                    doc[key] = float(val)
                except ValueError:
                    doc[key] = val
            elif key == 'negate':
                try:
                    doc[key] = int(val)
                except ValueError:
                    doc[key] = val
            elif key == 'image':
                doc[key] = val
        return doc

    def _map_package_from_yaml(self, yaml_path: str) -> dict:
        yaml_abs = self._abs_user_path(yaml_path)
        doc = self._parse_map_yaml(yaml_abs)
        image_name = str(doc.get('image') or
                         (os.path.splitext(os.path.basename(yaml_abs))[0] + '.pgm'))
        image_abs = image_name if os.path.isabs(image_name) else \
            os.path.abspath(os.path.join(os.path.dirname(yaml_abs), image_name))
        image_sha = self._sha256_file(image_abs) if os.path.exists(image_abs) else ''
        origin = doc.get('origin') if isinstance(doc.get('origin'), list) else [0.0, 0.0, 0.0]
        metadata = {
            'resolution': float(doc.get('resolution', 0.0) or 0.0),
            'origin': [float(v) for v in (origin + [0.0, 0.0, 0.0])[:3]],
            'negate': int(doc.get('negate', 0) or 0),
            'occupied_thresh': float(doc.get('occupied_thresh', 0.0) or 0.0),
            'free_thresh': float(doc.get('free_thresh', 0.0) or 0.0),
            'image_sha256': image_sha,
        }
        map_hash = hashlib.sha256(
            json.dumps(metadata, sort_keys=True, separators=(',', ':')).encode('utf-8')
        ).hexdigest()
        return {
            'map_id': f'map:{map_hash[:16]}',
            'map_hash': map_hash,
            'yaml': yaml_abs,
            'pgm': image_abs,
            'manifest': self._manifest_path_for_yaml(yaml_abs),
            'metadata': metadata,
        }

    def _active_map_identity(self) -> dict:
        with self._map_lock:
            return {
                'map_id': self._map_fingerprint,
                'map_fingerprint': self._map_fingerprint,
                'map_mode': self._map_mode,
                'map_yaml': self._active_map_yaml,
                'pgm': self._active_map_pgm,
                'manifest': self._active_manifest_path,
                'map_hash': self._active_map_hash,
            }

    def _stamp_waypoint_for_active_map(self, wp: dict) -> dict:
        ident = self._active_map_identity()
        entry = dict(wp)
        entry['map_id'] = ident['map_id']
        entry['map_fingerprint'] = ident['map_id']
        entry['map_mode'] = ident['map_mode']
        if ident.get('map_yaml'):
            entry['map_yaml'] = ident['map_yaml']
        if ident.get('manifest'):
            entry['manifest'] = ident['manifest']
        return entry

    @staticmethod
    def _waypoint_key(wp: dict) -> tuple:
        return (str(wp.get('name') or ''),
                str(wp.get('map_id') or wp.get('map_fingerprint') or ''))

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
                self._legacy_waypoints = [dict(w) for w in wps if isinstance(w, dict)]
            self.get_logger().info(f'loaded {len(wps)} waypoint(s) from {WAYPOINTS_PATH}')

    def _save_waypoints(self) -> None:
        try:
            os.makedirs(AUTONEXA_DATA_DIR, exist_ok=True)
        except OSError:
            pass
        with self._waypoints_lock:
            active = [dict(w) for w in self._waypoints]
            active_keys = {self._waypoint_key(w) for w in active}
            legacy = [
                dict(w) for w in self._legacy_waypoints
                if self._waypoint_key(w) not in active_keys
            ]
            self._legacy_waypoints = legacy + active
            doc = {'schema_version': 2, 'waypoints': list(self._legacy_waypoints)}
        try:
            tmp = WAYPOINTS_PATH + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh, indent=2)
            os.replace(tmp, WAYPOINTS_PATH)
        except OSError as exc:
            self.get_logger().error(f'waypoints save failed: {exc}')

    def _activate_live_session_waypoints(self) -> None:
        """Use old unscoped waypoints as temporary live-session spots.

        This preserves pre-map-package data long enough for the operator to
        save a map, at which point the spots are moved into that map manifest.
        """
        with self._waypoints_lock:
            active = []
            for wp in self._legacy_waypoints:
                mid = str(wp.get('map_id') or wp.get('map_fingerprint') or '')
                if not mid or not mid.startswith('map:'):
                    active.append(self._stamp_waypoint_for_active_map(wp))
            self._waypoints = active

    def _load_manifest_doc(self, manifest_path: str) -> dict:
        if not manifest_path or not os.path.exists(manifest_path):
            return {}
        try:
            with open(manifest_path, 'r', encoding='utf-8') as fh:
                doc = json.load(fh)
            return doc if isinstance(doc, dict) else {}
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f'map manifest load failed: {exc}')
            return {}

    def _save_active_manifest(self) -> None:
        ident = self._active_map_identity()
        if not ident.get('map_yaml') or not ident.get('manifest'):
            return
        existing = self._load_manifest_doc(ident['manifest'])
        created = existing.get('created_at') or self._active_map_created_at or self._utc_now()
        with self._waypoints_lock:
            spots = [self._stamp_waypoint_for_active_map(w) for w in self._waypoints]
        doc = {
            'schema_version': 1,
            'map_id': ident['map_id'],
            'map_hash': ident.get('map_hash', ''),
            'map_mode': 'saved_map',
            'yaml': ident['map_yaml'],
            'pgm': ident.get('pgm', ''),
            'manifest': ident['manifest'],
            'metadata': dict(self._active_map_metadata),
            'created_at': created,
            'updated_at': self._utc_now(),
            'spots': spots,
        }
        try:
            os.makedirs(os.path.dirname(ident['manifest']), exist_ok=True)
            tmp = ident['manifest'] + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fh:
                json.dump(doc, fh, indent=2)
            os.replace(tmp, ident['manifest'])
            self._save_waypoints()
        except OSError as exc:
            self.get_logger().error(f'map manifest save failed: {exc}')

    def _activate_saved_map_package(self, yaml_path: str,
                                    load_spots: bool = True) -> dict:
        try:
            pkg = self._map_package_from_yaml(yaml_path)
        except Exception as exc:
            self.get_logger().warning(
                f'active map package unavailable for {yaml_path!r}: {exc}')
            return {}

        manifest = self._load_manifest_doc(pkg['manifest'])
        manifest_id = str(manifest.get('map_id') or '')
        manifest_compatible = not manifest_id or manifest_id == pkg['map_id']
        if manifest_id and not manifest_compatible:
            self.get_logger().warning(
                f'ignoring manifest spots for changed map: '
                f'{manifest_id} != {pkg["map_id"]}')
        created = manifest.get('created_at') or self._utc_now()
        with self._map_lock:
            self._map_mode = 'amcl'
            self._map_fingerprint = pkg['map_id']
            self._active_map_yaml = pkg['yaml']
            self._active_map_pgm = pkg['pgm']
            self._active_manifest_path = pkg['manifest']
            self._active_map_hash = pkg['map_hash']
            self._active_map_metadata = dict(pkg['metadata'])
            self._active_map_created_at = created

        if load_spots:
            spots = (manifest.get('spots') or manifest.get('waypoints') or []) \
                if manifest_compatible else []
            active = []
            if isinstance(spots, list):
                for wp in spots:
                    if isinstance(wp, dict):
                        active.append(self._stamp_waypoint_for_active_map(wp))
            with self._waypoints_lock:
                self._waypoints = active
            self.get_logger().info(
                f'active map package {self._map_fingerprint}: '
                f'{len(active)} spot(s) from {pkg["manifest"]}')
        return pkg

    def _promote_active_session_to_saved_map(self, yaml_path: str) -> dict:
        pkg = self._activate_saved_map_package(yaml_path, load_spots=False)
        if not pkg:
            return {}
        with self._waypoints_lock:
            self._waypoints = [
                self._stamp_waypoint_for_active_map(w)
                for w in self._waypoints
            ]
        with self._map_lock:
            self._map_mode = 'saved_map'
        self._save_active_manifest()
        return pkg

    def _current_fingerprint(self) -> str:
        with self._map_lock:
            return self._map_fingerprint

    @staticmethod
    def _new_map_session_fingerprint() -> str:
        return f"session={int(time.time() * 1000)}"

    def list_waypoints(self, include_stale: bool = False) -> list:
        ident = self._active_map_identity()
        fp = ident['map_id']
        with self._waypoints_lock:
            active = [dict(w) for w in self._waypoints]
            legacy = [dict(w) for w in self._legacy_waypoints]
        out = []
        seen = set()
        for wp in active:
            entry = self._stamp_waypoint_for_active_map(wp)
            entry['stale'] = False
            out.append(entry)
            seen.add(self._waypoint_key(entry))
        if include_stale:
            for wp in legacy:
                key = self._waypoint_key(wp)
                if key in seen:
                    continue
                entry = dict(wp)
                mid = str(entry.get('map_id') or entry.get('map_fingerprint') or '')
                entry['map_id'] = mid
                entry['map_fingerprint'] = mid
                entry['stale'] = bool(fp) and mid != fp
                out.append(entry)
                seen.add(key)
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
        created_at = self._utc_now()
        with self._waypoints_lock:
            existing = next((w for w in self._waypoints
                             if w.get('name') == name), None)
            if existing and existing.get('created_at'):
                created_at = existing.get('created_at')
        entry = {
            'name': name,
            'kind': kind,
            'pose': {'x': x, 'y': y, 'yaw': yaw},
            'created_at': created_at,
            'updated_at': self._utc_now(),
        }
        entry = self._stamp_waypoint_for_active_map(entry)
        with self._waypoints_lock:
            self._waypoints = [w for w in self._waypoints if w.get('name') != name]
            self._waypoints.append(entry)
        if self._active_map_identity().get('manifest'):
            self._save_active_manifest()
        else:
            self._save_waypoints()
        return entry

    def delete_waypoint(self, name: str) -> bool:
        with self._waypoints_lock:
            before = len(self._waypoints)
            self._waypoints = [w for w in self._waypoints if w.get('name') != name]
            removed = len(self._waypoints) < before
        if removed:
            if self._active_map_identity().get('manifest'):
                self._save_active_manifest()
            else:
                self._save_waypoints()
        return removed

    def navigate_to_waypoint(self, name: str) -> bool:
        active_id = self._current_fingerprint()
        with self._waypoints_lock:
            wp = next((w for w in self._waypoints if w.get('name') == name), None)
        if wp is None:
            return False
        wp_id = str(wp.get('map_id') or wp.get('map_fingerprint') or '')
        if wp_id and wp_id != active_id:
            self.get_logger().warning(
                f'refusing stale waypoint {name!r}: {wp_id} != {active_id}')
            return False
        pose = wp.get('pose') or {}
        return self.send_nav_goal(
            pose.get('x', 0.0),
            pose.get('y', 0.0),
            pose.get('yaw', 0.0),
        )

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

    def detect_ctrl_speed_param(self, force: bool = False) -> str:
        """Speed-cap param name for whichever controller is active:
        MPPI -> 'FollowPath.vx_max', RPP -> 'FollowPath.desired_linear_vel'.
        Cached; pass force=True to re-detect after a relaunch into the other
        controller. Falls back to the MPPI key (current default) without
        caching when the controller isn't up yet."""
        cached = getattr(self, '_ctrl_speed_param', None)
        if cached is not None and not force:
            return cached
        names = self.list_remote_params('/controller_server')
        if 'FollowPath.vx_max' in names:
            self._ctrl_speed_param = 'FollowPath.vx_max'
        elif 'FollowPath.desired_linear_vel' in names:
            self._ctrl_speed_param = 'FollowPath.desired_linear_vel'
        else:
            return 'FollowPath.vx_max'
        return self._ctrl_speed_param

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
        on-disk entries for that node are preserved if not overwritten.

        For the nav2_pico_bridge node, only physical CALIBRATION keys are
        written to disk (BRIDGE_CALIBRATION_KEYS) — bridge tunables are kept
        out of the file so a stale phone value can't silently override the PC
        config on the next launch. The values are still applied live to the
        running node by the caller; they just aren't persisted."""
        if yaml is None:
            self.get_logger().warning(
                'PyYAML missing — runtime overrides not persisted to disk')
            return
        if node.lstrip('/') == 'nav2_pico_bridge':
            kept = {k: v for k, v in items.items() if k in BRIDGE_CALIBRATION_KEYS}
            dropped = [k for k in items if k not in BRIDGE_CALIBRATION_KEYS]
            if dropped:
                self.get_logger().info(
                    f'not persisting non-calibration bridge params '
                    f'(PC config authoritative): {sorted(dropped)}')
            items = kept
            if not items:
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

    def reset_runtime_overrides(self) -> dict:
        """Delete runtime_overrides.yaml so the PC config files (launch args +
        nav2 params) become the single source of truth on the next relaunch.
        Used by the app's 'Reset to PC defaults' control to clear stale
        phone-saved values. Live params on running nodes are left untouched;
        PC defaults are re-applied at the next launch."""
        if not os.path.exists(RUNTIME_OVERRIDES_PATH):
            return {'ok': True, 'existed': False}
        try:
            os.remove(RUNTIME_OVERRIDES_PATH)
        except OSError as exc:
            self.get_logger().error(f'overrides reset failed: {exc}')
            return {'ok': False, 'existed': True, 'reason': str(exc)}
        self.get_logger().warning(
            f'{RUNTIME_OVERRIDES_PATH} deleted via /api/reset_overrides — '
            f'PC config is authoritative on the next relaunch')
        return {'ok': True, 'existed': True}

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

    def _slam_reset_call(self, timeout: float = 8.0) -> dict:
        """Call /slam_toolbox/reset (slam_toolbox/srv/Reset). Wipes the
        pose graph + occupancy map in place while the node stays ACTIVE."""
        if self._slam_reset is None:
            return {'ok': False, 'reason': 'slam_toolbox/srv/Reset unavailable'}
        if not self._slam_reset.wait_for_service(timeout_sec=2.0):
            return {'ok': False, 'reason': '/slam_toolbox/reset not advertised'}
        req = SlamReset.Request()
        req.pause_new_measurements = False
        future = self._slam_reset.call_async(req)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {'ok': False, 'reason': 'timeout'}
        res = future.result()
        # RESULT_SUCCESS == 0.
        return {'ok': res.result == SlamReset.Response.RESULT_SUCCESS,
                'result': int(res.result)}

    def restart_mapping(self) -> dict:
        """Drop the current map and start fresh.

        Preferred path is the dedicated /slam_toolbox/reset service, which
        clears the pose graph + map in place while SLAM Toolbox stays ACTIVE.
        The previous deactivate->cleanup->configure->activate lifecycle cycle
        fought lifecycle_manager_slam and routinely timed out on the slow
        cleanup teardown — so the map never actually reset even though the app
        bumped its version. Falls back to that cycle only if the reset service
        isn't available. Costmaps are cleared either way so the planner doesn't
        carry stale obstacles into the fresh map."""
        reset = self._slam_reset_call()
        steps = [{'step': 'reset', **reset}]
        if not reset.get('ok') and self._slam_reset is not None and \
                reset.get('reason') == '/slam_toolbox/reset not advertised':
            # Older slam_toolbox without the reset service: fall back to the
            # lifecycle cycle (best-effort).
            for tid, label in (
                (Transition.TRANSITION_DEACTIVATE, 'deactivate'),
                (Transition.TRANSITION_CLEANUP, 'cleanup'),
                (Transition.TRANSITION_CONFIGURE, 'configure'),
                (Transition.TRANSITION_ACTIVATE, 'activate'),
            ):
                r = self._slam_change(tid, timeout=8.0)
                steps.append({'step': label, **r})
                if not r.get('ok'):
                    break
        ok = any(s['step'] == 'reset' and s.get('ok') for s in steps) or \
            any(s['step'] == 'activate' and s.get('ok') for s in steps)
        # Drop costmap obstacles; their map frame is about to change anyway.
        cm = self.clear_costmaps()
        # Bump map_version so the app refetches a blank PNG immediately.
        with self._map_lock:
            self._map_version += 1
            self._map_png = None
            self._map_fingerprint = self._new_map_session_fingerprint()
            self._map_mode = 'live_slam'
            self._active_map_yaml = ''
            self._active_map_pgm = ''
            self._active_manifest_path = ''
            self._active_map_hash = ''
            self._active_map_metadata = {}
            self._active_map_created_at = ''
        with self._waypoints_lock:
            self._waypoints = []
        return {'ok': ok, 'steps': steps, 'costmaps': cm}

    # --- Robot dimension live-edit + goal inflation escape ---

    def get_robot_dimensions(self) -> dict:
        """Effective dimensions = defaults + persisted YAML. Always full dict."""
        if _build_urdf is None:
            return {}
        return _build_urdf.merge_dimensions(_build_urdf.load_persisted_dimensions())

    def apply_robot_dimensions(self, overrides: dict) -> dict:
        """Render URDF with the requested overrides, push it to
        robot_state_publisher, sync both costmap footprints, and persist
        the merged result to ~/.autonexa/robot_dimensions.yaml.

        Returns {'dims': effective, 'urdf_ok': bool, 'costmaps': {...},
        'errors': [...]}.
        """
        if _build_urdf is None:
            return {'ok': False, 'reason': 'build_urdf module unavailable'}
        try:
            urdf_xml, footprint_str, dims = _build_urdf.render(overrides)
        except Exception as exc:
            return {'ok': False, 'reason': f'render failed: {exc}'}

        results = {'ok': True, 'dims': dims, 'errors': []}

        rsp = self.set_remote_params(
            '/robot_state_publisher', {'robot_description': urdf_xml})
        results['robot_state_publisher'] = rsp
        if not rsp.get('robot_description', {}).get('ok'):
            results['errors'].append(
                f"robot_state_publisher: {rsp.get('robot_description', {}).get('reason')}")

        cm_payload = {
            'footprint': footprint_str,
            'footprint_padding': float(dims['footprint_padding']),
        }
        gc = self.set_remote_params('/global_costmap/global_costmap', cm_payload)
        lc = self.set_remote_params('/local_costmap/local_costmap', cm_payload)
        results['global_costmap'] = gc
        results['local_costmap'] = lc
        for label, batch in (('global_costmap', gc), ('local_costmap', lc)):
            for k, r in batch.items():
                if not r.get('ok'):
                    results['errors'].append(f'{label}.{k}: {r.get("reason")}')

        # Persist the full merged dims so a relaunch picks up the values.
        try:
            _build_urdf.save_persisted_dimensions(dims)
        except Exception as exc:
            results['errors'].append(f'persist failed: {exc}')

        return results

    def _nudge_goal_from_walls(self, x: float, y: float, yaw: float,
                               min_clearance_m: float = 0.08) -> tuple:
        """Push goal away from static-map walls so the chassis ends up
        with enough breathing room to leave again.

        Reads the latest cached /map OccupancyGrid; computes the distance
        transform (cells away from any non-free cell); if the goal cell
        has less clearance than the robot's diagonal half-extent plus
        min_clearance_m, scans outward (Chebyshev ring) for the nearest
        cell that does. Unknown cells (-1) count as obstacle for nudging
        — we never push the goal into unmapped space.

        Returns (x, y, yaw); falls back to the input on any error so a
        bad nudge never blocks a goal from being sent.
        """
        try:
            with self._map_lock:
                grid = None if self._map_grid is None else self._map_grid
                info = dict(self._map_info)
                if grid is not None:
                    grid = grid.copy()
            if grid is None or not info:
                return (x, y, yaw)
            res = float(info.get('resolution', 0.0))
            ox = float(info.get('origin_x', 0.0))
            oy = float(info.get('origin_y', 0.0))
            h, w = grid.shape
            if res <= 0.0 or h == 0 or w == 0:
                return (x, y, yaw)

            # Robot half-extents from persisted URDF dims; fall back to
            # the values baked into nav2_navigation_params.yaml footprint.
            hx, hy = 0.135, 0.10
            try:
                if _build_urdf is not None and \
                   hasattr(_build_urdf, 'load_persisted_dimensions'):
                    dims = _build_urdf.load_persisted_dimensions() or {}
                    hx = float(dims.get('chassis_length', 2 * hx)) / 2.0
                    hy = float(dims.get('chassis_width', 2 * hy)) / 2.0
            except Exception:
                pass

            required_m = hypot(hx, hy) + max(0.0, min_clearance_m)
            required_cells = max(1, int(ceil(required_m / res)))

            obstacle = (grid != 0)  # unknown(-1) + occupied(>0) both block
            try:
                from scipy.ndimage import distance_transform_edt
                dist = distance_transform_edt(~obstacle)
            except Exception:
                # Cheap fallback: bounded multi-source BFS. Distances above
                # required_cells+1 are clipped — that's fine, we only need
                # to know "is this cell at least required_cells away?".
                dist = self._bounded_distance(obstacle, required_cells + 2)

            gx = int(round((x - ox) / res))
            gy = int(round((y - oy) / res))
            if not (0 <= gx < w and 0 <= gy < h):
                return (x, y, yaw)
            if dist[gy, gx] >= required_cells:
                return (x, y, yaw)

            # Chebyshev-ring search outward for the nearest clear cell.
            max_search = max(required_cells * 6, 30)
            best = None  # (sq_dist, cx, cy)
            for r in range(1, max_search + 1):
                ring_hit = False
                for dgy in range(-r, r + 1):
                    for dgx in range(-r, r + 1):
                        if max(abs(dgx), abs(dgy)) != r:
                            continue
                        cx, cy = gx + dgx, gy + dgy
                        if not (0 <= cx < w and 0 <= cy < h):
                            continue
                        if dist[cy, cx] >= required_cells:
                            d2 = dgx * dgx + dgy * dgy
                            if best is None or d2 < best[0]:
                                best = (d2, cx, cy)
                                ring_hit = True
                if ring_hit:
                    break

            if best is None:
                self.get_logger().warning(
                    f'goal nudge: no clear cell within '
                    f'{max_search * res:.2f} m of ({x:.2f},{y:.2f}); '
                    f'passing original goal through')
                return (x, y, yaw)

            _, cx, cy = best
            nx = cx * res + ox
            ny = cy * res + oy
            self.get_logger().info(
                f'goal nudge: ({x:.2f},{y:.2f}) -> ({nx:.2f},{ny:.2f}) '
                f'(target clearance {required_m * 100:.0f} cm)')
            return (nx, ny, yaw)
        except Exception as exc:
            self.get_logger().warning(f'goal nudge failed: {exc}')
            return (x, y, yaw)

    @staticmethod
    def _bounded_distance(obstacle, max_d: int):
        """Multi-source BFS clipped at max_d cells. 4-neighbor. Returns
        a float array shaped like `obstacle` (True = source = distance 0).
        Used as a no-scipy fallback for _nudge_goal_from_walls."""
        h, w = obstacle.shape
        dist = np.full((h, w), float(max_d + 1), dtype=np.float32)
        ys, xs = np.where(obstacle)
        if ys.size == 0:
            return dist
        from collections import deque as _dq
        q = _dq()
        for y0, x0 in zip(ys.tolist(), xs.tolist()):
            dist[y0, x0] = 0.0
            q.append((y0, x0))
        while q:
            cy, cx = q.popleft()
            d = dist[cy, cx]
            if d >= max_d:
                continue
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny_, nx_ = cy + dy, cx + dx
                if 0 <= ny_ < h and 0 <= nx_ < w and dist[ny_, nx_] > d + 1:
                    dist[ny_, nx_] = d + 1
                    q.append((ny_, nx_))
        return dist

    # --- Near-wall bypass monitor (items 3 + 4) ----------------------------
    # Replaces the old _escape_inflation_for_goal one-shot relax + fixed-timer
    # restorer. A background monitor continuously evaluates how close the robot
    # is to a wall and engages/releases a full bypass accordingly.

    def _near_wall_distance(self):
        """Distance (m) from base_link to the nearest LiDAR obstacle, or
        None if there is no fresh scan/pose. Reuses the map-frame scan
        points the bridge already caches for the app overlay."""
        with self._pose_lock:
            px = self._pose['x_m']
            py = self._pose['y_m']
            pose_stamp = self._pose['stamp']
        with self._scan_lock:
            pts = list(self._scan_points)
            scan_stamp = self._scan_stamp
        if not pts or pose_stamp <= 0.0 or (time.time() - scan_stamp) > 1.5:
            return None
        best = None
        for x, y in pts:
            d = hypot(x - px, y - py)
            if best is None or d < best:
                best = d
        return best

    def _relax_inflation(self) -> None:
        """Clear both costmaps and drop inflation_radius to the escape value
        so SMAC Hybrid-A* can plan from / through wall inflation. Idempotent —
        captures the pre-relax inflation only on the first call so a later
        _restore_inflation() puts back the exact operating halo."""
        global_key = '/global_costmap/global_costmap'
        local_key = '/local_costmap/local_costmap'
        ir_name = 'inflation_layer.inflation_radius'
        with self._near_wall_lock:
            if self._inflation_relaxed:
                return
            self._inflation_relaxed = True
        # Capture the live inflation so it can be restored exactly. Ignore a
        # reading that is already at (or below) the escape value — that means
        # a stale relax; fall back to the configured operating default.
        saved_g = self.get_remote_params(global_key, [ir_name]).get(ir_name)
        saved_l = self.get_remote_params(local_key, [ir_name]).get(ir_name)
        if saved_g is None or float(saved_g) <= self._inflation_escape_radius:
            saved_g = self._saved_inflation['global']
        if saved_l is None or float(saved_l) <= self._inflation_escape_radius:
            saved_l = self._saved_inflation['local']
        with self._near_wall_lock:
            self._saved_inflation = {'global': float(saved_g),
                                     'local': float(saved_l)}
        self.clear_costmaps()
        esc = self._inflation_escape_radius
        self.set_remote_params(global_key, {ir_name: esc})
        self.set_remote_params(local_key, {ir_name: esc})
        self.get_logger().info(
            f'near-wall: inflation relaxed {saved_g:.3f}/{saved_l:.3f} -> '
            f'{esc:.3f} (global/local)')

    def _restore_inflation(self) -> None:
        """Restore inflation_radius to the values captured before the last
        relax. Idempotent."""
        with self._near_wall_lock:
            if not self._inflation_relaxed:
                return
            self._inflation_relaxed = False
            saved = dict(self._saved_inflation)
        global_key = '/global_costmap/global_costmap'
        local_key = '/local_costmap/local_costmap'
        ir_name = 'inflation_layer.inflation_radius'
        self.set_remote_params(global_key, {ir_name: float(saved['global'])})
        self.set_remote_params(local_key, {ir_name: float(saved['local'])})
        self.get_logger().info(
            f'near-wall: inflation restored {saved["global"]:.3f}/'
            f'{saved["local"]:.3f} (global/local)')

    def _engage_near_wall_bypass(self, engage: bool) -> None:
        """Engage/release the full near-wall bypass. Engaged = collision_monitor
        skipped (nav_bypass_active) AND inflation relaxed — the same
        no-safety posture as manual 'off' mode, deliberately chosen so escape
        and final-park paths next to walls are never blocked. Released =
        collision_monitor authoritative + normal inflation. Edge-triggered."""
        with self._near_wall_lock:
            if engage == self._near_wall_engaged:
                return
            self._near_wall_engaged = engage
        if engage:
            self.get_logger().warning(
                'near-wall: BYPASS engaged (collision_monitor off, '
                'inflation relaxed)')
            self._relax_inflation()
            self._set_nav_bypass(True)
        else:
            self.get_logger().warning(
                'near-wall: bypass released (collision_monitor restored)')
            self._set_nav_bypass(False)
            self._restore_inflation()

    def _near_wall_monitor_loop(self) -> None:
        """Background daemon: while a Nav2 goal is active, engage the
        near-wall bypass when the robot is close to an obstacle and release
        it when clear. Covers every goal source (app /api/nav_goal and RViz
        /goal_pose both set _current_goal['active']). Replaces the old
        EXECUTING-phase unconditional bypass and the fixed-timer restorer."""
        while True:
            time.sleep(0.3)
            try:
                with self._goal_lock:
                    goal_active = self._current_goal['active']
                if not goal_active:
                    # No goal in flight — return to the safe default state.
                    with self._near_wall_lock:
                        engaged = self._near_wall_engaged
                    if engaged:
                        self._engage_near_wall_bypass(False)
                    continue
                dist = self._near_wall_distance()
                if dist is None:
                    continue
                with self._near_wall_lock:
                    engaged = self._near_wall_engaged
                if not engaged and dist < self._near_wall_enter_m:
                    # Robot is near a wall (start wall-parked, mid-trip stall,
                    # or final park approach) — drop to no-safety so the
                    # planner can route through the inflation halo and the
                    # car can reach the exact goal point.
                    self._engage_near_wall_bypass(True)
                elif engaged and dist > self._near_wall_exit_m:
                    self._engage_near_wall_bypass(False)
            except Exception as exc:
                self.get_logger().warning(f'near-wall monitor error: {exc}')

    # --- Planner mode + multi-point decompose-on-failure (item 2) ----------

    def _load_planner_mode(self) -> str:
        """Read the persisted planner mode; default 'standard'."""
        try:
            with open(PLANNER_MODE_PATH, 'r', encoding='utf-8') as fh:
                mode = fh.read().strip()
            if mode in ('standard', 'multipoint'):
                return mode
        except OSError:
            pass
        return 'standard'

    def _save_planner_mode(self, mode: str) -> None:
        try:
            os.makedirs(AUTONEXA_DATA_DIR, exist_ok=True)
            with open(PLANNER_MODE_PATH, 'w', encoding='utf-8') as fh:
                fh.write(mode)
        except OSError as exc:
            self.get_logger().warning(f'cannot persist planner mode: {exc}')

    def get_planner_mode(self) -> str:
        with self._planner_mode_lock:
            return self._planner_mode

    def set_planner_mode(self, mode: str) -> bool:
        if mode not in ('standard', 'multipoint'):
            return False
        with self._planner_mode_lock:
            self._planner_mode = mode
        self._save_planner_mode(mode)
        self.get_logger().info(f'planner mode -> {mode}')
        return True

    # --- AMCL periodic relocalize (Part E) ---------------------------
    def _load_relocalize_auto(self) -> dict:
        """Read persisted auto-relocalize config; default disabled / 20 s."""
        default = {'enabled': False, 'interval_s': 20.0}
        try:
            with open(RELOCALIZE_AUTO_PATH, 'r', encoding='utf-8') as fh:
                doc = json.load(fh) or {}
            return {
                'enabled': bool(doc.get('enabled', False)),
                'interval_s': float(doc.get('interval_s', 20.0)),
            }
        except (OSError, ValueError, TypeError):
            return default

    def _save_relocalize_auto(self) -> None:
        try:
            os.makedirs(AUTONEXA_DATA_DIR, exist_ok=True)
            with open(RELOCALIZE_AUTO_PATH, 'w', encoding='utf-8') as fh:
                json.dump({'enabled': self._relocalize_auto_enabled,
                           'interval_s': self._relocalize_auto_interval}, fh)
        except OSError as exc:
            self.get_logger().warning(f'cannot persist relocalize_auto: {exc}')

    def get_relocalize_auto(self) -> dict:
        with self._relocalize_auto_lock:
            return {'enabled': self._relocalize_auto_enabled,
                    'interval_s': self._relocalize_auto_interval}

    def set_relocalize_auto(self, enabled=None, interval_s=None) -> dict:
        with self._relocalize_auto_lock:
            if enabled is not None:
                self._relocalize_auto_enabled = bool(enabled)
            if interval_s is not None:
                self._relocalize_auto_interval = max(2.0, float(interval_s))
            self._save_relocalize_auto()
            result = {'enabled': self._relocalize_auto_enabled,
                      'interval_s': self._relocalize_auto_interval}
        self.get_logger().info(f'relocalize_auto -> {result}')
        return result

    def _relocalize_auto_cb(self) -> None:
        """1 Hz tick: when enabled and the interval has elapsed, ask AMCL to
        re-settle its particle filter against the latest scan. No-op when
        disabled or when /amcl isn't running (live-SLAM mode)."""
        with self._relocalize_auto_lock:
            enabled = self._relocalize_auto_enabled
            interval = self._relocalize_auto_interval
        if not enabled:
            return
        now = time.time()
        if now - self._relocalize_last_t < interval:
            return
        cli = self._amcl_nomotion_client
        if cli is None or not cli.service_is_ready():
            return   # AMCL absent (live-SLAM) — silent no-op
        self._relocalize_last_t = now
        # Fire-and-forget; retain the future so it isn't GC'd before delivery.
        self._relocalize_fut = cli.call_async(Empty.Request())

    # --- Encoder->EKF fusion launch flag (Part F) --------------------
    def get_use_ekf(self) -> bool:
        """Read the persisted use_ekf launch flag (~/.autonexa/use_ekf.txt)."""
        try:
            with open(USE_EKF_PATH, 'r', encoding='utf-8') as fh:
                return fh.read().strip().lower() in ('1', 'true', 'yes', 'on')
        except OSError:
            return False

    def set_use_ekf(self, enabled: bool) -> bool:
        """Persist the use_ekf launch flag. Takes effect on the NEXT relaunch —
        the EKF owns the odom->base_link TF, which can't be swapped live."""
        try:
            os.makedirs(AUTONEXA_DATA_DIR, exist_ok=True)
            with open(USE_EKF_PATH, 'w', encoding='utf-8') as fh:
                fh.write('true' if enabled else 'false')
            self.get_logger().info(
                f'use_ekf flag -> {enabled} (takes effect on next relaunch)')
            return True
        except OSError as exc:
            self.get_logger().warning(f'cannot persist use_ekf: {exc}')
            return False

    def _publish_raw_goal(self, x: float, y: float, yaw: float) -> None:
        """Build + publish a /goal_pose and refresh the cached goal overlay."""
        from math import cos as mcos, sin as msin
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = msin(float(yaw) / 2.0)
        msg.pose.orientation.w = mcos(float(yaw) / 2.0)
        with self._goal_lock:
            self._current_goal = {
                'x': float(x), 'y': float(y), 'yaw': float(yaw),
                'active': True, 'stamp': time.time(),
            }
        self._goal_pub.publish(msg)

    def _maybe_decompose_goal(self) -> None:
        """Multi-point decompose-on-failure: a goal just ABORTed. If a final
        goal is tracked and budget remains, drive to an intermediate open-
        space waypoint (the midpoint robot<->goal, nudged clear of walls)
        and let the SUCCEEDED handler resume the final goal afterwards.
        Runs the publish on a background thread so the status callback never
        blocks."""
        with self._mp_lock:
            final = self._mp_final_goal
            stage = self._mp_stage
            count = self._mp_decompose_count
            if final is None:
                return
            if stage == 'waypoint':
                # The intermediate leg itself failed — the waypoint is
                # unreachable; stop rather than chase it.
                self.get_logger().warning(
                    'multipoint: intermediate waypoint unreachable, giving up')
                self._mp_stage = 'idle'
                self._mp_final_goal = None
                return
            if count >= self._mp_decompose_max:
                self.get_logger().warning(
                    f'multipoint: decompose cap reached '
                    f'({count}/{self._mp_decompose_max}), giving up')
                self._mp_stage = 'idle'
                self._mp_final_goal = None
                return
            self._mp_decompose_count = count + 1
            self._mp_stage = 'waypoint'
            attempt = self._mp_decompose_count
            final = dict(final)

        def _decompose():
            try:
                from math import atan2
                with self._pose_lock:
                    rx = self._pose['x_m']
                    ry = self._pose['y_m']
                mx = (rx + final['x']) / 2.0
                my = (ry + final['y']) / 2.0
                myaw = atan2(final['y'] - ry, final['x'] - rx)
                wx, wy, wyaw = self._nudge_goal_from_walls(mx, my, myaw)
                self.get_logger().info(
                    f'multipoint {attempt}/{self._mp_decompose_max}: staging '
                    f'via waypoint ({wx:.2f},{wy:.2f}) toward final '
                    f'({final["x"]:.2f},{final["y"]:.2f})')
                self.clear_costmaps()
                time.sleep(0.3)
                self._publish_raw_goal(wx, wy, wyaw)
            except Exception as exc:
                self.get_logger().warning(f'multipoint decompose failed: {exc}')
                with self._mp_lock:
                    self._mp_stage = 'idle'
                    self._mp_final_goal = None

        threading.Thread(target=_decompose, daemon=True,
                         name='multipoint-decompose').start()

    def _mp_resume_final(self, final: dict) -> None:
        """Re-issue the final goal after an intermediate waypoint succeeded."""
        self.get_logger().info(
            f'multipoint: waypoint reached, resuming final goal '
            f'({final["x"]:.2f},{final["y"]:.2f})')
        time.sleep(0.3)
        self._publish_raw_goal(final['x'], final['y'], final['yaw'])

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
            if not self._map_info:
                return None
            info = dict(self._map_info)
            info.update({
                'map_id': self._map_fingerprint,
                'map_fingerprint': self._map_fingerprint,
                'map_mode': self._map_mode,
                'map_yaml': self._active_map_yaml,
                'pgm': self._active_map_pgm,
                'manifest': self._active_manifest_path,
                'map_hash': self._active_map_hash,
            })
            return info

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
          AUTO    — clear the Pico E-STOP latch; silence manual velocity
                    streams so Nav2 gets exclusive control.
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
            zero = Twist()
            self._cmd_vel_pub.publish(zero)
            self._cmd_vel_manual_pub.publish(zero)
            with self._control_lock:
                self._last_control_time = 0.0
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
            'nav_bypass_active': self._nav_bypass_active,
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
        """Publish a Nav2 goal and hand command authority to Nav2.

        The mobile app streams joystick frames continuously. If a goal is
        accepted while the bridge is still in MANUAL, those frames keep
        publishing zero/manual Twist messages and can fight the Nav2
        controller. Treat every nav goal as an AUTO transition first so the
        goal, plan, and vehicle command stream are one coherent operation.
        """
        with self._mode_lock:
            mode = self._mode
        if mode != 'AUTO':
            if not self.set_mode('AUTO'):
                return False
        else:
            if self._estop_latched:
                self._clear_pico_estop()

        zero = Twist()
        self._cmd_vel_pub.publish(zero)
        self._cmd_vel_manual_pub.publish(zero)
        with self._control_lock:
            self._last_control_time = 0.0

        # Pre-nudge goal off walls so the robot doesn't park itself into
        # an inflation halo that makes the *next* goal unplannable.
        nx, ny, nyaw = self._nudge_goal_from_walls(
            float(x), float(y), float(yaw))

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(nx)
        msg.pose.position.y = float(ny)
        from math import cos as mcos, sin as msin
        msg.pose.orientation.z = msin(float(nyaw) / 2.0)
        msg.pose.orientation.w = mcos(float(nyaw) / 2.0)
        send_stamp = time.time()
        with self._goal_lock:
            self._current_goal = {
                'x': float(nx),
                'y': float(ny),
                'yaw': float(nyaw),
                'active': True,
                'stamp': send_stamp,
            }

        # Track the goal for the auto-retry path; reset retry counter
        # whenever a brand-new goal is sent.
        with self._retry_lock:
            self._last_goal_send = {
                'x': float(nx), 'y': float(ny), 'yaw': float(nyaw),
                'stamp': send_stamp,
            }
            self._retry_count = 0

        # Multi-point planner: track the user's real goal so a later ABORT
        # can decompose it via an intermediate open-space waypoint.
        with self._mp_lock:
            self._mp_final_goal = {'x': float(nx), 'y': float(ny),
                                   'yaw': float(nyaw)}
            self._mp_stage = 'final'
            self._mp_decompose_count = 0

        # Wall-parked escape: if the robot is starting next to a wall, engage
        # the near-wall bypass now so SMAC's first plan already sees the
        # relaxed inflation. The near-wall monitor maintains it for the rest
        # of the trip and releases it once the chassis is clear of walls.
        try:
            d0 = self._near_wall_distance()
            if d0 is not None and d0 < self._near_wall_enter_m:
                self._engage_near_wall_bypass(True)
        except Exception as exc:
            self.get_logger().warning(f'near-wall pre-goal check failed: {exc}')

        self._goal_pub.publish(msg)
        self.get_logger().info(
            f'Published nav goal: ({nx:.2f}, {ny:.2f}, yaw={nyaw:.2f})')
        return True

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
        # User-initiated cancel must clear the auto-retry state so the
        # ABORT that the cancel triggers does NOT re-publish the goal.
        with self._retry_lock:
            self._last_goal_send = None
            self._retry_count = 0

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
    nav_status, _ = bridge_node.get_nav_status()
    return jsonify({
        'pose': {**pose, 'source': source},
        'scan': {
            'count': len(scan_points),
            'age_s': round(time.time() - scan_stamp, 2) if scan_stamp > 0 else None,
        },
        'map': map_info,
        'markers': {str(k): v for k, v in markers.items()},
        'nav_status': nav_status,
        # True while a Nav2 goal is executing and the chassis is following
        # /cmd_vel_smoothed (collision_monitor bypassed). Diagnostic-only;
        # the bridge's own SetParameters response is the source of truth.
        'nav_bypass_active': bridge_node._nav_bypass_active,
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
    if not bridge_node.send_nav_goal(data['x'], data['y'], yaw):
        return jsonify({'error': 'failed to enter AUTO mode'}), 409
    return jsonify({'status': 'ok', 'mode': bridge_node.get_mode()})


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
#  Robot dimensions live edit (URDF + Nav2 footprint)
# ---------------------------------------------------------------------------

@flask_app.route('/api/robot_config', methods=['GET', 'POST'])
def api_robot_config():
    """GET: current effective robot dimensions (defaults + persisted overrides).
    POST: {chassis_length?, chassis_width?, chassis_height?, wheelbase?,
           lidar_x?, lidar_y?, lidar_z?, camera_x?, camera_z?,
           footprint_padding?} — render URDF + sync costmaps + persist."""
    if _build_urdf is None:
        return jsonify({'error': 'build_urdf module unavailable'}), 500
    if request.method == 'GET':
        dims = bridge_node.get_robot_dimensions()
        return jsonify({'dims': dims,
                        'defaults': dict(_build_urdf.DEFAULT_DIMENSIONS),
                        'footprint': _build_urdf.footprint_string(
                            dims.get('chassis_length',
                                     _build_urdf.DEFAULT_DIMENSIONS['chassis_length']),
                            dims.get('chassis_width',
                                     _build_urdf.DEFAULT_DIMENSIONS['chassis_width']))})
    data = request.get_json(silent=True) or {}
    overrides = {}
    for k in _build_urdf.DIM_KEYS:
        if k in data:
            try:
                overrides[k] = float(data[k])
            except (TypeError, ValueError):
                return jsonify({'error': f'{k!r} must be numeric'}), 400
    if not overrides:
        return jsonify({'error': 'no editable dimension keys in request',
                        'editable_keys': list(_build_urdf.DIM_KEYS)}), 400
    result = bridge_node.apply_robot_dimensions(overrides)
    return jsonify(result)


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


@flask_app.route('/api/reset_overrides', methods=['POST'])
def api_reset_overrides():
    """Wipe runtime_overrides.yaml so the PC config files (launch args + nav2
    params) become the single source of truth again. Clears stale phone-saved
    calibration/tunables that were silently overriding the PC config on launch.
    Takes effect on the next relaunch."""
    res = bridge_node.reset_runtime_overrides()
    res['relaunch_required'] = True
    return jsonify(res), (200 if res.get('ok') else 500)


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
        include_stale = str(request.args.get('include_stale', '')).lower() \
            in ('1', 'true', 'yes', 'on')
        return jsonify({'waypoints': bridge_node.list_waypoints(include_stale),
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
        'map_id': wp.get('map_id') or wp.get('map_fingerprint', ''),
        'map_fingerprint': wp.get('map_fingerprint', ''),
        'map_yaml': wp.get('map_yaml', ''),
        'manifest': wp.get('manifest', ''),
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
    if not bridge_node.send_nav_goal(data['x'], data['y'], data.get('yaw', 0.0)):
        return jsonify({'error': 'failed to enter AUTO mode'}), 409
    return jsonify({'status': 'ok'})


@flask_app.route('/api/lock_map', methods=['POST'])
@flask_app.route('/api/save_map', methods=['POST'])
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
    yaml_path = prefix + '.yaml'
    pgm_path = prefix + '.pgm'
    try:
        pkg = bridge_node._promote_active_session_to_saved_map(yaml_path)
    except Exception as exc:
        return jsonify({
            'error': f'map saved but package manifest failed: {exc}',
            'yaml': yaml_path,
            'pgm': pgm_path,
            'stdout': proc.stdout[-2000:],
        }), 500
    return jsonify({
        'status': 'ok',
        'map_prefix': prefix,
        'yaml': yaml_path,
        'pgm': pgm_path,
        'map_id': pkg.get('map_id', ''),
        'manifest': pkg.get('manifest', ''),
        'stdout': proc.stdout[-2000:],
    })


# ---------------------------------------------------------------------------
#  Part E0 — Path planner mode (standard / multipoint)
# ---------------------------------------------------------------------------

@flask_app.route('/api/planner_mode', methods=['GET', 'POST'])
def api_planner_mode():
    """GET/POST the path-planner mode.

    'standard'   — single SMAC Hybrid-A* goal; on ABORT, one same-goal retry.
    'multipoint' — on ABORT, decompose: drive to an intermediate open-space
                   waypoint, then re-issue the final goal. Persists across
                   relaunches in ~/.autonexa/planner_mode.txt."""
    if request.method == 'GET':
        return jsonify({'planner_mode': bridge_node.get_planner_mode()})
    data = request.get_json(silent=True) or {}
    mode = str(data.get('planner_mode', '')).strip()
    if not bridge_node.set_planner_mode(mode):
        return jsonify(
            {'error': "planner_mode must be 'standard' or 'multipoint'"}), 400
    return jsonify({'planner_mode': mode})


@flask_app.route('/api/relocalize_auto', methods=['GET', 'POST'])
def api_relocalize_auto():
    """GET: current auto-relocalize config.
    POST {enabled?: bool, interval_s?: number in [2, 600]} — periodically
    calls AMCL request_nomotion_update to re-settle the particle filter
    (counters drift on featureless walls). Localization mode only; a no-op in
    live-SLAM mode (no /amcl). Persisted across relaunches; default off."""
    if request.method == 'GET':
        return jsonify(bridge_node.get_relocalize_auto())
    data = request.get_json(silent=True) or {}
    interval = data.get('interval_s')
    if interval is not None:
        try:
            interval = float(interval)
        except (TypeError, ValueError):
            return jsonify({'error': 'interval_s must be a number'}), 400
        if not 2.0 <= interval <= 600.0:
            return jsonify({'error': 'interval_s must be in [2, 600]'}), 400
    return jsonify(bridge_node.set_relocalize_auto(
        enabled=data.get('enabled'), interval_s=interval))


@flask_app.route('/api/ekf_mode', methods=['GET', 'POST'])
def api_ekf_mode():
    """GET: current use_ekf flag.
    POST {enabled: bool} — persist the encoder->EKF odometry-fusion launch
    flag (~/.autonexa/use_ekf.txt). The EKF owns the odom->base_link TF, a
    launch-time choice, so this takes effect on the NEXT relaunch. Disabling +
    relaunch returns to scan-matcher-owned TF (today's behavior)."""
    if request.method == 'GET':
        return jsonify({'use_ekf': bridge_node.get_use_ekf()})
    data = request.get_json(silent=True) or {}
    if 'enabled' not in data:
        return jsonify({'error': 'enabled (bool) required'}), 400
    enabled = bool(data['enabled'])
    ok = bridge_node.set_use_ekf(enabled)
    return jsonify({'use_ekf': enabled, 'persisted': ok,
                    'relaunch_required': True}), (200 if ok else 500)


# ---------------------------------------------------------------------------
#  Part E — Nav2 max linear speed
# ---------------------------------------------------------------------------

@flask_app.route('/api/nav2_speed', methods=['GET', 'POST'])
def api_nav2_speed():
    """Live-tune Nav2's linear speed cap. Sets the ACTIVE controller's
    speed-cap param (MPPI FollowPath.vx_max or RPP
    FollowPath.desired_linear_vel) and velocity_smoother max_velocity[0] in
    lockstep so the smoother doesn't override the controller. Persists to
    runtime_overrides.yaml under the respective node sections."""
    param = bridge_node.detect_ctrl_speed_param()
    if request.method == 'GET':
        ctrl = bridge_node.get_remote_params('/controller_server', [param])
        sm = bridge_node.get_remote_params(
            '/velocity_smoother', ['max_velocity'])
        desired = ctrl.get(param)
        return jsonify({
            'controller_speed_param': param,
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
    if not 0.16 <= target <= 0.25:
        return jsonify({'error': 'max_vel_x must be in [0.16, 0.25]'}), 400
    ctrl = bridge_node.set_remote_params('/controller_server', {param: target})
    # If the active controller changed since last detection (relaunch into the
    # other controller), the param name is now wrong — re-detect once + retry.
    if not (ctrl.get(param) or {}).get('ok', False):
        param = bridge_node.detect_ctrl_speed_param(force=True)
        ctrl = bridge_node.set_remote_params('/controller_server', {param: target})
    # velocity_smoother expects a 3-vector [vx, vy, wz]; preserve current vy/wz.
    cur = bridge_node.get_remote_params('/velocity_smoother', ['max_velocity'])
    vec = list(cur.get('max_velocity') or [target, 0.0, 0.5])
    vec[0] = target
    sm = bridge_node.set_remote_params(
        '/velocity_smoother', {'max_velocity': [float(v) for v in vec]})
    bridge_node.persist_runtime_overrides('controller_server', {param: target})
    bridge_node.persist_runtime_overrides(
        'velocity_smoother', {'max_velocity': [float(v) for v in vec]})
    return jsonify({'controller': ctrl, 'smoother': sm,
                    'controller_speed_param': param,
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
    # Persist *all* attempted items, not just the ones the live node
    # accepted. Some Nav2 plugins (notably SmacPlannerHybrid on
    # /planner_server) reject SetParameters at runtime even for params
    # that *are* settable at configure time — saving them to the YAML
    # means a launch restart picks them up. The response below tells the
    # app whether a restart is needed so the user knows to relaunch.
    bridge_node.persist_runtime_overrides(node.lstrip('/'), items)
    rejected = [k for k, r in results.items() if not r.get('ok')]
    restart_required = bool(rejected)
    return jsonify({
        'node': node,
        'results': results,
        'persisted': list(items.keys()),
        'rejected': rejected,
        'restart_required': restart_required,
    })


# ---------------------------------------------------------------------------
#  Part G3 — Topic / node health
# ---------------------------------------------------------------------------

@flask_app.route('/api/health')
def api_health():
    return jsonify({'topics': bridge_node.get_health()})


@flask_app.route('/api/system_status')
def api_system_status():
    return jsonify(bridge_node.get_system_status())


@flask_app.route('/api/command_profiles')
def api_command_profiles():
    return jsonify(bridge_node.get_command_profiles())


@flask_app.route('/api/command_audit')
def api_command_audit():
    return jsonify({'audit': bridge_node.get_command_audit()})


# ---------------------------------------------------------------------------
#  Desktop screenshot stream (1 Hz)
# ---------------------------------------------------------------------------

@flask_app.route('/api/desktop_version')
def api_desktop_version():
    """Cheap version probe for ETag-style polling. Bumps every time
    capture_desktop_once() succeeds (~1 Hz). Clients only refetch the
    JPEG when v changes."""
    _, v, stamp = bridge_node.get_desktop_jpeg()
    return jsonify({'v': v, 'stamp': stamp})


@flask_app.route('/api/desktop_shot')
def api_desktop_shot():
    """Latest desktop screenshot as JPEG bytes (~80–200 KB at 720p q=60)."""
    jpeg, _, _ = bridge_node.get_desktop_jpeg()
    if jpeg is None:
        return jsonify({'error': 'No desktop capture available yet'}), 503
    return Response(jpeg, mimetype='image/jpeg')


def _ws_send_json(ws, payload: dict) -> bool:
    payload.setdefault('timestamp', _now_stamp())
    try:
        ws.send(json.dumps(payload))
        return True
    except Exception:
        return False


def _terminate_process(proc: subprocess.Popen, force: bool = False) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        if os.name == 'posix':
            os.killpg(proc.pid, signal.SIGKILL if force else signal.SIGTERM)
        elif force:
            proc.kill()
        else:
            proc.terminate()
    except ProcessLookupError:
        pass
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _drain_command_queue(ws, out_q: queue.Queue, run_id: str) -> bool:
    ok = True
    while True:
        try:
            stream, text = out_q.get_nowait()
        except queue.Empty:
            break
        ok = _ws_send_json(ws, {
            'type': 'output',
            'run_id': run_id,
            'stream': stream,
            'text': text,
            'return_code': None,
        }) and ok
    return ok


def _run_external_command(ws, request_id: str, profile_id: str,
                          argv: list, values: dict, runtime_s: float) -> None:
    run_id = uuid.uuid4().hex[:12]
    if len(argv) >= 3 and argv[:3] == ['ros2', 'topic', 'pub']:
        _ws_send_json(ws, {
            'type': 'error',
            'request_id': request_id,
            'text': 'motion publishing is not available from this console',
        })
        return

    if not _ws_send_json(ws, {
        'type': 'started',
        'request_id': request_id,
        'run_id': run_id,
        'profile_id': profile_id,
        'argv': argv,
        'stream': 'system',
        'text': '',
        'return_code': None,
    }):
        return

    out_q = queue.Queue()

    def _reader(pipe, stream_name):
        try:
            for line in iter(pipe.readline, ''):
                if not line:
                    break
                out_q.put((stream_name, line))
        except Exception as exc:
            out_q.put(('stderr', f'[reader error] {exc}\n'))
        finally:
            try:
                pipe.close()
            except Exception:
                pass

    proc = None
    return_code = 127
    reason = ''
    try:
        proc = subprocess.Popen(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            shell=False,
            start_new_session=(os.name == 'posix'),
            env=dict(os.environ),
        )
        threading.Thread(
            target=_reader, args=(proc.stdout, 'stdout'), daemon=True).start()
        threading.Thread(
            target=_reader, args=(proc.stderr, 'stderr'), daemon=True).start()

        deadline = time.time() + max(1.0, runtime_s)
        while proc.poll() is None:
            if not _drain_command_queue(ws, out_q, run_id):
                reason = 'client_disconnected'
                _terminate_process(proc)
                break
            if time.time() >= deadline:
                reason = 'timeout'
                _ws_send_json(ws, {
                    'type': 'output',
                    'run_id': run_id,
                    'stream': 'system',
                    'text': f'[timeout after {runtime_s:.0f}s]\n',
                    'return_code': None,
                })
                _terminate_process(proc)
                break
            try:
                raw = ws.receive(timeout=0.05)
            except Exception:
                reason = 'client_disconnected'
                _terminate_process(proc)
                break
            if raw is None:
                continue
            try:
                msg = json.loads(raw)
            except (TypeError, ValueError):
                continue
            op = str(msg.get('op', '')).lower()
            if op in ('stop', 'kill'):
                reason = op
                _ws_send_json(ws, {
                    'type': 'output',
                    'run_id': run_id,
                    'stream': 'system',
                    'text': '[stop requested]\n',
                    'return_code': None,
                })
                _terminate_process(proc, force=(op == 'kill'))
                break
            if op == 'start':
                _ws_send_json(ws, {
                    'type': 'error',
                    'request_id': msg.get('request_id', ''),
                    'run_id': run_id,
                    'text': 'a command is already running in this tab',
                })

        if proc.poll() is None:
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                reason = reason or 'killed'
                _terminate_process(proc, force=True)
        try:
            return_code = proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            return_code = -9
        _drain_command_queue(ws, out_q, run_id)
    except FileNotFoundError as exc:
        reason = 'not_found'
        _ws_send_json(ws, {
            'type': 'output',
            'run_id': run_id,
            'stream': 'stderr',
            'text': f'{exc}\n',
            'return_code': None,
        })
    except Exception as exc:
        reason = 'error'
        _ws_send_json(ws, {
            'type': 'output',
            'run_id': run_id,
            'stream': 'stderr',
            'text': f'{exc}\n',
            'return_code': None,
        })
    finally:
        if proc is not None and proc.poll() is None:
            _terminate_process(proc, force=True)
        bridge_node.audit_command({
            'run_id': run_id,
            'profile_id': profile_id,
            'args': values,
            'return_code': return_code,
            'reason': reason,
        })
        _ws_send_json(ws, {
            'type': 'done',
            'request_id': request_id,
            'run_id': run_id,
            'profile_id': profile_id,
            'stream': 'system',
            'text': '',
            'return_code': return_code,
            'reason': reason,
        })


def _run_log_tail_command(ws, request_id: str, profile_id: str,
                          values: dict) -> None:
    run_id = uuid.uuid4().hex[:12]
    _ws_send_json(ws, {
        'type': 'started',
        'request_id': request_id,
        'run_id': run_id,
        'profile_id': profile_id,
        'stream': 'system',
        'text': '',
        'return_code': None,
    })
    result = bridge_node.tail_latest_ros_log(int(values.get('lines', 120)))
    return_code = 0 if result.get('ok') else 1
    if result.get('ok'):
        text = f"[{result.get('path')}]\n{result.get('text', '')}"
        stream = 'stdout'
    else:
        text = result.get('error', 'log tail failed') + '\n'
        stream = 'stderr'
    _ws_send_json(ws, {
        'type': 'output',
        'run_id': run_id,
        'stream': stream,
        'text': text,
        'return_code': None,
    })
    bridge_node.audit_command({
        'run_id': run_id,
        'profile_id': profile_id,
        'args': values,
        'return_code': return_code,
        'reason': '',
    })
    _ws_send_json(ws, {
        'type': 'done',
        'request_id': request_id,
        'run_id': run_id,
        'profile_id': profile_id,
        'stream': 'system',
        'text': '',
        'return_code': return_code,
    })


# =============================================================================
#  WebSocket endpoints — joystick (high-rate) + telemetry push
# =============================================================================

@sock.route('/ws/command')
def ws_command(ws):
    bridge_node.get_logger().info('WS /ws/command connected')
    try:
        while True:
            raw = ws.receive(timeout=30)
            if raw is None:
                continue
            try:
                data = json.loads(raw)
            except (TypeError, ValueError):
                _ws_send_json(ws, {
                    'type': 'error',
                    'text': 'invalid JSON request',
                })
                continue
            op = str(data.get('op', '')).lower()
            request_id = str(data.get('request_id', ''))
            if op == 'start':
                profile_id = str(data.get('profile_id', ''))
                try:
                    profile, argv, values, runtime_s = _build_command(
                        profile_id, data.get('args') or {})
                except ValueError as exc:
                    _ws_send_json(ws, {
                        'type': 'error',
                        'request_id': request_id,
                        'profile_id': profile_id,
                        'text': str(exc),
                    })
                    continue
                if profile.get('kind') == 'log_tail':
                    _run_log_tail_command(ws, request_id, profile_id, values)
                else:
                    _run_external_command(
                        ws, request_id, profile_id, argv, values, runtime_s)
            elif op in ('stop', 'kill'):
                _ws_send_json(ws, {
                    'type': 'idle',
                    'request_id': request_id,
                    'text': 'no command is running',
                })
            else:
                _ws_send_json(ws, {
                    'type': 'error',
                    'request_id': request_id,
                    'text': "op must be 'start', 'stop', or 'kill'",
                })
    except Exception as exc:
        bridge_node.get_logger().info(f'WS /ws/command closed: {exc}')


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


def _desktop_capture_loop():
    """Background thread: capture the GNOME desktop once per second and
    cache the JPEG. Failures are logged once via capture_desktop_once and
    silently retried on the next tick (the Pi's screenshot portal can
    blip during heavy load)."""
    period = 1.0
    while True:
        time.sleep(period)
        if bridge_node is None:
            continue
        try:
            bridge_node.capture_desktop_once()
        except Exception:
            # Last-resort safety net — never let this loop crash the bridge.
            pass


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


@flask_app.route('/desktop')
@flask_app.route('/desktop/')
def desktop_index():
    if not os.path.isdir(DESKTOP_WEB_DIR):
        return jsonify({
            'error': 'desktop web assets not installed',
            'path': DESKTOP_WEB_DIR,
        }), 404
    return send_from_directory(DESKTOP_WEB_DIR, 'index.html')


@flask_app.route('/desktop/<path:filename>')
def desktop_asset(filename):
    if not os.path.isdir(DESKTOP_WEB_DIR):
        return jsonify({
            'error': 'desktop web assets not installed',
            'path': DESKTOP_WEB_DIR,
        }), 404
    return send_from_directory(DESKTOP_WEB_DIR, filename)


@flask_app.route('/')
def index():
    return '''<!doctype html>
<html><head><title>AutoNexa ROS2 Bridge</title></head>
<body style="font-family:sans-serif;background:#111;color:#eee;padding:20px">
<h1>AutoNexa ROS2 Mobile Bridge</h1>
<p><a href="/desktop">Open Desktop Operator Console</a></p>
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
<li>POST /api/reset_overrides — wipe runtime_overrides.yaml (PC config wins)</li>
<li>POST /api/clear_costmaps · POST /api/restart_mapping</li>
<li>GET/POST/DELETE /api/waypoints — manual park/summon spots</li>
<li>POST /api/waypoints/&lt;name&gt;/navigate</li>
<li>GET/POST /api/spots · DELETE /api/spots/&lt;id&gt; — parking-plan static spots</li>
<li>POST /api/save_spot · POST /api/park_at · POST /api/summon</li>
<li>POST /api/lock_map — save current SLAM map under ~/.autonexa/maps</li>
<li>GET/POST /api/nav2_speed — live Nav2 target-speed slider</li>
<li>GET/POST /api/planner_mode — standard | multipoint path planner</li>
<li>GET/POST /api/relocalize_auto — periodic AMCL re-settle (localization mode)</li>
<li>GET/POST /api/ekf_mode — toggle encoder->EKF odom fusion (relaunch to apply)</li>
<li>POST /api/relocalize — set robot pose (x,y,yaw)</li>
<li>GET/POST /api/params?node=&lt;name&gt; — live param tuner (whitelist only)</li>
<li><a href="/api/health">/api/health</a> — topic rates / staleness</li>
<li><a href="/api/system_status">/api/system_status</a> — Pi + bridge status</li>
<li><a href="/api/command_profiles">/api/command_profiles</a> — guarded desktop command profiles</li>
<li><a href="/api/desktop_shot">/api/desktop_shot</a> — 1 Hz GNOME desktop JPEG (Wayland-friendly)</li>
<li><a href="/api/desktop_version">/api/desktop_version</a> — ETag counter for desktop_shot</li>
<li>WS /ws/control — joystick stream (input-only)</li>
<li>WS /ws/telemetry — server-pushed telemetry snapshots @ 10 Hz</li>
<li>WS /ws/command — guarded command runner for the desktop console</li>
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

    # Desktop capture loop disabled: 1 Hz gnome-screenshot calls were crashing
    # GNOME Shell. Desktop tab is non-essential. Re-enable behind a flag if needed.

    bridge_node.get_logger().info('Starting Flask server on http://0.0.0.0:5000')
    flask_app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
