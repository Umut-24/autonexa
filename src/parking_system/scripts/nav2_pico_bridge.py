#!/usr/bin/env python3
"""
Nav2 → Pico ASCII serial bridge for the AutoNexa CLI firmware.

Subscribes /cmd_vel, applies vx/wz limits + acceleration caps + 200 ms
watchdog, computes Ackermann inverse kinematics, and writes
SPEEDS + SERVO_PWM lines over /dev/ttyACM0 at 30 Hz.

Calibrated servo center and the hard servo bounds [us_min, us_max] live
in this bridge (RPi5 side), so the Pico's CLI firmware
(autonexa_pico.uf2) does not need any reflash.

Mutual exclusion: takes the serial port with `exclusive=True` plus an
fcntl lock at /tmp/nav2_pico_bridge.lock, so it cannot run at the same
time as test/pico_gui.py or another instance of itself.

Sign convention: standard Ackermann + standard ROS — positive
angular.z = left turn = front wheels rotate counter-clockwise (top
view). The firmware's bench-GUI calibration empirically gives
"servo µs < center = wheels turn left", so the mapping negates steer
before mapping to µs. Set `servo_polarity:=-1` to flip if your hardware
is mounted opposite. Set `reverse_steer_polarity:=-1` when forward
turns are correct but reverse-left/reverse-right are swapped.

Lifecycle:
    on launch: open serial → send `ENABLE` (if auto_enable=true)
    every tick:  send `SPEEDS <left> <right>` then (if changed) `SERVO_PWM <us>`
    on watchdog: ramp output to (0, 0); next tick sends SPEEDS 0 0
    on shutdown: send `STOP` → `DISABLE` → `SERVO_PWM <center>`
"""
import fcntl
import math
import os
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

try:
    import serial
except ImportError:
    raise SystemExit("pyserial required: sudo apt install python3-serial")

try:
    import yaml
except ImportError:
    yaml = None  # YAML overrides become a no-op if PyYAML is missing.

# Persistent overrides written by the mobile bridge's /api/calibrate_direction
# and /api/params endpoints. Loaded on startup *after* launch params so the
# user's last-known-good calibration survives a relaunch.
RUNTIME_OVERRIDES_PATH = os.path.expanduser('~/.autonexa/runtime_overrides.yaml')

# Calibration is physical to THIS chassis: which way is forward, which way
# the steering turns, and the servo's mechanical bounds. These are worth
# remembering across relaunches, so the mobile app persists them and the
# bridge replays them on startup (see _apply_runtime_overrides).
RUNTIME_CALIBRATION = (
    'vx_polarity',
    'servo_polarity',
    'reverse_steer_polarity',
    'servo_center_us',
    'servo_us_min',
    'servo_us_max',
)

# Tunables (speed/accel caps, creep gate, steer slew, manual window) are
# NOT replayed on startup. The PC config (launch args + nav2 params) is the
# single source of truth for these, so a stale phone-saved value can't
# silently override it on the next launch — that was the cause of the
# "car randomly drives reverse / uses a different speed after relaunch"
# behaviour. They remain live-settable for the running session via
# SetParameters (the app's sliders still work); they just reset to the PC
# values on the next relaunch.
RUNTIME_TUNABLE = (
    'max_vx_mps',
    'max_wz_radps',
    'max_ax_mps2',
    'max_aw_radps2',
    'max_steer_rate_radps',
    'min_vx_creep',
    'steer_ref_speed_mps',
    'track_width_m',
    'manual_priority_window_s',
    # nav_bypass_active is intentionally in neither tuple. It's a transient
    # runtime mode, not a configuration choice: the mobile bridge sets it
    # true while a Nav2 goal is executing and false again when the goal
    # completes. Persisting "always bypass" across a relaunch would be a
    # foot-gun (collision_monitor would stay off even with no goal active).
)

# Everything the bridge will accept an override for (union). Used only to
# bound what a hand-edited YAML can touch; the startup-replay path applies
# the RUNTIME_CALIBRATION subset only.
RUNTIME_OVERRIDABLE = RUNTIME_CALIBRATION + RUNTIME_TUNABLE


@dataclass
class MotionState:
    vx: float = 0.0
    wz: float = 0.0


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Nav2PicoBridge(Node):
    def __init__(self) -> None:
        super().__init__('nav2_pico_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # Optional second cmd_vel topic for the mobile app's safety-bypass
        # path (`/cmd_vel_manual`). Empty string disables the subscription.
        # Both topics drive the same Twist target — freshest msg wins via the
        # existing 200 ms watchdog. Routing decision is made by the publisher
        # (mobile bridge), not here.
        self.declare_parameter('manual_cmd_vel_topic', '/cmd_vel_manual')
        # When a /cmd_vel_manual message arrived within this window, prefer
        # it over /cmd_vel_safe (zero-frames from collision_monitor would
        # otherwise clobber the joystick at 20 Hz). 0.30 s gives the 50 Hz
        # joystick stream comfortable margin without permanently locking
        # out auto control after a stale manual frame.
        self.declare_parameter('manual_priority_window_s', 0.30)
        # /cmd_vel_smoothed is the velocity_smoother's output BEFORE
        # collision_monitor. When `nav_bypass_active` is true (mobile
        # bridge sets this while a Nav2 goal is executing), the bridge
        # follows /cmd_vel_smoothed instead of /cmd_vel_safe — the
        # chassis tracks the planner's path through what would otherwise
        # be a hard-stop, and only an operator E-STOP halts it. Default
        # off; never persisted to runtime_overrides.yaml on purpose.
        self.declare_parameter('smoothed_cmd_vel_topic', '/cmd_vel_smoothed')
        self.declare_parameter('nav_bypass_active', False)
        # Wheel-odometry readback: the Pico's CLI firmware emits a `TEL`
        # line carrying encoder-integrated odometry. The bridge parses it
        # and republishes as nav_msgs/Odometry. Topic name /pico/odom
        # matches what ros2_mobile_bridge + the EKF config already expect.
        self.declare_parameter('publish_wheel_odom', True)
        self.declare_parameter('wheel_odom_topic', '/pico/odom')
        self.declare_parameter('publish_motor_debug', True)
        self.declare_parameter('motor_debug_topic', '/pico/motor_debug')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('command_timeout_s', 0.20)

        self.declare_parameter('max_vx_mps', 0.25)
        self.declare_parameter('max_wz_radps', 0.8)
        self.declare_parameter('max_ax_mps2', 0.60)
        self.declare_parameter('max_aw_radps2', 0.50)

        # Commands below the floor-load movement band are silenced before they
        # reach the Pico. 0.10 starts the firmware kick earlier than the old
        # 0.15 gate, while Nav2's controller floors keep normal cruising above
        # the unstable low-speed band.
        self.declare_parameter('min_vx_creep', 0.10)
        # Reference speed for the Ackermann steering inverse. The denominator of
        # atan(L*wz/vx) is floored to this magnitude so steering stays continuous
        # and proportional to wz at low/zero speed instead of snapping to max,
        # and the drive direction (for reverse_steer_polarity) only commits past
        # +-this band — killing the mid-manoeuvre steering sign flips.
        self.declare_parameter('steer_ref_speed_mps', 0.06)

        self.declare_parameter('wheelbase_m', 0.25)
        self.declare_parameter('track_width_m', 0.20)
        # 120 = 30 / MOTOR_V_MAX_MPS(0.25). The firmware is now CLOSED-LOOP: it
        # maps SPEED -> a TARGET wheel speed (target = SPEED/30 * MOTOR_V_MAX_MPS)
        # and a velocity PI hits it. So for the commanded vx to equal the actual
        # speed end-to-end, scale must = max_speed_pulses / MOTOR_V_MAX_MPS:
        #   SPEED = vx*120 ; target = SPEED/30*0.25 = vx.  (vx 0.25 -> SPEED 30.)
        # KEEP THIS IN LOCKSTEP with pico config.h MOTOR_V_MAX_MPS: if that
        # changes, set scale = 30 / MOTOR_V_MAX_MPS. (Was 100 = open-loop duty map.)
        self.declare_parameter('vel_to_speed_scale', 120.0)
        self.declare_parameter('max_speed_pulses', 30)

        self.declare_parameter('servo_center_us', 1650)
        # ±525 µs from the calibrated 1650 µs mechanical-zero (was ±500). MG995
        # rotor moves ~0.090°/µs, so this opens an extra ±2.25° of rotor swing
        # (~±0.75° wheel via the Hiwonder Ackermann linkage) — modest enough
        # to keep the servo well inside its mechanical envelope while giving
        # the planner a tighter min turning radius to work with.
        self.declare_parameter('servo_us_min', 1125)
        self.declare_parameter('servo_us_max', 2175)
        self.declare_parameter('servo_max_steer_rad', 0.5236)
        # Default +1 on this chassis (hardware-verified 2026-06-01): ROS-positive
        # wz (left turn) maps to servo us < center, matching _steer_to_servo_us's
        # documented convention. Was -1, which steered the wrong way in both the
        # joystick and Nav2 (same code path). Override to -1 only if a linkage
        # rebuild later inverts the forward steering direction.
        self.declare_parameter('servo_polarity', +1)
        # -1 matches this chassis' reverse maneuvering; live-SLAM, AMCL, and
        # standalone bridge launch files all pass the same default.
        self.declare_parameter('reverse_steer_polarity', -1)
        # +1 = ROS-positive vx drives the chassis forward (standard).
        # -1 = forward/back swapped (use when motor wiring is reversed or the
        # LiDAR-defined map frame is yaw-flipped). Calibration wizard in the
        # mobile app toggles this live; persists via runtime_overrides.yaml.
        self.declare_parameter('vx_polarity', 1)
        # Servo slew-rate cap (rad/s). Lowered again 2.0 → 1.5 because
        # snake on straight paths still showed steering spikes faster
        # than the chassis could usefully respond. MG995's loaded slew
        # is ~2.7 rad/s; capping at 1.5 leaves headroom for voltage sag
        # and physically smooths short-wavelength steering commands.
        self.declare_parameter('max_steer_rate_radps', 1.5)

        self.declare_parameter('auto_enable', True)
        self.declare_parameter('bridge_lock_file', '/tmp/nav2_pico_bridge.lock')
        self.declare_parameter('dry_run', False)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.serial_baud = int(self.get_parameter('serial_baud').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.manual_priority_window = float(
            self.get_parameter('manual_priority_window_s').value)
        self.nav_bypass_active = bool(
            self.get_parameter('nav_bypass_active').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_timeout = float(self.get_parameter('command_timeout_s').value)

        self.max_vx = float(self.get_parameter('max_vx_mps').value)
        self.max_wz = float(self.get_parameter('max_wz_radps').value)
        self.max_ax = float(self.get_parameter('max_ax_mps2').value)
        self.max_aw = float(self.get_parameter('max_aw_radps2').value)
        self.min_vx_creep = float(self.get_parameter('min_vx_creep').value)
        self.steer_ref_speed = float(self.get_parameter('steer_ref_speed_mps').value)
        # Committed drive direction for the steering inverse (hysteresis state).
        self._steer_dir = 1

        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.vel_scale = float(self.get_parameter('vel_to_speed_scale').value)
        self.max_speed = int(self.get_parameter('max_speed_pulses').value)

        self.servo_center = int(self.get_parameter('servo_center_us').value)
        self.servo_us_min = int(self.get_parameter('servo_us_min').value)
        self.servo_us_max = int(self.get_parameter('servo_us_max').value)
        self.servo_max_steer = float(self.get_parameter('servo_max_steer_rad').value)
        self.servo_polarity = int(self.get_parameter('servo_polarity').value)
        self.reverse_steer_polarity = int(self.get_parameter('reverse_steer_polarity').value)
        self.vx_polarity = int(self.get_parameter('vx_polarity').value)
        self.max_steer_rate = float(self.get_parameter('max_steer_rate_radps').value)

        self.auto_enable = bool(self.get_parameter('auto_enable').value)
        self.lock_path = str(self.get_parameter('bridge_lock_file').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        if not (self.servo_us_min <= self.servo_center <= self.servo_us_max):
            raise RuntimeError(
                f"servo_center_us ({self.servo_center}) must be within "
                f"[{self.servo_us_min}, {self.servo_us_max}]")
        if self.servo_polarity not in (-1, 1):
            raise RuntimeError(f"servo_polarity must be +1 or -1, got {self.servo_polarity}")
        if self.reverse_steer_polarity not in (-1, 1):
            raise RuntimeError(
                f"reverse_steer_polarity must be +1 or -1, got {self.reverse_steer_polarity}")
        if self.vx_polarity not in (-1, 1):
            raise RuntimeError(f"vx_polarity must be +1 or -1, got {self.vx_polarity}")

        # Per-topic targets + timestamps. Three sources, three slots:
        #   auto      = /cmd_vel_safe (post-collision_monitor — Nav2's
        #               normal safety-chain output)
        #   smoothed  = /cmd_vel_smoothed (pre-collision_monitor — the
        #               planner's output before bumper / FootprintApproach
        #               can interfere). Selected only when
        #               `nav_bypass_active` is true.
        #   manual    = /cmd_vel_manual (mobile app OFF safety mode joystick)
        # Selection precedence in on_timer(): manual (within window) →
        # smoothed (if bypass active) → auto. `self.target` is the
        # *selected* target for the current tick (used by tests / logs).
        self._auto_target = MotionState()
        self._smoothed_target = MotionState()
        self._manual_target = MotionState()
        self.target = MotionState()
        self.output = MotionState()
        self._auto_cmd_time = self.get_clock().now()
        self._smoothed_cmd_time = self.get_clock().now() - Duration(seconds=10.0)
        self._manual_cmd_time = self.get_clock().now() - Duration(seconds=10.0)
        self.last_cmd_time = self._auto_cmd_time
        self._last_active_source = 'auto'  # 'auto' | 'smoothed' | 'manual'
        self._last_us_sent = None
        self._last_speed_sent = None
        self._last_steer_sent_rad = 0.0
        self._last_steer_t = self.get_clock().now()
        self._last_vx_sign = 0          # +1, -1, or 0
        self._cusp_cooldown_end = None  # Time object; non-None ⇒ in cooldown
        self._lock_fh = None
        self._serial = None
        self._write_lock = threading.Lock()
        self._shutting_down = False
        self._rx_thread = None          # serial RX / TEL-parse thread

        self._acquire_lock()
        # Register the validation callback first so any overrides we replay
        # below are routed through it (and so cached attrs stay in sync with
        # the parameter store).
        self.add_on_set_parameters_callback(self._on_param_set)
        self._apply_runtime_overrides()

        # Wheel-odometry publisher — fed by the serial RX thread (_rx_loop)
        # parsing the Pico's TEL telemetry line. Must be set up *before*
        # _open_serial(): _open_serial() starts _rx_loop, which reads
        # publish_wheel_odom and publishes via _wheel_odom_pub.
        self.publish_wheel_odom = bool(self.get_parameter('publish_wheel_odom').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self._wheel_odom_pub = None
        if self.publish_wheel_odom:
            self._wheel_odom_pub = self.create_publisher(
                Odometry, str(self.get_parameter('wheel_odom_topic').value), 10)
        self.publish_motor_debug = bool(self.get_parameter('publish_motor_debug').value)
        self._motor_debug_pub = None
        if self.publish_motor_debug:
            self._motor_debug_pub = self.create_publisher(
                String, str(self.get_parameter('motor_debug_topic').value), 10)

        if not self.dry_run:
            self._open_serial()
            if self.auto_enable:
                self._send("ENABLE")

        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.on_auto_cmd_vel, 20)
        manual_topic = str(self.get_parameter('manual_cmd_vel_topic').value)
        if manual_topic and manual_topic != self.cmd_vel_topic:
            self.manual_sub = self.create_subscription(
                Twist, manual_topic, self.on_manual_cmd_vel, 20)
        smoothed_topic = str(self.get_parameter('smoothed_cmd_vel_topic').value)
        if smoothed_topic and smoothed_topic != self.cmd_vel_topic:
            self.smoothed_sub = self.create_subscription(
                Twist, smoothed_topic, self.on_smoothed_cmd_vel, 20)

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(dt, self.on_timer)

        mode = "DRY-RUN" if self.dry_run else "live"
        self.get_logger().info(
            f"Bridge up [{mode}]: {self.cmd_vel_topic} -> "
            f"{self.serial_port}@{self.serial_baud}, rate={self.publish_rate_hz:.1f}Hz, "
            f"max_vx={self.max_vx:.2f}m/s max_wz={self.max_wz:.2f}rad/s, "
            f"min_vx_creep={self.min_vx_creep:.3f}m/s, "
            f"servo center={self.servo_center}us in [{self.servo_us_min},{self.servo_us_max}], "
            f"vx_polarity={self.vx_polarity:+d} servo_polarity={self.servo_polarity:+d}, "
            f"reverse_steer_polarity={self.reverse_steer_polarity:+d}, "
            f"max_steer_rate={self.max_steer_rate:.2f}rad/s")

    # ── Lock + serial ─────────────────────────────────────────────
    def _acquire_lock(self) -> None:
        try:
            self._lock_fh = open(self.lock_path, 'w', encoding='utf-8')
            fcntl.flock(self._lock_fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_fh.write(f"pid={os.getpid()}\n")
            self._lock_fh.flush()
        except OSError as exc:
            raise RuntimeError(
                f"Another nav2_pico_bridge is already running "
                f"(lock: {self.lock_path}): {exc}") from exc

    def _release_lock(self) -> None:
        if self._lock_fh is None:
            return
        try:
            fcntl.flock(self._lock_fh.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            self._lock_fh.close()
        except OSError:
            pass
        self._lock_fh = None

    def _open_serial(self) -> None:
        try:
            # exclusive=True is Linux-only; fails fast if pico_gui.py or another
            # process has the port open.
            self._serial = serial.Serial(
                self.serial_port, self.serial_baud, timeout=0.1, exclusive=True)
        except (serial.SerialException, OSError) as exc:
            raise RuntimeError(f"open {self.serial_port} failed: {exc}") from exc
        time.sleep(0.2)  # let Pico flush boot banner
        # Start the RX thread that parses the Pico's TEL telemetry and
        # republishes wheel odometry. pyserial tolerates read on this
        # thread while _send() writes on the timer thread.
        if self.publish_wheel_odom or self.publish_motor_debug:
            self._rx_thread = threading.Thread(
                target=self._rx_loop, name='pico_tel_rx', daemon=True)
            self._rx_thread.start()

    def _send(self, line: str) -> bool:
        if self.dry_run:
            self.get_logger().info(f"DRY -> {line}")
            return True
        if self._serial is None:
            return False
        data = (line.rstrip("\n") + "\n").encode("ascii", errors="ignore")
        try:
            with self._write_lock:
                self._serial.write(data)
                self._serial.flush()
            return True
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"serial write failed ({line!r}): {exc}")
            return False

    # ── Serial RX: parse TEL telemetry → publish wheel odometry ──
    def _rx_loop(self) -> None:
        """Background thread: read serial lines, publish odometry from any
        `TEL` line. Exits when the bridge shuts down or the port dies."""
        while not self._shutting_down:
            if self._serial is None:
                time.sleep(0.1)
                continue
            try:
                raw = self._serial.readline()   # honours the 0.1 s timeout
            except (serial.SerialException, OSError) as exc:
                if not self._shutting_down:
                    self.get_logger().error(f"serial read failed: {exc}")
                return
            if not raw:
                continue                        # timeout, no full line
            line = raw.decode('ascii', errors='ignore').strip()
            if line.startswith('TEL '):
                self._publish_tel_odom(line[4:])
            elif line.startswith('MOT '):
                self._publish_motor_debug(line)

    def _publish_motor_debug(self, line: str) -> None:
        if self._motor_debug_pub is None:
            return
        msg = String()
        msg.data = line
        self._motor_debug_pub.publish(msg)

    def _publish_tel_odom(self, payload: str) -> None:
        """Parse the CSV body of a TEL line and publish nav_msgs/Odometry.

        TEL body (13 fields):
          ms,spdL,spdR,steer,encL,encR,x,y,yaw,vx,wz,estop,timeout
        """
        if self._wheel_odom_pub is None:
            return
        parts = payload.split(',')
        if len(parts) != 13:
            return
        try:
            x = float(parts[6])
            y = float(parts[7])
            yaw = float(parts[8])
            vx = float(parts[9])
            wz = float(parts[10])
        except (ValueError, IndexError):
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # 2D yaw → quaternion (roll = pitch = 0).
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz
        # Diagonal covariances. Pose: wheel dead-reckoning drifts, so
        # x/y/yaw are loose; unused 3D dims pinned huge. Twist: encoders
        # measure vx well, yaw-rate moderately (wheel slip).
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.02      # x
        pose_cov[7] = 0.02      # y
        pose_cov[14] = 1.0e6    # z
        pose_cov[21] = 1.0e6    # roll
        pose_cov[28] = 1.0e6    # pitch
        pose_cov[35] = 0.05     # yaw
        twist_cov = [0.0] * 36
        twist_cov[0] = 0.01     # vx
        twist_cov[7] = 1.0e6    # vy
        twist_cov[14] = 1.0e6   # vz
        twist_cov[21] = 1.0e6   # vroll
        twist_cov[28] = 1.0e6   # vpitch
        twist_cov[35] = 0.02    # vyaw
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self._wheel_odom_pub.publish(msg)

    # ── Runtime override + parameter callback ────────────────────
    def _apply_runtime_overrides(self) -> None:
        """Replay persisted overrides (~/.autonexa/runtime_overrides.yaml)
        through the normal SetParameters path so the change-callback runs
        and validates them. Silent no-op if the file is missing."""
        if yaml is None or not os.path.exists(RUNTIME_OVERRIDES_PATH):
            return
        try:
            with open(RUNTIME_OVERRIDES_PATH, 'r', encoding='utf-8') as fh:
                doc = yaml.safe_load(fh) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(
                f'failed reading {RUNTIME_OVERRIDES_PATH}: {exc}')
            return
        section = doc.get('nav2_pico_bridge') or {}
        if not isinstance(section, dict):
            return
        kv = []
        ignored_tunables = []
        for key, value in section.items():
            # Tunables are PC-authoritative — never replayed at startup.
            if key in RUNTIME_TUNABLE:
                ignored_tunables.append(key)
                continue
            if key not in RUNTIME_CALIBRATION:
                continue
            try:
                param = self.get_parameter(key)
            except Exception:
                continue
            try:
                kv.append(rclpy.parameter.Parameter(key, param.type_, value))
            except Exception as exc:
                self.get_logger().warning(
                    f'calibration override {key}={value!r} rejected: {exc}')
        if ignored_tunables:
            self.get_logger().warning(
                f'{RUNTIME_OVERRIDES_PATH} still lists tunables that are NO '
                f'LONGER replayed (PC config is authoritative): '
                f'{sorted(ignored_tunables)}. Change them live from the app, '
                f'or edit the PC config files. Use /api/reset_overrides to clear.')
        if kv:
            results = self.set_parameters(kv)
            applied = {k.name: k.value for k, r in zip(kv, results) if r.successful}
            # WARNING level on purpose: the operator should always be able to
            # see which physical-calibration values the phone file is applying
            # on top of the PC launch defaults.
            self.get_logger().warning(
                f'CALIBRATION overrides applied over launch defaults '
                f'(from {RUNTIME_OVERRIDES_PATH}): {applied}')

    def _on_param_set(self, params):
        """Validate + apply parameter changes pushed from the mobile bridge
        (or `ros2 param set`). Refuses out-of-range values; mirrors the
        accepted ones into the cached attributes used by the hot path."""
        for p in params:
            if p.name == 'vx_polarity':
                if p.value not in (-1, 1):
                    return SetParametersResult(
                        successful=False, reason='vx_polarity must be +1 or -1')
                self.vx_polarity = int(p.value)
            elif p.name == 'servo_polarity':
                if p.value not in (-1, 1):
                    return SetParametersResult(
                        successful=False, reason='servo_polarity must be +1 or -1')
                self.servo_polarity = int(p.value)
            elif p.name == 'reverse_steer_polarity':
                if p.value not in (-1, 1):
                    return SetParametersResult(
                        successful=False, reason='reverse_steer_polarity must be +1 or -1')
                self.reverse_steer_polarity = int(p.value)
            elif p.name == 'max_vx_mps':
                if not 0.0 < float(p.value) <= 1.0:
                    return SetParametersResult(
                        successful=False, reason='max_vx_mps out of range (0, 1.0]')
                self.max_vx = float(p.value)
            elif p.name == 'max_wz_radps':
                if not 0.0 < float(p.value) <= 4.0:
                    return SetParametersResult(
                        successful=False, reason='max_wz_radps out of range (0, 4.0]')
                self.max_wz = float(p.value)
            elif p.name == 'max_ax_mps2':
                if not 0.0 < float(p.value) <= 5.0:
                    return SetParametersResult(successful=False, reason='max_ax_mps2 out of range')
                self.max_ax = float(p.value)
            elif p.name == 'max_aw_radps2':
                if not 0.0 < float(p.value) <= 8.0:
                    return SetParametersResult(successful=False, reason='max_aw_radps2 out of range')
                self.max_aw = float(p.value)
            elif p.name == 'max_steer_rate_radps':
                if not 0.05 <= float(p.value) <= 20.0:
                    return SetParametersResult(
                        successful=False, reason='max_steer_rate_radps out of range [0.05, 20]')
                self.max_steer_rate = float(p.value)
            elif p.name == 'min_vx_creep':
                if not 0.0 <= float(p.value) <= 0.5:
                    return SetParametersResult(successful=False, reason='min_vx_creep out of range')
                self.min_vx_creep = float(p.value)
            elif p.name == 'steer_ref_speed_mps':
                if not 0.01 <= float(p.value) <= 0.5:
                    return SetParametersResult(
                        successful=False, reason='steer_ref_speed_mps out of range [0.01, 0.5]')
                self.steer_ref_speed = float(p.value)
            elif p.name == 'track_width_m':
                if not 0.05 <= float(p.value) <= 1.0:
                    return SetParametersResult(
                        successful=False, reason='track_width_m out of range [0.05, 1.0]')
                self.track_width = float(p.value)
            elif p.name == 'servo_center_us':
                if not 800 <= int(p.value) <= 2200:
                    return SetParametersResult(successful=False, reason='servo_center_us out of range')
                self.servo_center = int(p.value)
            elif p.name == 'servo_us_min':
                # Hard floor at 1100 µs — beyond this the linkage binds and
                # the servo stalls/draws current. Param tuner can trim *up*
                # from the 1150 default if the user wants to spare the servo.
                if not 1100 <= int(p.value) <= self.servo_center:
                    return SetParametersResult(
                        successful=False,
                        reason='servo_us_min must be in [1100, servo_center_us]')
                self.servo_us_min = int(p.value)
            elif p.name == 'servo_us_max':
                # Hard ceiling at 2200 µs — keeps a margin below the Pico
                # firmware's absolute 2500 µs mechanical limit. Param tuner
                # can trim *down* from the 2150 default to ease the servo.
                if not self.servo_center <= int(p.value) <= 2200:
                    return SetParametersResult(
                        successful=False,
                        reason='servo_us_max must be in [servo_center_us, 2200]')
                self.servo_us_max = int(p.value)
            elif p.name == 'manual_priority_window_s':
                if not 0.0 <= float(p.value) <= 5.0:
                    return SetParametersResult(
                        successful=False,
                        reason='manual_priority_window_s out of range [0, 5]')
                self.manual_priority_window = float(p.value)
            elif p.name == 'nav_bypass_active':
                new_val = bool(p.value)
                if new_val != self.nav_bypass_active:
                    # Loud on purpose — this is a real safety mode flip.
                    self.get_logger().warning(
                        f"nav-bypass {'ON' if new_val else 'OFF'}: chassis "
                        f"{'now follows /cmd_vel_smoothed (collision_monitor inactive)' if new_val else 'back to /cmd_vel_safe (collision_monitor active)'}")
                self.nav_bypass_active = new_val
        return SetParametersResult(successful=True)

    # ── Mapping math ──────────────────────────────────────────────
    def _vx_to_speed_pulses(self, vx: float) -> int:
        s = int(round(self.vx_polarity * vx * self.vel_scale))
        return clamp(s, -self.max_speed, +self.max_speed)

    def _vx_steer_to_wheel_speeds(self, vx: float, steer: float) -> tuple[int, int]:
        """Convert body vx + actual commanded steering angle to rear-wheel
        SPEED targets. Using per-wheel targets avoids rear-axle scrub in turns,
        which is especially costly on this torque-limited L298N drivetrain."""
        if abs(vx) < 1.0e-6:
            return 0, 0
        wz_eff = vx * math.tan(steer) / self.wheelbase
        v_left = vx - wz_eff * (self.track_width / 2.0)
        v_right = vx + wz_eff * (self.track_width / 2.0)
        return self._vx_to_speed_pulses(v_left), self._vx_to_speed_pulses(v_right)

    def _vx_wz_to_steer(self, vx: float, wz: float) -> float:
        """Ackermann inverse with a reference-speed floor + direction hysteresis.

        delta = atan(L * wz / vx) is the body-frame Ackermann inverse, but it is
        speed-dependent and BLOWS UP / is discontinuous as vx -> 0. The chassis
        spends most of a park/manoeuvre in exactly that low-speed band, and the
        old code papered over it with a hard pivot branch (snap to +-max steer
        when |vx|<0.01). That snap, plus the reverse_steer_polarity flip the
        instant vx grazed negative, made the servo flip sign mid-manoeuvre even
        when the intent was steady (the "steers right while parking" bug).

        Two changes remove that:
          1) Reference-speed floor: clamp the denominator magnitude to
             steer_ref_speed so steer = atan(L*wz/v_eff) stays continuous and
             proportional to wz at all speeds — no snap-to-max.
          2) Direction hysteresis: the drive direction used for the floor sign
             and for the reverse_steer_polarity flip only commits past
             +-steer_ref_speed and is HELD inside the band. So a transient vx
             dip toward/below zero during a forward manoeuvre no longer triggers
             a spurious reverse flip. reverse_steer_polarity (value -1) still
             applies, but only for a genuinely committed reverse.
        """
        ref = self.steer_ref_speed
        # Commit drive direction only past +-ref; hold within the dead band.
        if vx > ref:
            self._steer_dir = 1
        elif vx < -ref:
            self._steer_dir = -1
        # else: keep self._steer_dir (last committed direction)
        v_eff = self._steer_dir * max(abs(vx), ref)
        steer = math.atan(self.wheelbase * wz / v_eff)
        if self._steer_dir < 0:
            steer *= self.reverse_steer_polarity
        return clamp(steer, -self.servo_max_steer, +self.servo_max_steer)

    def _steer_to_servo_us(self, steer_rad: float) -> int:
        """Map Ackermann steering angle to calibrated servo µs.

        ROS+Ackermann: positive steer = wheels turn left.
        User's bench calibration: us < center = wheels turn left.
        So negate steer before mapping to µs. servo_polarity flips the
        whole relationship if hardware is mounted opposite.
        """
        s = clamp(steer_rad, -self.servo_max_steer, +self.servo_max_steer)
        s_for_us = -self.servo_polarity * s  # +polarity: standard ROS sign
        if s_for_us >= 0:
            us = self.servo_center + (s_for_us / self.servo_max_steer) \
                 * (self.servo_us_max - self.servo_center)
        else:
            us = self.servo_center + (s_for_us / self.servo_max_steer) \
                 * (self.servo_center - self.servo_us_min)
        return clamp(int(round(us)), self.servo_us_min, self.servo_us_max)

    # ── ROS callbacks ─────────────────────────────────────────────
    def on_auto_cmd_vel(self, msg: Twist) -> None:
        """Nav2 / safety-chain command on /cmd_vel_safe. Updates the auto
        slot only — the on_timer arbiter decides whether this slot or the
        manual slot drives the chassis this tick."""
        self._auto_target.vx = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self._auto_target.wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self._auto_cmd_time = self.get_clock().now()

    def on_manual_cmd_vel(self, msg: Twist) -> None:
        """App-bypass command on /cmd_vel_manual (OFF safety mode). Updates
        the manual slot. While the manual timestamp stays inside
        `manual_priority_window_s`, the arbiter ignores the auto slot."""
        self._manual_target.vx = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self._manual_target.wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self._manual_cmd_time = self.get_clock().now()

    def on_smoothed_cmd_vel(self, msg: Twist) -> None:
        """Pre-collision_monitor command on /cmd_vel_smoothed. Used only
        when `nav_bypass_active` is true (mobile bridge sets that while
        a Nav2 goal is executing). Always updates the slot regardless of
        bypass state — keeps the timestamp fresh so the arbiter can
        switch in the moment bypass flips on."""
        self._smoothed_target.vx = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self._smoothed_target.wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self._smoothed_cmd_time = self.get_clock().now()

    def _apply_rate_limit(self, current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def on_timer(self) -> None:
        if self._shutting_down:
            return
        now = self.get_clock().now()
        dt = 1.0 / max(1.0, self.publish_rate_hz)

        # Pick the active source. Precedence (highest → lowest):
        #   1. manual    — inside `manual_priority_window_s` always wins
        #   2. smoothed  — when nav_bypass_active (Nav2 goal executing,
        #                  mobile bridge has flipped the flag)
        #   3. auto      — /cmd_vel_safe via collision_monitor (default)
        manual_age = (now - self._manual_cmd_time).nanoseconds * 1e-9
        if manual_age <= self.manual_priority_window:
            source = 'manual'
            self.target = self._manual_target
            self.last_cmd_time = self._manual_cmd_time
        elif self.nav_bypass_active:
            source = 'smoothed'
            self.target = self._smoothed_target
            self.last_cmd_time = self._smoothed_cmd_time
        else:
            source = 'auto'
            self.target = self._auto_target
            self.last_cmd_time = self._auto_cmd_time
        if source != self._last_active_source:
            self.get_logger().info(
                f"cmd_vel source -> {source} (manual_age={manual_age:.2f}s, "
                f"window={self.manual_priority_window:.2f}s, "
                f"bypass={self.nav_bypass_active})")
            self._last_active_source = source

        command_stale = (now - self.last_cmd_time) > Duration(seconds=self.command_timeout)
        desired = MotionState(0.0, 0.0) if command_stale else self.target

        self.output.vx = self._apply_rate_limit(self.output.vx, desired.vx, self.max_ax * dt)
        self.output.wz = self._apply_rate_limit(self.output.wz, desired.wz, self.max_aw * dt)

        # Gate sub-deadband output to 0 — see min_vx_creep param doc.
        # Steering still tracks the commanded wz so the wheels keep pointing
        # toward the goal direction even while the chassis is coasting.
        gated_vx = 0.0 if abs(self.output.vx) < self.min_vx_creep else self.output.vx

        # Cusp detection: when vx changes sign (forward↔reverse), hold speed
        # at zero for 500 ms so the servo can swing to the new steering angle
        # before the wheels drive. Was 250 ms; bumped to 500 ms because at
        # the lower max_steer_rate_radps (1.5 rad/s) a full ±15° wheel swing
        # takes ~350 ms — 250 ms didn't leave any margin, so the chassis
        # rolled a few centimeters with stale steering at every cusp.
        cur_sign = (1 if gated_vx > 0 else (-1 if gated_vx < 0 else 0))
        if (self._last_vx_sign != 0 and cur_sign != 0
                and cur_sign != self._last_vx_sign):
            self._cusp_cooldown_end = now + Duration(seconds=0.50)
            self.get_logger().info(
                f"Cusp detected ({self._last_vx_sign:+d}→{cur_sign:+d}), "
                f"holding speed=0 for 500 ms")
        if cur_sign != 0:
            self._last_vx_sign = cur_sign
        if (self._cusp_cooldown_end is not None
                and now < self._cusp_cooldown_end):
            gated_vx = 0.0
        elif self._cusp_cooldown_end is not None:
            self._cusp_cooldown_end = None

        if source == 'manual':
            # Direct / RC-style steering for the manual (OFF safety) bypass:
            # map the joystick's wz straight to a steering angle, independent of
            # speed and with no reverse flip. Stick-left -> wheels point left,
            # always — the intuitive feel for hand-driving the car into a spot,
            # and it sidesteps the speed-dependent atan entirely (no jitter).
            #
            # Use the RAW desired wz, NOT self.output.wz: output.wz is ramped by
            # the angular accel cap (max_aw), which would make the servo crawl to
            # full lock over >1 s ("direksiyon yavaş dönüyor"). The accel cap is
            # for smoothing the chassis' yaw rate, not the steering servo, so for
            # manual we want the wheels to follow the stick immediately.
            manual_wz = desired.wz
            if self.max_wz > 1.0e-6:
                steer = clamp(
                    (manual_wz / self.max_wz) * self.servo_max_steer,
                    -self.servo_max_steer, +self.servo_max_steer)
            else:
                steer = 0.0
        else:
            steer = self._vx_wz_to_steer(self.output.vx, self.output.wz)
        # Servo slew-rate limiter — runs in steering-angle space (rad) so it
        # respects the actual mechanical limit of the servo, not the µs scale.
        # Applied here, after Ackermann math, so it bounds the output the
        # firmware sees regardless of how fast the upstream wz changes.
        # SKIPPED for manual: hand-driving wants the servo to respond as fast as
        # the hardware allows, not crawl at max_steer_rate (1.5 rad/s ≈ 0.35 s to
        # full lock). The servo's own mechanical slew is the only limit in manual.
        steer_dt = (now - self._last_steer_t).nanoseconds * 1e-9
        if source != 'manual' and steer_dt > 0.0 and self.max_steer_rate > 0.0:
            max_delta = self.max_steer_rate * min(steer_dt, dt * 4.0)
            steer = self._apply_rate_limit(self._last_steer_sent_rad, steer, max_delta)
        self._last_steer_sent_rad = steer
        self._last_steer_t = now
        servo_us = self._steer_to_servo_us(steer)
        speed_left, speed_right = self._vx_steer_to_wheel_speeds(gated_vx, steer)

        # SPEEDS every tick — feeds the firmware's 200 ms watchdog
        # (safety_feed_watchdog() is called by the SPEEDS handler).
        self._send(f"SPEEDS {speed_left} {speed_right}")
        self._last_speed_sent = (speed_left, speed_right)
        # SERVO_PWM only on change to save serial bandwidth (and because
        # the SERVO_PWM handler does not feed the watchdog anyway).
        if servo_us != self._last_us_sent:
            self._send(f"SERVO_PWM {servo_us}")
            self._last_us_sent = servo_us

    # ── Shutdown ─────────────────────────────────────────────────
    def shutdown_safe_state(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self._send("STOP")
            self._send("DISABLE")
            self._send(f"SERVO_PWM {self.servo_center}")
        except Exception:
            pass

    def cleanup(self) -> None:
        self.shutdown_safe_state()   # sets _shutting_down → RX loop exits
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=0.5)
            self._rx_thread = None
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self._release_lock()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Nav2PicoBridge()
        rclpy.spin(node)
    except RuntimeError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"[nav2_pico_bridge] {exc}", flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.cleanup()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
