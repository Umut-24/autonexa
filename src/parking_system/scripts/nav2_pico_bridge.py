#!/usr/bin/env python3
"""
Nav2 → Pico ASCII serial bridge for the AutoNexa CLI firmware.

Subscribes /cmd_vel, applies vx/wz limits + acceleration caps + 200 ms
watchdog, computes Ackermann inverse kinematics, and writes
SPEED + SERVO_PWM lines over /dev/ttyACM0 at 30 Hz.

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
is mounted opposite.

Lifecycle:
    on launch: open serial → send `ENABLE` (if auto_enable=true)
    every tick:  send `SPEED <n>` then (if changed) `SERVO_PWM <us>`
    on watchdog: ramp output to (0, 0); next tick sends SPEED 0
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

# Parameters the bridge will accept overrides for. Anything outside this list
# is ignored when reading the YAML — keeps a stale or hand-edited file from
# poking at parameters that aren't safe to change at runtime (e.g. serial port).
RUNTIME_OVERRIDABLE = (
    'vx_polarity',
    'servo_polarity',
    'max_vx_mps',
    'max_wz_radps',
    'max_ax_mps2',
    'max_aw_radps2',
    'max_steer_rate_radps',
    'min_vx_creep',
    'servo_center_us',
    'servo_us_min',
    'servo_us_max',
)


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
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('command_timeout_s', 0.20)

        self.declare_parameter('max_vx_mps', 0.30)
        self.declare_parameter('max_wz_radps', 0.8)
        self.declare_parameter('max_ax_mps2', 0.8)
        self.declare_parameter('max_aw_radps2', 1.2)

        # Deadband gate: the L298N firmware snaps any non-zero SPEED to a
        # ~60% PWM kick (see MOTOR_DEADBAND_PCT in pico_firmware/include/config.h).
        # That makes Nav2's slow-creep approach lurch instead of trim. If the
        # smoothed |vx| is below this threshold, send SPEED 0 and let the
        # chassis coast — combined with a looser goal_xy_tolerance this gives
        # clean stops without overshoot.
        self.declare_parameter('min_vx_creep', 0.05)

        self.declare_parameter('wheelbase_m', 0.25)
        # 100 = vx 0.30 m/s -> SPEED 30 -> 100% PWM duty on the L298N path.
        # Was 63.7 for the old Hiwonder closed-loop "pulses/10ms" semantic.
        self.declare_parameter('vel_to_speed_scale', 100.0)
        self.declare_parameter('max_speed_pulses', 30)

        self.declare_parameter('servo_center_us', 1650)
        self.declare_parameter('servo_us_min', 1100)
        self.declare_parameter('servo_us_max', 1900)
        self.declare_parameter('servo_max_steer_rad', 0.5236)
        # Default -1 on this chassis: empirically the linkage geometry inverts
        # the firmware's intended sign so that ROS-positive wz (left turn)
        # needs us > center, not us < center. Override to +1 if the hardware
        # is rewired the other way.
        self.declare_parameter('servo_polarity', -1)
        # +1 = ROS-positive vx drives the chassis forward (standard).
        # -1 = forward/back swapped (use when motor wiring is reversed or the
        # LiDAR-defined map frame is yaw-flipped). Calibration wizard in the
        # mobile app toggles this live; persists via runtime_overrides.yaml.
        self.declare_parameter('vx_polarity', 1)
        # Servo slew-rate cap (rad/s). LD-1501MG datasheet ~3.0 rad/s @ no
        # load; lower values smooth aggressive Nav2 wz step changes that
        # otherwise make the steering "thunk".
        self.declare_parameter('max_steer_rate_radps', 3.0)

        self.declare_parameter('auto_enable', True)
        self.declare_parameter('bridge_lock_file', '/tmp/nav2_pico_bridge.lock')
        self.declare_parameter('dry_run', False)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.serial_baud = int(self.get_parameter('serial_baud').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_timeout = float(self.get_parameter('command_timeout_s').value)

        self.max_vx = float(self.get_parameter('max_vx_mps').value)
        self.max_wz = float(self.get_parameter('max_wz_radps').value)
        self.max_ax = float(self.get_parameter('max_ax_mps2').value)
        self.max_aw = float(self.get_parameter('max_aw_radps2').value)
        self.min_vx_creep = float(self.get_parameter('min_vx_creep').value)

        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.vel_scale = float(self.get_parameter('vel_to_speed_scale').value)
        self.max_speed = int(self.get_parameter('max_speed_pulses').value)

        self.servo_center = int(self.get_parameter('servo_center_us').value)
        self.servo_us_min = int(self.get_parameter('servo_us_min').value)
        self.servo_us_max = int(self.get_parameter('servo_us_max').value)
        self.servo_max_steer = float(self.get_parameter('servo_max_steer_rad').value)
        self.servo_polarity = int(self.get_parameter('servo_polarity').value)
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
        if self.vx_polarity not in (-1, 1):
            raise RuntimeError(f"vx_polarity must be +1 or -1, got {self.vx_polarity}")

        self.target = MotionState()
        self.output = MotionState()
        self.last_cmd_time = self.get_clock().now()
        self._last_us_sent = None
        self._last_speed_sent = None
        self._last_steer_sent_rad = 0.0
        self._last_steer_t = self.get_clock().now()
        self._lock_fh = None
        self._serial = None
        self._write_lock = threading.Lock()
        self._shutting_down = False

        self._acquire_lock()
        # Register the validation callback first so any overrides we replay
        # below are routed through it (and so cached attrs stay in sync with
        # the parameter store).
        self.add_on_set_parameters_callback(self._on_param_set)
        self._apply_runtime_overrides()

        if not self.dry_run:
            self._open_serial()
            if self.auto_enable:
                self._send("ENABLE")

        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)
        manual_topic = str(self.get_parameter('manual_cmd_vel_topic').value)
        if manual_topic and manual_topic != self.cmd_vel_topic:
            self.manual_sub = self.create_subscription(
                Twist, manual_topic, self.on_cmd_vel, 20)
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
        for key, value in section.items():
            if key not in RUNTIME_OVERRIDABLE:
                continue
            try:
                param = self.get_parameter(key)
            except Exception:
                continue
            try:
                kv.append(rclpy.parameter.Parameter(key, param.type_, value))
            except Exception as exc:
                self.get_logger().warning(
                    f'override {key}={value!r} rejected: {exc}')
        if kv:
            results = self.set_parameters(kv)
            applied = [k.name for k, r in zip(kv, results) if r.successful]
            self.get_logger().info(
                f'runtime overrides applied: {applied}')

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
            elif p.name == 'servo_center_us':
                if not 800 <= int(p.value) <= 2200:
                    return SetParametersResult(successful=False, reason='servo_center_us out of range')
                self.servo_center = int(p.value)
            elif p.name == 'servo_us_min':
                # Hard floor at 1100 µs — beyond this the linkage binds and
                # the servo stalls/draws current. Param tuner cannot widen
                # past mechanical safe range.
                if not 1100 <= int(p.value) <= self.servo_center:
                    return SetParametersResult(
                        successful=False,
                        reason='servo_us_min must be in [1100, servo_center_us]')
                self.servo_us_min = int(p.value)
            elif p.name == 'servo_us_max':
                # Hard ceiling at 1900 µs — same rationale as min above.
                if not self.servo_center <= int(p.value) <= 1900:
                    return SetParametersResult(
                        successful=False,
                        reason='servo_us_max must be in [servo_center_us, 1900]')
                self.servo_us_max = int(p.value)
        return SetParametersResult(successful=True)

    # ── Mapping math ──────────────────────────────────────────────
    def _vx_to_speed_pulses(self, vx: float) -> int:
        s = int(round(self.vx_polarity * vx * self.vel_scale))
        return clamp(s, -self.max_speed, +self.max_speed)

    def _vx_wz_to_steer(self, vx: float, wz: float) -> float:
        """Standard Ackermann inverse. Mirrors firmware ackermann.c:23."""
        if abs(vx) < 0.01:
            if abs(wz) < 0.01:
                return 0.0
            # vx≈0 with wz≠0 is a pivot request — Ackermann can't do it.
            # Match firmware behavior: command max steering toward sign(wz).
            return self.servo_max_steer if wz > 0 else -self.servo_max_steer
        return clamp(math.atan(self.wheelbase * wz / vx),
                     -self.servo_max_steer, +self.servo_max_steer)

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
    def on_cmd_vel(self, msg: Twist) -> None:
        self.target.vx = clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self.target.wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self.last_cmd_time = self.get_clock().now()

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

        command_stale = (now - self.last_cmd_time) > Duration(seconds=self.command_timeout)
        desired = MotionState(0.0, 0.0) if command_stale else self.target

        self.output.vx = self._apply_rate_limit(self.output.vx, desired.vx, self.max_ax * dt)
        self.output.wz = self._apply_rate_limit(self.output.wz, desired.wz, self.max_aw * dt)

        # Gate sub-deadband output to 0 — see min_vx_creep param doc.
        # Steering still tracks the commanded wz so the wheels keep pointing
        # toward the goal direction even while the chassis is coasting.
        gated_vx = 0.0 if abs(self.output.vx) < self.min_vx_creep else self.output.vx

        speed = self._vx_to_speed_pulses(gated_vx)
        steer = self._vx_wz_to_steer(self.output.vx, self.output.wz)
        # Servo slew-rate limiter — runs in steering-angle space (rad) so it
        # respects the actual mechanical limit of the servo, not the µs scale.
        # Applied here, after Ackermann math, so it bounds the output the
        # firmware sees regardless of how fast the upstream wz changes.
        steer_dt = (now - self._last_steer_t).nanoseconds * 1e-9
        if steer_dt > 0.0 and self.max_steer_rate > 0.0:
            max_delta = self.max_steer_rate * min(steer_dt, dt * 4.0)
            steer = self._apply_rate_limit(self._last_steer_sent_rad, steer, max_delta)
        self._last_steer_sent_rad = steer
        self._last_steer_t = now
        servo_us = self._steer_to_servo_us(steer)

        # SPEED every tick — feeds the firmware's 200 ms watchdog
        # (safety_feed_watchdog() is called by the SPEED handler).
        self._send(f"SPEED {speed}")
        self._last_speed_sent = speed
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
        self.shutdown_safe_state()
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
