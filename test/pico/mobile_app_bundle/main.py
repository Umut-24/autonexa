"""
Pico MicroPython firmware for Nav2 packet testing.

- Receives newline-delimited JSON over USB-CDC (sys.stdin) or UART.
- Drives L298N dual DC motor driver.
- Drives one steering servo.
- Reads quadrature encoders directly on GP10-13.
- Supports two command modes:
    1. Velocity stream: {"v_lin": 0.2, "v_ang": 0.0, ...}  (Nav2 bridge compatible)
    2. Goal commands:  {"cmd": "DRIVE", "distance_m": 1.0, "speed": 0.25, "seq": 1}
                       {"cmd": "TURN",  "angle_deg": 15.0,  "speed": 0.20, "seq": 2}
                       {"cmd": "STOP",  "seq": 3}
                       {"cmd": "RESET_ODOM", "seq": 4}
- Telemetry CSV at 5 Hz:
    t_ms,cmd,left_ticks,right_ticks,dist_m,heading_deg,state,left_pwm,right_pwm
"""

import machine
import time
import ujson
import sys
import math

# -----------------------------
# Motor pins (L298N)
# -----------------------------
IN1 = machine.Pin(2, machine.Pin.OUT)
IN2 = machine.Pin(3, machine.Pin.OUT)
ENA = machine.PWM(machine.Pin(4))

IN3 = machine.Pin(6, machine.Pin.OUT)
IN4 = machine.Pin(7, machine.Pin.OUT)
ENB = machine.PWM(machine.Pin(8))

# Servo
SERVO = machine.PWM(machine.Pin(15))

# Heartbeat LED
LED = machine.Pin(25, machine.Pin.OUT)

# -----------------------------
# Encoder pins (direct to Pico)
# JGB37-520R30: quadrature, 1320 edges/wheel rev
# Left:  A=GP10, B=GP11
# Right: A=GP12, B=GP13
# -----------------------------
ENC_LEFT_A  = machine.Pin(10, machine.Pin.IN, machine.Pin.PULL_UP)
ENC_LEFT_B  = machine.Pin(11, machine.Pin.IN, machine.Pin.PULL_UP)
ENC_RIGHT_A = machine.Pin(12, machine.Pin.IN, machine.Pin.PULL_UP)
ENC_RIGHT_B = machine.Pin(13, machine.Pin.IN, machine.Pin.PULL_UP)

# PWM frequencies
MOTOR_PWM_HZ = 1000
SERVO_PWM_HZ = 50
ENA.freq(MOTOR_PWM_HZ)
ENB.freq(MOTOR_PWM_HZ)
SERVO.freq(SERVO_PWM_HZ)

# -----------------------------
# Odometry constants
# -----------------------------
ENCODER_EDGES_PER_REV = 1320          # 11 CPR * 4 * 30 gear ratio
WHEEL_RADIUS_M        = 0.033
TRACK_WIDTH_M         = 0.20
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / ENCODER_EDGES_PER_REV
# ~= 0.0001575 m/tick

# -----------------------------
# Motor limits / tuning
# -----------------------------
MAX_V_LIN       = 0.40
MAX_V_ANG       = 1.50
MAX_PWM         = 65535
MIN_PWM_DEADBAND = 20000
WATCHDOG_MS     = 1000
RAMP_STEP       = 2000
CONTROL_DT_MS   = 50            # 20 Hz control loop

K_LIN = 1.0
K_ANG = 0.6

FRONT_WARN = 0.50
FRONT_STOP = 0.20

# Goal execution: slow down when within this fraction of target
DECEL_FRACTION = 0.20

SERVO_STATE_ANGLE = {
    "IDLE":          90,
    "TRACKING_PATH": 90,
    "RECOVERY":      30,
    "GOAL_REACHED":  120,
    "FAILED":        150,
    "DRIVE":         90,
    "TURN":          90,
}

# -----------------------------
# Encoder state (volatile — written in ISR)
# -----------------------------
left_ticks  = 0
right_ticks = 0


def enc_left_cb(pin):
    global left_ticks
    # Rising or falling on A: direction = B state
    # B=0 → forward (+1), B=1 → reverse (-1)
    left_ticks += 1 if ENC_LEFT_B.value() == 0 else -1


def enc_right_cb(pin):
    global right_ticks
    right_ticks += 1 if ENC_RIGHT_B.value() == 0 else -1


ENC_LEFT_A.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
    handler=enc_left_cb,
)
ENC_RIGHT_A.irq(
    trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
    handler=enc_right_cb,
)

# -----------------------------
# Velocity-stream state
# -----------------------------
last_rx_ms   = time.ticks_ms()
last_seq      = -1
parse_err     = 0
drop_old      = 0
watchdog_trips = 0

current_state = "IDLE"
recovery_sweep_dir = 1
recovery_angle     = 30

target_left  = 0
target_right = 0
current_left = 0
current_right = 0

pkt = {
    "state": "IDLE",
    "v_lin": 0.0,
    "v_ang": 0.0,
    "obstacle": {
        "front_m": 9.9,
        "left_m": 9.9,
        "right_m": 9.9,
        "emergency_stop": False,
    },
    "health": {
        "loc_ok": True,
        "planner_ok": True,
        "controller_ok": True,
    },
    "seq": 0,
}

# Active goal (None when idle)
active_goal = None   # {"type": "DRIVE"|"TURN", "target": float, "speed": float,
                     #  "start_left": int, "start_right": int}


# -----------------------------
# Helpers
# -----------------------------

def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def angle_to_duty_u16(angle_deg):
    angle = clamp(angle_deg, 0, 180)
    us = 500 + (2000 * angle / 180)
    return int(us * 65535 / 20000)


def set_servo(angle_deg):
    SERVO.duty_u16(angle_to_duty_u16(angle_deg))


def motor_stop():
    ENA.duty_u16(0)
    ENB.duty_u16(0)
    IN1.value(0)
    IN2.value(0)
    IN3.value(0)
    IN4.value(0)


def motor_apply(left_duty, right_duty):
    if left_duty >= 0:
        IN1.value(1); IN2.value(0)
        left_abs = left_duty
    else:
        IN1.value(0); IN2.value(1)
        left_abs = -left_duty

    if right_duty >= 0:
        IN3.value(1); IN4.value(0)
        right_abs = right_duty
    else:
        IN3.value(0); IN4.value(1)
        right_abs = -right_duty

    ENA.duty_u16(int(clamp(left_abs, 0, MAX_PWM)))
    ENB.duty_u16(int(clamp(right_abs, 0, MAX_PWM)))


def ramp_to_target(current, target, step):
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


def compute_speed_scale(front_m):
    if front_m <= FRONT_STOP:
        return 0.0
    if front_m <= FRONT_WARN:
        return (front_m - FRONT_STOP) / (FRONT_WARN - FRONT_STOP)
    return 1.0


def map_cmd_to_pwm(v_lin, v_ang, front_m):
    v_lin = clamp(v_lin, -MAX_V_LIN, MAX_V_LIN)
    v_ang = clamp(v_ang, -MAX_V_ANG, MAX_V_ANG)

    if v_lin > 0.0:
        v_lin *= compute_speed_scale(front_m)

    n_lin = v_lin / MAX_V_LIN if MAX_V_LIN > 0 else 0.0
    n_ang = v_ang / MAX_V_ANG if MAX_V_ANG > 0 else 0.0

    left  = K_LIN * n_lin - K_ANG * n_ang
    right = K_LIN * n_lin + K_ANG * n_ang

    mx = max(1.0, abs(left), abs(right))
    left /= mx
    right /= mx

    def to_pwm(x):
        if abs(x) < 0.05:
            return 0
        duty = int(MIN_PWM_DEADBAND + (MAX_PWM - MIN_PWM_DEADBAND) * abs(x))
        return duty if x >= 0 else -duty

    return to_pwm(left), to_pwm(right)


def speed_to_pwm(speed_norm):
    """Convert normalised speed [0..1] to PWM duty (above deadband)."""
    if speed_norm <= 0.0:
        return 0
    return int(MIN_PWM_DEADBAND + (MAX_PWM - MIN_PWM_DEADBAND) * clamp(speed_norm, 0.0, 1.0))


def vel_to_norm(v_mps):
    """Convert m/s goal speed to normalised [0..1]."""
    return clamp(abs(v_mps) / MAX_V_LIN, 0.05, 1.0)


# -----------------------------
# Serial I/O — USB CDC (sys.stdin) with uselect.ipoll
# uselect.ipoll() is the correct non-blocking read for MicroPython USB CDC.
# Telemetry goes to sys.stdout so mpremote / screen can monitor it.
# If you want physical UART wires instead, replace _read_stdin with UART.
# -----------------------------

import uselect as _usel

rx_buf = b""
_spoll = _usel.poll()
_spoll.register(sys.stdin, _usel.POLLIN)


def _read_stdin_chunk():
    """Read all currently available bytes from USB CDC (non-blocking)."""
    chunk = b""
    # ipoll(0) = return immediately with events present right now
    for _ in _spoll.ipoll(0):
        try:
            # Try buffer API first (fastest)
            buf = sys.stdin.buffer
            n = buf.any()
            if n > 0:
                chunk += buf.read(n)
                continue
        except AttributeError:
            pass
        # Fall back: read one char at a time while data available
        try:
            ch = sys.stdin.read(1)
            if ch:
                chunk += ch.encode() if isinstance(ch, str) else ch
        except Exception:
            pass
    return chunk


def read_serial_lines():
    """Return list of complete newline-delimited JSON strings (non-blocking)."""
    global rx_buf
    lines = []
    chunk = _read_stdin_chunk()
    if chunk:
        rx_buf += chunk
    while b"\n" in rx_buf:
        raw, rx_buf = rx_buf.split(b"\n", 1)
        raw = raw.strip()
        if raw:
            lines.append(raw.decode("utf-8", "ignore"))
    return lines


def send_line(s):
    sys.stdout.write(s + "\n")


# -----------------------------
# Packet parsing
# -----------------------------

def parse_line(line):
    global parse_err, drop_old, last_seq, last_rx_ms, pkt, current_state, active_goal
    global target_left, target_right, current_left, current_right

    try:
        data = ujson.loads(line)
    except Exception:
        parse_err += 1
        return False

    # --- Goal command packet ---
    if "cmd" in data:
        cmd = data["cmd"]
        seq = int(data.get("seq", last_seq + 1))
        if seq <= last_seq:
            drop_old += 1
            return False
        last_seq = seq
        last_rx_ms = time.ticks_ms()

        if cmd == "STOP":
            active_goal = None
            current_state = "IDLE"
            target_left = 0
            target_right = 0
            current_left = 0
            current_right = 0
            pkt["v_lin"] = 0.0
            pkt["v_ang"] = 0.0
            pkt["obstacle"]["emergency_stop"] = True
            motor_stop()

        elif cmd == "RESET_ODOM":
            global left_ticks, right_ticks
            left_ticks = 0
            right_ticks = 0
            active_goal = None
            current_state = "IDLE"

        elif cmd == "DRIVE":
            dist  = float(data.get("distance_m", 0.0))
            speed = float(data.get("speed", 0.20))
            active_goal = {
                "type":        "DRIVE",
                "target_m":    dist,
                "speed":       speed,
                "sign":        1 if dist >= 0 else -1,
                "start_left":  left_ticks,
                "start_right": right_ticks,
            }
            current_state = "DRIVE"

        elif cmd == "TURN":
            angle_deg = float(data.get("angle_deg", 0.0))
            speed     = float(data.get("speed", 0.20))
            active_goal = {
                "type":        "TURN",
                "target_rad":  math.radians(angle_deg),
                "speed":       speed,
                "sign":        1 if angle_deg >= 0 else -1,
                "start_left":  left_ticks,
                "start_right": right_ticks,
            }
            current_state = "TURN"

        return True

    # --- Velocity stream packet ---
    for key in ("state", "v_lin", "v_ang", "obstacle", "health", "seq"):
        if key not in data:
            parse_err += 1
            return False

    seq = int(data["seq"])
    if seq <= last_seq:
        drop_old += 1
        return False

    # Velocity stream cancels active goal
    active_goal   = None
    last_seq      = seq
    pkt           = data
    current_state = data.get("state", "IDLE")
    last_rx_ms    = time.ticks_ms()
    return True


def safety_fault(data):
    obstacle = data.get("obstacle", {})
    health   = data.get("health", {})
    if obstacle.get("emergency_stop", False):
        return True, "ESTOP"
    if not (health.get("loc_ok", True) and
            health.get("planner_ok", True) and
            health.get("controller_ok", True)):
        return True, "HEALTH_FAULT"
    return False, "OK"


# -----------------------------
# Goal execution step (called every CONTROL_DT_MS)
# Returns (left_pwm, right_pwm) or None when goal is done/aborted
# -----------------------------

def step_goal():
    global active_goal, current_state

    if active_goal is None:
        return None

    g = active_goal

    if g["type"] == "DRIVE":
        dl = left_ticks  - g["start_left"]
        dr = right_ticks - g["start_right"]
        travelled_m = abs((dl + dr) / 2.0 * DIST_PER_TICK)
        target_m    = abs(g["target_m"])

        if travelled_m >= target_m:
            active_goal   = None
            current_state = "IDLE"
            motor_stop()
            return (0, 0)

        # Deceleration ramp
        remaining = target_m - travelled_m
        norm = vel_to_norm(g["speed"])
        if remaining < DECEL_FRACTION * target_m:
            norm *= 0.5

        pwm = speed_to_pwm(norm)
        signed_pwm = pwm * g["sign"]
        return (signed_pwm, signed_pwm)

    elif g["type"] == "TURN":
        dl = left_ticks  - g["start_left"]
        dr = right_ticks - g["start_right"]
        # heading change: positive = right turn (right ticks > left ticks)
        delta_rad = (dr - dl) * DIST_PER_TICK / TRACK_WIDTH_M
        turned_rad  = abs(delta_rad)
        target_rad  = abs(g["target_rad"])

        if turned_rad >= target_rad:
            active_goal   = None
            current_state = "IDLE"
            motor_stop()
            return (0, 0)

        remaining = target_rad - turned_rad
        norm = vel_to_norm(g["speed"])
        if remaining < DECEL_FRACTION * target_rad:
            norm *= 0.5

        pwm = speed_to_pwm(norm)
        # sign > 0 = right turn: left fwd, right back
        left_pwm  =  pwm * g["sign"]
        right_pwm = -pwm * g["sign"]
        return (left_pwm, right_pwm)

    return None


# -----------------------------
# Boot
# -----------------------------

motor_stop()
set_servo(90)

for _ in range(4):
    LED.toggle()
    time.sleep_ms(150)
LED.value(0)

send_line("PICO_READY")

last_ctrl = time.ticks_ms()
last_log  = time.ticks_ms()
last_led  = time.ticks_ms()
led_state = 0

# -----------------------------
# Main loop
# -----------------------------

while True:
    # --- Read serial ---
    for line in read_serial_lines():
        parse_line(line)

    now = time.ticks_ms()

    # --- Control tick (20 Hz) ---
    if time.ticks_diff(now, last_ctrl) >= CONTROL_DT_MS:
        last_ctrl = now

        if active_goal is not None:
            # Goal execution mode — feed watchdog by resetting last_rx_ms each tick
            last_rx_ms = now
            result = step_goal()
            if result is not None:
                target_left, target_right = result
            current_left  = ramp_to_target(current_left,  target_left,  RAMP_STEP)
            current_right = ramp_to_target(current_right, target_right, RAMP_STEP)
            motor_apply(current_left, current_right)
            set_servo(SERVO_STATE_ANGLE.get(current_state, 90))

        else:
            # Velocity stream mode
            age_ms = time.ticks_diff(now, last_rx_ms)
            if age_ms > WATCHDOG_MS:
                watchdog_trips += 1
                current_state = "FAILED"
                target_left   = 0
                target_right  = 0
            else:
                fault, _reason = safety_fault(pkt)
                if fault:
                    current_state = "FAILED"
                    target_left   = 0
                    target_right  = 0
                else:
                    front_m = float(pkt.get("obstacle", {}).get("front_m", 9.9))
                    v_lin   = float(pkt.get("v_lin", 0.0))
                    v_ang   = float(pkt.get("v_ang", 0.0))
                    target_left, target_right = map_cmd_to_pwm(v_lin, v_ang, front_m)

            current_left  = ramp_to_target(current_left,  target_left,  RAMP_STEP)
            current_right = ramp_to_target(current_right, target_right, RAMP_STEP)
            motor_apply(current_left, current_right)

            if current_state == "RECOVERY":
                recovery_angle += 5 * recovery_sweep_dir
                if recovery_angle >= 150:
                    recovery_angle = 150; recovery_sweep_dir = -1
                elif recovery_angle <= 30:
                    recovery_angle = 30; recovery_sweep_dir = 1
                set_servo(recovery_angle)
            else:
                set_servo(SERVO_STATE_ANGLE.get(current_state, 90))

    # --- Telemetry (5 Hz) ---
    if time.ticks_diff(now, last_log) >= 200:
        last_log = now

        dl = left_ticks
        dr = right_ticks
        dist_m      = (dl + dr) / 2.0 * DIST_PER_TICK
        heading_deg = math.degrees((dr - dl) * DIST_PER_TICK / TRACK_WIDTH_M)

        # t_ms,cmd,left_ticks,right_ticks,dist_m,heading_deg,state,left_pwm,right_pwm
        goal_type = active_goal["type"] if active_goal else "NONE"
        send_line("{},{},{},{},{:.4f},{:.2f},{},{},{}".format(
            now,
            goal_type,
            dl,
            dr,
            dist_m,
            heading_deg,
            current_state,
            current_left,
            current_right,
        ))

    # --- Heartbeat LED (1 Hz) ---
    if time.ticks_diff(now, last_led) >= 500:
        last_led = now
        LED.toggle()
