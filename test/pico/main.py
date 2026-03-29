"""
Pico MicroPython firmware for Nav2 packet testing.

- Receives newline-delimited JSON over UART.
- Drives L298N dual DC motor driver.
- Drives one servo with state-based behavior.
- Enforces emergency stop, health faults, and watchdog timeout.
- Prints CSV-like debug lines for logging.
"""

import machine
import time
import ujson

# -----------------------------
# Pin config (edit for your wiring)
# -----------------------------
# Left motor (L298N)
IN1 = machine.Pin(2, machine.Pin.OUT)
IN2 = machine.Pin(3, machine.Pin.OUT)
ENA = machine.PWM(machine.Pin(4))

# Right motor (L298N)
IN3 = machine.Pin(6, machine.Pin.OUT)
IN4 = machine.Pin(7, machine.Pin.OUT)
ENB = machine.PWM(machine.Pin(8))

# Servo
SERVO = machine.PWM(machine.Pin(15))

# UART0 default pins for Pico: TX=GP0, RX=GP1
# If you use USB CDC instead, adapt read method accordingly.
uart = machine.UART(0, baudrate=115200)

# PWM frequencies
MOTOR_PWM_HZ = 1000
SERVO_PWM_HZ = 50
ENA.freq(MOTOR_PWM_HZ)
ENB.freq(MOTOR_PWM_HZ)
SERVO.freq(SERVO_PWM_HZ)

# -----------------------------
# Limits / tuning
# -----------------------------
MAX_V_LIN = 0.40
MAX_V_ANG = 1.50
MAX_PWM = 50000
MIN_PWM_DEADBAND = 8000
WATCHDOG_MS = 300
RAMP_STEP = 2000
CONTROL_DT_MS = 50

K_LIN = 1.0
K_ANG = 0.6

FRONT_WARN = 0.50
FRONT_STOP = 0.20

SERVO_STATE_ANGLE = {
    "IDLE": 90,
    "TRACKING_PATH": 90,
    "RECOVERY": 30,      # sweep around this zone
    "GOAL_REACHED": 120,
    "FAILED": 150,
}

# -----------------------------
# State
# -----------------------------
last_rx_ms = time.ticks_ms()
last_seq = -1
parse_err = 0
drop_old = 0
watchdog_trips = 0

current_state = "IDLE"
recovery_sweep_dir = 1
recovery_angle = 30

target_left = 0
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


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def angle_to_duty_u16(angle_deg):
    angle = clamp(angle_deg, 0, 180)
    us = 500 + (2000 * angle / 180)      # 0.5ms..2.5ms
    return int(us * 65535 / 20000)       # 20ms period at 50Hz


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
        IN1.value(1)
        IN2.value(0)
        left_abs = left_duty
    else:
        IN1.value(0)
        IN2.value(1)
        left_abs = -left_duty

    if right_duty >= 0:
        IN3.value(1)
        IN4.value(0)
        right_abs = right_duty
    else:
        IN3.value(0)
        IN4.value(1)
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

    # obstacle scaling only for forward motion
    if v_lin > 0.0:
        v_lin *= compute_speed_scale(front_m)

    n_lin = v_lin / MAX_V_LIN if MAX_V_LIN > 0 else 0.0
    n_ang = v_ang / MAX_V_ANG if MAX_V_ANG > 0 else 0.0

    left = K_LIN * n_lin - K_ANG * n_ang
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


def parse_line(line):
    global parse_err, drop_old, last_seq, last_rx_ms, pkt, current_state

    try:
        data = ujson.loads(line)

        for key in ("state", "v_lin", "v_ang", "obstacle", "health", "seq"):
            if key not in data:
                parse_err += 1
                return False

        seq = int(data["seq"])
        if seq <= last_seq:
            drop_old += 1
            return False

        last_seq = seq
        pkt = data
        current_state = data.get("state", "IDLE")
        last_rx_ms = time.ticks_ms()
        return True
    except Exception:
        parse_err += 1
        return False


def safety_fault(data):
    obstacle = data.get("obstacle", {})
    health = data.get("health", {})

    if obstacle.get("emergency_stop", False):
        return True, "ESTOP"

    if not (health.get("loc_ok", True) and health.get("planner_ok", True) and health.get("controller_ok", True)):
        return True, "HEALTH_FAULT"

    return False, "OK"


motor_stop()
set_servo(90)
print("PICO_READY")

rx_buf = b""
last_ctrl = time.ticks_ms()
last_log = time.ticks_ms()

while True:
    if uart.any():
        chunk = uart.read()
        if chunk:
            rx_buf += chunk
            while b"\n" in rx_buf:
                raw_line, rx_buf = rx_buf.split(b"\n", 1)
                raw_line = raw_line.strip()
                if raw_line:
                    parse_line(raw_line.decode("utf-8"))

    now = time.ticks_ms()

    if time.ticks_diff(now, last_ctrl) >= CONTROL_DT_MS:
        last_ctrl = now

        age_ms = time.ticks_diff(now, last_rx_ms)
        if age_ms > WATCHDOG_MS:
            watchdog_trips += 1
            current_state = "FAILED"
            target_left = 0
            target_right = 0
        else:
            fault, _reason = safety_fault(pkt)
            if fault:
                current_state = "FAILED"
                target_left = 0
                target_right = 0
            else:
                front_m = float(pkt.get("obstacle", {}).get("front_m", 9.9))
                v_lin = float(pkt.get("v_lin", 0.0))
                v_ang = float(pkt.get("v_ang", 0.0))
                target_left, target_right = map_cmd_to_pwm(v_lin, v_ang, front_m)

        current_left = ramp_to_target(current_left, target_left, RAMP_STEP)
        current_right = ramp_to_target(current_right, target_right, RAMP_STEP)
        motor_apply(current_left, current_right)

        if current_state == "RECOVERY":
            recovery_angle += 5 * recovery_sweep_dir
            if recovery_angle >= 150:
                recovery_angle = 150
                recovery_sweep_dir = -1
            elif recovery_angle <= 30:
                recovery_angle = 30
                recovery_sweep_dir = 1
            set_servo(recovery_angle)
        else:
            set_servo(SERVO_STATE_ANGLE.get(current_state, 90))

    if time.ticks_diff(now, last_log) >= 200:
        last_log = now
        front = pkt.get("obstacle", {}).get("front_m", 9.9)
        estop = pkt.get("obstacle", {}).get("emergency_stop", False)
        seq = pkt.get("seq", -1)

        # CSV-like log:
        # t_ms,seq,state,left_pwm,right_pwm,front_m,estop,parse_err,drop_old,watchdog_trips
        print(
            "{},{},{},{},{},{},{},{},{},{}".format(
                now,
                seq,
                current_state,
                current_left,
                current_right,
                front,
                int(estop),
                parse_err,
                drop_old,
                watchdog_trips,
            )
        )
