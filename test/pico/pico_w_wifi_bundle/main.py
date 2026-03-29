"""
Pico W MicroPython firmware: direct Wi-Fi control (no RPi bridge required).

Exposes HTTP endpoints compatible with the mobile app:
  GET  /api/status
  GET  /api/telemetry
  POST /api/control   JSON {x, y, e, speed_limit}
  POST /api/goal      JSON {cmd: DRIVE|TURN|STOP|RESET_ODOM, ...}
  POST /api/estop

Default mode creates Pico AP Wi-Fi from config.py, then serves HTTP on port 5001.
"""

import gc
import math
import machine
import network
import socket
import time
import ujson

from config import (
    WIFI_MODE,
    AP_SSID,
    AP_PASSWORD,
    AP_IP,
    STA_SSID,
    STA_PASSWORD,
    STA_TIMEOUT_MS,
    HTTP_PORT,
)

# -----------------------------------------------------------------------------
# Motor pins (L298N)
# -----------------------------------------------------------------------------
IN1 = machine.Pin(2, machine.Pin.OUT)
IN2 = machine.Pin(3, machine.Pin.OUT)
ENA = machine.PWM(machine.Pin(4))

IN3 = machine.Pin(6, machine.Pin.OUT)
IN4 = machine.Pin(7, machine.Pin.OUT)
ENB = machine.PWM(machine.Pin(8))

SERVO = machine.PWM(machine.Pin(15))
LED = machine.Pin(25, machine.Pin.OUT)

# Encoders
ENC_LEFT_A = machine.Pin(10, machine.Pin.IN, machine.Pin.PULL_UP)
ENC_LEFT_B = machine.Pin(11, machine.Pin.IN, machine.Pin.PULL_UP)
ENC_RIGHT_A = machine.Pin(12, machine.Pin.IN, machine.Pin.PULL_UP)
ENC_RIGHT_B = machine.Pin(13, machine.Pin.IN, machine.Pin.PULL_UP)

MOTOR_PWM_HZ = 1000
SERVO_PWM_HZ = 50
ENA.freq(MOTOR_PWM_HZ)
ENB.freq(MOTOR_PWM_HZ)
SERVO.freq(SERVO_PWM_HZ)

# -----------------------------------------------------------------------------
# Tuning/constants
# -----------------------------------------------------------------------------
ENCODER_EDGES_PER_REV = 1320
WHEEL_RADIUS_M = 0.033
TRACK_WIDTH_M = 0.20
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / ENCODER_EDGES_PER_REV

MAX_V_LIN = 0.40
MAX_V_ANG = 1.50
MAX_PWM = 65535
MIN_PWM_DEADBAND = 20000
WATCHDOG_MS = 1200
RAMP_STEP = 2000
CONTROL_DT_MS = 50
TELEMETRY_DT_MS = 200

K_LIN = 1.0
K_ANG = 0.6
DECEL_FRACTION = 0.20

SERVO_STATE_ANGLE = {
    "IDLE": 90,
    "TRACKING_PATH": 90,
    "E_STOP": 150,
    "DRIVE": 90,
    "TURN": 90,
}

# -----------------------------------------------------------------------------
# State
# -----------------------------------------------------------------------------
left_ticks = 0
right_ticks = 0

current_state = "IDLE"
wifi_ip = "0.0.0.0"

cmd_v_lin = 0.0
cmd_v_ang = 0.0
cmd_estop = False
last_control_ms = 0

active_goal = None

target_left = 0
target_right = 0
current_left = 0
current_right = 0

last_telem = {
    "left_wheel_vel": 0.0,
    "right_wheel_vel": 0.0,
    "steering_pos": 0.0,
    "odom_vx": 0.0,
    "odom_wz": 0.0,
    "odom_x": 0.0,
    "odom_y": 0.0,
    "odom_yaw": 0.0,
    "pico_state": "IDLE",
    "pico_state_raw": "IDLE",
    "left_ticks": 0,
    "right_ticks": 0,
    "heading_deg": 0.0,
    "goal_type": "NONE",
    "last_control_age_ms": 0,
}


# -----------------------------------------------------------------------------
# Utility
# -----------------------------------------------------------------------------
def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def ms_since(ts):
    return time.ticks_diff(time.ticks_ms(), ts)


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
    # Reserved for obstacle limiting. For direct control use full scale.
    if front_m <= 0.0:
        return 0.0
    return 1.0


def map_cmd_to_pwm(v_lin, v_ang, front_m):
    v_lin = clamp(v_lin, -MAX_V_LIN, MAX_V_LIN)
    v_ang = clamp(v_ang, -MAX_V_ANG, MAX_V_ANG)

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


def speed_to_pwm(speed_norm):
    if speed_norm <= 0.0:
        return 0
    return int(MIN_PWM_DEADBAND + (MAX_PWM - MIN_PWM_DEADBAND) * clamp(speed_norm, 0.0, 1.0))


def vel_to_norm(v_mps):
    return clamp(abs(v_mps) / MAX_V_LIN, 0.05, 1.0)


# -----------------------------------------------------------------------------
# Encoder ISR
# -----------------------------------------------------------------------------
def enc_left_cb(pin):
    global left_ticks
    left_ticks += 1 if ENC_LEFT_B.value() == 0 else -1


def enc_right_cb(pin):
    global right_ticks
    right_ticks += 1 if ENC_RIGHT_B.value() == 0 else -1


ENC_LEFT_A.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=enc_left_cb)
ENC_RIGHT_A.irq(trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING, handler=enc_right_cb)


# -----------------------------------------------------------------------------
# Goal handling
# -----------------------------------------------------------------------------
def set_goal_from_cmd(data):
    global active_goal, current_state
    global cmd_v_lin, cmd_v_ang, cmd_estop, last_control_ms
    global target_left, target_right, current_left, current_right
    global left_ticks, right_ticks

    cmd = str(data.get("cmd", "")).upper()
    last_control_ms = time.ticks_ms()

    if cmd == "STOP":
        active_goal = None
        cmd_v_lin = 0.0
        cmd_v_ang = 0.0
        cmd_estop = False
        current_state = "IDLE"
        target_left = 0
        target_right = 0
        current_left = 0
        current_right = 0
        motor_stop()
        return True, "stopped"

    if cmd == "RESET_ODOM":
        left_ticks = 0
        right_ticks = 0
        active_goal = None
        current_state = "IDLE"
        return True, "odom_reset"

    if cmd == "DRIVE":
        dist = float(data.get("distance_m", 0.0))
        speed = float(data.get("speed", 0.20))
        active_goal = {
            "type": "DRIVE",
            "target_m": dist,
            "speed": speed,
            "sign": 1 if dist >= 0 else -1,
            "start_left": left_ticks,
            "start_right": right_ticks,
        }
        current_state = "DRIVE"
        return True, "drive"

    if cmd == "TURN":
        angle_deg = float(data.get("angle_deg", 0.0))
        speed = float(data.get("speed", 0.20))
        active_goal = {
            "type": "TURN",
            "target_rad": math.radians(angle_deg),
            "speed": speed,
            "sign": 1 if angle_deg >= 0 else -1,
            "start_left": left_ticks,
            "start_right": right_ticks,
        }
        current_state = "TURN"
        return True, "turn"

    return False, "unknown_cmd"


def step_goal():
    global active_goal, current_state

    if active_goal is None:
        return None

    g = active_goal

    if g["type"] == "DRIVE":
        dl = left_ticks - g["start_left"]
        dr = right_ticks - g["start_right"]
        travelled_m = abs((dl + dr) / 2.0 * DIST_PER_TICK)
        target_m = abs(g["target_m"])

        if travelled_m >= target_m:
            active_goal = None
            current_state = "IDLE"
            return (0, 0)

        remaining = target_m - travelled_m
        norm = vel_to_norm(g["speed"])
        if target_m > 0 and remaining < DECEL_FRACTION * target_m:
            norm *= 0.5

        pwm = speed_to_pwm(norm)
        signed_pwm = pwm * g["sign"]
        return (signed_pwm, signed_pwm)

    if g["type"] == "TURN":
        dl = left_ticks - g["start_left"]
        dr = right_ticks - g["start_right"]
        delta_rad = (dr - dl) * DIST_PER_TICK / TRACK_WIDTH_M
        turned_rad = abs(delta_rad)
        target_rad = abs(g["target_rad"])

        if turned_rad >= target_rad:
            active_goal = None
            current_state = "IDLE"
            return (0, 0)

        remaining = target_rad - turned_rad
        norm = vel_to_norm(g["speed"])
        if target_rad > 0 and remaining < DECEL_FRACTION * target_rad:
            norm *= 0.5

        pwm = speed_to_pwm(norm)
        left_pwm = pwm * g["sign"]
        right_pwm = -pwm * g["sign"]
        return (left_pwm, right_pwm)

    return None


# -----------------------------------------------------------------------------
# Control command handling (from /api/control)
# -----------------------------------------------------------------------------
def apply_control(data):
    global cmd_v_lin, cmd_v_ang, cmd_estop, active_goal, last_control_ms

    x = float(data.get("x", 0.0))
    y = float(data.get("y", 0.0))
    e = int(data.get("e", 0))
    speed_limit = float(data.get("speed_limit", 0.8))

    speed_limit = clamp(speed_limit, 0.1, 1.0)

    active_goal = None
    cmd_estop = bool(e)
    if cmd_estop:
        cmd_v_lin = 0.0
        cmd_v_ang = 0.0
    else:
        cmd_v_lin = y * MAX_V_LIN * speed_limit
        cmd_v_ang = -x * MAX_V_ANG * speed_limit

    last_control_ms = time.ticks_ms()


# -----------------------------------------------------------------------------
# Wi-Fi and HTTP
# -----------------------------------------------------------------------------
def setup_wifi():
    global wifi_ip

    mode = str(WIFI_MODE).upper()

    if mode == "STA":
        sta = network.WLAN(network.STA_IF)
        sta.active(True)
        if not sta.isconnected():
            sta.connect(STA_SSID, STA_PASSWORD)
            start = time.ticks_ms()
            while not sta.isconnected() and ms_since(start) < int(STA_TIMEOUT_MS):
                time.sleep_ms(200)
        if not sta.isconnected():
            raise RuntimeError("STA connect failed")
        wifi_ip = sta.ifconfig()[0]
        return

    ap = network.WLAN(network.AP_IF)
    ap.active(True)

    # Configure AP IP if requested.
    if AP_IP:
        try:
            ap.ifconfig((AP_IP, "255.255.255.0", AP_IP, AP_IP))
        except Exception:
            pass

    # Configure AP security / name.
    if AP_PASSWORD and len(AP_PASSWORD) >= 8:
        try:
            ap.config(essid=AP_SSID, password=AP_PASSWORD, authmode=network.AUTH_WPA2_PSK)
        except Exception:
            try:
                ap.config(essid=AP_SSID, password=AP_PASSWORD)
            except Exception:
                ap.config(essid=AP_SSID)
    else:
        ap.config(essid=AP_SSID)

    # Wait until AP has an address.
    for _ in range(50):
        ip = ap.ifconfig()[0]
        if ip and ip != "0.0.0.0":
            wifi_ip = ip
            return
        time.sleep_ms(100)

    wifi_ip = ap.ifconfig()[0]


def _send_all(sock, data):
    sent = 0
    total = len(data)
    while sent < total:
        n = sock.send(data[sent:])
        if n is None or n <= 0:
            break
        sent += n


def send_json(sock, obj, status="200 OK"):
    body = ujson.dumps(obj)
    hdr = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: application/json\r\n"
        "Connection: close\r\n"
        "Content-Length: " + str(len(body)) + "\r\n\r\n"
    )
    _send_all(sock, hdr.encode("utf-8") + body.encode("utf-8"))


def send_text(sock, text, status="200 OK", ctype="text/plain"):
    hdr = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: " + ctype + "\r\n"
        "Connection: close\r\n"
        "Content-Length: " + str(len(text)) + "\r\n\r\n"
    )
    _send_all(sock, hdr.encode("utf-8") + text.encode("utf-8"))


def read_request(sock):
    sock.settimeout(0.3)
    data = b""
    while b"\r\n\r\n" not in data and len(data) < 4096:
        chunk = sock.recv(512)
        if not chunk:
            break
        data += chunk

    if not data:
        return None

    sep = data.find(b"\r\n\r\n")
    if sep < 0:
        return None

    head = data[:sep].decode("utf-8", "ignore")
    body_bytes = data[sep + 4 :]

    lines = head.split("\r\n")
    if not lines:
        return None

    parts = lines[0].split(" ")
    if len(parts) < 2:
        return None

    method = parts[0].upper()
    path = parts[1]

    headers = {}
    for line in lines[1:]:
        i = line.find(":")
        if i > 0:
            k = line[:i].strip().lower()
            v = line[i + 1 :].strip()
            headers[k] = v

    clen = int(headers.get("content-length", "0") or "0")
    while len(body_bytes) < clen:
        chunk = sock.recv(clen - len(body_bytes))
        if not chunk:
            break
        body_bytes += chunk

    body = body_bytes[:clen].decode("utf-8", "ignore") if clen > 0 else ""
    return method, path, headers, body


def status_payload():
    return {
        "mode": "micropython_direct_wifi",
        "pico_connected": True,
        "pico_state": last_telem.get("pico_state", "IDLE"),
        "pico_state_raw": last_telem.get("pico_state_raw", "IDLE"),
        "last_control_age_ms": int(last_telem.get("last_control_age_ms", 0)),
        "wifi_ip": wifi_ip,
        "wifi_mode": str(WIFI_MODE).upper(),
        "http_port": int(HTTP_PORT),
    }


def telemetry_payload():
    return dict(last_telem)


def handle_http_client(sock):
    req = read_request(sock)
    if req is None:
        send_json(sock, {"error": "bad_request"}, "400 Bad Request")
        return

    method, path, _headers, body = req

    if method == "GET" and path == "/":
        send_text(
            sock,
            "AutoNexa Pico W direct control online\n"
            "Endpoints: /api/status /api/telemetry /api/control /api/goal /api/estop\n",
        )
        return

    if method == "GET" and path == "/api/status":
        send_json(sock, status_payload())
        return

    if method == "GET" and path == "/api/telemetry":
        send_json(sock, telemetry_payload())
        return

    if method == "POST" and path == "/api/control":
        try:
            data = ujson.loads(body or "{}")
            apply_control(data)
            send_json(sock, {"status": "ok"})
        except Exception as e:
            send_json(sock, {"status": "error", "message": str(e)}, "400 Bad Request")
        return

    if method == "POST" and path == "/api/goal":
        try:
            data = ujson.loads(body or "{}")
            ok, msg = set_goal_from_cmd(data)
            if ok:
                send_json(sock, {"status": "ok", "message": msg})
            else:
                send_json(sock, {"status": "error", "message": msg}, "400 Bad Request")
        except Exception as e:
            send_json(sock, {"status": "error", "message": str(e)}, "400 Bad Request")
        return

    if method == "POST" and path == "/api/estop":
        apply_control({"x": 0.0, "y": 0.0, "e": 1, "speed_limit": 0.0})
        send_json(sock, {"status": "stopped"})
        return

    send_json(sock, {"error": "not_found"}, "404 Not Found")


# -----------------------------------------------------------------------------
# Boot / startup
# -----------------------------------------------------------------------------
motor_stop()
set_servo(90)

for _ in range(4):
    LED.toggle()
    time.sleep_ms(120)
LED.value(0)

last_control_ms = time.ticks_ms()

print("[pico] starting Wi-Fi...")
setup_wifi()
print("[pico] Wi-Fi ready IP=", wifi_ip)

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", int(HTTP_PORT)))
srv.listen(2)
srv.settimeout(0)

print("[pico] HTTP listening on", wifi_ip, ":", HTTP_PORT)

last_ctrl = time.ticks_ms()
last_log = time.ticks_ms()
last_led = time.ticks_ms()

# -----------------------------------------------------------------------------
# Main loop
# -----------------------------------------------------------------------------
while True:
    # Serve one HTTP request if present.
    try:
        cl, _addr = srv.accept()
        try:
            handle_http_client(cl)
        finally:
            cl.close()
    except OSError:
        pass

    now = time.ticks_ms()

    # Control update
    if time.ticks_diff(now, last_ctrl) >= CONTROL_DT_MS:
        last_ctrl = now

        if active_goal is not None:
            result = step_goal()
            if result is not None:
                target_left, target_right = result
            current_left = ramp_to_target(current_left, target_left, RAMP_STEP)
            current_right = ramp_to_target(current_right, target_right, RAMP_STEP)
            motor_apply(current_left, current_right)
            set_servo(SERVO_STATE_ANGLE.get(current_state, 90))

        else:
            age_ms = ms_since(last_control_ms)

            if cmd_estop:
                current_state = "E_STOP"
                target_left = 0
                target_right = 0
            elif age_ms > WATCHDOG_MS:
                current_state = "IDLE"
                target_left = 0
                target_right = 0
            else:
                target_left, target_right = map_cmd_to_pwm(cmd_v_lin, cmd_v_ang, 9.9)
                if abs(target_left) > 0 or abs(target_right) > 0:
                    current_state = "TRACKING_PATH"
                else:
                    current_state = "IDLE"

            current_left = ramp_to_target(current_left, target_left, RAMP_STEP)
            current_right = ramp_to_target(current_right, target_right, RAMP_STEP)
            motor_apply(current_left, current_right)
            set_servo(SERVO_STATE_ANGLE.get(current_state, 90))

    # Telemetry update
    if time.ticks_diff(now, last_log) >= TELEMETRY_DT_MS:
        last_log = now

        dl = left_ticks
        dr = right_ticks
        dist_m = (dl + dr) / 2.0 * DIST_PER_TICK
        heading_deg = math.degrees((dr - dl) * DIST_PER_TICK / TRACK_WIDTH_M)

        goal_type = active_goal["type"] if active_goal else "NONE"
        raw_state = current_state

        last_telem = {
            "left_wheel_vel": round(current_left / MAX_PWM, 5),
            "right_wheel_vel": round(current_right / MAX_PWM, 5),
            "steering_pos": 0.0,
            "odom_vx": 0.0,
            "odom_wz": 0.0,
            "odom_x": round(dist_m, 4),
            "odom_y": 0.0,
            "odom_yaw": round(math.radians(heading_deg), 6),
            "pico_state": raw_state,
            "pico_state_raw": raw_state,
            "left_ticks": int(dl),
            "right_ticks": int(dr),
            "heading_deg": round(heading_deg, 2),
            "goal_type": goal_type,
            "last_control_age_ms": int(ms_since(last_control_ms)),
        }

    # LED heartbeat
    if time.ticks_diff(now, last_led) >= 500:
        last_led = now
        LED.toggle()

    # Keep memory healthy on long runs.
    gc.collect()
    time.sleep_ms(2)
