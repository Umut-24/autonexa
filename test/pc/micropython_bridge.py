#!/usr/bin/env python3
"""Lightweight HTTP bridge between Flutter app and Pico MicroPython firmware.

No ROS2 dependencies — just pyserial + flask.

Accepts the same /api/control, /api/telemetry, /api/status endpoints as the
ROS2 mobile bridge so the Flutter app can switch modes with minimal changes.

Usage:
    python micropython_bridge.py --port /dev/ttyACM0
    python micropython_bridge.py --port COM8 --http-port 5001
"""

import argparse
import json
import math
import threading
import time

import serial
from flask import Flask, jsonify, request

# ---------------------------------------------------------------------------
# Constants (match Pico firmware and ros2_mobile_bridge.py)
# ---------------------------------------------------------------------------
MAX_V_LIN = 0.35       # m/s  (same as ros2_mobile_bridge)
MAX_V_ANG = 0.8        # rad/s
MAX_PWM = 50000
WATCHDOG_MS = 300

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Shared state (protected by locks)
# ---------------------------------------------------------------------------
ser_lock = threading.Lock()       # guards serial writes
telem_lock = threading.Lock()     # guards telemetry dict

ser_port: serial.Serial | None = None
pico_connected = False
seq_counter = 0
last_control_time = 0.0

latest_telemetry = {
    "t_ms": 0,
    "goal_type": "NONE",
    "left_ticks": 0,
    "right_ticks": 0,
    "dist_m": 0.0,
    "heading_deg": 0.0,
    "state": "UNKNOWN",
    "left_pwm": 0,
    "right_pwm": 0,
}


def next_seq() -> int:
    global seq_counter
    seq_counter += 1
    return seq_counter


def _normalized_pico_state(t: dict) -> str:
    """Map watchdog-idle FAILED state to IDLE for clearer mobile UX."""
    raw = str(t.get("state", "UNKNOWN"))
    left_pwm = int(t.get("left_pwm", 0))
    right_pwm = int(t.get("right_pwm", 0))
    moving = (left_pwm != 0) or (right_pwm != 0)
    recently_commanded = (time.time() - last_control_time) < 1.5
    if raw == "FAILED" and (not moving) and (not recently_commanded):
        return "IDLE"
    return raw


# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------
def serial_reader():
    global pico_connected, latest_telemetry

    while True:
        if ser_port is None or not ser_port.is_open:
            pico_connected = False
            time.sleep(1.0)
            continue

        try:
            raw = ser_port.readline()
            if not raw:
                continue
            line = raw.decode(errors="ignore").strip()
            if not line:
                continue

            if line == "PICO_READY":
                pico_connected = True
                print("[bridge] PICO_READY received")
                continue

            parts = line.split(",")
            if len(parts) == 9:
                pico_connected = True
                with telem_lock:
                    latest_telemetry = {
                        "t_ms": int(parts[0]),
                        "goal_type": parts[1],
                        "left_ticks": int(parts[2]),
                        "right_ticks": int(parts[3]),
                        "dist_m": float(parts[4]),
                        "heading_deg": float(parts[5]),
                        "state": parts[6],
                        "left_pwm": int(parts[7]),
                        "right_pwm": int(parts[8]),
                    }
        except serial.SerialException:
            pico_connected = False
            print("[bridge] Serial read error — will retry")
            time.sleep(1.0)
        except Exception as e:
            print(f"[bridge] Reader error: {e}")


def serial_write(data: str):
    """Write a line to serial port (thread-safe)."""
    if ser_port is None or not ser_port.is_open:
        return False
    try:
        with ser_lock:
            ser_port.write((data + "\n").encode("utf-8"))
        return True
    except Exception as e:
        print(f"[bridge] Serial write error: {e}")
        return False


# ---------------------------------------------------------------------------
# Velocity stream packet (same format as pc_sender.py make_pkt)
# ---------------------------------------------------------------------------
def make_velocity_pkt(v_lin: float, v_ang: float, estop: bool = False) -> dict:
    return {
        "state": "TRACKING_PATH",
        "v_lin": float(v_lin),
        "v_ang": float(v_ang),
        "obstacle": {
            "front_m": 9.9,
            "left_m": 9.9,
            "right_m": 9.9,
            "emergency_stop": bool(estop),
        },
        "health": {"loc_ok": True, "planner_ok": True, "controller_ok": True},
        "seq": next_seq(),
    }


# ---------------------------------------------------------------------------
# Flask endpoints
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return "<h3>AutoNexa MicroPython Bridge</h3><p>Endpoints: /api/status, /api/control, /api/telemetry, /api/goal, /api/estop</p>"


@app.route("/api/status", methods=["GET"])
def api_status():
    with telem_lock:
        t = dict(latest_telemetry)
    state = _normalized_pico_state(t)
    return jsonify({
        "mode": "micropython",
        "pico_connected": pico_connected,
        "pico_state": state,
        "pico_state_raw": t.get("state", "UNKNOWN"),
        "last_control_age_ms": int(max(0.0, (time.time() - last_control_time) * 1000.0)),
    })


@app.route("/api/control", methods=["POST"])
def api_control():
    """Joystick control — same format as ros2_mobile_bridge."""
    global last_control_time
    data = request.get_json(silent=True) or {}
    x = float(data.get("x", 0.0))
    y = float(data.get("y", 0.0))
    e = int(data.get("e", 0))
    speed_limit = float(data.get("speed_limit", 0.5))

    if e:
        pkt = make_velocity_pkt(0.0, 0.0, estop=True)
    else:
        v_lin = y * MAX_V_LIN * speed_limit
        v_ang = -x * MAX_V_ANG * speed_limit
        pkt = make_velocity_pkt(v_lin, v_ang)

    ok = serial_write(json.dumps(pkt))
    last_control_time = time.time()
    return jsonify({"status": "ok" if ok else "serial_error"})


@app.route("/api/telemetry", methods=["GET"])
def api_telemetry():
    """Return Pico telemetry mapped to PicoTelemetry.fromJson() field names."""
    with telem_lock:
        t = dict(latest_telemetry)

    left_norm = t["left_pwm"] / MAX_PWM if MAX_PWM else 0
    right_norm = t["right_pwm"] / MAX_PWM if MAX_PWM else 0

    state = _normalized_pico_state(t)
    return jsonify({
        # Fields expected by PicoTelemetry.fromJson()
        "left_wheel_vel": left_norm,
        "right_wheel_vel": right_norm,
        "steering_pos": 0,
        "odom_vx": 0,
        "odom_wz": 0,
        "odom_x": t["dist_m"],
        "odom_y": 0,
        "odom_yaw": math.radians(t["heading_deg"]),
        # Extra MicroPython-specific fields
        "pico_state": state,
        "pico_state_raw": t["state"],
        "left_ticks": t["left_ticks"],
        "right_ticks": t["right_ticks"],
        "heading_deg": t["heading_deg"],
        "goal_type": t["goal_type"],
        "last_control_age_ms": int(max(0.0, (time.time() - last_control_time) * 1000.0)),
    })


@app.route("/api/goal", methods=["POST"])
def api_goal():
    """Forward goal commands (DRIVE, TURN, STOP, RESET_ODOM) to Pico."""
    global last_control_time
    data = request.get_json(silent=True) or {}
    cmd = data.get("cmd", "").upper()

    if cmd not in ("DRIVE", "TURN", "STOP", "RESET_ODOM"):
        return jsonify({"error": f"Unknown command: {cmd}"}), 400

    pkt = {"cmd": cmd, "seq": next_seq()}

    if cmd == "DRIVE":
        pkt["distance_m"] = float(data.get("distance_m", 0))
        pkt["speed"] = float(data.get("speed", 0.20))
    elif cmd == "TURN":
        pkt["angle_deg"] = float(data.get("angle_deg", 0))
        pkt["speed"] = float(data.get("speed", 0.20))

    ok = serial_write(json.dumps(pkt))
    if ok:
        last_control_time = time.time()
    return jsonify({"status": "ok" if ok else "serial_error", "sent": pkt})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    """Emergency stop — sends STOP command to Pico."""
    global last_control_time
    pkt = {"cmd": "STOP", "seq": next_seq()}
    serial_write(json.dumps(pkt))
    # Also send zero velocity with estop flag
    vel_pkt = make_velocity_pkt(0.0, 0.0, estop=True)
    serial_write(json.dumps(vel_pkt))
    last_control_time = time.time()
    return jsonify({"status": "stopped"})


@app.route("/api/estop_clear", methods=["POST"])
def api_estop_clear():
    return jsonify({"status": "cleared"})


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    global ser_port, seq_counter

    parser = argparse.ArgumentParser(description="MicroPython HTTP bridge for AutoNexa")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Pico serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--http-port", type=int, default=5001,
                        help="HTTP server port (default: 5001)")
    args = parser.parse_args()

    # Start from a very large monotonic-ish value so Pico doesn't reject
    # commands as "old" after bridge restarts.
    # Use microseconds (not milliseconds) to stay far above any previous
    # ms-based counter values.
    seq_counter = int(time.time() * 1_000_000)

    print(f"[bridge] Opening serial {args.port} @ {args.baud}")
    try:
        ser_port = serial.Serial(args.port, args.baud, timeout=0.5)
        print(f"[bridge] Serial port opened")
    except Exception as e:
        print(f"[bridge] WARNING: Could not open serial port: {e}")
        print(f"[bridge] Starting HTTP server anyway (telemetry will be empty)")

    reader_thread = threading.Thread(target=serial_reader, daemon=True)
    reader_thread.start()

    print(f"[bridge] HTTP server starting on 0.0.0.0:{args.http_port}")
    app.run(host="0.0.0.0", port=args.http_port, threaded=True)


if __name__ == "__main__":
    main()
