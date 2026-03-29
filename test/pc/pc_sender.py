#!/usr/bin/env python3
"""Manual scenario sender/logger for Pico nav packet tests.

Usage:
    python pc_sender.py --port /dev/ttyACM0 --scenario basic
    python pc_sender.py --port COM5          --scenario distance
    python pc_sender.py --port /dev/ttyACM0 --scenario all
"""

import argparse
import json
import threading
import time
from collections import deque

import serial


class Sender:
    def __init__(self, port: str, baud: int, rate_hz: float):
        self.port = port
        self.baud = baud
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.seq = 0
        self.running = True
        self.rx_log = deque(maxlen=20000)
        self._last_state = "UNKNOWN"
        self._state_lock = threading.Lock()

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        print(f"Connected to {self.port} @ {self.baud}")

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.ser.close()

    # ------------------------------------------------------------------
    # Velocity stream packets (Nav2-compatible)
    # ------------------------------------------------------------------

    def make_pkt(self, state="TRACKING_PATH", v_lin=0.0, v_ang=0.0,
                 front=1.0, estop=False):
        self.seq += 1
        return {
            "t_ms":            int(time.time() * 1000),
            "state":           state,
            "v_lin":           float(v_lin),
            "v_ang":           float(v_ang),
            "dist_to_goal_m":  1.0,
            "heading_err_rad": 0.0,
            "progress_pct":    50,
            "eta_s":           10,
            "obstacle": {
                "front_m":       float(front),
                "left_m":        1.0,
                "right_m":       1.0,
                "emergency_stop": bool(estop),
            },
            "health": {"loc_ok": True, "planner_ok": True, "controller_ok": True},
            "seq": self.seq,
        }

    def send_pkt(self, pkt):
        self.ser.write((json.dumps(pkt) + "\n").encode("utf-8"))

    def hold(self, duration_s, fn_pkt):
        end_t = time.time() + duration_s
        while time.time() < end_t:
            self.send_pkt(fn_pkt())
            time.sleep(self.dt)

    # ------------------------------------------------------------------
    # Goal command packets (distance / angle)
    # ------------------------------------------------------------------

    def make_goal(self, cmd: str, **kwargs) -> dict:
        self.seq += 1
        pkt = {"cmd": cmd, "seq": self.seq}
        pkt.update(kwargs)
        return pkt

    def send_cmd(self, cmd: str, **kwargs):
        """Send a single goal command (DRIVE / TURN / STOP / RESET_ODOM)."""
        pkt = self.make_goal(cmd, **kwargs)
        self.ser.write((json.dumps(pkt) + "\n").encode("utf-8"))
        print(f"  >> sent: {pkt}")

    def wait_for_done(self, timeout_s: float = 15.0):
        """Block until Pico telemetry reports state=IDLE or timeout."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._state_lock:
                if self._last_state == "IDLE":
                    print("  << goal done (IDLE)")
                    return True
            time.sleep(0.05)
        print(f"  !! wait_for_done: timeout after {timeout_s}s")
        return False

    # ------------------------------------------------------------------
    # Reader thread — parses telemetry CSV and updates _last_state
    # ------------------------------------------------------------------

    def reader(self):
        while self.running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                stamp = time.time()
                self.rx_log.append((stamp, line))
                print("PICO:", line)

                # Parse telemetry CSV:
                # t_ms,cmd,left_ticks,right_ticks,dist_m,heading_deg,state,left_pwm,right_pwm
                parts = line.split(",")
                if len(parts) == 9:
                    state_field = parts[6].strip()
                    with self._state_lock:
                        self._last_state = state_field
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Scenarios
    # ------------------------------------------------------------------

    def scenario_basic(self):
        """Original velocity-stream test cases."""
        print("\n=== scenario_basic ===")

        print("TC05: forward 3s")
        self.hold(3.0, lambda: self.make_pkt(v_lin=0.20, v_ang=0.0, front=1.0))

        print("TC07: rotate right 2s")
        self.hold(2.0, lambda: self.make_pkt(v_lin=0.0, v_ang=-0.8, front=1.0))

        print("TC08: curve left 3s")
        self.hold(3.0, lambda: self.make_pkt(v_lin=0.18, v_ang=0.4, front=1.0))

        print("TC10: emergency stop")
        self.hold(1.0, lambda: self.make_pkt(v_lin=0.2, v_ang=0.0, front=1.0, estop=True))

        print("TC11: watchdog — no packets for 1s")
        time.sleep(1.0)

    def scenario_obstacle(self):
        """Obstacle distance scaling test cases."""
        print("\n=== scenario_obstacle ===")
        distances = [1.0, 0.6, 0.45, 0.3, 0.18, 0.5, 1.0]
        for d in distances:
            print(f"TC12: obstacle @ {d:.2f}m")
            self.hold(1.5, lambda dd=d: self.make_pkt(v_lin=0.25, v_ang=0.0, front=dd))

    def scenario_distance(self):
        """Encoder-based distance and angle goal tests.

        Each test sends one command and waits for Pico to report IDLE.
        Measure actual travel / rotation against the target to verify accuracy.
        """
        print("\n=== scenario_distance ===")

        # Reset odometry first
        self.send_cmd("RESET_ODOM")
        time.sleep(0.3)

        print("TC-D01: Go forward 1.0 m  (target <5% error)")
        self.send_cmd("DRIVE", distance_m=1.0, speed=0.25)
        self.wait_for_done(timeout_s=12)
        time.sleep(0.5)

        print("TC-D02: Turn right 15 degrees")
        self.send_cmd("TURN", angle_deg=15.0, speed=0.20)
        self.wait_for_done(timeout_s=6)
        time.sleep(0.5)

        print("TC-D03: Turn left 30 degrees")
        self.send_cmd("TURN", angle_deg=-30.0, speed=0.20)
        self.wait_for_done(timeout_s=8)
        time.sleep(0.5)

        print("TC-D04: Go forward 0.5 m")
        self.send_cmd("DRIVE", distance_m=0.5, speed=0.25)
        self.wait_for_done(timeout_s=8)
        time.sleep(0.5)

        print("TC-D05: Reverse 0.3 m")
        self.send_cmd("DRIVE", distance_m=-0.3, speed=0.20)
        self.wait_for_done(timeout_s=6)
        time.sleep(0.5)

        print("TC-D06: Turn right 90 degrees")
        self.send_cmd("TURN", angle_deg=90.0, speed=0.20)
        self.wait_for_done(timeout_s=12)
        time.sleep(0.5)

        print("TC-D07: Emergency stop mid-drive")
        self.send_cmd("DRIVE", distance_m=2.0, speed=0.25)
        time.sleep(1.0)   # let it drive for 1 second
        self.send_cmd("STOP")
        time.sleep(0.5)

        print("\n=== scenario_distance complete ===")


def parse_args():
    p = argparse.ArgumentParser(description="Pico nav packet sender/logger")
    p.add_argument("--port",     default="/dev/ttyACM0",
                   help="Serial port (e.g. /dev/ttyACM0 or COM5)")
    p.add_argument("--baud",     type=int,   default=115200)
    p.add_argument("--rate",     type=float, default=10.0,
                   help="Packet send rate in Hz (velocity stream mode)")
    p.add_argument("--scenario", choices=["basic", "obstacle", "distance", "all"],
                   default="distance")
    return p.parse_args()


def main():
    args = parse_args()
    s = Sender(args.port, args.baud, args.rate)
    s.open()

    th = threading.Thread(target=s.reader, daemon=True)
    th.start()

    try:
        time.sleep(1.0)   # wait for PICO_READY
        if args.scenario in ("basic", "all"):
            s.scenario_basic()
        if args.scenario in ("obstacle", "all"):
            s.scenario_obstacle()
        if args.scenario in ("distance", "all"):
            s.scenario_distance()
    finally:
        s.close()


if __name__ == "__main__":
    main()
