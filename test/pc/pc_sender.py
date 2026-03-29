#!/usr/bin/env python3
"""Manual scenario sender/logger for Pico nav packet tests."""

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

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        print(f"Connected to {self.port} @ {self.baud}")

    def close(self):
        self.running = False
        time.sleep(0.2)
        self.ser.close()

    def make_pkt(self, state="TRACKING_PATH", v_lin=0.0, v_ang=0.0, front=1.0, estop=False):
        self.seq += 1
        return {
            "t_ms": int(time.time() * 1000),
            "state": state,
            "v_lin": float(v_lin),
            "v_ang": float(v_ang),
            "dist_to_goal_m": 1.0,
            "heading_err_rad": 0.0,
            "progress_pct": 50,
            "eta_s": 10,
            "obstacle": {
                "front_m": float(front),
                "left_m": 1.0,
                "right_m": 1.0,
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

    def reader(self):
        while self.running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    stamp = time.time()
                    self.rx_log.append((stamp, line))
                    print("PICO:", line)
            except Exception:
                pass

    def scenario_basic(self):
        print("TC05 forward")
        self.hold(3.0, lambda: self.make_pkt(v_lin=0.20, v_ang=0.0, front=1.0))

        print("TC07 rotate right")
        self.hold(2.0, lambda: self.make_pkt(v_lin=0.0, v_ang=-0.8, front=1.0))

        print("TC08 curve left")
        self.hold(3.0, lambda: self.make_pkt(v_lin=0.18, v_ang=0.4, front=1.0))

        print("TC10 estop")
        self.hold(1.0, lambda: self.make_pkt(v_lin=0.2, v_ang=0.0, front=1.0, estop=True))

        print("TC11 watchdog: no packets for 1s")
        time.sleep(1.0)

    def scenario_obstacle(self):
        print("TC12 obstacle scaling")
        distances = [1.0, 0.6, 0.45, 0.3, 0.18, 0.5, 1.0]
        for d in distances:
            self.hold(1.5, lambda dd=d: self.make_pkt(v_lin=0.25, v_ang=0.0, front=dd))


def parse_args():
    p = argparse.ArgumentParser(description="Pico nav packet sender/logger")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port (e.g. /dev/ttyACM0 or COM5)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--rate", type=float, default=10.0, help="Packet send rate in Hz")
    p.add_argument("--scenario", choices=["basic", "obstacle", "all"], default="all")
    return p.parse_args()


def main():
    args = parse_args()
    s = Sender(args.port, args.baud, args.rate)
    s.open()

    th = threading.Thread(target=s.reader, daemon=True)
    th.start()

    try:
        time.sleep(1.0)
        if args.scenario in ("basic", "all"):
            s.scenario_basic()
        if args.scenario in ("obstacle", "all"):
            s.scenario_obstacle()
    finally:
        s.close()


if __name__ == "__main__":
    main()
