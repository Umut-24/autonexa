#!/usr/bin/env python3
"""
AutoNexa Pico bench GUI — legacy C firmware (autonexa_pico.uf2).

Hold-to-drive WASD over USB CDC at 115200 baud, with a live `TEL` telemetry
display, raw per-channel motor diagnostics, and a calibrated servo trim.
No Pico-side changes.

Usage:
    python3 test/pico_gui.py [--port /dev/ttyACM0] [--baud 115200]

Keys (window must be focused):
    W / S  hold = SPEED ±throttle, release = SPEED 0
    A / D  hold = SERVO_PWM (center ∓ Δus), release = SERVO_PWM (center)
    SPACE / x   STOP
    e           toggle ESTOP / ESTOP_CLEAR

Bench panel (motor diagnostics):
    Per-channel RAW_PWM toggles bypass closed-loop. While any bench channel
    is non-zero, the WASD SPEED stream is suppressed so RAW_PWM and SPEED
    don't fight each other on different I2C registers every 50 ms.

Servo trim:
    The firmware's 1500 µs is "servo center", not necessarily "wheels
    straight" with the linkage installed. Set "Center µs" to the pulse
    width that points the wheels straight ahead, then A/D drive ±Δus.
    Saved to ~/.config/autonexa/pico_gui.json on quit.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.stderr.write("pyserial required: sudo apt install python3-serial\n")
    sys.exit(2)


PICO_VID = 0x2E8A
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

UI_TICK_MS = 50          # 20 Hz: drains rx queue, refreshes labels
DRIVE_TICK_MS = 50       # 20 Hz: re-issues SPEED/RAW_PWM/SERVO_PWM
RELEASE_DEBOUNCE_MS = 30 # X11 auto-repeat fires fake KeyRelease+KeyPress pairs
LOG_MAX_LINES = 200
TEL_STALE_MS = 500
RX_QUEUE_MAX = 512

# Drive defaults (overridden by config file)
THROTTLE_DEFAULT = 10
THROTTLE_MIN, THROTTLE_MAX = 1, 30

# Servo (matches firmware bounds in pico_firmware/include/config.h)
SERVO_US_MIN, SERVO_US_MAX = 1100, 1900
SERVO_CENTER_US_DEFAULT = 1650
STEER_DELTA_US_DEFAULT = 400
STEER_DELTA_MIN, STEER_DELTA_MAX = 10, 1000
CENTER_NUDGE_US = 50

# Bench (open-loop RAW_PWM, register 0x1F)
BENCH_PWM_MAG_DEFAULT = 30
BENCH_PWM_MIN, BENCH_PWM_MAX = 1, 100

CONFIG_PATH = os.path.expanduser("~/.config/autonexa/pico_gui.json")
CONFIG_KEYS = ("servo_center_us", "steer_delta_us", "throttle", "bench_pwm_mag")


def find_pico_port() -> str | None:
    for p in serial.tools.list_ports.comports():
        if getattr(p, "vid", None) == PICO_VID:
            return p.device
    return None


def load_config() -> dict:
    try:
        with open(CONFIG_PATH, "r") as f:
            data = json.load(f)
            return {k: data[k] for k in CONFIG_KEYS if k in data}
    except (OSError, ValueError):
        return {}


def save_config(values: dict) -> None:
    try:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        with open(CONFIG_PATH, "w") as f:
            json.dump(values, f, indent=2)
    except OSError:
        pass  # best-effort; not fatal


class PicoLink:
    """Thread-safe serial wrapper. Reader thread pushes lines into rx_queue."""

    def __init__(self) -> None:
        self._ser: serial.Serial | None = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self.rx_queue: queue.Queue[str] = queue.Queue(maxsize=RX_QUEUE_MAX)
        self.error: str | None = None

    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self, port: str, baud: int) -> None:
        self.close()
        self.error = None
        self._stop.clear()
        self._ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.2)  # let Pico flush boot banner
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=0.5)
            self._reader = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def send(self, line: str) -> bool:
        if not self.is_open():
            return False
        data = (line.rstrip("\n") + "\n").encode("ascii", errors="ignore")
        try:
            with self._write_lock:
                self._ser.write(data)  # type: ignore[union-attr]
                self._ser.flush()  # type: ignore[union-attr]
            return True
        except (serial.SerialException, OSError) as exc:
            self.error = str(exc)
            return False

    def _read_loop(self) -> None:
        ser = self._ser
        assert ser is not None
        while not self._stop.is_set():
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as exc:
                self.error = str(exc)
                return
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            try:
                self.rx_queue.put_nowait(line)
            except queue.Full:
                try:
                    self.rx_queue.get_nowait()
                    self.rx_queue.put_nowait(line)
                except queue.Empty:
                    pass


def clamp_int(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else hi if v > hi else v


class App:
    DRIVE_KEYS = ("w", "a", "s", "d")

    def __init__(self, root: tk.Tk, default_port: str, default_baud: int) -> None:
        self.root = root
        self.link = PicoLink()
        self.held: set[str] = set()
        self._release_after: dict[str, str] = {}
        self._last_speed_cmd: int | None = None
        self._last_servo_us: int | None = None
        self._last_raw_pwm: tuple[int, int] | None = None
        self._last_tel_monotonic: float | None = None
        self._estop_latched = False
        self._ui_after: str | None = None
        self._drive_after: str | None = None

        # Encoder-rate tracking (filled from TEL deltas).
        self._prev_enc: tuple[int, int] | None = None
        self._prev_enc_time: float | None = None
        self._enc_rate_l = 0.0
        self._enc_rate_r = 0.0

        # Bench RAW_PWM state — non-zero values activate bench mode.
        self.bench_m1 = 0
        self.bench_m2 = 0

        cfg = load_config()
        self._cfg = cfg

        root.title("AutoNexa Pico Bench (C firmware)")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        # ── Connection bar ───────────────────────────────────────
        bar = ttk.Frame(root, padding=(8, 6))
        bar.grid(row=0, column=0, sticky="ew")
        ttk.Label(bar, text="Port:").grid(row=0, column=0)
        self.port_var = tk.StringVar(value=default_port)
        ttk.Entry(bar, textvariable=self.port_var, width=24).grid(row=0, column=1, padx=4)
        self.baud_var = tk.IntVar(value=default_baud)
        self.connect_btn = ttk.Button(bar, text="Connect", command=self.on_connect)
        self.connect_btn.grid(row=0, column=2, padx=2)
        self.disconnect_btn = ttk.Button(bar, text="Disconnect", command=self.on_disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=3, padx=2)
        self.led = tk.Canvas(bar, width=16, height=16, highlightthickness=0)
        self.led_id = self.led.create_oval(2, 2, 14, 14, fill="#aa2222", outline="")
        self.led.grid(row=0, column=4, padx=(8, 4))
        self.conn_label = ttk.Label(bar, text="disconnected")
        self.conn_label.grid(row=0, column=5)

        # ── Body: drive | bench | telemetry ──────────────────────
        body = ttk.Frame(root, padding=(8, 4))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.columnconfigure(2, weight=1)

        self._build_drive_pane(body)
        self._build_bench_pane(body)
        self._build_telemetry_pane(body)

        # ── Log pane ─────────────────────────────────────────────
        log_frame = ttk.LabelFrame(root, text="Log", padding=(6, 4))
        log_frame.grid(row=2, column=0, sticky="nsew", padx=8, pady=(4, 4))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, height=12, width=86, wrap="none", state="disabled",
                           font=("monospace", 9))
        self.log.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scroll.set)
        ttk.Button(log_frame, text="Clear", command=self._clear_log).grid(row=1, column=0, sticky="e", pady=(4, 0))

        # ── Free-form serial entry ───────────────────────────────
        send_frame = ttk.Frame(root, padding=(8, 0, 8, 8))
        send_frame.grid(row=3, column=0, sticky="ew")
        send_frame.columnconfigure(1, weight=1)
        ttk.Label(send_frame, text="Send (advanced —\nRAW_PWM / I2C_WRITE):",
                  foreground="#666", font=("TkDefaultFont", 8), justify="right").grid(
            row=0, column=0, padx=(0, 4))
        self.raw_cmd_var = tk.StringVar()
        raw_entry = ttk.Entry(send_frame, textvariable=self.raw_cmd_var, font=("monospace", 9))
        raw_entry.grid(row=0, column=1, sticky="ew")
        raw_entry.bind("<Return>", lambda e: self._send_raw_entry())
        ttk.Button(send_frame, text="Send", command=self._send_raw_entry).grid(row=0, column=2, padx=(4, 0))

        root.columnconfigure(0, weight=1)
        root.rowconfigure(2, weight=1)

        self._bind_keys()
        self._set_connected(False)
        self._ui_after = root.after(UI_TICK_MS, self._ui_tick)

    # ── Pane builders ────────────────────────────────────────────
    def _build_drive_pane(self, parent: ttk.Frame) -> None:
        f = ttk.LabelFrame(parent, text="Drive", padding=(8, 6))
        f.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        btns = [
            ("ENABLE",       lambda: self._send_one("ENABLE")),
            ("DISABLE",      lambda: self._send_one("DISABLE")),
            ("STOP",         lambda: self._send_one("STOP")),
            ("Servo center", self._servo_center_pressed),
            ("ESTOP",        self._estop_pressed),
            ("ESTOP_CLEAR",  self._estop_clear_pressed),
            ("STATUS",       lambda: self._send_one("STATUS")),
            ("ENC_READ",     lambda: self._send_one("ENC_READ")),
        ]
        for i, (label, fn) in enumerate(btns):
            ttk.Button(f, text=label, command=fn, width=14).grid(
                row=i // 2, column=i % 2, padx=3, pady=2, sticky="ew")

        # Built-in firmware sweep: 1000→1500→2000→1500 µs × 3 cycles. Useful for
        # confirming both directions of servo travel during bringup. Spans both
        # columns so the descriptive label fits.
        ttk.Button(f, text="SERVO_SWEEP  (1000→1500→2000 µs ×3)",
                   command=lambda: self._send_one("SERVO_SWEEP")).grid(
            row=4, column=0, columnspan=2, padx=3, pady=2, sticky="ew")

        ttk.Separator(f, orient="horizontal").grid(row=5, column=0, columnspan=2, sticky="ew", pady=6)

        # Throttle slider (closed-loop SPEED magnitude).
        ttk.Label(f, text="Throttle (pulses/10ms)").grid(row=6, column=0, columnspan=2, sticky="w")
        self.throttle_var = tk.IntVar(value=int(self._cfg.get("throttle", THROTTLE_DEFAULT)))
        self.throttle_label = ttk.Label(f, text=f"{self.throttle_var.get()}")
        ttk.Scale(f, from_=THROTTLE_MIN, to=THROTTLE_MAX, orient="horizontal",
                  variable=self.throttle_var,
                  command=lambda v: self.throttle_label.configure(
                      text=f"{int(float(v))}")).grid(
            row=7, column=0, sticky="ew")
        self.throttle_label.grid(row=7, column=1, padx=4)

        # Servo: Center µs entry with ± nudge buttons.
        ttk.Label(f, text="Servo center (µs)").grid(row=8, column=0, columnspan=2, sticky="w", pady=(6, 0))
        self.center_us_var = tk.IntVar(value=int(self._cfg.get("servo_center_us", SERVO_CENTER_US_DEFAULT)))
        center_row = ttk.Frame(f)
        center_row.grid(row=9, column=0, columnspan=2, sticky="ew")
        center_row.columnconfigure(1, weight=1)
        ttk.Button(center_row, text=f"−{CENTER_NUDGE_US}", width=4,
                   command=lambda: self._nudge_center(-CENTER_NUDGE_US)).grid(row=0, column=0)
        center_entry = ttk.Entry(center_row, textvariable=self.center_us_var,
                                 font=("monospace", 10), justify="center", width=8)
        center_entry.grid(row=0, column=1, padx=4, sticky="ew")
        center_entry.bind("<Return>", lambda e: self._apply_center())
        center_entry.bind("<FocusOut>", lambda e: self._apply_center())
        ttk.Button(center_row, text=f"+{CENTER_NUDGE_US}", width=4,
                   command=lambda: self._nudge_center(+CENTER_NUDGE_US)).grid(row=0, column=2)

        # Servo: Steer Δus slider (pulse delta from center on hold-A/D).
        ttk.Label(f, text="Steer Δµs").grid(row=10, column=0, columnspan=2, sticky="w", pady=(6, 0))
        self.delta_us_var = tk.IntVar(value=int(self._cfg.get("steer_delta_us", STEER_DELTA_US_DEFAULT)))
        self.delta_us_label = ttk.Label(f, text=f"{self.delta_us_var.get()}")
        ttk.Scale(f, from_=STEER_DELTA_MIN, to=STEER_DELTA_MAX, orient="horizontal",
                  variable=self.delta_us_var,
                  command=lambda v: self.delta_us_label.configure(
                      text=f"{int(float(v))}")).grid(
            row=11, column=0, sticky="ew")
        self.delta_us_label.grid(row=11, column=1, padx=4)

        ttk.Separator(f, orient="horizontal").grid(row=12, column=0, columnspan=2, sticky="ew", pady=6)
        self.held_label = ttk.Label(f, text="Held: ░W ░A ░S ░D", font=("monospace", 11))
        self.held_label.grid(row=13, column=0, columnspan=2, sticky="w")
        ttk.Label(f, text="Hold W/A/S/D to drive\nSPACE / x = STOP\ne = toggle ESTOP",
                  foreground="#555").grid(row=14, column=0, columnspan=2, sticky="w", pady=(4, 0))
        f.columnconfigure(0, weight=1)

    def _build_bench_pane(self, parent: ttk.Frame) -> None:
        f = ttk.LabelFrame(parent, text="Bench (RAW_PWM, open-loop)", padding=(8, 6))
        f.grid(row=0, column=1, sticky="nsew", padx=(4, 4))

        ttk.Label(f, text="PWM magnitude").grid(row=0, column=0, columnspan=3, sticky="w")
        self.bench_pwm_mag_var = tk.IntVar(value=int(self._cfg.get("bench_pwm_mag", BENCH_PWM_MAG_DEFAULT)))
        self.bench_pwm_mag_label = ttk.Label(f, text=f"{self.bench_pwm_mag_var.get()}")
        ttk.Scale(f, from_=BENCH_PWM_MIN, to=BENCH_PWM_MAX, orient="horizontal",
                  variable=self.bench_pwm_mag_var,
                  command=lambda v: self.bench_pwm_mag_label.configure(
                      text=f"{int(float(v))}")).grid(
            row=1, column=0, columnspan=2, sticky="ew")
        self.bench_pwm_mag_label.grid(row=1, column=2, padx=4)

        ttk.Separator(f, orient="horizontal").grid(row=2, column=0, columnspan=3, sticky="ew", pady=6)

        # M1 row
        ttk.Label(f, text="M1", font=("TkDefaultFont", 10, "bold")).grid(row=3, column=0, sticky="w")
        ttk.Button(f, text="+ fwd", width=7, command=lambda: self._bench_set(1, +1)).grid(row=3, column=1, padx=2)
        ttk.Button(f, text="− rev", width=7, command=lambda: self._bench_set(1, -1)).grid(row=3, column=2, padx=2)
        ttk.Button(f, text="M1 stop", width=14, command=lambda: self._bench_set(1, 0)).grid(
            row=4, column=1, columnspan=2, padx=2, pady=(2, 4), sticky="ew")

        # M2 row
        ttk.Label(f, text="M2", font=("TkDefaultFont", 10, "bold")).grid(row=5, column=0, sticky="w", pady=(4, 0))
        ttk.Button(f, text="+ fwd", width=7, command=lambda: self._bench_set(2, +1)).grid(row=5, column=1, padx=2, pady=(4, 0))
        ttk.Button(f, text="− rev", width=7, command=lambda: self._bench_set(2, -1)).grid(row=5, column=2, padx=2, pady=(4, 0))
        ttk.Button(f, text="M2 stop", width=14, command=lambda: self._bench_set(2, 0)).grid(
            row=6, column=1, columnspan=2, padx=2, pady=(2, 4), sticky="ew")

        ttk.Separator(f, orient="horizontal").grid(row=7, column=0, columnspan=3, sticky="ew", pady=4)
        ttk.Button(f, text="ALL STOP (RAW_PWM 0 0 0 0)", command=lambda: self._bench_set(0, 0)).grid(
            row=8, column=0, columnspan=3, sticky="ew", pady=2)

        ttk.Separator(f, orient="horizontal").grid(row=9, column=0, columnspan=3, sticky="ew", pady=6)
        self.bench_status_label = ttk.Label(f, text="bench OFF (M1=0  M2=0)", font=("monospace", 10))
        self.bench_status_label.grid(row=10, column=0, columnspan=3, sticky="w")
        ttk.Label(f, text="While bench mode is active,\nWASD SPEED is suppressed.\nSet motors DISABLED first.",
                  foreground="#555").grid(row=11, column=0, columnspan=3, sticky="w", pady=(4, 0))
        f.columnconfigure(1, weight=1)
        f.columnconfigure(2, weight=1)

    def _build_telemetry_pane(self, parent: ttk.Frame) -> None:
        f = ttk.LabelFrame(parent, text="Telemetry (TEL)", padding=(8, 6))
        f.grid(row=0, column=2, sticky="nsew", padx=(4, 0))

        self.tel_vars: dict[str, tk.StringVar] = {
            k: tk.StringVar(value="—") for k in
            ("speed_L", "speed_R", "steer", "steer_deg",
             "enc_L", "enc_R", "enc_L_rate", "enc_R_rate",
             "odom_x", "odom_y", "odom_yaw",
             "estop", "timeout", "age")
        }

        rows = [
            ("speed_L",    "speed_L"),
            ("speed_R",    "speed_R"),
            ("steer",      "steer (rad)"),
            ("steer_deg",  "        (deg)"),
            ("enc_L",      "enc_L"),
            ("enc_L_rate", "        Δ/s"),
            ("enc_R",      "enc_R"),
            ("enc_R_rate", "        Δ/s"),
            ("odom_x",     "odom x (m)"),
            ("odom_y",     "odom y (m)"),
            ("odom_yaw",   "odom yaw (rad)"),
            ("estop",      "estop"),
            ("timeout",    "timeout"),
            ("age",        "TEL age (ms)"),
        ]
        self.tel_value_labels: dict[str, ttk.Label] = {}
        for i, (key, label) in enumerate(rows):
            ttk.Label(f, text=label + ":", anchor="w").grid(row=i, column=0, sticky="w", pady=1)
            lbl = ttk.Label(f, textvariable=self.tel_vars[key], font=("monospace", 10), anchor="e", width=14)
            lbl.grid(row=i, column=1, sticky="e", padx=(8, 0))
            self.tel_value_labels[key] = lbl
        f.columnconfigure(0, weight=1)

    # ── Connection ───────────────────────────────────────────────
    def on_connect(self) -> None:
        port = self.port_var.get().strip() or (find_pico_port() or DEFAULT_PORT)
        self.port_var.set(port)
        try:
            self.link.open(port, self.baud_var.get())
        except (serial.SerialException, OSError) as exc:
            self._append_log(f"[!] open {port} failed: {exc}")
            return
        self._append_log(f"[+] connected {port} @ {self.baud_var.get()}")
        self._send_one("STOP")  # known clean state
        self._set_connected(True)
        self._drive_after = self.root.after(DRIVE_TICK_MS, self._drive_tick)

    def on_disconnect(self) -> None:
        if self._drive_after is not None:
            self.root.after_cancel(self._drive_after)
            self._drive_after = None
        # Best-effort safe-state on the way out.
        self.link.send("STOP")
        self.link.send("DISABLE")
        self.link.send(f"SERVO_PWM {self._center_us()}")
        self.link.close()
        self.bench_m1 = 0
        self.bench_m2 = 0
        self._refresh_bench_status()
        self._set_connected(False)
        self._append_log("[-] disconnected")

    def _set_connected(self, ok: bool) -> None:
        self.led.itemconfigure(self.led_id, fill="#22aa22" if ok else "#aa2222")
        self.conn_label.configure(text="connected" if ok else "disconnected")
        self.connect_btn.configure(state="disabled" if ok else "normal")
        self.disconnect_btn.configure(state="normal" if ok else "disabled")

    # ── Servo trim helpers ──────────────────────────────────────
    def _center_us(self) -> int:
        try:
            v = int(self.center_us_var.get())
        except (ValueError, tk.TclError):
            v = SERVO_CENTER_US_DEFAULT
            self.center_us_var.set(v)
        return clamp_int(v, SERVO_US_MIN, SERVO_US_MAX)

    def _delta_us(self) -> int:
        return clamp_int(int(self.delta_us_var.get()), STEER_DELTA_MIN, STEER_DELTA_MAX)

    def _apply_center(self) -> None:
        # Clamp the entry's value, then re-emit if no A/D held so the wheels
        # immediately track the new neutral.
        c = self._center_us()
        self.center_us_var.set(c)
        if "a" not in self.held and "d" not in self.held and self.link.is_open():
            self.link.send(f"SERVO_PWM {c}")
            self._last_servo_us = c

    def _nudge_center(self, step: int) -> None:
        self.center_us_var.set(clamp_int(self._center_us() + step, SERVO_US_MIN, SERVO_US_MAX))
        self._apply_center()

    def _servo_center_pressed(self) -> None:
        c = self._center_us()
        self._send_one(f"SERVO_PWM {c}")
        self._last_servo_us = c

    # ── Bench helpers ───────────────────────────────────────────
    def _bench_set(self, channel: int, sign: int) -> None:
        """channel: 0=both stop, 1=M1, 2=M2. sign: +1/-1/0."""
        mag = clamp_int(int(self.bench_pwm_mag_var.get()), BENCH_PWM_MIN, BENCH_PWM_MAX)
        if channel == 0:
            self.bench_m1 = 0
            self.bench_m2 = 0
        elif channel == 1:
            self.bench_m1 = sign * mag
        elif channel == 2:
            self.bench_m2 = sign * mag
        self._refresh_bench_status()
        # Send immediately so feedback is instant (drive_tick will keep it alive).
        if self.link.is_open():
            self.link.send(f"RAW_PWM {self.bench_m1} {self.bench_m2} 0 0")
            self._last_raw_pwm = (self.bench_m1, self.bench_m2)

    def _refresh_bench_status(self) -> None:
        active = self.bench_m1 != 0 or self.bench_m2 != 0
        self.bench_status_label.configure(
            text=f"bench {'ON ' if active else 'OFF'} (M1={self.bench_m1:+d}  M2={self.bench_m2:+d})",
            foreground="#bb6600" if active else "")

    # ── Key bindings ─────────────────────────────────────────────
    def _bind_keys(self) -> None:
        for k in self.DRIVE_KEYS:
            self.root.bind_all(f"<KeyPress-{k}>",   lambda e, k=k: self._on_press(k))
            self.root.bind_all(f"<KeyRelease-{k}>", lambda e, k=k: self._on_release(k))
            self.root.bind_all(f"<KeyPress-{k.upper()}>",   lambda e, k=k: self._on_press(k))
            self.root.bind_all(f"<KeyRelease-{k.upper()}>", lambda e, k=k: self._on_release(k))
        self.root.bind_all("<KeyPress-space>", lambda e: self._send_one("STOP"))
        self.root.bind_all("<KeyPress-x>",     lambda e: self._send_one("STOP"))
        self.root.bind_all("<KeyPress-X>",     lambda e: self._send_one("STOP"))
        self.root.bind_all("<KeyPress-e>",     lambda e: self._toggle_estop())
        self.root.bind_all("<KeyPress-E>",     lambda e: self._toggle_estop())

    def _on_press(self, key: str) -> None:
        # Don't consume keystrokes while typing in the raw entry.
        focused = self.root.focus_get()
        if isinstance(focused, (ttk.Entry, tk.Entry)):
            return
        pending = self._release_after.pop(key, None)
        if pending is not None:
            try:
                self.root.after_cancel(pending)
            except Exception:
                pass
        if key not in self.held:
            self.held.add(key)
            self._refresh_held_label()

    def _on_release(self, key: str) -> None:
        focused = self.root.focus_get()
        if isinstance(focused, (ttk.Entry, tk.Entry)):
            return
        prev = self._release_after.pop(key, None)
        if prev is not None:
            try:
                self.root.after_cancel(prev)
            except Exception:
                pass
        self._release_after[key] = self.root.after(
            RELEASE_DEBOUNCE_MS, lambda: self._confirm_release(key))

    def _confirm_release(self, key: str) -> None:
        self._release_after.pop(key, None)
        if key in self.held:
            self.held.discard(key)
            self._refresh_held_label()

    def _refresh_held_label(self) -> None:
        parts = []
        for k in self.DRIVE_KEYS:
            mark = "▓" if k in self.held else "░"
            parts.append(f"{mark}{k.upper()}")
        self.held_label.configure(text="Held: " + " ".join(parts))

    # ── Drive loop ───────────────────────────────────────────────
    def _drive_tick(self) -> None:
        self._drive_after = None
        if not self.link.is_open():
            return

        bench_active = self.bench_m1 != 0 or self.bench_m2 != 0

        if bench_active:
            # Bench mode: open-loop RAW_PWM; suppress WASD SPEED entirely so we
            # don't fight the closed-loop register on alternating ticks.
            cmd = (self.bench_m1, self.bench_m2)
            self.link.send(f"RAW_PWM {self.bench_m1} {self.bench_m2} 0 0")
            self._last_raw_pwm = cmd
            # Reset WASD memory so a clean transition emits SPEED 0 once.
            self._last_speed_cmd = None
        else:
            # If we just left bench mode, send one explicit RAW_PWM 0s so the
            # board's open-loop register doesn't drift.
            if self._last_raw_pwm not in (None, (0, 0)):
                self.link.send("RAW_PWM 0 0 0 0")
                self._last_raw_pwm = (0, 0)

            # Throttle: W xor S held → SPEED ±N; neither → SPEED 0 once.
            w, s = "w" in self.held, "s" in self.held
            if w and not s:
                spd = int(self.throttle_var.get())
            elif s and not w:
                spd = -int(self.throttle_var.get())
            else:
                spd = 0
            if spd != 0:
                self.link.send(f"SPEED {spd}")
                self._last_speed_cmd = spd
            elif self._last_speed_cmd not in (None, 0):
                self.link.send("SPEED 0")
                self._last_speed_cmd = 0

        # Steering — independent of bench mode (servo is unrelated to motors).
        a, d = "a" in self.held, "d" in self.held
        center = self._center_us()
        delta = self._delta_us()
        if a and not d:
            us = clamp_int(center - delta, SERVO_US_MIN, SERVO_US_MAX)
        elif d and not a:
            us = clamp_int(center + delta, SERVO_US_MIN, SERVO_US_MAX)
        else:
            us = None
        if us is not None:
            self.link.send(f"SERVO_PWM {us}")
            self._last_servo_us = us
        elif self._last_servo_us is not None and self._last_servo_us != center:
            self.link.send(f"SERVO_PWM {center}")
            self._last_servo_us = center

        self._drive_after = self.root.after(DRIVE_TICK_MS, self._drive_tick)

    # ── UI tick: drain rx, refresh staleness ────────────────────
    def _ui_tick(self) -> None:
        self._ui_after = None
        for _ in range(64):
            try:
                line = self.link.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_rx(line)

        if self.link.error is not None and self.link.is_open():
            self._append_log(f"[!] serial error: {self.link.error}")
            self.on_disconnect()

        if self._last_tel_monotonic is not None:
            age_ms = int((time.monotonic() - self._last_tel_monotonic) * 1000)
            self.tel_vars["age"].set(f"{age_ms}")
            stale = age_ms > TEL_STALE_MS
            self.tel_value_labels["age"].configure(foreground="#cc2222" if stale else "")

        self._ui_after = self.root.after(UI_TICK_MS, self._ui_tick)

    def _handle_rx(self, line: str) -> None:
        if line.startswith("TEL "):
            self._handle_tel(line[4:])
            return
        self._append_log(f"< {line}")

    def _handle_tel(self, payload: str) -> None:
        parts = payload.split(",")
        if len(parts) != 11:
            return
        try:
            sL, sR = int(parts[1]), int(parts[2])
            steer = float(parts[3])
            eL, eR = int(parts[4]), int(parts[5])
            x, y, yaw = float(parts[6]), float(parts[7]), float(parts[8])
            estop = int(parts[9])
            timeout = int(parts[10])
        except ValueError:
            return

        now = time.monotonic()
        if self._prev_enc is not None and self._prev_enc_time is not None:
            dt = now - self._prev_enc_time
            if dt > 0:
                self._enc_rate_l = (eL - self._prev_enc[0]) / dt
                self._enc_rate_r = (eR - self._prev_enc[1]) / dt
        self._prev_enc = (eL, eR)
        self._prev_enc_time = now

        v = self.tel_vars
        v["speed_L"].set(f"{sL:+d}")
        v["speed_R"].set(f"{sR:+d}")
        v["steer"].set(f"{steer:+.3f}")
        v["steer_deg"].set(f"{math.degrees(steer):+.1f}")
        v["enc_L"].set(f"{eL}")
        v["enc_R"].set(f"{eR}")
        v["enc_L_rate"].set(f"{self._enc_rate_l:+.0f}")
        v["enc_R_rate"].set(f"{self._enc_rate_r:+.0f}")
        v["odom_x"].set(f"{x:+.3f}")
        v["odom_y"].set(f"{y:+.3f}")
        v["odom_yaw"].set(f"{yaw:+.2f}")
        v["estop"].set("YES" if estop else "no")
        v["timeout"].set("YES" if timeout else "no")
        self.tel_value_labels["estop"].configure(foreground="#cc2222" if estop else "")
        self.tel_value_labels["timeout"].configure(foreground="#cc2222" if timeout else "")
        self._last_tel_monotonic = now

    # ── Buttons / log ────────────────────────────────────────────
    def _send_one(self, cmd: str) -> None:
        if not self.link.is_open():
            self._append_log(f"[!] not connected; ignoring '{cmd}'")
            return
        if self.link.send(cmd):
            self._append_log(f"> {cmd}")
        else:
            self._append_log(f"[!] write failed: {cmd}")

    def _send_raw_entry(self) -> None:
        cmd = self.raw_cmd_var.get().strip()
        if not cmd:
            return
        self._send_one(cmd)
        self.raw_cmd_var.set("")

    def _estop_pressed(self) -> None:
        self._send_one("ESTOP")
        self._estop_latched = True

    def _estop_clear_pressed(self) -> None:
        self._send_one("ESTOP_CLEAR")
        self._estop_latched = False

    def _toggle_estop(self) -> None:
        if self._estop_latched:
            self._estop_clear_pressed()
        else:
            self._estop_pressed()

    def _append_log(self, line: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        excess = int(self.log.index("end-1c").split(".")[0]) - LOG_MAX_LINES
        if excess > 0:
            self.log.delete("1.0", f"{excess + 1}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    # ── Shutdown ─────────────────────────────────────────────────
    def _persist(self) -> None:
        save_config({
            "servo_center_us": self._center_us(),
            "steer_delta_us":  self._delta_us(),
            "throttle":        clamp_int(int(self.throttle_var.get()), THROTTLE_MIN, THROTTLE_MAX),
            "bench_pwm_mag":   clamp_int(int(self.bench_pwm_mag_var.get()), BENCH_PWM_MIN, BENCH_PWM_MAX),
        })

    def _on_close(self) -> None:
        self._persist()
        if self._ui_after is not None:
            try:
                self.root.after_cancel(self._ui_after)
            except Exception:
                pass
        if self._drive_after is not None:
            try:
                self.root.after_cancel(self._drive_after)
            except Exception:
                pass
        if self.link.is_open():
            self.link.send("RAW_PWM 0 0 0 0")
            self.link.send("STOP")
            self.link.send("DISABLE")
            self.link.send(f"SERVO_PWM {self._center_us()}")
            self.link.close()
        self.root.destroy()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = ap.parse_args()

    root = tk.Tk()
    App(root, args.port, args.baud)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
