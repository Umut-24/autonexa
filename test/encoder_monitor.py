#!/usr/bin/env python3
"""
AutoNexa encoder monitor — a focused Tk UI for verifying the two wheel
encoders on the bench.

Reads the Pico's `TEL` telemetry on /dev/ttyACM0 at 115200 baud and shows
the encoder counts, their rate, the metres-equivalent distance per wheel,
and the integrated odometry — with inline notes on what each number means
and how to interpret it.

Run with no other process holding /dev/ttyACM0 (no nav2_pico_bridge.py,
no pico_gui.py). Push the robot by hand and watch.

    python3 test/encoder_monitor.py [--port /dev/ttyACM0]
"""
from __future__ import annotations

import argparse
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
except ImportError:
    raise SystemExit("pyserial required: sudo apt install python3-serial")

# ── Mirrors pico_firmware/include/config.h ──────────────────────
ENCODER_EDGES_PER_REV = 1320            # 11 CPR × 4 quadrature × 30 gear
WHEEL_RADIUS_M        = 0.033           # 66 mm diameter
WHEEL_CIRCUMFERENCE_M = 2.0 * math.pi * WHEEL_RADIUS_M
M_PER_EDGE            = WHEEL_CIRCUMFERENCE_M / ENCODER_EDGES_PER_REV

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

# Visual cues
COLOR_GOOD    = "#2a9d3f"
COLOR_WARN    = "#cc7722"
COLOR_BAD     = "#cc2222"
COLOR_MUTED   = "#888888"
COLOR_BG      = "#1e1e1e"
COLOR_PANEL   = "#262626"
COLOR_TEXT    = "#e6e6e6"
COLOR_DIM     = "#a0a0a0"


class EncoderMonitor(tk.Tk):
    def __init__(self, port: str, baud: int) -> None:
        super().__init__()
        self.title("AutoNexa Encoder Monitor")
        self.geometry("760x640")
        self.configure(bg=COLOR_BG)

        # Latest telemetry, written by the RX thread, read by the Tk redraw.
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._last_rx_monotonic: float | None = None
        # Tare offsets applied to displayed counts.
        self._tare_l = 0
        self._tare_r = 0
        # Auto-tare on the first TEL received so the display always opens
        # at 0/0 regardless of what counts the firmware has racked up
        # since boot (handling, vibration, earlier tests). The firmware
        # counters keep their absolute value; only the display is offset.
        self._auto_tared = False
        # Rate calculation (Δedges / Δt).
        self._prev_enc: tuple[int, int] | None = None
        self._prev_enc_time: float | None = None
        self._rate_l = 0.0
        self._rate_r = 0.0
        self._connected = False

        self._port = port
        self._baud = baud
        self._stop = threading.Event()
        self._serial: serial.Serial | None = None

        self._build_ui()
        threading.Thread(target=self._rx_loop, daemon=True).start()
        self.after(100, self._tick_ui)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── UI layout ────────────────────────────────────────────────
    def _build_ui(self) -> None:
        # Status bar
        bar = tk.Frame(self, bg=COLOR_BG)
        bar.pack(fill="x", padx=10, pady=(10, 4))
        tk.Label(bar, text=f"Serial: {self._port} @ {self._baud}",
                 bg=COLOR_BG, fg=COLOR_DIM, font=("monospace", 10)
                 ).pack(side="left")
        self.status_var = tk.StringVar(value="● Connecting…")
        self.status_lbl = tk.Label(bar, textvariable=self.status_var,
                                   bg=COLOR_BG, fg=COLOR_WARN,
                                   font=("monospace", 10, "bold"))
        self.status_lbl.pack(side="right")

        # Encoders pane (side by side)
        enc_frame = tk.Frame(self, bg=COLOR_BG)
        enc_frame.pack(fill="x", padx=10, pady=4)
        self.left_panel  = self._make_encoder_panel(enc_frame, "LEFT  (M1)")
        self.right_panel = self._make_encoder_panel(enc_frame, "RIGHT (M2)")
        self.left_panel["frame"].pack(side="left", fill="both", expand=True, padx=(0, 5))
        self.right_panel["frame"].pack(side="left", fill="both", expand=True, padx=(5, 0))

        # Odometry pane
        odom_frame = tk.LabelFrame(self, text=" ODOMETRY (integrated from both wheels) ",
                                   bg=COLOR_PANEL, fg=COLOR_TEXT,
                                   font=("monospace", 10, "bold"), bd=1)
        odom_frame.pack(fill="x", padx=10, pady=6)
        self.odom_vars = {k: tk.StringVar(value="—") for k in
                          ("x", "y", "yaw", "vx", "wz")}
        odom_grid = tk.Frame(odom_frame, bg=COLOR_PANEL)
        odom_grid.pack(padx=8, pady=8, fill="x")
        self._odom_cell(odom_grid, 0, 0, "x",   "m",      "x",   "Cumulative forward")
        self._odom_cell(odom_grid, 0, 1, "y",   "m",      "y",   "Cumulative sideways")
        self._odom_cell(odom_grid, 0, 2, "yaw", "°",      "yaw", "Cumulative heading")
        self._odom_cell(odom_grid, 1, 0, "vx",  "m/s",    "vx",  "Current forward speed")
        self._odom_cell(odom_grid, 1, 1, "wz",  "rad/s",  "wz",  "Current turn rate")

        # Meanings / hints
        meanings = tk.LabelFrame(self, text=" MEANINGS — what to look for ",
                                 bg=COLOR_PANEL, fg=COLOR_TEXT,
                                 font=("monospace", 10, "bold"), bd=1)
        meanings.pack(fill="both", expand=True, padx=10, pady=6)
        hint = (
            "count      = signed 4× quadrature edges since boot\n"
            "Δ/s        = edges per second (= wheel-speed indicator)\n"
            f"metres     = count × {M_PER_EDGE:.6f} m/edge\n"
            f"             (wheel circumference {WHEEL_CIRCUMFERENCE_M*100:.1f} cm "
            f"÷ {ENCODER_EDGES_PER_REV} edges/rev)\n"
            "x, y, yaw  = differential-drive odometry on the Pico\n"
            "             (yaw rate = (vR − vL) / TRACK_WIDTH)\n"
            "\n"
            "WHAT GOOD LOOKS LIKE:\n"
            "  • Push 1 m straight forward: each metres ≈ +1.0,\n"
            "    counts within ±20 % of each other, odom_x ≈ +1.0,\n"
            "    odom_yaw stays near 0 (small drift OK).\n"
            "  • Push backward: counts fall.\n"
            "  • Rotate in place: counts have OPPOSITE signs, odom_yaw climbs.\n"
            "\n"
            "WHAT BAD LOOKS LIKE:\n"
            "  • One side never moves              → loose wire / encoder dead\n"
            "  • Forward push, one count falls     → flip ENCODER_*_SIGN in config.h\n"
            "  • Rotate left, odom_yaw decreases   → encoders swapped L↔R\n"
            "  • Counts jump 1000s instantly       → noise / loose ground / signal short\n"
        )
        tk.Label(meanings, text=hint, bg=COLOR_PANEL, fg=COLOR_DIM,
                 font=("monospace", 9), justify="left", anchor="w").pack(
                 padx=10, pady=8, fill="both", expand=True)

        # Buttons
        btns = tk.Frame(self, bg=COLOR_BG)
        btns.pack(fill="x", padx=10, pady=(0, 10))
        tk.Button(btns, text="Reset display (tare)",
                  command=self._tare,
                  bg=COLOR_PANEL, fg=COLOR_TEXT,
                  activebackground=COLOR_MUTED, activeforeground=COLOR_TEXT
                  ).pack(side="left")
        self.last_rx_var = tk.StringVar(value="last TEL: —")
        tk.Label(btns, textvariable=self.last_rx_var,
                 bg=COLOR_BG, fg=COLOR_DIM, font=("monospace", 9)
                 ).pack(side="right")

    def _make_encoder_panel(self, parent, label: str) -> dict:
        f = tk.LabelFrame(parent, text=f" {label} ",
                          bg=COLOR_PANEL, fg=COLOR_TEXT,
                          font=("monospace", 11, "bold"), bd=1)
        count_var  = tk.StringVar(value="—")
        rate_var   = tk.StringVar(value="—")
        meters_var = tk.StringVar(value="—")
        # count: big, prominent
        inner = tk.Frame(f, bg=COLOR_PANEL)
        inner.pack(padx=10, pady=10, fill="both", expand=True)
        tk.Label(inner, text="count",
                 bg=COLOR_PANEL, fg=COLOR_DIM, font=("monospace", 9, "bold")
                 ).pack(anchor="w")
        count_lbl = tk.Label(inner, textvariable=count_var,
                             bg=COLOR_PANEL, fg=COLOR_TEXT,
                             font=("monospace", 22, "bold"))
        count_lbl.pack(anchor="e", pady=(0, 8))
        # rate
        tk.Label(inner, text="Δ / s (rate)",
                 bg=COLOR_PANEL, fg=COLOR_DIM, font=("monospace", 9, "bold")
                 ).pack(anchor="w")
        rate_lbl = tk.Label(inner, textvariable=rate_var,
                            bg=COLOR_PANEL, fg=COLOR_TEXT,
                            font=("monospace", 14))
        rate_lbl.pack(anchor="e", pady=(0, 8))
        # metres
        tk.Label(inner, text="metres rolled",
                 bg=COLOR_PANEL, fg=COLOR_DIM, font=("monospace", 9, "bold")
                 ).pack(anchor="w")
        meters_lbl = tk.Label(inner, textvariable=meters_var,
                              bg=COLOR_PANEL, fg=COLOR_TEXT,
                              font=("monospace", 14))
        meters_lbl.pack(anchor="e")
        return {
            "frame": f, "count_var": count_var, "rate_var": rate_var,
            "meters_var": meters_var,
            "count_lbl": count_lbl, "rate_lbl": rate_lbl,
        }

    def _odom_cell(self, parent, row, col, name, unit, key, hint):
        f = tk.Frame(parent, bg=COLOR_PANEL)
        f.grid(row=row, column=col, padx=8, pady=4, sticky="nsew")
        parent.columnconfigure(col, weight=1)
        tk.Label(f, text=f"{name} ({unit})",
                 bg=COLOR_PANEL, fg=COLOR_DIM, font=("monospace", 9, "bold")
                 ).pack(anchor="w")
        tk.Label(f, textvariable=self.odom_vars[key],
                 bg=COLOR_PANEL, fg=COLOR_TEXT, font=("monospace", 14)
                 ).pack(anchor="e")
        tk.Label(f, text=hint,
                 bg=COLOR_PANEL, fg=COLOR_MUTED, font=("monospace", 8, "italic")
                 ).pack(anchor="w")

    # ── Serial RX ────────────────────────────────────────────────
    def _rx_loop(self) -> None:
        backoff = 0.5
        while not self._stop.is_set():
            try:
                self._serial = serial.Serial(self._port, self._baud, timeout=0.5)
                self._connected = True
                backoff = 0.5
            except (serial.SerialException, OSError) as exc:
                self._connected = False
                self._status_msg(f"● Serial open failed: {exc}", error=True)
                time.sleep(backoff)
                backoff = min(backoff * 1.5, 4.0)
                continue
            try:
                while not self._stop.is_set():
                    raw = self._serial.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line.startswith("TEL "):
                        self._parse_tel(line[4:])
            except (serial.SerialException, OSError) as exc:
                self._connected = False
                self._status_msg(f"● Serial drop: {exc}", error=True)
            finally:
                try:
                    if self._serial:
                        self._serial.close()
                except Exception:
                    pass
                self._serial = None
            time.sleep(0.3)

    def _parse_tel(self, payload: str) -> None:
        parts = payload.split(",")
        # 13 fields since 2026-05-16: vx/wz added after odom_yaw.
        if len(parts) != 13:
            return
        try:
            data = {
                "ms":      int(parts[0]),
                "spdL":    int(parts[1]),
                "spdR":    int(parts[2]),
                "steer":   float(parts[3]),
                "encL":    int(parts[4]),
                "encR":    int(parts[5]),
                "x":       float(parts[6]),
                "y":       float(parts[7]),
                "yaw":     float(parts[8]),
                "vx":      float(parts[9]),
                "wz":      float(parts[10]),
                "estop":   int(parts[11]),
                "timeout": int(parts[12]),
            }
        except (ValueError, IndexError):
            return
        now = time.monotonic()
        # Rate calculation
        if self._prev_enc is not None and self._prev_enc_time is not None:
            dt = now - self._prev_enc_time
            if dt > 0:
                self._rate_l = (data["encL"] - self._prev_enc[0]) / dt
                self._rate_r = (data["encR"] - self._prev_enc[1]) / dt
        self._prev_enc = (data["encL"], data["encR"])
        self._prev_enc_time = now
        with self._lock:
            if not self._auto_tared:
                self._tare_l = data["encL"]
                self._tare_r = data["encR"]
                self._auto_tared = True
            self._latest = data
            self._last_rx_monotonic = now

    # ── Tk redraw ────────────────────────────────────────────────
    def _tick_ui(self) -> None:
        with self._lock:
            data = dict(self._latest) if self._latest else None
            rx_t = self._last_rx_monotonic
            rate_l = self._rate_l
            rate_r = self._rate_r

        if data is None:
            self.status_var.set("● Waiting for TEL…")
            self.status_lbl.configure(fg=COLOR_WARN)
        else:
            age_ms = int((time.monotonic() - rx_t) * 1000)
            if age_ms < 500:
                self.status_var.set("● Live")
                self.status_lbl.configure(fg=COLOR_GOOD)
            elif age_ms < 2000:
                self.status_var.set(f"● Stale ({age_ms} ms)")
                self.status_lbl.configure(fg=COLOR_WARN)
            else:
                self.status_var.set(f"● Stale ({age_ms} ms)")
                self.status_lbl.configure(fg=COLOR_BAD)
            self.last_rx_var.set(f"last TEL: {age_ms} ms ago   "
                                 f"(pico ms={data['ms']})")

            # Encoder panels — apply tare for display.
            encL_disp = data["encL"] - self._tare_l
            encR_disp = data["encR"] - self._tare_r
            mL = encL_disp * M_PER_EDGE
            mR = encR_disp * M_PER_EDGE
            self.left_panel["count_var"].set(f"{encL_disp:+d}")
            self.left_panel["rate_var"].set(f"{rate_l:+.0f}")
            self.left_panel["meters_var"].set(f"{mL:+.3f} m")
            self.right_panel["count_var"].set(f"{encR_disp:+d}")
            self.right_panel["rate_var"].set(f"{rate_r:+.0f}")
            self.right_panel["meters_var"].set(f"{mR:+.3f} m")

            # Colour cues on the rate: green if moving, dim if still.
            self.left_panel["rate_lbl"].configure(
                fg=self._rate_colour(rate_l))
            self.right_panel["rate_lbl"].configure(
                fg=self._rate_colour(rate_r))
            # Colour cues on the count: red if it diverges wildly from the
            # other (one wheel dead while pushing). Cheap heuristic: if
            # |rate_l - rate_r| is large AND one rate is ~0, paint it red.
            self.left_panel["count_lbl"].configure(
                fg=self._mismatch_colour(rate_l, rate_r))
            self.right_panel["count_lbl"].configure(
                fg=self._mismatch_colour(rate_r, rate_l))

            # Odometry
            self.odom_vars["x"].set(f"{data['x']:+.3f}")
            self.odom_vars["y"].set(f"{data['y']:+.3f}")
            self.odom_vars["yaw"].set(f"{math.degrees(data['yaw']):+.1f}")
            self.odom_vars["vx"].set(f"{data['vx']:+.3f}")
            self.odom_vars["wz"].set(f"{data['wz']:+.3f}")

        self.after(100, self._tick_ui)

    @staticmethod
    def _rate_colour(rate: float) -> str:
        if abs(rate) < 10:
            return COLOR_DIM
        return COLOR_GOOD

    @staticmethod
    def _mismatch_colour(rate_self: float, rate_other: float) -> str:
        # Both moving same way → text colour. One dead while other moving → red.
        if abs(rate_other) > 50 and abs(rate_self) < 10:
            return COLOR_BAD
        return COLOR_TEXT

    # ── Buttons ──────────────────────────────────────────────────
    def _tare(self) -> None:
        with self._lock:
            if self._latest is None:
                return
            self._tare_l = self._latest["encL"]
            self._tare_r = self._latest["encR"]

    def _status_msg(self, msg: str, error: bool = False) -> None:
        # Called from the RX thread; Tk widget update has to bounce to main.
        self.after(0, lambda: (
            self.status_var.set(msg),
            self.status_lbl.configure(fg=COLOR_BAD if error else COLOR_WARN),
        ))

    def _on_close(self) -> None:
        self._stop.set()
        try:
            if self._serial:
                self._serial.close()
        except Exception:
            pass
        self.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = ap.parse_args()
    EncoderMonitor(args.port, args.baud).mainloop()


if __name__ == "__main__":
    main()
