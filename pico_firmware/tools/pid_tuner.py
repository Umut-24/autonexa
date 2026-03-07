#!/usr/bin/env python3
"""
PID Tuner — Autonexa Pico Firmware

Real-time PID tuning tool with live plotting.
Sends RPM setpoints, reads actual RPM, and plots the response.

Usage:
    python3 pid_tuner.py --port /dev/ttyUSB0
    python3 pid_tuner.py --port /dev/tty.usbmodem* --target-rpm 60
"""

import argparse
import serial
import time
import sys
import threading
from collections import deque

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False
    print("Warning: matplotlib not found. Plotting disabled.")


# ── Data collection ──────────────────────────────────────────────
MAX_POINTS = 500
timestamps   = deque(maxlen=MAX_POINTS)
target_left  = deque(maxlen=MAX_POINTS)
actual_left  = deque(maxlen=MAX_POINTS)
target_right = deque(maxlen=MAX_POINTS)
actual_right = deque(maxlen=MAX_POINTS)

running = True
ser = None


def parse_telemetry(line: str):
    """Parse TEL lines: TEL timestamp,tgt_l,act_l,tgt_r,act_r,steer,estop,timeout"""
    if not line.startswith("TEL "):
        return
    try:
        parts = line[4:].split(",")
        if len(parts) >= 5:
            t = float(parts[0]) / 1000.0  # ms → s
            timestamps.append(t)
            target_left.append(float(parts[1]))
            actual_left.append(float(parts[2]))
            target_right.append(float(parts[3]))
            actual_right.append(float(parts[4]))
    except (ValueError, IndexError):
        pass


def serial_reader():
    """Background thread to read serial data."""
    global running
    buf = ""
    while running:
        try:
            if ser and ser.in_waiting:
                data = ser.read(ser.in_waiting).decode(errors="replace")
                buf += data
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        parse_telemetry(line)
                        # Print non-TEL lines
                        if not line.startswith("TEL "):
                            print(f"  < {line}")
            else:
                time.sleep(0.01)
        except Exception:
            time.sleep(0.01)


def send_cmd(cmd: str):
    """Send command to Pico."""
    if ser:
        ser.write((cmd + "\n").encode())
        time.sleep(0.05)


def animate(frame, ax_l, ax_r, line_tgt_l, line_act_l, line_tgt_r, line_act_r):
    """Update plot."""
    if not timestamps:
        return

    t = list(timestamps)
    t0 = t[0]
    t_rel = [x - t0 for x in t]

    line_tgt_l.set_data(t_rel, list(target_left))
    line_act_l.set_data(t_rel, list(actual_left))
    line_tgt_r.set_data(t_rel, list(target_right))
    line_act_r.set_data(t_rel, list(actual_right))

    for ax in (ax_l, ax_r):
        ax.relim()
        ax.autoscale_view()

    return line_tgt_l, line_act_l, line_tgt_r, line_act_r


def interactive_commands(initial_rpm: float):
    """Interactive command prompt (runs in main thread when no plotting)."""
    global running

    send_cmd("ENABLE")
    time.sleep(0.1)
    send_cmd(f"TARGET {initial_rpm}")

    print("\nInteractive mode. Commands:")
    print("  t <rpm>        - Set target RPM (both wheels)")
    print("  p <kp> <ki> <kd> - Set PID gains")
    print("  s              - Step test (0 → target → 0)")
    print("  stop           - Stop motors")
    print("  status         - Print status")
    print("  q              - Quit")
    print()

    while running:
        try:
            cmd = input("pid> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue
        elif cmd == "q":
            break
        elif cmd.startswith("t "):
            rpm = cmd[2:].strip()
            send_cmd(f"TARGET {rpm}")
        elif cmd.startswith("p "):
            send_cmd(f"PID {cmd[2:]}")
        elif cmd == "s":
            print("  Step test: 0 → target → 0")
            send_cmd("TARGET 0")
            time.sleep(1)
            send_cmd(f"TARGET {initial_rpm}")
            time.sleep(3)
            send_cmd("TARGET 0")
            time.sleep(2)
            print("  Step test complete.")
        elif cmd == "stop":
            send_cmd("STOP")
        elif cmd == "status":
            send_cmd("STATUS")
        else:
            send_cmd(cmd)

    send_cmd("STOP")
    send_cmd("DISABLE")
    running = False


def main():
    global ser, running

    parser = argparse.ArgumentParser(description="PID Tuner for Autonexa Pico")
    parser.add_argument("--port", required=True, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--target-rpm", type=float, default=60.0,
                        help="Initial target RPM")
    parser.add_argument("--no-plot", action="store_true",
                        help="Disable live plot (text-only mode)")
    args = parser.parse_args()

    print("╔══════════════════════════════════════╗")
    print("║  Autonexa PID Tuner                  ║")
    print("╚══════════════════════════════════════╝")
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2)
        ser.read(ser.in_waiting)  # Flush
        print(f"Connected to {args.port}")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Start serial reader thread
    reader_thread = threading.Thread(target=serial_reader, daemon=True)
    reader_thread.start()

    # Enable motors and set initial target
    send_cmd("ENABLE")
    time.sleep(0.2)
    send_cmd(f"TARGET {args.target_rpm}")

    if HAS_PLOT and not args.no_plot:
        # Live plotting mode
        fig, (ax_l, ax_r) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        ax_l.set_title("Left Wheel")
        ax_l.set_ylabel("RPM")
        ax_l.grid(True, alpha=0.3)
        line_tgt_l, = ax_l.plot([], [], "b--", label="Target", linewidth=1.5)
        line_act_l, = ax_l.plot([], [], "r-", label="Actual", linewidth=1.5)
        ax_l.legend(loc="upper right")

        ax_r.set_title("Right Wheel")
        ax_r.set_xlabel("Time [s]")
        ax_r.set_ylabel("RPM")
        ax_r.grid(True, alpha=0.3)
        line_tgt_r, = ax_r.plot([], [], "b--", label="Target", linewidth=1.5)
        line_act_r, = ax_r.plot([], [], "g-", label="Actual", linewidth=1.5)
        ax_r.legend(loc="upper right")

        fig.suptitle(f"PID Tuner — Target: {args.target_rpm} RPM")
        fig.tight_layout()

        # Start interactive commands in background
        cmd_thread = threading.Thread(
            target=interactive_commands,
            args=(args.target_rpm,),
            daemon=True
        )
        cmd_thread.start()

        ani = animation.FuncAnimation(
            fig, animate,
            fargs=(ax_l, ax_r, line_tgt_l, line_act_l, line_tgt_r, line_act_r),
            interval=100, blit=False, cache_frame_data=False
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
    else:
        # Text-only mode
        interactive_commands(args.target_rpm)

    running = False
    send_cmd("STOP")
    send_cmd("DISABLE")
    ser.close()
    print("\nDone. Serial port closed.")


if __name__ == "__main__":
    main()
