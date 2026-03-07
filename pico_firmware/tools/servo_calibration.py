#!/usr/bin/env python3
"""
Servo Calibration Tool — Autonexa Pico Firmware

Interactive tool to find the correct PWM pulse widths for:
  - Center (straight ahead)
  - Full left turn
  - Full right turn

Usage:
    python3 servo_calibration.py --port /dev/ttyUSB0
    python3 servo_calibration.py --port /dev/tty.usbmodem*
"""

import argparse
import serial
import time
import sys


def send_cmd(ser: serial.Serial, cmd: str) -> str:
    """Send a text command and read response."""
    ser.write((cmd + "\n").encode())
    time.sleep(0.1)
    response = ""
    while ser.in_waiting:
        response += ser.read(ser.in_waiting).decode(errors="replace")
    return response.strip()


def main():
    parser = argparse.ArgumentParser(description="Servo Calibration Tool")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print("╔══════════════════════════════════════╗")
    print("║  Autonexa Servo Calibration Tool     ║")
    print("╚══════════════════════════════════════╝")
    print()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2)  # Wait for Pico to reset
        # Flush startup messages
        ser.read(ser.in_waiting)
        print(f"Connected to {args.port}\n")
    except serial.SerialException as e:
        print(f"Error: Could not open {args.port}: {e}")
        sys.exit(1)

    results = {}

    # --- Step 1: Manual full-range sweep ---
    print("=" * 50)
    print("STEP 1: Full PWM Sweep")
    print("Sweeping from 500 µs to 2500 µs in 50 µs steps.")
    print("Watch the servo arm and note any mechanical limits.")
    print("=" * 50)

    input("Press Enter to start sweep...")

    for pwm_us in range(500, 2550, 50):
        resp = send_cmd(ser, f"SERVO_PWM {pwm_us}")
        sys.stdout.write(f"\r  PWM: {pwm_us:5d} µs")
        sys.stdout.flush()
        time.sleep(0.3)

    print("\n")

    # --- Step 2: Find center ---
    print("=" * 50)
    print("STEP 2: Find CENTER Position")
    print("Use +/- keys to adjust. Press Enter when wheels point straight.")
    print("  +  → increase PWM (move arm one direction)")
    print("  -  → decrease PWM (move arm other direction)")
    print("  Enter → confirm center")
    print("=" * 50)

    current_pwm = 1500
    send_cmd(ser, f"SERVO_PWM {current_pwm}")

    while True:
        cmd = input(f"  Center PWM = {current_pwm} µs  [+/-/Enter]: ").strip()
        if cmd == "+":
            current_pwm += 10
        elif cmd == "++":
            current_pwm += 50
        elif cmd == "-":
            current_pwm -= 10
        elif cmd == "--":
            current_pwm -= 50
        elif cmd == "":
            results["center"] = current_pwm
            break
        else:
            try:
                current_pwm = int(cmd)
            except ValueError:
                print("  Enter +, -, ++, --, a number, or Enter to confirm")
                continue
        send_cmd(ser, f"SERVO_PWM {current_pwm}")

    print(f"  ✅ CENTER = {results['center']} µs\n")

    # --- Step 3: Find full left ---
    print("=" * 50)
    print("STEP 3: Find FULL LEFT Position")
    print("=" * 50)

    current_pwm = results["center"]
    while True:
        cmd = input(f"  Left PWM = {current_pwm} µs  [+/-/Enter]: ").strip()
        if cmd == "+":
            current_pwm += 10
        elif cmd == "++":
            current_pwm += 50
        elif cmd == "-":
            current_pwm -= 10
        elif cmd == "--":
            current_pwm -= 50
        elif cmd == "":
            results["left"] = current_pwm
            break
        else:
            try:
                current_pwm = int(cmd)
            except ValueError:
                continue
        send_cmd(ser, f"SERVO_PWM {current_pwm}")

    print(f"  ✅ FULL LEFT = {results['left']} µs\n")

    # --- Step 4: Find full right ---
    print("=" * 50)
    print("STEP 4: Find FULL RIGHT Position")
    print("=" * 50)

    current_pwm = results["center"]
    while True:
        cmd = input(f"  Right PWM = {current_pwm} µs  [+/-/Enter]: ").strip()
        if cmd == "+":
            current_pwm += 10
        elif cmd == "++":
            current_pwm += 50
        elif cmd == "-":
            current_pwm -= 10
        elif cmd == "--":
            current_pwm -= 50
        elif cmd == "":
            results["right"] = current_pwm
            break
        else:
            try:
                current_pwm = int(cmd)
            except ValueError:
                continue
        send_cmd(ser, f"SERVO_PWM {current_pwm}")

    print(f"  ✅ FULL RIGHT = {results['right']} µs\n")

    # --- Return to center ---
    send_cmd(ser, f"SERVO_PWM {results['center']}")

    # --- Summary ---
    print("=" * 50)
    print("CALIBRATION RESULTS")
    print("=" * 50)
    print(f"  CENTER : {results['center']:5d} µs")
    print(f"  LEFT   : {results['left']:5d} µs")
    print(f"  RIGHT  : {results['right']:5d} µs")
    print(f"  Range  : {abs(results['left'] - results['right']):5d} µs")
    print()
    print("Update these values in pico_firmware/include/config.h:")
    print(f"  #define SERVO_PWM_CENTER_US  {results['center']}")
    print(f"  #define SERVO_PWM_MIN_US     {min(results['left'], results['right'])}")
    print(f"  #define SERVO_PWM_MAX_US     {max(results['left'], results['right'])}")
    print()

    ser.close()
    print("Done. Serial port closed.")


if __name__ == "__main__":
    main()
