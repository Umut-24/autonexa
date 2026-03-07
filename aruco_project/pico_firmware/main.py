"""
Pico WH — Ackermann Chassis Controller (Main Entry Point)
==========================================================

This is the main program that runs on the Raspberry Pi Pico WH.

Architecture:
    ┌──────────┐   UDP    ┌──────────┐   I2C    ┌───────────────┐
    │  Phone   │ ───────► │ Pico WH  │ ───────► │ Motor Driver  │
    │  App     │ ◄─────── │          │          │ Board (0x34)  │
    │ Joystick │ Telemetry│ main.py  │   PWM    │ ┌───────────┐ │
    └──────────┘          │          │ ───────► │ │ Encoders  │ │
                          └──────────┘  GP15    │ │ PID       │ │
                                      (servo)   │ │ YX-4055AM │ │
                                                └─┴───────────┘─┘

The motor driver board has its OWN onboard MCU that handles:
  - Encoder reading (all 4 channels)
  - PID speed control (closed-loop)
  - Motor driving (YX-4055AM chips)

So the Pico's job is simple:
  1. Connect WiFi
  2. Receive joystick commands via UDP
  3. Compute Ackermann differential
  4. Send I2C speed commands to the motor board
  5. Set servo angle for steering
  6. Send telemetry back to the app

Upload ALL .py files to the Pico's root filesystem:
    main.py, config.py, wifi_manager.py, motor.py,
    servo.py, ackermann.py
"""

import time
import json
import socket
import config
import wifi_manager
from motor import MotorDriver
from servo import Servo
from ackermann import AckermannController


def main():
    print("=" * 40)
    print(" Ackermann Chassis Controller v2.0")
    print(" (I2C Motor Driver Edition)")
    print("=" * 40)
    
    # ---- Step 1: Connect to WiFi ----
    ip = wifi_manager.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
    if ip is None:
        print("[FATAL] WiFi connection failed. Halting.")
        while True:
            time.sleep(1)
    
    wifi_manager.blink_ip(ip)
    
    # ---- Step 2: Initialize hardware ----
    print("[Init] Setting up motor driver (I2C)...")
    motor_driver = MotorDriver()
    
    print("[Init] Setting up steering servo (GP15)...")
    steering_servo = Servo()
    
    print("[Init] Setting up Ackermann controller...")
    ackermann = AckermannController()
    
    # ---- Step 3: Start UDP server ----
    print(f"[UDP] Starting server on {ip}:{config.UDP_PORT}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, config.UDP_PORT))
    sock.setblocking(False)
    
    print("[Ready] Waiting for commands...")
    print(f"[Ready] Send UDP to {ip}:{config.UDP_PORT}")
    
    # ---- State variables ----
    cmd_x = 0.0
    cmd_y = 0.0
    emergency = False
    last_cmd_time = time.ticks_ms()
    last_telemetry_time = time.ticks_ms()
    client_addr = None
    last_enc_left = 0
    last_enc_right = 0
    last_enc_time = time.ticks_ms()
    left_speed_est = 0
    right_speed_est = 0
    prev_left_cmd = 0
    prev_right_cmd = 0
    
    # ---- Main control loop ----
    while True:
        now_ms = time.ticks_ms()
        
        # ---- Receive UDP commands (non-blocking) ----
        # Drain all pending packets, keep only the latest
        got_new = False
        while True:
            try:
                data, addr = sock.recvfrom(256)
                if data:
                    try:
                        msg = json.loads(data.decode())
                        cmd_x = float(msg.get('x', 0))
                        cmd_y = float(msg.get('y', 0))
                        emergency = bool(msg.get('e', 0))
                        last_cmd_time = now_ms
                        client_addr = addr
                        got_new = True
                    except (ValueError, KeyError):
                        pass
            except OSError:
                break  # No more pending data
        
        # ---- Safety watchdog ----
        time_since_cmd = time.ticks_diff(now_ms, last_cmd_time)
        if time_since_cmd > config.COMMAND_TIMEOUT_MS:
            cmd_x = 0.0
            cmd_y = 0.0
        
        # ---- Emergency stop ----
        if emergency:
            cmd_x = 0.0
            cmd_y = 0.0
        
        # ---- Compute Ackermann outputs ----
        left_speed, right_speed, servo_angle = ackermann.compute(cmd_x, cmd_y)
        
        # ---- Set steering servo (instant — direct PWM, no I2C) ----
        steering_servo.set_angle(servo_angle)
        
        # ---- Send speed to motor driver board via I2C ----
        # Only send if values changed (reduces I2C bus overhead)
        int_left = int(left_speed)
        int_right = int(right_speed)
        if int_left != prev_left_cmd or int_right != prev_right_cmd:
            motor_driver.set_speed(int_left, int_right)
            prev_left_cmd = int_left
            prev_right_cmd = int_right
        
        # ---- Read encoders & estimate speed (for telemetry) ----
        enc_elapsed = time.ticks_diff(now_ms, last_enc_time)
        if enc_elapsed >= 100:  # Read encoders at 10Hz
            enc_left, enc_right = motor_driver.read_encoders()
            dt_sec = enc_elapsed / 1000.0
            if dt_sec > 0:
                left_speed_est = (enc_left - last_enc_left) / dt_sec
                right_speed_est = (enc_right - last_enc_right) / dt_sec
            last_enc_left = enc_left
            last_enc_right = enc_right
            last_enc_time = now_ms
        
        # ---- Send telemetry ----
        telemetry_elapsed = time.ticks_diff(now_ms, last_telemetry_time)
        if telemetry_elapsed >= config.TELEMETRY_INTERVAL_MS and client_addr is not None:
            last_telemetry_time = now_ms
            
            telemetry = {
                'lr': round(left_speed_est, 1),     # Left encoder speed
                'rr': round(right_speed_est, 1),     # Right encoder speed
                'sa': round(steering_servo.get_angle(), 1),
                'el': round(left_speed, 1),          # Commanded left speed
                'er': round(right_speed, 1),         # Commanded right speed
                'bat': 0,
            }
            
            try:
                sock.sendto(json.dumps(telemetry).encode(), client_addr)
            except OSError:
                pass
        
        # Minimal sleep — just enough to yield for interrupts
        time.sleep_ms(2)


# ---- Auto-start on boot ----
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n[Shutdown] Stopping motors...")
        try:
            driver = MotorDriver()
            driver.stop_all()
        except:
            pass
        try:
            servo = Servo()
            servo.center()
            servo.deinit()
        except:
            pass
        print("[Shutdown] Done.")
    except Exception as e:
        print(f"[FATAL] {e}")
        import machine
        machine.reset()
