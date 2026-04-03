# AutoNexa — Motor & Servo Hardware Test (Pico W)

This folder contains everything needed to test the **two DC motors (with encoders)** and the **servo** on the **Raspberry Pi Pico W**.

---

## File Overview

| File | Where it runs | Purpose |
|---|---|---|
| `pico_motor_servo_test.py` | **Pico W** (MicroPython) | Firmware — listens for commands, controls motors & servo |
| `pc_controller.py` | **PC** (Python 3) | Interactive menu to send test commands via USB serial |

---

## Step 1 — Install Prerequisites on Your PC

```powershell
pip install pyserial mpremote
```

---

## Step 1b — Flash MicroPython on the Pico W (first time only)

> **Important:** The Pico W needs its own firmware — the plain Pico `.uf2` will NOT work.

1. Download the latest **Pico W** MicroPython `.uf2` from:  
   👉 https://micropython.org/download/RPI_PICO_W/
2. Hold the **BOOTSEL** button on the Pico W and plug in the USB cable.
3. A drive called `RPI-RP2` appears on your PC.
4. Drag and drop the downloaded `.uf2` file onto that drive.
5. The Pico W reboots automatically into MicroPython.

---

## Step 2 — Upload the Pico W File

1. Connect the Pico W to your PC with the USB cable.
2. Check which COM port it shows up on (Device Manager → Ports → look for **USB Serial Device**).
3. Open a PowerShell / terminal window in this `test` folder:

```powershell
cd "C:\Users\umut2\Desktop\autonexa\autonexa\test"
```

4. Upload the firmware file:

```powershell
mpremote connect COM3 cp pico_motor_servo_test.py :main.py
```

> **Replace `COM3`** with your actual port (e.g. `COM4`, `COM5`).
> Copying it as `main.py` means the Pico runs it automatically on every boot.

5. Reset / power-cycle the Pico W. You should see the onboard LED blinking at ~2 Hz — this is the heartbeat.

> **Pico W LED note:** The LED on Pico W is driven through the CYW43 Wi-Fi chip.
> The firmware uses `Pin("LED")` (not `Pin(25)`) which is correct for Pico W.

---

## Step 3 — Run the PC Controller

```powershell
python pc_controller.py --port COM3
```

Or let the script auto-detect the Pico:

```powershell
python pc_controller.py
```

You'll see a menu like this:

```
─── AutoNexa Hardware Test Menu ───────────────────────
  [1]  - Set servo to specific angle
  [2]  - Sweep servo 0→180→0
  [3]  - Straight-line 1 m encoder test
  [S]  - STOP motors (emergency)
  [T]  - Print encoder status
  [P]  - Ping Pico
  [Q]  - Quit
───────────────────────────────────────────────────────
```

---

## Test A — Servo Angle Verification

### What it tests
That the pulse-width math produces the correct real-world servo angle.

### Procedure

1. Press **`1`** and enter an angle (e.g. `90`).
2. Let the servo settle (~0.5 s).
3. Hold a **protractor** against the servo horn base.
4. Read the **real angle**.
5. Compare with the commanded angle shown in the `[SERVO]` line:

```
[SERVO] deg_input=90.0  pulse_us=1500  (measure real angle with protractor)
```

| Commanded | Expected pulse | If real angle is off… |
|-----------|---------------|----------------------|
| 0°        | 1000 µs       | Adjust `SERVO_MIN_US` in `pico_motor_servo_test.py` |
| 90°       | 1500 µs       | Midpoint check — centre horn |
| 180°      | 2000 µs       | Adjust `SERVO_MAX_US` in `pico_motor_servo_test.py` |

> Use **`[2] Sweep`** to see all angles in one pass.

---

## Test B — Straight-Line 1 m Encoder Test

### What it tests
1. That the encoder odometry stops the robot at exactly 1 m.
2. That the robot travels in a **straight line** (left ≈ right encoder counts).

### Setup
```
[Wall / tape mark]──────────────────────────────────[Stop mark]
         ↑                                                ↑
   Start of robot                                  Should be 1.000 m
```

### Procedure

1. Lay a measuring tape on the floor.
2. Mark the **front** of the robot with tape at the 0 m mark.
3. Press **`3`**, enter a speed (e.g. `40`).
4. When prompted, press **Enter** to start.
5. Watch the live `[ENC]` output:

```
[ENC] left=  42  right=  41  avg=  41.5  target=98  dist_m=0.4211
```

6. Robot stops. Note the `[DONE]` line:

```
[DONE] Motors stopped. Odometry says 1.0000 m. Measure with ruler and compare!
[INFO] Straight-line deviation: left-right encoder diff = 3 pulses
```

7. Measure the **actual distance** with the ruler.

### Interpreting Results

| Observation | Likely cause | Fix |
|---|---|---|
| Odometry says 1 m, ruler says 1 m ✅ | Perfect calibration | — |
| Odometry > ruler | `PULSES_PER_METER` too low | Increase `PULSES_PER_REV` or decrease `WHEEL_DIAMETER_M` |
| Odometry < ruler | `PULSES_PER_METER` too high | Decrease `PULSES_PER_REV` or increase `WHEEL_DIAMETER_M` |
| Robot curves left | Right motor faster | Reduce right motor speed slightly (in firmware) |
| Robot curves right | Left motor faster | Reduce left motor speed slightly (in firmware) |
| Large left-right encoder diff | One encoder not working | Check wiring GP10-GP13, pull-ups |

### Calibration Parameters (in `pico_motor_servo_test.py`)

```python
PULSES_PER_REV   = 20      # ← pulses your encoder produces per motor shaft rev
GEAR_RATIO       = 1.0     # ← set to actual gearbox ratio if you have one
WHEEL_DIAMETER_M = 0.065   # ← measure your wheel diameter in metres (e.g. 65 mm → 0.065)
```

**To find `PULSES_PER_REV`:** Spin the wheel exactly one revolution by hand while watching `[T] STATUS`. The count that appears is your `PULSES_PER_REV`.

---

## Troubleshooting

| Symptom | Check |
|---|---|
| `Pico not found` | USB cable plugged in? Try different port. Run Device Manager. |
| `Cannot open COMx` | Port in use by another program (e.g. Thonny). Close it first. |
| Motors don't move | L298N powered? 12 V battery connected? Check ENA/ENB wires. |
| Servo jitter / no movement | 5 V from L298N? GP15 connected to signal wire? |
| Encoders show 0 always | Check GP10-GP13 wiring. Pull-ups enabled in code. |
| One motor spins backwards | Swap OUT1↔OUT2 or OUT3↔OUT4 on L298N for that motor. |

---

## Quick Reference — Pin Table

| GPIO | Function | 
|------|---------|
| GP2  | L298N IN1 (left fwd) |
| GP3  | L298N IN2 (left rev) |
| GP4  | L298N ENA (left PWM) |
| GP6  | L298N IN3 (right fwd) |
| GP7  | L298N IN4 (right rev) |
| GP8  | L298N ENB (right PWM) |
| GP10 | Left encoder A |
| GP11 | Left encoder B |
| GP12 | Right encoder A |
| GP13 | Right encoder B |
| GP15 | Servo signal (PWM) |
| GP25 | **Not used** (Pico W LED is internal to CYW43 chip — `Pin("LED")` in firmware) |
