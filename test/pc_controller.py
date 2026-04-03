"""
AutoNexa - PC-side Motor & Servo Test Controller
Run this on your PC while the Pico is connected via USB cable.

Usage:
    python pc_controller.py [--port COM3]   (Windows example)
    python pc_controller.py --port /dev/ttyACM0  (Linux)

If --port is omitted the script auto-detects the Pico's USB serial port.

Requirements:
    pip install pyserial
"""

import argparse
import sys
import time
import textwrap
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("ERROR: pyserial not installed. Run:  pip install pyserial")

# ── ANSI colour helpers ────────────────────────────────────────────────────────
def _c(code, txt):
    return "\033[{}m{}\033[0m".format(code, txt)

CYAN    = lambda t: _c("96", t)
GREEN   = lambda t: _c("92", t)
YELLOW  = lambda t: _c("93", t)
RED     = lambda t: _c("91", t)
BOLD    = lambda t: _c("1",  t)
MAGENTA = lambda t: _c("95", t)

# ── Port detection ─────────────────────────────────────────────────────────────
PICO_VID = 0x2E8A   # Raspberry Pi

def find_pico_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == PICO_VID:
            return p.device
    return None

# ── Serial reader thread ───────────────────────────────────────────────────────
_stop_reader = threading.Event()

def reader_thread(ser):
    """Reads lines from Pico and pretty-prints them to the console."""
    while not _stop_reader.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith("[INFO]"):
                print(CYAN(line))
            elif line.startswith("[SERVO]"):
                print(GREEN(line))
            elif line.startswith("[ENC]"):
                print(YELLOW(line))
            elif line.startswith("[DONE]"):
                print(BOLD(GREEN(line)))
            elif line.startswith("[ERR]"):
                print(RED(line))
            else:
                print(line)
        except serial.SerialException:
            break
        except Exception:
            pass

# ── Send helper ────────────────────────────────────────────────────────────────
def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()

# ── Menu ──────────────────────────────────────────────────────────────────────
MENU = """
{header}
  {a}  - Set servo to specific angle
  {b}  - Sweep servo 0→180→0
  {c}  - Straight-line 1 m encoder test
  {s}  - STOP motors (emergency)
  {t}  - Print encoder status
  {p}  - Ping Pico
  {q}  - Quit
{div}
""".format(
    header=BOLD("─── AutoNexa Hardware Test Menu ───────────────────────"),
    a=GREEN("[1]"),
    b=GREEN("[2]"),
    c=GREEN("[3]"),
    s=RED("[S]"),
    t=CYAN("[T]"),
    p=CYAN("[P]"),
    q=YELLOW("[Q]"),
    div=BOLD("───────────────────────────────────────────────────────")
)

# ── Servo angle table ──────────────────────────────────────────────────────────
SERVO_TABLE = textwrap.dedent("""
    ┌─────────────────────────────────────────────────────────┐
    │  Standard servo pulse widths (50 Hz, 20 ms period)     │
    ├────────┬──────────┬──────────────────────────────────── │
    │  Deg   │ Pulse µs │ Notes                               │
    ├────────┼──────────┼──────────────────────────────────── │
    │    0°  │  1000 µs │ Full left / minimum                 │
    │   45°  │  1250 µs │                                     │
    │   90°  │  1500 µs │ Centre (should point straight)      │
    │  135°  │  1750 µs │                                     │
    │  180°  │  2000 µs │ Full right / maximum                │
    └────────┴──────────┴──────────────────────────────────── ┘
    HOW TO VERIFY:
      1. Command a known angle (e.g. 90°).
      2. Let servo settle (~1 s).
      3. Place a protractor against the servo horn.
      4. Compare measured angle vs commanded angle.
      5. If offset: adjust SERVO_MIN_US / SERVO_MAX_US in pico_motor_servo_test.py
""")

# ── Straight-line test notes ───────────────────────────────────────────────────
STRAIGHT_NOTES = textwrap.dedent("""
    ┌────────────────────────────────────────────────────────────────┐
    │  Straight-line 1 m Encoder Test — Setup Checklist             │
    ├────────────────────────────────────────────────────────────────┤
    │  1. Place robot on a flat, clear surface.                      │
    │  2. Mark the starting position of the front of the robot.      │
    │  3. Lay a measuring tape / ruler along the intended path.      │
    │  4. Enter speed (recommend 30-50 % for indoor testing).        │
    │  5. Robot will drive forward and stop automatically.           │
    │  6. Mark the final position of the front of the robot.         │
    │  7. Measure the actual distance with the ruler.                │
    │                                                                │
    │  KEY VALUES TO NOTE:                                           │
    │    • Odometry distance  → printed in [DONE] line (dist_m)      │
    │    • Measured distance  → your ruler reading                   │
    │    • Encoder deviation  → left vs right counts (straight check)│
    │                                                                │
    │  IMPORTANT: If odometry ≠ ruler reading, update these in       │
    │  pico_motor_servo_test.py:                                      │
    │    PULSES_PER_REV   = <your encoder spec>                      │
    │    GEAR_RATIO       = <motor gearbox ratio>                    │
    │    WHEEL_DIAMETER_M = <actual wheel diameter in metres>        │
    └────────────────────────────────────────────────────────────────┘
""")

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="AutoNexa Motor/Servo PC Controller")
    parser.add_argument("--port", help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    args = parser.parse_args()

    port = args.port
    if not port:
        port = find_pico_port()
        if not port:
            sys.exit(RED("ERROR: Pico not found. Plug in USB or use --port COM<N>"))
        print(GREEN("Auto-detected Pico on: {}".format(port)))

    print(CYAN("Connecting to {} at {} baud...".format(port, args.baud)))
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(RED("Cannot open {}: {}".format(port, e)))

    time.sleep(1.5)  # wait for Pico to boot / reset after opening port
    ser.reset_input_buffer()

    # Start background reader
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    # Ping to confirm connection
    send(ser, "PING")
    time.sleep(0.5)

    print(MENU)

    try:
        while True:
            try:
                choice = input(BOLD("Command> ")).strip().upper()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == "1":
                print(SERVO_TABLE)
                try:
                    deg = float(input("  Enter angle (0-180): "))
                    send(ser, "SERVO:{:.1f}".format(deg))
                    time.sleep(1.5)
                except ValueError:
                    print(RED("  Invalid number."))

            elif choice == "2":
                print(CYAN("  Sending SERVO_SWEEP command..."))
                send(ser, "SERVO_SWEEP")
                time.sleep(15)  # sweep takes ~14 s (37 steps × 400 ms)

            elif choice == "3":
                print(STRAIGHT_NOTES)
                try:
                    spd = float(input("  Enter motor speed (5-100 %): "))
                    if not 5 <= spd <= 100:
                        raise ValueError
                    print(YELLOW("  Starting test. Place robot and press Enter when ready..."))
                    input()
                    send(ser, "STRAIGHT:{:.1f}".format(spd))
                    print(CYAN("  Driving... (live [ENC] lines will appear)"))
                    # Wait until DONE comes back (max 60 s)
                    deadline = time.time() + 60
                    while time.time() < deadline:
                        time.sleep(0.5)
                        # reader thread handles printing; we just wait
                        # User can press Ctrl+C to abort
                except ValueError:
                    print(RED("  Invalid speed (must be 5-100)."))
                except KeyboardInterrupt:
                    send(ser, "STOP")
                    print(YELLOW("  Aborted — STOP sent."))

            elif choice in ("S", "STOP"):
                send(ser, "STOP")

            elif choice == "T":
                send(ser, "STATUS")
                time.sleep(0.3)

            elif choice == "P":
                send(ser, "PING")
                time.sleep(0.3)

            elif choice == "Q":
                break

            else:
                print(RED("  Unknown choice. Type 1, 2, 3, S, T, P or Q."))

    finally:
        _stop_reader.set()
        send(ser, "STOP")
        ser.close()
        print(YELLOW("\nConnection closed. Bye!"))


if __name__ == "__main__":
    main()
