"""
AutoNexa - Motor & Servo Hardware Test
Runs on Raspberry Pi Pico W (MicroPython)
Receives commands from PC via USB UART (stdin/stdout)
Commands are newline-terminated strings.

COMMAND SET (PC -> Pico):
  SERVO:<deg>       Set servo to <deg> degrees (0-180)
  SERVO_SWEEP       Sweep servo 0->180->0 with 10° steps, reporting each angle
  STRAIGHT:<speed>  Drive straight for exactly 1 m then stop (speed = 0-100 %)
  STOP              Emergency stop all motors
  STATUS            Print encoder counts and motor state
  PING              Respond PONG (connection check)

RESPONSE FORMAT (Pico -> PC):
  Lines are prefixed with tags so the PC script can parse them:
  [INFO]  <message>
  [SERVO] deg_input=<d>  pulse_us=<p>  (use a protractor to verify angle)
  [ENC]   left=<L>  right=<R>  target=<T>  dist_m=<dist>
  [DONE]  <message>
  [ERR]   <message>
"""

import sys
import time
import select
from machine import Pin, PWM, Timer

# ── Pin definitions ────────────────────────────────────────────────────────────
# On Pico W the LED is wired through the CYW43 Wi-Fi chip — NOT GP25.
# Pin("LED") is the correct cross-compatible way to drive it.
LED = Pin("LED", Pin.OUT)

# Left motor
IN1 = Pin(2, Pin.OUT)
IN2 = Pin(3, Pin.OUT)
ENA = PWM(Pin(4))

# Right motor
IN3 = Pin(6, Pin.OUT)
IN4 = Pin(7, Pin.OUT)
ENB = PWM(Pin(8))

# Encoders
ENC_L_A = Pin(10, Pin.IN, Pin.PULL_UP)
ENC_L_B = Pin(11, Pin.IN, Pin.PULL_UP)
ENC_R_A = Pin(12, Pin.IN, Pin.PULL_UP)
ENC_R_B = Pin(13, Pin.IN, Pin.PULL_UP)

# Servo
SERVO_PIN = PWM(Pin(15))

# ── PWM setup ─────────────────────────────────────────────────────────────────
ENA.freq(1000)
ENB.freq(1000)
SERVO_PIN.freq(50)        # standard servo: 50 Hz

# ── Encoder state ─────────────────────────────────────────────────────────────
enc_left  = 0
enc_right = 0

def _isr_enc_left(pin):
    global enc_left
    if ENC_L_B.value() == 0:
        enc_left += 1
    else:
        enc_left -= 1

def _isr_enc_right(pin):
    global enc_right
    if ENC_R_B.value() == 0:
        enc_right += 1
    else:
        enc_right -= 1

ENC_L_A.irq(trigger=Pin.IRQ_RISING, handler=_isr_enc_left)
ENC_R_A.irq(trigger=Pin.IRQ_RISING, handler=_isr_enc_right)

# ── Encoder calibration ───────────────────────────────────────────────────────
# Change these to match YOUR encoder + wheel specs.
# Typical: 20 pulses/rev (hall sensor) or 360+ pulses/rev (optical)
PULSES_PER_REV  = 20          # encoder pulses per motor shaft revolution
GEAR_RATIO      = 1.0         # set if there is a gearbox (output_rev = shaft_rev / gear_ratio)
WHEEL_DIAMETER_M = 0.065      # metres — measure your actual wheel

import math
PULSES_PER_METER = (PULSES_PER_REV * GEAR_RATIO) / (math.pi * WHEEL_DIAMETER_M)

def counts_to_metres(counts):
    return counts / PULSES_PER_METER

# ── Motor helpers ─────────────────────────────────────────────────────────────
def _duty(percent):
    """Convert 0-100 % to 16-bit duty cycle value."""
    return int(min(max(percent, 0), 100) / 100 * 65535)

def motors_forward(speed_pct):
    IN1.value(1); IN2.value(0)
    IN3.value(1); IN4.value(0)
    ENA.duty_u16(_duty(speed_pct))
    ENB.duty_u16(_duty(speed_pct))

def motors_stop():
    IN1.value(0); IN2.value(0)
    IN3.value(0); IN4.value(0)
    ENA.duty_u16(0)
    ENB.duty_u16(0)

# ── Servo helpers ─────────────────────────────────────────────────────────────
# Standard servo: 1000 µs = 0°,  1500 µs = 90°,  2000 µs = 180°
# If your servo differs, tune SERVO_MIN_US / SERVO_MAX_US.
SERVO_MIN_US = 1000    # pulse width for 0°
SERVO_MAX_US = 2000    # pulse width for 180°

def degrees_to_pulse_us(deg):
    deg = max(0, min(180, deg))
    return int(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0)

def servo_set(deg):
    pulse_us = degrees_to_pulse_us(deg)
    # PWM period at 50 Hz = 20 000 µs → duty fraction = pulse_us / 20000
    duty = int(pulse_us / 20000 * 65535)
    SERVO_PIN.duty_u16(duty)
    return pulse_us

# ── Heartbeat LED ─────────────────────────────────────────────────────────────
_blink_state = False
def heartbeat(t):
    global _blink_state
    _blink_state = not _blink_state
    LED.value(_blink_state)

hb_timer = Timer()
hb_timer.init(freq=2, mode=Timer.PERIODIC, callback=heartbeat)

# ── Output helpers ─────────────────────────────────────────────────────────────
def log(tag, msg):
    print("[{}] {}".format(tag, msg))
    # flush is automatic on USB CDC in MicroPython

# ── Command handlers ──────────────────────────────────────────────────────────

def cmd_servo(arg):
    """Set servo to a specific angle and report."""
    try:
        deg = float(arg)
    except ValueError:
        log("ERR", "SERVO requires a numeric degree argument, got: {}".format(arg))
        return
    pulse = servo_set(deg)
    time.sleep_ms(600)   # give servo time to move
    log("SERVO", "deg_input={:.1f}  pulse_us={}  "
        "(measure real angle with protractor)".format(deg, pulse))


def cmd_servo_sweep():
    """Sweep 0 -> 180 -> 0 in 10° steps."""
    log("INFO", "Starting servo sweep 0->180->0 in 10 deg steps")
    for deg in list(range(0, 181, 10)) + list(range(180, -1, -10)):
        pulse = servo_set(deg)
        time.sleep_ms(400)
        log("SERVO", "deg_input={:3d}  pulse_us={}".format(deg, pulse))
    log("DONE", "Servo sweep complete")


def cmd_straight(arg):
    """Drive straight for 1 m using encoder odometry then stop."""
    global enc_left, enc_right

    try:
        speed = float(arg)
        if not (5 <= speed <= 100):
            raise ValueError
    except ValueError:
        log("ERR", "STRAIGHT requires speed 5-100, got: {}".format(arg))
        return

    TARGET_M = 1.0
    target_pulses = int(TARGET_M * PULSES_PER_METER)

    log("INFO",
        "Straight-line test: target=1.000 m  "
        "target_pulses={}  speed={:.0f}%".format(target_pulses, speed))
    log("INFO",
        "ENCODER CONFIG: {:.0f} pulses/rev  gear={:.2f}  "
        "wheel_diam={:.3f} m  => {:.1f} pulses/m".format(
            PULSES_PER_REV, GEAR_RATIO, WHEEL_DIAMETER_M, PULSES_PER_METER))

    # Reset encoders
    enc_left  = 0
    enc_right = 0

    motors_forward(speed)

    last_print = time.ticks_ms()

    while True:
        left  = abs(enc_left)
        right = abs(enc_right)
        avg   = (left + right) / 2.0
        dist  = counts_to_metres(avg)

        # Print live update every 200 ms
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) >= 200:
            log("ENC",
                "left={:6d}  right={:6d}  avg={:6.1f}  "
                "target={}  dist_m={:.4f}".format(
                    left, right, avg, target_pulses, dist))
            last_print = now

        if avg >= target_pulses:
            break

        time.sleep_ms(10)

    motors_stop()

    left  = abs(enc_left)
    right = abs(enc_right)
    avg   = (left + right) / 2.0
    dist  = counts_to_metres(avg)

    log("ENC",
        "FINAL  left={:6d}  right={:6d}  avg={:6.1f}  "
        "target={}  dist_m={:.4f}".format(left, right, avg, target_pulses, dist))
    log("DONE",
        "Motors stopped. Odometry says {:.4f} m. "
        "Measure with ruler and compare!".format(dist))
    log("INFO",
        "Straight-line deviation: left-right encoder diff = {} pulses".format(
            abs(abs(enc_left) - abs(enc_right))))


def cmd_stop():
    motors_stop()
    log("DONE", "All motors stopped")


def cmd_status():
    log("INFO",
        "enc_left={}  enc_right={}  "
        "dist_left={:.4f} m  dist_right={:.4f} m".format(
            enc_left, enc_right,
            counts_to_metres(abs(enc_left)),
            counts_to_metres(abs(enc_right))))


# ── Main loop ─────────────────────────────────────────────────────────────────
log("INFO", "AutoNexa Motor+Servo Test Ready. Waiting for commands...")
log("INFO", "Commands: SERVO:<deg>  SERVO_SWEEP  STRAIGHT:<speed>  STOP  STATUS  PING")

while True:
    # Non-blocking check for incoming USB data
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    events = poll.poll(100)   # 100 ms timeout

    if not events:
        continue

    line = sys.stdin.readline().strip()
    if not line:
        continue

    line = line.upper()

    if line == "PING":
        log("INFO", "PONG")

    elif line.startswith("SERVO_SWEEP"):
        cmd_servo_sweep()

    elif line.startswith("SERVO:"):
        cmd_servo(line[6:])

    elif line.startswith("STRAIGHT:"):
        cmd_straight(line[9:])

    elif line == "STOP":
        cmd_stop()

    elif line == "STATUS":
        cmd_status()

    else:
        log("ERR", "Unknown command: {}".format(line))
