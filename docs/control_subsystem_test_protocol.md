# Control Subsystem Test Protocol (Servo + Pico Motor Input)

This protocol is focused on your two requested checks:

1. Servo test with known angle commands vs real measured wheel angle.
2. Decide Pico motor input type from RPi5 Nav2 output, then validate it with real measurements.

## 1) Servo test protocol

### Goal

Quantify steering command tracking quality and repeatability.

### Steps

1. Put robot on test stand or low-speed flat floor.
2. Send fixed commands: `-25, -15, 0, +15, +25` degrees (repeat each at least 3 times).
3. For each command:
   - Record commanded angle.
   - Measure actual wheel angle (digital angle gauge, protractor, or camera method).
   - Record settling time (time from command issue to stable angle).
4. Repeat sequence in both directions (CCW then CW) to reveal hysteresis.

### Metrics

- Error per point: `measured - commanded` (deg)
- Mean absolute error (deg)
- Max absolute error (deg)
- Average settling time (ms)
- Hysteresis gap at same setpoint for opposite directions (deg)

### Suggested pass criteria (adjust to your platform)

- Mean absolute steering error <= 2.0 deg
- Max absolute error <= 4.0 deg
- Settling time <= 250 ms

## 2) Pico motor input-type decision protocol

### Candidate input types

A. `TwistStamped(vx, wz)`
- Best alignment with Nav2 output from RPi5.
- Requires conversion on Pico side to wheel targets.

B. `Left/Right wheel speed setpoints` (RPM or rad/s)
- Most direct for motor-speed loops.
- Requires conversion on RPi5 side from `(vx, wz)`.

C. `Normalized PWM` (e.g., `[-1, +1]`)
- Simplest low-level interface.
- Weakest physical meaning and less robust closed-loop behavior.

### Decision approach

Score each candidate from 1 to 5 on:

- Ease of integration with Nav2
- Control precision / tunability
- Safety handling (timeouts, limits, predictable fallback)

Use average score as the "final score," then keep the top option for field tests.

### Recommended default

Start with `TwistStamped(vx, wz)` for system integration simplicity, then optionally compare against wheel-speed commands for tighter low-level control.

## 3) Real-life validation after input type selection

### Test sequence

1. Send step commands from chosen input format at multiple operating points.
2. Measure and log:
   - Left and right RPM
   - Measured linear speed (m/s)
   - Command-to-response latency (ms)
   - Stop-on-timeout delay (ms)
3. Repeat for:
   - Straight motion
   - Right turn
   - Left turn
   - Stop / resume

### Key outputs

- RPM symmetry (% mismatch)
- Speed tracking quality
- Latency stability
- Timeout safety response

## Dashboard file

Use this dashboard for direct data entry and graph generation:

- `docs/control_subsystem_test_dashboard.html`

