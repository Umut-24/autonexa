# AutoNexa Subsystem Test Plan

This document breaks down practical tests for three subsystems:

1. Battery subsystem
2. Object detection subsystem (LiDAR-based)
3. Control subsystem (Pico + L298N + servo + dual DC motors)

The companion data-entry dashboard is `docs/subsystem_test_data_dashboard.html`.

---

## 1) Battery subsystem test plan

### A. Goals

- Verify battery can safely deliver required voltage/current during idle, cruise, steering events, and startup surge.
- Estimate realistic runtime under representative mission profiles.
- Detect voltage sag and thermal risks before failures occur.

### B. Instrumentation

- Inline current/voltage monitor (e.g., INA219/INA226 module, DC power analyzer, or calibrated multimeter + shunt).
- Temperature probe for battery pack and motor driver heat sink.
- Timestamped logging source (ROS2 node, laptop CSV logger, or manual sheet).

### C. Core test cases

#### Test B1 — Open-circuit and baseline health check

- Measure open-circuit voltage after full charge and after 15-minute rest.
- Measure no-load current in standby state (electronics on, motors disabled).
- Pass criteria:
  - Open-circuit voltage inside expected chemistry range.
  - Standby current within budgeted idle draw.

#### Test B2 — Dynamic load profile (voltage sag and current peaks)

Run the robot through repeated modes:

1. Idle (60 s)
2. Slow straight drive (60 s)
3. Aggressive steer left/right cycles (60 s)
4. Start-stop bursts (10 cycles)
5. Combined turn + acceleration (60 s)

At each second, log:

- Time
- Battery voltage
- Current
- Estimated power (V×I)
- Optional battery and driver temperature

Pass criteria:

- Voltage never drops below subsystem minimum operating threshold.
- Current peaks remain within battery and driver safe limits.
- No brownout resets on Pico or RPi.

#### Test B3 — Runtime endurance

- Run a repeatable mission loop (e.g., patrol/path follow) until low-voltage cutoff.
- Record total runtime and energy consumed.
- Repeat for at least three trials and compute average + standard deviation.

Pass criteria:

- Runtime >= required mission duration with safety margin (recommend 20%).

### D. Useful metrics

- Min/avg/max voltage
- Min/avg/max current
- Peak power
- Runtime to cutoff
- Voltage sag ratio: `(V_oc - V_load_min) / V_oc`

---

## 2) Object detection subsystem test plan (LiDAR)

### A. Goals

- Validate detection reliability against known object positions and sizes.
- Quantify distance error compared to ground-truth measurements.
- Check robustness for angle, surface reflectivity, and range.

### B. Test setup

- Use objects with known dimensions (box, cylinder, wall panel).
- Mark precise ground-truth distances (e.g., tape marks every 0.25 m).
- Keep robot fixed for static tests; use a controlled path for dynamic tests.

### C. Core test cases

#### Test O1 — Static distance accuracy

For each target distance (e.g., 0.5, 1.0, 1.5, 2.0, 2.5 m):

- Place object at measured distance.
- Capture N frames/scans.
- Compute mean detected distance.
- Calculate error: `detected - ground_truth`.

Pass criteria:

- Mean absolute error below chosen threshold (example: <= 5 cm indoors).
- Error trend should not diverge sharply across tested range.

#### Test O2 — Angular coverage test

- Keep distance fixed (e.g., 1.5 m).
- Move object across left-center-right sectors in the LiDAR FOV.
- Measure detection consistency and distance error by angle bin.

Pass criteria:

- Detection remains stable in all required angle sectors.

#### Test O3 — Surface/shape robustness

- Repeat static test with different materials:
  - Matte cardboard
  - Glossy plastic
  - Dark fabric/object
- Compare missed detections and noisy readings.

Pass criteria:

- Missed-detection rate remains below accepted threshold for mission environment.

#### Test O4 — Dynamic obstacle test

- Move object at controlled speeds across robot path.
- Measure detection latency and continuity.
- Compare to safety stopping distance constraints.

Pass criteria:

- End-to-end detection latency supports safe stop margin.

### D. Useful metrics

- Mean absolute distance error (m)
- RMSE (m)
- Detection success rate (%)
- False positives per minute
- Detection latency (ms)

---

## 3) Control subsystem test plan (Pico + L298N + servo + 2x DC motor)

### A. Goals

- Validate command chain from high-level velocity command to actuator response.
- Verify steering response, motor symmetry, and command safety handling.
- Ensure reliable operation under transient and fault scenarios.

### B. Layered approach (recommended)

#### Layer C1 — Bench test each actuator independently

1. **Servo test (steering):**
   - Send known angle commands (e.g., -25°, -15°, 0°, +15°, +25°).
   - Measure actual wheel angle with protractor or camera-based estimate.
   - Calculate command vs actual error and hysteresis.

2. **Motor test (left/right):**
   - Sweep PWM values in steps.
   - Measure wheel RPM (encoder or tachometer) for each motor.
   - Compare left-right mismatch.

Pass criteria:

- Servo angle error within steering tolerance.
- Left/right RPM mismatch within tolerance (example: <= 10% at same command).

#### Layer C2 — Embedded control chain (Pico + driver)

- Send setpoint profiles (step, ramp, sine-like) to Pico interfaces.
- Record:
  - Command timestamp
  - Applied PWM
  - Encoder-derived speed
  - Safety enable/disable state
- Verify timeout behavior by intentionally stopping commands.

Pass criteria:

- Timeout forces safe stop within configured deadline.
- No unstable oscillation in speed control.

#### Layer C3 — Full system integration (ROS2 -> Pico -> mechanics)

- Publish `/cmd_vel` patterns and compare achieved motion:
  - Straight line tracking
  - Constant-radius turns
  - Start/stop repeatability
- Validate command-to-motion latency and drift.

Pass criteria:

- Robot follows expected trajectories within acceptable lateral/heading error.
- Emergency disable path always overrides motion commands.

### C. Fault injection tests (high value)

- Drop command stream for > timeout.
- Simulate low battery (lower supply voltage in controlled bench setting).
- Introduce motor load asymmetry (light braking on one wheel, safely).

Expected:

- Controlled degradation, safe stop, and no runaway behavior.

### D. Useful metrics

- Command-to-actuation latency (ms)
- Servo steady-state error (deg)
- Overshoot/settling time for speed response
- Left-right speed mismatch (%)
- Timeout-to-stop delay (ms)

---

## Data logging guidance (all subsystems)

- Use a consistent test ID format (e.g., `BAT-B2-T01`, `OBJ-O1-T03`, `CTRL-C3-T02`).
- Record firmware/software version, battery state-of-charge, floor type, and payload mass.
- Capture at least three repeated trials per condition.
- Keep "Notes" field mandatory for anomalies.

