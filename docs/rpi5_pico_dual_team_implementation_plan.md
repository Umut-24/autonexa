# RPi5 + Pico Dual-Team Implementation Plan for Nav2 Ackermann Control

## 1) Current System Analysis (from repository)

### What is already working on the RPi5 side
- The project is already structured around ROS2 + Nav2 + SLAM/AMCL, with a parking-focused navigation workflow and RViz operation. The README explicitly states that current demos run without real motor actuation and are manually moved. 
- `parking_navigation.launch.py` starts LiDAR, scan matcher odometry, Nav2 servers, and optional parking helpers. This means high-level navigation is present and producing velocity commands.
- Nav2 stack currently uses `nav2_controller` + `velocity_smoother`, which is the right place to source motion commands for a hardware bridge.

### What is missing for real Ackermann drive
- There is no hardware interface node/plugin in the repo that converts Nav2 output into actual steering + rear motor commands for the Pico.
- Current odometry in launch flows comes from `laser_scan_matcher` rather than wheel+steering kinematics from a vehicle controller.
- The project currently assumes differential-style AMCL motion model and DWB defaults in `nav2_navigation_params.yaml`; this is acceptable for early bring-up but not ideal for final Ackermann parking performance.

### Consequence
- You can **and should** split work into two parallel teams:
  1. **Pico Control Team** (deterministic low-level control, safety, and kinematics)
  2. **RPi5 Nav2 Integration Team** (ROS2 interfaces, command shaping, odometry fusion, Nav2 tuning)

This split minimizes coupling and follows your bottom-up integration strategy.

---

## 2) Team Split and Ownership

## Team A — Pico Control / Embedded Team
**Mission:** deterministic execution of steering + traction control for Ackermann chassis.

**Owns:**
- Servo calibration and angle mapping
- Encoder drivers and interrupt reliability
- Motor PID loops (left/right)
- Ackermann inverse kinematics on MCU
- micro-ROS client or serial protocol endpoint
- Safety watchdogs (command timeout, e-stop behavior)

**Primary deliverable:** a stable firmware that accepts body commands and reliably actuates/senses hardware.

## Team B — RPi5 ROS2/Nav2 Team
**Mission:** transform Nav2 goals/path output into clean vehicle commands and consume feedback for localization/control.

**Owns:**
- LiDAR + Nav2 + localization orchestration
- RPi5↔Pico communication layer (agent/node/bridge)
- Command conditioning (rate limit, curvature/speed profiling)
- Odometry ingestion and EKF integration
- Nav2 controller/planner tuning for constrained parking

**Primary deliverable:** ROS2 pipeline from `NavigateToPose` goal to stable low-level command stream and robust localization.

---

## 3) Interface Contract Between Teams (work separately, integrate safely)

To enable parallel development, freeze an interface early.

### Command interface (RPi5 -> Pico)
Use one message contract independent of transport first:
- `stamp`
- `vx_mps` (longitudinal speed)
- `wz_radps` (yaw rate)
- `mode` (`MANUAL`, `AUTO`, `ESTOP`)
- `enable` (bool)

> On Pico, convert (`vx`, `wz`) to steering angle + wheel targets using Ackermann inverse kinematics.

### Feedback interface (Pico -> RPi5)
- `stamp`
- `steering_angle_rad` (measured or estimated)
- `wheel_rpm_left`, `wheel_rpm_right`
- `battery_voltage` (optional)
- `fault_code`
- `odom` (`x,y,yaw,vx,wz`) if computed on Pico

### Safety contract
- Command timeout (e.g., 100–200 ms): if no command, set PWM to neutral/brake.
- Explicit `ESTOP` command has highest priority.
- Heartbeat topic/channel from RPi5 to Pico.

---

## 4) Phased Roadmap (adapted for two-team execution)

## Phase 1 — Embedded Subsystem Isolation (Team A lead, Team B support)

### Team A tasks
1. **Servo calibration mapping**
   - Measure exact PWM for center, max-left, max-right.
   - Fit piecewise-linear map: `steering_rad -> pwm_us`.
2. **Encoder interrupt validation**
   - Validate 32-bit counters for both wheels under manual spin and drill/high-RPM test.
3. **PID velocity control**
   - Tune independent L/R PID with chassis load.
   - Acceptance: <10% overshoot, <5% steady-state error at multiple setpoints.

### Team B tasks in parallel
- Build ROS2 test publisher for synthetic command streams at target control rate (20–50 Hz).
- Define message schemas and log format used later by Team A bench tests.

**Exit criteria**
- Pico can hold wheel speed and steering setpoints reproducibly without ROS dependency.

## Phase 2 — Kinematics + Middleware Link (joint)

### Team A tasks
1. Implement Ackermann inverse kinematics on Pico:
   - Input (`vx`, `wz`) -> steering angle + `V_L`, `V_R`.
2. Integrate transport endpoint:
   - Option A: micro-ROS client on Pico over UART/USB.
   - Option B: lightweight binary serial protocol + ROS2 bridge on RPi5.
3. Publish actuator/encoder feedback.

### Team B tasks
1. Bring up micro-ROS agent (if Option A) or bridge node (if Option B).
2. Build integration tests:
   - Step commands, sine sweep, stop/start, timeout behavior.
3. Record rosbag for command vs feedback latency and jitter.

**Exit criteria**
- End-to-end RPi5 command reaches Pico with deterministic timing.
- Pico feedback visible in ROS2 topics and loggable.

## Phase 3 — ROS2 Control Abstraction + Sensor Fusion (Team B lead)

### Team B tasks
1. Introduce hardware abstraction layer:
   - `cmd_vel` (or smoothed velocity) -> bridge command topic.
   - Feedback -> odometry publisher.
2. Deploy `robot_localization` EKF (2D mode):
   - Fuse Pico odom + IMU (+ optionally scan-matcher/AMCL constraints).
3. Validate teleop closed-loop motion in 2x2m area.

### Team A tasks
- Finalize firmware safety and saturation behavior under aggressive transients.
- Add diagnostics counters (missed frames, watchdog trips, encoder overflow).

**Exit criteria**
- Straight-line and turn tracking are stable.
- EKF output is drift-reduced and consistent in RViz figure-eight tests.

## Phase 4 — Nav2 Autonomy for Parking (Team B lead, Team A on standby)

### Team B tasks
1. Tune costmaps for tight space (high resolution + inflation tuning).
2. Move from generic differential assumptions toward Ackermann-friendly config:
   - Evaluate Smac Hybrid planner + TEB/RPP-style local control for curvature constraints.
3. Refine behavior tree for parking (remove non-useful spin recovery for car-like platform).
4. Execute repeated autonomous parking goals and tune tolerances.

### Team A tasks
- Support performance tuning from real-world logs (steering latency, wheel asymmetry compensation).

**Exit criteria**
- Repeatable autonomous parking with bounded final pose error.

---

## 5) Practical answer to your question: can teams work separately?

**Yes — absolutely, and this is the recommended approach.**

The key is to keep the integration boundary stable:
- freeze command/feedback message contracts,
- define timing and safety requirements,
- run CI-style replay tests on recorded command traces.

If this interface is fixed early, Team A can complete deterministic control while Team B advances Nav2 and localization independently.

---

## 6) Immediate next sprint (first 2 weeks)

## Team A (Pico) sprint backlog
1. Servo calibration utility + calibration constants in firmware.
2. Encoder ISR + stress test and pulse-loss report.
3. Dual PID loop with runtime gain tuning over serial.
4. Safety watchdog + estop command path.

## Team B (RPi5) sprint backlog
1. ROS2 `cmd_vel` capture node and command-shaping node.
2. Communication bridge stub (mock Pico first, real transport second).
3. Topic contracts (`control_cmd`, `control_feedback`) + rosbag test scripts.
4. EKF launch skeleton prepared for Pico odometry/IMU fusion.

## Joint milestone demo
- Send commanded speed/yaw profiles from RPi5 to Pico on bench,
- show measured wheel response and steering tracking in ROS plots,
- verify timeout-to-safe-stop behavior.
