# Architectural Patterns

## 1. Two-Tier Control Architecture

High-level planning runs on RPi5 (Linux/ROS2) while deterministic real-time control runs on Pico (bare-metal C). The interface contract is `(vx, wz)` — linear and angular velocity — allowing each tier to evolve independently.

- RPi5 side: `src/parking_system/scripts/cmd_vel_to_pico_bridge.py`
- Pico side: `pico_firmware/src/main.c:106` (VEL command handler)

## 2. Bridge Pattern (Transport Decoupling)

The bridge node (`cmd_vel_to_pico_bridge.py`) sits between Nav2 and the Pico. It publishes to three topics — TwistStamped, JSON String, and Bool heartbeat — so transport layers (serial, micro-ROS, CAN) can subscribe independently without coupling to Nav2.

The serial transceiver (`pico_serial_transceiver.py`) subscribes to the JSON topic and converts to ASCII CLI commands. This separation means swapping serial for micro-ROS requires only replacing the transceiver, not the bridge.

Topology: Nav2 `/cmd_vel` → Bridge → `/pico/control_cmd_json` → Transceiver → serial

## 3. Safety Layering

Safety is enforced at multiple levels to prevent single-point failures:

| Layer | Implementation | Mechanism |
|-------|---------------|-----------|
| Bridge (RPi5) | `cmd_vel_to_pico_bridge.py:96-104` | 200ms command timeout → ramp to zero via acceleration limiter |
| Bridge (RPi5) | `cmd_vel_to_pico_bridge.py:44-48` | Velocity and acceleration clamping |
| Transceiver | `pico_serial_transceiver.py:107-115` | ENABLE/DISABLE/ESTOP state transitions |
| Pico watchdog | `pico_firmware/src/safety.c:30-46` | Independent 200ms timeout → hard motor stop |
| Pico E-STOP | `pico_firmware/src/safety.c:69-74` | Highest priority, overrides all commands |
| Heartbeat LED | `pico_firmware/src/safety.c:54-61` | Visual state indicator (5Hz = ESTOP, 1Hz = normal) |

## 4. Configuration-Driven Behavior

All runtime-tunable parameters live in external config files, not in source code:

- Nav2 stack: `config/nav2_params.yaml`
- AMCL particle filter: `config/amcl_params.yaml`, `config/amcl_small_scale.yaml`
- SLAM: `config/slam_toolbox_mapping.yaml`, `config/slam_toolbox_localization.yaml`
- Scan filtering: `config/scan_filter.yaml`
- Hardware constants: `pico_firmware/include/config.h`

ROS2 nodes declare all parameters via `declare_parameter()` with defaults, and launch files override them. See `rpi5_pico_bridge.launch.py:30-39` for the bridge parameter override pattern.

## 5. ROS2 Node Pattern

All Python nodes follow the same structure:

1. Class inherits from `rclpy.node.Node`
2. `__init__`: declare parameters → get parameters → create pub/sub/timers
3. Callbacks for subscriptions and timers
4. `main()`: `rclpy.init()` → instantiate → `rclpy.spin()` → cleanup in `finally`

Examples: `cmd_vel_to_pico_bridge.py:30-78`, `pico_serial_transceiver.py:30-69`, `pico_joint_feedback_to_odom.py:25-64`

## 6. Odometry Pipeline (Encoder → TF)

Raw encoder data flows through a multi-stage pipeline:

1. **Pico firmware** reads encoder deltas at 50Hz and computes on-board odom (`main.c:376-397`)
2. **Pico** emits `TEL` telemetry at 10Hz with raw encoder counts (`main.c:300-315`)
3. **Transceiver** parses TEL, converts ticks→rad/s, publishes JointState (`pico_serial_transceiver.py:141-187`)
4. **Odom node** integrates wheel velocities + steering via Ackermann forward kinematics (`pico_joint_feedback_to_odom.py:99-110`)
5. **EKF** (robot_localization) optionally fuses wheel odom with laser scan matcher odom

Design choice: ROS-side odom integration (step 4) is preferred over Pico's on-board odom (step 1) for TF tree consistency. See `pico_serial_transceiver.py:156-157`.

## 7. Ackermann Kinematics

Inverse and forward kinematics are implemented in `pico_firmware/src/ackermann.c`:

- **Inverse** (`ackermann_inverse`): `(vx, wz)` → `(steering_angle, v_left, v_right)`
- **Forward** (`ackermann_forward`): `(v_left, v_right, steering_angle, dt)` → odometry update

The same kinematic model is mirrored in Python for odom integration (`pico_joint_feedback_to_odom.py:103-110`): `vx = avg(v_left, v_right)`, `wz = vx * tan(steer) / wheelbase`.

## 8. Launch Composition

Each operational mode has its own launch file that composes the needed subset of nodes. The bridge launch (`rpi5_pico_bridge.launch.py`) demonstrates the pattern: declare arguments with defaults → create Node actions with parameter overrides → return LaunchDescription.

Navigation launches (`parking_navigation.launch.py`, `nav2_live_slam.launch.py`) compose larger stacks by including Nav2 bringup, robot description, and bridge nodes together.

## 9. Pico Firmware State Machine

The firmware uses a simple three-mode model defined in `config.h:60-64`:

- `MODE_MANUAL` — direct CLI control
- `MODE_AUTO` — accepts VEL commands, watchdog enforced
- `MODE_ESTOP` — all outputs forced to safe state

Motor commands only execute when `motors_enabled && safety_is_ok()` (`main.c:125`, `main.c:403`). The watchdog (`safety.c:30-46`) and E-STOP (`safety.c:48-52`) are checked independently every control loop iteration.

## 10. Pico Serial Protocol Design

The protocol uses human-readable ASCII for debuggability:

- **Commands**: verb-first format (`VEL 0.1 0.5`, `ENABLE`, `STATUS`)
- **Responses**: prefixed (`OK`, `ERR`, `TEL`, `ENC`, `STATUS`)
- **Telemetry**: CSV format at 10Hz for efficient parsing

This makes bench testing trivial (connect any serial terminal) while remaining machine-parseable. See `main.c:53-280` for the full command parser.
