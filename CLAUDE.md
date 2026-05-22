# AutoNexa — Autonomous Parking System

Autonomous parking robot on Raspberry Pi 5 (Ubuntu 24.04, ROS2 Jazzy) with a Raspberry Pi Pico handling low-level Ackermann actuation. The system builds a map live with SLAM Toolbox while Nav2 navigates — **no pre-saved map is required**. Static-map / AMCL / custom A* planner / parking slot selector workflows have been archived to `_archive/`.

Pico firmware is C (Pico SDK + micro-ROS) under `pico_firmware/`. It builds two UF2s from one source tree: `autonexa_pico_uros.uf2` for ROS2/Nav2 integration and `autonexa_pico.uf2` for a serial-CLI bench-test build. A short-lived MicroPython migration was abandoned on 2026-05-01 — the C firmware is the only actuation firmware path.

Workspace root: `/home/autonexa/autonexa` (older docs may reference `~/intelligent_parking_ws` — same workspace).

## Recent additions (2026-05-07) — App-driven operator console

The Flutter app is now a full operator console (the user typically has no monitor / RViz access). Headline additions:

- **Manual safety bypass** — Control tab has a SOFT/OFF chip. OFF publishes joystick to `/cmd_vel_manual` (separate from the Nav2-safety-chain `/cmd_vel_safe`); `nav2_pico_bridge` subscribes to both and the freshest message in the last 200 ms wins. Bridge clamps + watchdog still apply.
- **Direction calibration wizard** — Settings → Calibrate Direction. Two pulses (forward + left) flip `vx_polarity` / `servo_polarity` if the robot moved the wrong way. Values persist in `~/.autonexa/runtime_overrides.yaml` and are re-applied on bridge startup.
- **Servo slew-rate limiter** — `nav2_pico_bridge` now rate-limits steering in radian-space (default `max_steer_rate_radps: 3.0`) so Nav2 wz step changes don't make the servo "thunk".
- **Map / Nav2 reset from app** — Map tab ⋮ menu: "Clear obstacles" (calls `clear_entirely_global_costmap` + `clear_entirely_local_costmap`) and "Restart mapping" (slam_toolbox lifecycle deactivate→cleanup→configure→activate; bumps map version so the app refetches blank PNG).
- **Pose reset / relocalize** — long-press on the Map tab publishes a `PoseWithCovarianceStamped` on `/initialpose` so AMCL or SLAM Toolbox snaps to the user's claimed pose.
- **Manual waypoints** — Parking tab now has a "Manual Spots" sub-tab. Save current pose as `park` / `summon` / `home`. Stored in `~/.autonexa/waypoints.json` with a per-map fingerprint (`width,height,resolution,origin,crc32`). After a SLAM restart the fingerprint changes and stale waypoints are flagged.
- **Live Nav2 max-speed slider** — Settings → Nav2 Max Speed. POSTs to `/api/nav2_speed`, which calls `SetParameters` against `/controller_server` (the active controller's speed cap — `FollowPath.vx_max` under MPPI, `FollowPath.desired_linear_vel` under RPP; auto-detected) and `/velocity_smoother` (`max_velocity[0]`) in lockstep + persists.
- **Topic health panel** — Diagnostics tab shows expected-Hz vs observed EWMA rate per topic (`/scan`, `/map`, `/odom`, `/cmd_vel_safe`, `/pico/joint_feedback`, `/plan`) with green/yellow/red dots.
- **Generic param tuner** — Diagnostics tab → "Open param tuner". Whitelisted nodes only (`/nav2_pico_bridge`, `/controller_server`, `/planner_server`, `/velocity_smoother`, `/global_costmap/global_costmap`, `/local_costmap/local_costmap`). Numeric edits persist to `runtime_overrides.yaml` under the node's section.

Persistent operator data lives in `~/.autonexa/` outside the ROS workspace:
- `runtime_overrides.yaml` — per-node parameter overrides replayed on bridge startup (loaded via `add_on_set_parameters_callback` so the validation path runs).
- `waypoints.json` — manual park/summon/home spots with map fingerprint binding.

## Tech Stack

| Layer | Technology |
|-------|-----------|
| OS / SBC | Ubuntu 24.04 (Noble) on Raspberry Pi 5 |
| Robotics framework | ROS2 Jazzy |
| Navigation | Nav2 — `SmacPlannerHybrid` (global) + switchable local controller: `MPPIController` (default) / Regulated Pure Pursuit (fallback, `controller:=rpp`) |
| SLAM | SLAM Toolbox (`async_slam_toolbox_node`, mapping mode, 2 cm/pixel) |
| Odometry | `ros2_laser_scan_matcher` (ICP scan-to-scan → `odom→base_link` TF + `/odom`) |
| Sensor fusion | `robot_localization` EKF skeleton ready (inactive until IMU added) |
| High-level nodes | Python 3 (rclpy) |
| Embedded firmware | C on Pico (Pico SDK + micro-ROS, `pico_firmware/`) |
| RPi5 ↔ Pico comm | USB CDC serial @ **115200 baud**. Two transports: micro-ROS agent (XRCE-DDS) for Nav2 integration; ASCII line CLI for bench testing |
| Pico ↔ motors | I2C0 @ 100 kHz to Hiwonder YX-4055AM motor driver |
| LIDAR | SLAMTEC C1 (`sllidar_ros2` driver, 460800 baud, ~10 Hz scans) |
| Build | colcon (ROS2 + `--cmake-args "-DCMAKE_PREFIX_PATH=/usr/local"`), CMake (Pico firmware) |
| Mobile app | Flutter (ArUco `DICT_4X4_50` IDs 0–9 + HTTP bridge on port 5000) |
| Visualization | RViz2 |

## Hardware

### Pico pinout (mirrored in `pico_firmware/include/config.h`)

| GPIO | Function | Target |
|------|----------|--------|
| 2 | GPIO out | L298N IN1 (right motor direction A) |
| 3 | GPIO out | L298N IN2 (right motor direction B) |
| 4 | PWM @ 10 kHz | L298N ENA (right motor enable / duty) |
| 6 | GPIO out | L298N IN3 (left motor direction A) |
| 7 | GPIO out | L298N IN4 (left motor direction B) |
| 8 | PWM @ 10 kHz | L298N ENB (left motor enable / duty) |
| 15 | PWM @ 50 Hz | Steering servo (500–2500 µs, ±30°) |
| 25 | LED | Heartbeat: 1 Hz normal, **5 Hz = E-STOP latched** |
| USB | CDC serial | RPi5 @ 115200 baud |

L298N output mapping: **OUT1-OUT2 → right motor**, **OUT3-OUT4 → left motor**. Motors are JGB37-520R30 12 V DC, 1:30 gearbox; encoders not currently wired (open-loop only, no on-board feedback). Switched from the Hiwonder I2C smart driver on 2026-05-06 after its MCU burned. Logical channel numbering for the CLI verbs: **M1 = left, M2 = right** (matches GUI bench panel).

### Vehicle parameters (mirrored in `pico_firmware/include/config.h`)
Wheelbase 0.25 m · Track 0.20 m · Wheel radius 0.033 m · Max steering ±30° (0.5236 rad) · Control loop 50 Hz · **Command timeout 200 ms** (firmware watchdog).

## Pico Firmware

`pico_firmware/` produces two UF2s from the same control core:

| UF2 | Purpose |
|-----|---------|
| `autonexa_pico_uros.uf2` | micro-ROS client over USB serial. Production target for Nav2 integration. |
| `autonexa_pico.uf2` | ASCII serial CLI — bench test only. Driven from the RPi5 by `test/pico_gui.py`. |

Build selection is at compile time via `USE_MICRO_ROS` in CMake (see `pico_firmware/CMakeLists.txt`). Flash via BOOTSEL: `cp build/<target>.uf2 /media/$USER/RPI-RP2/`.

Bench-test (CLI) command surface and `TEL` telemetry format are documented in [docs/pico_control_test_guide.md](docs/pico_control_test_guide.md).

## Project Structure

```
src/parking_system/              # Main ROS2 package (ament_cmake + Python)
  launch/
    nav2_live_slam.launch.py     # Mapping mode: SLAM + Nav2 + LiDAR + Pico bridge + RViz
    nav2_amcl_navigation.launch.py # Localization mode: pre-saved map + AMCL + Nav2 (no live SLAM)
    rpi5_pico_bridge.launch.py   # Pico bridge only (micro-ROS agent + cmd_vel bridge)
    ekf_fusion.launch.py         # Optional EKF skeleton (publish_tf: false, no IMU yet)
    lidar_visualization.launch.py
    visualization.launch.py      # Robot URDF only, no hardware
    robot_description_launch.py
  config/
    nav2_navigation_params.yaml  # Nav2 planner/controller/costmap/smoother/collision_monitor
    slam_toolbox_mapping.yaml    # Async mapping, 2 cm/pixel, 0.05 m/0.05 rad update
    ekf_2d_no_imu.yaml           # EKF skeleton (publish_tf: false to avoid TF conflict)
    laser_scan_matcher.yaml      # ICP: 0.10 m / 10° keyframe thresholds
    scan_filter.yaml             # 4-stage filter chain (range→shadow→median→outlier)
  scripts/
    cmd_vel_to_pico_bridge.py    # /cmd_vel_safe → /pico/control_cmd (30 Hz)
    ros2_mobile_bridge.py        # Flask HTTP bridge on :5000 for Flutter app
    nav2_activator.py            # Programmatic NavigateToPose goal sender
    print_robot_position.py
    diagnose_scan_quality.py
    diagnose_localization.py
    diagnose_tf_tree.py
    diagnose_control_chain.py    # Topic/type + flow + single-publisher checks
    record_control_chain_bag.py  # Standardized rosbag capture
  rviz/                          # navigation.rviz + visualization.rviz
  urdf/robot.urdf                # laser_link +150 mm fwd, +120 mm up; camera +100 mm fwd, +50 mm up

pico_firmware/                   # C firmware (Pico SDK + micro-ROS, bare-metal)
  include/   config.h ackermann.h servo.h motor_control.h hiwonder_driver.h safety.h uros_transport.h
  src/       main.c ackermann.c servo.c motor_control.c hiwonder_driver.c safety.c uros_transport.c uros_time_shim.c
  micro_ros_sdk/  # Pico-side USB transport for the micro-ROS build

src/micro-ROS-Agent/             # Vendored RPi5-side XRCE-DDS agent (ROS2 package)

test/
  pico_gui.py                    # Tk GUI — bench-drives autonexa_pico.uf2 via serial CLI

aruco_project/                   # Vision: ArUco marker system + Flutter mobile_app/
docs/                            # Design docs (CDRR, test protocols, implementation plan)
_archive/                        # Archived: static-map workflows, custom A*, parking slot nodes
```

## Build & Run

```bash
# ROS2 workspace
source /opt/ros/jazzy/setup.bash
# -DCMAKE_PREFIX_PATH=/usr/local lets colcon find a system-installed micro_ros_agent
# if the vendored src/micro-ROS-Agent isn't being built from source.
colcon build --symlink-install --cmake-args "-DCMAKE_PREFIX_PATH=/usr/local"
source install/setup.bash

# Pico firmware (C)
cd pico_firmware && mkdir -p build && cd build && cmake .. && make
# Production: flash autonexa_pico_uros.uf2 (Nav2 integration via micro-ROS).
# Bench test: flash autonexa_pico.uf2 and drive with python3 test/pico_gui.py.
```

### Launch commands

```bash
# Mapping mode — full system (SLAM + Nav2 + LiDAR + Pico bridge + RViz):
ros2 launch parking_system nav2_live_slam.launch.py

# Full system with by-id serial paths (deployment):
ros2 launch parking_system nav2_live_slam.launch.py \
  use_pico_bridge:=true \
  enforce_single_publisher:=true \
  serial_port:=/dev/serial/by-id/<lidar-id> \
  pico_serial_port:=/dev/serial/by-id/<pico-id>

# Localization mode — once a map is saved, re-launch against the pre-saved map.
# Map a static testbed once with live-SLAM above, then save via the mobile bridge:
curl -X POST http://localhost:5000/api/lock_map
# (writes ~/.autonexa/maps/garage_<YYYYMMDD_HHMMSS>.{pgm,yaml})
ros2 launch parking_system nav2_amcl_navigation.launch.py \
  map_yaml:=$HOME/.autonexa/maps/garage_<ts>.yaml \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
# AMCL anchors to the saved prior so pose doesn't drift away from the map
# the way live-SLAM does in a multi-minute session. Use /api/relocalize
# (or RViz 2D Pose Estimate) to nudge the pose if it loses lock.

# Bridge only (manual drive / unit tests):
ros2 launch parking_system rpi5_pico_bridge.launch.py

# EKF skeleton (no IMU — publishes /odometry/filtered, TF off)
ros2 launch parking_system ekf_fusion.launch.py
```

### Launch arguments (nav2_live_slam)

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | LiDAR serial (prefer `/dev/serial/by-id/...`) |
| `serial_baudrate` | `460800` | LiDAR baud |
| `pico_serial_port` | `/dev/ttyACM0` | Pico USB serial |
| `use_pico_bridge` | `true` | Launches the micro-ROS agent + `cmd_vel_to_pico_bridge.py`. Set to `false` for headless/sim |
| `enforce_single_publisher` | `true` | Bridge self-terminates if duplicate `/pico/*` publishers detected |
| `bridge_lock_file` | `/tmp/cmd_vel_to_pico_bridge.lock` | fcntl lock for the bridge |
| `use_rviz` | `true` |  |
| `controller` | `mppi` | Local controller: `mppi` (default, obstacle-aware sampling MPC, `config/controller_mppi.yaml`) or `rpp` (Regulated Pure Pursuit fallback, FollowPath block in `nav2_navigation_params.yaml`). Switchable with no rebuild. Both launch files expose this. |
| `bridge_cmd_vel_topic` | `/cmd_vel_safe` | Velocity topic consumed by the bridge |

Note: the launch files still expose a `use_micropython_bridge` argument that's now dead code (the MicroPython bridge script was removed). Leave it at its default `false`.

### Diagnostics

```bash
ros2 run parking_system diagnose_control_chain.py --ros-args \
  -p expect_pico_bridge:=true \
  -p require_single_pico_publisher:=true \
  -p require_flow:=true -p window_s:=12.0

ros2 run parking_system diagnose_scan_quality.py     # /scan rate + range sanity
ros2 run parking_system diagnose_localization.py
ros2 run parking_system diagnose_tf_tree.py
ros2 run parking_system record_control_chain_bag.py  # standardized rosbag
```

## Key Data Flow

```
Goal (RViz 2D Nav Goal / mobile app)
  → BT Navigator → SmacPlannerHybrid (global, /map) → MPPIController (default) or RPP (controller:=rpp)
  → /cmd_vel → velocity_smoother (20 Hz, 1.5 m/s², 2.0 rad/s²)
  → /cmd_vel_smoothed → collision_monitor (1.2 s lookahead, FootprintApproach)
  → /cmd_vel_safe → cmd_vel_to_pico_bridge.py (30 Hz, clamp + accel-limit + 200 ms timeout)
  → /pico/control_cmd (TwistStamped) + /pico/enable + /pico/heartbeat
  → micro-ROS agent (XRCE-DDS over USB CDC 115200)
  → Pico firmware (50 Hz: Ackermann IK → servo PWM + I2C motor speeds)

SLAMTEC C1 → /scan (raw)
  → laser_filters (range 0.05–4.0 m → shadow 10°–170° → median 5 → outlier 0.5 m/win 5)
  → filtered /scan
  → laser_scan_matcher → /odom + odom→base_link TF
  → SLAM Toolbox (async mapping, 2 cm/pixel) → /map + map→odom TF
  → Nav2 costmaps (both at 2 cm resolution; local 2×2 m, global full-map)
```

**Goal-to-motor latency:** ~500–800 ms (planner-dominated). Steady-state path-follow: ~200–300 ms obstacle-to-response.

## TF Tree

```
map → odom → base_link → laser_link
      (laser_scan_matcher)  (robot_state_publisher)
(SLAM Toolbox in mapping mode, or AMCL in localization mode)
```

`map→odom` ownership swaps depending on launch file: SLAM Toolbox publishes it when running `nav2_live_slam.launch.py`; `nav2_amcl` publishes it when running `nav2_amcl_navigation.launch.py` against a pre-saved map. `laser_scan_matcher` always owns `odom→base_link`.

EKF (when IMU lands) will take over `odom→base_link`; `ekf_2d_no_imu.yaml` is pre-configured with `publish_tf: false` to avoid fighting the scan matcher until the swap is made.

## Pico ROS Topics

The Pico publishes/subscribes these directly via micro-ROS over USB CDC.

| Topic | Type | Direction | Rate | Notes |
|-------|------|-----------|------|-------|
| `/pico/control_cmd` | `TwistStamped` | RPi5 → Pico | 30 Hz | `frame_id: base_link`; vx (m/s), wz (rad/s) |
| `/pico/enable` | `Bool` | RPi5 → Pico | 30 Hz | Bridge sets `false` on timeout |
| `/pico/heartbeat` | `Bool` | RPi5 → downstream | 5 Hz | Liveness indicator |
| `/pico/odom` | `Odometry` | Pico → RPi5 | 20 Hz | Forward kinematics from encoders; **not fused into TF** (scan matcher owns `odom→base_link`) |
| `/pico/joint_feedback` | `JointState` | Pico → RPi5 | 10 Hz | `left_wheel_joint` / `right_wheel_joint` / `steering_joint` |
| `/pico/estop` (service) | `SetBool` | RPi5 → Pico | on demand | `data:true` latches E-STOP, `data:false` clears |

For the bench-test CLI build (`autonexa_pico.uf2`) — a separate ASCII line protocol over USB CDC, not micro-ROS — see [docs/pico_control_test_guide.md](docs/pico_control_test_guide.md).

## Nav2 Tuning Key Values

| Item | Value | Rationale |
|------|------:|-----------|
| Costmap resolution | 0.02 m (2 cm) | Parking slot (~0.5×1.0 m) = 25×50 cells — enough for ArUco docking |
| `desired_linear_vel` (RPP) | 0.15 m/s | Tight-testbed tuning; user can live-tune via speed slider |
| `max_vel_theta` | 0.50 rad/s | ~28°/s |
| Goal tolerance | 0.15 m XY, 0.10 rad yaw | Loosened for L298N deadband |
| Local costmap | 2×2 m rolling, 20 cm inflation | Generous margin (robot radius 10 cm; URDF footprint underestimates linkage) |
| Global costmap | full `/map`, 15 cm inflation | Smaller than local so planner doesn't route unnecessarily wide |
| `allow_unknown` | `false` | Robot won't plan into unmapped space — survey first, then goal |
| RPP lookahead | 0.40 m (min 0.30, max 0.60) | Shortened for 1-2 m testbed; prevents carrot landing beyond turns |

Planner is SMAC Hybrid-A* (REEDS_SHEPP, `allow_reversing: true`). The local
controller is **switchable** via the `controller` launch arg:
- **`mppi` (default)** — `nav2_mppi_controller::MPPIController`, an obstacle-aware
  sampling MPC (Ackermann motion model, `config/controller_mppi.yaml`). It keeps
  clearance from walls *while continuing to drive*, so it doesn't need the binary
  collision-monitor stop. This is why the AMCL launch can leave collision_monitor
  disabled — MPPI's `CostCritic` + the Pico-bridge clamps/200 ms watchdog/E-STOP are
  the obstacle-safety layers. **MPPI is the heaviest Nav2 node — benchmark CPU +
  `/cmd_vel` Hz on the Pi 5 (see TEST.md) before trusting tuning; back off
  `batch_size`/`time_steps`/`controller_frequency` if it can't hold rate.**
- **`rpp` (fallback)** — Regulated Pure Pursuit (the `FollowPath` block in
  `nav2_navigation_params.yaml`); the tuning values in the table above apply to it.
  Use `controller:=rpp` if MPPI can't sustain the control rate on the Pi.

Requires both controller packages installed on the Pi:
`sudo apt install ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-regulated-pure-pursuit-controller`.

## Pico Bridge (cmd_vel → micro-ROS)

`cmd_vel_to_pico_bridge.py` consumes `/cmd_vel_safe`, applies output limits + accel cap + a 200 ms input watchdog, and republishes to `/pico/control_cmd` for the Pico's micro-ROS subscriber.

| Parameter | Default | Description |
|-----------|--------:|-------------|
| `publish_rate_hz` | 30.0 | Output to `/pico/control_cmd` |
| `command_timeout_s` | 0.20 | Zero-vel ramp + `enable:=false` if no input |
| `max_vx_mps` | 0.35 | Hard floor above DWB limit |
| `max_wz_radps` | 0.8 | Hard yaw cap |
| `max_ax_mps2` | 0.8 | Per-cycle accel clamp |
| `max_aw_radps2` | 1.2 | Per-cycle angular accel clamp |

**Single-publisher guard** runs at two levels: (1) `fcntl` lock (`/tmp/cmd_vel_to_pico_bridge.lock`) blocks a 2nd process on the same host; (2) every 1 s (after 3 s startup) the bridge counts publishers on `/pico/control_cmd` and if > 1, publishes zero-vel safe stop and shuts down.

## Nav2 ASCII Bridge (cmd_vel → serial CLI)

`nav2_pico_bridge.py` is the **active actuation path** for the L298N CLI Pico build (`autonexa_pico.uf2`). Subscribes to `/cmd_vel_safe` *and* `/cmd_vel_manual` (freshest message in the last 200 ms wins), applies vx/wz clamps + accel cap + steer slew-rate limit, computes Ackermann inverse kinematics, and writes `SPEED <n>` + `SERVO_PWM <us>` lines over USB CDC at 30 Hz. Mutually exclusive with the legacy micro-ROS path via `use_serial_bridge:=true`.

| Parameter | Default | Description |
|-----------|--------:|-------------|
| `cmd_vel_topic` | `/cmd_vel_safe` | Nav2 / safety-chain input |
| `manual_cmd_vel_topic` | `/cmd_vel_manual` | App safety-bypass input (set empty to disable) |
| `publish_rate_hz` | 30.0 | Tick rate; SPEED feeds firmware watchdog |
| `command_timeout_s` | 0.20 | Stale-input cutoff |
| `max_vx_mps` | 0.30 | Linear cap |
| `max_wz_radps` | 0.8 | Yaw cap |
| `max_ax_mps2` / `max_aw_radps2` | 0.8 / 1.2 | Per-cycle accel caps |
| `max_steer_rate_radps` | 3.0 | Servo slew limit (rad/s in steering-angle space) |
| `min_vx_creep` | 0.02 | Sub-deadband vx → SPEED 0 (lowered to let slow curve speeds through) |
| `vx_polarity` | +1 | Forward/back inversion (calibration wizard flips this) |
| `servo_polarity` | -1 | Steering inversion (chassis-specific) |
| `servo_center_us` / `servo_us_min` / `servo_us_max` | 1650 / 1100 / 1900 | Calibrated servo bounds |

Runtime overrides are loaded from `~/.autonexa/runtime_overrides.yaml` (under key `nav2_pico_bridge`) at startup and re-applied via `set_parameters` so the validation callback runs. The mobile bridge writes that file when the user toggles polarity / changes a tunable from the app.

## Safety Layer Stack (outermost → innermost)

The user-facing **safety mode** ('soft' default | 'off') chooses *which* of two parallel chains a manual joystick takes; AUTO mode always uses the soft chain. Layers 3+ apply on both.

| # | Layer | Soft (default) | Off (manual bypass) |
|---|-------|---------------|---------------------|
| 1 | Velocity smoother | Accel-limit ramp | bypassed |
| 2 | Collision monitor | Slow / zero on predicted collision (1.2 s) | bypassed |
| 3 | Bridge clamping | Hard clip vx/wz | Hard clip vx/wz |
| 4 | Bridge accel cap | Per-cycle accel limit | Per-cycle accel limit |
| 4.5 | Servo slew limit | rad/s cap on steering | rad/s cap on steering |
| 5A | Bridge command watchdog (RPi5) | 200 ms → zero | 200 ms → zero |
| 5B | Pico command watchdog (`safety.c`) | 200 ms → motors off | 200 ms → motors off |
| 6 | Pico E-STOP (latching) | Explicit clear only; 5 Hz LED | Explicit clear only; 5 Hz LED |

Layers 5A and 5B share the 200 ms threshold but are independent — if USB CDC drops, 5A is blind but 5B still fires. "Off" exists for the case where collision_monitor refuses to let the user nudge a stuck robot off a wall — bridge clamps + watchdogs + E-STOP still bound the danger.

## Mobile Bridge (`ros2_mobile_bridge.py`) HTTP / WebSocket Surface

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/status`, `/api/scan`, `/api/map`, `/api/map_info`, `/api/map_version`, `/api/plan`, `/api/goal`, `/api/pose`, `/api/markers`, `/api/telemetry`, `/api/nav_status` | GET | State |
| `/api/control`, `/api/nav_goal`, `/api/cancel_nav`, `/api/estop`, `/api/estop_clear` | POST | Drive / nav |
| `/api/mode` | GET/POST | AUTO / MANUAL / ESTOP |
| `/api/safety_mode` | GET/POST | soft (default) / off — picks `/cmd_vel` vs `/cmd_vel_manual` for joystick output |
| `/api/calibrate_direction` | GET/POST | `vx_polarity` / `servo_polarity` ±1 → SetParameters + persist |
| `/api/clear_costmaps` | POST | `nav2_msgs/ClearEntireCostmap` on global + local |
| `/api/restart_mapping` | POST | slam_toolbox lifecycle deactivate→cleanup→configure→activate |
| `/api/relocalize` | POST | Publish `PoseWithCovarianceStamped` on `/initialpose` |
| `/api/waypoints` | GET/POST | List / upsert manual park/summon/home spots (with map fingerprint) |
| `/api/waypoints/<name>` | DELETE | Remove |
| `/api/waypoints/<name>/navigate` | POST | Republish stored pose on `/goal_pose` |
| `/api/nav2_speed` | GET/POST | active controller speed cap (`FollowPath.vx_max` MPPI / `FollowPath.desired_linear_vel` RPP, auto-detected) + `velocity_smoother.max_velocity[0]` in lockstep |
| `/api/planner_mode` | GET/POST | `standard` (single SMAC goal) / `multipoint` (on ABORT, stage via an intermediate waypoint). Persisted in `~/.autonexa/planner_mode.txt` |
| `/api/params` | GET/POST | Generic SetParameters / ListParameters / GetParameters; whitelist enforced |
| `/api/health` | GET | Per-topic EWMA rate + age + ok flag |
| `/ws/control` | WS | 50 Hz joystick frames |
| `/ws/telemetry` | WS | 10 Hz snapshot push (pose + telemetry + mode + safety_mode + nav_status + goal + map_fingerprint) |
| `/video_feed` | GET | Camera MJPEG |

## Integration Test Ordering

`TEST.md` gates stages 0–10; each gatekeeps the next (do not skip):

```
0 Build + flash autonexa_pico_uros.uf2     6 LiDAR scan + SLAM map
1 USB serial bytes from Pico               7 Mobile app joystick
2 /pico/* topics appear                    8 Autonomous nav to RViz goal
3 Motors spin on direct cmd_vel            9 App tap-to-navigate + E-STOP cancel
4 /cmd_vel → /cmd_vel_safe → /pico chain  10 ArUco parking approach
5 /pico/estop service latches and clears
```

If Stage 3 fails, Stage 7 cannot work. Stage 3 commands must be published at `--rate 10` to keep the 200 ms watchdog alive.

## Known Open Items

- **Pico odom not fused into TF** — published at 20 Hz but only scan matcher owns `odom→base_link`. Fusion waits on IMU.
- **No IMU connected** — EKF skeleton staged with `publish_tf: false`.
- **Control-source arbitration** — AUTO/MANUAL/ESTOP state machine plus a SOFT/OFF safety mode now live in `ros2_mobile_bridge.py`. Joystick is mode-gated (only published in MANUAL); safety_mode picks `/cmd_vel` (full chain) vs `/cmd_vel_manual` (bypass). Single-publisher guard remains as a backstop.
- **LiDAR stale-process lock** — killing `sllidar_node` uncleanly can hold `/dev/ttyUSB0`; next launch hits `SL_RESULT_OPERATION_TIMEOUT`. Fix: `sudo fuser -k /dev/ttyUSB0`. Prefer `/dev/serial/by-id/` paths.
- **`pico_firmware/micro_ros_sdk`** — submodule has local changes outside main commit chain.
- **Dead `use_micropython_bridge` launch arg** — `nav2_live_slam.launch.py` and `rpi5_pico_bridge.launch.py` still declare it; the `:=true` branch points at a removed script and will fail. Default `false` is fine.

## Archive

`_archive/` preserves (excluded from build via `.gitignore`):
- Static-map workflows (mapping-only launch, AMCL localization, saved-map navigation)
- Custom A* planner + PyQt5 GUI
- Parking slot selector + coordinator nodes
- Shell debug scripts, TF frame dumps, loose test scripts
- Saved map `.pgm` + `.yaml`

## Additional Documentation

| File | When to read |
|------|--------------|
| `TEST.md` | Before any hardware bringup — staged integration tests 0–10 |
| `docs/pico_control_test_guide.md` | Bench-testing Pico CLI commands, servo/motor/encoder verification, `TEL` telemetry format |
| `test/pico_gui.py` | Bench GUI — hold-to-drive WASD + live `TEL` display against the CLI build |
| `docs/cdrr_perception_navigation.md` | Deep dive on LiDAR filter chain, SLAM params, DWB critics, costmap rationale |
| `docs/rpi5_pico_dual_team_implementation_plan.md` | RPi5 ↔ Pico interface contract + two-team split |
| `docs/IMPLEMENTATION_STATUS_AND_REMAINING_PLAN_2026-03-14.md` | Remaining P0/P1/P2 work and decision gates |
| `docs/AutoNexa_Critical_Design_Review_Report_2026-03-24.md` | CDR-level system-of-systems view, requirements, power budget |
| `docs/subsystem_test_plan.md` | Battery / object-detection / control subsystem test protocols |
| `.claude/docs/architectural_patterns.md` | Design patterns (two-tier control, bridge, safety layering) when adding features |
| `aruco_project/ARCHITECTURE_RECOMMENDATIONS.md` | Vision/ArUco subsystem recommendations |
| `aruco_project/INTEGRATION_GUIDE.md` | Flutter app ↔ RPi5 HTTP bridge |
