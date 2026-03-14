# AutoNexa — Autonomous Parking System

## Overview

AutoNexa is an autonomous parking robot running on a Raspberry Pi 5 (Ubuntu 24.04, ROS2 Jazzy). A Raspberry Pi Pico handles low-level Ackermann steering via micro-ROS. The system builds a map in real time using SLAM Toolbox while navigating with Nav2 — no pre-saved map is required.

**Current operational mode: Live SLAM + Nav2 navigation only.** Static-map workflows (AMCL localization, pre-built map navigation, custom A* planner, parking slot selection) have been archived to `_archive/`.

## Tech Stack

| Layer | Technology |
|-------|-----------|
| OS / SBC | Ubuntu 24.04 (Noble) on Raspberry Pi 5 |
| Robotics framework | ROS2 Jazzy |
| Navigation | Nav2 (planner: NavfnPlanner, controller: DWBLocalPlanner) |
| SLAM | SLAM Toolbox (async mapping mode) |
| Odometry | ros2_laser_scan_matcher (LiDAR scan matching → odom TF) |
| High-level nodes | Python 3 (rclpy) |
| Embedded firmware | C (Pico SDK + micro-ROS, bare-metal ARM Cortex-M0+) |
| RPi5 ↔ Pico comm | micro-ROS agent over USB serial (XRCE-DDS, 115200 baud) |
| Pico ↔ motors | I2C (Hiwonder motor driver board) |
| LIDAR | SLAMTEC C1 (sllidar_ros2 driver) |
| Build | colcon (ROS2), CMake (Pico firmware) |
| Mobile app | Flutter (ArUco marker detection + HTTP bridge) |
| Visualization | RViz2 |

## Project Structure

```
src/parking_system/              # Main ROS2 package (ament_cmake + Python)
  launch/
    nav2_live_slam.launch.py     # Primary launch: SLAM + Nav2 + LiDAR + Pico bridge + RViz
    rpi5_pico_bridge.launch.py   # Standalone Pico bridge (micro-ROS agent + cmd_vel bridge)
    ekf_fusion.launch.py         # Optional EKF fusion skeleton (Pico odom -> /odometry/filtered)
    lidar_visualization.launch.py # LiDAR-only visualization
    visualization.launch.py      # Robot model visualization (no sensors)
    robot_description_launch.py  # URDF publisher only
  config/
    nav2_navigation_params.yaml  # Nav2 planner/controller/costmap parameters
    slam_toolbox_mapping.yaml    # SLAM Toolbox async mapping config
    ekf_2d_no_imu.yaml           # EKF skeleton config (no IMU yet)
    laser_scan_matcher.yaml      # Scan matcher parameters
    scan_filter.yaml             # Laser scan filter chain
  scripts/
    cmd_vel_to_pico_bridge.py    # Nav2 cmd_vel → rate/accel limited /pico/control_cmd
    ros2_mobile_bridge.py        # HTTP bridge for Flutter mobile app control
    nav2_activator.py            # Programmatic Nav2 goal sender
    print_robot_position.py      # Debug: print current robot pose
    diagnose_scan_quality.py     # Diagnostic: LiDAR scan health
    diagnose_localization.py     # Diagnostic: localization status
    diagnose_tf_tree.py          # Diagnostic: TF tree integrity
  rviz/
    navigation.rviz              # Full navigation view (used by nav2_live_slam)
    visualization.rviz           # Basic robot model view
  urdf/
    robot.urdf                   # Robot URDF description

pico_firmware/                   # Raspberry Pi Pico micro-ROS firmware (C)
  include/
    config.h                     # Vehicle parameters, pin assignments, limits
    ackermann.h                  # Ackermann inverse kinematics
    servo.h                      # Steering servo control
    motor_control.h              # DC motor PWM + encoder interface
    hiwonder_driver.h            # I2C Hiwonder motor driver protocol
    safety.h                     # E-stop, timeout, watchdog
    uros_transport.h             # micro-ROS custom serial transport
  src/
    main.c                       # Entry point, micro-ROS setup, control loop
    ackermann.c                  # Ackermann geometry calculations
    servo.c                      # PWM servo driver
    motor_control.c              # Motor PWM + encoder reading
    hiwonder_driver.c            # I2C register-level motor driver
    safety.c                     # Safety state machine
    uros_transport.c             # Custom UART transport for micro-ROS
    uros_time_shim.c             # Clock sync shim for micro-ROS

aruco_project/                   # Vision: ArUco marker system + Flutter mobile app
  mobile_app/                    # Flutter app source

docs/                            # Design documents
_archive/                        # Archived files from earlier dev phases (not in use)
```

## Build & Run

### ROS2 Package

```bash
# From workspace root ~/intelligent_parking_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select parking_system
source install/setup.bash
```

### Pico Firmware

```bash
cd pico_firmware && mkdir -p build && cd build
cmake .. && make
# Flash: copy autonexa_pico.uf2 to Pico in BOOTSEL mode
```

### Launch Commands

```bash
# Full system: live SLAM + Nav2 + LiDAR + Pico bridge + RViz
ros2 launch parking_system nav2_live_slam.launch.py

# Pico bridge only (for manual driving or testing)
ros2 launch parking_system rpi5_pico_bridge.launch.py

# Optional EKF fusion (no IMU)
ros2 launch parking_system ekf_fusion.launch.py

# LiDAR visualization only
ros2 launch parking_system lidar_visualization.launch.py

# Robot model visualization (no hardware needed)
ros2 launch parking_system visualization.launch.py
```

### Launch Arguments (nav2_live_slam)

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | LiDAR serial port |
| `serial_baudrate` | `460800` | LiDAR baud rate |
| `pico_serial_port` | `/dev/ttyACM0` | Pico USB serial port |
| `use_pico_bridge` | `false` | Enable/disable Pico bridge nodes |
| `enforce_single_publisher` | `true` | Stop bridge if duplicate `/pico/*` publishers are detected |
| `bridge_lock_file` | `/tmp/cmd_vel_to_pico_bridge.lock` | Host lock file to prevent multiple bridge instances |
| `use_rviz` | `true` | Enable/disable RViz |
| `bridge_cmd_vel_topic` | `/cmd_vel_safe` | Velocity topic consumed by bridge |

### Diagnostics

```bash
ros2 run parking_system diagnose_scan_quality.py
ros2 run parking_system diagnose_localization.py
ros2 run parking_system diagnose_tf_tree.py
ros2 run parking_system print_robot_position.py
ros2 run parking_system diagnose_control_chain.py
ros2 run parking_system record_control_chain_bag.py
```

Strict duplicate-publisher check:
```bash
ros2 run parking_system diagnose_control_chain.py --ros-args \
  -p expect_pico_bridge:=true \
  -p require_single_pico_publisher:=true
```

## Key Data Flow

```
RViz goal click
  → Nav2 BT navigator → planner_server → controller_server
  → /cmd_vel → velocity_smoother → collision_monitor → /cmd_vel_safe
  → cmd_vel_to_pico_bridge (rate/accel limiting)
  → /pico/control_cmd (TwistStamped) + /pico/enable (Bool)
  → micro-ROS agent (XRCE-DDS over USB serial)
  → Pico firmware (Ackermann IK → servo + motors via I2C)

SLAMTEC C1 LiDAR → /scan
  → laser_scan_matcher → odom TF (map→odom→base_link)
  → SLAM Toolbox (async mapping) → /map
  → Nav2 costmaps (obstacle avoidance)
```

## TF Tree

```
map → odom → base_link → laser_link
      (laser_scan_matcher)  (static TF)
(SLAM Toolbox)
```

## micro-ROS Topics (Pico)

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/pico/control_cmd` | `TwistStamped` | RPi5 → Pico | Velocity command |
| `/pico/enable` | `Bool` | RPi5 → Pico | Motor enable/disable |
| `/pico/heartbeat` | `Bool` | RPi5 → Pico | Keepalive (timeout = 200ms) |

## Vehicle Parameters

Defined in `pico_firmware/include/config.h`:
- Wheelbase: 0.25m, Track: 0.20m, Wheel radius: 0.033m
- Max steering: ±30° (0.5236 rad)
- Encoder: 1320 edges/rev (JGB37-520R30, 1:30 gear ratio)
- Control loop: 50Hz, Command timeout: 200ms

## cmd_vel_to_pico_bridge Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate_hz` | 30.0 | Output publish rate |
| `command_timeout_s` | 0.20 | Zero-vel after no input |
| `max_vx_mps` | 0.35 | Max linear velocity (m/s) |
| `max_wz_radps` | 0.8 | Max angular velocity (rad/s) |
| `max_ax_mps2` | 0.8 | Max linear acceleration |
| `max_aw_radps2` | 1.2 | Max angular acceleration |

## Archive

`_archive/` contains files from earlier development phases that are no longer active:
- Static map workflows (mapping, AMCL localization, saved-map navigation)
- Custom A* path planner with PyQt5 GUI
- Parking slot selection and coordination nodes
- Shell debug scripts, TF frame dumps, loose test scripts
- Saved map data (`.pgm` + `.yaml`)

These are preserved for reference but excluded from the build via `.gitignore`.

## Additional Documentation

| File | When to check |
|------|--------------|
| `.claude/docs/architectural_patterns.md` | Modifying nodes, adding features, understanding design decisions |
| `docs/rpi5_pico_dual_team_implementation_plan.md` | Understanding RPi5 ↔ Pico integration design |
| `aruco_project/ARCHITECTURE_RECOMMENDATIONS.md` | Working on the vision/ArUco subsystem |
| `aruco_project/INTEGRATION_GUIDE.md` | Mobile app integration |
