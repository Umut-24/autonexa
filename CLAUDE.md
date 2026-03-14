# AutoNexa — Autonomous Parking System

## Overview

AutoNexa is an autonomous parking robot using ROS2 Nav2 on a Raspberry Pi 5 with a Raspberry Pi Pico handling low-level Ackermann steering control. The system maps indoor parking environments via SLAM, localizes within saved maps, and navigates autonomously to parking spots.

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Robotics framework | ROS2 Humble |
| Navigation | Nav2 (planner: Navfn, controller: DWB) |
| SLAM / Localization | SLAM Toolbox, AMCL |
| Sensor fusion | robot_localization (EKF) |
| High-level nodes | Python 3 (rclpy) |
| Embedded firmware | C (Pico SDK, bare-metal ARM Cortex-M0+) |
| Communication | Serial USB 115200 baud (RPi5 ↔ Pico), I2C (Pico ↔ motor driver) |
| Build | colcon (ROS2 pkg), CMake (Pico firmware) |
| Visualization | RViz2 |

## Project Structure

```
src/parking_system/          # ROS2 package (ament_cmake + Python)
  launch/                    # Launch files for each operational mode
  config/                    # YAML parameters (Nav2, AMCL, SLAM, scan filters)
  scripts/                   # Python ROS2 nodes
    custom_navigation/       # A* pathfinding with PyQt5 GUI
    parking_system/          # Coordinator, slot selector, path monitor
    diagnostics/             # Scan quality, localization, TF tree checks
  urdf/                      # Robot URDF description
  rviz/                      # RViz config per mode (mapping, localization, nav)

pico_firmware/               # Raspberry Pi Pico C firmware
  include/                   # Headers: config.h, ackermann.h, servo.h, safety.h
  src/                       # Implementation: main.c, ackermann.c, servo.c, safety.c

aruco_project/               # Vision-based ArUco marker system + Flutter mobile app
maps/                        # Saved occupancy grids (.pgm + .yaml)
```

## Build & Run

### ROS2 Package

```bash
# Build (from workspace root, e.g. ~/intelligent_parking_ws)
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Pico Firmware

```bash
cd pico_firmware && mkdir -p build && cd build
cmake .. && make
# Flash: copy autonexa_pico.uf2 to Pico in BOOTSEL mode
```

### Operational Modes

```bash
# Mapping (create new map)
ros2 launch parking_system mapping.launch.py

# Localization (use saved map)
ros2 launch parking_system localization.launch.py map_file:=maps/parking_map.yaml

# Full navigation
ros2 launch parking_system parking_navigation.launch.py map_yaml:=$PWD/maps/parking_map.yaml

# Live SLAM + Nav
ros2 launch parking_system nav2_live_slam.launch.py

# RPi5 ↔ Pico bridge only
ros2 launch parking_system rpi5_pico_bridge.launch.py
```

### Diagnostics

```bash
ros2 run parking_system diagnose_scan_quality.py
ros2 run parking_system diagnose_localization.py
ros2 run parking_system diagnose_tf_tree.py
```

## Key Data Flow

```
Nav2 /cmd_vel → Bridge (rate/accel limiting) → Serial → Pico (Ackermann inverse kinematics) → Motors+Servo
Pico encoders → TEL serial → Transceiver → JointState → Odom integration → EKF → AMCL
LIDAR /scan → AMCL (localization) + Nav2 costmaps
```

## Pico Serial Protocol

- **Downlink**: ASCII commands — `VEL <vx> <wz>`, `ENABLE`, `DISABLE`, `ESTOP`, `STATUS`
- **Uplink**: `TEL <ms>,<L_pwm>,<R_pwm>,<steer>,<enc_L>,<enc_R>,<x>,<y>,<yaw>,<estop>,<timeout>` at 10Hz
- Full command list: `pico_firmware/src/main.c:258`

## Vehicle Parameters

Defined in `pico_firmware/include/config.h`:
- Wheelbase: 0.25m, Track: 0.20m, Wheel radius: 0.033m
- Max steering: ±30° (0.5236 rad)
- Encoder: 1320 edges/rev (JGB37-520R30, 1:30 gear ratio)
- Control loop: 50Hz, Command timeout: 200ms

## Additional Documentation

Check these files when working on specific areas:

| File | When to check |
|------|--------------|
| `.claude/docs/architectural_patterns.md` | Modifying nodes, adding features, understanding design decisions |
| `aruco_project/ARCHITECTURE_RECOMMENDATIONS.md` | Working on the vision/ArUco subsystem |
| `docs/rpi5_pico_dual_team_implementation_plan.md` | Understanding RPi5 ↔ Pico integration design |
| `CUSTOM_NAVIGATION_SETUP.md` | Working on the custom A* path planner |
| `aruco_project/INTEGRATION_GUIDE.md` | Mobile app integration |
