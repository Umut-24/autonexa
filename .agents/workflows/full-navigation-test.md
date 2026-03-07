---
description: How to start the full navigation system with Pico motor control
---

# Full Navigation Test (RPi5 + Pico + LiDAR + Nav2)

## Prerequisites
- Pico flashed with `autonexa_pico.uf2` and plugged into RPi5 via USB
- LiDAR plugged into RPi5 (usually `/dev/ttyUSB0`)
- Workspace built: `source /opt/ros/jazzy/setup.bash && cd ~/intelligent_parking_ws && colcon build --symlink-install`

---

## Phase A: Verify Hardware Connections

1. Check that the Pico is detected on USB:
// turbo
```bash
ls /dev/ttyACM*
```
Expected: `/dev/ttyACM0` (the Pico)

2. Check that the LiDAR is connected:
// turbo
```bash
ls /dev/ttyUSB*
```
Expected: `/dev/ttyUSB0` (the LiDAR)

3. Make sure your user has permission to access serial ports:
```bash
sudo usermod -aG dialout $USER
```
(Logout and login again if this is the first time)

---

## Phase B: Start the Pico Bridge (Terminal 1)

This connects ROS2 to the physical Pico motors.

4. Launch the bridge:
```bash
cd ~/intelligent_parking_ws
source install/setup.bash
ros2 launch parking_system rpi5_pico_bridge.launch.py
```

Watch the logs. You should see:
- `cmd_vel_to_pico_bridge: Bridge started`
- `pico_serial_transceiver: Connected to Pico on /dev/ttyACM0`
- `pico_joint_feedback_to_odom: Odom node started`

---

## Phase C: Quick Motor Sanity Check (Terminal 2)

Before launching Nav2, verify the motors actually respond.

5. Source the workspace:
// turbo
```bash
cd ~/intelligent_parking_ws && source install/setup.bash
```

6. Send a gentle forward command (0.1 m/s for 2 seconds):
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```
**⚠️ The robot should move forward slightly!** After 200ms timeout, it will stop automatically.

7. Send a gentle turn command:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}"
```
**The steering servo should turn.**

8. Verify odometry is being published:
// turbo
```bash
ros2 topic echo /pico/odom --once
```

9. Check the full topic list:
// turbo
```bash
ros2 topic list | grep pico
```
Expected topics:
- `/pico/control_cmd` (bridge → transceiver)
- `/pico/control_cmd_json` (JSON commands)
- `/pico/joint_feedback` (encoder data from Pico)
- `/pico/odom` (calculated odometry)

---

## Phase D: Launch Full Navigation (Terminal 3)

10. For **SLAM mode** (building a new map while driving):
```bash
cd ~/intelligent_parking_ws && source install/setup.bash
ros2 launch parking_system nav2_live_slam.launch.py
```

11. For **Navigation mode** (using an existing map):
```bash
cd ~/intelligent_parking_ws && source install/setup.bash
ros2 launch parking_system parking_navigation.launch.py map_yaml:=maps/parking_map.yaml
```

---

## Phase E: Drive via RViz

12. In RViz (opens automatically), click **"2D Goal Pose"** in the toolbar
13. Click on the map where you want the robot to go
14. Nav2 will plan a path and drive the robot there using the Pico motors!

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Failed to open /dev/ttyACM0` | Pico not plugged in, or needs `sudo chmod 666 /dev/ttyACM0` |
| Motors don't move | Check Pico LED — if steady, firmware is running. Try `I2C_SCAN` via serial terminal |
| No odometry data | Check that TEL lines appear in transceiver logs |
| EKF warnings | Normal on startup until both `/pico/odom` and `/laser_odom` are publishing |
| Robot moves wrong direction | Motor wiring may be swapped — check `config.h` motor channel mapping |
