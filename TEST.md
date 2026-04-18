# AutoNexa — System Test Guide

Progressive hardware + software integration tests.
**Work through stages in order. Fix failures before proceeding.**

All commands run on the **Raspberry Pi 5** unless marked otherwise.

---

## Prerequisites

```bash
# Source ROS2 on every new terminal
source /opt/ros/jazzy/setup.bash
source ~/intelligent_parking_ws/install/setup.bash
```

---

## Stage 0 — Flash the correct Pico firmware

Run on your **dev machine** (Windows or Linux with Pico SDK):

```bash
cd pico_firmware
mkdir -p build && cd build
cmake ..
make
```

Two UF2 targets are produced:

| File | Purpose |
|------|---------|
| `autonexa_pico.uf2` | ASCII serial only — **do NOT flash for normal use** |
| `autonexa_pico_uros.uf2` | micro-ROS + Hiwonder driver — **flash this** |

```bash
# Put Pico in BOOTSEL mode (hold BOOTSEL button, plug USB cable, release button)
# Drive mounts as RPI-RP2

# Linux:
cp build/autonexa_pico_uros.uf2 /media/$USER/RPI-RP2/

# Windows (PowerShell):
Copy-Item build\autonexa_pico_uros.uf2 E:\   # replace E: with your drive letter
```

**Expected result:** Pico reboots, onboard LED blinks at ~1 Hz (heartbeat).

---

## Stage 1 — Pico visible on USB serial

```bash
# Verify the serial port exists
ls /dev/ttyACM*
# Expected: /dev/ttyACM0
```

```bash
# Raw serial sanity check — micro-ROS sends XRCE framing bytes continuously
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
time.sleep(0.5)
data = s.read(64)
print('bytes received:', len(data))
print('hex:', data.hex())
s.close()
"
```

**Pass:** `bytes received: 64` and non-zero hex output.

**Fail / fix:**
- 0 bytes → wrong UF2 flashed (ASCII-serial version). Re-flash `autonexa_pico_uros.uf2`.
- Port not found → check USB cable, try `dmesg | tail -20` to see if device enumerates.

---

## Stage 2 — micro-ROS agent connects, Pico topics appear

```bash
# Terminal 1 — start the micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

```bash
# Terminal 2 — wait for agent handshake, then list topics
sleep 4 && ros2 topic list | grep pico
```

**Expected output:**
```
/pico/control_cmd
/pico/enable
/pico/heartbeat
```

```bash
# Confirm heartbeat is publishing
ros2 topic echo /pico/heartbeat --once
# Expected: data: true  (or false — either means Pico is alive)
```

**Pass:** All three `/pico/*` topics are present and heartbeat echoes.

**Fail / fix:**
- Topics missing after 10 s → agent can't parse the serial stream. Confirm baud is `115200` on both sides (`config.h` → `UROS_SERIAL_BAUDRATE`).
- `ERROR: Failed to create participant` → another micro-ROS agent or process is holding the port. `sudo fuser -k /dev/ttyACM0`

---

## Stage 3 — Motors respond to direct commands

> **Safety:** Put the robot on a stand with wheels off the ground for this stage.

```bash
# Terminal 1 — micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

```bash
# Terminal 2 — enable motors
ros2 topic pub --once /pico/enable std_msgs/msg/Bool "{data: true}"
```

```bash
# Terminal 3 — slow forward command directly to Pico (bypasses Nav2 entirely)
ros2 topic pub /pico/control_cmd geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --rate 10
```

**Expected:** Both rear wheels spin forward slowly (~10 cm/s).

```bash
# Stop motors
ros2 topic pub --once /pico/control_cmd geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

### 3a — Test reverse

```bash
ros2 topic pub /pico/control_cmd geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    twist: {linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --rate 10
```

**Expected:** Both wheels spin in reverse.

### 3b — Test steering

```bash
# Steer left (positive angular.z = left turn in ROS convention)
ros2 topic pub /pico/control_cmd geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    twist: {linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}" \
  --rate 10
```

**Expected:** Front wheels angle left, rear wheels spin slowly forward.

```bash
# Steer right
ros2 topic pub /pico/control_cmd geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, \
    twist: {linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}}" \
  --rate 10
```

**Expected:** Front wheels angle right.

**Pass:** Forward, reverse, left turn, right turn all produce correct physical motion.

**Fail / fix:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| No motor movement, topics present | `/pico/enable` not latched | Re-publish enable: `ros2 topic pub --once /pico/enable std_msgs/msg/Bool "{data: true}"` |
| One motor silent | Hiwonder channel wiring | Check I2C wiring; swap `MOTOR_LEFT_CHANNEL` / `MOTOR_RIGHT_CHANNEL` in `config.h` |
| Both motors spin but backwards | Motor polarity flipped | Negate `v_l` and `v_r` in `pico_firmware/src/ackermann.c` |
| One motor runs backwards | Single channel polarity | Negate that channel's speed before calling `hiwonder_set_motor_speed()` |
| Steering goes wrong direction | Servo polarity | Flip sign of `steering_angle` in `servo.c` |
| Motors stop after ~200 ms | Watchdog timeout | Publish at `--rate 10` (≥ 5 Hz keeps the 200 ms watchdog alive) |

---

## Stage 4 — cmd_vel safety chain flows end-to-end

```bash
# Terminal 1 — micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# Terminal 2 — bridge stack only (no LiDAR, no SLAM, no Nav2)
ros2 launch parking_system rpi5_pico_bridge.launch.py
```

```bash
# Terminal 3 — publish to /cmd_vel (same topic Nav2 uses)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

```bash
# Terminal 4 — verify the full chain is flowing
ros2 topic hz /cmd_vel           # ~10 Hz  (your publisher)
ros2 topic hz /cmd_vel_safe      # ~10 Hz  (after velocity_smoother + collision_monitor)
ros2 topic hz /pico/control_cmd  # ~30 Hz  (bridge output rate)
```

**Pass:** All three topics show expected Hz and wheels spin.

**Fail / fix:**
- `/cmd_vel_safe` silent → collision_monitor is blocking. LiDAR not running in this stage, so check `nav2_navigation_params.yaml` — collision_monitor may need `/scan`. Run Stage 6 first if this is the case.
- `/pico/control_cmd` silent but `/cmd_vel_safe` active → cmd_vel_to_pico_bridge crashed. Check `ros2 node list | grep pico_bridge`.

---

## Stage 5 — E-STOP works

With Stage 4 running and motors spinning:

```bash
# Engage E-STOP via service
ros2 service call /pico/estop std_srvs/srv/SetBool "{data: true}"
```

**Expected:** Motors stop immediately. `/pico/control_cmd` stops producing motion.

```bash
# Clear E-STOP
ros2 service call /pico/estop std_srvs/srv/SetBool "{data: false}"
```

**Expected:** Normal operation resumes — motors respond to `/cmd_vel` again.

**Pass:** Hard stop on estop=true, resumes on estop=false.

---

## Stage 6 — LiDAR scan and SLAM map

```bash
# Start full stack without Pico bridge (hardware not needed for this test)
ros2 launch parking_system nav2_live_slam.launch.py \
  use_pico_bridge:=false \
  use_rviz:=true
```

```bash
# In a second terminal — scan quality diagnostic
ros2 run parking_system diagnose_scan_quality.py
```

**Expected scan output:**
```
Scan rate:    ~10 Hz
Range count:  > 200 valid points per scan
Min range:    > 0.10 m
Max range:    < 12.0 m
```

```bash
# TF tree must be intact
ros2 run parking_system diagnose_tf_tree.py
```

**Expected TF chain:**
```
map → odom → base_link → laser_link
```

In **RViz:**
1. Add → By topic → `/scan` → LaserScan → should show rotating dots around robot
2. Add → By topic → `/map` → Map → slowly fills in as you push the robot around by hand

**Pass:** `/scan` at ≥ 8 Hz, TF chain intact, map builds when robot moves.

**Fail / fix:**
- No `/scan` → LiDAR not detected. Check `ls /dev/ttyUSB*`. Adjust `serial_port` arg: `ros2 launch parking_system nav2_live_slam.launch.py serial_port:=/dev/ttyUSB1`
- `map → odom` transform missing → SLAM Toolbox not running. Check `ros2 node list | grep slam`
- `odom → base_link` missing → laser_scan_matcher not running or LiDAR producing bad scans

---

## Stage 7 — Mobile app joystick drives the robot

```bash
# Start full stack
ros2 launch parking_system nav2_live_slam.launch.py \
  use_pico_bridge:=true \
  use_rviz:=false
```

```bash
# Confirm Flask bridge is up (from RPi5)
curl http://localhost:5000/api/status
# Expected: JSON with pose, scan, map fields
```

**On the phone:**
1. Open AutoNexa app
2. **Settings** tab → Server: `<RPi5_IP>:5000` → tap **Connect**
3. Connection indicator turns **green**
4. **Control** tab → set speed limit to **30%**
5. Drag joystick slightly forward

**Checklist:**
- [ ] App connects (green dot in header)
- [ ] Latency badge < 100 ms
- [ ] Joystick moves forward → robot moves forward
- [ ] Joystick left/right → robot steers
- [ ] Release joystick → robot stops within 500 ms (watchdog)
- [ ] E-STOP button (red) → robot stops immediately
- [ ] E-STOP clear (orange GO button) → robot responds to joystick again

**Pass:** Full manual control from the app.

**Fail / fix:**
- App can't connect → check firewall: `sudo ufw allow 5000`
- Connects but joystick does nothing → check Flask bridge launched: `ros2 node list | grep mobile_bridge`
- E-STOP won't clear → `/pico/estop` service not available. Confirm micro-ROS agent running.

---

## Stage 8 — Autonomous navigation (RViz goal)

```bash
ros2 launch parking_system nav2_live_slam.launch.py use_pico_bridge:=true use_rviz:=true
```

1. **Build an initial map:** push the robot around by hand for ~30–60 seconds until the room outline appears in RViz
2. Place robot on the floor in a known position
3. In RViz: click **"2D Nav Goal"** in the toolbar
4. Click and drag on the map to set goal position + orientation

**Expected:**
- Green path appears in RViz from robot to goal
- Robot drives autonomously along the path
- Robot decelerates and stops at the goal (within ~0.2 m)

**Pass:** Robot navigates to goal without hitting obstacles.

**Fail / fix:**
- No path planned → Nav2 planner failed. Check costmap: `ros2 topic echo /global_costmap/costmap_updates --once`
- Path planned but robot doesn't move → controller or bridge issue. Check `ros2 topic hz /cmd_vel`
- Robot oscillates / overshoots → tune DWB parameters in `config/nav2_navigation_params.yaml` (`max_vel_x`, `decel_lim_x`)

---

## Stage 9 — App sends Nav2 goals (tap-to-navigate)

```bash
ros2 launch parking_system nav2_live_slam.launch.py use_pico_bridge:=true use_rviz:=false
```

**On the phone:**
1. **Map** tab → wait for SLAM map to appear (5–10 s after launch)
2. Tap a clear area on the map
3. Confirm the Nav Goal dialog (X, Y pre-filled from tap coordinates)
4. Robot drives to the tapped location

**Also test cancel:**
1. Send a nav goal to a far point
2. While robot is moving, tap the **E-STOP** button → robot stops, Nav2 cancels plan
3. Tap GO → joystick control resumes

**Pass:** Tap-to-navigate works. E-STOP cancels autonomous navigation.

---

## Stage 10 — Parking mission (ArUco markers)

```bash
ros2 launch parking_system nav2_live_slam.launch.py use_pico_bridge:=true use_rviz:=false
```

**On the phone:**
1. **Parking** tab → Parking Spots sub-tab
2. Hold printed ArUco marker (DICT_4X4_50, IDs 0–9) in front of camera
3. Marker appears in list with ID, distance, bearing, status = **Live**
4. Tap **Navigate** on a marker → robot drives toward the marker's position

**Pass:** Markers detected, robot navigates toward them.

---

## Full system diagnostics (run any time)

```bash
# Control chain health (checks all topics are alive)
ros2 run parking_system diagnose_control_chain.py

# Strict mode (checks no duplicate publishers)
ros2 run parking_system diagnose_control_chain.py --ros-args \
  -p expect_pico_bridge:=true \
  -p require_single_pico_publisher:=true

# LiDAR scan quality
ros2 run parking_system diagnose_scan_quality.py

# Localization status (AMCL / odom / SLAM)
ros2 run parking_system diagnose_localization.py

# TF tree integrity
ros2 run parking_system diagnose_tf_tree.py

# Print current robot pose
ros2 run parking_system print_robot_position.py

# Record 30-second bag of the full control chain for offline analysis
ros2 run parking_system record_control_chain_bag.py
```

---

## Recommended first-run order (summary)

```
Stage 0  →  Flash autonexa_pico_uros.uf2
Stage 1  →  Confirm USB serial bytes from Pico
Stage 2  →  Confirm micro-ROS topics appear
Stage 3  →  Motors spin with direct TwistStamped commands
Stage 4  →  /cmd_vel → /cmd_vel_safe → /pico/control_cmd chain flows
Stage 5  →  E-STOP latches and clears
Stage 6  →  LiDAR scan OK, SLAM map builds
Stage 7  →  App joystick drives robot
Stage 8  →  Autonomous navigation to RViz goal
Stage 9  →  App tap-to-navigate + E-STOP cancel
Stage 10 →  ArUco marker detection + parking approach
```

Each stage gate-keeps the next. If Stage 3 fails there is no point testing Stage 7.
