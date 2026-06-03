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

# Nav2 controllers (both required for the switchable controller:=mppi|rpp arg).
# MPPI is the default; RPP is the fallback.
sudo apt install ros-jazzy-nav2-mppi-controller \
                 ros-jazzy-nav2-regulated-pure-pursuit-controller
```

---

## Stage 0 — Build and flash Pico firmware

```bash
cd pico_firmware && mkdir -p build && cd build && cmake .. && make
```

The build produces two UF2s:

| UF2 | Use |
|-----|-----|
| `autonexa_pico_uros.uf2` | **Production / Nav2 integration.** micro-ROS client over USB CDC. Required for stages 2 onward. |
| `autonexa_pico.uf2` | **Bench-test CLI.** ASCII line protocol; drive from `python3 test/pico_gui.py`. Independent of the rest of the stages. |

Flash by holding **BOOTSEL**, plugging USB to mount the `RPI-RP2` drive, then:

```bash
cp build/autonexa_pico_uros.uf2 /media/$USER/RPI-RP2/
```

The Pico re-enumerates after ~1 s as `/dev/ttyACM0`.

**Pass:** Pico LED blinks at ~1 Hz steady. (5 Hz blink means a latched E-STOP — clear it via Stage 5 once the bridge is up.)

---

## Stage 1 — Pico visible on USB serial

```bash
ls /dev/ttyACM*
# Expected: /dev/ttyACM0
```

```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyACM0', 115200, timeout=2); time.sleep(0.5)
data = s.read(64); print('bytes received:', len(data), 'hex:', data.hex()); s.close()"
```

**Pass:** `bytes received: 64` of non-zero XRCE-DDS framing.

**Fail / fix:**
- 0 bytes → wrong UF2 flashed (likely the CLI build) or firmware crashed. Re-flash `autonexa_pico_uros.uf2`.
- Port not found → check USB cable; `dmesg | tail -20`.
- Permission denied → `sudo usermod -aG dialout $USER && newgrp dialout`.

---

## Stage 2 — Bridge comes up, `/pico/*` topics appear

```bash
ros2 launch parking_system rpi5_pico_bridge.launch.py
```

This launches `micro_ros_agent` on `/dev/ttyACM0` plus `cmd_vel_to_pico_bridge.py`.

```bash
# In a new terminal — wait for startup, then list topics
sleep 4 && ros2 topic list | grep pico
```

**Expected output:**
```
/pico/control_cmd
/pico/enable
/pico/heartbeat
/pico/odom
/pico/joint_feedback
```

```bash
# Confirm telemetry is flowing
ros2 topic echo /pico/heartbeat --once   # data: true when Pico serial is up
ros2 topic echo /pico/odom --once        # position / twist fields present
ros2 topic hz   /pico/heartbeat          # ~5 Hz
ros2 topic hz   /pico/odom               # ~20 Hz
```

**Pass:** All five `/pico/*` topics are present and `/pico/heartbeat` echoes `data: true`.

**Fail / fix:**
- Topics missing → bridge crashed. Check launch log for `Another cmd_vel_to_pico_bridge instance already running` (lock file) or `open /dev/ttyACM0 failed` (port busy). `sudo fuser -k /dev/ttyACM0` to clear a stale holder.
- `ERROR: Failed to create participant` → another `micro_ros_agent` is holding the port.

---

## Stage 3 — Motors respond to direct commands

> **Safety:** Put the robot on a stand with wheels off the ground for this stage.

Start the bridge from Stage 2 and drive commands through `/cmd_vel`. The bridge handles enable + accel limits + serial translation.

```bash
# Terminal 1 — bridge
ros2 launch parking_system rpi5_pico_bridge.launch.py
```

```bash
# Terminal 2 — slow forward command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

**Expected:** Both rear wheels spin forward slowly (~10 cm/s).

> **Bench-only alternative (no bridge, no Nav2):** flash `autonexa_pico.uf2` instead and use `python3 test/pico_gui.py` for hold-to-drive WASD + live telemetry. Independent of Stages 2–10.

### 3a — Test reverse

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
```

**Expected:** Both wheels spin in reverse.

### 3b — Test steering

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --rate 10
```

**Expected:** Steering servo angles left, rear wheels spin slowly forward.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}" --rate 10
```

**Expected:** Steering angles right.

**Pass:** Forward, reverse, left turn, right turn all produce correct physical motion.

**Fail / fix:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| No motor movement, topics present | Bridge timed out; publisher below 5 Hz | Publish at `--rate 10` (≥ 5 Hz keeps the 200 ms watchdog alive) |
| One motor silent | Hiwonder channel wiring | Check I2C wiring; verify M1/M2 leads |
| Both motors spin but backwards | Motor polarity flipped | Swap motor leads or negate `v_l`/`v_r` in `pico_firmware/src/ackermann.c` |
| Steering goes wrong direction | Servo polarity | Flip sign in `pico_firmware/src/servo.c` |

### 3c — Verify proportional speed + calibrate the kick-start (speed-floor fix)

The firmware now uses an **encoder-aware kick-start** instead of the old permanent
60% PWM floor (`pico_firmware/include/config.h`: `MOTOR_KICK_PCT` / `MOTOR_KICK_MS` /
`MOTOR_MIN_RUN_PCT`). This is the fix for "Nav2 sends 10, the Pico does 20" — the old
floor snapped every slow command up to 60% duty, so the robot could not slow for a
wall or creep onto a goal. Confirm low commands now produce *slow* motion, not the
old all-or-nothing lurch.

**Bench (wheels off ground), CLI build `autonexa_pico.uf2` + `python3 test/pico_gui.py`:**
- `SPEED 5`, `SPEED 10`, `SPEED 20`, `SPEED 30` must produce visibly *increasing*
  speeds — each start shows a brief kick, then settles. Read the `TEL` `vx` field: it
  should rise monotonically with SPEED, **not** sit at one fixed value for 5–17.
- `SPEED 0` coasts to a stop. `ENC_READ` now returns live encoder counts.

**On-the-ground calibration of `vel_to_speed_scale` (open-loop m/s mapping):**
1. Clear flat floor. Run the bridge (Stage 2) so `/pico/odom` is published.
2. Command a steady speed and read the measured speed back:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}}" --rate 10 &
   ros2 topic echo --once /pico/odom   # twist.twist.linear.x = encoder-measured vx
   ```
   Repeat for ~0.06 / 0.10 / 0.15 m/s.
3. If commanded vx and measured vx disagree, scale the `nav2_pico_bridge`
   `vel_to_speed_scale` param by `commanded / measured` (default 100 assumes
   SPEED 30 = 0.30 m/s). It is a TUNABLE (now PC-authoritative), so set it in the PC
   config / launch — a value typed in the app won't survive a relaunch by design.

**Tuning the kick-start (`config.h`, requires rebuild + reflash):**
- Low command **lurches** (wheel starts, stalls, re-kicks): raise `MOTOR_MIN_RUN_PCT`
  until a rolling wheel keeps rolling at that duty.
- Wheel **won't start** at low commands: raise `MOTOR_KICK_PCT` or `MOTOR_KICK_MS`.
- `MOTOR_ENC_MOVING_EDGES` is the "is it rolling?" threshold (encoder edges/tick @ 50 Hz).

**Pass:** measured `/pico/odom` vx increases monotonically with commanded vx, and the
robot can hold a slow steady creep with no 60%-duty lurch.

---

## Stage 4 — cmd_vel safety chain flows end-to-end

```bash
# Terminal 1 — bridge stack (no LiDAR, no SLAM, no Nav2)
ros2 launch parking_system rpi5_pico_bridge.launch.py

# Terminal 2 — publish to /cmd_vel (same topic Nav2 uses)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  --rate 10
```

```bash
# Terminal 3 — verify the full chain is flowing
ros2 topic hz /cmd_vel           # ~10 Hz  (your publisher)
ros2 topic hz /cmd_vel_safe      # ~10 Hz  (after velocity_smoother + collision_monitor)
ros2 topic hz /pico/control_cmd  # ~30 Hz  (bridge output)
ros2 topic hz /pico/odom         # ~20 Hz  (telemetry back from Pico)

# Or run the packaged diagnostic (covers all of the above + single-publisher guard):
ros2 run parking_system diagnose_control_chain.py --ros-args \
  -p expect_pico_bridge:=true \
  -p require_single_pico_publisher:=true \
  -p require_flow:=true -p window_s:=12.0
```

**Pass:** `diagnose_control_chain.py` exits 0 and wheels spin.

**Fail / fix:**
- `/cmd_vel_safe` silent → collision_monitor / velocity_smoother not running in bridge-only launch. Bypass by publishing to `/cmd_vel_safe` directly, or launch `nav2_live_slam.launch.py`.
- `/pico/control_cmd` silent but `/cmd_vel_safe` active → bridge crashed. Check `ros2 node list | grep bridge` and the launch log.

### Stage 4b — Single-publisher guard

```bash
# Leave the bridge running. In another terminal:
ros2 run parking_system cmd_vel_to_pico_bridge.py
```

**Expected:** second process exits immediately with `Another cmd_vel_to_pico_bridge instance is already running (lock: /tmp/cmd_vel_to_pico_bridge.lock)`.

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

> **Operational tip (avoids the SLAM symmetry/ambiguity problem):** start each
> mapping session from roughly the **same physical start pose**, and when later
> re-launching against a saved map, seed the initial pose near where mapping
> began. Starting from a consistent origin removes the left/right-flip ambiguity
> a featureless symmetric room can otherwise produce. In localization mode you
> can also enable periodic re-settling via `POST /api/relocalize_auto
> {"enabled": true, "interval_s": 20}` to counter slow drift.

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
# Default controller is MPPI (obstacle-aware). Add controller:=rpp to fall back
# to Regulated Pure Pursuit (no rebuild) if MPPI can't hold rate on the Pi.
ros2 launch parking_system nav2_live_slam.launch.py use_pico_bridge:=true use_rviz:=true
ros2 launch parking_system nav2_live_slam.launch.py controller:=rpp use_pico_bridge:=true use_rviz:=true
```

> **MPPI benchmark gate (run once on the Pi 5 before trusting MPPI tuning).**
> MPPI is the heaviest Nav2 node. While a goal is executing, measure:
> `pidstat -p $(pgrep -f controller_server) 1` (CPU) and `ros2 topic hz /cmd_vel`
> (rate). **Accept:** `/cmd_vel` ≥ 8 Hz sustained, no multi-second dropouts,
> controller_server < ~120% CPU, lifecycle stays `active`. **Back-off ladder if it
> fails (edit `config/controller_mppi.yaml`):** `batch_size` 1000→800→600 →
> `time_steps` 40→30→24 → `controller_frequency`+`model_dt` 10/0.1→8/0.125 →
> `CostCritic.consider_footprint` true→false. If `configure()` trips the lifecycle
> bond at startup, raise `bond_timeout` 10.0→20.0 in the launch file. Otherwise use
> `controller:=rpp`.

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
- Path planned but robot doesn't move → controller or bridge issue. Check `ros2 topic hz /cmd_vel`. Under MPPI, also confirm the rate meets the benchmark gate above (a starved controller_server publishes slowly/erratically).
- Robot oscillates / overshoots → **MPPI:** tune critic weights in `config/controller_mppi.yaml` (raise `PathAlignCritic.cost_weight` to hug the path, lower `vx_max`); **RPP** (`controller:=rpp`): tune `FollowPath.lookahead_dist` / `desired_linear_vel` in `config/nav2_navigation_params.yaml`.
- Robot won't drive close past a wall / halts → MPPI's `CostCritic.cost_weight` too high (lower it), or you're on `controller:=rpp` where collision behavior differs. Note: collision_monitor is disabled in the AMCL launch by design.

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

**Also verify the app controls work under MPPI:**
1. Settings → Nav2 Max Speed slider → confirm it changes speed (under MPPI it sets
   `FollowPath.vx_max`, under RPP `FollowPath.desired_linear_vel`; velocity_smoother
   tracks in lockstep and the value persists across relaunch).
2. Diagnostics → Open param tuner → Nav2 Controller (MPPI/RPP) → confirm the MPPI quick
   params show (vx_max, batch_size, critic weights…) and a live critic-weight edit takes
   effect. Launch with `controller:=rpp` and confirm the RPP params show instead.

**Pass:** Tap-to-navigate works. E-STOP cancels autonomous navigation. App speed slider +
param tuner operate on whichever controller is active.

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

## Stage 11 — Stop & reroute around NEW obstacles (AMCL mode)

Verifies the map/goal-aware obstacle discriminator: the car still parks precisely
against the intended wall/slot, but a NEW (unaccumulated) obstacle placed in its
path makes it STOP and reroute instead of driving through it.

```bash
# Saved-map / AMCL mode (the mode the precise park is tuned for).
# collision_monitor stop polygons are now ENABLED in this launch.
ros2 launch parking_system nav2_amcl_navigation.launch.py \
  map_yaml:=$HOME/.autonexa/maps/garage_<ts>.yaml \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

Setup: open floor, one wall/parking target, one ~20 cm box (tall enough for the C1
to see — above the costmap `min_height: 0.0`). Useful watchers:

```bash
ros2 topic echo /collision_monitor_state          # shows stop events
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
# (~0.22 normal; 0.015 means the near-wall BYPASS is engaged)
# Bridge log lines to watch for: "novel obstacle at <d> m — releasing bypass"
```

**11a — Regression gate (RUN FIRST, must pass before 11b):**
Park against the wall/slot with NO box in the path.
**Pass:** final position error **< 3 cm** (the EXPECTED wall/target still engages
the bypass, so the polygons never freeze the park).
**If it regresses:** raise `_goal_vicinity_m` (0.40 → 0.50) in `ros2_mobile_bridge.py`
and retry; if still short, fall back to the discriminator-commanded-stop variant and
leave the AMCL polygons disabled. **Do not proceed to 11b until 11a passes.**

**11b — New obstacle in the path:**
Send a cross-room goal; while the car is driving in open space, place the box
0.6–1.0 m ahead of it. **Pass, in order:**
1. Car **STOPS** > 0.12 m from the box (`/collision_monitor_state` shows a stop).
2. Bridge log shows the bypass released / never engaged as the box entered range.
3. A fresh `/plan` that routes AROUND the box appears within ~0.3–0.5 s — the active
   re-plan (bridge log `novel obstacle re-plan: re-issuing goal …`), not only at the
   next 2 Hz BT tick. A lethal cell is marked at the box in the global costmap.
4. Car reaches the goal **without contact**.
**Fail check:** if it drives through the box, confirm `inflation_radius` ≈ 0.22 (NOT
0.015 — 0.015 means the bypass is still wrongly engaged).

**11c — Obstacle between robot and the target wall:**
Place the box between the chassis and the parking slot. **Pass:** car stops + reroutes,
then completes the **< 3 cm** park once the direct path is clear (NOVEL dominates the
nearby EXPECTED wall).

**11d — Spurious-point immunity:**
Wave a thin object briefly across the LiDAR in open space. **Pass:** no force-release
and no spurious full stop (a momentary FootprintApproach slow-down is acceptable).

**11e — SLAM accretion (optional, live-SLAM only):**
Leave the box in place several seconds. **Pass:** it enters `/map`; later plans treat
it as a permanent wall; no oscillation in the bypass log.

**11f — Dead-ahead escape (reverse + curve):**
Place the box directly in front, blocking the straight path to the goal but with room
to the sides. **Pass:** car stops, then the new `/plan` includes a **reverse** segment
(Reeds-Shepp) — the car backs up to gain room, curves around, and reaches the goal.
collision_monitor's direction-aware reverse polygon permits the back-up (free space
behind). No forced BackUp nudge is used — this relies on the planner.

**11g — Fully blocked (multipoint decompose):**
Box + walls leave no path around. **Pass:** after the recovery ladder the goal ABORTs
and the bridge logs `ABORT after novel obstacle — decomposing via waypoint` →
`multipoint … staging via waypoint`; the car attempts an alternative approach via an
intermediate open-space waypoint. If no path exists it gives up cleanly at the decompose
cap **without driving into the box**. (Decompose needs an app-issued goal so `_mp_final_goal`
is set; RViz `/goal_pose` goals don't trigger it.)

**11h — Park staging fallback (two-leg strategy):**
Issue a **park** goal whose staging pose (0.55 m back from the slot) is hard to reach
(e.g. start the car so the far staging pose is boxed, but a closer one is open).
**Pass, in order:**
1. Bridge logs `staging candidate 0 … d=0.55`. If that leg ABORTs, it logs
   `staging candidate 0 unreachable; re-staging to candidate 1 d=0.40`, then `… d=0.28`.
2. The car re-stages **closer** to the slot rather than abandoning straight-in immediately.
3. Only after **all** candidates are exhausted does it log `staging exhausted … falling
   back to standard goal` and drive directly to the slot.
4. Park substate (`/api/status` → park stage) shows `approach` across retries, then
   briefly `blocked`, then the normal park completes (or the direct goal does).
A staging candidate is **valid** only if BOTH (i) its approach→slot corridor is clear in
the static map (`_corridor_clear`) AND (ii) **no unaccumulated obstacle** sits within
`park_staging_clear_diam_m` (default 0.60 m diameter / 0.30 m radius, ≥`_novel_min_points`
returns on free map cells) of the staging point (`_staging_blocked_by_novel`). So the car
will not stage onto a freshly-placed obstacle that isn't in the map yet — it skips to the
next (closer) candidate, or falls back to a plain slot goal if none is valid.

**Fallback-from-leg-strategy contract** — it abandons the two-leg strategy and issues a
plain slot goal when ANY of: (a) no staging candidate is valid at issue (corridor blocked
OR novel obstacle within the staging clearance — Check #1), (b) every staging candidate's
leg-1 plan ABORTed, (c) the push-in leg (leg 2) aborts or Check #2 finds the corridor
blocked before push-in, (d) it can't enter AUTO. The fallback never does worse than the
pre-existing single-goal behavior.

**11i — Obstacle on the staging point:** place a box right where the 0.55 m staging pose
would be (but leave a closer staging spot open). **Pass:** bridge logs that candidate 0 is
skipped (novel within staging clearance) and stages to candidate 1/2 instead, or falls back
to a plain slot goal if all staging spots are fouled — it never drives onto the box.

**Pass:** 11a + 11b + 11c all green — precise parking preserved AND new obstacles
trigger stop + reroute. 11f/11g green — escape + blocked-route recovery work.
11h green — staging retries then clean fallback.

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
Stage 0  →  Build + flash autonexa_pico_uros.uf2
Stage 1  →  Confirm USB serial bytes from Pico
Stage 2  →  Confirm micro-ROS topics appear
Stage 3  →  Motors spin with direct cmd_vel commands
Stage 4  →  /cmd_vel → /cmd_vel_safe → /pico/control_cmd chain flows
Stage 5  →  E-STOP latches and clears
Stage 6  →  LiDAR scan OK, SLAM map builds
Stage 7  →  App joystick drives robot
Stage 8  →  Autonomous navigation to RViz goal
Stage 9  →  App tap-to-navigate + E-STOP cancel
Stage 10 →  ArUco marker detection + parking approach
Stage 11 →  Stop & reroute around NEW obstacles (AMCL); precise park preserved
```

Each stage gate-keeps the next. If Stage 3 fails there is no point testing Stage 7.
