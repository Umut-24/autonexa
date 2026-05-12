# AutoNexa — Critical Design Review Report
## Sections 2, 3 & 4: Embedded, Perception, and Navigation Subsystems

---

# Section 2 — Embedded Subsystem

## 2.1 Overview

The embedded subsystem runs on a Raspberry Pi Pico (RP2040) and sits at the bottom of the software stack. Its job is simple in principle: take a velocity command from ROS2 and turn it into physical motion. In practice it also owns all hardware-level safety — watchdog, E-STOP, and actuator gating — independently of anything running on the Raspberry Pi 5.

The firmware is written in C using the Pico SDK with no RTOS. A single 50 Hz hardware-timer loop drives everything: read encoders, update odometry, check safety, apply actuator commands, handle comms. The build system produces two targets:

| Build Target | Purpose |
|---|---|
| `autonexa_pico` | ASCII serial CLI — used for bench testing without ROS2 |
| `autonexa_pico_uros` | micro-ROS — production mode; receives commands as ROS2 topics |

Both share the same control core. The CLI build was developed first to validate kinematics and hardware drivers in isolation before adding the micro-ROS layer.

---

## 2.2 Hardware Interfaces

The Pico bridges upward to the RPi5 over USB serial and downward to actuators over PWM and I2C.

| Interface | Protocol | Connection | Device |
|---|---|---|---|
| Steering servo | PWM, 50 Hz | GPIO 12 | LD-1501MG (±30°, 500–2500 µs) |
| Motor driver | I2C0, 100 kHz | SDA=GPIO0, SCL=GPIO1 | Hiwonder YX-4055AM (addr 0x34) |
| Left rear motor | — | Driver ch. M2 | JGB37-520R30, 12V DC |
| Right rear motor | — | Driver ch. M4 | JGB37-520R30, 12V DC |
| Host comms | USB CDC | — | Raspberry Pi 5 (115,200 baud) |

---

## 2.3 Vehicle Parameters

All constants live in `pico_firmware/include/config.h`.

| Parameter | Value |
|---|---|
| Wheelbase | 0.25 m |
| Track width | 0.20 m |
| Wheel radius | 0.033 m |
| Max steering angle | ±30° (±0.5236 rad) |
| Motor gear ratio | 1:30 |
| Encoder edges per wheel rev | 1320 (11 × 4 × 30) |
| Control loop rate | 50 Hz (20 ms) |
| Command timeout | 200 ms |

---

## 2.4 Control Loop

Every 20 ms the loop executes this fixed sequence:

```
[50 Hz timer tick]
        |
        v
  Read encoders (I2C from motor driver)
        |
        v
  Forward kinematics -> update odometry (x, y, yaw)
        |
        v
  Safety check (timeout counter, E-STOP state, LED update)
        |
        v
  Apply actuator commands  <-- gated by safety_is_ok()
    - Steering angle -> servo PWM
    - Wheel speeds   -> I2C motor driver
        |
        v
  Comms (one of):
    micro-ROS: spin subscribers, publish odom @ 20 Hz / joints @ 10 Hz
    Serial CLI: parse incoming command, print telemetry @ 10 Hz
```

---

## 2.5 Ackermann Kinematics

ROS2 sends body-frame velocity commands `(vx, wz)`. The firmware converts these to physical actuator setpoints using Ackermann geometry:

- **Inverse kinematics** (command → actuators): computes the required steering angle and the left/right differential wheel speeds for a given `(vx, wz)` pair. Straight-line motion is handled as a degenerate case where both wheel speeds equal `vx`.
- **Forward kinematics** (encoders → odometry): integrates encoder ticks each cycle into an `(x, y, yaw)` pose estimate using the bicycle model. This is published on `/pico/odom` at 20 Hz.

The Pico odometry is currently not fused into the main TF chain — the RPi5 uses laser scan matcher as the primary odom source. It becomes the fallback once the EKF is active.

---

## 2.6 Safety System

Safety is enforced at the firmware level before any actuator write. Two independent conditions exist:

| State | Cause | Motors | Steering | LED | Clears When |
|---|---|---|---|---|---|
| Normal | — | Running | Active | 1 Hz blink | — |
| Timed out | No command for 200 ms | Stopped | Centred | 1 Hz blink | Next valid command arrives |
| E-STOP | Explicit trigger | Stopped | Centred | 5 Hz blink | Explicit clear only |

The E-STOP latches — it does not self-recover. This is intentional. Every actuator write is gated through `safety_is_ok()`; nothing can reach the motors without passing this check.

---

## 2.7 micro-ROS Topics

In production mode, the Pico runs as a micro-ROS node (`pico_controller`, domain 0) communicating over XRCE-DDS on USB serial. The RPi5 runs the micro-ROS agent that bridges these topics into the full ROS2 graph. On boot, the Pico waits in a ping loop until the agent responds before starting the control loop.

| Topic | Direction | Type | Rate |
|---|---|---|---|
| `/pico/control_cmd` | RPi5 → Pico | `TwistStamped` | 30 Hz |
| `/pico/enable` | RPi5 → Pico | `Bool` | 30 Hz |
| `/pico/heartbeat` | RPi5 → Pico | `Bool` | 5 Hz |
| `/pico/odom` | Pico → RPi5 | `Odometry` | 20 Hz |
| `/pico/joint_feedback` | Pico → RPi5 | `JointState` | 10 Hz |
| `/pico/estop` (service) | RPi5 → Pico | `SetBool` | on demand |

---

## 2.8 Known Limitations and Open Items

| Item | Severity | Notes |
|---|---|---|
| Pico odometry not yet fused | Medium | Published but not consumed by Nav2; will be integrated when EKF goes active |
| USB CDC sync loss on long sessions | Low | Pico watchdog catches it and stops motors; requires agent relaunch to recover |
| No motor stall / overcurrent detection | Low | Hardware addition needed; out of current scope |

---
---

# Section 3 — Perception Subsystem

## 3.1 Overview

The perception subsystem is responsible for two things: building a consistent geometric model of the environment in real time, and providing the rest of the system with a reliable pose estimate so the robot knows where it is within that model. Everything downstream — path planning, obstacle avoidance, velocity commands — depends on what perception produces. If the map is noisy or the odometry drifts, the navigation stack will not recover gracefully.

AutoNexa uses a **LiDAR-first architecture**. The 2D rotating LiDAR is the single source of truth for both mapping (via SLAM Toolbox) and incremental dead-reckoning (via laser scan matching). Wheel encoders exist in the firmware and publish raw odometry, but they are not yet fused into the ROS2 odometry chain — that integration is staged for when the IMU arrives.

The system is designed from the start to support two operational modes:

| Mode | Active Sensors | Odometry Source | Map Source | Use Case |
|------|---------------|-----------------|------------|----------|
| **A — LiDAR Only** | SLAMTEC C1 | laser_scan_matcher | SLAM Toolbox | Current default; fully functional |
| **B — LiDAR + Camera** | SLAMTEC C1 + IMX219 | EKF (LiDAR + IMU) | SLAM Toolbox | Planned; camera adds ArUco marker detection for precise parking slot identification |

Mode B is not yet active. The camera link is already defined in the URDF, the mobile app already decodes ArUco markers, and the ROS2 bridge node exists — the remaining work is wiring the camera feed into a ROS2 image publisher and feeding its pose output into the navigation goal pipeline. Switching between modes will be controlled by a launch argument, not a code change.

---

## 3.2 Sensor Hardware — SLAMTEC C1 LiDAR

The primary sensor is a SLAMTEC C1, a compact 2D rotating LiDAR that covers a full 360° field of view. It communicates with the Raspberry Pi 5 over USB serial using the SLAMTEC serial protocol, and the `sllidar_ros2` driver handles everything from transport to topic publication.

**Physical mounting** is defined in `src/parking_system/urdf/robot.urdf`:
- Forward offset from `base_link`: **+150 mm**
- Height above `base_link` origin: **+120 mm**
- Mounted axially so the scan plane is horizontal

| Property | Value |
|----------|-------|
| Sensor model | SLAMTEC C1 |
| Measurement principle | ToF / triangulation (rotating 2D) |
| Field of view | 360° |
| Operational range (configured) | 0.05 m – 4.0 m |
| Scan frequency | ~10 Hz (C1 default) |
| ROS2 driver | `sllidar_ros2` |
| Published topic | `/scan` (`sensor_msgs/LaserScan`) |
| Frame ID | `laser_link` |
| Serial interface | USB CDC at **460,800 baud** |
| Default device node | `/dev/ttyUSB0` |
| Recommended device node | `/dev/serial/by-id/<lidar-id>` |

**Serial port stability note.** In testing, we encountered a specific failure mode: if the `sllidar_node` is killed without a clean shutdown, the OS does not always release the file descriptor on `/dev/ttyUSB0`. The next launch attempt then hits an `SL_RESULT_OPERATION_TIMEOUT` during initialization. The fix is straightforward — kill any leftover processes and relaunch — but it's worth noting that using the by-id path (`/dev/serial/by-id/`) makes this easier to diagnose because the path is deterministic regardless of USB enumeration order. The launch file already accepts a `serial_port` argument to override the default; for deployment the by-id path should be hardcoded.

---

## 3.3 Raw Scan Processing — Filter Chain

The raw `/scan` topic coming off the driver is not used directly by SLAM or the costmaps. It goes through a four-stage `laser_filters` chain first, configured in `config/scan_filter.yaml`. The chain runs synchronously — every incoming scan goes through all four stages before anything downstream sees it.

```
/scan (raw, from sllidar_node)
         │
         ▼
 ┌───────────────────────────────┐
 │  Stage 1: Range Filter        │
 │  min: 0.05 m  max: 4.0 m     │
 └──────────────┬────────────────┘
                │
                ▼
 ┌───────────────────────────────┐
 │  Stage 2: Shadow Filter       │
 │  angle: 10°–170°, nbrs: 20   │
 └──────────────┬────────────────┘
                │
                ▼
 ┌───────────────────────────────┐
 │  Stage 3: Median Filter       │
 │  observations: 5, queue: 10  │
 └──────────────┬────────────────┘
                │
                ▼
 ┌───────────────────────────────┐
 │  Stage 4: Outlier Filter      │
 │  threshold: 0.5 m, window: 5 │
 └──────────────┬────────────────┘
                │
                ▼
    /scan (filtered, used by SLAM,
     scan matcher, and costmaps)
```

| Stage | Filter Type | Key Parameters | Why It's There |
|-------|-------------|----------------|----------------|
| 1 | `LaserScanRangeFilter` | min: 0.05 m, max: 4.0 m | The C1 produces garbage returns below ~5 cm (too close to the optical center) and beyond its reliable range. These points confuse SLAM and mark walls as obstacles. |
| 2 | `ScanShadowsFilter` | angle range: 10°–170°, neighbors: 20 | When a scan ray grazes the edge of an object, the return point lands on the background surface but appears to float in space. Shadow filtering removes these grazing-angle artifacts by checking whether neighboring beams agree. |
| 3 | `MedianFilter` | observations: 5, queue: 10 | Applies a temporal median over the last 5 scans per beam angle. This kills flickering returns on reflective surfaces and glass — something the C1 is susceptible to in indoor environments. |
| 4 | `LaserScanOutlierFilter` | threshold: 0.5 m, window: 5 | Removes isolated spikes that differ from their neighbors by more than 50 cm. These are usually multi-path reflections or scanner motor artifacts, not real objects. |

The output of this chain is the filtered `/scan` topic. Both the laser scan matcher and SLAM Toolbox subscribe to this cleaned version, as do both costmaps.

---

## 3.4 Laser Scan Matcher — Odometry

Since wheel encoder odometry is not yet integrated into the ROS2 side, all incremental dead-reckoning is done by matching consecutive LiDAR scans using ICP (Iterative Closest Point). The node used is `ros2_laser_scan_matcher`, and it publishes both a `/odom` topic and the `odom→base_link` TF transform.

**How it works.** The matcher keeps a keyframe — a stored scan used as the reference. As new scans arrive, ICP is run between the incoming scan and the keyframe to compute the relative rigid-body transform (translation + rotation). Once the robot has moved more than a threshold distance or angle from the keyframe, the current scan is promoted to the new keyframe. This prevents drift from compounding in place but still generates a new pose estimate every scan cycle.

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Keyframe distance threshold | 0.10 m | New keyframe after 10 cm of motion |
| Keyframe angle threshold | 10° | New keyframe after 10° of rotation |
| ICP max iterations | 30 | Convergence cap per scan |
| Max correspondence distance | 0.30 m | Points further apart than 30 cm are not matched |
| Max angular correction | 45° | Guards against catastrophically wrong initial alignment |
| Scan queue size | 10 | Buffered scan history |
| Output topic | `/odom` | `nav_msgs/Odometry` with covariance |
| Output TF edge | `odom → base_link` | Published continuously |

**Covariance.** The node computes covariance estimates from the ICP residual and includes them in the Odometry message. SLAM Toolbox and Nav2 both read these covariances when deciding how much to trust the odometry. If ICP converges poorly (e.g., in a featureless corridor), the covariance grows and the system places less weight on the odometry estimate.

**Current limitation.** Because this is scan-to-scan matching only, there is an inherent drift accumulation over time. For short parking maneuvers (a few meters of travel), the drift is negligible. For a robot that drives continuously for several minutes, it will be noticeable — SLAM Toolbox's loop closure will correct the map, but the local odometry frame itself accumulates error. This is the main motivation for the IMU fusion work described in Section 3.8.

---

## 3.5 SLAM Toolbox — Live Mapping

SLAM Toolbox runs in **asynchronous mapping mode** (`async_slam_toolbox_node`). In this mode, it continuously incorporates incoming scans to refine the map without blocking the main processing pipeline. There is no pre-loaded map. The map starts empty and grows as the robot explores.

The configuration lives in `config/slam_toolbox_mapping.yaml`.

| Parameter | Value | Notes |
|-----------|-------|-------|
| Mode | `mapping` | Live SLAM; no saved-map localization |
| Map resolution | **0.02 m (2 cm/pixel)** | Chosen deliberately for parking precision |
| Update distance | 0.05 m | Map updated after 5 cm of motion |
| Update heading | 0.05 rad (~3°) | Map updated after small rotations |
| Fallback update interval | 1.0 s | Update even if robot is stationary |
| TF wait timeout | 0.5 s | How long to wait for a TF before skipping scan |
| TF buffer duration | 30 s | Transform history kept for delayed scan processing |
| Scan buffer size | 20 scans | History used for scan-to-map correlation |
| Use scan matching | `true` | Trusts laser_scan_matcher for local odometry |
| Provide odometry frame | `false` | laser_scan_matcher owns `odom→base_link` |
| Loop closure search radius | 3.0 m | Candidates within 3 m are evaluated |

**Why 2 cm resolution?** A parking space for this robot class is roughly 0.5 m × 1.0 m. At 2 cm/pixel, a slot occupies about 25 × 50 cells in the occupancy grid — enough to represent the geometry clearly without being computationally prohibitive. At 5 cm (the more common choice for mobile robots), a parking slot would only be 10 × 20 cells, which is too coarse for the planned ArUco-refined docking sequence.

**Outputs:**
- `/map` (`nav_msgs/OccupancyGrid`) — the live occupancy grid, published at the configured update rate
- `map → odom` TF transform — published continuously; represents the correction between the cumulative map frame and the local odometry frame

---

## 3.6 TF Tree

The transform tree for the perception subsystem is kept intentionally simple. There are only two dynamic TF publishers and one static one.

```
map
 │
 │  ← published by: SLAM Toolbox (async_slam_toolbox_node)
 │    (represents map-level pose correction; updated as map refines)
 │
odom
 │
 │  ← published by: laser_scan_matcher
 │    (represents incremental dead-reckoning; updated at ~10 Hz)
 │
base_link
 │
 │  ← static transform; published by: static_transform_publisher in launch file
 │    (represents fixed sensor placement on chassis)
 │
laser_link
```

**Why the split between `map→odom` and `odom→base_link`?**
This is the standard ROS2 convention. `odom→base_link` is the best continuous estimate of motion since the last reset — it never jumps. `map→odom` is the correction layer; SLAM Toolbox adjusts it to keep the robot's position consistent with the built map. Nav2 reads the full chain (`map→odom→base_link`) for planning and control, but the continuity guarantee on `odom→base_link` prevents sudden jumps in the velocity commands.

**EKF note.** When the IMU is integrated, `robot_localization` (EKF) will take over publishing `odom→base_link` using fused laser + IMU data. The `ekf_fusion.launch.py` and `ekf_2d_no_imu.yaml` files already exist as the skeleton for this. The EKF is configured with `publish_tf: false` in the meantime to avoid conflicting with laser_scan_matcher.

---

## 3.7 Perception Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SLAMTEC C1 LiDAR                             │
│             USB serial @ 460,800 baud → /dev/ttyUSB0                │
└──────────────────────────────┬──────────────────────────────────────┘
                               │  sllidar_ros2 driver
                               ▼
                         /scan  (raw LaserScan)
                               │
                               ▼
                    ┌──────────────────────┐
                    │   laser_filters      │
                    │   4-stage chain      │
                    │   (range → shadow →  │
                    │    median → outlier) │
                    └──────────┬───────────┘
                               │  /scan  (filtered)
               ┌───────────────┴──────────────────┐
               │                                  │
               ▼                                  ▼
  ┌─────────────────────────┐        ┌────────────────────────────┐
  │  laser_scan_matcher     │        │  async_slam_toolbox_node   │
  │  (ICP scan-to-scan)     │        │  (async mapping mode)      │
  │                         │        │                            │
  │  → /odom                │        │  → /map  (OccupancyGrid)   │
  │  → odom→base_link TF    │        │  → map→odom TF             │
  └─────────────────────────┘        └────────────────────────────┘
               │                                  │
               └─────────────┬────────────────────┘
                             │
                             ▼
              Full TF chain: map → odom → base_link → laser_link
              /odom topic: consumed by Nav2, velocity smoother
              /map topic:  consumed by global costmap, planner
```

---

## 3.8 Planned Extensions

### IMU Fusion (in progress)

The EKF integration skeleton is already in the repository (`ekf_fusion.launch.py`, `ekf_2d_no_imu.yaml`). Once an IMU is physically connected and a ROS2 IMU driver is running, the plan is:

1. Feed `/imu/data` and laser_scan_matcher's `/odom` into `robot_localization` EKF
2. EKF publishes `/odometry/filtered` + the new `odom→base_link` TF (with `publish_tf: true`)
3. Disable laser_scan_matcher's TF output to avoid the conflict
4. Nav2 and SLAM Toolbox are pointed at `/odometry/filtered` instead of `/odom`

The immediate benefit is significantly reduced drift during sharp turns and acceleration events — situations where scan matching alone tends to slip because the scan changes too fast between frames.

### Camera Integration (planned — Mode B)

The IMX219 camera link is already defined in the URDF at +100 mm forward, +50 mm up from `base_link`. The Flutter mobile app already decodes ArUco markers in its camera feed and sends the detected marker pose to the ROS2 bridge via HTTP. The remaining work on the robot side is:

1. Add a ROS2 node that streams the IMX219 feed as `/camera/image_raw`
2. Optionally run an onboard ArUco detector to produce `/aruco/detected_pose`
3. In the parking maneuver node, use the detected marker pose to refine the Nav2 goal before commanding the final approach

The system is designed so that this addition does not change the SLAM or odometry pipeline at all — the camera output feeds into the goal-setting logic, not into the map or TF chain. Mode A (LiDAR-only) continues to work unmodified.

---

## 3.9 Known Limitations and Open Items

| Item | Severity | Current Behavior | Resolution Path |
|------|----------|------------------|-----------------|
| Scan-only odometry drift | Medium | Accumulates over multi-minute runs; SLAM loop closure corrects map but not local odom | Resolved by IMU fusion (Section 3.8) |
| LiDAR serial port lock | Medium | Stale `sllidar_node` process can block the serial device on next launch | Kill stale processes; enforce `/dev/serial/by-id/` path discipline |
| No camera feed yet | Low | ArUco-based parking refinement unavailable | Camera integration planned; mobile app already handles marker detection as interim |
| Scan matching in featureless areas | Low | ICP diverges in corridors with no geometric features; odometry jumps | Park in environments with distinguishable features; longer-term: use IMU to bridge featureless segments |
| Differential motion model mismatch | Low | Laser scan matcher assumes small incremental motion; aggressive steering can cause scan registration errors | Limit angular velocity in bridge node (currently capped at 0.8 rad/s) |

---
---

# Section 4 — Navigation Subsystem

## 4.1 Overview

The navigation subsystem takes a goal pose — either from a user clicking in RViz or from the mobile app sending a parking command — and produces motor velocity commands that move the robot to that goal while avoiding obstacles. It uses the ROS2 Nav2 stack, running on top of the live SLAM map produced by the perception subsystem.

There is no pre-loaded map and no separate localization phase. The robot starts SLAM when it boots, builds the map as it drives, and navigation requests can be sent at any point once the TF tree is healthy. This is the only supported operational mode — the static-map and AMCL-based workflows from earlier development phases have been archived.

The full pipeline from goal to motor output involves several nodes in sequence:

```
Goal (RViz / mobile app)
        │
        ▼
   BT Navigator          ← orchestrates planner and controller
        │
        ├── Planner Server (NavfnPlanner)     ← computes global path
        │
        ├── Controller Server (DWBLocalPlanner) ← generates /cmd_vel
        │
        ├── Velocity Smoother                  ← rate-limits acceleration
        │
        ├── Collision Monitor                  ← safety veto on velocity
        │
        └── cmd_vel_to_pico_bridge             ← final clamping + Pico publish
```

Each stage is described in detail below.

---

## 4.2 Global Planner — NavfnPlanner

The global planner computes a path from the robot's current pose to the goal pose using the live `/map` from SLAM Toolbox. The planner used is `nav2_navfn_planner::NavfnPlanner`, which implements a wavefront A* expansion over the global costmap.

| Parameter | Value | Notes |
|-----------|-------|-------|
| Plugin | `nav2_navfn_planner::NavfnPlanner` | Standard Nav2 A* planner |
| Goal tolerance | 0.05 m | Accepts path endpoints within 5 cm of the requested goal |
| Allow planning through unknown cells | `false` | Robot does not speculatively plan through unmapped space |
| Input | Global costmap (`/map` + live obstacles) | Replanned on each new goal or significant map change |
| Output | `/plan` (`nav_msgs/Path`) | Sequence of poses from current position to goal |

**Practical behavior.** Because `allow_unknown` is false, the robot will refuse to plan a path into areas that SLAM has not yet mapped. This is intentional for a parking use case — we want the robot to first survey the lot before attempting to navigate to a slot, not drive blind into unmapped space. In practice this means the operator should do a slow drive-through first to populate the map before sending a parking goal.

---

## 4.3 Local Controller — DWB Local Planner

The local controller runs at 10 Hz and is responsible for actually tracking the global path while reacting to obstacles that weren't on the map when the path was planned. It uses the **DWB (Dynamic Window-Based) local planner**, which works by sampling a set of short-horizon velocity trajectories, scoring each one against a set of critics, and commanding the highest-scoring trajectory.

### Velocity and Acceleration Limits

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max forward speed (`max_vel_x`) | 0.30 m/s | Conservative for parking precision |
| Max reverse speed (`min_vel_x`) | −0.10 m/s | Limited reverse available |
| Max lateral speed | 0.0 m/s | No holonomic motion |
| Max yaw rate (`max_vel_theta`) | 0.50 rad/s | ~28°/s |
| Forward acceleration limit | 1.5 m/s² | |
| Angular acceleration limit | 2.0 rad/s² | |
| Forward deceleration limit | −1.5 m/s² | |
| Angular deceleration limit | −2.0 rad/s² | |

### Goal Tolerance

| Axis | Tolerance | Notes |
|------|-----------|-------|
| XY position | 0.05 m (5 cm) | Tight enough for parking without being unreachable |
| Yaw | 0.10 rad (~6°) | Reasonable heading precision for the servo hardware |

### DWB Critics

DWB scores each candidate trajectory by summing weighted critic scores. The seven critics active in this configuration are:

| Critic | Weight | What It Penalizes |
|--------|--------|-------------------|
| `PathAlign` | 32.0 | Trajectories that deviate from the planned path direction |
| `PathDist` | 32.0 | Distance from the trajectory endpoint to the nearest planned path point |
| `RotateToGoal` | 32.0 | Not rotating toward the goal heading when close |
| `GoalAlign` | 24.0 | Heading misalignment with the goal pose |
| `GoalDist` | 24.0 | Distance from trajectory endpoint to the final goal |
| `BaseObstacle` | 0.02 | Closeness to obstacles in the local costmap |
| `Oscillation` | (default) | Penalizes repetitive back-and-forth motion |

The high weights on `PathAlign`, `PathDist`, and `RotateToGoal` bias the controller toward path-following behavior, which is appropriate for a structured parking lot. `BaseObstacle` has a low absolute weight, but the collision monitor downstream provides an independent hard veto on any trajectory that would actually hit something.

---

## 4.4 Costmap Configuration

Both the global and local costmaps use the same resolution as the SLAM map (2 cm/pixel) so there is no resampling artifact when obstacle data is transferred between them.

### Local Costmap

The local costmap is a rolling window that follows the robot. It is used exclusively by the local controller for real-time obstacle avoidance.

| Parameter | Value |
|-----------|-------|
| Frame | `odom` |
| Window size | 2.0 m × 2.0 m |
| Resolution | 0.02 m |
| Update frequency | 5.0 Hz |
| Publish frequency | 2.0 Hz |
| Robot radius | 0.10 m |

**Layers:**

| Layer | Configuration | Purpose |
|-------|---------------|---------|
| `obstacle_layer` | Source: `/scan`; raytrace range: 2.5 m; obstacle range: 2.0 m | Marks and clears obstacles based on live LiDAR |
| `inflation_layer` | Radius: **0.20 m**; cost scaling: 5.0 | Expands obstacles so the planner path keeps clearance |

The 20 cm inflation radius on the local costmap is relatively generous for a robot with a 10 cm physical radius. The extra margin accounts for the fact that the URDF footprint describes the chassis only — the servo linkage and any attached hardware add a few centimeters in practice, and parking maneuvers involve close-range approach to static structures.

### Global Costmap

The global costmap covers the full mapped area and is used by NavfnPlanner to compute the initial path.

| Parameter | Value |
|-----------|-------|
| Frame | `map` |
| Coverage | Full map extent (follows `/map` size) |
| Resolution | 0.02 m |
| Update frequency | 1.0 Hz |
| Publish frequency | 1.0 Hz |

**Layers:**

| Layer | Configuration | Purpose |
|-------|---------------|---------|
| `static_layer` | Source: `/map` | Brings in the SLAM-generated occupancy grid |
| `obstacle_layer` | Source: `/scan` | Adds live obstacles on top of the static map |
| `inflation_layer` | Radius: **0.15 m**; cost scaling: 3.5 | Expands obstacles for path planning clearance |

The global costmap uses a slightly smaller inflation radius (15 cm vs 20 cm) than the local one because it is used for path computation, not real-time collision checking. Making it too large in the global costmap would cause the planner to route unnecessarily wide paths in tight parking spaces.

---

## 4.5 Velocity Pipeline

Between the controller output and the Pico microcontroller, the velocity command passes through three additional nodes. Each one adds either smoothing or safety filtering.

```
Controller Server
      │  /cmd_vel  (Twist, ~10 Hz)
      ▼
Velocity Smoother
      │  /cmd_vel_smoothed  (Twist, 20 Hz)
      ▼
Collision Monitor
      │  /cmd_vel_safe  (Twist)
      ▼
cmd_vel_to_pico_bridge
      │  /pico/control_cmd  (TwistStamped, 30 Hz)
      │  /pico/enable       (Bool)
      │  /pico/heartbeat    (Bool)
      ▼
micro-ROS agent  (USB serial, 115,200 baud)
      ▼
Pico firmware  (50 Hz control loop)
      ▼
Motors + Steering Servo
```

| Node | Input | Output | Key Behavior |
|------|-------|--------|--------------|
| **Velocity Smoother** | `/cmd_vel` from controller | `/cmd_vel_smoothed` at 20 Hz | Limits acceleration; runs in OPEN_LOOP mode (does not feed back from `/odom`) |
| **Collision Monitor** | `/cmd_vel_smoothed` + `/scan` | `/cmd_vel_safe` | Simulates trajectory 1.2 s forward; reduces or stops velocity if collision is predicted |
| **cmd_vel_to_pico_bridge** | `/cmd_vel_safe` | `/pico/control_cmd` at 30 Hz | Final velocity clamping, per-cycle acceleration limiting, 200 ms timeout |

**Velocity Smoother parameters:**

| Parameter | Value |
|-----------|-------|
| Output frequency | 20.0 Hz |
| Max linear velocity | 0.30 m/s |
| Max angular velocity | 0.50 rad/s |
| Max linear acceleration | 1.5 m/s² |
| Max angular acceleration | 2.0 rad/s² |
| Feedback mode | `OPEN_LOOP` |

**Collision Monitor parameters:**

| Parameter | Value |
|-----------|-------|
| Polygon type | `FootprintApproach` |
| Lookahead time | 1.2 s |
| Simulation step | 0.1 s |
| LiDAR source topic | `/scan` |

---

## 4.6 cmd_vel_to_pico_bridge

This node (`scripts/cmd_vel_to_pico_bridge.py`) is the last software stage before commands leave the RPi5 and go to the Pico over USB serial. It has several responsibilities that the upstream Nav2 nodes do not cover.

### Velocity Clamping

The bridge enforces hard velocity limits independently of what Nav2 is configured to send. This provides a second layer of protection against misconfiguration upstream.

| Limit | Value | Notes |
|-------|-------|-------|
| Max forward speed | ±0.35 m/s | Slightly higher than DWB limit; bridge is the hard floor |
| Max yaw rate | ±0.80 rad/s | Hard cap regardless of Nav2 config |

### Per-Cycle Acceleration Limiting

Even after the velocity smoother, the bridge applies its own acceleration cap per 33 ms cycle (at 30 Hz). This is because the velocity smoother operates at 20 Hz and the smoother output can still have sharp edges when viewed at 30 Hz. The bridge rate-limits at:

| Parameter | Value |
|-----------|-------|
| Max linear acceleration step | 0.8 m/s² |
| Max angular acceleration step | 1.2 rad/s² |

The algorithm each cycle:
```
delta = target - current
clamp delta to ± (max_accel × dt)
new_command = current + clamped_delta
```

### Command Timeout

If no new message arrives on `/cmd_vel_safe` for more than 200 ms, the bridge immediately ramps the output to zero and publishes `enable = false`. This protects against the situation where Nav2 freezes or the topic stops publishing for any reason — the robot does not continue at its last commanded velocity indefinitely.

### Single-Publisher Guard

Multiple bridge instances publishing simultaneously would fight over the Pico, potentially alternating between incompatible commands. The guard works at two levels:

1. **File-based lock:** On startup, the bridge acquires an exclusive lock on `/tmp/cmd_vel_to_pico_bridge.lock`. A second instance trying to acquire the same lock gets an `OSError` and exits immediately.

2. **ROS graph monitoring:** Every 1.0 s (after a 3.0 s startup delay), the bridge counts the number of publishers on `/pico/control_cmd`. If more than one is detected, the bridge publishes a zero-velocity safe stop and then shuts itself down.

### Published Outputs

| Topic | Type | Rate | Content |
|-------|------|------|---------|
| `/pico/control_cmd` | `geometry_msgs/TwistStamped` | 30 Hz | Rate-limited vx and wz; `frame_id: base_link` |
| `/pico/enable` | `std_msgs/Bool` | 30 Hz | `true` when commands are active; `false` on timeout or error |
| `/pico/heartbeat` | `std_msgs/Bool` | 5 Hz | Liveness signal; Pico watchdog times out at 200 ms if missing |

---

## 4.7 Safety Layer Stack

Safety in this system is layered. No single layer is expected to catch everything; the design assumes that any layer can fail and the next one will still prevent damage. The layers from outermost to innermost are:

| Layer | Node | Mechanism | Trigger | Action |
|-------|------|-----------|---------|--------|
| 1 | Velocity Smoother | Acceleration limiting | Any commanded velocity change | Ramps output rather than stepping; prevents sudden motor torque |
| 2 | Collision Monitor | Forward trajectory simulation | Predicted collision within 1.2 s | Reduces or zeros velocity before the obstacle is reached |
| 3 | cmd_vel_to_pico_bridge | Velocity clamping | Any command exceeding limits | Hard clips vx and wz to configured maximums |
| 4 | cmd_vel_to_pico_bridge | Per-cycle acceleration cap | Excessive delta between cycles | Smooths any step changes the smoother missed |
| 5 | cmd_vel_to_pico_bridge | Command timeout | No command for 200 ms | Ramps to zero, disables motors via `enable = false` |
| 6 | Pico firmware | Command timeout watchdog | No `/pico/control_cmd` for 200 ms | Disables motors at the hardware level; steering returns to center |
| 7 | Pico firmware | E-STOP | E-STOP signal received | Latching motor disable; persists until explicitly cleared via `safety_estop_clear()` |

**Layers 5 and 6 both use the same 200 ms threshold**, but they operate independently. Layer 5 acts in software on the RPi5 side; layer 6 acts on the Pico itself. If the USB serial link drops (killing micro-ROS communication), layer 5 will not even be aware — but layer 6 will still fire because the Pico's watchdog sees no incoming messages.

**E-STOP behavior.** The Pico firmware implements a latching E-STOP: once activated, it does not automatically clear when commands resume. The `safety_estop_clear()` function must be called explicitly. During E-STOP the on-board LED flashes at 5 Hz; normal operation blinks at 1 Hz. This provides a visual indicator without requiring a serial connection to check state.

---

## 4.8 Full Navigation Data Flow

```
                         ┌─────────────┐
  RViz goal click ──────►│ BT Navigator│
  Mobile app command ───►│ (nav2_bt_   │
                         │  navigator) │
                         └──────┬──────┘
                                │
               ┌────────────────┼────────────────┐
               │                                 │
               ▼                                 ▼
    ┌─────────────────────┐           ┌──────────────────────┐
    │   Planner Server    │           │  Global Costmap       │
    │   NavfnPlanner      │◄──────────│  static_layer         │
    │                     │           │  + obstacle_layer     │
    │  Input: /map, pose  │           │  + inflation_layer    │
    │  Output: /plan      │           └──────────────────────┘
    └──────────┬──────────┘
               │  /plan
               ▼
    ┌─────────────────────┐           ┌──────────────────────┐
    │  Controller Server  │           │  Local Costmap        │
    │  DWBLocalPlanner    │◄──────────│  obstacle_layer       │
    │                     │           │  + inflation_layer    │
    │  Input: /plan, /odom│           └──────────────────────┘
    │  Output: /cmd_vel   │                     ▲
    └──────────┬──────────┘                     │ /scan (filtered)
               │  /cmd_vel  (~10 Hz)             │
               ▼
    ┌─────────────────────┐
    │  Velocity Smoother  │
    │  nav2_velocity_     │
    │  smoother           │
    │  Output: /cmd_vel_  │
    │         smoothed    │
    └──────────┬──────────┘
               │  /cmd_vel_smoothed  (20 Hz)
               ▼
    ┌─────────────────────┐
    │  Collision Monitor  │◄───── /scan (filtered)
    │  nav2_collision_    │
    │  monitor            │
    │  Simulates 1.2 s    │
    │  Output: /cmd_vel_  │
    │         safe        │
    └──────────┬──────────┘
               │  /cmd_vel_safe
               ▼
    ┌─────────────────────┐
    │  cmd_vel_to_pico_   │
    │  bridge (RPi5)      │
    │  - velocity clamp   │
    │  - accel limiting   │
    │  - 200 ms timeout   │
    │  - duplicate guard  │
    └──────────┬──────────┘
               │  USB serial (115,200 baud)
               │  XRCE-DDS via micro-ROS agent
               ▼
    ┌─────────────────────────────────────────────┐
    │  Pico Firmware  (50 Hz control loop)         │
    │                                              │
    │  1. Receive /pico/control_cmd (vx, wz)       │
    │  2. Ackermann IK:                            │
    │       steer = atan(L × wz / vx)             │
    │       v_L = vx × (R − W/2) / R              │
    │       v_R = vx × (R + W/2) / R              │
    │  3. Steering servo → GPIO 12 PWM (50 Hz)    │
    │  4. Motors → I2C Hiwonder driver (0x34)     │
    │  5. Encoders → /pico/odom (20 Hz)           │
    └──────────┬──────────────────────────────────┘
               │
      ┌────────┴────────┐
      ▼                 ▼
  Rear DC Motors   Steering Servo
  (Hiwonder I2C)   (PWM, ±30°)
      │
      ▼
  Robot motion
```

---

## 4.9 Update Rates and End-to-End Latency

The table below shows the typical operating rate and latency contribution of each processing stage. The end-to-end latency from a Nav2 goal being accepted to motors responding is approximately 500–800 ms under normal conditions.

| Stage | Update Rate | Typical Latency |
|-------|-------------|-----------------|
| LiDAR scan publication | ~10 Hz | < 100 ms from physical measurement |
| Scan filter chain | Inline with scan | ~20–30 ms |
| Laser scan matcher | Per keyframe (variable) | 20–50 ms per ICP solve |
| SLAM Toolbox map update | 1.0 Hz (configurable) | 100–200 ms |
| NavfnPlanner (goal received) | On-demand | ~200 ms first path |
| DWB Controller | 10 Hz | 100 ms per cycle |
| Velocity Smoother | 20 Hz | ~50 ms |
| Collision Monitor | Async (scan-driven) | ~100 ms |
| cmd_vel_to_pico_bridge | 30 Hz | ~33 ms |
| micro-ROS serial transport | Up to 30 Hz | 10–50 ms |
| Pico control loop | 50 Hz | 20 ms |

**Total goal-to-motor latency:** ~500–800 ms. This is dominated by the planner computation and DWB's 10 Hz cycle. For steady-state path following, the controller is producing commands continuously and the latency is simply the pipeline delay — about 200–300 ms from an obstacle appearing in `/scan` to the motor responding.

---

## 4.10 Planned Extensions

### IMU Integration

The `robot_localization` EKF skeleton is already present in the repository. Once an IMU is connected and its ROS2 driver is running, the integration steps are:

1. Enable `robot_localization` node in `ekf_fusion.launch.py`
2. Set `publish_tf: true` in `ekf_2d_no_imu.yaml`; disable TF from laser_scan_matcher
3. Point Nav2's `odom_topic` parameter at `/odometry/filtered`
4. Tune EKF process and measurement noise matrices against real hardware

The navigation stack itself does not need to change. Only the odometry source changes.

### Camera-Guided Parking Refinement

Once the camera is integrated into Mode B, the parking flow will change slightly:

1. Operator selects a parking slot in the mobile app (identified by ArUco marker ID)
2. Mobile app sends the detected marker pose (via the HTTP bridge, `ros2_mobile_bridge.py`) to the RPi5
3. A parking coordinator node converts the marker pose to a Nav2 goal and calls the navigation action
4. For the final approach (last ~0.5 m), the goal is refined using the live marker detection rather than dead-reckoned SLAM pose

This architecture keeps SLAM and Nav2 unchanged. The camera only influences goal selection, not the underlying path planning or control.

### Smac Hybrid-A* Migration

The DWBLocalPlanner was chosen because it is well-tested and stable in Nav2. However, it uses a differential-drive motion model internally — it does not know that this robot has Ackermann kinematics and cannot make zero-radius turns. In practice, the velocity limits are conservative enough that this mismatch rarely causes problems. But for tighter parking spaces or sharper maneuvers, the Smac Hybrid-A* planner (which explicitly models non-holonomic constraints including minimum turning radius) would produce better trajectories. This migration is identified but deferred until the basic parking workflow is validated end-to-end.

---

## 4.11 Known Limitations and Open Items

| Item | Severity | Current Behavior | Resolution Path |
|------|----------|------------------|-----------------|
| DWB uses differential-drive model | Medium | Cannot plan geometrically correct Ackermann paths; sharp turns may be physically unreachable | Smac Hybrid-A* migration planned (Section 4.10) |
| EKF not active | Medium | Odometry susceptible to drift; Nav2 odometry quality limited | IMU integration resolves this |
| Control-source arbitration undefined | Medium | No formal policy for manual vs. autonomous vs. e-stop priority; currently enforced by single-publisher guard only | Explicit state machine (AUTO / MANUAL / ESTOP) planned before final system demo |
| Long-duration drift not validated | Low | Only short parking runs tested; multi-minute SLAM sessions not soak-tested | Field endurance tests scheduled |
| Planner replanning latency | Low | First path after a new goal takes ~200 ms; rapid goal changes feel sluggish | Acceptable for parking use case; not a priority |
