# AutoNexa Parking Plan — Park & Summon Mission

**Status:** Plan / proposed
**Author:** Anıl Çolak (with Claude assistance), 2026-05-07
**Scope:** End-to-end design for autonomous **park** and **summon** behaviors on the AutoNexa Ackermann robot, layered on top of the existing live-SLAM + Nav2 baseline.

> This document is the single reference for finishing the project's headline mission — "the car parks and summons itself autonomously" — as called for by `cdrr_perception_navigation.md` §3.1 (Mode B). It supersedes ad-hoc parking notes in earlier docs.

---

## 1. Mission Goals (recap from the project description)

| Goal | Definition of done |
|---|---|
| **Park** | From any reachable pose, app issues "Park at slot N"; car drives to slot N, performs whatever maneuver is required (forward, perpendicular, or reverse), and ends with marker centered ±2° and rear-bumper distance ≤ 5 cm. No human intervention after the tap. |
| **Summon** | From a parked state, app issues "Summon"; car exits the slot, navigates to the operator's chosen pickup point, and stops within ±15 cm. |
| **Slot DB** | At least 4 named slots can be defined, persisted across reboots, and listed in the app. |
| **Safety** | E-STOP at any point during a parking mission cancels the goal and halts ≤200 ms; mode auto-switches to ESTOP. |

These four are the acceptance criteria for "project complete on the autonomy side." Everything below is in service of them.

---

## 2. Current Baseline (what we have)

| Layer | Status |
|---|---|
| Live SLAM (SLAM Toolbox, 2 cm/pixel) | ✅ working |
| Nav2: NavfnPlanner + DWBLocalPlanner | ✅ working — but slow (~500–800 ms goal-to-motion) and **non-Ackermann-aware** |
| Mobile app: tap-to-navigate, joystick, mode switcher (AUTO/MANUAL/ESTOP), Nav2 status, RViz-quality map | ✅ shipped 2026-05-07 |
| Pico micro-ROS firmware: Ackermann IK, 50 Hz, 200 ms watchdog, E-STOP latch | ✅ working |
| ArUco vision (camera + decoder in app) | ⚠ exists in `aruco_project/`, **not yet wired as a ROS2 publisher** |
| Parking maneuver | ❌ **absent** — current behavior is "drive to (x, y, yaw) and stop within 15 cm" |
| Slot database | ❌ none — every goal is ad-hoc |
| Mission orchestration | ❌ none |

The honest gap analysis: we have a *driving* robot, not a *parking* robot. This plan closes that gap.

---

## 3. Reference Implementations Surveyed

Before writing custom code, we anchored each layer to a proven open-source or commercial reference so we can borrow patterns and config.

| Layer | Reference | Why it matters |
|---|---|---|
| Path planner | **Nav2 `SmacPlannerHybrid`** ([docs](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)) | Highly optimized Hybrid-A* with Dubin and Reeds-Shepp models, multi-resolution search, 10× smaller motion primitives — built specifically for Ackermann/car-like robots. Drop-in plugin. |
| Path follower | **Nav2 Regulated Pure Pursuit** ([docs](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)) | No trajectory sampling — closed-form pursuit + curvature regulation. ~10× cheaper than DWB and a better fit for Ackermann. |
| Docking server | **opennav_docking** ([repo](https://github.com/open-navigation/opennav_docking), [Nav2 tutorial](https://docs.nav2.org/tutorials/docs/using_docking.html)) | Nav2-compatible task server that does the staging-pose-then-vision-controlled-final-approach loop generically. Subscribes to `geometry_msgs/PoseStamped` for the dock pose, so we can feed it our ArUco detection directly. **This means we don't write our own docking state machine — we configure this one.** |
| ArUco docking, ROS 2 Jazzy | **Vor7reX/ibt_ros2_autodocking** ([repo](https://github.com/Vor7reX/ibt_ros2_autodocking)), **dawan0111/Auto-Marker-Docking** ([repo](https://github.com/dawan0111/Auto-Marker-Docking)) | Working ROS2 packages that combine Nav2 high-level navigation with ArUco-based fine alignment. Useful as code reference for the dock-pose publisher. |
| Visual servoing tutorial | **mohamedeyaad/aruco_visual_servoing** ([repo](https://github.com/mohamedeyaad/aruco_visual_servoing)) | ROS 2 Jazzy + ArUco, includes ID sorting & sequential visiting — informs our slot DB design. |
| AprilTags + Nav2 docking tutorial | **automaticaddison.com** ([guide](https://automaticaddison.com/autonomous-docking-with-apriltags-using-nav2-ros-2-jazzy/)) | End-to-end practical writeup; we follow the same pattern but with ArUco. |
| Geometric parking maths | **MDPI: Geometric Path Plans for Perpendicular/Parallel Reverse Parking** ([paper](https://www.mdpi.com/2624-8921/4/4/63)) | Two-stage geometric paths for narrow slots — used as a sanity check for the staging/final-pose offsets we feed to Smac. |
| Reverse parking benchmark | **Rohith-K Autonomous-Parallel-Parking-Gazebo-ROS** ([repo](https://github.com/Rohith-K/Autonomous-Parallel-Parking-Car-like-Robot-Gazebo-ROS)) | Reference implementation we can replay in Gazebo before hardware. |
| Full-stack valet | **Autoware Autonomous Valet Parking demo** ([docs](https://autowareauto.readthedocs.io/en/release/avpdemo.html)), **MATLAB Automated Parking Valet with ROS 2** ([docs](https://www.mathworks.com/help/ros/ug/automated-valet-using-ros2-matlab.html)) | Architectural references — confirms our split (planner / controller / mission BT / docking) matches industry patterns. |

**Decision:** the cleanest stack for us is **Smac Hybrid-A* (REEDS_SHEPP) + Regulated Pure Pursuit + opennav_docking + custom ArUco dock-pose publisher**. We are not inventing — we are wiring proven Nav2 primitives in the configuration our chassis needs.

---

## 4. Target Architecture

```
                                    ┌──────────────────────┐
                                    │  Mobile app (Flutter)│
                                    │  AUTO/MANUAL/ESTOP   │
                                    │  Spots tab + summon  │
                                    └─────────┬────────────┘
                                              │ HTTP + WS
                                  ┌───────────▼──────────────┐
                                  │  ros2_mobile_bridge.py   │
                                  │  /api/park_at /api/summon│
                                  └───────────┬──────────────┘
                                              │ ROS2 service
                                  ┌───────────▼──────────────┐
                                  │  parking_mission_node.py │  (orchestrator)
                                  └────┬─────────────┬───────┘
                                       │             │
                  ┌────────────────────▼─┐         ┌─▼───────────────────────┐
                  │ Nav2 BT (NavToPose)  │         │ Nav2 Docking Server      │
                  │  Smac Hybrid-A* +    │         │  (opennav_docking)       │
                  │  Regulated PP        │         │  staging → vision dock   │
                  └──────┬───────────────┘         └────┬─────────────────────┘
                         │ /cmd_vel                     │ subscribes
                         │                              │ /detected_dock_pose
                         │                              │
              ┌──────────▼──────────────────────────────▼──────────┐
              │            camera_aruco_node (new)                 │
              │  publishes /target_marker (id, pose) + dock pose   │
              └─────────────────────────────────────────────────────┘
                                       │
                  velocity_smoother → collision_monitor → cmd_vel_to_pico_bridge
                                       │
                                       ▼
                               Pico micro-ROS (50 Hz)
```

Three new ROS2 nodes are added; the rest is configuration.

---

## 5. Slot Database Design

Two-tier — both required:

### 5.1 ArUco-anchored slot record (primary)

One ArUco marker (DICT_4X4_50) per slot, mounted at the rear wall, at car-camera height. The marker IS the slot identity — robust to SLAM map drift between sessions.

```yaml
# config/parking_spots.yaml
spots:
  - id: A1
    name: "Reserved 1"
    marker_id: 3
    # Pre-pose: where Nav2 stops before handing off to the docking server.
    # Expressed relative to the marker frame (x forward = away from wall).
    staging_offset: { x: 0.70, y: 0.00, yaw: 3.14159 }   # 70 cm in front, facing marker
    # Final pose: target rear-bumper position. yaw faces away from the wall.
    final_offset:   { x: 0.05, y: 0.00, yaw: 3.14159 }   # 5 cm clearance from wall
    park_kind: "perpendicular_reverse"   # informs which Smac model to use
  - id: A2
    name: "Reserved 2"
    marker_id: 4
    staging_offset: { x: 0.70, y: 0.00, yaw: 3.14159 }
    final_offset:   { x: 0.05, y: 0.00, yaw: 3.14159 }
    park_kind: "perpendicular_forward"
```

`park_kind` determines the planner motion model used for the **staging → final** leg only (the long approach always uses DUBIN forward-only):

| `park_kind` | Smac model for final leg | Notes |
|---|---|---|
| `perpendicular_forward` | DUBIN | Drive in nose-first |
| `perpendicular_reverse` | REEDS_SHEPP | Reverse in — most common for parking |
| `parallel` | REEDS_SHEPP | Standard parallel-parking maneuver, two cusps |

### 5.2 Map-frame static record (fallback)

For slots whose marker is occluded or absent. Coordinates only valid against a *saved* SLAM map.

```yaml
# config/parking_spots_static.yaml
map_file: "maps/garage_2026-05.yaml"   # bound to a specific saved map
spots:
  - id: B1
    name: "Visitor 1"
    final_pose:    { x: 3.42, y: 1.18, yaw: 1.5708 }
    staging_pose:  { x: 2.72, y: 1.18, yaw: 1.5708 }
```

Selection rule at runtime: if `marker_id` is present and the marker has been seen in the last 30 s, use the ArUco record; else fall back to the map-frame record.

### 5.3 Slot-management UX (app)

New `SpotsTab`:

| Action | Wire path |
|---|---|
| **+ Add slot** | Drive there manually → tap "Save here as A1" → app POSTs `/api/save_spot {id, name, marker_id?}` → bridge captures latest pose (and live marker pose if visible) → appends to `parking_spots.yaml`. |
| **List spots** | App polls `/api/spots` (cached) → renders cards: id, name, distance from current pose, last-used timestamp, "Park" / "Edit" / "Delete" buttons. |
| **Park here** | Tap "Park" on slot card → POST `/api/park_at {id}` → mission node takes over. |
| **Summon** | Persistent button on Map tab → POST `/api/summon {x, y, yaw}` (defaults to last-known operator pose). |
| **Lock map** | Settings → "Lock current SLAM map" → bridge calls `nav2_map_server map_saver_cli` and writes `maps/garage_<date>.yaml`. Required before adding map-only slots. |

---

## 6. Nav2 Reconfiguration

### 6.1 Costmap

| Parameter | Old | New | Why |
|---|---|---|---|
| `global_costmap.resolution` | 0.02 m | **0.05 m** | 4× faster planning; parking precision lives in the local costmap |
| `local_costmap.resolution` | 0.02 m | 0.02 m (unchanged) | 25×50 cells per slot — needed for docking |
| `inflation_layer.inflation_radius` (global) | 0.15 | **0.12** | Smaller gradient, cheaper A* heuristic |
| `inflation_layer.cost_scaling_factor` | 3.0 | **2.5** | Slightly less aggressive penalty around obstacles |

### 6.2 Planner: Smac Hybrid-A* (replaces NavfnPlanner)

`config/nav2_navigation_params.yaml`:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlannerHybrid"
      tolerance: 0.10
      downsample_costmap: false
      allow_unknown: false
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      motion_model_for_search: "REEDS_SHEPP"   # default; mission node may override per-leg
      angle_quantization_bins: 72              # 5° resolution
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.45             # wheelbase / tan(max_steer) = 0.25 / tan(30°)
      reverse_penalty: 2.0
      change_penalty: 0.5
      non_straight_penalty: 1.20
      cost_penalty: 1.5
      retrospective_penalty: 0.015
      lookup_table_size: 20.0
      cache_obstacle_heuristic: true
      smooth_path: true
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

The mission node sets `motion_model_for_search` per leg via dynamic parameters: `DUBIN` for the long approach (no reverse, ~3× faster planning) and `REEDS_SHEPP` for the staging → final leg in slots that need reverse.

### 6.3 Controller: Regulated Pure Pursuit (replaces DWB)

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.30
      lookahead_dist: 0.45
      min_lookahead_dist: 0.25
      max_lookahead_dist: 0.70
      use_velocity_scaled_lookahead_dist: true
      transform_tolerance: 0.2
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.2
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.6
      cost_scaling_gain: 1.0
      regulated_linear_scaling_min_radius: 0.45
      regulated_linear_scaling_min_speed: 0.10
      use_rotate_to_heading: false               # Ackermann can't rotate-in-place
      allow_reversing: true                      # honor REEDS_SHEPP plans
      max_robot_pose_search_dist: 5.0
```

Two key flags for our chassis: `use_rotate_to_heading: false` (we have no zero-radius turn) and `allow_reversing: true` (so RPP follows the reverse segments Smac produces).

### 6.4 BT XML

Switch the default to `nav_to_pose_w_replanning_only_if_path_becomes_invalid_bt.xml` — replans only on path invalidation (obstacle drift), not on a periodic timer. Roughly 70% fewer plans during steady-state navigation.

### 6.5 Expected speedup (from §1 of last design discussion)

Goal-to-motion latency: **~700 ms → ~200–300 ms** with the costmap downsample + Smac + RPP changes combined.

---

## 7. Docking Server Integration (`opennav_docking`)

`opennav_docking` is a Nav2-compatible task server that handles the staging → fine-approach → contact pattern generically. We use it for the final-approach (last ~70 cm) of every parking mission.

### 7.1 What it does for us

1. Receives a `DockRobot` action goal (slot id + dock pose).
2. Drives to a configured staging pose using Nav2's `NavigateToPose`.
3. Enters a vision-control loop that subscribes to `/detected_dock_pose` and refines its trajectory in real-time as the dock pose updates.
4. Exits on configurable conditions: distance threshold, contact sensor, charging signal, or a custom predicate.

We pick the **distance-threshold** exit predicate: stop when range to dock < 5 cm and bearing < 2°.

### 7.2 Dock plugin we publish

In `parking_dock_plugin` (new package):

```cpp
// AutoNexa ArUco-marker dock plugin — minimal subclass of nav2_docking::ChargingDock
class ArucoMarkerDock : public nav2_docking::ChargingDock {
  // getStagingPose():    returns staging_offset transformed into map frame
  // getRefinedPose():    returns latest /detected_dock_pose (5 Hz from camera_aruco_node)
  // isDocked():          true when |range| < 0.05 AND |bearing_deg| < 2
  // hasStoppedCharging(): unused — we're not a charging robot
};
```

Reference implementations to model on:
- [open-navigation/opennav_docking](https://github.com/open-navigation/opennav_docking) — official examples
- [Vor7reX/ibt_ros2_autodocking](https://github.com/Vor7reX/ibt_ros2_autodocking) — ArUco + Nav2 fine alignment
- [dawan0111/Auto-Marker-Docking](https://github.com/dawan0111/Auto-Marker-Docking) — ArUco docking with pose refinement

### 7.3 Camera ArUco node (new)

`scripts/camera_aruco_node.py`, runs on RPi5:

- Subscribes to camera topic (or reads `/dev/video0` directly via OpenCV — see `aruco_project/aruco_server.py` for existing detection pipeline).
- Detects DICT_4X4_50 markers, runs `cv2.solvePnP` against a calibrated intrinsic to get marker→camera transform.
- Transforms to map frame using `tf2`.
- Publishes:
  - `/target_marker` (`geometry_msgs/PoseStamped`) — selected marker (driven by mission node setting target id).
  - `/detected_dock_pose` (`geometry_msgs/PoseStamped`) — same data, namespaced as expected by `opennav_docking`.
- Rate: 5 Hz (matches `opennav_docking` refinement loop).

Camera intrinsics live in `config/camera_intrinsics.yaml`; calibrate once with `ros2 run camera_calibration cameracalibrator`.

---

## 8. Mission Orchestrator (`parking_mission_node.py`)

Custom Python node — small, no BT XML needed. Exposes services `park_at_slot(string id)` and `summon(geometry_msgs/Pose target)`.

### 8.1 Park state machine

```
LOAD_SLOT       → read parking_spots.yaml entry for `id`
                  decide ArUco-anchored vs map-frame
                  compute staging_pose, final_pose in map frame
APPROACH        → set Smac motion_model_for_search = DUBIN
                  Nav2 NavigateToPose(staging_pose) — fast, forward only
                  on SUCCEEDED → next; on ABORTED → FAILED
DOCK            → set Smac motion_model_for_search per slot.park_kind
                  send DockRobot action goal to opennav_docking
                  docking server handles fine alignment via /detected_dock_pose
                  on SUCCEEDED (within 5 cm + 2°) → PARKED
                  on ABORTED → fallback: re-issue NavigateToPose(final_pose) once,
                                          then declare FAILED
PARKED          → publish /pico/enable false, hold pose
```

### 8.2 Summon state machine

```
EXIT            → if currently inside a slot, drive forward 0.7 m to clear walls
                  (NavigateToPose with REEDS_SHEPP, allow forward only)
NAVIGATE        → Smac DUBIN, NavigateToPose(target_pose)
ARRIVED         → publish /pico/enable false; mission node returns SUCCEEDED
```

### 8.3 Failure & E-STOP handling

- The mission node subscribes to `/api/mode` updates (or directly to the bridge's mode publisher — TBD).
- Mode change to ESTOP at any state → cancel the active Nav2 / opennav_docking goal, return ABORTED.
- Mode change to MANUAL → cancel mission cleanly, leave robot where it is.

---

## 9. Phased Execution Plan

Each phase is independently testable and ships standalone value. Total estimated effort: **~5–7 working days** of focused work.

### Phase 1 — Nav2 speed + Ackermann correctness (½ day)
- Edit `config/nav2_navigation_params.yaml`: costmap resolution, Smac Hybrid-A*, RPP, BT XML.
- Verify with existing tap-to-navigate flow (no parking yet).
- **Gate:** plan latency < 300 ms; existing TEST.md Stage 8 still passes; obvious obstacles avoided.

### Phase 2 — Map-save / static slot DB + SpotsTab UI (1 day)
- Bridge endpoints: `/api/spots` (GET/POST/DELETE), `/api/save_spot`, `/api/lock_map`.
- File: `config/parking_spots_static.yaml` schema + Python read/write helpers in bridge.
- App: new `SpotsTab` with cards + "Park here" / "Summon" buttons (no parking maneuver yet — just NavigateToPose to `staging_pose`).
- **Gate:** save 2 spots, restart bridge, list comes back; "Park here" drives to staging pose ±15 cm.

### Phase 3 — Camera ArUco ROS2 publisher (½–1 day)
- New file: `scripts/camera_aruco_node.py`.
- Calibrate camera intrinsics (one-time).
- Add launch entry in `nav2_live_slam.launch.py` behind a `use_aruco:=true` arg.
- **Gate:** `ros2 topic echo /target_marker` shows pose updates at 5 Hz when marker visible; pose is consistent with TF tree.

### Phase 4 — opennav_docking integration (1 day)
- Add `opennav_docking` to `package.xml` exec_depends; install on RPi5.
- New tiny package `parking_dock_plugin` with `ArucoMarkerDock` (≈200 LOC C++).
- Configure docking server in `config/nav2_docking.yaml`; launch alongside Nav2.
- **Gate:** Nav2 BT can run `DockRobot` action against a single test slot end-to-end; final pose within 5 cm + 2°.

### Phase 5 — Smac per-leg motion model + park_kind support (½ day)
- Mission node: dynamic-parameter set on `planner_server` per leg.
- Verify Reeds-Shepp paths visible in RViz for `perpendicular_reverse` slots.
- **Gate:** RViz shows the planner produces a one- or two-cusp path for a reverse-park slot; robot executes it without collision.

### Phase 6 — `parking_mission_node` + ArUco-anchored slot DB (1–1.5 days)
- New file: `scripts/parking_mission_node.py`.
- Bridge endpoints: `/api/park_at`, `/api/summon`, both async (return mission UUID, expose `/api/mission_status`).
- App: wire the SpotsTab buttons to the new endpoints; add a "Mission running" overlay with cancel.
- Slot YAML: add `marker_id`, `park_kind`, ArUco-relative offsets.
- **Gate:** Stage 11 of TEST.md (below) passes end-to-end.

### Phase 7 (optional polish) — App refinements (½ day)
- Show planned path differently for parking vs. summon legs (color-coded).
- Visualize slot polygons on the map (small rectangles at slot poses).
- "Last mission" log card in Diagnostics tab.

---

## 10. New TEST.md Stage 11 — Park/Summon

Each step gates the next.

```
11.0  parking_mission_node + opennav_docking running, no errors in logs
11.1  parking_spots.yaml has at least 2 entries; one ArUco-anchored, one map-only
11.2  App "Park at A1" with marker visible →
        - approach completes (staging pose ±10 cm)
        - dock completes (range < 5 cm, bearing < 2°)
        - mode auto-returns to MANUAL after PARKED
11.3  App "Park at B1" (map-only, no marker) →
        - approach completes
        - fallback NavigateToPose to final_pose succeeds (±10 cm)
        - mission marked SUCCEEDED with degraded-precision warning
11.4  App "Summon" while parked →
        - exit maneuver clears slot walls
        - drives to summon point (±15 cm)
11.5  Mid-mission E-STOP →
        - mission ABORTED within 200 ms
        - mode = ESTOP, Pico LED at 5 Hz
        - app shows "Mission aborted by E-STOP"
11.6  Power-cycle test: save 4 slots, reboot RPi5, list survives
```

---

## 11. Files Touched

| Path | Type | Notes |
|---|---|---|
| `config/nav2_navigation_params.yaml` | edit | Costmap, Smac, RPP, BT XML |
| `config/nav2_docking.yaml` | new | opennav_docking server config |
| `config/parking_spots.yaml` | new | ArUco-anchored slot DB |
| `config/parking_spots_static.yaml` | new | Map-frame fallback slot DB |
| `config/camera_intrinsics.yaml` | new | One-time calibration output |
| `launch/nav2_live_slam.launch.py` | edit | Launch arg `use_aruco`, docking server, mission node |
| `scripts/camera_aruco_node.py` | new | ArUco detector → `/target_marker`, `/detected_dock_pose` |
| `scripts/parking_mission_node.py` | new | Park/summon orchestrator |
| `scripts/ros2_mobile_bridge.py` | edit | Endpoints: `/api/spots`, `/api/save_spot`, `/api/park_at`, `/api/summon`, `/api/mission_status`, `/api/lock_map` |
| `parking_dock_plugin/` | new ROS2 package | C++ `ArucoMarkerDock` plugin for opennav_docking |
| `aruco_project/mobile_app/lib/tabs/spots_tab.dart` | new | SpotsTab UI |
| `aruco_project/mobile_app/lib/models/parking_spot.dart` | new | Slot model |
| `aruco_project/mobile_app/lib/services/connection_service.dart` | edit | Spot CRUD + mission API methods |
| `aruco_project/mobile_app/lib/main.dart` | edit | Add SpotsTab to bottom nav |
| `package.xml` | edit | Add `opennav_docking`, `nav2_smac_planner`, `nav2_regulated_pure_pursuit_controller` exec_depends |
| `TEST.md` | edit | Add Stage 11 |
| `CLAUDE.md` | edit | Update "Known Open Items" — close control-source arbitration, parking maneuver |

---

## 12. Risk Register

| Risk | Likelihood | Mitigation |
|---|---|---|
| Smac Hybrid-A* too slow on Pi5 with 5 cm grid | Low–Med | Multi-resolution search is on by default; if needed, downsample further to 7 cm and limit `max_iterations` |
| Camera FOV too narrow to keep marker visible during reverse-park | Medium | Mount marker on rear wall + use rear-facing camera once added; until then, use perpendicular_forward `park_kind` for sketchy slots |
| ArUco false positives in cluttered scenes | Low | DICT_4X4_50 has Hamming distance 4; mission node only accepts pose within 0.5 m of expected staging pose |
| Dock plugin cannot find marker → infinite loop | Medium | `opennav_docking.dock_timeout: 30 s`; mission node falls back to map-frame pose if dock ABORTED |
| Pico open-loop dead reckoning drifts during slow reverse | High in slot, Low overall | The fine-alignment loop is closed by camera, not encoder odometry — closed-loop visual servo dominates |
| Map drift between sessions invalidates static slots | High over weeks | Tier B static slots are bound to a saved map file; Tier A ArUco slots are immune. Encourage Tier A as default. |
| L298N PWM deadband (~60%) prevents fine creep speeds | High at runtime | Already handled in current bridge — `min_vel_x: 0.05` in RPP and Pico's deadband compensation. Verify in Phase 1 gate. |
| `use_rotate_to_heading: false` confuses operators expecting in-place spin | Low | Documented in CLAUDE.md "Known Open Items" — Ackermann is non-holonomic by construction |

---

## 13. Out of Scope

Documented here so they don't creep in:

- **TEB local planner** instead of RPP — possibly better for very tight slots, but heavyweight, ROS1-era, and unnecessary for our slot sizes.
- **Behavior Tree XML for the mission** — we use a Python orchestrator instead. Easier to debug, sufficient for our state count.
- **Multi-robot coordination** — single robot only.
- **Map merging across sessions** — out of scope; each session re-SLAMs unless the user explicitly "Lock Map".
- **Charging dock** — `opennav_docking` supports it but our chassis has no charging contacts. We use the docking framework purely for visual alignment.
- **Reinforcement-learning parking** ([IET reference](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/itr2.12614)) — surveyed, deferred. Hybrid-A* is fully sufficient for our slot geometry and we save the training cost.

---

## 14. References

### Nav2 stack
1. [Nav2 — SmacPlannerHybrid configuration](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
2. [Nav2 — Regulated Pure Pursuit Controller](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)
3. [Nav2 — Smac Planner overview](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)
4. [navigation2 — nav2_smac_planner README (GitHub)](https://github.com/ros-navigation/navigation2/blob/main/nav2_smac_planner/README.md)
5. [Nav2 — Using Docking Server tutorial](https://docs.nav2.org/tutorials/docs/using_docking.html)
6. [Nav2 — Configuring Docking Server](https://docs.nav2.org/configuration/packages/configuring-docking-server.html)

### Docking + ArUco implementations
7. [open-navigation/opennav_docking (GitHub)](https://github.com/open-navigation/opennav_docking)
8. [Vor7reX/ibt_ros2_autodocking — ArUco + Nav2 (GitHub)](https://github.com/Vor7reX/ibt_ros2_autodocking)
9. [dawan0111/Auto-Marker-Docking — ArUco docking (GitHub)](https://github.com/dawan0111/Auto-Marker-Docking)
10. [mohamedeyaad/aruco_visual_servoing — ROS 2 Jazzy (GitHub)](https://github.com/mohamedeyaad/aruco_visual_servoing)
11. [Automatic Addison — Autonomous Docking with AprilTags Using Nav2 (ROS 2 Jazzy)](https://automaticaddison.com/autonomous-docking-with-apriltags-using-nav2-ros-2-jazzy/)
12. [Automatic Addison — Auto-docking to Recharge Battery, ArUco Marker + ROS 2](https://automaticaddison.com/auto-docking-to-recharge-battery-aruco-marker-and-ros-2/)
13. [Aruco_Ros2 documentation](https://aruco-ros2.readthedocs.io/en/latest/index.html)

### Parking maneuvers — algorithms & references
14. [MDPI — Geometric Path Plans for Perpendicular/Parallel Reverse Parking in a Narrow Parking Spot](https://www.mdpi.com/2624-8921/4/4/63)
15. [Rohith-K — Autonomous Parallel Parking, Car-like Robot, Gazebo + ROS (GitHub)](https://github.com/Rohith-K/Autonomous-Parallel-Parking-Car-like-Robot-Gazebo-ROS)
16. [arXiv — Automatic parking planning control method based on improved A* algorithm](https://arxiv.org/html/2406.15429v1)
17. [arXiv — Autonomous Docking via Non-linear Model Predictive Control](https://arxiv.org/pdf/2312.16629)

### Full-stack valet references
18. [Autoware — Autonomous Valet Parking Demonstration](https://autowareauto.readthedocs.io/en/release/avpdemo.html)
19. [MathWorks — Automated Parking Valet with ROS 2 in MATLAB](https://www.mathworks.com/help/ros/ug/automated-valet-using-ros2-matlab.html)
20. [MDPI — Perception, Positioning and Decision-Making for an Autonomous Valet Parking System (single LiDAR)](https://www.mdpi.com/1424-8220/22/3/979)
21. [Springer — Survey of Technology in Autonomous Valet Parking System](https://link.springer.com/article/10.1007/s12239-023-0127-1)

### Internal AutoNexa documents (already in repo)
22. `docs/cdrr_perception_navigation.md` — current Nav2 + SLAM design
23. `docs/IMPLEMENTATION_STATUS_AND_REMAINING_PLAN_2026-03-14.md` — Smac migration noted as P1
24. `docs/AutoNexa_Critical_Design_Review_Report_2026-03-24.md` — system-of-systems view
25. `aruco_project/ARCHITECTURE_RECOMMENDATIONS.md` — original ArUco docking concept
26. `aruco_project/INTEGRATION_GUIDE.md` — Flutter ↔ HTTP bridge contract

---

## 15. Decision Log

| Date | Decision | Rationale |
|---|---|---|
| 2026-05-07 | Use opennav_docking instead of writing a custom docking node | Saves ~300 LOC; battle-tested; subscribes to a generic dock-pose topic that we already plan to publish |
| 2026-05-07 | Smac Hybrid-A* + Regulated Pure Pursuit | Replaces NavfnPlanner + DWB. RPP is the natural Ackermann follower; Smac respects min turning radius and produces Reeds-Shepp paths for parking. |
| 2026-05-07 | Slot DB is two-tier (ArUco primary, map-frame fallback) | ArUco is robust to map drift; map-frame covers slots without markers. Both are needed in practice. |
| 2026-05-07 | Mission orchestration as a Python node, not a Nav2 BT XML | Project state count is small; debugging Python is faster than BT XML; we keep BT for Nav2-internal recovery only. |
| 2026-05-07 | Default boot mode = MANUAL | Already shipped — keeps the robot from autonomous motion at startup. The mission node only takes over on explicit `park_at` / `summon` requests. |

---
*End of plan. Approve to begin Phase 1.*
