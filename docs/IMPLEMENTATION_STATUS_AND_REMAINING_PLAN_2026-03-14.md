# AutoNexa Implementation Status and Remaining Plan

Last updated: 2026-03-14  
Scope: ROS2 Jazzy + Nav2 live SLAM + micro-ROS Pico control chain

## 1) Executive Snapshot

Current working baseline:
- Live SLAM + Nav2 + RViz is running.
- LiDAR mapping instability root cause was identified (stale process locking `/dev/ttyUSB0`), and recovery workflow is documented.
- Nav2 command chain to Pico is implemented with safety layers:
  `/cmd_vel -> /cmd_vel_smoothed -> /cmd_vel_safe -> /pico/control_cmd`.
- Single-publisher safety guard is implemented to prevent duplicate bridge instances from commanding hardware.
- Diagnostic tooling and rosbag capture tooling are implemented.
- Optional EKF skeleton (no IMU yet) is added for next-stage fusion work.

## 2) What Is Implemented (Done)

## 2.1 Core Runtime and Launch
- `nav2_live_slam.launch.py` is the primary runtime stack.
- `use_pico_bridge:=false` default keeps LiDAR/SLAM validation isolated from drive-chain issues.
- Optional Pico chain can be enabled with `use_pico_bridge:=true`.
- Single-publisher guard parameters are exposed in launch:
  - `enforce_single_publisher` (default `true`)
  - `bridge_lock_file` (default `/tmp/cmd_vel_to_pico_bridge.lock`)

## 2.2 Nav2 Control Chain
- Nav2 params include:
  - `velocity_smoother` enabled.
  - `collision_monitor` enabled (`cmd_vel_smoothed -> cmd_vel_safe`).
  - Controller/planner returned to conservative DWB + Navfn baseline for stability.
  - `progress_checker_plugins` naming corrected for current Nav2 API expectations.

## 2.3 Pico Bridge Hardening
- `cmd_vel_to_pico_bridge.py` includes:
  - velocity clamping
  - acceleration limiting
  - timeout-to-zero behavior
  - heartbeat + enable topics
  - process lock file (prevents multiple bridge processes on same host)
  - ROS graph duplicate-publisher detection
  - safe-stop publish before self-shutdown on duplicate detection

## 2.4 Diagnostics and Observability
- `diagnose_control_chain.py` added:
  - required topic + type checks
  - optional live-flow checks (`require_flow:=true`)
  - single-publisher requirement checks (`require_single_pico_publisher:=true`)
  - pass/fail exit codes for scripted use
- `record_control_chain_bag.py` added:
  - standardized topic capture for debugging/tuning
  - predictable output path naming by timestamp

## 2.5 Sensor Fusion Foundation
- `ekf_fusion.launch.py` added (optional).
- `ekf_2d_no_imu.yaml` added:
  - fuses `/pico/odom` into `/odometry/filtered`
  - `publish_tf: false` to avoid TF conflict with scan matcher at current stage
- `robot_localization` added to package runtime dependencies.

## 3) What Has Been Tested So Far

Note: this section reflects tests run during implementation sessions in this workspace.

## 3.1 Build and Syntax
- Python compile checks passed for updated scripts and launches.
- `colcon build --packages-select parking_system` passed repeatedly after each major change.

## 3.2 Launch Argument Validation
- `ros2 launch ... --show-args` verified for:
  - `nav2_live_slam.launch.py`
  - `rpi5_pico_bridge.launch.py`
  - `ekf_fusion.launch.py`

## 3.3 LiDAR Stability Root Cause and Recovery
- Observed failure signature:
  - `SL_RESULT_OPERATION_TIMEOUT`
  - RViz queue full warnings as downstream symptom.
- Root cause identified:
  - stale `sllidar_node` process holding `/dev/ttyUSB0`.
- Recovery validated:
  - stop stale processes
  - relaunch on stable by-id serial path
  - LiDAR health and scan mode recovered.

## 3.4 Control Chain Diagnostics
- Topic/type diagnostic run passed in topic-check mode.
- Live-flow diagnostic correctly fails when no command source is active (`require_flow:=true`), which is expected behavior.
- Live-flow diagnostic passes with active publishers.

## 3.5 Single-Publisher Guard
- Duplicate bridge instance test:
  - second instance blocked by lock file.
- Duplicate publisher simulation:
  - guard detected multiple publishers on `/pico/*`
  - bridge published safe stop
  - bridge shut itself down.

## 3.6 Rosbag Recorder
- `record_control_chain_bag.py` verified to start and subscribe to expected topics.
- Deprecated rosbag positional-topic warning was removed (`--topics` used).

## 3.7 Not Yet Fully Verified in Physical Closed Loop
- Full autonomous drive trials with repeated real vehicle motion.
- End-to-end e-stop service response on physical platform under motion.
- EKF impact on navigation behavior in real driving.
- Long-duration soak (>30 min) under continuous command traffic.

## 4) Known Risks / Gaps (Current)

1. Stale process risk still exists if old launches are left running manually.
2. No IMU connected yet, so fusion robustness is limited in dynamic maneuvers.
3. EKF is optional and not yet integrated as primary odometry source for Nav2.
4. Submodule `pico_firmware/micro_ros_sdk` has local changes outside the main repo commit chain (needs explicit submodule strategy).
5. Mobile control and Nav2 control can conflict if both are active without mode arbitration policy.
6. System currently relies on operator discipline for run-order and preflight steps.

## 5) Detailed Remaining Work Plan

Priority legend:
- P0 = required before reliable autonomous movement tests
- P1 = high-value reliability/performance
- P2 = optimization and polish

## 5.1 P0 — Operational Safety and Deterministic Bringup

1. Add automated preflight checker script (ports, stale processes, topic sanity)
   - Outcome: single command returns GO/NO-GO before launch.
   - Acceptance:
     - fails if `/dev/ttyUSB0` or Pico serial is locked unexpectedly
     - fails if duplicate `/pico/*` publishers detected.

2. Add launch-time startup ordering guardrails
   - Ensure micro-ROS agent + bridge come up deterministically when enabled.
   - Acceptance:
     - no race-caused false duplicate detections
     - repeatable bringup 10/10 runs.

3. Formalize control-source arbitration policy (Nav2 vs mobile joystick)
   - Introduce explicit mode ownership policy (`AUTO`, `MANUAL`, `ESTOP`).
   - Acceptance:
     - only one source can command drive at a time
     - mode transitions are logged and testable.

4. Verify emergency-stop path under motion
   - Confirm `/pico/estop` service behavior and motor disable timing.
   - Acceptance:
     - stop command execution within agreed latency threshold.

## 5.2 P1 — Fusion and Feedback Quality

1. Integrate EKF in staged manner
   - Stage A: run EKF side-by-side and monitor `/odometry/filtered`.
   - Stage B: switch selected consumers from `/odom` to filtered output if stable.
   - Acceptance:
     - drift reduction visible in repeated loop tests
     - no TF conflicts.

2. Add IMU integration when hardware is available
   - Update EKF config to fuse IMU yaw/angular velocity.
   - Acceptance:
     - improved heading stability during acceleration/deceleration.

3. Latency and jitter measurement workflow
   - Use standardized bags for command→feedback delay profiling.
   - Acceptance:
     - measured median and p95 reported and tracked.

## 5.3 P1 — Nav2 and Parking Behavior Quality

1. Tune collision monitor polygon behavior for chassis geometry
   - Validate stop/approach trigger distances.
2. Tune DWB parameters with real chassis dynamics
   - Reduce oscillation and improve goal convergence near parking positions.
3. Evaluate Ackermann-friendly planner/controller migration (future)
   - Candidate: Smac Hybrid + RPP.
   - Keep as controlled A/B test, not immediate switch.

## 5.4 P2 — Tooling, CI, and Documentation Consolidation

1. Add scripted regression test targets (`make` or shell wrappers)
2. Add bag replay test harness for repeatable validation without hardware
3. Consolidate stale/legacy documentation into archive references
4. Add release checklist per field test run

## 6) Test Plan From Now On (Step-by-Step)

## 6.1 Preflight (Every Run)

1. Verify serial-by-id devices are present.
2. Check no stale process holds LiDAR port.
3. Ensure no extra bridge process is active.
4. Run strict control chain topic check:
   `ros2 run parking_system diagnose_control_chain.py --ros-args -p expect_pico_bridge:=true -p require_single_pico_publisher:=true`

Pass criteria:
- no missing required topics
- no type mismatches
- exactly one publisher on `/pico/control_cmd`, `/pico/enable`, `/pico/heartbeat`.

## 6.2 LiDAR/SLAM Baseline Validation

Run:
- `ros2 launch parking_system nav2_live_slam.launch.py use_pico_bridge:=false serial_port:=/dev/serial/by-id/<lidar-id>`

Checks:
- `/scan` stable
- map updates continuously
- no repeated LiDAR timeout errors.

## 6.3 Full Control Chain Validation

Run:
- `ros2 launch parking_system nav2_live_slam.launch.py use_pico_bridge:=true enforce_single_publisher:=true serial_port:=/dev/serial/by-id/<lidar-id> pico_serial_port:=/dev/serial/by-id/<pico-id>`

Then:
- strict diagnostic:
  `ros2 run parking_system diagnose_control_chain.py --ros-args -p expect_pico_bridge:=true -p require_single_pico_publisher:=true`
- flow diagnostic (while sending command):
  `ros2 run parking_system diagnose_control_chain.py --ros-args -p expect_pico_bridge:=true -p require_single_pico_publisher:=true -p require_flow:=true -p window_s:=12.0`

Pass criteria:
- strict diagnostic passes
- flow diagnostic passes when goal/joystick command is sent.

## 6.4 Safety Behavior Tests

1. Command timeout test:
   - send motion command, then stop command stream.
   - verify bridge output ramps to zero and disable state updates.
2. Duplicate publisher test:
   - attempt second bridge startup.
   - verify it is blocked or self-terminates safely.
3. E-stop test:
   - invoke e-stop while moving.
   - verify immediate command zeroing and persistent safe state.

## 6.5 Data Capture For Each Field Session

Run recorder:
- `ros2 run parking_system record_control_chain_bag.py`

Required topics to review post-run:
- `/cmd_vel`, `/cmd_vel_smoothed`, `/cmd_vel_safe`
- `/pico/control_cmd`, `/pico/enable`, `/pico/heartbeat`
- `/pico/odom`, `/pico/joint_feedback`
- `/scan`, `/tf`, `/tf_static`.

## 7) Engineering Decision Gates (Require Your Approval)

These are the next decision points where implementation direction changes system behavior.

1. Odom source handoff strategy
   - Option A: keep scan-matcher `/odom` as primary until IMU arrives (recommended now).
   - Option B: switch Nav2 to EKF output immediately.

2. Control-source arbitration policy
   - Option A: explicit mode topic/service (`AUTO` vs `MANUAL`) before any joystick+Nav2 mixed use (recommended).
   - Option B: priority by “latest command wins”.

3. E-stop architecture
   - Option A: keep `/pico/estop` service + bridge timeout.
   - Option B: add hardware interlock layer only.

4. Planner/controller migration timing
   - Option A: keep DWB/Navfn until repeatable closed-loop tests pass (recommended).
   - Option B: move now to Smac/RPP and tune immediately.

5. Serial-port policy
   - Option A: enforce `/dev/serial/by-id` in launch defaults (recommended for deployment image).
   - Option B: keep `/dev/ttyUSB*` and override per run.

## 8) Immediate Next Sprint (Recommended)

Week 1:
1. Implement automated preflight checker (P0).
2. Implement control-source arbitration skeleton (P0).
3. Run 10 repeated bringup/teardown cycles with strict diagnostics.

Week 2:
1. Execute safety test matrix (timeout, duplicate guard, estop).
2. Run commanded motion profile tests and bag every run.
3. Evaluate EKF side-by-side metrics without switching primary odom yet.

Exit criteria for sprint completion:
- deterministic bringup success in repeated runs
- strict diagnostics pass reliably
- safety behaviors verified on physical platform
- baseline bag dataset collected for tuning.
