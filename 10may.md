# 10 May Test Checklist

## Build & Flash

1. **Build Pico firmware** (kick-start motor change)
   ```bash
   cd ~/autonexa/pico_firmware/build
   cmake .. && make
   ```

2. **Flash Pico** — hold BOOTSEL, plug USB, then:
   ```bash
   cp autonexa_pico.uf2 /media/$USER/RPI-RP2/
   ```

3. **Test motor kick-start** — open `pico_gui.py`, try:
   - `SPEED 15` → motors should spin
   - `SPEED 10` → should kick at 60% then drop to 30% (slower!)
   - `SPEED 5` → same kick, then 30% floor (slowest possible)
   - If motors stall after the kick, raise `MOTOR_MIN_RUN_PCT` from 30 → 35 or 40 in `config.h`, rebuild & reflash

4. **Build ROS2 workspace** (Nav2 params + bridge changes)
   ```bash
   cd ~/autonexa
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --cmake-args "-DCMAKE_PREFIX_PATH=/usr/local"
   source install/setup.bash
   ```

5. **Build Flutter app** (no app-side changes needed for nav fixes, but rebuild if you want latest)
   ```bash
   cd ~/autonexa/aruco_project/mobile_app
   flutter build apk --release
   ```
   Install on phone.

## Launch & Verify Params

6. **Launch full system**
   ```bash
   ros2 launch parking_system nav2_live_slam.launch.py
   ```

7. **Verify new params loaded** (spot-check in a second terminal)
   ```bash
   ros2 param get /controller_server FollowPath.desired_linear_vel
   # expect: 0.15
   ros2 param get /controller_server FollowPath.min_lookahead_dist
   # expect: 0.30
   ros2 param get /controller_server FollowPath.max_robot_pose_search_dist
   # expect: 1.0
   ```

8. **Check runtime overrides aren't stale** — if the speed slider was previously used, `~/.autonexa/runtime_overrides.yaml` may override `desired_linear_vel` back to 0.20 or 0.30. If so:
   ```bash
   cat ~/.autonexa/runtime_overrides.yaml
   # If it has a higher desired_linear_vel, either delete the file
   # or set the slider to 0.15 from the app
   ```

## Navigation Tests

9. **Test 1 — straight forward goal** (should work like before)
   - Tap a goal ~1m ahead, same heading
   - Robot should drive straight at ~0.15 m/s
   - Pass = reaches goal

10. **Test 2 — goal requiring a turn** (this is where it used to fail)
    - Tap a goal ~1m away, 90° to the side
    - Watch RViz: does the path have cusps (reverse segments)?
    - Watch the robot: does it follow the path?
    - Pass = robot follows path through the turn

11. **Test 3 — goal requiring opposite heading** (hardest case)
    - Tap a goal behind the robot, or facing opposite direction
    - This will force REEDS_SHEPP to generate a K-turn/cusp
    - Watch for "Cusp detected" in bridge logs:
      ```bash
      ros2 topic echo /rosout --field msg | grep -i cusp
      ```
    - Pass = robot pauses briefly at cusp, then drives correctly in new direction

12. **Test 4 — manual joystick regression** (make sure we didn't break anything)
    - Switch to MANUAL mode in app
    - Drive forward, backward, left, right
    - Test both SOFT and OFF safety modes
    - Pass = feels the same as before

## If Things Go Wrong

13. **If motors don't spin at all after flash** → `MOTOR_MIN_RUN_PCT` too low, bump to 40 in `config.h`, rebuild & reflash

14. **If robot still deviates on turns** → check `ros2 topic echo /cmd_vel_safe` during failure:
    - Is `linear.x` going negative? (reverse segments)
    - Is `angular.z` very large? (sharp turn demand)

15. **If robot hunts/oscillates near goal** → increase `xy_goal_tolerance` from 0.20 → 0.25 via:
    ```bash
    ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 0.25
    ```

16. **If robot seems too slow everywhere** → use the speed slider in app Settings to bump to 0.18-0.20

## Changes Summary (what's different from yesterday)

| What | Old | New |
|------|-----|-----|
| Cruise speed | 0.20 m/s | **0.15** |
| Lookahead | 0.45–0.55–0.80 m | **0.30–0.40–0.60** |
| Pose search dist | 5.0 m | **1.0** |
| Min curve speed | 0.18 m/s | **0.10** |
| Goal tolerance | 0.15 m | **0.20** |
| Smoother deadband | 0.05 m/s | **0.0** |
| Bridge min_vx_creep | 0.05 m/s | **0.02** |
| Cusp cooldown | (none) | **250 ms pause** |
| Motor deadband | flat 60% always | **60% kick 60ms → 30% sustain** |
