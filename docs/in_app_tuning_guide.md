# AutoNexa In-App Tuning Guide

Use the mobile app's Diagnostics -> Open param tuner for most of these.
Successful edits persist to `~/.autonexa/runtime_overrides.yaml`, so write
down known-good values before experimenting.

## Steering And Direction

`/nav2_pico_bridge.vx_polarity`

- Flips forward/back motor direction.
- Use only if "forward" drives the car backward, or reverse drives forward.
- Valid values: `1` or `-1`.

`/nav2_pico_bridge.servo_polarity`

- Flips all steering, both forward and reverse.
- Use only if forward-left and forward-right are wrong.
- Valid values: `1` or `-1`.

`/nav2_pico_bridge.reverse_steer_polarity`

- Flips steering only while `vx < 0`.
- Use if forward turns are correct but reverse-left/reverse-right are swapped.
- Default for this chassis: `-1`. If reverse still turns opposite, try `1`.

`/nav2_pico_bridge.max_steer_rate_radps`

- Limits how fast the steering servo is allowed to swing.
- Lower value = smoother, less twitchy steering.
- Higher value = follows sharp path changes faster, but may jerk.

## Path Following

`/controller_server.FollowPath.lookahead_dist`

- Main Regulated Pure Pursuit carrot distance.
- Lower value = follows the orange path more tightly.
- Too low = wobble, hunting, or over-correcting.

`/controller_server.FollowPath.min_lookahead_dist`

- Lower bound for the carrot at slow speeds.
- Lower this slightly if the car cuts corners in tight parking maneuvers.
- Raise it if the car oscillates near the path.

`/controller_server.FollowPath.max_lookahead_dist`

- Upper bound for the carrot on longer/faster paths.
- Lower this if the car arcs around the path too broadly.
- Raise it if long straight paths become nervous.

`/controller_server.general_goal_checker.xy_goal_tolerance`

- How close the robot must get to count as "arrived".
- Lower value = more exact parking.
- Too low = the car may hunt around the goal.

## Speed And Smoothness

Settings -> Nav2 Max Speed

- Sets `/controller_server.FollowPath.desired_linear_vel` and
  `/velocity_smoother.max_velocity[0]` together.
- Lower speed usually improves path tracking in the small testbed.

`/velocity_smoother.max_accel`

- How quickly speed ramps up.
- Lower first element if launches feel jumpy.
- Example: `[0.5, 0.0, 1.0]` is softer than `[1.5, 0.0, 2.0]`.

`/velocity_smoother.max_decel`

- How quickly speed ramps down.
- More negative first element stops harder.
- Less negative first element stops more gently.

`/nav2_pico_bridge.min_vx_creep`

- Speeds below this become `SPEED 0`.
- Raise if the car micro-lurches near the goal.
- Lower if tiny slow commands are being ignored too early.

## Obstacle And Wall Bubble Size

`/local_costmap/local_costmap.inflation_layer.inflation_radius`

- Local obstacle halo radius used by the controller.
- Precision-testbed default: `0.05` m.
- Lower value = walls/scan obstacles look less bulky and allow tighter motion.
- Too low = less safety margin around obstacles.

`/global_costmap/global_costmap.inflation_layer.inflation_radius`

- Global obstacle halo radius used by the planner.
- Precision-testbed default: `0.05` m.
- Keep close to the local value so planner and controller agree.

`inflation_layer.cost_scaling_factor`

- Higher value = inflated cost drops off faster, so halos look tighter.
- Lower value = inflated cost fades more gradually, so walls feel larger.

## Quick Symptom Map

- Forward steering wrong: flip `servo_polarity`.
- Reverse steering wrong but forward is correct: flip `reverse_steer_polarity`.
- Robot drives backward when commanded forward: flip `vx_polarity`.
- Car goes around the path too widely: lower RPP lookahead values a little.
- Car wiggles or hunts: raise lookahead or goal tolerance slightly.
- Obstacle bubbles are too large: lower inflation radius slightly or raise cost scaling.
- Car gets too close to walls: raise inflation radius or lower cost scaling.
