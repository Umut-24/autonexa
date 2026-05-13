# AutoNexa — Teacher Demo Plan (next week)

## Goal

Demonstrate end-to-end intelligent parking + recall against the
project brief (`~/Downloads/Intelligent_Parking_System.pdf`):
> Park into a designated space, summon back to user-defined position,
> all driven from the mobile app over Wi-Fi, with manual E-STOP.

We have a working integrated system as of HEAD `5f020f2`. The one
thing the brief asks for that we don't have wired yet is the
**Park / Summon UX** in the app. That is the only must-build for
demo. Encoders + ±5 cm closed-loop accuracy are stretch.

---

## What we already have (no work needed)

- **Hardware**: Pico WH + L298N + 2× JGB37 motors, calibrated servo
  on GP15, SLAMTEC C1 LiDAR, RPi5/Ubuntu 24.04/ROS2 Jazzy.
- **Pico firmware** (`autonexa_pico.uf2`): `SPEED`, `SERVO_PWM`,
  `ENABLE`/`DISABLE`, `ESTOP`/`ESTOP_CLEAR`. 60% deadband floor for
  static-friction kick.
- **Nav2 stack**: `nav2_live_slam.launch.py` brings up LiDAR +
  laser_scan_matcher (odom from ICP) + SLAM Toolbox (live mapping)
  + Nav2 (planner / DWB controller / smoother / collision_monitor)
  + the new ASCII bridge to Pico + the mobile bridge.
- **Mobile bridge** (`ros2_mobile_bridge.py`): WebSocket
  `/ws/control` + `/ws/telemetry` (10 Hz push), AUTO/MANUAL/ESTOP
  state machine via `/api/mode`, Nav2 action status, `/api/nav_goal`,
  `/api/cancel_nav`, `/api/map` (ETag), `/api/scan`, `/video_feed`.
- **Flutter app**: Control (joystick + E-STOP + ModeBar), Map
  (occupancy grid + scan overlay + planner path + goal indicator +
  tap-to-navigate), Parking (markers + missions), Settings.

Everything below assumes the chassis can already drive manually and
follow a Nav2 goal you tap on the map. Verified end-to-end.

---

## Demo storyboard (the 5-minute pitch)

1. **Stage**: 2 m × 2 m floor area, 2-3 cardboard "parking spots"
   marked with tape; chassis placed at one corner; phone in hand.
2. **Build a map** (~30 s): switch ModeBar to **MANUAL**, joystick
   the chassis around the floor while pointing at the live map
   filling in on the Map tab. Narrate: *"SLAM is building the map
   from LiDAR scans in real time."*
3. **Switch to AUTO**: tap **AUTO** on ModeBar. Joystick is now
   inert (state-machine gated).
4. **PARK**: open Parking tab → Spots → tap **Park here** on Spot A.
   Narrate: *"Spot coordinates pre-defined for the testbed; the app
   sends a Nav2 goal."* Chassis plans → drives → stops at spot.
5. **SUMMON**: back to Control tab → tap **Summon Home**. Chassis
   drives back to the home pose set earlier.
6. **E-STOP**: while it's driving, tap the red **E-STOP** button.
   Chassis halts within 200 ms (firmware watchdog backstop). ModeBar
   flips to ESTOP. Tap **release-E-STOP** → ModeBar back to AUTO,
   chassis can be commanded again.

If any step fails mid-demo, the **manual-mode joystick is always one
tap away** — switch to MANUAL and drive the chassis out of trouble.

---

## What we need to build for the demo

Single PR. Estimated 1 day of focused work, can be split between
two people.

### 1. `src/parking_system/config/parking_spots.yaml` (NEW)

```yaml
home: {x: 0.0, y: 0.0, yaw: 0.0}
spots:
  - {name: "A", x: 1.0, y: 0.5, yaw: 0.0}
  - {name: "B", x: 1.5, y: 1.5, yaw: 1.5708}
  - {name: "C", x: 0.5, y: 1.0, yaw: -1.5708}
```

User edits to match their actual testbed layout.

### 2. Bridge endpoints — `src/parking_system/scripts/ros2_mobile_bridge.py`

Add at startup:
- `_load_parking_spots()`: read YAML from package share.
- `_save_parking_spots()`: write YAML back (for `POST /api/home`).

Add HTTP routes:
- `GET  /api/parking_spots` → `{spots: [{name, x, y, yaw}, ...]}`.
- `GET  /api/home` → `{x, y, yaw}`.
- `POST /api/home` body `{x, y, yaw}` → updates in-memory + writes
  YAML back. Empty body → snapshot current robot pose as home.

All Park / Summon actions reuse the existing `POST /api/nav_goal`.
No new Nav2 plumbing.

### 3. Flutter app

- **`lib/models/parking_spot.dart`** (NEW): `ParkingSpot {name, x,
  y, yaw}` + `fromJson`.
- **`lib/services/connection_service.dart`**:
  - `Future<List<ParkingSpot>> fetchParkingSpots()`.
  - `Future<NavGoal?> fetchHome()`.
  - `Future<bool> setHome({NavGoal? pose})` (null pose ⇒ snapshot
    current robot pose server-side).
  - `Future<bool> summonHome()` and `Future<bool> parkAt(ParkingSpot
    spot)` — both call `sendNavGoal` under the hood.
- **`lib/tabs/control_tab.dart`**: add prominent **Summon Home**
  button next to E-STOP (gated to AUTO mode).
- **`lib/tabs/parking_tab.dart`**: add a **Spots** sub-tab next to
  Markers, listing YAML spots with a **Park here** button per row.
- **`lib/tabs/settings_tab.dart`**: add a **Save Current Pose as
  Home** button (calls `setHome(pose: null)`).

### 4. Optional polish (do only if Phase-1 finishes early)

- Map tab: draw small icons for the parking spots (squares) and
  home (house). Pulled from the same `/api/parking_spots` and
  `/api/home`.
- Long-press on map → "Set this as home" via dialog.

---

## Build / install timeline

| Day | Task | Owner |
|---|---|---|
| Mon | parking_spots.yaml + bridge endpoints + curl smoke test | Backend |
| Mon evening | App: model + connection_service methods | Frontend |
| Tue | App: Control + Parking + Settings UI; build + sideload APK | Frontend |
| Tue evening | Full dry run (one person on app, one on chassis) | Both |
| Wed | Fix anything wrong from dry run | Whoever finds it |
| Wed evening | Second full dry run | Both |
| Thu | Polish; record a backup screen-capture of a successful run | Both |
| Fri | Final dry run morning. Demo afternoon. | — |

Buffer: Wed-Thu intentionally light to handle real life.

---

## Pre-demo checklist (run morning-of)

- [ ] Pico flashed with current `autonexa_pico.uf2` (HEAD's build).
      `cp pico_firmware/build/autonexa_pico.uf2 /media/$USER/RPI-RP2/`
- [ ] L298N motor battery topped up; servo 6 V buck on.
- [ ] LiDAR USB plugged → `/dev/ttyUSB0`.
- [ ] Pico USB plugged → `/dev/ttyACM0`.
- [ ] Phone on same Wi-Fi as RPi5; can ping `<rpi5-ip>`.
- [ ] `parking_spots.yaml` matches today's actual tape layout.
- [ ] Home pose either committed in YAML *or* set via app at the
      tape position you'll start the demo from.
- [ ] Wheels off ground; quick `python3 test/pico_gui.py` sanity:
      ENABLE → hold W → both motors spin → release → STOP. Then
      kill the GUI before the demo (it owns `/dev/ttyACM0`).
- [ ] `ros2 launch parking_system nav2_live_slam.launch.py` →
      wait for `Managed nodes are active` and `Bridge up [live]`.
- [ ] Phone connects to `<rpi5-ip>:5000` → green indicator → map
      appears within 5 s after pushing chassis around.

---

## Demo script (read-this-during-demo cheatsheet)

```
1. (introduce) "Pi 5 + Pico + L298N + LiDAR; ROS2 Jazzy + Nav2."
2. (open app, show Control tab) "Manual mode — virtual joystick."
   joystick a few seconds → wheels move proportionally.
3. (open Map tab) "SLAM Toolbox builds the map live from LiDAR."
   joystick to push around briefly; the map fills.
4. (ModeBar → AUTO) "Switching to autonomous mode."
   joystick is now disabled; pill turns green.
5. (Parking tab → Spots → tap A → "Park here" → confirm)
   "App sends a Nav2 goal; planner draws the orange path; chassis
   follows it autonomously."
6. (back to Control → tap "Summon Home")
   "Same path planner, target is our saved home pose."
7. (mid-drive: hit big red E-STOP) "Hardware-style E-STOP via app —
   200 ms watchdog on the firmware backs it up." chassis halts.
8. (tap release-E-STOP) "Cleared; ready for the next command."
9. (close) "Spec compliance: <0.5 m/s, app-only commands after
   start, manual E-STOP, 2x2 m testbed, indoor stable lighting."
```

---

## Contingency plans (when something breaks)

| Failure | What you'll see | Fix during demo |
|---|---|---|
| LiDAR didn't enumerate | No map dots in app | Unplug/replug LiDAR USB, relaunch in Terminal 1 |
| Pico locked port | `open /dev/ttyACM0 failed` | `pkill -f pico_gui.py; pkill -f nav2_pico_bridge`, relaunch |
| Goal sent but chassis doesn't move | Path drawn, motors silent | Switch ModeBar → MANUAL → joystick to verify motors. Then back to AUTO. Often a stale pose; `Save Current Pose as Home` from Settings to refresh, retry |
| Servo lurches wrong direction | Wheels going opposite to expected | Relaunch with `servo_polarity:=1` (or back to `-1`) |
| Chassis overshoots Spot A | Stops outside the tape square | Acceptable for demo — current goal tolerance is 15 cm, ±5 cm is the encoder-stretch goal. Narrate it. |
| App lost connection | Indicator red | Phone Settings tab → re-enter `<rpi5-ip>:5000` → Connect. Persistent socket reconnects with backoff anyway |
| Everything on fire | Anything | Hit the physical Pico USB unplug. Watchdog stops motors in 200 ms. Recover, relaunch. |

---

## Stretch: encoders for ±5 cm parking accuracy

Out of demo scope **unless Mon-Wed work finishes a day early**.
JGB37-520R30 has built-in quadrature Hall encoders. To wire:

- Pico GP10/11 → left motor encoder A/B.
- Pico GP12/13 → right motor encoder A/B.
- Encoder VCC: 3V3 from Pico.
- Encoder GND: common ground.

Firmware:
- `pico_firmware/src/encoders.{c,h}`: ISR-based quadrature counting,
  4 atomic 32-bit accumulators (one extra per channel for safety).
- Add encoder ticks to the `TEL` line at 10 Hz.
- Add a closed-loop PI layer: vx command → encoder-fed PI → PWM
  duty (instead of the current deadband-floor mapping).
- Lower `MOTOR_DEADBAND_PCT` to ~20%; the integrator handles the
  static-friction kick.

Bridge:
- Parse encoder ticks from `TEL`, publish a true `/odom` from the
  Pico instead of relying on laser_scan_matcher's ICP.

Then in `nav2_navigation_params.yaml`:
- `xy_goal_tolerance: 0.05` (was 0.15).
- `min_vel_x: 0.0` and remove the bridge's `min_vx_creep` gate.

Estimated 2 days. Defer if Mon-Wed slips.

---

## Spec compliance scorecard at demo time

What you can claim with a straight face after Phase 1 ships:

| Spec bullet | Status at demo |
|---|---|
| Detect obstacles, boundaries, parking spaces | ✅ LiDAR + costmap; spaces from YAML |
| Autonomous park into designated space | ✅ via "Park here" button → Nav2 |
| Summon to user-defined position | ✅ via "Summon Home" → Nav2 |
| Trajectory generation, smooth/safe nav | ✅ Nav2 NavfnPlanner + DWB + smoother + collision_monitor |
| Mobile app commands + status (Wi-Fi) | ✅ Flutter + Flask/WebSocket bridge |
| 2 m × 2 m testbed | ✅ costmap 2 cm res, parking-tuned |
| < 0.5 m/s | ✅ max_vel_x = 0.30 |
| ±5 cm parking accuracy | ⚠️ ~15 cm with current open-loop. Encoder stretch closes the gap. |
| 5 cm obstacle detection | ✅ verify on demo day with a small box |
| Indoor stable lighting | ✅ environment-only |
| 15 min battery | ❓ measure during dry run; report number |
| All-autonomous after command | ✅ |
| < 30 cm chassis | ❓ tape-measure on demo day |
| Wireless ≥ 3 m | ✅ Wi-Fi LAN |
| Manual E-STOP (hw or app) | ✅ app red button + firmware backstop |

Be honest about the ⚠️ items if asked.

---

## Coordination

Anıl's last commit (`5f020f2`) is the WebSocket + ModeBar overhaul.
Phase 1 above (Park/Summon UX) adds new app-side surface — ping him
before opening the PR so we don't duplicate model classes /
ConnectionService methods.
