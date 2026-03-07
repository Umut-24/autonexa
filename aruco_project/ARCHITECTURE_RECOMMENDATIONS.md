# AutoNexa Autonomous Parking System - Architecture Recommendations

**Project Context:** 30×20cm autonomous vehicle with LiDAR + camera for parking in 2m×2m testbed using ArUco marker detection and navigation.

---

## 1. PATH FINDING & NAVIGATION (ArUco + Vision)

### Current Implementation Analysis
Your `aruco_server.py` currently:
- Detects target ArUco marker by ID
- Computes distance (cm) and bearing (angle) via PnP pose estimation
- Provides real-time guidance (turn left/right, go straight)
- Outputs to mobile app via WebView

**Strengths:**
✓ Simple, real-time marker detection
✓ Relative positioning (bearing + distance) is appropriate for short-range guidance
✓ Calibration support for marker size accuracy
✓ WebView telemetry streaming is lightweight

**Limitations & Improvements:**

### 1.1 Enhance Path Planning for 30cm Car
**Issue:** Current guidance is purely reactive (bearing + distance). For a 30cm car navigating a 2m×2m space, you need **trajectory planning** and **collision avoidance**.

**Recommendation:**
1. **Add odometry tracking** on the mobile app to estimate car position over time:
   - Use car wheel encoder feedback (via ROS2 to Raspberry Pi)
   - Combine with bearing/distance from camera
   - Maintain local SLAM-like state on Pi

2. **Implement simple path smoothing:**
   - Replace hard angle transitions with **B-spline or clothoid curves**
   - Prevents jerky steering (especially for small wheeled vehicles)
   - Reduces skidding on linoleum/tiles in testbed

3. **Add parking maneuver logic:**
   - When `distance < 50cm` and marker is centered: execute **reverse-parallel park** routine
   - Three-point turn logic if misaligned
   - Define success criteria: `distance < 5cm AND |bearing| < 2°`

**Code improvement (conceptual):**
```python
def generate_parking_trajectory(car_width_cm=30, marker_x_cm, marker_y_cm, marker_bearing_deg):
    """Generate smooth path to park next to marker"""
    # 1. Check if aligned within 10° and within 100cm
    # 2. If aligned: straight approach
    # 3. If misaligned: compute arc trajectory to realign
    # 4. Final: dock maneuver (reverse slowly, center on marker)
    pass
```

### 1.2 Multi-Marker Sequencing
**Issue:** Your app supports "All-IDs mode" but server doesn't plan multi-marker routes.

**Recommendation:**
1. **Store all detected markers in frame:**
   ```python
   # In your generate_frames loop:
   detected_markers = {}  # {id: {'pos': (x,y), 'bearing': angle, 'dist': cm}, ...}
   # For each marker in frame, store its relative pose
   ```

2. **Implement simple waypoint system:**
   - User selects destination ID (parking spot)
   - App calculates: "go via marker 5 → then marker 12 → then park at marker 8"
   - Priority: avoid other cars (if your project has dynamic obstacles)

### 1.3 Camera Calibration Improvements
**Current:** Dummy calibration matrix + per-distance scaling factor.

**Recommendation for production testbed:**
```python
# Before deployment:
# 1. Run camera calibration once using checkerboard pattern
# 2. Save calibration matrices to config file
# 3. Load them instead of dummy values

# In testbed: Perform field calibration:
# - Place marker at known distances (30cm, 50cm, 100cm)
# - Record measured distances vs detected distances
# - Fit calibration curve (often linear or polynomial)
```

---

## 2. SENSOR FUSION: LiDAR + Camera

### Architecture Decision: Where to Fuse?

**Option A: On Raspberry Pi (ROS2 Recommended)**
- **Pro:** Real-time processing, tighter loop control, lower latency
- **Con:** Pi has limited CPU (4 cores, ~2GB RAM shared with video)
- **Best for:** Parking spot detection, obstacle avoidance

**Option B: On Mobile App (not recommended)**
- **Pro:** Offload Pi, real-time visualization
- **Con:** Network latency kills perception loop, complex multi-sensor sync
- **Avoid:** Latency too high for closed-loop control

### 2.1 Recommended ROS2 Pipeline on Raspberry Pi

```
Camera (ArUco)     LiDAR (2D)
    ↓                 ↓
    └─→ [Sensor Fusion Node] ←─┘
            ↓
    [Occupancy Grid / Marker Map]
            ↓
    [Path Planner + MPC Controller]
            ↓
    [Motor Commands]
            ↓
        Car
```

**Setup on Raspberry Pi 5 (4GB):**

1. **ROS2 Humble** (lightweight distro optimized for Pi)
   - Install lightweight version: `ros-humble-core` only
   - Disable unnecessary components (rviz, gazebo run server-side only)

2. **Sensor Fusion Strategy:**
   - **Camera (Fast, 30 FPS):** Provides bearing/distance to parking spots (ArUco IDs)
   - **LiDAR (10 Hz):** Provides obstacle map, wall distances
   - **Fusion:** Combine in occupancy grid:
     ```
     - LiDAR: wall locations, obstacles
     - Camera: target marker location + precise position estimate
     - Output: "Marker at (X, Y) in global frame, nearest obstacle at (X', Y')"
     ```

3. **Implementation (ROS2 nodes):**
   ```
   • camera_aruco_node: publishes /target_marker (id, pose)
   • lidar_node: publishes /scan (laser_scan msg)
   • fusion_node: subscribes to both, publishes /occupancy_grid + /robot_pose
   • navigation_node: subscribes to grid + target, commands motor controller
   ```

**Key Advantage:** Decouples UI from control loop. Mobile app queries slow ROS2 services for visualization, while car uses fast local feedback.

---

## 3. MOBILE APP UI: Map + Camera Overlay

### Current Status
- WebView shows video feed
- Telemetry shows bearing/distance
- Missing: **map context + localization visualization**

### 3.1 New UI Layout (Recommended)

```
┌─────────────────────────────────────┐
│  [RViz 2D Map from Raspberry Pi]    │
│  - Shows walls (LiDAR)              │
│  - Shows parking spots (ArUco IDs)  │
│  - Shows car position (small icon)  │
│                                      │
│  [Small Camera Feed Overlay]        │ ← NEW
│  (pinned at car location on map)    │
│                                      │
│  Selected Target: Spot #5           │
│  Distance: 120cm, Bearing: -15°     │
└─────────────────────────────────────┘

Bottom Controls:
[Pre-select ID] [All-IDs] [Navigate] [Calibrate]
```

### 3.2 Implementation Strategy

**Phase 1: Static map + camera feed (3-4 hours)**
1. Receive map image from ROS2 (occupancy grid as PNG)
2. Display in Flutter using `Image.network()`
3. Overlay car icon at estimated position
4. Pin camera feed as small video widget

**Phase 2: Real-time localization (2-3 days)**
1. ROS2 node publishes robot pose (`/tf` or custom message)
2. Mobile app subscribes to pose updates
3. Animate car icon moving on map

**Phase 3: Camera overlay with detected markers (1 day)**
1. Detect ArUco markers in camera feed on Pi
2. Project them onto map display
3. Show AR-style annotations

### 3.3 Flutter Implementation Outline

```dart
// New widget: MapWithOverlay
class MapWithCameraOverlay extends StatefulWidget {
  @override
  State<MapWithCameraOverlay> createState() => _MapWithCameraOverlayState();
}

class _MapWithCameraOverlayState extends State<MapWithCameraOverlay> {
  // Receive map image from ROS2 HTTP server
  Image? mapImage;
  RobotPose? robotPose;
  List<ParkingSpot> spots = [];

  @override
  void initState() {
    super.initState();
    _pollMapUpdates();
    _pollPoseUpdates();
  }

  void _pollMapUpdates() async {
    while (true) {
      try {
        final mapUrl = 'http://raspberry_pi:5000/map_image';
        final resp = await http.get(Uri.parse(mapUrl));
        if (resp.statusCode == 200) {
          setState(() => mapImage = Image.memory(resp.bodyBytes));
        }
      } catch (_) {}
      await Future.delayed(Duration(milliseconds: 500));
    }
  }

  void _pollPoseUpdates() async {
    while (true) {
      try {
        final resp = await http.get(Uri.parse('http://raspberry_pi:5000/robot_pose'));
        if (resp.statusCode == 200) {
          final json = jsonDecode(resp.body);
          setState(() => robotPose = RobotPose.fromJson(json));
        }
      } catch (_) {}
      await Future.delayed(Duration(milliseconds: 100));
    }
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        // Background map
        if (mapImage != null) mapImage! else Placeholder(),
        
        // Car icon
        if (robotPose != null)
          Positioned(
            left: robotPose!.x,
            top: robotPose!.y,
            child: Transform.rotate(
              angle: robotPose!.theta,
              child: Icon(Icons.directions_car, size: 24, color: Colors.green),
            ),
          ),
        
        // Parking spots
        ...spots.map((spot) => Positioned(
          left: spot.x, top: spot.y,
          child: Container(
            width: 16, height: 16,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: spot.id == selectedId ? Colors.red : Colors.blue,
            ),
            child: Center(child: Text('${spot.id}', style: TextStyle(fontSize: 10))),
          ),
        )),
        
        // Camera feed (small overlay)
        Positioned(
          bottom: 10, left: 10,
          child: Container(
            width: 120, height: 90,
            decoration: BoxDecoration(border: Border.all(color: Colors.white)),
            child: WebViewWidget(controller: _webViewController),
          ),
        ),
      ],
    );
  }
}
```

**Data structures needed from Pi:**
```python
# On Raspberry Pi, add these endpoints:

@app.route('/map_image')
def get_map_image():
    """Return current occupancy grid as PNG"""
    # ROS2 node publishes grid → saved to /tmp/occupancy_grid.png
    return send_file('/tmp/occupancy_grid.png')

@app.route('/robot_pose')
def get_robot_pose():
    """Return current robot pose (x, y, theta)"""
    return jsonify({
        'x': robot_x_px,  # in map pixels
        'y': robot_y_px,
        'theta': robot_theta_rad,
        'timestamp': time.time()
    })

@app.route('/parking_spots')
def get_parking_spots():
    """Return all detected parking spots (ArUco markers)"""
    return jsonify([
        {'id': 1, 'x': 150, 'y': 200, 'detected': True},
        {'id': 5, 'x': 320, 'y': 180, 'detected': True},
        # ...
    ])
```

---

## 4. IMPROVED PATH: Realism Check

### Your current path (Estimated):
Camera detects marker → compute bearing + distance → send command to car

### Issues for realistic testbed:
1. **No odometry** → car drifts if visual updates delayed
2. **No obstacle avoidance** → car will hit walls or other cars
3. **No multi-goal planning** → can't chain "go to spot A, then B, then park"
4. **Jerky steering** → hard angle transitions

### Improved realistic path:
```
Car's internal loop (10 Hz, on Raspberry Pi):
├─ Read wheel encoders (odometry)
├─ Read LiDAR (obstacle map)
├─ Read camera (ArUco marker pose)
├─ Fuse all three in EKF (Extended Kalman Filter)
├─ Update robot pose estimate
├─ Compute smooth path to goal (RRT* or hybrid A*)
├─ Generate steering + speed commands
└─ Execute motor control

Mobile app (0.5 Hz, on phone):
├─ Receive fused map + pose
├─ Display visualization
├─ Accept user input (select parking spot)
└─ Send high-level commands ("park at spot 5")
```

---

## 5. ROS2 Recommended Packages

**Install on Raspberry Pi 5:**
```bash
sudo apt install ros-humble-core
sudo apt install ros-humble-common-msgs
sudo apt install ros-humble-geometry2
sudo apt install ros-humble-nav2-bringup  # Path planning

# Optional (if space allows):
sudo apt install ros-humble-diagnostics
```

**Custom nodes to write:**
- `aruco_detector_node` (publishes detected markers)
- `lidar_processor_node` (processes 2D LiDAR into occupancy grid)
- `sensor_fusion_node` (EKF to combine odometry + camera + LiDAR)
- `path_planner_node` (RRT* or hybrid A* for parking maneuver)
- `web_interface_node` (HTTP REST to serve map + pose for mobile app)

---

## 6. Mobile App UI Roadmap

**Immediate (1-2 days):**
- [x] Connection to server IP
- [x] Camera feed with ArUco detection
- [x] Telemetry display (distance, bearing)
- [x] ID selection + pre-select
- [x] Custom calibration
- [ ] **Map display with car icon**
- [ ] **Camera feed overlay**

**Short-term (3-5 days):**
- [ ] Real-time pose updates from ROS2
- [ ] Parking spot markers on map
- [ ] Visual path display (planned route)
- [ ] Sensor diagnostics panel

**Medium-term (1-2 weeks):**
- [ ] RViz remote (lightweight 2D renderer on phone)
- [ ] Multi-camera support (if adding Pi Camera + USB camera)
- [ ] Motion planning visualization
- [ ] Replay/logging of past runs

---

## 7. Quick Start: First Integration

**Week 1 goals:**
1. Get ROS2 running on Pi with sensor publishers
2. Implement fusion node (combine odometry + camera bearing)
3. Add `/map_image` and `/robot_pose` endpoints
4. Update Flutter app to fetch and display map + car icon

**Minimum viable demo:**
- Car drives using camera guidance (current)
- App shows map with car position (new)
- App shows parking spot markers (new)
- User can select destination and app guides them

---

## Summary Table

| Aspect | Current | Recommended | Effort |
|--------|---------|-------------|--------|
| **Path Finding** | Reactive bearing+dist | Smooth trajectory + parking maneuver | 3-4 days |
| **Localization** | Camera only | Fuse odometry+LiDAR+camera (EKF) | 5-7 days |
| **Sensor Fusion** | None | ROS2 on Pi | 2-3 days |
| **App UI** | Video + telemetry | Map + camera overlay + pose | 2-3 days |
| **Control Loop** | Manual per frame | ROS2 + Nav2 stack | 1-2 weeks |

**Total realistic timeline: 3-4 weeks** for a robust autonomous parking demo.

