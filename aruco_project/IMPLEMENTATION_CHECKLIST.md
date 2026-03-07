# AutoNexa Implementation Checklist

## ✅ Phase 0: Current (Complete)

### Mobile App
- [x] Custom calibration distance input (replaces hardcoded 50cm)
- [x] Pre-select detection ID feature
- [x] All-IDs mode with telemetry tracking
- [x] ID switching on-the-road (lock onto different marker)
- [x] Android network security config (allows HTTP)
- [x] Release APK built and tested (46.2 MB)

### Server
- [x] ArUco marker detection (IDs 0-15)
- [x] Real-time pose estimation (bearing + distance)
- [x] Guidance commands (turn left/right, go straight)
- [x] Custom calibration support
- [x] Telemetry streaming (SSE)
- [x] WebView integration
- [x] Enhanced server with map endpoints

### Documentation
- [x] Architecture recommendations document
- [x] Integration guide for UI
- [x] ROS2 sensor fusion template
- [x] Project summary
- [x] Quick reference card
- [x] Visual summary

### Ready to Test
- [x] Server deployable on PC or Raspberry Pi
- [x] Mobile app installable on Android phone
- [x] All features tested locally
- [x] Network config allows HTTP to local server

---

## ⏳ Phase 1: Map UI Integration (1-2 hours)

Choose one path:

### Option A: Minimal Integration (5 minutes)
```dart
// Just display map image
Image.network('${baseUrl}/map_image')
```
- [ ] Verify `/map_image` endpoint works in browser
- [ ] Test Image.network() in app
- [ ] See parking spots display
- **Time:** 5-10 min

### Option B: Full Integration (30-60 minutes)
```dart
// Use new MapWithCameraOverlay component
MapWithCameraOverlay(baseUrl: baseUrl)
```
- [ ] Copy `lib/map_overlay.dart` to project
- [ ] Import new components in `main.dart`
- [ ] Replace old UI with new layout
- [ ] Add telemetry panel
- [ ] Test map + camera overlay
- [ ] Test robot position updates
- [ ] Rebuild APK
- **Time:** 1-2 hours

### Either Way
- [ ] Place ArUco markers in camera view
- [ ] Verify markers appear as blue circles on map
- [ ] Verify robot position shows as green dot
- [ ] Verify camera feed displays in corner
- [ ] Test telemetry display updates
- [ ] Deploy to phone for testing

---

## ⏳ Phase 2: Path Planning Improvements (2-3 days)

### Trajectory Smoothing
- [ ] Replace hard angle transitions with B-spline curves
- [ ] Implement clothoid spiral for smooth steering
- [ ] Test on testbed - verify car doesn't jerk/slip
- [ ] Tune acceleration/deceleration profiles

### Parking Maneuver Logic
- [ ] Add three-point turn algorithm
- [ ] Detect parking position (bearing + distance within threshold)
- [ ] Implement reverse maneuver
- [ ] Add success criteria (distance < 5cm, bearing < 2°)
- [ ] Test parking on actual car

### Multi-Marker Sequencing
- [ ] Store all detected markers in global map
- [ ] Add waypoint planning (route through multiple spots)
- [ ] Implement simple A* or Dijkstra for waypoint order
- [ ] Test with 3+ markers in view

### Testing
- [ ] Test on 2m×2m testbed with actual car
- [ ] Record video of smooth vs jerky paths
- [ ] Measure accuracy at dock
- [ ] Document any issues

---

## ⏳ Phase 3: Odometry Integration (3-4 days)

### Hardware Setup
- [ ] Connect wheel encoders to Raspberry Pi GPIO
- [ ] Test encoder signal reading
- [ ] Calibrate encoder counts per cm
- [ ] Add IMU (optional but recommended)

### Odometry Node
- [ ] Create odometry publisher node
- [ ] Calculate wheel velocity from encoders
- [ ] Estimate pose change between frames
- [ ] Publish to `/odom` topic
- [ ] Test accuracy over known distance

### Testing
- [ ] Drive car 1 meter straight - measure drift
- [ ] Drive car in circle - measure error
- [ ] Compare camera pose vs odometry pose
- [ ] Note maximum drift over time

---

## ⏳ Phase 4: LiDAR Integration (4-5 days)

### Hardware Setup
- [ ] Connect LiDAR to Raspberry Pi
  - [ ] RPLiDAR: USB connection
  - [ ] VL53L0X: I2C connection
  - [ ] Other: follow manufacturer guide
- [ ] Test LiDAR data reading
- [ ] Verify scan ranges are reasonable

### LiDAR Processor Node
- [ ] Read LiDAR scan messages
- [ ] Convert polar (angle, range) to cartesian (x, y)
- [ ] Create occupancy grid (2cm cells)
- [ ] Mark obstacles at range points
- [ ] Publish OccupancyGrid to `/map` topic
- [ ] Save grid as PNG for visualization

### Obstacle Detection
- [ ] Identify walls (constant ranges)
- [ ] Identify free space (no returns)
- [ ] Mark safety margin around obstacles
- [ ] Test on actual testbed walls

### Testing
- [ ] Verify occupancy grid shows walls correctly
- [ ] Check grid resolution (2cm cells recommended)
- [ ] Validate against manual measurements
- [ ] Test obstacle detection accuracy

---

## ⏳ Phase 5: Sensor Fusion (2-3 weeks)

### ROS2 Setup on Raspberry Pi 5
- [ ] Install Ubuntu 22.04 on Pi
- [ ] Install ROS2 Humble (lightweight build)
- [ ] Create workspace: `~/autonex_ws`
- [ ] Build test packages: `colcon build`
- [ ] Test installation: `ros2 node list`

### Camera Node
- [ ] Copy `camera_aruco_node.py` from template
- [ ] Adapt to your camera hardware
- [ ] Publish marker poses to `/camera/marker_pose`
- [ ] Test node output: `ros2 topic echo /camera/marker_pose`

### LiDAR Node
- [ ] Copy `lidar_processor_node.py` from template
- [ ] Adapt to your LiDAR hardware
- [ ] Publish occupancy grid to `/map`
- [ ] Test node output: `ros2 topic echo /map`

### Sensor Fusion (EKF) Node
- [ ] Copy `sensor_fusion_node.py` from template
- [ ] Subscribe to camera, LiDAR, odometry
- [ ] Implement EKF update equations
- [ ] Tune Q and R matrices
- [ ] Publish fused pose to `/robot_pose`
- [ ] Verify fusion quality
- [ ] Test with all sensors running

### Web Server Bridge Node
- [ ] Copy `web_server_node.py` from template
- [ ] Subscribe to ROS2 topics
- [ ] Serve data via HTTP (for mobile app)
- [ ] Endpoints:
  - [ ] `/robot_pose` → JSON
  - [ ] `/map_image` → PNG
  - [ ] `/parking_spots` → JSON
- [ ] Test endpoints from phone

### Path Planner Node
- [ ] Implement RRT* or Hybrid A* algorithm
- [ ] Subscribe to goal and occupancy grid
- [ ] Generate collision-free path
- [ ] Publish path to `/plan` topic
- [ ] Test on various scenarios

### Motor Controller Node
- [ ] Implement MPC (Model Predictive Control)
- [ ] Subscribe to path and current pose
- [ ] Calculate steering + speed commands
- [ ] Publish to motor hardware
- [ ] Test closed-loop control

### Integration Testing
- [ ] Launch all nodes: `ros2 launch autonex_bringup bringup.launch.py`
- [ ] Verify all topics publishing data
- [ ] Check latencies
- [ ] Monitor CPU/memory usage
- [ ] Test end-to-end (app → Pi → car → motion)

---

## Testing Checklist

### Unit Tests (Each Phase)
- [ ] Marker detection accuracy (>95%)
- [ ] Pose estimation accuracy (<5cm error)
- [ ] Distance measurement (±10% error)
- [ ] Bearing accuracy (<5° error)

### Integration Tests
- [ ] Server ↔ Mobile app connection
- [ ] Video stream quality
- [ ] Telemetry update rate (>10 Hz)
- [ ] Map generation and display
- [ ] Parking spot detection

### System Tests (Full Stack)
- [ ] Car can detect target marker
- [ ] Car can navigate to marker
- [ ] Car can dock at marker (<5cm error)
- [ ] Car avoids obstacles (if LiDAR + ROS2)
- [ ] Multi-car scenario (if applicable)

### Performance Tests
- [ ] Latency: camera → command < 200ms
- [ ] CPU usage: < 80% on Pi
- [ ] Memory usage: < 1GB on Pi
- [ ] Battery life: > 2 hours per charge
- [ ] Network bandwidth: < 5 Mbps

### Stress Tests
- [ ] 30+ minute continuous operation
- [ ] Multiple rapid ID changes
- [ ] Spotty WiFi connection
- [ ] Low light conditions
- [ ] Testbed with clutter

---

## Troubleshooting During Implementation

### Issue: Markers not detected
- [ ] Check marker size is 10cm (or update MARKER_SIZE constant)
- [ ] Verify marker is in frame (not cropped)
- [ ] Check lighting (needs good contrast)
- [ ] Ensure ArUco dict is DICT_4X4_50
- [ ] Print new markers if damaged

### Issue: Distance reading is off
- [ ] Run calibration with known distance (e.g. 50cm)
- [ ] Check camera is not zoomed
- [ ] Verify focal length calculation
- [ ] Use camera calibration matrix instead of dummy

### Issue: App not connecting
- [ ] Check server IP address
- [ ] Verify port 5000 is not blocked
- [ ] Check network security config is applied
- [ ] Rebuild APK if not updated
- [ ] Check firewall on PC/Pi

### Issue: Map not showing
- [ ] Verify `/map_image` endpoint returns PNG
- [ ] Check map resolution (should be 200x200 at 2cm scale = 400x400 pixels)
- [ ] Verify robot is within testbed bounds
- [ ] Check memory for large map generation

### Issue: Jerky steering
- [ ] Reduce acceleration profile
- [ ] Add trajectory smoothing (B-spline)
- [ ] Increase control loop rate
- [ ] Check motor response time
- [ ] Verify weight distribution on car

### Issue: ROS2 node crashes
- [ ] Check for Python syntax errors
- [ ] Verify dependencies installed (numpy, cv2, etc.)
- [ ] Check topic names match publishers/subscribers
- [ ] Monitor system resources (CPU, memory)
- [ ] Enable ROS2 logging: `export ROS_LOG_DIR=/tmp/ros_logs`

---

## Deployment Checklist

### Before Live Testing
- [ ] All documentation reviewed
- [ ] Code reviewed and tested locally
- [ ] APK rebuilt with all changes
- [ ] Network security verified
- [ ] Markers printed and placed
- [ ] Camera calibrated
- [ ] Testbed prepared (cleared of obstacles)

### Safety Checks
- [ ] Emergency stop button ready
- [ ] Motor power supply tested
- [ ] Wheel friction verified (should not slip)
- [ ] Speed limits set (conservative start: 50% power)
- [ ] Obstacle margins set (conservative: 30cm minimum)

### First Run Protocol
1. [ ] Place car at rest in testbed
2. [ ] Place marker in front of car (visible in camera)
3. [ ] Connect mobile app to server
4. [ ] Verify telemetry shows marker detection
5. [ ] Tap "Next" to verify car responds
6. [ ] Manually set ID to marker's ID
7. [ ] Observe guidance commands (should point toward marker)
8. [ ] Increase power gradually (25% → 50% → 75% → 100%)
9. [ ] Record video for later analysis
10. [ ] Stop and check if car drifted

### Post-Run Analysis
- [ ] Review video footage
- [ ] Check dock accuracy
- [ ] Measure path smoothness
- [ ] Note any jerky movements
- [ ] Collect telemetry data
- [ ] Adjust parameters if needed

---

## Documentation Status

| Document | Status | Purpose |
|----------|--------|---------|
| QUICK_REFERENCE.md | ✅ Complete | Start here - commands + features |
| ARCHITECTURE_RECOMMENDATIONS.md | ✅ Complete | Detailed analysis + roadmap |
| INTEGRATION_GUIDE.md | ✅ Complete | How to add map UI to app |
| PROJECT_SUMMARY.md | ✅ Complete | Overview + timeline |
| VISUAL_SUMMARY.md | ✅ Complete | Diagrams + comparisons |
| ROS2_SENSOR_FUSION_TEMPLATE.md | ✅ Complete | ROS2 setup + node code |
| This file | ✅ Complete | Checklist + troubleshooting |

---

## Success Criteria

### Phase 1 (This Week)
- ✅ App successfully connects to server
- ✅ Map displays with robot position
- ✅ Parking spots show on map
- ✅ Camera overlay works
- ✅ Manual marker selection works

### Phase 2 (Next Week)
- ✅ Car navigation is smooth (no jerky turns)
- ✅ Parking maneuver executes correctly
- ✅ Accuracy: < 10cm at dock
- ✅ Multi-marker planning works

### Phase 3-4 (2-3 Weeks)
- ✅ LiDAR obstacle mapping works
- ✅ Odometry fusion improves localization
- ✅ Sensor fusion running on Pi without crashes
- ✅ Accuracy: < 5cm at dock

### Phase 5 (4-6 Weeks)
- ✅ Full ROS2 stack deployed on Pi
- ✅ Multi-goal waypoint planning works
- ✅ Obstacle avoidance 100% reliable
- ✅ System runs autonomously for 30+ minutes

---

## Metrics to Track

```
Weekly Metrics
├─ Features implemented: _ / _
├─ Tests passing: _ / _
├─ Known issues: _ / _
├─ Performance score: _ / 100
└─ Milestones completed: _ / 5

Monthly Goals
├─ Week 1: Map UI + testing
├─ Week 2: Path planning improvements
├─ Week 3: LiDAR integration
├─ Week 4-5: ROS2 sensor fusion
└─ Week 6+: Optimization + scaling

Success Rates
├─ Navigation accuracy: >95%
├─ Parking success: >90%
├─ System uptime: >99%
├─ Sensor reliability: >95%
└─ Overall demo quality: >90%
```

---

## Sign-Off

- [ ] Reviewed all documentation
- [ ] Understood architecture and roadmap
- [ ] Identified priorities for your use case
- [ ] Ready to start Phase 1 testing
- [ ] Have questions? See QUICK_REFERENCE.md

**Next Action:** 
1. Run enhanced server: `python aruco_server_enhanced.py`
2. Connect mobile app
3. Place ArUco markers in camera view
4. Verify map and parking spots display
5. Choose UI integration path (minimal or full)

**Happy coding! 🚗**
