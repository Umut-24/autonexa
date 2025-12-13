# Advanced ArUco + LiDAR Navigation System

This document describes the redesigned ROS2 navigation system that integrates ArUco marker detection with LiDAR localization for robust autonomous navigation, even when markers are not currently visible.

## 🎯 Key Features

- **Persistent Marker Map**: Stores and updates known marker positions across sessions
- **Intermittent Detection Handling**: Continues navigation using map knowledge during occlusions
- **Advanced Sensor Fusion**: Uses robot_localization EKF for proper multi-sensor fusion
- **Real-time Visualization**: Shows all known markers, current detections, and fused pose
- **Navigation to Unseen Markers**: Can navigate to markers not currently in camera view

## 🏗️ System Architecture

```
Camera → ArUco Detector → Marker Map Manager ← Persistent Storage
    ↓              ↙          ↓
TF Broadcasting → Enhanced Sensor Fusion → Robot Localization EKF
                   ↓              ↓
            ArUco Navigation ← Marker Selector
                   ↓
               Nav2 Stack
                   ↓
                RViz (Visualization)
```

## 📋 Core Components

### 1. Marker Map Manager (`marker_map_manager.py`)
**Purpose**: Persistent storage and management of ArUco marker positions
- **Loads/Saves**: Marker map to/from YAML file
- **Updates**: Real-time position updates from detections
- **Publishes**: Complete marker map as PoseArray
- **Fusion**: Weighted averaging for position updates

### 2. Enhanced ArUco Detector (`aruco_detector.py`)
**Purpose**: Detect markers and provide TF transforms
- **Detection**: OpenCV ArUco detection with calibration
- **TF Broadcasting**: Publishes marker transforms for coordinate consistency
- **Map Updates**: Sends pose updates to marker map manager
- **Visualization**: Publishes debug images and telemetry

### 3. Enhanced Sensor Fusion (`enhanced_sensor_fusion.py`)
**Purpose**: Bridge between ArUco and robot_localization EKF
- **Marker Tracking**: Maintains tracks for intermittent detections
- **EKF Integration**: Publishes ArUco poses with proper covariances
- **Track Management**: Cleans up stale marker tracks

### 4. ArUco Navigation (`aruco_navigation.py`)
**Purpose**: Navigate to selected markers using known positions
- **Map Integration**: Uses marker map for goal setting
- **Nav2 Interface**: Sends goals to navigation stack
- **Status Monitoring**: Tracks navigation progress

## 🚀 Usage

### Launch the Complete System
```bash
ros2 launch parking_system navigation.launch.py
```

### Load Existing Marker Map
```bash
ros2 service call /load_marker_map std_srvs/srv/Trigger
```

### Select Navigation Target
```bash
# Navigate to marker ID 5 (even if not currently visible)
ros2 topic pub /select_marker_id std_msgs/Int32 "data: 5"
```

### Save Updated Marker Map
```bash
ros2 service call /save_marker_map std_srvs/srv/Trigger
```

### Monitor System Status
```bash
# Navigation status
ros2 topic echo /navigation_status

# Marker map status
ros2 topic echo /marker_map_status

# ArUco telemetry
ros2 topic echo /aruco_telemetry
```

## 📊 RViz Visualization

The system provides comprehensive visualization:

- **Green Path**: Planned navigation trajectory with movement commands
- **Marker Map**: All known marker positions (persistent)
- **Current Detections**: Real-time ArUco marker poses
- **Fused Pose**: Combined localization estimate
- **TF Tree**: Coordinate frame relationships

## 🔧 Configuration

### Marker Map File
The system uses `marker_map.yaml` for persistent storage:
```yaml
0:
  position: {x: 1.0, y: 0.5, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  last_update: 1640995200.0
  confidence: 0.8
```

### Robot Localization EKF
Configured in `ekf.yaml` with:
- **odom0**: Wheel odometry input
- **pose0**: ArUco marker poses
- **pose1**: AMCL LiDAR localization
- **Process noise**: Tuned for intermittent measurements

## 🎯 Navigation Workflow

1. **Map Loading**: System loads known marker positions
2. **Detection**: Camera detects visible markers, updates positions
3. **Fusion**: EKF combines ArUco, LiDAR, and odometry
4. **Selection**: User selects target marker ID
5. **Planning**: Nav2 plans path to known marker position
6. **Execution**: Robot follows path with movement commands
7. **Recovery**: If marker becomes occluded, continues using map knowledge

## 🔄 Handling Intermittent Detections

The system robustly handles marker visibility:

- **Track Maintenance**: Keeps marker tracks during occlusions
- **Velocity Prediction**: Estimates marker movement during gaps
- **Confidence Weighting**: Uses confidence scores for position updates
- **Timeout Management**: Removes stale tracks automatically

## 📈 Benefits Over Previous System

- **Persistent Knowledge**: Remembers marker positions across sessions
- **Robust Navigation**: Works even with partial marker visibility
- **Better Fusion**: Proper EKF integration vs custom Kalman filter
- **Coordinate Consistency**: TF-based transform management
- **Scalability**: Handles dynamic marker maps and additions

## 🛠️ Integration with Mobile App

The system provides topics for mobile app integration:

- **`/marker_map`**: Complete marker map for visualization
- **`/select_marker_id`**: Receive navigation commands
- **`/navigation_status`**: Send status updates
- **`/aruco_telemetry`**: Live detection data

## 🔧 Tuning and Calibration

### Marker Detection
- Adjust `marker_size` parameter for accurate pose estimation
- Tune camera calibration for better accuracy
- Set appropriate detection timeouts

### Sensor Fusion
- Adjust covariances in `ekf.yaml` based on sensor characteristics
- Tune process noise for expected motion dynamics
- Configure sensor timeouts for intermittent detections

### Navigation
- Set appropriate goal tolerances in Nav2 config
- Adjust planner parameters for path quality
- Configure recovery behaviors for occlusions

This redesigned system provides enterprise-grade ArUco navigation with robust handling of real-world challenges like intermittent detections and partial marker visibility.</content>
<parameter name="filePath">c:\Users\Anıl\OneDrive\Belgeler\GitHub\autonexa\ADVANCED_ARUCO_NAVIGATION_README.md