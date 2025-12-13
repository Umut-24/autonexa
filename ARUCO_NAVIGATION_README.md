# ArUco + LiDAR Navigation Integration

This document explains how to use the integrated ArUco marker detection with LiDAR-based navigation in ROS2.

## Overview

The system combines:
- **ArUco Camera**: Detects markers for precise positioning and goal selection
- **LiDAR + AMCL**: Provides robust localization and obstacle avoidance
- **Sensor Fusion**: Kalman filter combines camera and LiDAR data
- **Navigation**: Nav2 stack plans paths to selected markers

## Architecture

```
Camera → ArUco Detector → Sensor Fusion ← LiDAR (AMCL)
    ↓                    ↓
Marker Selector → ArUco Navigation
    ↓                    ↓
RViz (Visual) ← Navigation Stack
```

## Key Components

### 1. ArUco Detector (`aruco_detector.py`)
- Detects ArUco markers using OpenCV
- Publishes marker poses and debug images
- Calibrates distance measurements

### 2. Sensor Fusion (`sensor_fusion.py`)
- Kalman filter combining ArUco and AMCL poses
- Publishes fused robot pose with covariance
- Handles marker position lookup in map frame

### 3. ArUco Navigation (`aruco_navigation.py`)
- Sets Nav2 goals based on selected marker IDs
- Monitors navigation progress
- Publishes status updates

### 4. Marker Selector (`marker_selector.py`)
- Allows selection of target markers via topics/services
- Manages active marker tracking

## Usage

### Launch the System
```bash
ros2 launch parking_system navigation.launch.py
```

### Select a Target Marker
```bash
# Via topic
ros2 topic pub /select_marker_id std_msgs/Int32 "data: 5"

# Via service
ros2 service call /set_marker_active std_srvs/SetBool "{data: true}"
```

### Monitor Status
```bash
# Navigation status
ros2 topic echo /navigation_status

# ArUco telemetry
ros2 topic echo /aruco_telemetry

# Selector status
ros2 topic echo /selector_status
```

## RViz Visualization

In RViz, you'll see:
- **Green path**: Planned navigation route
- **White text labels**: Movement commands along path
- **ArUco markers**: Detected marker positions
- **Fused pose**: Combined localization estimate
- **Target pose**: Selected marker position

## Marker Configuration

Update marker positions in `sensor_fusion.py` and `aruco_navigation.py`:

```python
self.marker_positions = {
    0: {'x': 1.0, 'y': 0.5, 'yaw': 0.0},    # Bottom-left
    1: {'x': 1.8, 'y': 0.5, 'yaw': 1.57},   # Bottom-right
    2: {'x': 1.8, 'y': 1.8, 'yaw': 3.14},   # Top-right
    3: {'x': 0.5, 'y': 1.8, 'yaw': -1.57},  # Top-left
}
```

## Key Features

### Robust Navigation
- **Vision + LiDAR fusion**: Camera provides precision, LiDAR provides robustness
- **Occlusion handling**: Continues navigation using map knowledge when markers are blocked
- **Real-time updates**: Adapts to changing marker visibility

### Visual Feedback
- **Path visualization**: See planned route and movement commands
- **Marker tracking**: Real-time marker detection display
- **Status monitoring**: Clear feedback on navigation progress

### Easy Control
- **ID-based selection**: Simple marker selection via topics
- **Automatic navigation**: One command starts full navigation sequence
- **Status reporting**: Comprehensive logging and status updates

## Integration with Mobile App

The system publishes to topics that can be consumed by your mobile app:
- `/aruco_telemetry`: Real-time marker detection data
- `/navigation_status`: Navigation progress updates
- `/movement_commands`: Discrete movement instructions

This enables your app to show live camera feed with marker overlays and navigation guidance.</content>
<parameter name="filePath">c:\Users\Anıl\OneDrive\Belgeler\GitHub\autonexa\ARUCO_NAVIGATION_README.md