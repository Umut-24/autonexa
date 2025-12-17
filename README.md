# Intelligent Parking System

An autonomous parking system using ROS2, SLAM Toolbox, and Nav2 for Raspberry Pi 5 with Slamtec C1 LIDAR.

## Hardware Components

- **Raspberry Pi 5** (Ubuntu Desktop 24.04)
- **Slamtec C1 LIDAR**
- **IMX 219 Camera** (for future ArUco marker detection)
- **Ackerman Chassis** (ordered, not needed for test demo)
- **12VDC Motors** (ordered, not needed for test demo)
- **Raspberry Pi Pico WH** (ordered, not needed for test demo)
- **Servo Motor** (ordered, not needed for test demo)

## Test Demo Features

1. **Map Creation**: Create a 2D map of parking environment using slam_toolbox
2. **Localization**: After saving the map, robot localizes itself when placed at random locations and shows pose in RViz
3. **Navigation**: Navigate to selected parking slot (motors not used, robot moved by hand)
4. **Path Monitoring**: System computes path to selected parking slot, continuously checks if robot is on path and provides feedback

## Project Structure

```
intelligent_parking_ws/
├── src/
│   └── parking_system/
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── launch/
│       │   ├── mapping.launch.py      # For creating maps
│       │   ├── localization.launch.py # For localization only
│       │   └── navigation.launch.py   # For navigation and path planning
│       ├── config/
│       │   ├── slam_toolbox_mapping.yaml
│       │   ├── slam_toolbox_localization.yaml
│       │   └── nav2_params.yaml
│       ├── scripts/
│       │   ├── parking_system/
│       │   │   ├── parking_slot_selector.py  # Parking slot selection service
│       │   │   ├── path_monitor.py           # Path following feedback
│       │   │   └── parking_coordinator.py    # Main coordinator
│       │   └── test_parking_slot_selection.py # Test script
│       ├── rviz/
│       │   ├── mapping.rviz
│       │   ├── localization.rviz
│       │   └── navigation.rviz
│       └── urdf/
│           └── robot.urdf
├── maps/  # Created after mapping
│   ├── parking_map.pgm
│   └── parking_map.yaml
└── README.md
```

## Installation

### Prerequisites

1. **ROS2 Humble** (for Ubuntu 24.04, use ROS2 Jazzy if available, or install Humble)
2. Required ROS2 packages:
   ```bash
   sudo apt update
   sudo apt install -y \
     ros-humble-slam-toolbox \
     ros-humble-nav2-bringup \
     ros-humble-nav2-map-server \
     ros-humble-nav2-planner \
     ros-humble-nav2-controller \
     ros-humble-nav2-recoveries \
     ros-humble-nav2-bt-navigator \
     ros-humble-nav2-waypoint-follower \
     ros-humble-nav2-velocity-smoother \
     ros-humble-nav2-lifecycle-manager \
     ros-humble-nav2-amcl \
     ros-humble-dwb-core \
     ros-humble-nav2-navfn-planner \
     ros-humble-nav2-smoother \
     ros-humble-rviz2 \
     ros-humble-robot-state-publisher \
     python3-rosdep
   ```

3. **LIDAR Driver**: Install Slamtec RPLIDAR driver
   ```bash
   # For Slamtec C1, install rplidar_ros package
   cd ~/intelligent_parking_ws/src
   git clone https://github.com/Slamtec/rplidar_ros2.git
   cd rplidar_ros2
   rosdep install --from-paths src --ignore-src -r -y
   ```

### Building the Workspace

```bash
cd ~/intelligent_parking_ws
source /opt/ros/humble/setup.bash  # or your ROS2 distro
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1. Start LIDAR Driver

First, ensure your Slamtec C1 LIDAR is connected and start the driver:

```bash
# Start LIDAR driver (adjust device path if needed)
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=256000 \
  -p frame_id:=laser_link
```

### 2. Create Map (Feature 1)

Launch the mapping mode to create a 2D map:

```bash
cd ~/intelligent_parking_ws
source install/setup.bash
ros2 launch parking_system mapping.launch.py
```

**Instructions:**
- In RViz, use "2D Pose Estimate" tool to set initial pose
- Manually move the robot around the parking environment
- The robot will create a map using slam_toolbox
- To save the map, open a new terminal and run:
  ```bash
  cd ~/intelligent_parking_ws
  source install/setup.bash
  mkdir -p maps
  ros2 run nav2_map_server map_saver_cli -f maps/parking_map
  ```

### 3. Localization (Feature 2)

After saving the map, test localization:

```bash
cd ~/intelligent_parking_ws
source install/setup.bash
ros2 launch parking_system localization.launch.py map_file:=maps/parking_map.yaml
```

**Instructions:**
- Place the robot at a random location in the mapped area
- In RViz, use "2D Pose Estimate" tool to provide initial pose estimate
- The robot should localize itself and show correct pose on the map
- You can verify localization by checking:
  - `/amcl_pose` topic for robot pose
  - `/particlecloud` topic for AMCL particles (shown in RViz)

### 4. Navigation and Path Planning (Features 3 & 4)

Launch navigation mode with path planning and monitoring:

```bash
cd ~/intelligent_parking_ws
source install/setup.bash
ros2 launch parking_system navigation.launch.py map_file:=maps/parking_map.yaml
```

#### Recommended (Safest) Navigation Bringup
If you want the simplest, least error-prone Nav2 setup (no ArUco / custom nodes), use:

```bash
cd ~/intelligent_parking_ws
source install/setup.bash
ros2 launch parking_system nav2_only.launch.py map_file:=/full/path/to/parking_map.yaml
```

**Instructions:**
1. **Set Initial Pose**: In RViz, use "2D Pose Estimate" to set robot's current pose
2. **Select Parking Slot**: 
   - Use RViz "2D Goal Pose" tool to click on a parking slot, OR
   - Use service call:
     ```bash
     ros2 service call /select_parking_slot std_srvs/srv/SetBool "{data: true}"  # Selects slot_1
     ```
3. **Path Planning**: The system will compute a path to the selected parking slot
4. **Path Monitoring**: 
   - The path will be displayed in RViz (green line)
   - Check feedback on topics:
     - `/path_feedback` - Text feedback about path following
     - `/path_distance_error` - Distance error from path
     - `/path_angular_error` - Angular error from path
5. **Manual Movement**: Move the robot by hand following the path
   - The system continuously checks if robot is on the planned path
   - Feedback is published to `/path_feedback` topic

### Viewing Feedback

Monitor path following feedback in real-time:

```bash
# Terminal 1: View path feedback messages
ros2 topic echo /path_feedback

# Terminal 2: View distance error
ros2 topic echo /path_distance_error

# Terminal 3: View angular error
ros2 topic echo /path_angular_error

# Terminal 4: View parking status
ros2 topic echo /parking_status
```

## Configuration

### Parking Slots

Edit parking slot positions in `src/parking_system/scripts/parking_system/parking_slot_selector.py`:

```python
self.parking_slots = {
    'slot_1': {'x': 1.0, 'y': 1.0, 'yaw': 0.0},
    'slot_2': {'x': 1.5, 'y': 1.0, 'yaw': 0.0},
    # Add more slots...
}
```

### SLAM Parameters

Adjust SLAM parameters in `config/slam_toolbox_mapping.yaml` for better mapping performance.

### Nav2 Parameters

Modify navigation parameters in `config/nav2_params.yaml` to tune path planning, obstacle avoidance, etc.

## Future Enhancements

- **ArUco Marker Detection**: Integrate camera and ArUco markers to identify and navigate to parking spots
- **Motor Control**: Integrate motor control when hardware arrives
- **Mobile App**: Develop mobile application for remote control and monitoring
- **Multi-Vehicle Support**: Add communication protocols for multiple vehicles

## Troubleshooting

### LIDAR Not Detected
- Check USB connection: `ls -l /dev/ttyUSB*`
- Check permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches LIDAR specifications

### Localization Issues
- Ensure initial pose estimate is accurate
- Check that map frame and robot frames are correctly configured
- Verify `/scan` topic is publishing: `ros2 topic echo /scan`

### Path Planning Fails
- Check that robot is localized: `ros2 topic echo /amcl_pose`
- Verify map is loaded: `ros2 topic echo /map`
- Ensure goal is within map bounds

### TF Transform Issues
- Check TF tree: `ros2 run tf2_tools view_frames`
- Verify all frames are publishing: `ros2 topic echo /tf`

## License

MIT License

## Authors

Final Year Project Team

