# Custom Navigation System - Setup Guide

## Overview

A complete Python-based navigation system with GUI, built from scratch without Nav2.

## Installation

### 1. Install Dependencies

```bash
# PyQt5 for GUI
sudo apt-get install python3-pyqt5

# Python packages (if not already installed)
pip3 install numpy pillow opencv-python pyyaml
```

### 2. Build Package

```bash
cd ~/intelligent_parking_ws
colcon build --packages-select parking_system
source install/setup.bash
```

## Usage

### Step 1: Start Localization (AMCL)

```bash
ros2 launch parking_system amcl_localization.launch.py \
  map_yaml:=/home/autonexa/intelligent_parking_ws/maps/emre.yaml
```

### Step 2: Run Navigation GUI

```bash
ros2 run parking_system custom_navigation_gui.py \
  --map /home/autonexa/intelligent_parking_ws/maps/emre.yaml \
  --spots /home/autonexa/intelligent_parking_ws/maps/parking_spots.yaml
```

### Step 3: Use the GUI

1. **Wait for AMCL pose**: The GUI will display "Waiting for AMCL pose..." until it receives the first pose from `/amcl_pose`
2. **Select parking spot**: Choose a spot from the dropdown (e.g., `spot_1`, `spot_2`)
3. **Watch the path**: A green path will appear from the robot to the selected spot
4. **Real-time updates**: As the robot moves (or AMCL updates), the path will automatically replan

## Architecture Details

### Coordinate System

- **World coordinates**: Meters (x, y) in the map frame
- **Pixel coordinates**: Image pixels (row, col) in the PGM image
- **Y-axis flip**: Image Y-axis is inverted compared to world Y-axis

### Path Planning

- **Algorithm**: A* on grid-based occupancy map
- **Resolution**: Uses map resolution (typically 0.02m = 2cm per pixel)
- **Robot radius**: Inflates obstacles by robot radius (default 0.1m)
- **Replanning**: Path is replanned every time AMCL pose updates

### Performance

For a 2m×2m map with 2cm resolution (100×100 pixels):
- **Path planning**: <50ms per replan
- **GUI update rate**: 20Hz (50ms interval)
- **AMCL update rate**: Typically 10-20Hz

## File Structure

```
scripts/custom_navigation/
├── map_loader.py          # Map loading and coordinate transforms
├── path_planner.py        # A* path planning algorithm
├── parking_spots.py       # Parking spot management
├── amcl_subscriber.py     # ROS 2 AMCL pose subscriber
├── navigation_gui.py      # PyQt5 GUI application
└── README.md              # Detailed documentation
```

## Troubleshooting

### "PyQt5 not found"
```bash
sudo apt-get install python3-pyqt5
```

### "No path found"
- Check that start and goal are in free space
- Verify occupancy map is correct (check map image)
- Try increasing robot_radius inflation

### "AMCL pose not updating"
- Verify `/amcl_pose` topic has data: `ros2 topic echo /amcl_pose`
- Check AMCL node is running: `ros2 node list | grep amcl`
- Set initial pose in RViz using "2D Pose Estimate"

### GUI is slow or frozen
- Reduce ROS timer frequency (increase interval in navigation_gui.py)
- Simplify path visualization (reduce path points)
- Check CPU usage

## Advanced Configuration

### Customize Robot Radius

Edit `navigation_gui.py`:
```python
self.planner = SimpleAStarPlanner(
    self.map_loader.get_occupancy_map(),
    self.map_loader.resolution,
    robot_radius=0.15  # Change from 0.1 to 0.15
)
```

### Add Road Constraints

Future enhancement: Load road mask and use it to constrain paths.

### Path Smoothing

Future enhancement: Add B-spline or Bezier curve smoothing to the planned path.

