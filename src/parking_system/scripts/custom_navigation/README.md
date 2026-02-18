# Custom Navigation System

A Python-based navigation system with GUI for ROS 2, built from scratch without Nav2.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  GUI (PyQt5)                            │
│  - Map visualization                                    │
│  - Robot pose display                                   │
│  - Path visualization                                   │
│  - Parking spot selection                               │
└─────────────────┬───────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────┐
│              Navigation Controller                       │
│  - Path planning (A*)                                   │
│  - Path replanning on pose updates                      │
└─────────────────┬───────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────┐
│                  ROS Interface                           │
│  - AMCL pose subscriber (/amcl_pose)                    │
│  - Map loader (from map_server topic or file)           │
└──────────────────────────────────────────────────────────┘
```

## Modules

### 1. `map_loader.py`
- Loads PGM map + YAML metadata
- Coordinate transformations: world ↔ pixel
- Occupancy checking

### 2. `path_planner.py`
- A* path planning algorithm
- Grid-based planning on occupancy map
- Obstacle inflation for robot radius

### 3. `parking_spots.py`
- Loads parking spots from YAML
- Spot selection by ID

### 4. `amcl_subscriber.py`
- Subscribes to `/amcl_pose` topic
- Provides current robot pose (x, y, theta)

### 5. `navigation_gui.py`
- PyQt5 GUI application
- Real-time visualization
- Path planning and replanning

## Coordinate Transformations

### World to Pixel:
```python
px = (wy - origin_y) / resolution
py = (wx - origin_x) / resolution
px = height - 1 - px  # Flip Y-axis
```

### Pixel to World:
```python
px_flipped = height - 1 - px
wy = px_flipped * resolution + origin_y
wx = py * resolution + origin_x
```

## Usage

### 1. Start localization (AMCL):
```bash
ros2 launch parking_system amcl_localization.launch.py map_yaml:=/path/to/map.yaml
```

### 2. Run navigation GUI:
```bash
ros2 run parking_system custom_navigation_gui.py \
  --map /path/to/map.yaml \
  --spots /path/to/parking_spots.yaml
```

### 3. In GUI:
- Select parking spot from dropdown
- Path will be automatically planned and updated as robot moves
- Robot pose updates in real-time from AMCL

## Performance Considerations

1. **Path Replanning**: Path is replanned every time AMCL pose updates. For 2m×2m maps, A* typically completes in <50ms.

2. **GUI Updates**: ROS spinning happens in a QTimer (20Hz) to keep GUI responsive.

3. **Occupancy Map**: Pre-computed once at startup for fast path planning.

4. **Optimization**: Consider path smoothing or waypoint reduction if path is too dense.

## Future Enhancements

- Add road mask support (constrain paths to roads)
- Path smoothing (B-spline or Bezier)
- Velocity profile generation
- Obstacle avoidance from LiDAR
- Save/load planned paths

