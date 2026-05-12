# AutoNexa Mobile Controller

> Flutter mobile app for the AutoNexa Intelligent Parking and Vehicle Recall System.
> Control your autonomous parking robot via Wi-Fi through the RPi5 ROS2 bridge.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Getting Started](#getting-started)
4. [App Structure](#app-structure)
5. [Screens & Features](#screens--features)
   - [Home Dashboard](#1-home-dashboard)
   - [Control (Joystick)](#2-control-joystick)
   - [Map (LIDAR View)](#3-map-lidar-view)
   - [Parking & Missions](#4-parking--missions)
   - [Camera](#5-camera)
   - [Diagnostics](#6-diagnostics)
   - [Settings](#7-settings)
6. [Emergency Stop (E-STOP)](#emergency-stop-e-stop)
7. [Themes](#themes)
8. [Communication Protocol](#communication-protocol)
9. [Control Chain](#control-chain)
10. [Project Files](#project-files)
11. [Building the APK](#building-the-apk)
12. [Troubleshooting](#troubleshooting)

---

## Overview

AutoNexa is a small-scale autonomous vehicle project designed for an **Intelligent Parking and Vehicle Recall System**. The vehicle operates within a 2m x 2m testbed, detects parking spots using ArUco markers and computer vision, parks itself autonomously, and can be summoned back to a user-defined position -- all controlled from this mobile app.

**Key capabilities of the app:**

- **Manual driving** via virtual joystick (portrait + fullscreen landscape)
- **Autonomous navigation** -- send Nav2 goals by tapping the LIDAR map or entering coordinates
- **Autonomous parking** -- detect ArUco markers and navigate to parking spots
- **Vehicle summoning** -- recall the vehicle to a saved position with one tap
- **Mission planning** -- create multi-waypoint navigation sequences, save/load them
- **Live monitoring** -- real-time telemetry, battery level, obstacle proximity alerts
- **SLAM map visualization** -- pan/zoom LIDAR map with robot pose, path trail, scan points
- **Live camera feed** -- MJPEG stream from the RPi5 with ArUco marker overlay info
- **Emergency Stop** -- global floating button that latches the hardware E-STOP on the Pico
- **Multi-theme UI** -- Dark, AMOLED Black, and Light themes with one-tap cycling
- **Event logging** -- categorized logs with filtering, export to file
- **Persistent settings** -- server history, speed limits, summon point, missions saved locally

---

## System Architecture

```
+-------------------+         Wi-Fi (HTTP)         +---------------------+
|                   |  ---------------------------> |                     |
|   AutoNexa App    |  POST /api/control  (20 Hz)  |  Raspberry Pi 5     |
|   (Flutter)       |  GET  /api/status   (2 Hz)   |  (ROS2 Jazzy)       |
|                   |  GET  /api/telemetry (5 Hz)  |                     |
|                   |  POST /api/estop             |  Flask Bridge        |
|                   |  POST /api/nav_goal          |  (ros2_mobile_       |
|                   |  GET  /api/map               |   bridge.py)         |
|                   |  GET  /api/scan              |                     |
|                   |  GET  /video_feed            |  Nav2 + SLAM         |
|                   |  <--------------------------- |  Toolbox             |
+-------------------+        JSON responses        +---------+-----------+
                                                             |
                                                      ROS2 Topics
                                                             |
                                                   +---------v-----------+
                                                   |  Raspberry Pi       |
                                                   |  Pico               |
                                                   |  (micro-ROS)        |
                                                   |                     |
                                                   |  Motor Driver       |
                                                   |  (I2C -> PCA9685)   |
                                                   +---------------------+
```

**Full control chain:**

```
App Joystick -> HTTP POST /api/control -> Flask Bridge -> /cmd_vel topic
  -> velocity_smoother -> collision_monitor -> cmd_vel_to_pico_bridge
  -> /pico/control_cmd -> micro-ROS -> Pico -> I2C PCA9685 -> Servo + DC Motor
```

---

## Getting Started

### Prerequisites

- Android phone (Android 5.0+)
- Raspberry Pi 5 running ROS2 Jazzy with the Flask bridge (`ros2_mobile_bridge.py`)
- Both devices on the same Wi-Fi network
- The RPi5 Flask bridge running on port 5000 (default)

### Install the App

1. Transfer the APK to your phone:
   ```
   aruco_project/mobile_app/build/app/outputs/flutter-apk/app-release.apk
   ```
2. Enable "Install from Unknown Sources" on your phone
3. Install the APK

### Connect to the Robot

1. Open the app
2. Go to **Settings** (tap "More" in the bottom bar, then Settings)
3. Enter the RPi5's IP address and port (e.g., `192.168.1.5:5000`)
4. Tap **Connect**
5. The connection indicator in the top-right turns green when connected

---

## App Structure

The app uses 4 primary tabs + a "More" bottom sheet for additional pages:

```
Bottom Navigation:
  [Home]  [Control]  [Map]  [Parking]  [More ...]
                                          |
                                          +-- Camera
                                          +-- Diagnostics
                                          +-- Settings
```

A floating **E-STOP** button appears on all primary tabs when connected.

### State Management

The app uses **Provider** for reactive state management:

| Provider | Purpose |
|----------|---------|
| `ConnectionService` | HTTP connection, polling, joystick control, telemetry |
| `ThemeProvider` | Dark/Light/AMOLED theme switching with persistence |
| `EventLogger` | In-memory ring buffer (500 entries) with categorized logging |
| `PreferencesService` | SharedPreferences wrapper for all persistent settings |
| `AppState` | Top-level container providing cross-service access |

---

## Screens & Features

### 1. Home Dashboard

**Purpose:** At-a-glance overview of the entire system.

**What you see:**

| Section | Details |
|---------|---------|
| **Header bar** | AutoNexa logo, battery indicator, theme toggle button, latency badge, connection dot |
| **Connection card** | Connected/Not Connected status, server URL, uptime timer |
| **Obstacle alert** | Orange "OBSTACLE NEARBY" or red "COLLISION WARNING" banner when LiDAR detects objects < 15cm / < 5cm |
| **Battery warning** | Yellow banner when battery drops below 15% |
| **Robot Status** | Position (X, Y), Velocity (m/s), Heading (degrees) |
| **Motor Telemetry** | Left wheel velocity, Right wheel velocity, Steering angle |
| **Detected Markers** | Horizontal scrollable list of ArUco markers with live/stale/lost badges |
| **Quick Actions** | 6 action buttons in a 3x2 grid |
| **System Info** | Pose source, scan point count, map dimensions, latency, command count |

**Quick Actions explained:**

| Button | What it does |
|--------|-------------|
| **E-STOP** | Engages or releases the emergency stop (toggles) |
| **Go Home** | Sends navigation goal to (0, 0, 0) -- the map origin |
| **Summon** | If no summon point is saved: prompts to save the current position. If already saved: navigates to the saved position |
| **Nav Goal** | Opens a dialog to enter X, Y, Yaw coordinates for Nav2 |
| **Auto Park** | Automatically navigates toward the nearest detected ArUco marker |
| **Full Stop** | Immediately zeroes the joystick output (stops the robot without E-STOP) |

**Theme toggle:** Tap the moon/sun icon in the header to cycle through Dark -> AMOLED -> Light themes.

---

### 2. Control (Joystick)

**Purpose:** Manual driving of the robot.

**Portrait mode:**
- Virtual joystick in the center (drag to steer & throttle)
- Connection badge (LINKED / NO LINK)
- Speed limiter slider (10% to 100%)
- Real-time telemetry chips: Left wheel, Right wheel, Linear velocity (Vx), Angular velocity (Wz)
- Odometry position readout: X, Y, Yaw
- **EMERGENCY STOP** button (full-width, red)
- **Send Nav2 Goal** button (when connected)
- **Fullscreen** button to enter landscape mode

**Fullscreen landscape mode:**
- Joystick fills 75% of screen height
- Left HUD: Steer, Throttle, L Vel, R Vel, Vx values
- Right HUD: Speed control (+/- buttons), compact E-STOP
- Connection badge (top-left), Exit button (top-right)
- Immersive mode hides system bars

**How the joystick works:**
- **X-axis (horizontal):** Steering. Left = turn left, Right = turn right
- **Y-axis (vertical):** Throttle. Up = forward, Down = reverse
- Values are clamped to -1.0 to +1.0 and multiplied by the speed limit
- Commands are sent at **20 Hz** (every 50ms) via HTTP POST to `/api/control`
- Haptic feedback on each joystick movement

---

### 3. Map (LIDAR View)

**Purpose:** Visualize the SLAM map, see where the robot is, and set navigation goals by tapping.

**Features:**
- **SLAM map image** fetched from `/api/map` as PNG (refreshed every 2 seconds)
- **Robot pose overlay** -- colored circle + heading arrow drawn on the map
- **Scan points** -- green dots showing live LiDAR scan data
- **Path trail** -- accent-colored line showing the robot's historical trajectory (up to 500 points)
- **Pinch to zoom** -- InteractiveViewer supports 0.5x to 10x zoom
- **Tap to navigate** -- tap any point on the map to open a Nav Goal dialog pre-filled with the tapped coordinates (automatically converted from pixel to map frame using resolution and origin)

**Toggle buttons (top-right):**
- **Scan** -- show/hide LiDAR scan points
- **Trail** -- show/hide path trail
- **Markers** -- show/hide ArUco marker positions

**Info bar (bottom):** X, Y, Yaw, Scan count, Pose source

**Clear trail:** Button in bottom-right to reset the path trail history.

**Coordinate conversion:**
```
mapX = originX + pixelX * resolution
mapY = originY + (mapHeight - pixelY) * resolution
```

---

### 4. Parking & Missions

**Purpose:** Manage detected parking spots and create multi-waypoint navigation missions.

This screen has **two sub-tabs**: Parking Spots and Missions.

#### Parking Spots Tab

Lists all ArUco markers currently detected by the camera, sorted by ID.

Each marker card shows:
- **ID badge** -- large marker ID number
- **Status** -- color-coded badge:
  - **Live** (green) -- marker seen within the last 2 seconds
  - **Stale** (yellow) -- marker last seen 2-10 seconds ago
  - **Lost** (gray) -- marker not seen for >10 seconds
- **Distance** -- how far the marker is from the robot (meters)
- **Bearing** -- angle to the marker (degrees)
- **Navigate button** -- opens Nav Goal dialog to drive to this parking spot

**When no markers are detected:** Shows an empty state message suggesting to ensure camera line-of-sight to ArUco markers.

#### Missions Tab

Create sequences of waypoints and execute them automatically.

**Creating a mission:**
1. Tap **Add** to enter X, Y, Yaw coordinates and an optional label
2. Repeat to add more waypoints
3. **Drag to reorder** waypoints using the handle on the left
4. **Delete** individual waypoints with the X button

**Executing a mission:**
1. Tap **Execute** -- the robot navigates to each waypoint sequentially
2. A progress bar shows the current waypoint index
3. Tap **Stop** to abort the mission at any time

**Saving & Loading:**
- Tap the **Save** icon, enter a mission name, and it is saved to device storage (SharedPreferences)
- Tap the **Load** icon to select from saved missions
- Delete saved missions from the load dialog

---

### 5. Camera

**Access:** More -> Camera

**Features:**
- **Live MJPEG stream** from the RPi5 camera via `/video_feed`
- Uses Flutter's native `Image.network` for the stream (no WebView dependency)
- **Detected markers section** -- horizontal scrollable list of ArUco markers with status
- **Telemetry bar** -- shows nearest marker ID, distance, and bearing

**Use case:** See what the camera sees in real-time. Verify ArUco marker detection is working. Monitor the robot's field of view during autonomous parking.

---

### 6. Diagnostics

**Access:** More -> Diagnostics

**Features:**
- **Network stats card** -- latency (with color coding: green <50ms, yellow >=50ms), connection status, total commands sent, uptime
- **Filter chips** -- filter log entries by category: All, Connection, Control, Nav
- **Event log** -- scrollable list of all events, newest first:
  - Each entry shows: colored dot (info/success/warning/error), timestamp (HH:MM:SS), message
- **Clear log** -- trash icon in the app bar
- **Export log** -- download icon saves the log as a `.txt` file to the app's documents directory

**Log categories:**

| Category | Events logged |
|----------|--------------|
| Connection | Connect, disconnect, connection lost, restored |
| Control | E-STOP engaged/released, joystick errors |
| Navigation | Nav goals sent/failed, missions started/stopped/completed |
| System | General system events |

---

### 7. Settings

**Access:** More -> Settings

**Sections:**

#### Server Connection
- Text field to enter the RPi5 IP:port (e.g., `192.168.1.5:5000`)
- **Server history dropdown** -- expand to see and select from up to 10 previously used servers
- **Connect / Disconnect** buttons
- Connection status indicator when connected

#### Appearance
- **Theme toggle** -- switch between Dark and Light modes
- **AMOLED black mode** -- toggle for true black background (only visible in dark mode). Uses `#000000` background instead of `#08080F`

#### Communication
- Shows current protocol info: HTTP (Flask bridge)
- Control rate: 20 Hz (50ms)
- Telemetry rate: 5 Hz (200ms)
- Status rate: 2 Hz (500ms)
- **Upgrade tip** -- suggests switching to WebSocket via Foxglove Bridge for lower latency

#### Control
- **Default speed limit slider** -- 10% to 100%, persisted across app restarts
- **Auto-reconnect toggle** -- automatically reconnect when connection is lost
- **Haptic feedback toggle** -- enable/disable vibration on joystick interaction

#### Summon Point
- Shows the saved summon coordinates (X, Y, Yaw) if set
- Delete button to clear the summon point
- To set a new summon point: tap "Summon" on the Home tab

#### Display
- **Map refresh rate** -- choose how often the LIDAR map image is fetched: 1s, 2s, or 5s

#### About
- App version, logo, and feature summary

---

## Emergency Stop (E-STOP)

The E-STOP system is the most critical safety feature.

### How it works

1. **Floating red button** appears on all 4 primary tabs when connected
   - Pulses with a glow animation to draw attention
   - Shows "STOP" normally, turns orange with "GO" when engaged

2. **When you press E-STOP:**
   - App immediately sends `POST /api/estop` to the Flask bridge
   - Bridge calls the `/pico/estop` ROS2 service
   - **Pico hardware latches** -- all motors stop immediately
   - Joystick output is zeroed and locked
   - Button turns orange ("GO") indicating E-STOP is engaged

3. **When you press to release (GO):**
   - App sends `POST /api/estop_clear` to the Flask bridge
   - Bridge calls the clear service to unlatch the Pico
   - Joystick control is re-enabled
   - Button returns to red ("STOP")

### E-STOP is also available:
- As a quick action button on the Home tab
- As a full-width button on the Control tab
- As a compact button in fullscreen driving mode

---

## Themes

The app supports **3 visual themes** that can be changed at any time:

| Theme | Background | Surface | Description |
|-------|-----------|---------|-------------|
| **Dark** | `#08080F` | `#101018` | Default. Dark blue-gray. Easy on the eyes |
| **AMOLED** | `#000000` | `#050505` | True black. Saves battery on OLED screens |
| **Light** | `#F0F0F5` | `#FFFFFF` | Light mode for outdoor visibility |

**How to change:**
- **Quick:** Tap the theme icon in the Home tab header to cycle through all 3
- **Precise:** Go to Settings -> Appearance -> use the Dark/Light toggle and AMOLED switch

All themes use the **AutoNexa brand color** (`#E94560` red) as the primary accent, with consistent green for success, red for danger, yellow for warnings, and cyan for info across all themes.

Theme preference is **persisted** -- the app remembers your choice.

---

## Communication Protocol

The app communicates with the RPi5 over **HTTP** via the Flask bridge (`ros2_mobile_bridge.py`).

### API Endpoints

| Endpoint | Method | Rate | Purpose |
|----------|--------|------|---------|
| `/api/control` | POST | 20 Hz | Send joystick X, Y, E-STOP flag, speed limit |
| `/api/status` | GET | 2 Hz | Robot pose, detected markers, scan info, map info |
| `/api/telemetry` | GET | 5 Hz | Wheel velocities, steering, odometry, battery, obstacle distance |
| `/api/map` | GET | 0.5 Hz | SLAM map as PNG image |
| `/api/scan` | GET | 1 Hz (on-demand) | LiDAR scan points as JSON array |
| `/api/estop` | POST | On press | Engage emergency stop |
| `/api/estop_clear` | POST | On press | Release emergency stop |
| `/api/nav_goal` | POST | On request | Send Nav2 goal (x, y, yaw) |
| `/api/cancel_nav` | POST | On request | Cancel current Nav2 goal |
| `/video_feed` | GET | Stream | MJPEG camera stream |

### Control packet format
```json
{
  "x": 0.45,
  "y": 0.80,
  "e": 0,
  "speed_limit": 0.50
}
```
- `x`: steering (-1.0 to 1.0)
- `y`: throttle (-1.0 to 1.0)
- `e`: e-stop flag (0 or 1)
- `speed_limit`: speed multiplier (0.1 to 1.0)

### Network monitoring
- Latency is measured every 2 seconds via health check
- Connection is marked as **lost** after 3 consecutive health check failures
- Latency badge shows in the header (green < 50ms, yellow >= 50ms)

---

## Control Chain

The complete data flow from your finger to the motor:

```
 1. Your finger drags the joystick
 2. JoystickWidget calls onMove(x, y) at touch event rate
 3. ConnectionService.updateJoystick(x, y) stores the values
 4. Timer fires every 50ms (20 Hz):
      -> HTTP POST /api/control { x, y, e, speed_limit }
 5. Flask bridge (ros2_mobile_bridge.py) receives the POST
 6. Bridge publishes geometry_msgs/Twist to /cmd_vel
      -> linear.x = y * speed_limit * max_speed
      -> angular.z = x * speed_limit * max_angular
 7. velocity_smoother smooths the commands
 8. collision_monitor checks for obstacles
 9. cmd_vel_to_pico_bridge converts to Ackermann
10. Published to /pico/control_cmd
11. Pico receives via micro-ROS subscriber
12. PCA9685 I2C driver sets servo angle + motor PWM
13. Vehicle moves!
```

---

## Project Files

```
lib/
|-- main.dart                          # App entry point, Provider setup, navigation shell
|-- models/
|   |-- mission.dart                   # Waypoint & Mission data classes (JSON serializable)
|   |-- robot_state.dart               # RobotPose, MarkerInfo, MapInfo, ScanInfo, RobotStatus
|   +-- telemetry.dart                 # PicoTelemetry (wheels, odom, battery, obstacles)
|-- services/
|   |-- connection_service.dart        # HTTP connection, polling, control, E-STOP, nav goals
|   |-- event_logger.dart              # Ring buffer logger with levels & categories
|   +-- preferences_service.dart       # SharedPreferences wrapper for all settings
|-- state/
|   +-- app_state.dart                 # Top-level ChangeNotifier combining all services
|-- tabs/
|   |-- home_tab.dart                  # Dashboard with status, telemetry, quick actions
|   |-- control_tab.dart               # Joystick driving (portrait + fullscreen landscape)
|   |-- map_tab.dart                   # LIDAR map with overlays and tap-to-navigate
|   |-- parking_tab.dart               # Parking spots + mission planner
|   |-- camera_tab.dart                # Live MJPEG camera feed
|   |-- diagnostics_tab.dart           # Event log viewer + network stats
|   +-- settings_tab.dart              # Server connection, themes, preferences
|-- theme/
|   |-- app_colors.dart                # Color system for all 3 themes
|   |-- app_theme.dart                 # ThemeData builder
|   +-- theme_provider.dart            # Theme mode/variant management with persistence
+-- widgets/
    |-- autonexa_logo.dart             # Custom-painted brand logo (red/black crescents + network)
    |-- battery_indicator.dart         # Battery icon + percentage with adaptive coloring
    |-- connection_indicator.dart      # Animated status dot (green/yellow/red/gray)
    |-- estop_fab.dart                 # Floating E-STOP button with pulse animation
    |-- glass_card.dart                # Glassmorphic card with translucent surface
    |-- joystick_widget.dart           # Virtual joystick with circular base and drag knob
    |-- marker_chip.dart               # Compact ArUco marker info chip
    |-- nav_goal_dialog.dart           # Dialog for entering Nav2 coordinates
    |-- obstacle_alert.dart            # Proximity warning banner (warning/critical)
    +-- stat_tile.dart                 # Compact metric display tile
```

### Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `http` | ^1.1.0 | HTTP client for Flask bridge communication |
| `provider` | ^6.1.0 | Reactive state management |
| `shared_preferences` | ^2.2.0 | Persistent local storage |
| `path_provider` | ^2.1.0 | File system paths for log export |
| `google_fonts` | ^6.0.0 | Inter font family |
| `cupertino_icons` | ^1.0.2 | iOS-style icons |

---

## Building the APK

### Prerequisites
- Flutter SDK installed
- Android SDK installed
- Java 17+ (Java 21 recommended)

### Build command

```bash
cd aruco_project/mobile_app

# If you have a non-ASCII Windows username (e.g., with characters like i, o, u),
# set this environment variable first:
export JAVA_TOOL_OPTIONS="-Djdk.net.unixdomain.tmpdir=C:/gradle_tmp"
mkdir -p C:/gradle_tmp

# Build the release APK
flutter build apk --release
```

The APK will be at:
```
build/app/outputs/flutter-apk/app-release.apk
```

### Common build issue: Gradle loopback failure

If you see `Could not receive a message from the daemon`, this is caused by Java 21's Unix domain sockets requiring a short ASCII temp path. Windows usernames with non-ASCII characters cause the socket path to exceed limits.

**Fix:** Set the `JAVA_TOOL_OPTIONS` environment variable as shown above. To make it permanent, add it to your Windows System Environment Variables:
```
Variable: JAVA_TOOL_OPTIONS
Value: -Djdk.net.unixdomain.tmpdir=C:/gradle_tmp
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **Can't connect to robot** | Verify both devices are on the same Wi-Fi. Check the IP address. Ensure the Flask bridge is running on the RPi5 (`python3 ros2_mobile_bridge.py`) |
| **Connection keeps dropping** | Check Wi-Fi signal strength. The health check will show "Connection lost" after 3 failures. Enable auto-reconnect in Settings |
| **Joystick doesn't move the robot** | Check the Control tab shows "LINKED". Verify E-STOP is not engaged (button should be red, not orange). Check speed limit is above 10% |
| **No markers detected** | Ensure ArUco markers are in the camera's field of view. Check lighting conditions. Go to Camera tab to verify the video feed is working |
| **Map shows nothing** | SLAM Toolbox needs to be running on the RPi5. The robot needs to move a bit for the map to generate. Check that LiDAR is producing scan data |
| **E-STOP won't release** | Tap the orange "GO" button. If it doesn't work, check if the Flask bridge has the `/api/estop_clear` endpoint. The Pico hardware latch requires the clear command |
| **App crashes on start** | Clear app data and restart. The SharedPreferences may have corrupted data |
| **Camera feed blank** | The MJPEG stream requires the camera node to be running on the RPi5. Check `/video_feed` endpoint in a browser first |
| **High latency (>100ms)** | Consider switching to WebSocket via Foxglove Bridge (see Settings -> Communication -> Upgrade tip) |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 2.1.0 | 2026-03-16 | Multi-theme support (Dark/AMOLED/Light), custom-painted logo, battery monitoring, obstacle proximity alerts, path trail on map, vehicle summon feature, haptic feedback toggle |
| 2.0.0 | 2026-03-15 | Complete redesign: Provider architecture, glassmorphic UI, ConnectionService replacing fragmented polling, E-STOP hardware unlatch fix, parking spot management, mission planner, native map rendering, fullscreen joystick mode |
| 1.0.0 | 2026-03-11 | Initial version with basic joystick control, WebView-based map, UDP communication |

---

*Built with Flutter -- Controlled by ROS2 Jazzy -- Powered by Raspberry Pi 5 + Pico*
