# AutoNexa - Integration Guide for Map + Camera UI

## Quick Start

### 1. Server Setup
Two server options:

**Option A: Original Server (Current)**
```bash
cd C:\aruco_project
python aruco_server.py
```
- Location: `http://192.168.1.X:5000`
- Basic ArUco detection
- Mobile app works but no map visualization

**Option B: Enhanced Server (Recommended for ROS2 prep)**
```bash
cd C:\aruco_project
python aruco_server_enhanced.py
```
- Same features as Option A
- PLUS: `/map_image` endpoint (occupancy grid)
- PLUS: `/robot_pose` endpoint (position on map)
- PLUS: `/parking_spots` endpoint (all detected markers)
- Ready for ROS2 integration

### 2. Mobile App Integration

#### Option 1: Minimal Integration (5 minutes)
Keep existing `main.dart`, but fetch map occasionally:

```dart
// Add to _HomePageState in main.dart:

Uint8List? _mapImage;

void _updateMap() async {
  if (_baseUrl == null) return;
  try {
    final resp = await http.get(Uri.parse('$_baseUrl/map_image'));
    if (resp.statusCode == 200) {
      setState(() => _mapImage = resp.bodyBytes);
    }
  } catch (_) {}
}

// In build(), add a tab or button:
if (_mapImage != null)
  Image.memory(_mapImage!, height: 300)
else
  Text('Map not available')
```

#### Option 2: Full Integration (30 minutes)
Use the new `map_overlay.dart` components:

1. **Copy the new file:**
   ```
   lib/map_overlay.dart  (already created)
   ```

2. **Update pubspec.yaml** (if needed):
   ```yaml
   dependencies:
     flutter:
       sdk: flutter
     http: ^1.1.0
     webview_flutter: ^4.0.7
     google_fonts: ^6.0.0
     cupertino_icons: ^1.0.2
   ```

3. **Update `main.dart`** to use the new view:
   ```dart
   import 'map_overlay.dart';

   // In _HomePageState, replace the entire build() method body with:
   @override
   Widget build(BuildContext context) {
     return Scaffold(
       appBar: AppBar(
         title: Row(children: [
           // ... logo code from before
           Text('AutoNexa'),
         ]),
       ),
       body: SafeArea(
         child: _baseUrl == null
             ? _buildConnectionPanel()  // Show connection UI
             : _buildMainView(),  // Show map + camera view
       ),
     );
   }

   Widget _buildConnectionPanel() {
     return Center(
       child: Column(
         mainAxisAlignment: MainAxisAlignment.center,
         children: [
           Expanded(
             child: Padding(
               padding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
               child: Row(
                 children: [
                   Expanded(
                     child: TextField(
                       controller: _serverController,
                       decoration: InputDecoration(
                         hintText: 'Server IP:port',
                         filled: true,
                         fillColor: Colors.white10,
                         border: OutlineInputBorder(
                           borderRadius: BorderRadius.circular(8),
                           borderSide: BorderSide.none,
                         ),
                       ),
                     ),
                   ),
                   SizedBox(width: 8),
                   ElevatedButton(
                     onPressed: _connect,
                     child: Text('Connect'),
                   ),
                 ],
               ),
             ),
           ),
         ],
       ),
     );
   }

   Widget _buildMainView() {
     return Column(
       children: [
         Expanded(
           child: Stack(
             children: [
               // Map with overlay
               MapWithCameraOverlay(
                 baseUrl: _baseUrl!,
                 selectedId: _state['target_id'],
               ),
               // Camera feed in corner
               Positioned(
                 bottom: 10,
                 left: 10,
                 child: Container(
                   width: 120,
                   height: 90,
                   decoration: BoxDecoration(
                     border: Border.all(color: Colors.white),
                     borderRadius: BorderRadius.circular(4),
                   ),
                   child: ClipRRect(
                     borderRadius: BorderRadius.circular(4),
                     child: _webViewController == null
                         ? Placeholder()
                         : WebViewWidget(controller: _webViewController!),
                   ),
                 ),
               ),
             ],
           ),
         ),
         // Telemetry + controls panel
         _buildControlPanel(),
       ],
     );
   }

   Widget _buildControlPanel() {
     return Padding(
       padding: EdgeInsets.symmetric(horizontal: 12, vertical: 10),
       child: SingleChildScrollView(
         child: Column(
           crossAxisAlignment: CrossAxisAlignment.stretch,
           children: [
             // Telemetry
             Container(
               padding: EdgeInsets.all(8),
               decoration: BoxDecoration(
                 color: Colors.white10,
                 borderRadius: BorderRadius.circular(8),
               ),
               child: Text(
                 'ID=${_state['target_id'] ?? '-'} | '
                 'Dist=${_state['distance_cm']?.toStringAsFixed(1) ?? '-'} cm | '
                 'Bearing=${_state['bearing']?.toStringAsFixed(1) ?? '-'}°',
                 style: TextStyle(fontSize: 12),
               ),
             ),
             SizedBox(height: 8),
             // Controls
             Row(children: [
               Expanded(child: OutlinedButton(
                 onPressed: _prevId,
                 child: Text('◄ Prev'),
               )),
               SizedBox(width: 6),
               Expanded(child: OutlinedButton(
                 onPressed: _nextId,
                 child: Text('Next ►'),
               )),
               SizedBox(width: 6),
               Expanded(child: ElevatedButton(
                 style: ElevatedButton.styleFrom(backgroundColor: Colors.red),
                 onPressed: _quit,
                 child: Text('Quit'),
               )),
             ]),
             // ... rest of existing controls (ID grid, calibration, etc.)
           ],
         ),
       ),
     );
   }
   ```

### 3. Test the Integration

**Start enhanced server:**
```powershell
cd C:\aruco_project
python aruco_server_enhanced.py
```

**Connect mobile app:**
1. Launch app or run `flutter run`
2. Enter server IP: `192.168.X.X:5000`
3. Tap Connect
4. You should now see:
   - Map with robot position (green dot + arrow)
   - Parking spots (blue circles with IDs)
   - Camera feed (small video in corner)
   - Telemetry panel

---

## Data Flow

```
┌──────────────────────┐
│  Raspberry Pi / PC   │
│   (Future: ROS2)     │
└──────────┬───────────┘
           │
    ┌──────┴──────┐
    │              │
┌───▼────┐  ┌─────▼────┐
│ Camera │  │ LiDAR    │
└───┬────┘  └─────┬────┘
    │              │
    └──────┬───────┘
          [aruco_server_enhanced.py]
           │
    ┌──────┴──────────────────────┐
    │                              │
┌───▼──────────┐  ┌──────────┬────▼────┐
│ /video_feed  │  │/map_image│/pose    │
│ (MJPEG)      │  │(PNG)     │(JSON)   │
└──────────────┘  └──────────┴─────────┘
    │
    │              Mobile App (Flutter)
    │              ┌──────────────────────┐
    │              │ MapWithCameraOverlay │
    │              │ - Fetches /map_image │
    │              │ - Fetches /robot_pose│
    │              │ - Draws spots        │
    │              │ - Shows camera feed  │
    │              └──────────────────────┘
```

---

## ROS2 Integration (Future)

When you add ROS2 to the Raspberry Pi:

### 1. Create ROS2 bridge node:
```python
# ros2_bridge_node.py
import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from aruco_msgs.msg import MarkerArray  # if using aruco_ros

class ROS2BridgeNode:
    def subscribe_occupancy_grid(self):
        """Receive /map from Nav2, convert to PNG for HTTP endpoint"""
        pass
    
    def subscribe_robot_pose(self):
        """Receive /robot_pose from localization, publish via HTTP"""
        pass
    
    def subscribe_markers(self):
        """Receive detected markers from camera node"""
        pass
```

### 2. Update enhanced server to fetch from ROS2 instead of local computation:
```python
# In aruco_server_enhanced.py

# Replace local map generation with ROS2 subscription:
def get_occupancy_grid_from_ros2():
    """Fetch pre-computed OccupancyGrid from ROS2 /map topic"""
    # Subscribe to ROS2 topics and cache latest data
    pass

@app.route('/map_image')
def map_image():
    # Use ROS2 data instead of computing locally
    grid = get_occupancy_grid_from_ros2()
    return convert_grid_to_png(grid)
```

### 3. Add sensor fusion:
- Camera: provides ArUco marker detections (fast, 30 Hz)
- LiDAR: provides obstacle map (10 Hz)
- Odometry: provides wheel encoder feedback (10 Hz)
- Fusion: combine in EKF node to estimate robot pose

---

## File Structure After Integration

```
aruco_project/
├── aruco_server.py          (original, basic)
├── aruco_server_enhanced.py (new, map-ready)
├── ARCHITECTURE_RECOMMENDATIONS.md
└── mobile_app/
    ├── lib/
    │   ├── main.dart        (updated with new UI)
    │   ├── map_overlay.dart (new - map components)
    │   └── ...
    ├── android/
    │   └── app/src/main/
    │       └── AndroidManifest.xml (has network config)
    ├── pubspec.yaml
    └── build/app/outputs/flutter-apk/
        └── app-release.apk (46.2 MB)
```

---

## Troubleshooting

### Map not loading
- Verify enhanced server is running: `python aruco_server_enhanced.py`
- Check `/map_image` endpoint: open browser to `http://192.168.X.X:5000/map_image`
- Should download a PNG file

### Camera overlay not showing
- Ensure WebView is properly initialized
- Check `/video_feed` separately: open `http://192.168.X.X:5000/video_feed` in browser

### Parking spots not appearing
- Verify ArUco markers are in camera view
- Check `/parking_spots` endpoint: should return JSON array
- Confirm marker size matches `MARKER_SIZE = 10.0` cm

### Performance issues
- Map updates every 500ms - if too fast, increase interval in `map_overlay.dart`
- Camera overlay: if laggy, reduce WebView frame rate (Android setting)
- Monitor network bandwidth: MJPEG stream can be ~2-5 Mbps

---

## Next Steps

1. **Test with enhanced server:** `python aruco_server_enhanced.py`
2. **Try minimal integration** (Option 1) first to verify endpoints work
3. **Move to full integration** (Option 2) for production UI
4. **Plan ROS2 deployment** on Raspberry Pi 5
5. **Add sensor fusion** (camera + LiDAR + odometry)

See `ARCHITECTURE_RECOMMENDATIONS.md` for detailed roadmap.
