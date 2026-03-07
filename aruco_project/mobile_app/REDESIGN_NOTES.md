# AutoNexa Mobile App - UI Redesign v2

## Overview

The mobile app has been completely redesigned with a **tabbed interface** similar to Instagram (Reels vs Main Feed), providing better separation of concerns and a more integrated, professional experience.

## New Tabbed Architecture

### Tab 1: **CAMERA** 🎥
**Purpose:** Live camera feed with real-time telemetry

**Contents:**
- Full-screen video feed from server (WebView)
- Live telemetry display (ID, Distance, Bearing)
- Quick navigation controls (Prev/Next ID)
- All-IDs mode toggle button
- Real-time connection status in AppBar

**Why separate?**
- Users need focused, distraction-free monitoring
- Full-screen view maximizes camera visibility
- Quick controls for rapid ID changes

---

### Tab 2: **DASHBOARD** 📊
**Purpose:** Comprehensive system control and monitoring

**Contents:**
- **Connection Status Card**: Shows server URL and connection state
- **ID Selection Card**: 4×4 grid of all 16 detection IDs with visual feedback
  - Green = currently selected
  - Orange = tracked (in all-IDs mode)
  - Grey = untracked
- **Pre-select ID Button**: Choose ID before connecting
- **Telemetry Card**: Displays current metrics in organized rows
  - Target ID
  - Distance (cm)
  - Bearing (degrees)
  - All-IDs count badge (when active)
- **All-IDs Mode Card**: Dedicated toggle with description

**Why separate?**
- Professional dashboard layout
- Better organization of all controls
- Easy visual scanning of metrics
- Plenty of space for each element

---

### Tab 3: **MAP** 🗺️
**Purpose:** Spatial visualization (placeholder, ready for future enhancement)

**Contents:**
- Informational card about map view
- Button to open server dashboard
- Ready for integration with `/map_image` and `/robot_pose` endpoints

**Why separate?**
- Dedicated space for 2D/3D visualization
- Doesn't compete with camera feed
- Can be extended independently

---

### Tab 4: **SETTINGS** ⚙️
**Purpose:** Configuration and server control

**Contents:**
- **Server Connection Card**: 
  - Text field for server IP/URL
  - Connect button
  - Connection status text
- **Calibration Card**:
  - Distance input (cm)
  - Calibrate button
  - Help text
- **Server Control Card**:
  - Stop Server button (red)
  - Description

**Why separate?**
- Keeps settings from cluttering other tabs
- One-time configuration flow
- Safe destructive operations isolated

---

## Design Improvements

### Visual Hierarchy
- **Card-based design** in Dashboard, Settings, Map
- Each card = one logical function
- Consistent spacing (12px margins, 8px internal)
- Clear section headings with `titleMedium` text style

### Color Coding
- **Green**: Active, selected, or optimal state (ID selected, all-IDs ON)
- **Orange**: Secondary state (tracked in all-IDs mode)
- **Grey**: Inactive, untracked, or help text
- **Red**: Destructive actions (Stop Server)
- **Teal**: Data viewing (All-IDs Telemetry popup)

### Typography
- **AppBar Title**: `titleLarge` (24px)
- **Card Titles**: `titleMedium` (18px)
- **Body Text**: Default (14px)
- **Help Text**: Small grey (12px)
- **Telemetry Values**: Bold white (14px)

### Interaction Patterns
- **Tabs**: Swipe left/right to navigate or tap tab directly
- **Cards**: Scrollable settings tab, fixed dashboard cards
- **Buttons**: Full-width in cards for touch accuracy
- **Feedback**: All buttons have visual states (disabled = grey)

---

## Integration Changes

### Server Integration
The app now better separates concerns:
- **Camera Tab** talks to `/video_feed` endpoint
- **Dashboard Tab** polls `/state` for telemetry
- **Settings Tab** sends commands (`/set_id`, `/calibrate`, `/quit`)
- All API calls go through existing HTTP methods

### No Breaking Changes
- All existing API endpoints unchanged
- All existing functionality preserved
- Enhanced server (`aruco_server_enhanced.py`) optional
- Original server (`aruco_server.py`) still works

---

## User Flow Examples

### Flow 1: Standard Monitoring
1. Settings Tab → Enter server IP → Connect
2. Camera Tab → Watch live feed + telemetry
3. Dashboard Tab → View metrics or switch IDs
4. Back to Camera Tab for continuous monitoring

### Flow 2: Multi-ID Detection
1. Dashboard Tab → Turn on "All-IDs Mode"
2. Dashboard Tab → View tracked IDs in grid (orange)
3. Dashboard Tab → Click "View All-IDs Telemetry"
4. Select a marker to lock onto it
5. Camera Tab → Monitor selected marker

### Flow 3: Calibration
1. Place car at known distance
2. Settings Tab → Enter distance
3. Click "Calibrate Distance"
4. Distance measurements now accurate

---

## Technical Details

### State Management
```dart
class _HomePageState extends State<HomePage> with TickerProviderStateMixin {
  late TabController _tabController;  // Controls tab switching
  
  // Shared state (read by all tabs)
  String? _baseUrl;
  Map<String, dynamic> _state;
  bool _allIdsMode;
  Map<int, Map<String, dynamic>> _allIdsTelemetry;
  int? _preSelectedId;
}
```

### Build Method Structure
```
Scaffold
├─ AppBar
│  ├─ Logo + Title
│  ├─ Connection indicator (green/grey dot)
│  └─ TabBar (4 tabs)
└─ TabBarView
   ├─ Tab 1: _buildCameraTab()
   ├─ Tab 2: _buildDashboardTab()
   ├─ Tab 3: _buildMapTab()
   └─ Tab 4: _buildSettingsTab()
```

### Responsive Design
- Tablets: Same layout, larger text
- Phones: Optimized for portrait orientation
- All cards use `crossAxisAlignment: CrossAxisAlignment.stretch`
- All buttons full-width within containers
- ScrollableColumns where needed

---

## Migration from Old Layout

### What Changed
| Old | New |
|-----|-----|
| Single column scroll | Tab-based sections |
| WebView at top | Full Camera tab |
| Buttons scattered | Organized Dashboard cards |
| Settings mixed in | Dedicated Settings tab |
| All state visible | Focused on current task |

### What's Same
- All API endpoints
- All state management
- All functionality
- All dialogs (Pre-select, All-IDs Telemetry)

---

## Future Enhancements

### Map Tab Integration
```dart
// Replace placeholder with:
Widget _buildMapTab() {
  return Column(
    children: [
      _buildMapImage(),      // GET /map_image → PNG
      _buildRobotPosition(), // GET /robot_pose → JSON
      _buildParkingSpots(),  // GET /parking_spots → JSON
    ],
  );
}
```

### Additional Tabs (Optional)
- **Logs**: Server output and debug messages
- **Statistics**: Session analytics (total distance, time, accuracy)
- **Presets**: Save/load calibrations and configurations

---

## Performance

- **APK Size**: 46.6 MB (same as before)
- **Memory**: Slight increase due to tab controller (~2-5 MB)
- **Build Time**: ~40s (same)
- **Startup Time**: <1s (same)

---

## Testing Checklist

- [ ] Connect to server from Settings tab
- [ ] Camera feed loads in Camera tab
- [ ] Telemetry updates in real-time
- [ ] ID grid selects IDs correctly
- [ ] All-IDs mode tracks markers
- [ ] Calibration field submits correctly
- [ ] Pre-select dialog shows all 16 IDs
- [ ] Tab switching works smoothly
- [ ] AppBar connection indicator updates
- [ ] All buttons are responsive
- [ ] Text fields accept input correctly
- [ ] Dialogs close properly

---

## Deployment

### Build Command
```bash
cd C:\aruco_project\mobile_app
flutter build apk --release
```

### Output
`build/app/outputs/flutter-apk/app-release.apk` (46.6 MB)

### Installation
```bash
adb install -r app-release.apk
```

---

## Support

### Common Issues

**Q: Tab is blank when I switch to it**
- A: Connection might be lost. Check Settings → Connection Status

**Q: Camera feed is black**
- A: Ensure camera is active on server. Check Dashboard → Telemetry

**Q: Can't connect to server**
- A: Use format `IP:PORT` without `http://`. App adds protocol automatically.

---

Generated: 2025-01-09
Version: 2.0 (Tabbed Interface)
