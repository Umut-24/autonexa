import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../models/robot_state.dart';
import '../widgets/glass_card.dart';
import '../widgets/nav_goal_dialog.dart';

/// RViz-style map view: SLAM occupancy grid as background, with overlays for
/// LiDAR scan, robot pose, planner path, current goal, and ArUco markers.
/// Tap-to-navigate sends a Nav2 goal; the prominent "Cancel Goal" pill aborts
/// any active goal at any instant.
class MapTab extends StatefulWidget {
  const MapTab({super.key});

  @override
  State<MapTab> createState() => _MapTabState();
}

class _MapTabState extends State<MapTab> {
  Timer? _scanTimer;
  final TransformationController _transformController = TransformationController();
  bool _showScan = true;
  bool _showMarkers = true;
  bool _showTrail = true;
  bool _showPlan = true;

  // Most recent tap in map-frame meters. Surfaced in the bottom info bar so
  // the user can sanity-check the tap-to-coordinate math against the robot's
  // displayed pose / known wall locations / RViz when debugging.
  double? _lastTapMx;
  double? _lastTapMy;
  double? _lastTapYaw;

  // Decoded map image — rebuilt asynchronously when the PNG bytes change so
  // the painter can blit it directly via canvas.drawImage. Caching by length
  // is good enough as a change-detection cheat (the bridge increments
  // map_version on every emit, so the bytes' length nearly always changes).
  ui.Image? _decodedMap;
  int _decodedMapLen = -1;

  @override
  void initState() {
    super.initState();
    // Poll scan points at 5 Hz so the LiDAR overlay feels live (was 1 Hz).
    // SLAMTEC C1 publishes /scan at ~10 Hz, so 5 Hz on the app side stays
    // well under wire bandwidth while looking continuous to the eye.
    _scanTimer = Timer.periodic(const Duration(milliseconds: 200), (_) {
      final conn = context.read<ConnectionService>();
      if (conn.isConnected) conn.fetchScan();
    });
  }

  @override
  void dispose() {
    _scanTimer?.cancel();
    _transformController.dispose();
    _decodedMap?.dispose();
    super.dispose();
  }

  Future<void> _maybeDecodeMap(Uint8List? bytes) async {
    if (bytes == null) {
      if (_decodedMap != null) {
        _decodedMap?.dispose();
        _decodedMap = null;
        _decodedMapLen = -1;
        if (mounted) setState(() {});
      }
      return;
    }
    if (bytes.length == _decodedMapLen && _decodedMap != null) return;
    final codec = await ui.instantiateImageCodec(bytes);
    final frame = await codec.getNextFrame();
    if (!mounted) return;
    setState(() {
      _decodedMap?.dispose();
      _decodedMap = frame.image;
      _decodedMapLen = bytes.length;
    });
  }

  /// Convert a screen-local tap into map-frame meters. Returns null if the
  /// map metadata isn't loaded yet. Uses a `[x, y]` 2-list to stay compatible
  /// with the project's pre-Dart-3 SDK constraint.
  List<double>? _tapToMapCoords(Offset localPos, MapInfo? mapInfo) {
    if (mapInfo == null) return null;
    final matrix = _transformController.value;
    final inverseMatrix = Matrix4.inverted(matrix);
    final localPoint = MatrixUtils.transformPoint(inverseMatrix, localPos);
    final mapX = mapInfo.originX + localPoint.dx * mapInfo.resolution;
    final mapY = mapInfo.originY + (mapInfo.height - localPoint.dy) * mapInfo.resolution;
    return [mapX, mapY];
  }

  void _onTapMap(BuildContext context, TapUpDetails details, ConnectionService conn) {
    final coords = _tapToMapCoords(details.localPosition, conn.robotStatus.mapInfo);
    if (coords == null || conn.mapImage == null) return;
    // Auto-yaw: have the robot face TOWARD the tapped point (its direction of
    // approach), so the planner doesn't have to construct contorted paths
    // just to satisfy a hardcoded yaw=0. User can still override in the
    // dialog before sending.
    final pose = conn.robotStatus.pose;
    final autoYaw = math.atan2(coords[1] - pose.y, coords[0] - pose.x);
    setState(() {
      _lastTapMx = coords[0];
      _lastTapMy = coords[1];
      _lastTapYaw = autoYaw;
    });
    NavGoalDialog.show(
      context,
      conn,
      initialX: double.parse(coords[0].toStringAsFixed(2)),
      initialY: double.parse(coords[1].toStringAsFixed(2)),
      initialYaw: double.parse(autoYaw.toStringAsFixed(2)),
    );
  }

  /// Long-press = spot selection. If the press landed near a saved
  /// waypoint, open its context menu (Navigate / Rename / Delete). If it
  /// landed on empty floor, open a full picker of all saved waypoints
  /// (tap-to-navigate). Pose reset is no longer bound to long-press —
  /// it now lives in the ⋮ menu (see `_showResetMenu`).
  Future<void> _onLongPressMap(
      BuildContext context, LongPressStartDetails details, ConnectionService conn) async {
    final coords = _tapToMapCoords(details.localPosition, conn.robotStatus.mapInfo);
    if (coords == null || conn.mapImage == null) return;

    // Waypoint hit-test in map-frame meters. 0.30 m gives fat-finger
    // forgiveness — the marker glyph is small relative to a phone tap.
    NamedWaypoint? hit;
    double hitDist = 0.30;
    for (final wp in conn.namedWaypoints) {
      final d = math.sqrt(
          math.pow(wp.x - coords[0], 2) + math.pow(wp.y - coords[1], 2));
      if (d < hitDist) {
        hit = wp;
        hitDist = d;
      }
    }
    if (hit != null) {
      await _waypointContextMenu(context, conn, hit);
      return;
    }
    await _waypointPicker(context, conn);
  }

  /// Bottom sheet shown when long-press lands on empty floor — lists every
  /// saved waypoint; tap one to dispatch a Nav2 goal there.
  Future<void> _waypointPicker(
      BuildContext context, ConnectionService conn) async {
    final colors = context.read<ThemeProvider>().colors;
    final wps = conn.namedWaypoints;
    if (wps.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
        content: Text('No saved spots — add one in the Parking tab.'),
      ));
      return;
    }
    final picked = await showModalBottomSheet<NamedWaypoint>(
      context: context,
      backgroundColor: colors.surface,
      isScrollControlled: true,
      builder: (ctx) => SafeArea(
        child: Padding(
          padding: const EdgeInsets.fromLTRB(16, 16, 16, 8),
          child: Column(mainAxisSize: MainAxisSize.min, children: [
            const Row(children: [
              Icon(Icons.place_rounded, color: AppColors.info),
              SizedBox(width: 8),
              Text('Go to a saved spot',
                  style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
            ]),
            const SizedBox(height: 12),
            Flexible(
              child: ListView.separated(
                shrinkWrap: true,
                itemCount: wps.length,
                separatorBuilder: (_, __) => const SizedBox(height: 4),
                itemBuilder: (_, i) {
                  final wp = wps[i];
                  final kindColor = wp.kind == 'park'
                      ? AppColors.success
                      : wp.kind == 'summon'
                          ? AppColors.info
                          : wp.kind == 'home'
                              ? AppColors.brand
                              : colors.textTertiary;
                  return ListTile(
                    leading: Icon(Icons.location_on_rounded,
                        color: wp.stale ? colors.textTertiary : kindColor),
                    title: Text(wp.name,
                        style: const TextStyle(fontWeight: FontWeight.w600)),
                    subtitle: Text(
                      '${wp.kind}  •  (${wp.x.toStringAsFixed(2)}, '
                      '${wp.y.toStringAsFixed(2)})'
                      '  yaw ${(wp.yaw * 57.2958).toStringAsFixed(0)}°'
                      '${wp.stale ? '  • stale (map changed)' : ''}',
                      style: const TextStyle(fontSize: 11, fontFamily: 'monospace'),
                    ),
                    enabled: !wp.stale,
                    onTap: wp.stale ? null : () => Navigator.pop(ctx, wp),
                  );
                },
              ),
            ),
            const SizedBox(height: 4),
            TextButton(
              onPressed: () => Navigator.pop(ctx),
              child: const Text('Cancel'),
            ),
          ]),
        ),
      ),
    );
    if (picked == null) return;
    final ok = await conn.navigateToNamedWaypoint(picked.name);
    if (!context.mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      content: Text(ok ? 'Going to "${picked.name}"' : 'Navigate failed'),
    ));
  }

  /// Pose-reset dialog. Replaces the long-press flow; now invoked from
  /// the ⋮ menu. Pre-fills the current pose so a small correction is one
  /// edit away.
  Future<void> _resetPoseDialog(BuildContext context, ConnectionService conn) async {
    final pose = conn.robotStatus.pose;
    final xCtrl = TextEditingController(text: pose.x.toStringAsFixed(2));
    final yCtrl = TextEditingController(text: pose.y.toStringAsFixed(2));
    final yawCtrl = TextEditingController(text: pose.yaw.toStringAsFixed(2));
    final ok = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Reset robot pose'),
        content: Column(mainAxisSize: MainAxisSize.min, children: [
          const Text(
            'Publish PoseWithCovarianceStamped on /initialpose so AMCL or '
            'SLAM Toolbox snaps the robot to a known location.',
            style: TextStyle(fontSize: 12),
          ),
          const SizedBox(height: 12),
          TextField(
            controller: xCtrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(labelText: 'x (m)'),
          ),
          TextField(
            controller: yCtrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(labelText: 'y (m)'),
          ),
          TextField(
            controller: yawCtrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(labelText: 'yaw (rad)'),
          ),
        ]),
        actions: [
          TextButton(onPressed: () => Navigator.pop(ctx, false), child: const Text('Cancel')),
          ElevatedButton(
            style: ElevatedButton.styleFrom(backgroundColor: AppColors.warning),
            onPressed: () => Navigator.pop(ctx, true),
            child: const Text('Set pose'),
          ),
        ],
      ),
    );
    if (ok != true) return;
    final x = double.tryParse(xCtrl.text) ?? pose.x;
    final y = double.tryParse(yCtrl.text) ?? pose.y;
    final yaw = double.tryParse(yawCtrl.text) ?? pose.yaw;
    final success = await conn.relocalize(x, y, yaw);
    if (!context.mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      content: Text(success ? 'Pose reset to ($x, $y)' : 'Pose reset failed'),
    ));
  }

  /// Bottom sheet shown when a long-press lands on a saved waypoint marker.
  Future<void> _waypointContextMenu(
      BuildContext context, ConnectionService conn, NamedWaypoint wp) async {
    final colors = context.read<ThemeProvider>().colors;
    await showModalBottomSheet(
      context: context,
      backgroundColor: colors.surface,
      builder: (ctx) => SafeArea(
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 14, 16, 4),
            child: Row(children: [
              Icon(Icons.location_on_rounded,
                  color: wp.kind == 'park'
                      ? AppColors.success
                      : wp.kind == 'summon'
                          ? AppColors.info
                          : AppColors.brand),
              const SizedBox(width: 8),
              Expanded(
                child: Text(wp.name,
                    style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
              ),
              Text(wp.kind,
                  style: TextStyle(fontSize: 11, color: colors.textTertiary)),
            ]),
          ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Text(
              '(${wp.x.toStringAsFixed(2)}, ${wp.y.toStringAsFixed(2)})'
              '  yaw ${(wp.yaw * 57.2958).toStringAsFixed(0)}°'
              '${wp.stale ? '  • stale' : ''}',
              style: TextStyle(
                  fontSize: 11, fontFamily: 'monospace', color: colors.textSecondary),
            ),
          ),
          const Divider(),
          ListTile(
            leading: const Icon(Icons.navigation_rounded, color: AppColors.info),
            title: const Text('Navigate here'),
            enabled: !wp.stale,
            onTap: wp.stale
                ? null
                : () async {
                    Navigator.pop(ctx);
                    final ok = await conn.navigateToNamedWaypoint(wp.name);
                    if (!context.mounted) return;
                    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                      content: Text(
                          ok ? 'Going to "${wp.name}"' : 'Navigate failed'),
                    ));
                  },
          ),
          ListTile(
            leading: const Icon(Icons.edit_rounded, color: AppColors.brand),
            title: const Text('Rename'),
            onTap: () async {
              Navigator.pop(ctx);
              await _renameWaypoint(context, conn, wp);
            },
          ),
          ListTile(
            leading: const Icon(Icons.delete_outline_rounded, color: AppColors.danger),
            title: const Text('Delete'),
            onTap: () async {
              Navigator.pop(ctx);
              final ok = await conn.deleteNamedWaypoint(wp.name);
              if (!context.mounted) return;
              ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                content: Text(ok ? 'Deleted "${wp.name}"' : 'Delete failed'),
              ));
            },
          ),
          const SizedBox(height: 8),
        ]),
      ),
    );
  }

  /// Rename a waypoint via DELETE-old + POST-new (the bridge keys waypoints
  /// by name, so this is an atomic-enough swap).
  Future<void> _renameWaypoint(
      BuildContext context, ConnectionService conn, NamedWaypoint wp) async {
    final ctrl = TextEditingController(text: wp.name);
    final newName = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Rename waypoint'),
        content: TextField(
          controller: ctrl,
          autofocus: true,
          decoration: const InputDecoration(labelText: 'Name'),
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx), child: const Text('Cancel')),
          ElevatedButton(
            onPressed: () => Navigator.pop(ctx, ctrl.text.trim()),
            child: const Text('Save'),
          ),
        ],
      ),
    );
    if (newName == null || newName.isEmpty || newName == wp.name) return;
    // Upsert under the new name first; if that fails we still have the old
    // entry untouched. Only delete the old name on success.
    final saved = await conn.saveNamedWaypoint(
      name: newName, kind: wp.kind, x: wp.x, y: wp.y, yaw: wp.yaw,
    );
    if (saved == null) {
      if (!context.mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Rename failed: could not save new name')));
      return;
    }
    await conn.deleteNamedWaypoint(wp.name);
    if (!context.mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Renamed to "$newName"')));
  }

  Future<void> _showResetMenu(BuildContext context, ConnectionService conn) async {
    final colors = context.read<ThemeProvider>().colors;
    await showModalBottomSheet(
      context: context,
      backgroundColor: colors.surface,
      builder: (ctx) => SafeArea(
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          ListTile(
            leading: const Icon(Icons.refresh_rounded, color: AppColors.info),
            title: const Text('Clear obstacles'),
            subtitle: const Text('Wipe global + local costmaps. Keeps the SLAM map.'),
            onTap: () async {
              Navigator.pop(ctx);
              final ok = await conn.clearCostmaps();
              if (!context.mounted) return;
              ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                  content: Text(ok ? 'Costmaps cleared' : 'Costmap clear failed')));
            },
          ),
          ListTile(
            leading: const Icon(Icons.my_location_rounded, color: AppColors.warning),
            title: const Text('Reset robot pose…'),
            subtitle: const Text(
                'Tell SLAM / AMCL where the robot actually is right now.'),
            onTap: () {
              Navigator.pop(ctx);
              _resetPoseDialog(context, conn);
            },
          ),
          ListTile(
            leading: const Icon(Icons.restart_alt_rounded, color: AppColors.warning),
            title: const Text('Restart mapping'),
            subtitle: const Text('Drops the current SLAM map. Robot pose resets to origin.'),
            onTap: () async {
              Navigator.pop(ctx);
              final confirm = await showDialog<bool>(
                context: context,
                builder: (c2) => AlertDialog(
                  title: const Text('Restart mapping?'),
                  content: const Text(
                      'The current map will be discarded. Use this when relocating the robot to a new area.'),
                  actions: [
                    TextButton(onPressed: () => Navigator.pop(c2, false), child: const Text('Cancel')),
                    ElevatedButton(
                      style: ElevatedButton.styleFrom(backgroundColor: AppColors.warning),
                      onPressed: () => Navigator.pop(c2, true),
                      child: const Text('Restart SLAM'),
                    ),
                  ],
                ),
              );
              if (confirm == true) {
                final ok = await conn.restartMapping();
                if (!context.mounted) return;
                ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                    content: Text(ok ? 'SLAM restarted' : 'SLAM restart failed')));
              }
            },
          ),
        ]),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final conn = context.watch<ConnectionService>();
    final mapImage = conn.mapImage;
    final status = conn.robotStatus;

    // Kick off async decode if the bytes changed.
    _maybeDecodeMap(mapImage);

    final goal = conn.currentGoal;

    return SafeArea(
      child: Column(
        children: [
          // Header
          Padding(
            padding: const EdgeInsets.fromLTRB(20, 12, 20, 8),
            child: Row(
              children: [
                Text('Map',
                    style: TextStyle(fontSize: 22, fontWeight: FontWeight.w700,
                        color: colors.textPrimary, letterSpacing: -0.5)),
                const Spacer(),
                _toggleChip('Scan', _showScan, () => setState(() => _showScan = !_showScan), colors),
                const SizedBox(width: 6),
                _toggleChip('Plan', _showPlan, () => setState(() => _showPlan = !_showPlan), colors),
                const SizedBox(width: 6),
                _toggleChip('Trail', _showTrail, () => setState(() => _showTrail = !_showTrail), colors),
                const SizedBox(width: 6),
                _toggleChip('Markers', _showMarkers, () => setState(() => _showMarkers = !_showMarkers), colors),
                const SizedBox(width: 6),
                IconButton(
                  iconSize: 20,
                  padding: EdgeInsets.zero,
                  visualDensity: VisualDensity.compact,
                  tooltip: 'Reset map / costmaps',
                  icon: Icon(Icons.more_vert_rounded, color: colors.textSecondary),
                  onPressed: conn.isConnected ? () => _showResetMenu(context, conn) : null,
                ),
              ],
            ),
          ),

          // Map view
          Expanded(
            child: Container(
              margin: const EdgeInsets.fromLTRB(16, 0, 16, 8),
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(16),
                border: Border.all(color: colors.border),
                color: colors.surface,
              ),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(16),
                child: mapImage == null || !conn.isConnected || _decodedMap == null
                    ? _placeholder(colors)
                    : Stack(
                        children: [
                          GestureDetector(
                            onTapUp: (d) => _onTapMap(context, d, conn),
                            onLongPressStart: (d) => _onLongPressMap(context, d, conn),
                            child: InteractiveViewer(
                              transformationController: _transformController,
                              minScale: 0.5,
                              maxScale: 10,
                              constrained: false,
                              child: CustomPaint(
                                painter: _MapPainter(
                                  mapImage: _decodedMap!,
                                  pose: status.pose,
                                  markers: _showMarkers ? status.markers : {},
                                  scanPoints: _showScan ? conn.scanPoints : [],
                                  pathTrail: _showTrail ? conn.pathTrail : [],
                                  plannedPath: _showPlan ? conn.plannedPath : [],
                                  goal: goal.active ? goal : null,
                                  waypoints: conn.namedWaypoints,
                                  mapInfo: status.mapInfo,
                                  accentColor: colors.accent,
                                ),
                                size: Size(
                                  _decodedMap!.width.toDouble(),
                                  _decodedMap!.height.toDouble(),
                                ),
                              ),
                            ),
                          ),
                          // Cardinal-direction compass: pinned to top-right,
                          // does NOT pan/zoom with the map. Lets the user
                          // verify which world axis the screen is showing as
                          // "up" — should match RViz's orientation.
                          const Positioned(
                            top: 8,
                            right: 8,
                            child: _MapCompass(),
                          ),
                        ],
                      ),
              ),
            ),
          ),

          // Info bar
          GlassCard(
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
            child: Row(
              children: [
                _infoItem('X', '${status.pose.x.toStringAsFixed(2)}m', colors),
                _infoItem('Y', '${status.pose.y.toStringAsFixed(2)}m', colors),
                _infoItem('Yaw', '${(status.pose.yaw * 57.2958).toStringAsFixed(0)}°', colors),
                _infoItem('Scan', '${status.scan.count}', colors),
                _infoItem('Plan', '${conn.plannedPath.length}', colors),
                _infoItem('Nav2', conn.navStatus, colors),
                _infoItem(
                  'Last tap',
                  _lastTapMx == null
                      ? '—'
                      : '${_lastTapMx!.toStringAsFixed(2)}, ${_lastTapMy!.toStringAsFixed(2)} '
                        '@${(_lastTapYaw! * 57.2958).toStringAsFixed(0)}°',
                  colors,
                ),
              ],
            ),
          ),

          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
            child: Row(
              children: [
                Text(
                  goal.active
                      ? 'Goal: (${goal.x.toStringAsFixed(2)}, ${goal.y.toStringAsFixed(2)})'
                      : 'Tap to set a goal · long-press to reset robot pose',
                  style: TextStyle(fontSize: 11, color: colors.textTertiary),
                ),
                const Spacer(),
                if (conn.pathTrail.isNotEmpty)
                  GestureDetector(
                    onTap: () => conn.clearPathTrail(),
                    child: Container(
                      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                      decoration: BoxDecoration(
                        color: colors.surfaceLight,
                        borderRadius: BorderRadius.circular(6),
                      ),
                      child: Text('Clear trail',
                          style: TextStyle(fontSize: 10, color: colors.textSecondary)),
                    ),
                  ),
                if (goal.active) ...[
                  const SizedBox(width: 8),
                  GestureDetector(
                    onTap: () => conn.cancelNavGoal(),
                    child: Container(
                      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                      decoration: BoxDecoration(
                        color: const Color(0xFFE53935),
                        borderRadius: BorderRadius.circular(8),
                        boxShadow: [
                          BoxShadow(
                            color: const Color(0xFFE53935).withValues(alpha: 0.4),
                            blurRadius: 6,
                            offset: const Offset(0, 2),
                          ),
                        ],
                      ),
                      child: const Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Icon(Icons.cancel_rounded, size: 14, color: Colors.white),
                          SizedBox(width: 4),
                          Text('Cancel Goal',
                              style: TextStyle(fontSize: 11, fontWeight: FontWeight.w700,
                                  color: Colors.white, letterSpacing: 0.3)),
                        ],
                      ),
                    ),
                  ),
                ],
              ],
            ),
          ),
          const SizedBox(height: 8),
        ],
      ),
    );
  }

  Widget _toggleChip(String label, bool active, VoidCallback onTap, ResolvedColors colors) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
        decoration: BoxDecoration(
          color: active ? colors.accentSurface : colors.surfaceLight,
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: active ? colors.accent.withValues(alpha: 0.3) : colors.border),
        ),
        child: Text(label,
            style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600,
                color: active ? colors.accent : colors.textSecondary)),
      ),
    );
  }

  Widget _placeholder(ResolvedColors colors) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(Icons.map_rounded, size: 48, color: colors.textTertiary),
          const SizedBox(height: 14),
          Text('Connect to view LIDAR map',
              style: TextStyle(fontSize: 14, color: colors.textSecondary)),
        ],
      ),
    );
  }

  Widget _infoItem(String label, String value, ResolvedColors colors) {
    return Expanded(
      child: Column(
        children: [
          Text(label, style: TextStyle(fontSize: 9, color: colors.textTertiary,
              fontWeight: FontWeight.w600, letterSpacing: 0.8)),
          const SizedBox(height: 2),
          Text(value, style: TextStyle(fontSize: 12, fontFamily: 'monospace',
              fontWeight: FontWeight.w600, color: colors.textPrimary)),
        ],
      ),
    );
  }
}

// ─── Map Painter ─────────────────────────────────────────────────────────────

class _MapPainter extends CustomPainter {
  final ui.Image mapImage;
  final RobotPose pose;
  final Map<int, MarkerInfo> markers;
  final List<List<double>> scanPoints;
  final List<List<double>> pathTrail;
  final List<List<double>> plannedPath;
  final NavGoal? goal;
  final List<NamedWaypoint> waypoints;
  final MapInfo? mapInfo;
  final Color accentColor;

  _MapPainter({
    required this.mapImage,
    required this.pose,
    required this.markers,
    required this.scanPoints,
    required this.pathTrail,
    required this.plannedPath,
    required this.goal,
    this.waypoints = const [],
    this.mapInfo,
    required this.accentColor,
  });

  @override
  void paint(Canvas canvas, Size size) {
    // 1. Draw the SLAM occupancy grid as the background. Walls black, free
    //    white, unknown gray — exactly as the bridge encoded the PNG (with Y
    //    already flipped to image coords). Nearest-neighbor filtering keeps
    //    cell edges crisp at high zoom.
    final imgPaint = Paint()
      ..filterQuality = FilterQuality.none
      ..isAntiAlias = false;
    canvas.drawImage(mapImage, Offset.zero, imgPaint);

    if (mapInfo == null) return;

    final res = mapInfo!.resolution;
    final ox = mapInfo!.originX;
    final oy = mapInfo!.originY;
    final h = mapInfo!.height;

    // Helper: map coords (meters) -> pixel coords
    Offset toPixel(double mx, double my) {
      final px = (mx - ox) / res;
      final py = h - (my - oy) / res; // flip Y
      return Offset(px, py);
    }

    // 2. Planner path (orange polyline, RViz-style) — drawn under scan + pose
    //    so live data sits on top.
    if (plannedPath.length >= 2) {
      final planPaint = Paint()
        ..color = const Color(0xFFFF9500)
        ..strokeWidth = 2.5
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round
        ..style = PaintingStyle.stroke;
      final planPath = Path();
      final p0 = toPixel(plannedPath[0][0], plannedPath[0][1]);
      planPath.moveTo(p0.dx, p0.dy);
      for (int i = 1; i < plannedPath.length; i++) {
        final p = toPixel(plannedPath[i][0], plannedPath[i][1]);
        planPath.lineTo(p.dx, p.dy);
      }
      canvas.drawPath(planPath, planPaint);
    }

    // 3. Path trail (where the robot has been)
    if (pathTrail.length >= 2) {
      final trailPaint = Paint()
        ..color = accentColor.withValues(alpha: 0.45)
        ..strokeWidth = 1.8
        ..strokeCap = StrokeCap.round
        ..style = PaintingStyle.stroke;
      final path = Path();
      final first = toPixel(pathTrail[0][0], pathTrail[0][1]);
      path.moveTo(first.dx, first.dy);
      for (int i = 1; i < pathTrail.length; i++) {
        final pt = toPixel(pathTrail[i][0], pathTrail[i][1]);
        path.lineTo(pt.dx, pt.dy);
      }
      canvas.drawPath(path, trailPaint);
    }

    // 4. Scan points
    if (scanPoints.isNotEmpty) {
      final scanPaint = Paint()
        ..color = AppColors.success.withValues(alpha: 0.7)
        ..strokeCap = StrokeCap.round;
      for (final pt in scanPoints) {
        if (pt.length >= 2) {
          canvas.drawCircle(toPixel(pt[0], pt[1]), 1.2, scanPaint);
        }
      }
    }

    // 4.5. Named waypoints (manual park / summon / home spots). Drawn under
    //      the goal marker so an in-flight goal still pops; drawn over the
    //      scan/trail so they remain visible against the LiDAR overlay.
    if (waypoints.isNotEmpty) {
      for (final wp in waypoints) {
        final p = toPixel(wp.x, wp.y);
        Color color;
        switch (wp.kind) {
          case 'park': color = AppColors.brand; break;
          case 'summon': color = AppColors.info; break;
          case 'home': color = AppColors.success; break;
          default: color = const Color(0xFF6F87FF);
        }
        // Stale waypoints render half-faded so the user notices.
        final alpha = wp.stale ? 0.45 : 0.95;
        final fill = Paint()..color = color.withValues(alpha: alpha);
        final outline = Paint()
          ..color = Colors.white.withValues(alpha: 0.85)
          ..style = PaintingStyle.stroke
          ..strokeWidth = 1.0;
        const r = 5.5;
        final diamond = Path()
          ..moveTo(p.dx, p.dy - r)
          ..lineTo(p.dx + r, p.dy)
          ..lineTo(p.dx, p.dy + r)
          ..lineTo(p.dx - r, p.dy)
          ..close();
        canvas.drawPath(diamond, fill);
        canvas.drawPath(diamond, outline);
        // Heading tick — short stub so users can tell the saved orientation.
        final ya = 9.0;
        final ydx = ya * math.cos(-wp.yaw);
        final ydy = ya * math.sin(-wp.yaw);
        final yaPaint = Paint()
          ..color = color.withValues(alpha: alpha)
          ..strokeWidth = 1.6
          ..strokeCap = StrokeCap.round;
        canvas.drawLine(p, p + Offset(ydx, ydy), yaPaint);
        // Name label, small and offset so it doesn't overlap the marker.
        final tp = TextPainter(
          text: TextSpan(
            text: wp.name,
            style: TextStyle(
              fontSize: 9, fontWeight: FontWeight.w600,
              color: Colors.white.withValues(alpha: 0.95),
              shadows: const [Shadow(color: Color(0xCC000000), blurRadius: 2)],
            ),
          ),
          textDirection: TextDirection.ltr,
        )..layout();
        tp.paint(canvas, Offset(p.dx + r + 3, p.dy - r - 2));
      }
    }

    // 5. Goal marker (red diamond + heading arrow). RViz draws the same.
    if (goal != null) {
      final g = toPixel(goal!.x, goal!.y);
      final goalFill = Paint()..color = const Color(0xFFE53935);
      final goalEdge = Paint()
        ..color = Colors.white
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5;
      const r = 7.0;
      final diamond = Path()
        ..moveTo(g.dx, g.dy - r)
        ..lineTo(g.dx + r, g.dy)
        ..lineTo(g.dx, g.dy + r)
        ..lineTo(g.dx - r, g.dy)
        ..close();
      canvas.drawPath(diamond, goalFill);
      canvas.drawPath(diamond, goalEdge);
      // Heading arrow
      final ga = 14.0;
      final gdx = ga * math.cos(-goal!.yaw);
      final gdy = ga * math.sin(-goal!.yaw);
      final goalArrow = Paint()
        ..color = const Color(0xFFE53935)
        ..strokeWidth = 2.0
        ..strokeCap = StrokeCap.round;
      canvas.drawLine(g, g + Offset(gdx, gdy), goalArrow);
    }

    // 6. Robot pose (cyan/accent circle + heading arrow)
    final robotPos = toPixel(pose.x, pose.y);
    final robotPaint = Paint()..color = accentColor;
    canvas.drawCircle(robotPos, 6, robotPaint);
    final arrowLen = 14.0;
    final dx = arrowLen * math.cos(-pose.yaw);
    final dy = arrowLen * math.sin(-pose.yaw);
    final arrowPaint = Paint()
      ..color = accentColor
      ..strokeWidth = 2.5
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(robotPos, robotPos + Offset(dx, dy), arrowPaint);
    final outlinePaint = Paint()
      ..color = Colors.white.withValues(alpha: 0.5)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;
    canvas.drawCircle(robotPos, 6, outlinePaint);

    // 7. Debug: world-frame axis gizmo at (0, 0). Mirrors RViz's tf axes
    //    display so we can A/B compare orientation. RED = +X, GREEN = +Y.
    //    If these arrows point the same direction in the app and in RViz,
    //    the maps are oriented identically. If they don't, that's the bug.
    final origin = toPixel(0.0, 0.0);
    const axisLenM = 0.30; // 30 cm in world frame
    final pXEnd = toPixel(axisLenM, 0.0);
    final pYEnd = toPixel(0.0, axisLenM);
    final xPaint = Paint()
      ..color = AppColors.danger
      ..strokeWidth = 2.5
      ..strokeCap = StrokeCap.round;
    final yPaint = Paint()
      ..color = AppColors.success
      ..strokeWidth = 2.5
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(origin, pXEnd, xPaint);
    canvas.drawLine(origin, pYEnd, yPaint);
    // Origin dot
    canvas.drawCircle(origin, 2.5,
        Paint()..color = Colors.white.withValues(alpha: 0.95));
    // Axis tip labels
    void drawTip(Offset p, String label, Color color) {
      final tp = TextPainter(
        text: TextSpan(
          text: label,
          style: TextStyle(
            fontSize: 10, fontWeight: FontWeight.w700,
            color: color,
            shadows: const [Shadow(color: Color(0xCC000000), blurRadius: 2)],
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, Offset(p.dx + 3, p.dy - 6));
    }
    drawTip(pXEnd, 'X', AppColors.danger);
    drawTip(pYEnd, 'Y', AppColors.success);

    // 8. Debug: scale bar (0.5 m horizontal). Drawn in painter coords so it
    //    pans/zooms with the map; its on-screen length always represents
    //    0.5 m in the world frame.
    const scaleM = 0.5;
    final barLenPx = scaleM / mapInfo!.resolution;
    final barY = mapInfo!.height - 14.0; // ~14 px above the bottom edge
    final barXStart = 14.0;
    final barPaint = Paint()
      ..color = Colors.white.withValues(alpha: 0.9)
      ..strokeWidth = 2.0
      ..strokeCap = StrokeCap.square;
    canvas.drawLine(
      Offset(barXStart, barY),
      Offset(barXStart + barLenPx, barY),
      barPaint,
    );
    // Tick marks on each end
    canvas.drawLine(Offset(barXStart, barY - 4),
        Offset(barXStart, barY + 4), barPaint);
    canvas.drawLine(Offset(barXStart + barLenPx, barY - 4),
        Offset(barXStart + barLenPx, barY + 4), barPaint);
    final scaleTp = TextPainter(
      text: TextSpan(
        text: '0.5 m',
        style: TextStyle(
          fontSize: 10, fontWeight: FontWeight.w600,
          color: Colors.white.withValues(alpha: 0.95),
          shadows: const [Shadow(color: Color(0xCC000000), blurRadius: 2)],
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    scaleTp.paint(canvas, Offset(barXStart + barLenPx / 2 - scaleTp.width / 2,
        barY - 14));
  }

  @override
  bool shouldRepaint(covariant _MapPainter oldDelegate) => true;
}

// ─── Compass overlay ────────────────────────────────────────────────────────

/// Static cardinal-direction indicator pinned outside the InteractiveViewer
/// (so it does NOT pan/zoom with the map). Shows N/E/S/W around the rim and
/// labels the screen's "up" direction with the world axis it corresponds to.
///
/// With the bridge's `np.flipud` rendering, screen-up = world +Y, so this
/// always shows "+Y up". If the user sees that in RViz "screen up" maps to
/// a different world axis (e.g. +X, or -Y), that's the bug.
class _MapCompass extends StatelessWidget {
  const _MapCompass();

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.55),
        borderRadius: BorderRadius.circular(10),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          SizedBox(
            width: 36, height: 36,
            child: CustomPaint(painter: _CompassPainter()),
          ),
          const SizedBox(width: 8),
          const Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('screen↑',
                  style: TextStyle(fontSize: 9, color: Colors.white70,
                      fontWeight: FontWeight.w600, letterSpacing: 0.4)),
              Text('= world +Y',
                  style: TextStyle(fontSize: 11, color: Colors.white,
                      fontFamily: 'monospace', fontWeight: FontWeight.w700)),
            ],
          ),
        ],
      ),
    );
  }
}

class _CompassPainter extends CustomPainter {
  @override
  void paint(Canvas canvas, Size size) {
    final c = Offset(size.width / 2, size.height / 2);
    final r = size.width / 2 - 1;
    final ring = Paint()
      ..color = Colors.white.withValues(alpha: 0.7)
      ..strokeWidth = 1
      ..style = PaintingStyle.stroke;
    canvas.drawCircle(c, r, ring);
    // Arrow pointing UP (north on screen)
    final arrow = Path()
      ..moveTo(c.dx, c.dy - r + 4)
      ..lineTo(c.dx - 4, c.dy + 2)
      ..lineTo(c.dx + 4, c.dy + 2)
      ..close();
    canvas.drawPath(arrow,
        Paint()..color = Colors.redAccent.withValues(alpha: 0.95));
    // Cardinal letters around the rim (N at top of screen)
    void cardinal(String letter, Offset offset) {
      final tp = TextPainter(
        text: TextSpan(
          text: letter,
          style: const TextStyle(
            fontSize: 8, fontWeight: FontWeight.w800, color: Colors.white,
          ),
        ),
        textDirection: TextDirection.ltr,
      )..layout();
      tp.paint(canvas, offset - Offset(tp.width / 2, tp.height / 2));
    }
    cardinal('N', Offset(c.dx, c.dy - r + 6));
    cardinal('S', Offset(c.dx, c.dy + r - 6));
    cardinal('E', Offset(c.dx + r - 6, c.dy));
    cardinal('W', Offset(c.dx - r + 6, c.dy));
  }

  @override
  bool shouldRepaint(covariant _CompassPainter oldDelegate) => false;
}
