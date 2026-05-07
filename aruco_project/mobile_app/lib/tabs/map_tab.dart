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
    NavGoalDialog.show(
      context,
      conn,
      initialX: double.parse(coords[0].toStringAsFixed(2)),
      initialY: double.parse(coords[1].toStringAsFixed(2)),
    );
  }

  /// Long-press = pose reset. The user is asserting "this is where the robot
  /// actually is right now"; we publish PoseWithCovarianceStamped on
  /// /initialpose. AMCL or SLAM Toolbox snaps to the new pose.
  Future<void> _onLongPressMap(
      BuildContext context, LongPressStartDetails details, ConnectionService conn) async {
    final coords = _tapToMapCoords(details.localPosition, conn.robotStatus.mapInfo);
    if (coords == null || conn.mapImage == null) return;
    final yawCtrl = TextEditingController(
        text: conn.robotStatus.pose.yaw.toStringAsFixed(2));
    final ok = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Reset robot pose here?'),
        content: Column(mainAxisSize: MainAxisSize.min, children: [
          Text('x: ${coords[0].toStringAsFixed(2)}  y: ${coords[1].toStringAsFixed(2)}',
              style: const TextStyle(fontFamily: 'monospace')),
          const SizedBox(height: 8),
          TextField(
            controller: yawCtrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(labelText: 'Yaw (rad)'),
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
    if (ok == true) {
      final yaw = double.tryParse(yawCtrl.text) ?? 0.0;
      await conn.relocalize(coords[0], coords[1], yaw);
    }
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
                    : GestureDetector(
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
  }

  @override
  bool shouldRepaint(covariant _MapPainter oldDelegate) => true;
}
