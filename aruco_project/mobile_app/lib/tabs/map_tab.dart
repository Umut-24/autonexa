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

  void _onTapMap(BuildContext context, TapUpDetails details, ConnectionService conn) {
    final mapInfo = conn.robotStatus.mapInfo;
    if (mapInfo == null || conn.mapImage == null) return;

    // Get the tap position in the InteractiveViewer's coordinate space
    final matrix = _transformController.value;
    final inverseMatrix = Matrix4.inverted(matrix);
    final localPoint = MatrixUtils.transformPoint(inverseMatrix, details.localPosition);

    // Convert pixel coords to map coords (image origin top-left, ROS origin bottom-left)
    final pixelX = localPoint.dx;
    final pixelY = localPoint.dy;
    final mapX = mapInfo.originX + pixelX * mapInfo.resolution;
    final mapY = mapInfo.originY + (mapInfo.height - pixelY) * mapInfo.resolution;

    NavGoalDialog.show(
      context,
      conn,
      initialX: double.parse(mapX.toStringAsFixed(2)),
      initialY: double.parse(mapY.toStringAsFixed(2)),
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
                      : 'Tap on the map to set a navigation goal',
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
