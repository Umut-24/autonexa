import 'dart:async';
import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../models/robot_state.dart';
import '../widgets/glass_card.dart';
import '../widgets/nav_goal_dialog.dart';

/// Native LiDAR map view with tap-to-navigate, robot pose overlay,
/// marker overlay, and scan point visualization.
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
    super.dispose();
  }

  void _onTapMap(BuildContext context, TapUpDetails details, ConnectionService conn) {
    final mapInfo = conn.robotStatus.mapInfo;
    if (mapInfo == null || conn.mapImage == null) return;

    // Get the tap position in the InteractiveViewer's coordinate space
    final matrix = _transformController.value;
    final inverseMatrix = Matrix4.inverted(matrix);
    final localPoint = MatrixUtils.transformPoint(inverseMatrix, details.localPosition);

    // Convert pixel coords to map coords
    final pixelX = localPoint.dx;
    final pixelY = localPoint.dy;

    // Map image is flipped vertically (ROS origin bottom-left)
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
                // Toggle buttons
                _toggleChip('Scan', _showScan, () => setState(() => _showScan = !_showScan), colors),
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
                child: mapImage == null || !conn.isConnected
                    ? _placeholder(colors)
                    : GestureDetector(
                        onTapUp: (d) => _onTapMap(context, d, conn),
                        child: InteractiveViewer(
                          transformationController: _transformController,
                          minScale: 0.5,
                          maxScale: 10,
                          child: CustomPaint(
                            painter: _MapPainter(
                              mapImage: mapImage,
                              pose: status.pose,
                              markers: _showMarkers ? status.markers : {},
                              scanPoints: _showScan ? conn.scanPoints : [],
                              pathTrail: _showTrail ? conn.pathTrail : [],
                              mapInfo: status.mapInfo,
                              accentColor: colors.accent,
                            ),
                            size: Size(
                              (status.mapInfo?.width ?? 200).toDouble(),
                              (status.mapInfo?.height ?? 200).toDouble(),
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
                _infoItem('Source', status.pose.source, colors),
              ],
            ),
          ),

          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Row(
              children: [
                Text(
                  'Tap on the map to set a navigation goal',
                  style: TextStyle(fontSize: 11, color: colors.textTertiary),
                ),
                const Spacer(),
                if (conn.pathTrail.isNotEmpty)
                  GestureDetector(
                    onTap: () => conn.clearPathTrail(),
                    child: Container(
                      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                      decoration: BoxDecoration(
                        color: colors.surfaceLight,
                        borderRadius: BorderRadius.circular(6),
                      ),
                      child: Text('Clear trail',
                          style: TextStyle(fontSize: 10, color: colors.textSecondary)),
                    ),
                  ),
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
  final List<int> mapImage;
  final RobotPose pose;
  final Map<int, MarkerInfo> markers;
  final List<List<double>> scanPoints;
  final List<List<double>> pathTrail;
  final MapInfo? mapInfo;
  final Color accentColor;

  _MapPainter({
    required this.mapImage,
    required this.pose,
    required this.markers,
    required this.scanPoints,
    required this.pathTrail,
    this.mapInfo,
    required this.accentColor,
  });

  @override
  void paint(Canvas canvas, Size size) {
    // The map image is rendered by the Image widget underneath via DecorationImage;
    // here we draw overlays.

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

    // Draw scan points
    if (scanPoints.isNotEmpty) {
      final scanPaint = Paint()
        ..color = AppColors.success.withValues(alpha: 0.6)
        ..strokeWidth = 2
        ..strokeCap = StrokeCap.round;

      for (final pt in scanPoints) {
        if (pt.length >= 2) {
          canvas.drawCircle(toPixel(pt[0], pt[1]), 1.5, scanPaint);
        }
      }
    }

    // Draw path trail
    if (pathTrail.length >= 2) {
      final trailPaint = Paint()
        ..color = accentColor.withValues(alpha: 0.5)
        ..strokeWidth = 2.0
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

      // Draw trail dots at intervals
      final dotPaint = Paint()
        ..color = accentColor.withValues(alpha: 0.35)
        ..style = PaintingStyle.fill;
      for (int i = 0; i < pathTrail.length; i += 10) {
        final pt = toPixel(pathTrail[i][0], pathTrail[i][1]);
        canvas.drawCircle(pt, 1.5, dotPaint);
      }
    }

    // Draw robot pose
    final robotPos = toPixel(pose.x, pose.y);
    final robotPaint = Paint()..color = accentColor;
    canvas.drawCircle(robotPos, 6, robotPaint);

    // Heading arrow
    final arrowLen = 14.0;
    final dx = arrowLen * math.cos(-pose.yaw);
    final dy = arrowLen * math.sin(-pose.yaw);
    final arrowPaint = Paint()
      ..color = accentColor
      ..strokeWidth = 2.5
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(robotPos, robotPos + Offset(dx, dy), arrowPaint);

    // Robot outline
    final outlinePaint = Paint()
      ..color = Colors.white.withValues(alpha: 0.4)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;
    canvas.drawCircle(robotPos, 6, outlinePaint);
  }

  @override
  bool shouldRepaint(covariant _MapPainter oldDelegate) => true;
}
