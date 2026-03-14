import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'dart:ui' as ui;
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;

/// Real-time map view with LIDAR scan overlay and robot pose.
///
/// Renders:
/// 1. SLAM occupancy grid (fetched as PNG from /api/map)
/// 2. LIDAR scan points in map frame (from /api/scan)
/// 3. Robot pose arrow (from /api/pose)
///
/// All coordinates in meters, map frame (consistent with ROS2).
class LidarMapView extends StatefulWidget {
  final String baseUrl;

  const LidarMapView({super.key, required this.baseUrl});

  @override
  State<LidarMapView> createState() => _LidarMapViewState();
}

class _LidarMapViewState extends State<LidarMapView> {
  Timer? _statusTimer;
  Timer? _mapTimer;

  // Map image
  ui.Image? _mapImage;
  MapInfo? _mapInfo;

  // Scan points (meters, map frame)
  List<List<double>> _scanPoints = [];
  int _scanCount = 0;
  double? _scanAge;

  // Robot pose (meters, map frame)
  double _robotX = 0;
  double _robotY = 0;
  double _robotYaw = 0;
  String _poseSource = 'none';

  // Connection state
  bool _connected = false;
  String? _error;

  @override
  void initState() {
    super.initState();
    _startPolling();
  }

  @override
  void dispose() {
    _statusTimer?.cancel();
    _mapTimer?.cancel();
    super.dispose();
  }

  @override
  void didUpdateWidget(LidarMapView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.baseUrl != widget.baseUrl) {
      _statusTimer?.cancel();
      _mapTimer?.cancel();
      _startPolling();
    }
  }

  void _startPolling() {
    // Poll status (scan + pose) at 200ms
    _statusTimer = Timer.periodic(const Duration(milliseconds: 200), (_) => _fetchStatus());
    // Poll map image at 2s (map changes slowly)
    _mapTimer = Timer.periodic(const Duration(seconds: 2), (_) => _fetchMap());
    // Initial fetch
    _fetchMap();
    _fetchStatus();
  }

  Future<void> _fetchStatus() async {
    try {
      final r = await http
          .get(Uri.parse('${widget.baseUrl}/api/status'))
          .timeout(const Duration(seconds: 2));
      if (r.statusCode == 200) {
        final data = json.decode(r.body) as Map<String, dynamic>;
        final pose = data['pose'] as Map<String, dynamic>? ?? {};
        final scan = data['scan'] as Map<String, dynamic>? ?? {};
        final mapData = data['map'] as Map<String, dynamic>?;

        // Update map info if available from status
        if (mapData != null && mapData.containsKey('resolution')) {
          _mapInfo = MapInfo.fromJson(mapData);
        }

        setState(() {
          _robotX = (pose['x_m'] as num?)?.toDouble() ?? 0;
          _robotY = (pose['y_m'] as num?)?.toDouble() ?? 0;
          _robotYaw = (pose['yaw_rad'] as num?)?.toDouble() ?? 0;
          _poseSource = pose['source'] as String? ?? 'none';
          _scanCount = (scan['count'] as num?)?.toInt() ?? 0;
          _scanAge = (scan['age_s'] as num?)?.toDouble();
          _connected = true;
          _error = null;
        });

        // Fetch scan points separately (can be large)
        _fetchScan();
      }
    } catch (e) {
      setState(() {
        _connected = false;
        _error = 'Connection failed';
      });
    }
  }

  Future<void> _fetchScan() async {
    try {
      final r = await http
          .get(Uri.parse('${widget.baseUrl}/api/scan'))
          .timeout(const Duration(seconds: 2));
      if (r.statusCode == 200) {
        final data = json.decode(r.body) as Map<String, dynamic>;
        final points = (data['points'] as List?)
            ?.map((p) => [(p[0] as num).toDouble(), (p[1] as num).toDouble()])
            .toList() ?? [];
        setState(() => _scanPoints = points);
      }
    } catch (_) {}
  }

  Future<void> _fetchMap() async {
    try {
      final r = await http
          .get(Uri.parse('${widget.baseUrl}/api/map'))
          .timeout(const Duration(seconds: 5));
      if (r.statusCode == 200) {
        final codec = await ui.instantiateImageCodec(r.bodyBytes);
        final frame = await codec.getNextFrame();
        setState(() => _mapImage = frame.image);
      }

      // Also fetch map info
      final infoR = await http
          .get(Uri.parse('${widget.baseUrl}/api/map_info'))
          .timeout(const Duration(seconds: 2));
      if (infoR.statusCode == 200) {
        final data = json.decode(infoR.body) as Map<String, dynamic>;
        setState(() => _mapInfo = MapInfo.fromJson(data));
      }
    } catch (_) {}
  }

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Column(
        children: [
          // Status bar
          _buildStatusBar(),
          // Map view
          Expanded(
            child: Container(
              margin: const EdgeInsets.all(8),
              decoration: BoxDecoration(
                color: Colors.black,
                borderRadius: BorderRadius.circular(12),
                border: Border.all(color: Colors.white12),
              ),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(12),
                child: _buildMapContent(),
              ),
            ),
          ),
          // Info bar
          _buildInfoBar(),
        ],
      ),
    );
  }

  Widget _buildStatusBar() {
    final scanStatus = _scanAge != null
        ? (_scanAge! < 1.0 ? 'Live' : '${_scanAge!.toStringAsFixed(1)}s ago')
        : 'No data';

    return Container(
      margin: const EdgeInsets.fromLTRB(8, 4, 8, 0),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.white10,
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          Container(
            width: 8,
            height: 8,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: _connected ? Colors.green : Colors.red,
            ),
          ),
          const SizedBox(width: 8),
          Text(
            _connected ? 'ROS2 Bridge' : (_error ?? 'Disconnected'),
            style: TextStyle(
              fontSize: 12,
              color: _connected ? Colors.green : Colors.red,
            ),
          ),
          const Spacer(),
          Text(
            'Scan: $scanStatus ($_scanCount pts)',
            style: const TextStyle(fontSize: 11, color: Colors.grey),
          ),
          const SizedBox(width: 12),
          Text(
            'Pose: $_poseSource',
            style: TextStyle(
              fontSize: 11,
              color: _poseSource == 'amcl' ? Colors.green : Colors.orange,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMapContent() {
    if (!_connected && _mapImage == null) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.map, size: 48, color: Colors.grey),
            const SizedBox(height: 12),
            Text(
              _error ?? 'Waiting for ROS2 bridge...',
              style: const TextStyle(color: Colors.grey),
            ),
            const SizedBox(height: 8),
            Text(
              widget.baseUrl,
              style: const TextStyle(fontSize: 11, color: Colors.grey),
            ),
          ],
        ),
      );
    }

    return InteractiveViewer(
      minScale: 0.5,
      maxScale: 10.0,
      child: LayoutBuilder(
        builder: (context, constraints) {
          return CustomPaint(
            size: Size(constraints.maxWidth, constraints.maxHeight),
            painter: _MapPainter(
              mapImage: _mapImage,
              mapInfo: _mapInfo,
              scanPoints: _scanPoints,
              robotX: _robotX,
              robotY: _robotY,
              robotYaw: _robotYaw,
            ),
          );
        },
      ),
    );
  }

  Widget _buildInfoBar() {
    return Container(
      margin: const EdgeInsets.fromLTRB(8, 0, 8, 8),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      decoration: BoxDecoration(
        color: Colors.white10,
        borderRadius: BorderRadius.circular(8),
      ),
      child: Row(
        children: [
          _infoChip('X', '${_robotX.toStringAsFixed(2)}m'),
          const SizedBox(width: 12),
          _infoChip('Y', '${_robotY.toStringAsFixed(2)}m'),
          const SizedBox(width: 12),
          _infoChip('Yaw', '${(_robotYaw * 180 / pi).toStringAsFixed(1)}°'),
          const Spacer(),
          if (_mapInfo != null)
            Text(
              '${_mapInfo!.width}x${_mapInfo!.height} @ ${(_mapInfo!.resolution * 100).toStringAsFixed(0)}cm/px',
              style: const TextStyle(fontSize: 10, color: Colors.grey),
            ),
        ],
      ),
    );
  }

  Widget _infoChip(String label, String value) {
    return Row(
      children: [
        Text(label, style: const TextStyle(fontSize: 10, color: Colors.grey)),
        const SizedBox(width: 4),
        Text(value, style: const TextStyle(fontSize: 12, fontWeight: FontWeight.bold, fontFamily: 'monospace')),
      ],
    );
  }
}

// =============================================================================
//  Map metadata
// =============================================================================

class MapInfo {
  final int width;
  final int height;
  final double resolution;
  final double originX;
  final double originY;

  const MapInfo({
    required this.width,
    required this.height,
    required this.resolution,
    required this.originX,
    required this.originY,
  });

  factory MapInfo.fromJson(Map<String, dynamic> json) {
    return MapInfo(
      width: (json['width'] as num?)?.toInt() ?? 0,
      height: (json['height'] as num?)?.toInt() ?? 0,
      resolution: (json['resolution'] as num?)?.toDouble() ?? 0.05,
      originX: (json['origin_x'] as num?)?.toDouble() ?? 0,
      originY: (json['origin_y'] as num?)?.toDouble() ?? 0,
    );
  }
}

// =============================================================================
//  CustomPainter for map + scan + robot
// =============================================================================

class _MapPainter extends CustomPainter {
  final ui.Image? mapImage;
  final MapInfo? mapInfo;
  final List<List<double>> scanPoints;
  final double robotX;
  final double robotY;
  final double robotYaw;

  _MapPainter({
    this.mapImage,
    this.mapInfo,
    required this.scanPoints,
    required this.robotX,
    required this.robotY,
    required this.robotYaw,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final bgPaint = Paint()..color = const Color(0xFF1A1A2E);
    canvas.drawRect(Rect.fromLTWH(0, 0, size.width, size.height), bgPaint);

    if (mapImage == null || mapInfo == null || mapInfo!.width == 0 || mapInfo!.height == 0) {
      // Draw scan points relative to robot if no map
      _drawScanNoMap(canvas, size);
      return;
    }

    final info = mapInfo!;
    final img = mapImage!;

    // Compute scale to fit map image into widget
    final imgW = img.width.toDouble();
    final imgH = img.height.toDouble();
    final scaleX = size.width / imgW;
    final scaleY = size.height / imgH;
    final scale = min(scaleX, scaleY);

    // Center the map
    final offsetX = (size.width - imgW * scale) / 2;
    final offsetY = (size.height - imgH * scale) / 2;

    // Draw occupancy grid
    final src = Rect.fromLTWH(0, 0, imgW, imgH);
    final dst = Rect.fromLTWH(offsetX, offsetY, imgW * scale, imgH * scale);
    canvas.drawImageRect(img, src, dst, Paint()..filterQuality = FilterQuality.low);

    // Coordinate transform: meters (map frame) -> widget pixels
    // Map frame: origin at (info.originX, info.originY), each cell = info.resolution meters
    // Image: Y is flipped (top of image = top of flipped map)
    Offset metersToPixels(double mx, double my) {
      final px = (mx - info.originX) / info.resolution * scale + offsetX;
      // Flip Y: image was flipped in the bridge, so image row 0 = map top
      final py = (imgH - (my - info.originY) / info.resolution) * scale + offsetY;
      return Offset(px, py);
    }

    // Draw LIDAR scan points
    final scanPaint = Paint()
      ..color = const Color(0xFF00FF88)
      ..style = PaintingStyle.fill;
    for (final pt in scanPoints) {
      final pos = metersToPixels(pt[0], pt[1]);
      if (pos.dx >= offsetX && pos.dx <= offsetX + imgW * scale &&
          pos.dy >= offsetY && pos.dy <= offsetY + imgH * scale) {
        canvas.drawCircle(pos, 1.5, scanPaint);
      }
    }

    // Draw robot pose
    final robotPos = metersToPixels(robotX, robotY);
    _drawRobotArrow(canvas, robotPos.dx, robotPos.dy, robotYaw, scale);
  }

  void _drawScanNoMap(Canvas canvas, Size size) {
    if (scanPoints.isEmpty) return;

    // Auto-scale: find bounding box of scan points + robot
    double minX = robotX, maxX = robotX, minY = robotY, maxY = robotY;
    for (final pt in scanPoints) {
      if (pt[0] < minX) minX = pt[0];
      if (pt[0] > maxX) maxX = pt[0];
      if (pt[1] < minY) minY = pt[1];
      if (pt[1] > maxY) maxY = pt[1];
    }
    final rangeX = (maxX - minX).clamp(1.0, double.infinity);
    final rangeY = (maxY - minY).clamp(1.0, double.infinity);
    final margin = 0.1;
    final scale = min(
      size.width / (rangeX * (1 + margin * 2)),
      size.height / (rangeY * (1 + margin * 2)),
    );
    final cx = size.width / 2;
    final cy = size.height / 2;
    final midX = (minX + maxX) / 2;
    final midY = (minY + maxY) / 2;

    Offset toPixels(double mx, double my) {
      return Offset(
        cx + (mx - midX) * scale,
        cy - (my - midY) * scale, // Y flipped
      );
    }

    // Grid lines
    final gridPaint = Paint()..color = const Color(0xFF222244)..strokeWidth = 0.5;
    final gridStep = _niceGridStep(max(rangeX, rangeY));
    final gridMinX = (minX / gridStep).floor() * gridStep;
    final gridMinY = (minY / gridStep).floor() * gridStep;
    for (var gx = gridMinX; gx <= maxX; gx += gridStep) {
      final p1 = toPixels(gx, minY);
      final p2 = toPixels(gx, maxY);
      canvas.drawLine(p1, p2, gridPaint);
    }
    for (var gy = gridMinY; gy <= maxY; gy += gridStep) {
      final p1 = toPixels(minX, gy);
      final p2 = toPixels(maxX, gy);
      canvas.drawLine(p1, p2, gridPaint);
    }

    // Scan points
    final scanPaint = Paint()..color = const Color(0xFF00FF88);
    for (final pt in scanPoints) {
      canvas.drawCircle(toPixels(pt[0], pt[1]), 1.5, scanPaint);
    }

    // Robot
    final rp = toPixels(robotX, robotY);
    _drawRobotArrow(canvas, rp.dx, rp.dy, robotYaw, scale);
  }

  double _niceGridStep(double range) {
    final raw = range / 10;
    final mag = pow(10, (log(raw) / ln10).floor()).toDouble();
    final residual = raw / mag;
    if (residual <= 1) return mag;
    if (residual <= 2) return 2 * mag;
    if (residual <= 5) return 5 * mag;
    return 10 * mag;
  }

  void _drawRobotArrow(Canvas canvas, double rx, double ry, double yaw, double mapScale) {
    final arrowLen = (20.0 * mapScale).clamp(12.0, 40.0);
    final bodyPaint = Paint()
      ..color = const Color(0xFFFF4444)
      ..style = PaintingStyle.fill;
    final outlinePaint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;

    // Robot body circle
    canvas.drawCircle(Offset(rx, ry), 6, bodyPaint);
    canvas.drawCircle(Offset(rx, ry), 6, outlinePaint);

    // Heading arrow — note: screen Y is inverted vs map Y
    final tipX = rx + arrowLen * cos(-yaw);
    final tipY = ry + arrowLen * sin(-yaw);
    final arrowPaint = Paint()
      ..color = const Color(0xFFFF4444)
      ..strokeWidth = 2.5
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(Offset(rx, ry), Offset(tipX, tipY), arrowPaint);

    // Arrow head
    final headLen = arrowLen * 0.35;
    final headAngle = 0.5;
    final h1x = tipX - headLen * cos(-yaw + headAngle);
    final h1y = tipY - headLen * sin(-yaw + headAngle);
    final h2x = tipX - headLen * cos(-yaw - headAngle);
    final h2y = tipY - headLen * sin(-yaw - headAngle);
    final headPath = Path()
      ..moveTo(tipX, tipY)
      ..lineTo(h1x, h1y)
      ..moveTo(tipX, tipY)
      ..lineTo(h2x, h2y);
    canvas.drawPath(headPath, arrowPaint);
  }

  @override
  bool shouldRepaint(covariant _MapPainter old) {
    return old.mapImage != mapImage ||
        old.scanPoints != scanPoints ||
        old.robotX != robotX ||
        old.robotY != robotY ||
        old.robotYaw != robotYaw;
  }
}
