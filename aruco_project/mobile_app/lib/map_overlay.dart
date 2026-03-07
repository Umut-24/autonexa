/// Enhanced Flutter UI components for map + camera overlay
/// Add these to your main.dart in the mobile app

import 'dart:async';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:typed_data';

/// Robot pose data from /robot_pose endpoint
class RobotPose {
  final double x_cm;
  final double y_cm;
  final double theta_deg;
  final DateTime timestamp;

  RobotPose({
    required this.x_cm,
    required this.y_cm,
    required this.theta_deg,
    required this.timestamp,
  });

  factory RobotPose.fromJson(Map<String, dynamic> json) {
    return RobotPose(
      x_cm: (json['x_cm'] as num).toDouble(),
      y_cm: (json['y_cm'] as num).toDouble(),
      theta_deg: (json['theta_deg'] as num).toDouble(),
      timestamp: DateTime.fromMillisecondsSinceEpoch(
        (json['timestamp'] as num).toInt() * 1000,
      ),
    );
  }
}

/// Parking spot (detected ArUco marker)
class ParkingSpot {
  final int id;
  final double x_cm;
  final double y_cm;
  final double bearing_deg;
  final double distance_cm;

  ParkingSpot({
    required this.id,
    required this.x_cm,
    required this.y_cm,
    required this.bearing_deg,
    required this.distance_cm,
  });

  factory ParkingSpot.fromJson(Map<String, dynamic> json) {
    return ParkingSpot(
      id: json['id'] as int,
      x_cm: (json['x_cm'] as num).toDouble(),
      y_cm: (json['y_cm'] as num).toDouble(),
      bearing_deg: (json['bearing_deg'] as num).toDouble(),
      distance_cm: (json['distance_cm'] as num).toDouble(),
    );
  }
}

/// Map with camera overlay widget
class MapWithCameraOverlay extends StatefulWidget {
  final String baseUrl;
  final int? selectedId;
  final VoidCallback? onMapTap;

  const MapWithCameraOverlay({
    Key? key,
    required this.baseUrl,
    this.selectedId,
    this.onMapTap,
  }) : super(key: key);

  @override
  State<MapWithCameraOverlay> createState() => _MapWithCameraOverlayState();
}

class _MapWithCameraOverlayState extends State<MapWithCameraOverlay> {
  Uint8List? mapImage;
  RobotPose? robotPose;
  List<ParkingSpot> parkingSpots = [];
  Timer? mapUpdateTimer;
  Timer? poseUpdateTimer;
  Timer? spotsUpdateTimer;

  // Testbed dimensions (cm)
  static const double TESTBED_WIDTH = 200;
  static const double TESTBED_HEIGHT = 200;
  static const double MAP_PIXEL_SCALE = 2; // 1 cm = 2 pixels

  double get mapWidth => TESTBED_WIDTH * MAP_PIXEL_SCALE;
  double get mapHeight => TESTBED_HEIGHT * MAP_PIXEL_SCALE;

  @override
  void initState() {
    super.initState();
    _startUpdatingMap();
    _startUpdatingPose();
    _startUpdatingSpots();
  }

  @override
  void dispose() {
    mapUpdateTimer?.cancel();
    poseUpdateTimer?.cancel();
    spotsUpdateTimer?.cancel();
    super.dispose();
  }

  void _startUpdatingMap() {
    mapUpdateTimer = Timer.periodic(Duration(milliseconds: 500), (_) async {
      if (widget.baseUrl.isEmpty) return;
      try {
        final response = await http
            .get(Uri.parse('${widget.baseUrl}/map_image'))
            .timeout(Duration(seconds: 2));
        if (response.statusCode == 200) {
          setState(() => mapImage = response.bodyBytes);
        }
      } catch (_) {}
    });
  }

  void _startUpdatingPose() {
    poseUpdateTimer = Timer.periodic(Duration(milliseconds: 100), (_) async {
      if (widget.baseUrl.isEmpty) return;
      try {
        final response = await http
            .get(Uri.parse('${widget.baseUrl}/robot_pose'))
            .timeout(Duration(seconds: 2));
        if (response.statusCode == 200) {
          final json = jsonDecode(response.body);
          setState(() => robotPose = RobotPose.fromJson(json));
        }
      } catch (_) {}
    });
  }

  void _startUpdatingSpots() {
    spotsUpdateTimer = Timer.periodic(Duration(seconds: 1), (_) async {
      if (widget.baseUrl.isEmpty) return;
      try {
        final response = await http
            .get(Uri.parse('${widget.baseUrl}/parking_spots'))
            .timeout(Duration(seconds: 2));
        if (response.statusCode == 200) {
          final json = jsonDecode(response.body) as List;
          setState(
            () => parkingSpots =
                json.map((e) => ParkingSpot.fromJson(e)).toList(),
          );
        }
      } catch (_) {}
    });
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
        border: Border.all(color: Colors.white24),
        borderRadius: BorderRadius.circular(8),
      ),
      child: mapImage == null
          ? Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  CircularProgressIndicator(),
                  SizedBox(height: 16),
                  Text('Loading map...'),
                ],
              ),
            )
          : Stack(
              children: [
                // Background map image
                Image.memory(
                  mapImage!,
                  fit: BoxFit.contain,
                  width: double.infinity,
                ),

                // Overlay: parking spots
                ...parkingSpots.map((spot) {
                  final isSelected = spot.id == widget.selectedId;
                  return Positioned(
                    left: spot.x_cm * MAP_PIXEL_SCALE - 8,
                    top: spot.y_cm * MAP_PIXEL_SCALE - 8,
                    child: GestureDetector(
                      onTap: () {
                        // Optional: navigate to this spot
                        if (widget.onMapTap != null) widget.onMapTap!();
                      },
                      child: Container(
                        width: 16,
                        height: 16,
                        decoration: BoxDecoration(
                          shape: BoxShape.circle,
                          color: isSelected ? Colors.red : Colors.blue,
                          boxShadow: [
                            BoxShadow(
                              color: isSelected ? Colors.red : Colors.blue,
                              blurRadius: 6,
                              spreadRadius: 1,
                            ),
                          ],
                        ),
                        child: Center(
                          child: Text(
                            '${spot.id}',
                            style: TextStyle(
                              fontSize: 8,
                              color: Colors.white,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                        ),
                      ),
                    ),
                  );
                }).toList(),

                // Overlay: robot pose (handled by server-side map image, but we can add client-side overlay)
                // This layer is redundant since server draws it, but kept for illustration
              ],
            ),
    );
  }
}

/// Mini camera feed overlay widget (to be placed on map)
class CameraFeedOverlay extends StatelessWidget {
  final String videoFeedUrl;
  final EdgeInsets position;

  const CameraFeedOverlay({
    Key? key,
    required this.videoFeedUrl,
    this.position = const EdgeInsets.only(bottom: 10, left: 10),
  }) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Positioned(
      bottom: position.bottom,
      left: position.left,
      child: Container(
        width: 120,
        height: 90,
        decoration: BoxDecoration(
          border: Border.all(color: Colors.white, width: 2),
          borderRadius: BorderRadius.circular(4),
          color: Colors.black,
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(2),
          child: Image.network(
            videoFeedUrl,
            fit: BoxFit.cover,
            errorBuilder: (context, error, stackTrace) => Center(
              child: Icon(Icons.videocam_off, color: Colors.white54, size: 32),
            ),
            loadingBuilder: (context, child, loadingProgress) {
              if (loadingProgress == null) return child;
              return Center(
                child: SizedBox(
                  width: 30,
                  height: 30,
                  child: CircularProgressIndicator(
                    strokeWidth: 2,
                    value: loadingProgress.expectedTotalBytes != null
                        ? loadingProgress.cumulativeBytesLoaded /
                            loadingProgress.expectedTotalBytes!
                        : null,
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );
  }
}

/// Integrated view: Map + Camera + Telemetry
class AutonomousParkingView extends StatefulWidget {
  final String baseUrl;

  const AutonomousParkingView({Key? key, required this.baseUrl}) : super(key: key);

  @override
  State<AutonomousParkingView> createState() => _AutonomousParkingViewState();
}

class _AutonomousParkingViewState extends State<AutonomousParkingView> {
  int? selectedId;
  RobotPose? robotPose;
  Map<String, dynamic>? telemetry;

  @override
  void initState() {
    super.initState();
    _pollTelemetry();
  }

  void _pollTelemetry() async {
    while (mounted) {
      try {
        final resp = await http
            .get(Uri.parse('${widget.baseUrl}/state'))
            .timeout(Duration(seconds: 2));
        if (resp.statusCode == 200) {
          setState(() => telemetry = jsonDecode(resp.body));
          selectedId = telemetry?['target_id'];
        }
      } catch (_) {}
      await Future.delayed(Duration(milliseconds: 200));
    }
  }

  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        // Map with camera overlay
        Expanded(
          child: Stack(
            children: [
              MapWithCameraOverlay(
                baseUrl: widget.baseUrl,
                selectedId: selectedId,
              ),
              // Camera feed overlay (bottom-left corner)
              CameraFeedOverlay(
                videoFeedUrl: '${widget.baseUrl}/video_feed',
                position: EdgeInsets.only(bottom: 10, left: 10),
              ),
            ],
          ),
        ),
        // Telemetry panel
        Container(
          color: Colors.black26,
          padding: EdgeInsets.all(12),
          child: telemetry == null
              ? Text('Connecting...')
              : Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Target: ID ${telemetry?['target_id'] ?? '-'} | '
                      'Distance: ${telemetry?['distance_cm']?.toStringAsFixed(1) ?? '-'} cm | '
                      'Bearing: ${telemetry?['bearing']?.toStringAsFixed(1) ?? '-'}°',
                      style: TextStyle(
                        fontSize: 14,
                        fontWeight: FontWeight.bold,
                        color: Colors.greenAccent,
                      ),
                    ),
                    SizedBox(height: 4),
                    Text(
                      'Position: X=${telemetry?['tx_cm']?.toStringAsFixed(1) ?? '-'} cm, '
                      'Y=${telemetry?['ty_cm']?.toStringAsFixed(1) ?? '-'} cm',
                      style: TextStyle(fontSize: 12, color: Colors.white70),
                    ),
                  ],
                ),
        ),
      ],
    );
  }
}
