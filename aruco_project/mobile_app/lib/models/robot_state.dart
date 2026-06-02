/// Robot pose in the map frame.
class RobotPose {
  final double x;
  final double y;
  final double yaw;
  final String source;
  final double stamp;

  const RobotPose({
    this.x = 0,
    this.y = 0,
    this.yaw = 0,
    this.source = 'none',
    this.stamp = 0,
  });

  factory RobotPose.fromJson(Map<String, dynamic> json) {
    return RobotPose(
      x: (json['x_m'] ?? 0).toDouble(),
      y: (json['y_m'] ?? 0).toDouble(),
      yaw: (json['yaw_rad'] ?? 0).toDouble(),
      source: json['source']?.toString() ?? 'none',
      stamp: (json['stamp'] ?? 0).toDouble(),
    );
  }
}

/// Detected ArUco marker information.
class MarkerInfo {
  final int id;
  final double distance;
  final double bearing;
  final DateTime lastSeen;

  const MarkerInfo({
    required this.id,
    required this.distance,
    required this.bearing,
    required this.lastSeen,
  });

  factory MarkerInfo.fromJson(int id, Map<String, dynamic> json) {
    return MarkerInfo(
      id: id,
      distance: (json['distance_m'] ?? 0).toDouble(),
      bearing: (json['bearing_deg'] ?? 0).toDouble(),
      lastSeen: DateTime.now(),
    );
  }

  /// Freshness status based on time since last detection.
  MarkerStatus get status {
    final age = DateTime.now().difference(lastSeen);
    if (age.inSeconds < 2) return MarkerStatus.live;
    if (age.inSeconds < 10) return MarkerStatus.stale;
    return MarkerStatus.lost;
  }
}

enum MarkerStatus { live, stale, lost }

/// Current Nav2 goal pose — populated from /api/goal. `active` is false when
/// no goal is in flight (e.g. just cancelled).
class NavGoal {
  final double x;
  final double y;
  final double yaw;
  final bool active;
  final double stamp;

  const NavGoal({
    this.x = 0,
    this.y = 0,
    this.yaw = 0,
    this.active = false,
    this.stamp = 0,
  });

  factory NavGoal.fromJson(Map<String, dynamic> json) {
    return NavGoal(
      x: (json['x'] ?? 0).toDouble(),
      y: (json['y'] ?? 0).toDouble(),
      yaw: (json['yaw'] ?? 0).toDouble(),
      active: json['active'] == true,
      stamp: (json['stamp'] ?? 0).toDouble(),
    );
  }
}

/// Map metadata from /api/map_info.
class MapInfo {
  final int width;
  final int height;
  final double resolution;
  final double originX;
  final double originY;
  final String mapId;
  final String mapMode;
  final String mapYaml;
  final String manifest;

  const MapInfo({
    this.width = 0,
    this.height = 0,
    this.resolution = 0.05,
    this.originX = 0,
    this.originY = 0,
    this.mapId = '',
    this.mapMode = '',
    this.mapYaml = '',
    this.manifest = '',
  });

  factory MapInfo.fromJson(Map<String, dynamic> json) {
    return MapInfo(
      width: (json['width'] ?? 0).toInt(),
      height: (json['height'] ?? 0).toInt(),
      resolution: (json['resolution'] ?? 0.05).toDouble(),
      originX: (json['origin_x'] ?? 0).toDouble(),
      originY: (json['origin_y'] ?? 0).toDouble(),
      mapId: (json['map_id'] ?? json['map_fingerprint'] ?? '').toString(),
      mapMode: (json['map_mode'] ?? '').toString(),
      mapYaml: (json['map_yaml'] ?? '').toString(),
      manifest: (json['manifest'] ?? '').toString(),
    );
  }
}

/// Scan info from /api/status.
class ScanInfo {
  final int count;
  final double? age;

  const ScanInfo({this.count = 0, this.age});

  factory ScanInfo.fromJson(Map<String, dynamic> json) {
    return ScanInfo(
      count: (json['count'] ?? 0).toInt(),
      age: json['age_s']?.toDouble(),
    );
  }
}

/// Combined status response from /api/status.
class RobotStatus {
  final RobotPose pose;
  final ScanInfo scan;
  final MapInfo? mapInfo;
  final Map<int, MarkerInfo> markers;

  const RobotStatus({
    this.pose = const RobotPose(),
    this.scan = const ScanInfo(),
    this.mapInfo,
    this.markers = const {},
  });

  factory RobotStatus.fromJson(Map<String, dynamic> json) {
    final markerMap = <int, MarkerInfo>{};
    if (json['markers'] is Map) {
      (json['markers'] as Map).forEach((key, value) {
        final id = int.tryParse(key.toString());
        if (id != null && value is Map<String, dynamic>) {
          markerMap[id] = MarkerInfo.fromJson(id, value);
        }
      });
    }

    return RobotStatus(
      pose: json['pose'] is Map<String, dynamic>
          ? RobotPose.fromJson(json['pose'])
          : const RobotPose(),
      scan: json['scan'] is Map<String, dynamic>
          ? ScanInfo.fromJson(json['scan'])
          : const ScanInfo(),
      mapInfo: json['map'] is Map<String, dynamic>
          ? MapInfo.fromJson(json['map'])
          : null,
      markers: markerMap,
    );
  }
}
