/// Robot physical dimensions, fetched from /api/robot_config.
///
/// Mirrors the keys in parking_system/scripts/parking_system/build_urdf.py
/// (DEFAULT_DIMENSIONS). The Flutter side uses these to draw the chassis
/// footprint + LiDAR marker overlay on the map tab and to populate the
/// Settings → Robot Dimensions editor.
class RobotConfig {
  final double chassisLength;
  final double chassisWidth;
  final double chassisHeight;
  final double wheelbase;
  final double lidarX;
  final double lidarY;
  final double lidarZ;
  final double cameraX;
  final double cameraZ;
  final double footprintPadding;

  const RobotConfig({
    this.chassisLength = 0.30,
    this.chassisWidth = 0.20,
    this.chassisHeight = 0.10,
    this.wheelbase = 0.25,
    this.lidarX = 0.01,
    this.lidarY = 0.00,
    this.lidarZ = 0.07,
    this.cameraX = 0.10,
    this.cameraZ = 0.05,
    this.footprintPadding = 0.01,
  });

  static const RobotConfig defaults = RobotConfig();

  factory RobotConfig.fromJson(Map<String, dynamic> json) {
    final dims = (json['dims'] as Map?) ?? json;
    double pick(String k, double fallback) =>
        (dims[k] as num?)?.toDouble() ?? fallback;
    return RobotConfig(
      chassisLength: pick('chassis_length', defaults.chassisLength),
      chassisWidth: pick('chassis_width', defaults.chassisWidth),
      chassisHeight: pick('chassis_height', defaults.chassisHeight),
      wheelbase: pick('wheelbase', defaults.wheelbase),
      lidarX: pick('lidar_x', defaults.lidarX),
      lidarY: pick('lidar_y', defaults.lidarY),
      lidarZ: pick('lidar_z', defaults.lidarZ),
      cameraX: pick('camera_x', defaults.cameraX),
      cameraZ: pick('camera_z', defaults.cameraZ),
      footprintPadding: pick('footprint_padding', defaults.footprintPadding),
    );
  }

  Map<String, dynamic> toJson() => {
        'chassis_length': chassisLength,
        'chassis_width': chassisWidth,
        'chassis_height': chassisHeight,
        'wheelbase': wheelbase,
        'lidar_x': lidarX,
        'lidar_y': lidarY,
        'lidar_z': lidarZ,
        'camera_x': cameraX,
        'camera_z': cameraZ,
        'footprint_padding': footprintPadding,
      };

  RobotConfig copyWith({
    double? chassisLength,
    double? chassisWidth,
    double? chassisHeight,
    double? wheelbase,
    double? lidarX,
    double? lidarY,
    double? lidarZ,
    double? cameraX,
    double? cameraZ,
    double? footprintPadding,
  }) {
    return RobotConfig(
      chassisLength: chassisLength ?? this.chassisLength,
      chassisWidth: chassisWidth ?? this.chassisWidth,
      chassisHeight: chassisHeight ?? this.chassisHeight,
      wheelbase: wheelbase ?? this.wheelbase,
      lidarX: lidarX ?? this.lidarX,
      lidarY: lidarY ?? this.lidarY,
      lidarZ: lidarZ ?? this.lidarZ,
      cameraX: cameraX ?? this.cameraX,
      cameraZ: cameraZ ?? this.cameraZ,
      footprintPadding: footprintPadding ?? this.footprintPadding,
    );
  }
}
