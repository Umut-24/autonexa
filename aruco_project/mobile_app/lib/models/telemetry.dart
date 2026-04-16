/// Telemetry data received from the RPi5 bridge (Pico via ROS2).
class PicoTelemetry {
  final double leftVel;
  final double rightVel;
  final double steerPos;
  final double odomVx;
  final double odomWz;
  final double odomX;
  final double odomY;
  final double odomYaw;
  final bool connected;

  // Battery & power
  final double batteryVoltage;    // Volts (0 = unknown)
  final double batteryCurrent;    // Amps (0 = unknown)
  final int batteryPercent;       // 0-100 (-1 = unknown)

  // Obstacle proximity (min distance from scan, meters)
  final double minObstacleDistance;

  const PicoTelemetry({
    this.leftVel = 0,
    this.rightVel = 0,
    this.steerPos = 0,
    this.odomVx = 0,
    this.odomWz = 0,
    this.odomX = 0,
    this.odomY = 0,
    this.odomYaw = 0,
    this.connected = false,
    this.batteryVoltage = 0,
    this.batteryCurrent = 0,
    this.batteryPercent = -1,
    this.minObstacleDistance = 999,
  });

  factory PicoTelemetry.fromJson(Map<String, dynamic> json) {
    return PicoTelemetry(
      leftVel: (json['left_wheel_vel'] ?? json['left_wheel_joint_vel'] ?? 0).toDouble(),
      rightVel: (json['right_wheel_vel'] ?? json['right_wheel_joint_vel'] ?? 0).toDouble(),
      steerPos: (json['steering_pos'] ?? json['steering_joint_pos'] ?? 0).toDouble(),
      odomVx: (json['odom_vx'] ?? 0).toDouble(),
      odomWz: (json['odom_wz'] ?? 0).toDouble(),
      odomX: (json['odom_x'] ?? 0).toDouble(),
      odomY: (json['odom_y'] ?? 0).toDouble(),
      odomYaw: (json['odom_yaw'] ?? 0).toDouble(),
      connected: true,
      batteryVoltage: (json['battery_voltage'] ?? json['bat_v'] ?? 0).toDouble(),
      batteryCurrent: (json['battery_current'] ?? json['bat_a'] ?? 0).toDouble(),
      batteryPercent: (json['battery_percent'] ?? json['bat_pct'] ?? -1).toInt(),
      minObstacleDistance: (json['min_obstacle_dist'] ?? 999).toDouble(),
    );
  }

  /// Estimate battery percent from voltage if not reported directly.
  /// Assumes 2S LiPo (6.0V dead - 8.4V full).
  int get estimatedPercent {
    if (batteryPercent >= 0) return batteryPercent;
    if (batteryVoltage <= 0) return -1;
    final pct = ((batteryVoltage - 6.0) / (8.4 - 6.0) * 100).clamp(0, 100);
    return pct.round();
  }

  /// True if obstacle is within warning distance (< 15cm).
  bool get obstacleWarning => minObstacleDistance < 0.15;

  /// True if obstacle is within critical distance (< 5cm).
  bool get obstacleCritical => minObstacleDistance < 0.05;
}
