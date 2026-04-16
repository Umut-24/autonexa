import 'dart:convert';

/// A single navigation waypoint.
class Waypoint {
  final double x;
  final double y;
  final double yaw;
  final String? label;

  const Waypoint({
    required this.x,
    required this.y,
    this.yaw = 0,
    this.label,
  });

  Map<String, dynamic> toJson() => {
    'x': x,
    'y': y,
    'yaw': yaw,
    if (label != null) 'label': label,
  };

  factory Waypoint.fromJson(Map<String, dynamic> json) => Waypoint(
    x: (json['x'] ?? 0).toDouble(),
    y: (json['y'] ?? 0).toDouble(),
    yaw: (json['yaw'] ?? 0).toDouble(),
    label: json['label']?.toString(),
  );
}

/// A sequence of waypoints to execute as a mission.
class Mission {
  final String name;
  final List<Waypoint> waypoints;
  final DateTime createdAt;

  Mission({
    required this.name,
    required this.waypoints,
    DateTime? createdAt,
  }) : createdAt = createdAt ?? DateTime.now();

  String toJsonString() => jsonEncode({
    'name': name,
    'waypoints': waypoints.map((w) => w.toJson()).toList(),
    'createdAt': createdAt.toIso8601String(),
  });

  factory Mission.fromJsonString(String s) {
    final json = jsonDecode(s) as Map<String, dynamic>;
    return Mission(
      name: json['name'] ?? 'Unnamed',
      waypoints: (json['waypoints'] as List? ?? [])
          .map((w) => Waypoint.fromJson(w as Map<String, dynamic>))
          .toList(),
      createdAt: DateTime.tryParse(json['createdAt'] ?? '') ?? DateTime.now(),
    );
  }
}

enum MissionExecutionState { idle, running, paused, completed, failed }
