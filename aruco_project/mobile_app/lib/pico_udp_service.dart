import 'dart:async';
import 'dart:convert';
import 'package:http/http.dart' as http;

/// Telemetry data received from the RPi5 bridge (Pico via ROS2 or MicroPython).
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
  final String picoState;      // MicroPython mode: IDLE, DRIVE, TURN, FAILED
  final int leftTicks;         // MicroPython mode: raw encoder ticks
  final int rightTicks;
  final double headingDeg;     // MicroPython mode: heading in degrees

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
    this.picoState = '',
    this.leftTicks = 0,
    this.rightTicks = 0,
    this.headingDeg = 0,
  });

  factory PicoTelemetry.fromJson(Map<String, dynamic> json) {
    return PicoTelemetry(
      leftVel: (json['left_wheel_vel'] ?? 0).toDouble(),
      rightVel: (json['right_wheel_vel'] ?? 0).toDouble(),
      steerPos: (json['steering_pos'] ?? 0).toDouble(),
      odomVx: (json['odom_vx'] ?? 0).toDouble(),
      odomWz: (json['odom_wz'] ?? 0).toDouble(),
      odomX: (json['odom_x'] ?? 0).toDouble(),
      odomY: (json['odom_y'] ?? 0).toDouble(),
      odomYaw: (json['odom_yaw'] ?? 0).toDouble(),
      connected: true,
      picoState: (json['pico_state'] ?? '').toString(),
      leftTicks: (json['left_ticks'] ?? 0).toInt(),
      rightTicks: (json['right_ticks'] ?? 0).toInt(),
      headingDeg: (json['heading_deg'] ?? 0).toDouble(),
    );
  }
}

/// HTTP service for communicating with the RPi5 ROS2 bridge.
///
/// Sends joystick commands as HTTP POST to /api/control and polls
/// /api/telemetry for Pico motor/encoder data routed through the RPi5.
/// Includes a safety watchdog: the RPi5 bridge auto-zeros cmd_vel
/// if no command arrives within 500ms.
class PicoUdpService {
  static const Duration _sendTimeout = Duration(milliseconds: 1000);
  static const Duration _telemetryTimeout = Duration(milliseconds: 1500);
  static const Duration _healthTimeout = Duration(seconds: 3);

  String? _baseUrl;
  Timer? _sendTimer;
  Timer? _telemetryTimer;
  Timer? _healthTimer;

  double _lastX = 0;
  double _lastY = 0;
  double _speedLimit = 1.0;
  bool _emergencyStop = false;
  bool _isConnected = false;
  int _consecutiveFailures = 0;

  /// Current telemetry from Pico (via RPi5).
  PicoTelemetry telemetry = const PicoTelemetry();

  /// Callback when telemetry is updated.
  void Function(PicoTelemetry)? onTelemetryUpdate;

  /// Callback when connection status changes.
  void Function(bool connected)? onConnectionChanged;

  bool get isConnected => _isConnected;
  double get speedLimit => _speedLimit;

  /// Connect to the RPi5 bridge at the given URL.
  /// [url] should be like "http://192.168.1.5:5000"
  Future<bool> connect(String url) async {
    try {
      await disconnect();

      // Normalize URL
      var normalized = url.trim();
      if (!normalized.startsWith('http')) {
        normalized = 'http://$normalized';
      }
      normalized = normalized.replaceAll(RegExp(r'/*$'), '');
      _baseUrl = normalized;

      // Health check: prefer /api/status, fall back to /api/telemetry.
      bool healthy = false;
      try {
        final resp = await http.get(
          Uri.parse('$_baseUrl/api/status'),
        ).timeout(_healthTimeout);
        healthy = resp.statusCode == 200;
      } catch (_) {}

      if (!healthy) {
        try {
          final resp = await http.get(
            Uri.parse('$_baseUrl/api/telemetry'),
          ).timeout(_telemetryTimeout);
          healthy = resp.statusCode == 200;
        } catch (_) {}
      }

      if (!healthy) {
        _baseUrl = null;
        return false;
      }

      // Start periodic command sending at 20Hz (50ms)
      _sendTimer = Timer.periodic(const Duration(milliseconds: 50), (_) {
        _sendCommand();
      });

      // Poll telemetry at 5Hz (200ms)
      _telemetryTimer = Timer.periodic(const Duration(milliseconds: 200), (_) {
        _fetchTelemetry();
      });

      // Health check every 2s
      _healthTimer = Timer.periodic(const Duration(seconds: 2), (_) {
        _checkHealth();
      });

      _markHealthy();
      return true;
    } catch (e) {
      _baseUrl = null;
      _isConnected = false;
      onConnectionChanged?.call(false);
      return false;
    }
  }

  /// Disconnect and clean up resources.
  Future<void> disconnect() async {
    _sendTimer?.cancel();
    _telemetryTimer?.cancel();
    _healthTimer?.cancel();
    _sendTimer = null;
    _telemetryTimer = null;
    _healthTimer = null;

    // Send stop command before disconnecting
    if (_baseUrl != null) {
      _emergencyStop = true;
      _sendCommand();
      await Future.delayed(const Duration(milliseconds: 100));
    }

    _baseUrl = null;
    _isConnected = false;
    _emergencyStop = false;
    _lastX = 0;
    _lastY = 0;
    _consecutiveFailures = 0;
    telemetry = const PicoTelemetry();
    onConnectionChanged?.call(false);
  }

  /// Update joystick values. Called by the joystick widget.
  void updateJoystick(double x, double y) {
    _lastX = x;
    _lastY = y;
  }

  /// Set speed limiter (0.0 to 1.0).
  void setSpeedLimit(double limit) {
    _speedLimit = limit.clamp(0.0, 1.0);
  }

  /// Trigger emergency stop.
  void emergencyStop() {
    _emergencyStop = true;
    _lastX = 0;
    _lastY = 0;
    _sendCommand();
    // Also hit the dedicated estop endpoint
    if (_baseUrl != null) {
      http.post(Uri.parse('$_baseUrl/api/estop'))
          .timeout(const Duration(seconds: 1))
          .catchError((_) => http.Response('', 500));
    }
  }

  /// Release emergency stop.
  void releaseEmergencyStop() {
    _emergencyStop = false;
  }

  void _sendCommand() {
    if (_baseUrl == null) return;

    final data = jsonEncode({
      'x': double.parse((_emergencyStop ? 0.0 : _lastX).toStringAsFixed(3)),
      'y': double.parse((_emergencyStop ? 0.0 : _lastY).toStringAsFixed(3)),
      'e': _emergencyStop ? 1 : 0,
      'speed_limit': double.parse(_speedLimit.toStringAsFixed(2)),
    });

    http.post(
      Uri.parse('$_baseUrl/api/control'),
      headers: {'Content-Type': 'application/json'},
      body: data,
    ).timeout(_sendTimeout).catchError((_) {
      return http.Response('', 500);
    });
  }

  void _fetchTelemetry() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(
        Uri.parse('$_baseUrl/api/telemetry'),
      ).timeout(_telemetryTimeout);

      if (resp.statusCode == 200) {
        final json = jsonDecode(resp.body) as Map<String, dynamic>;
        telemetry = PicoTelemetry.fromJson(json);
        _markHealthy();
        onTelemetryUpdate?.call(telemetry);
      }
    } catch (_) {}
  }

  void _checkHealth() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(
        Uri.parse('$_baseUrl/api/status'),
      ).timeout(_healthTimeout);

      if (resp.statusCode == 200) {
        _markHealthy();
      } else {
        _onHealthFailure();
      }
    } catch (_) {
      _onHealthFailure();
    }
  }

  void _onHealthFailure() {
    _consecutiveFailures++;
    if (_consecutiveFailures >= 5 && _isConnected) {
      _isConnected = false;
      telemetry = const PicoTelemetry(connected: false);
      onConnectionChanged?.call(false);
      onTelemetryUpdate?.call(telemetry);
    }
  }

  void _markHealthy() {
    _consecutiveFailures = 0;
    if (!_isConnected) {
      _isConnected = true;
      onConnectionChanged?.call(true);
    }
  }
}
