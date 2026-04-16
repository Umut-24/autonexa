import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;

import '../models/telemetry.dart';
import '../models/robot_state.dart';
import '../services/event_logger.dart';

enum ConnectionStatus { disconnected, connecting, connected, error }

/// Manages HTTP connection to the RPi5 ROS2 Flask bridge.
/// Polls for status/telemetry, sends control commands, and tracks latency.
class ConnectionService extends ChangeNotifier {
  final EventLogger logger;

  ConnectionService({required this.logger});

  String? _baseUrl;
  ConnectionStatus _status = ConnectionStatus.disconnected;
  Timer? _statusTimer;
  Timer? _telemetryTimer;
  Timer? _controlTimer;
  Timer? _healthTimer;
  Timer? _mapTimer;

  // Current data
  RobotStatus _robotStatus = const RobotStatus();
  PicoTelemetry _telemetry = const PicoTelemetry();
  Uint8List? _mapImage;
  List<List<double>> _scanPoints = [];

  // Path trail (history of robot positions for map overlay)
  final List<List<double>> _pathTrail = [];
  static const int _maxTrailPoints = 500;

  // Control state
  double _joystickX = 0;
  double _joystickY = 0;
  double _speedLimit = 0.5;
  bool _emergencyStopped = false;

  // Network stats
  int _latencyMs = 0;
  int _consecutiveFailures = 0;
  DateTime? _connectedSince;
  int _commandsSent = 0;

  // Getters
  String? get baseUrl => _baseUrl;
  ConnectionStatus get status => _status;
  RobotStatus get robotStatus => _robotStatus;
  PicoTelemetry get telemetry => _telemetry;
  Uint8List? get mapImage => _mapImage;
  List<List<double>> get scanPoints => _scanPoints;
  List<List<double>> get pathTrail => _pathTrail;
  bool get emergencyStopped => _emergencyStopped;
  double get speedLimit => _speedLimit;
  int get latencyMs => _latencyMs;
  DateTime? get connectedSince => _connectedSince;
  int get commandsSent => _commandsSent;
  bool get isConnected => _status == ConnectionStatus.connected;
  String get videoFeedUrl => _baseUrl != null ? '$_baseUrl/video_feed' : '';

  /// Connect to the RPi5 bridge.
  Future<bool> connect(String url) async {
    _status = ConnectionStatus.connecting;
    notifyListeners();

    var normalized = url.trim();
    if (!normalized.startsWith('http')) normalized = 'http://$normalized';
    normalized = normalized.replaceAll(RegExp(r'/*$'), '');

    try {
      final sw = Stopwatch()..start();
      final resp = await http.get(Uri.parse('$normalized/api/status'))
          .timeout(const Duration(seconds: 3));
      sw.stop();

      if (resp.statusCode != 200) {
        _status = ConnectionStatus.error;
        notifyListeners();
        logger.error('Connection failed: HTTP ${resp.statusCode}', LogCategory.connection);
        return false;
      }

      _baseUrl = normalized;
      _latencyMs = sw.elapsedMilliseconds;
      _consecutiveFailures = 0;
      _connectedSince = DateTime.now();
      _commandsSent = 0;
      _status = ConnectionStatus.connected;

      _startPolling();
      notifyListeners();
      logger.success('Connected to $normalized (${_latencyMs}ms)', LogCategory.connection);
      return true;
    } catch (e) {
      _status = ConnectionStatus.error;
      notifyListeners();
      logger.error('Connection failed: $e', LogCategory.connection);
      return false;
    }
  }

  /// Disconnect and clean up.
  Future<void> disconnect() async {
    _stopPolling();

    if (_baseUrl != null) {
      // Send stop command before disconnecting
      try {
        await http.post(
          Uri.parse('$_baseUrl/api/control'),
          headers: {'Content-Type': 'application/json'},
          body: jsonEncode({'x': 0, 'y': 0, 'e': 1, 'speed_limit': 0}),
        ).timeout(const Duration(milliseconds: 300));
      } catch (_) {}
    }

    final wasConnected = _status == ConnectionStatus.connected;
    _baseUrl = null;
    _status = ConnectionStatus.disconnected;
    _robotStatus = const RobotStatus();
    _telemetry = const PicoTelemetry();
    _mapImage = null;
    _scanPoints = [];
    _pathTrail.clear();
    _joystickX = 0;
    _joystickY = 0;
    _emergencyStopped = false;
    _consecutiveFailures = 0;
    _connectedSince = null;
    notifyListeners();

    if (wasConnected) {
      logger.info('Disconnected', LogCategory.connection);
    }
  }

  void _startPolling() {
    // Status poll 500ms
    _statusTimer = Timer.periodic(const Duration(milliseconds: 500), (_) => _fetchStatus());
    // Telemetry poll 200ms
    _telemetryTimer = Timer.periodic(const Duration(milliseconds: 200), (_) => _fetchTelemetry());
    // Control send 50ms (20Hz)
    _controlTimer = Timer.periodic(const Duration(milliseconds: 50), (_) => _sendControl());
    // Health check 2s
    _healthTimer = Timer.periodic(const Duration(seconds: 2), (_) => _checkHealth());
    // Map image 2s
    _mapTimer = Timer.periodic(const Duration(seconds: 2), (_) => _fetchMap());
  }

  void _stopPolling() {
    _statusTimer?.cancel();
    _telemetryTimer?.cancel();
    _controlTimer?.cancel();
    _healthTimer?.cancel();
    _mapTimer?.cancel();
    _statusTimer = null;
    _telemetryTimer = null;
    _controlTimer = null;
    _healthTimer = null;
    _mapTimer = null;
  }

  // --- Control ---

  void updateJoystick(double x, double y) {
    _joystickX = x;
    _joystickY = y;
  }

  void setSpeedLimit(double limit) {
    _speedLimit = limit.clamp(0.0, 1.0);
    notifyListeners();
  }

  void emergencyStop() {
    _emergencyStopped = true;
    _joystickX = 0;
    _joystickY = 0;
    notifyListeners();
    logger.warn('E-STOP engaged', LogCategory.control);

    // Immediate estop POST
    if (_baseUrl != null) {
      http.post(Uri.parse('$_baseUrl/api/estop'))
          .timeout(const Duration(seconds: 1))
          .catchError((_) => http.Response('', 500));
    }
  }

  void releaseEmergencyStop() {
    _emergencyStopped = false;
    notifyListeners();
    logger.success('E-STOP released', LogCategory.control);

    // Clear the Pico hardware latch
    if (_baseUrl != null) {
      http.post(Uri.parse('$_baseUrl/api/estop_clear'))
          .timeout(const Duration(seconds: 1))
          .catchError((_) => http.Response('', 500));
    }
  }

  Future<bool> sendNavGoal(double x, double y, double yaw) async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/nav_goal'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'x': x, 'y': y, 'yaw': yaw}),
      ).timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) {
        logger.success('Nav goal sent: (${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)}, yaw=${yaw.toStringAsFixed(2)})', LogCategory.navigation);
      } else {
        logger.error('Nav goal failed: HTTP ${resp.statusCode}', LogCategory.navigation);
      }
      return ok;
    } catch (e) {
      logger.error('Nav goal failed: $e', LogCategory.navigation);
      return false;
    }
  }

  /// Clear the recorded path trail.
  void clearPathTrail() {
    _pathTrail.clear();
    notifyListeners();
  }

  /// Cancel the current Nav2 goal.
  Future<bool> cancelNavGoal() async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/cancel_nav'),
      ).timeout(const Duration(seconds: 2));
      final ok = resp.statusCode == 200;
      if (ok) {
        logger.info('Navigation cancelled', LogCategory.navigation);
      }
      return ok;
    } catch (e) {
      logger.error('Cancel nav failed: $e', LogCategory.navigation);
      return false;
    }
  }

  // --- Polling callbacks ---

  void _sendControl() {
    if (_baseUrl == null) return;
    _commandsSent++;

    final data = jsonEncode({
      'x': double.parse((_emergencyStopped ? 0.0 : _joystickX).toStringAsFixed(3)),
      'y': double.parse((_emergencyStopped ? 0.0 : _joystickY).toStringAsFixed(3)),
      'e': _emergencyStopped ? 1 : 0,
      'speed_limit': double.parse(_speedLimit.toStringAsFixed(2)),
    });

    http.post(
      Uri.parse('$_baseUrl/api/control'),
      headers: {'Content-Type': 'application/json'},
      body: data,
    ).timeout(const Duration(milliseconds: 200)).catchError((_) => http.Response('', 500));
  }

  void _fetchStatus() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/status'))
          .timeout(const Duration(seconds: 1));
      if (resp.statusCode == 200) {
        final json = jsonDecode(resp.body) as Map<String, dynamic>;
        _robotStatus = RobotStatus.fromJson(json);
        // Record path trail
        final pose = _robotStatus.pose;
        if (pose.x != 0 || pose.y != 0) {
          if (_pathTrail.isEmpty ||
              (_pathTrail.last[0] - pose.x).abs() > 0.01 ||
              (_pathTrail.last[1] - pose.y).abs() > 0.01) {
            _pathTrail.add([pose.x, pose.y]);
            if (_pathTrail.length > _maxTrailPoints) _pathTrail.removeAt(0);
          }
        }
        notifyListeners();
      }
    } catch (_) {}
  }

  void _fetchTelemetry() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/telemetry'))
          .timeout(const Duration(milliseconds: 500));
      if (resp.statusCode == 200) {
        final json = jsonDecode(resp.body) as Map<String, dynamic>;
        _telemetry = PicoTelemetry.fromJson(json);
        notifyListeners();
      }
    } catch (_) {}
  }

  void _fetchMap() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/map'))
          .timeout(const Duration(seconds: 3));
      if (resp.statusCode == 200 && resp.bodyBytes.isNotEmpty) {
        _mapImage = resp.bodyBytes;
        notifyListeners();
      }
    } catch (_) {}
  }

  void _checkHealth() async {
    if (_baseUrl == null) return;
    try {
      final sw = Stopwatch()..start();
      final resp = await http.get(Uri.parse('$_baseUrl/api/status'))
          .timeout(const Duration(seconds: 2));
      sw.stop();

      if (resp.statusCode == 200) {
        _latencyMs = sw.elapsedMilliseconds;
        _consecutiveFailures = 0;
        if (_status != ConnectionStatus.connected) {
          _status = ConnectionStatus.connected;
          logger.success('Connection restored', LogCategory.connection);
        }
        notifyListeners();
      } else {
        _onHealthFailure();
      }
    } catch (_) {
      _onHealthFailure();
    }
  }

  void _onHealthFailure() {
    _consecutiveFailures++;
    if (_consecutiveFailures >= 3 && _status == ConnectionStatus.connected) {
      _status = ConnectionStatus.error;
      _telemetry = const PicoTelemetry();
      notifyListeners();
      logger.error('Connection lost (3 consecutive failures)', LogCategory.connection);
    }
  }

  /// Fetch scan points on demand (not polled automatically due to size).
  Future<void> fetchScan() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/scan'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode == 200) {
        final json = jsonDecode(resp.body) as Map<String, dynamic>;
        final points = json['points'] as List? ?? [];
        _scanPoints = points.map<List<double>>((p) {
          final list = p as List;
          return [list[0].toDouble(), list[1].toDouble()];
        }).toList();
        notifyListeners();
      }
    } catch (_) {}
  }

  @override
  void dispose() {
    _stopPolling();
    super.dispose();
  }
}
