import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as ws_status;

import '../models/telemetry.dart';
import '../models/robot_state.dart';
import '../services/event_logger.dart';

enum ConnectionStatus { disconnected, connecting, connected, error }

/// Top-level control mode mirrored from the bridge state machine.
enum ControlMode { auto, manual, estop }

ControlMode _modeFromString(String? s) {
  switch ((s ?? '').toUpperCase()) {
    case 'AUTO':
      return ControlMode.auto;
    case 'ESTOP':
      return ControlMode.estop;
    case 'MANUAL':
    default:
      return ControlMode.manual;
  }
}

String _modeToString(ControlMode m) {
  switch (m) {
    case ControlMode.auto:
      return 'AUTO';
    case ControlMode.estop:
      return 'ESTOP';
    case ControlMode.manual:
      return 'MANUAL';
  }
}

/// Manages HTTP connection to the RPi5 ROS2 Flask bridge.
/// Polls for status/telemetry, sends control commands, and tracks latency.
class ConnectionService extends ChangeNotifier {
  final EventLogger logger;

  ConnectionService({required this.logger});

  String? _baseUrl;
  ConnectionStatus _status = ConnectionStatus.disconnected;
  Timer? _statusTimer;
  Timer? _controlTimer;
  Timer? _healthTimer;
  Timer? _mapTimer;

  // WebSockets — joystick out, telemetry in. We keep them out of the
  // public surface and reconnect with backoff if they drop.
  WebSocketChannel? _ctrlChannel;
  WebSocketChannel? _telemetryChannel;
  StreamSubscription? _telemetrySub;
  Timer? _wsReconnectTimer;
  bool _wsClosing = false;
  int _wsReconnectAttempts = 0;

  // Current data
  RobotStatus _robotStatus = const RobotStatus();
  PicoTelemetry _telemetry = const PicoTelemetry();
  Uint8List? _mapImage;
  int _mapVersion = -1;            // Last fetched server version (-1 = none)
  List<List<double>> _scanPoints = [];

  // Nav2 visualization
  List<List<double>> _plannedPath = [];
  NavGoal _currentGoal = const NavGoal();
  Timer? _navTimer;

  // Control mode + Nav2 action status (mirrored from the bridge)
  ControlMode _mode = ControlMode.manual;
  String _navStatus = 'IDLE';
  double _navStatusStamp = 0;

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
  List<List<double>> get plannedPath => _plannedPath;
  NavGoal get currentGoal => _currentGoal;
  ControlMode get mode => _mode;
  String get navStatus => _navStatus;
  double get navStatusStamp => _navStatusStamp;
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
      _openSockets();
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
    _closeSockets();

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
    _mapVersion = -1;
    _scanPoints = [];
    _pathTrail.clear();
    _plannedPath = [];
    _currentGoal = const NavGoal();
    _mode = ControlMode.manual;
    _navStatus = 'IDLE';
    _navStatusStamp = 0;
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
    // Status poll 500ms — keeps RobotStatus / scan-info / map-info / markers
    // fresh. Pose + telemetry now arrive on the telemetry WebSocket so we
    // don't poll those over HTTP anymore.
    _statusTimer = Timer.periodic(const Duration(milliseconds: 500), (_) => _fetchStatus());
    // Joystick send 50ms (20 Hz). Each tick sends a JSON frame on the
    // /ws/control WebSocket — same payload as the old /api/control POST,
    // just orders of magnitude cheaper (no TCP+HTTP overhead per frame).
    _controlTimer = Timer.periodic(const Duration(milliseconds: 50), (_) => _sendControl());
    // Health check 2s
    _healthTimer = Timer.periodic(const Duration(seconds: 2), (_) => _checkHealth());
    // Map image: ETag-style poll every 2s — only refetches PNG when version
    // changes (saves ~15 KB / fetch when the map is stable).
    _mapTimer = Timer.periodic(const Duration(seconds: 2), (_) => _fetchMap());
    // Nav2 plan + goal: 2 Hz so the planner overlay updates smoothly during
    // replanning without dominating bandwidth (~3 KB/s combined).
    _navTimer = Timer.periodic(const Duration(milliseconds: 500), (_) {
      _fetchPlan();
      _fetchGoal();
    });
  }

  void _stopPolling() {
    _statusTimer?.cancel();
    _controlTimer?.cancel();
    _healthTimer?.cancel();
    _mapTimer?.cancel();
    _navTimer?.cancel();
    _statusTimer = null;
    _controlTimer = null;
    _healthTimer = null;
    _mapTimer = null;
    _navTimer = null;
  }

  // --- WebSockets ---

  String? _wsBase() {
    if (_baseUrl == null) return null;
    // Convert http://host:5000 -> ws://host:5000 (and https -> wss).
    return _baseUrl!.replaceFirst(RegExp(r'^http'), 'ws');
  }

  void _openSockets() {
    _wsClosing = false;
    _wsReconnectAttempts = 0;
    _connectControlSocket();
    _connectTelemetrySocket();
  }

  void _closeSockets() {
    _wsClosing = true;
    _wsReconnectTimer?.cancel();
    _wsReconnectTimer = null;
    _telemetrySub?.cancel();
    _telemetrySub = null;
    try { _ctrlChannel?.sink.close(ws_status.normalClosure); } catch (_) {}
    try { _telemetryChannel?.sink.close(ws_status.normalClosure); } catch (_) {}
    _ctrlChannel = null;
    _telemetryChannel = null;
  }

  void _connectControlSocket() {
    final base = _wsBase();
    if (base == null) return;
    try {
      _ctrlChannel = WebSocketChannel.connect(Uri.parse('$base/ws/control'));
      // We don't expect inbound messages, but listening with cancelOnError
      // gives us a way to notice the socket dying without a foreground send.
      _ctrlChannel!.stream.listen(
        (_) {},
        onError: (_) => _scheduleSocketReconnect(),
        onDone: () => _scheduleSocketReconnect(),
        cancelOnError: true,
      );
      logger.info('WS /ws/control connected', LogCategory.connection);
    } catch (e) {
      logger.error('WS /ws/control failed: $e', LogCategory.connection);
      _scheduleSocketReconnect();
    }
  }

  void _connectTelemetrySocket() {
    final base = _wsBase();
    if (base == null) return;
    try {
      _telemetryChannel = WebSocketChannel.connect(Uri.parse('$base/ws/telemetry'));
      _telemetrySub = _telemetryChannel!.stream.listen(
        _onTelemetryFrame,
        onError: (_) => _scheduleSocketReconnect(),
        onDone: () => _scheduleSocketReconnect(),
        cancelOnError: true,
      );
      logger.info('WS /ws/telemetry connected', LogCategory.connection);
    } catch (e) {
      logger.error('WS /ws/telemetry failed: $e', LogCategory.connection);
      _scheduleSocketReconnect();
    }
  }

  void _scheduleSocketReconnect() {
    if (_wsClosing || _baseUrl == null) return;
    if (_wsReconnectTimer != null) return;
    _wsReconnectAttempts = (_wsReconnectAttempts + 1).clamp(1, 6);
    // Exponential backoff capped at 5 s — typical Wi-Fi blip recovers in 1–2.
    final delayMs = (250 * (1 << (_wsReconnectAttempts - 1))).clamp(250, 5000);
    _wsReconnectTimer = Timer(Duration(milliseconds: delayMs), () {
      _wsReconnectTimer = null;
      // Tear down both sockets and rebuild — simpler than tracking which
      // half failed.
      try { _telemetrySub?.cancel(); } catch (_) {}
      _telemetrySub = null;
      try { _ctrlChannel?.sink.close(); } catch (_) {}
      try { _telemetryChannel?.sink.close(); } catch (_) {}
      _ctrlChannel = null;
      _telemetryChannel = null;
      if (!_wsClosing) {
        _connectControlSocket();
        _connectTelemetrySocket();
      }
    });
  }

  void _onTelemetryFrame(dynamic raw) {
    try {
      final json = jsonDecode(raw as String) as Map<String, dynamic>;
      // Pose
      if (json['pose'] is Map<String, dynamic>) {
        final pose = RobotPose.fromJson(json['pose']);
        // Update only the pose half of RobotStatus so other fields (mapInfo,
        // markers, scan-info) keep their last REST-fetched values.
        _robotStatus = RobotStatus(
          pose: pose,
          scan: _robotStatus.scan,
          mapInfo: _robotStatus.mapInfo,
          markers: _robotStatus.markers,
        );
        // Trail
        if (pose.x != 0 || pose.y != 0) {
          if (_pathTrail.isEmpty ||
              (_pathTrail.last[0] - pose.x).abs() > 0.01 ||
              (_pathTrail.last[1] - pose.y).abs() > 0.01) {
            _pathTrail.add([pose.x, pose.y]);
            if (_pathTrail.length > _maxTrailPoints) _pathTrail.removeAt(0);
          }
        }
      }
      if (json['telemetry'] is Map<String, dynamic>) {
        _telemetry = PicoTelemetry.fromJson(json['telemetry']);
      }
      if (json['mode'] is String) {
        _mode = _modeFromString(json['mode'] as String);
      }
      if (json['nav_status'] is String) {
        _navStatus = json['nav_status'] as String;
        _navStatusStamp = (json['nav_status_stamp'] ?? 0).toDouble();
      }
      if (json['goal'] is Map<String, dynamic>) {
        _currentGoal = NavGoal.fromJson(json['goal']);
      }
      _wsReconnectAttempts = 0; // success — reset backoff
      notifyListeners();
    } catch (_) {/* ignore malformed frame */}
  }

  // --- Control mode ---

  /// Request a control mode transition on the bridge.
  Future<bool> setMode(ControlMode mode) async {
    if (_baseUrl == null) return false;
    // Optimistic local update so the UI reacts instantly.
    _mode = mode;
    if (mode == ControlMode.estop) {
      _emergencyStopped = true;
      _currentGoal = const NavGoal();
      _plannedPath = [];
    } else {
      _emergencyStopped = false;
    }
    notifyListeners();
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/mode'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'mode': _modeToString(mode)}),
      ).timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) {
        logger.error('Mode change failed: HTTP ${resp.statusCode}', LogCategory.control);
        return false;
      }
      logger.success('Mode -> ${_modeToString(mode)}', LogCategory.control);
      return true;
    } catch (e) {
      logger.error('Mode change failed: $e', LogCategory.control);
      return false;
    }
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
    // Optimistically clear the goal locally so the map overlay updates
    // immediately even before the bridge round-trip completes.
    _currentGoal = const NavGoal();
    _plannedPath = [];
    notifyListeners();
    logger.warn('E-STOP engaged', LogCategory.control);

    if (_baseUrl != null) {
      // /api/estop already cancels Nav2 on the bridge (since 2026-05-07),
      // but we also POST /api/cancel_nav as defense-in-depth in case the
      // app is talking to an older bridge.
      http.post(Uri.parse('$_baseUrl/api/estop'))
          .timeout(const Duration(seconds: 1))
          .catchError((_) => http.Response('', 500));
      http.post(Uri.parse('$_baseUrl/api/cancel_nav'))
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
        // Seed locally so the marker appears before the next /api/goal poll.
        _currentGoal = NavGoal(x: x, y: y, yaw: yaw, active: true,
            stamp: DateTime.now().millisecondsSinceEpoch / 1000.0);
        notifyListeners();
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

  /// Cancel the current Nav2 goal at the next instant. Optimistically
  /// clears the local plan/goal so the map overlay updates immediately.
  Future<bool> cancelNavGoal() async {
    if (_baseUrl == null) return false;
    _currentGoal = const NavGoal();
    _plannedPath = [];
    notifyListeners();
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
    final ch = _ctrlChannel;
    if (ch == null) return;
    _commandsSent++;

    final data = jsonEncode({
      'x': double.parse((_emergencyStopped ? 0.0 : _joystickX).toStringAsFixed(3)),
      'y': double.parse((_emergencyStopped ? 0.0 : _joystickY).toStringAsFixed(3)),
      'e': _emergencyStopped ? 1 : 0,
      'speed_limit': double.parse(_speedLimit.toStringAsFixed(2)),
    });
    try {
      ch.sink.add(data);
    } catch (_) {
      // Sink is dead — let the stream listener trigger reconnect.
    }
  }

  void _fetchStatus() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/status'))
          .timeout(const Duration(seconds: 1));
      if (resp.statusCode == 200) {
        final json = jsonDecode(resp.body) as Map<String, dynamic>;
        // Pose is already coming over the telemetry WS at 10 Hz; here we only
        // refresh the slower-moving fields (mapInfo, markers, scan-info).
        final fresh = RobotStatus.fromJson(json);
        _robotStatus = RobotStatus(
          pose: _robotStatus.pose,
          scan: fresh.scan,
          mapInfo: fresh.mapInfo,
          markers: fresh.markers,
        );
        notifyListeners();
      }
    } catch (_) {}
  }

  void _fetchMap() async {
    if (_baseUrl == null) return;
    try {
      // Cheap version probe (~30 B). Skip the PNG fetch if unchanged.
      final vResp = await http.get(Uri.parse('$_baseUrl/api/map_version'))
          .timeout(const Duration(seconds: 1));
      if (vResp.statusCode == 200) {
        final v = (jsonDecode(vResp.body)['v'] ?? 0) as int;
        if (v == _mapVersion && _mapImage != null) return; // up-to-date
        final resp = await http.get(Uri.parse('$_baseUrl/api/map'))
            .timeout(const Duration(seconds: 3));
        if (resp.statusCode == 200 && resp.bodyBytes.isNotEmpty) {
          _mapImage = resp.bodyBytes;
          _mapVersion = v;
          notifyListeners();
        }
      }
    } catch (_) {}
  }

  Future<void> _fetchPlan() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/plan'))
          .timeout(const Duration(seconds: 1));
      if (resp.statusCode == 200) {
        final json = jsonDecode(resp.body) as Map<String, dynamic>;
        final pts = json['points'] as List? ?? [];
        _plannedPath = pts.map<List<double>>((p) {
          final list = p as List;
          return [list[0].toDouble(), list[1].toDouble()];
        }).toList();
        notifyListeners();
      }
    } catch (_) {}
  }

  Future<void> _fetchGoal() async {
    if (_baseUrl == null) return;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/goal'))
          .timeout(const Duration(seconds: 1));
      if (resp.statusCode == 200) {
        final json = jsonDecode(resp.body) as Map<String, dynamic>;
        _currentGoal = NavGoal.fromJson(json);
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
    _closeSockets();
    super.dispose();
  }
}
