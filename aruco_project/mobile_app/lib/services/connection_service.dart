import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:http/http.dart' as http;
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as ws_status;

import '../models/telemetry.dart';
import '../models/robot_state.dart';
import '../models/robot_config.dart';
import '../services/event_logger.dart';

enum ConnectionStatus { disconnected, connecting, connected, error }

/// Top-level control mode mirrored from the bridge state machine.
enum ControlMode { auto, manual, estop }

/// Safety chain mode for MANUAL drive.
/// soft = goes through velocity_smoother + collision_monitor (default).
/// off  = bypasses safety chain — joystick reaches the wheels directly.
enum SafetyMode { soft, off }

/// One per-param result from /api/params POST. Carries both the boolean
/// success and the bridge's `reason` string so the UI can show *why* a
/// SetParameters call was rejected (e.g. "parameter not declared",
/// "read-only", "out of range") instead of a generic failure message.
class ParamSetResult {
  final bool ok;
  final String reason;
  const ParamSetResult({required this.ok, this.reason = ''});
}

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

SafetyMode _safetyFromString(String? s) {
  return (s ?? '').toLowerCase() == 'off' ? SafetyMode.off : SafetyMode.soft;
}

String _safetyToString(SafetyMode m) =>
    m == SafetyMode.off ? 'off' : 'soft';

/// A user-defined point on the SLAM map (parking spot, summon point, etc.).
/// `mapFingerprint` lets the app warn the user if the underlying map has
/// been replaced by a SLAM restart since the waypoint was saved.
class NamedWaypoint {
  final String name;
  final String kind; // park | summon | home | custom
  final double x;
  final double y;
  final double yaw;
  final String mapFingerprint;
  final bool stale;

  const NamedWaypoint({
    required this.name,
    required this.kind,
    required this.x,
    required this.y,
    required this.yaw,
    this.mapFingerprint = '',
    this.stale = false,
  });

  factory NamedWaypoint.fromJson(Map<String, dynamic> j) {
    final pose = (j['pose'] as Map?) ?? {};
    return NamedWaypoint(
      name: (j['name'] ?? '').toString(),
      kind: (j['kind'] ?? 'custom').toString(),
      x: (pose['x'] ?? 0).toDouble(),
      y: (pose['y'] ?? 0).toDouble(),
      yaw: (pose['yaw'] ?? 0).toDouble(),
      mapFingerprint: (j['map_fingerprint'] ?? '').toString(),
      stale: j['stale'] == true,
    );
  }
}

/// One row of the topic-health panel.
class HealthRow {
  final String topic;
  final String label;
  final double expectedHz;
  final double rateHz;
  final double? lastAgeS;
  final bool ok;
  const HealthRow({
    required this.topic,
    required this.label,
    required this.expectedHz,
    required this.rateHz,
    required this.lastAgeS,
    required this.ok,
  });
  factory HealthRow.fromJson(Map<String, dynamic> j) => HealthRow(
        topic: (j['topic'] ?? '').toString(),
        label: (j['label'] ?? '').toString(),
        expectedHz: (j['expected_hz'] ?? 0).toDouble(),
        rateHz: (j['rate_hz'] ?? 0).toDouble(),
        lastAgeS: j['last_age_s'] == null ? null : (j['last_age_s'] as num).toDouble(),
        ok: j['ok'] == true,
      );
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

  // Manual waypoints — kept in-memory and refreshed periodically so the map
  // overlay and Home-tab Summon button can read them synchronously.
  List<NamedWaypoint> _namedWaypoints = [];
  Timer? _waypointTimer;

  // Control mode + Nav2 action status (mirrored from the bridge)
  ControlMode _mode = ControlMode.manual;
  SafetyMode _safetyMode = SafetyMode.soft;
  String _navStatus = 'IDLE';
  double _navStatusStamp = 0;
  String _mapFingerprint = '';

  // Robot physical dimensions, refreshed from /api/robot_config on connect.
  // Used by the map overlay (chassis footprint + LiDAR marker) and the
  // Settings -> Robot Dimensions editor.
  RobotConfig _robotConfig = RobotConfig.defaults;

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
  SafetyMode get safetyMode => _safetyMode;
  List<NamedWaypoint> get namedWaypoints => List.unmodifiable(_namedWaypoints);
  String get navStatus => _navStatus;
  double get navStatusStamp => _navStatusStamp;
  String get mapFingerprint => _mapFingerprint;
  RobotConfig get robotConfig => _robotConfig;
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
      // Best-effort robot dimensions fetch; failure leaves the cached defaults.
      // ignore: unawaited_futures
      fetchRobotConfig();
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
    _namedWaypoints = [];
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
    // Waypoint refresh: 5 s. Slow-changing data (user-initiated) so 0.2 Hz
    // is plenty. Fetches happen on save/delete too, so this is just to pick
    // up changes made from another phone or directly on the bridge.
    _waypointTimer = Timer.periodic(const Duration(seconds: 5), (_) => _fetchWaypoints());
    _fetchWaypoints();
  }

  void _stopPolling() {
    _statusTimer?.cancel();
    _controlTimer?.cancel();
    _healthTimer?.cancel();
    _mapTimer?.cancel();
    _navTimer?.cancel();
    _waypointTimer?.cancel();
    _statusTimer = null;
    _controlTimer = null;
    _healthTimer = null;
    _mapTimer = null;
    _navTimer = null;
    _waypointTimer = null;
  }

  Future<void> _fetchWaypoints() async {
    if (_baseUrl == null) return;
    final wps = await _fetchWaypointsRaw();
    if (wps == null) return; // transient HTTP error — keep last-known list.
    _namedWaypoints = wps;
    notifyListeners();
  }

  /// Same call as `listNamedWaypoints` but distinguishes empty (legitimately
  /// no waypoints) from null (HTTP failure). Used by the poll loop.
  Future<List<NamedWaypoint>?> _fetchWaypointsRaw() async {
    if (_baseUrl == null) return null;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/waypoints'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return null;
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final list = (json['waypoints'] as List?) ?? [];
      return list.map<NamedWaypoint>((e) => NamedWaypoint.fromJson(e as Map<String, dynamic>)).toList();
    } catch (_) {
      return null;
    }
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
      if (json['safety_mode'] is String) {
        _safetyMode = _safetyFromString(json['safety_mode'] as String);
      }
      if (json['nav_status'] is String) {
        _navStatus = json['nav_status'] as String;
        _navStatusStamp = (json['nav_status_stamp'] ?? 0).toDouble();
      }
      if (json['map_fingerprint'] is String) {
        _mapFingerprint = json['map_fingerprint'] as String;
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
        _mode = ControlMode.auto;
        _emergencyStopped = false;
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

  // --- Safety mode (Part A) ---

  Future<bool> setSafetyMode(SafetyMode mode) async {
    if (_baseUrl == null) return false;
    _safetyMode = mode;
    notifyListeners();
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/safety_mode'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'safety_mode': _safetyToString(mode)}),
      ).timeout(const Duration(seconds: 2));
      final ok = resp.statusCode == 200;
      if (ok) {
        logger.warn('Safety -> ${_safetyToString(mode)}', LogCategory.control);
      }
      return ok;
    } catch (e) {
      logger.error('safety_mode failed: $e', LogCategory.control);
      return false;
    }
  }

  // --- Direction calibration (Part B) ---

  /// Apply polarity flip(s) to nav2_pico_bridge live + persist to disk.
  /// Returns map of {param: ok}.
  Future<Map<String, bool>> calibrateDirection(
      {int? vxPolarity, int? servoPolarity}) async {
    if (_baseUrl == null) return {};
    final body = <String, dynamic>{};
    if (vxPolarity != null) body['vx_polarity'] = vxPolarity;
    if (servoPolarity != null) body['servo_polarity'] = servoPolarity;
    if (body.isEmpty) return {};
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/calibrate_direction'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode(body),
      ).timeout(const Duration(seconds: 3));
      if (resp.statusCode != 200) {
        logger.error('calibrate_direction HTTP ${resp.statusCode}', LogCategory.control);
        return {};
      }
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final results = (json['results'] as Map?) ?? {};
      final out = <String, bool>{};
      results.forEach((k, v) {
        out[k.toString()] = (v as Map?)?['ok'] == true;
      });
      logger.success('Calibration applied: $out', LogCategory.control);
      return out;
    } catch (e) {
      logger.error('calibrate_direction: $e', LogCategory.control);
      return {};
    }
  }

  // --- Robot dimensions (Settings -> Robot Dimensions) ---

  /// Fetch live robot dimensions from the bridge. Updates the cached
  /// `robotConfig` getter on success.
  Future<RobotConfig?> fetchRobotConfig() async {
    if (_baseUrl == null) return null;
    try {
      final resp = await http
          .get(Uri.parse('$_baseUrl/api/robot_config'))
          .timeout(const Duration(seconds: 3));
      if (resp.statusCode != 200) return null;
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      _robotConfig = RobotConfig.fromJson(json);
      notifyListeners();
      return _robotConfig;
    } catch (e) {
      logger.error('fetchRobotConfig: $e', LogCategory.control);
      return null;
    }
  }

  /// Push a (partial) dimensions update to the bridge. Bridge regenerates
  /// the URDF, syncs both Nav2 costmap footprints, and persists to
  /// ~/.autonexa/robot_dimensions.yaml.
  Future<bool> setRobotConfig(Map<String, double> overrides) async {
    if (_baseUrl == null || overrides.isEmpty) return false;
    try {
      final resp = await http
          .post(
            Uri.parse('$_baseUrl/api/robot_config'),
            headers: {'Content-Type': 'application/json'},
            body: jsonEncode(overrides),
          )
          .timeout(const Duration(seconds: 4));
      if (resp.statusCode != 200) {
        logger.error('robot_config HTTP ${resp.statusCode}', LogCategory.control);
        return false;
      }
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      if (json['ok'] == false) {
        logger.error(
            'robot_config: ${json['reason'] ?? 'bridge rejected'}',
            LogCategory.control);
        return false;
      }
      _robotConfig = RobotConfig.fromJson(json);
      notifyListeners();
      logger.success(
          'Robot dimensions applied (${overrides.keys.join(', ')})',
          LogCategory.control);
      return true;
    } catch (e) {
      logger.error('setRobotConfig: $e', LogCategory.control);
      return false;
    }
  }

  /// Read current vx/servo polarities from the bridge.
  Future<Map<String, int>> getCalibration() async {
    if (_baseUrl == null) return {};
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/calibrate_direction'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return {};
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final out = <String, int>{};
      json.forEach((k, v) {
        if (v is num) out[k] = v.toInt();
      });
      return out;
    } catch (_) { return {}; }
  }

  /// Brief "drive forward" pulse used by the calibration wizard.
  /// Forces MANUAL mode + safety=soft, sends a small vx for `durationMs`,
  /// then E-stops. Returns false if not connected.
  ///
  /// Magnitude is tuned to break the L298N chassis's static friction:
  /// vx=0.20 m/s commands SPEED 20 to the firmware, which produces ~67%
  /// PWM on the L298N (above the 60% deadband kick). Lower values can
  /// stall on carpet / coarse floor — observed during round-1 use.
  Future<bool> calibrationPulse({
    double vx = 0.20,
    double wz = 0.0,
    int durationMs = 1200,
  }) async {
    if (_baseUrl == null) return false;
    if (_mode != ControlMode.manual) {
      await setMode(ControlMode.manual);
    }
    if (_safetyMode != SafetyMode.soft) {
      await setSafetyMode(SafetyMode.soft);
    }
    final stop = DateTime.now().add(Duration(milliseconds: durationMs));
    while (DateTime.now().isBefore(stop)) {
      try {
        await http.post(
          Uri.parse('$_baseUrl/api/control'),
          headers: {'Content-Type': 'application/json'},
          body: jsonEncode({
            // Joystick 'y' maps to vx (linear), 'x' to wz (angular). Normalize
            // to [-1,1] against bridge clamps (max_vx=0.35, max_wz=0.8).
            // Note: the bridge inverts 'x' before publishing to angular.z (so
            // joystick-right = right turn). Pre-invert here so a positive
            // wz argument (= ROS-positive = left turn) is honored.
            'y': (vx / 0.35).clamp(-1.0, 1.0),
            'x': (-wz / 0.8).clamp(-1.0, 1.0),
            'e': 0,
            'speed_limit': 1.0,
          }),
        ).timeout(const Duration(milliseconds: 200));
      } catch (_) {}
      // 80 ms send interval keeps the bridge's 200 ms watchdog comfortably
      // fed even with HTTP jitter; round-1 used 100 ms which was on the edge.
      await Future.delayed(const Duration(milliseconds: 80));
    }
    // Hard stop.
    try {
      await http.post(
        Uri.parse('$_baseUrl/api/control'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'x': 0, 'y': 0, 'e': 1, 'speed_limit': 0}),
      ).timeout(const Duration(milliseconds: 300));
    } catch (_) {}
    return true;
  }

  // --- Map / Nav2 reset (Part C) ---

  Future<bool> clearCostmaps() async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(Uri.parse('$_baseUrl/api/clear_costmaps'))
          .timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) logger.info('Costmaps cleared', LogCategory.navigation);
      return ok;
    } catch (e) {
      logger.error('clear_costmaps: $e', LogCategory.navigation);
      return false;
    }
  }

  Future<bool> restartMapping() async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(Uri.parse('$_baseUrl/api/restart_mapping'))
          .timeout(const Duration(seconds: 8));
      final ok = resp.statusCode == 200;
      if (ok) {
        // Map is gone — drop our local copy so the next /api/map_version probe
        // forces a fresh fetch.
        _mapImage = null;
        _mapVersion = -1;
        _pathTrail.clear();
        notifyListeners();
        logger.warn('SLAM restarted — map cleared', LogCategory.navigation);
      }
      return ok;
    } catch (e) {
      logger.error('restart_mapping: $e', LogCategory.navigation);
      return false;
    }
  }

  /// Persist the current SLAM map via the bridge's map_saver_cli
  /// (POST /api/lock_map). Returns the saved .yaml path on success, or null
  /// on failure. The map keeps mapping afterwards — this is a snapshot, not a
  /// lifecycle change.
  Future<String?> saveMap() async {
    if (_baseUrl == null) return null;
    try {
      // map_saver_cli writes the .pgm + .yaml synchronously; give it room.
      final resp = await http.post(Uri.parse('$_baseUrl/api/lock_map'))
          .timeout(const Duration(seconds: 35));
      if (resp.statusCode != 200) {
        logger.error('save_map: HTTP ${resp.statusCode} ${resp.body}',
            LogCategory.navigation);
        return null;
      }
      final body = jsonDecode(resp.body) as Map<String, dynamic>;
      final yaml = body['yaml'] as String?;
      logger.info('Map saved: $yaml', LogCategory.navigation);
      return yaml;
    } catch (e) {
      logger.error('save_map: $e', LogCategory.navigation);
      return null;
    }
  }

  // --- Nav2 max linear speed (Part E) ---

  Future<bool> setNav2MaxSpeed(double mps) async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/nav2_speed'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'max_vel_x': mps}),
      ).timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) logger.info('Nav2 desired_linear_vel -> ${mps.toStringAsFixed(2)} m/s', LogCategory.navigation);
      return ok;
    } catch (e) {
      logger.error('nav2_speed: $e', LogCategory.navigation);
      return false;
    }
  }

  Future<double?> getNav2MaxSpeed() async {
    if (_baseUrl == null) return null;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/nav2_speed'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return null;
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final v = json['controller_desired_linear_vel'] ?? json['controller_max_vel_x'];
      return v is num ? v.toDouble() : null;
    } catch (_) { return null; }
  }

  // --- Path planner mode (standard / multipoint) ---

  Future<String?> getPlannerMode() async {
    if (_baseUrl == null) return null;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/planner_mode'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return null;
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final m = json['planner_mode'];
      return m is String ? m : null;
    } catch (_) { return null; }
  }

  Future<bool> setPlannerMode(String mode) async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/planner_mode'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'planner_mode': mode}),
      ).timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) logger.info('Planner mode -> $mode', LogCategory.navigation);
      return ok;
    } catch (e) {
      logger.error('planner_mode: $e', LogCategory.navigation);
      return false;
    }
  }

  // --- Runtime overrides reset (calibration-only hygiene) ---

  /// Wipe ~/.autonexa/runtime_overrides.yaml so the PC config files become the
  /// single source of truth on the next relaunch. Clears stale phone-saved
  /// values (the "wrong direction / wrong speed after relaunch" foot-gun).
  Future<bool> resetOverrides() async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(Uri.parse('$_baseUrl/api/reset_overrides'))
          .timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) {
        logger.warn('runtime_overrides wiped — PC config authoritative on next relaunch',
            LogCategory.control);
      }
      return ok;
    } catch (e) {
      logger.error('reset_overrides: $e', LogCategory.control);
      return false;
    }
  }

  // --- Auto re-localize (AMCL periodic re-settle, localization mode) ---

  Future<Map<String, dynamic>?> getRelocalizeAuto() async {
    if (_baseUrl == null) return null;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/relocalize_auto'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return null;
      return jsonDecode(resp.body) as Map<String, dynamic>;
    } catch (_) { return null; }
  }

  Future<bool> setRelocalizeAuto({required bool enabled, double? intervalS}) async {
    if (_baseUrl == null) return false;
    try {
      final body = <String, dynamic>{'enabled': enabled};
      if (intervalS != null) body['interval_s'] = intervalS;
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/relocalize_auto'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode(body),
      ).timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) logger.info('Auto re-localize -> $enabled', LogCategory.navigation);
      return ok;
    } catch (e) {
      logger.error('relocalize_auto: $e', LogCategory.navigation);
      return false;
    }
  }

  // --- Encoder->EKF odometry fusion (launch flag; relaunch to apply) ---

  Future<bool?> getEkfMode() async {
    if (_baseUrl == null) return null;
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/ekf_mode'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return null;
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      return json['use_ekf'] == true;
    } catch (_) { return null; }
  }

  Future<bool> setEkfMode(bool enabled) async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/ekf_mode'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'enabled': enabled}),
      ).timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) logger.warn('EKF fusion -> $enabled (relaunch to apply)', LogCategory.navigation);
      return ok;
    } catch (e) {
      logger.error('ekf_mode: $e', LogCategory.navigation);
      return false;
    }
  }

  // --- NamedWaypoints (Part D) ---

  Future<List<NamedWaypoint>> listNamedWaypoints() async {
    if (_baseUrl == null) return [];
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/waypoints'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return [];
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final list = (json['waypoints'] as List?) ?? [];
      return list.map<NamedWaypoint>((e) => NamedWaypoint.fromJson(e as Map<String, dynamic>)).toList();
    } catch (_) { return []; }
  }

  Future<NamedWaypoint?> saveNamedWaypoint({
    required String name,
    required String kind,
    required double x,
    required double y,
    required double yaw,
  }) async {
    if (_baseUrl == null) return null;
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/waypoints'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({
          'name': name, 'kind': kind,
          'pose': {'x': x, 'y': y, 'yaw': yaw},
        }),
      ).timeout(const Duration(seconds: 3));
      if (resp.statusCode != 200) {
        logger.error('save waypoint: HTTP ${resp.statusCode}', LogCategory.navigation);
        return null;
      }
      final wp = NamedWaypoint.fromJson(jsonDecode(resp.body) as Map<String, dynamic>);
      logger.success('NamedWaypoint saved: $name', LogCategory.navigation);
      // Refresh the cached list so the map overlay + Home tab pick it up
      // immediately, without waiting for the next 5 s poll.
      // ignore: unawaited_futures
      _fetchWaypoints();
      return wp;
    } catch (e) {
      logger.error('save waypoint: $e', LogCategory.navigation);
      return null;
    }
  }

  Future<bool> deleteNamedWaypoint(String name) async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.delete(
              Uri.parse('$_baseUrl/api/waypoints/${Uri.encodeComponent(name)}'))
          .timeout(const Duration(seconds: 2));
      final ok = resp.statusCode == 200;
      if (ok) {
        // ignore: unawaited_futures
        _fetchWaypoints();
      }
      return ok;
    } catch (_) { return false; }
  }

  Future<bool> navigateToNamedWaypoint(String name) async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(
              Uri.parse('$_baseUrl/api/waypoints/${Uri.encodeComponent(name)}/navigate'))
          .timeout(const Duration(seconds: 3));
      final ok = resp.statusCode == 200;
      if (ok) logger.success('Navigating to "$name"', LogCategory.navigation);
      return ok;
    } catch (e) {
      logger.error('navigate to waypoint: $e', LogCategory.navigation);
      return false;
    }
  }

  // --- Pose reset (Part G1) ---

  Future<bool> relocalize(double x, double y, double yaw) async {
    if (_baseUrl == null) return false;
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/relocalize'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'x': x, 'y': y, 'yaw': yaw}),
      ).timeout(const Duration(seconds: 2));
      final ok = resp.statusCode == 200;
      if (ok) {
        // Optimistically clear trail — old positions are now misleading
        // relative to the corrected pose.
        _pathTrail.clear();
        notifyListeners();
        logger.warn('Pose reset to (${x.toStringAsFixed(2)}, ${y.toStringAsFixed(2)})', LogCategory.navigation);
      }
      return ok;
    } catch (e) {
      logger.error('relocalize: $e', LogCategory.navigation);
      return false;
    }
  }

  // --- Param tuner (Part G2) ---

  Future<Map<String, dynamic>> listParams(String node) async {
    if (_baseUrl == null) return {};
    try {
      final resp = await http.get(
              Uri.parse('$_baseUrl/api/params?node=${Uri.encodeQueryComponent(node)}'))
          .timeout(const Duration(seconds: 3));
      if (resp.statusCode != 200) return {};
      return jsonDecode(resp.body) as Map<String, dynamic>;
    } catch (_) { return {}; }
  }

  Future<Map<String, ParamSetResult>> setParams(
      String node, Map<String, dynamic> items) async {
    if (_baseUrl == null) {
      return {
        for (final k in items.keys)
          k: const ParamSetResult(ok: false, reason: 'no bridge connection'),
      };
    }
    try {
      final resp = await http.post(
        Uri.parse('$_baseUrl/api/params'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode({'node': node, 'params': items}),
      ).timeout(const Duration(seconds: 3));
      if (resp.statusCode != 200) {
        return {
          for (final k in items.keys)
            k: ParamSetResult(
                ok: false, reason: 'HTTP ${resp.statusCode}'),
        };
      }
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final results = (json['results'] as Map?) ?? {};
      final out = <String, ParamSetResult>{};
      results.forEach((k, v) {
        final m = (v as Map?) ?? const {};
        out[k.toString()] = ParamSetResult(
          ok: m['ok'] == true,
          reason: (m['reason'] ?? '').toString(),
        );
      });
      return out;
    } catch (e) {
      return {
        for (final k in items.keys)
          k: ParamSetResult(ok: false, reason: e.toString()),
      };
    }
  }

  // --- Health (Part G3) ---

  Future<List<HealthRow>> getHealth() async {
    if (_baseUrl == null) return [];
    try {
      final resp = await http.get(Uri.parse('$_baseUrl/api/health'))
          .timeout(const Duration(seconds: 2));
      if (resp.statusCode != 200) return [];
      final json = jsonDecode(resp.body) as Map<String, dynamic>;
      final list = (json['topics'] as List?) ?? [];
      return list.map<HealthRow>((e) => HealthRow.fromJson(e as Map<String, dynamic>)).toList();
    } catch (_) { return []; }
  }

  @override
  void dispose() {
    _stopPolling();
    _closeSockets();
    super.dispose();
  }
}
