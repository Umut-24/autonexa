import 'dart:async';
import 'dart:convert';
import 'dart:io';

/// Telemetry data received from the Pico W.
class PicoTelemetry {
  final double leftRPM;
  final double rightRPM;
  final double battery;
  final double pidErrorL;
  final double pidErrorR;
  final double servoAngle;
  final bool connected;

  const PicoTelemetry({
    this.leftRPM = 0,
    this.rightRPM = 0,
    this.battery = 0,
    this.pidErrorL = 0,
    this.pidErrorR = 0,
    this.servoAngle = 90,
    this.connected = false,
  });

  factory PicoTelemetry.fromJson(Map<String, dynamic> json) {
    return PicoTelemetry(
      leftRPM: (json['lr'] ?? 0).toDouble(),
      rightRPM: (json['rr'] ?? 0).toDouble(),
      battery: (json['bat'] ?? 0).toDouble(),
      pidErrorL: (json['el'] ?? 0).toDouble(),
      pidErrorR: (json['er'] ?? 0).toDouble(),
      servoAngle: (json['sa'] ?? 90).toDouble(),
      connected: true,
    );
  }
}

/// UDP service for communicating with the Raspberry Pi Pico WH.
///
/// Sends joystick commands as JSON and receives telemetry responses.
/// Includes a safety heartbeat: sends zero command if idle, and a
/// watchdog concept where the Pico stops motors if no packet arrives.
class PicoUdpService {
  RawDatagramSocket? _socket;
  InternetAddress? _picoAddress;
  int _picoPort = 4210;
  Timer? _sendTimer;
  Timer? _heartbeatTimer;
  Timer? _timeoutTimer;

  double _lastX = 0;
  double _lastY = 0;
  double _speedLimit = 1.0; // 0.0 to 1.0
  bool _emergencyStop = false;
  bool _isConnected = false;
  DateTime? _lastReceived;

  /// Current telemetry from Pico.
  PicoTelemetry telemetry = const PicoTelemetry();

  /// Callback when telemetry is updated.
  void Function(PicoTelemetry)? onTelemetryUpdate;

  /// Callback when connection status changes.
  void Function(bool connected)? onConnectionChanged;

  bool get isConnected => _isConnected;
  double get speedLimit => _speedLimit;

  /// Connect to the Pico W at the given IP and port.
  Future<bool> connect(String ip, {int port = 4210}) async {
    try {
      await disconnect();

      _picoAddress = InternetAddress(ip);
      _picoPort = port;

      // Bind to any available port for receiving
      _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
      _socket!.broadcastEnabled = true;

      // Listen for incoming telemetry
      _socket!.listen((event) {
        if (event == RawSocketEvent.read) {
          final datagram = _socket!.receive();
          if (datagram != null) {
            _handleIncoming(datagram);
          }
        }
      });

      // Start periodic command sending at 20Hz (every 50ms)
      _sendTimer = Timer.periodic(const Duration(milliseconds: 50), (_) {
        _sendCommand();
      });

      // Heartbeat: send zero command every 200ms even if idle
      _heartbeatTimer = Timer.periodic(const Duration(milliseconds: 200), (_) {
        // The regular send timer handles this, but heartbeat ensures
        // the Pico knows we're still alive
      });

      // Connection timeout checker
      _timeoutTimer = Timer.periodic(const Duration(seconds: 1), (_) {
        if (_lastReceived != null) {
          final elapsed = DateTime.now().difference(_lastReceived!);
          if (elapsed.inSeconds > 2 && _isConnected) {
            _isConnected = false;
            telemetry = const PicoTelemetry(connected: false);
            onConnectionChanged?.call(false);
            onTelemetryUpdate?.call(telemetry);
          }
        }
      });

      // Send an initial ping
      _sendCommand();
      _isConnected = true;
      onConnectionChanged?.call(true);
      return true;
    } catch (e) {
      _isConnected = false;
      onConnectionChanged?.call(false);
      return false;
    }
  }

  /// Disconnect and clean up resources.
  Future<void> disconnect() async {
    _sendTimer?.cancel();
    _heartbeatTimer?.cancel();
    _timeoutTimer?.cancel();
    _sendTimer = null;
    _heartbeatTimer = null;
    _timeoutTimer = null;

    // Send stop command before disconnecting
    if (_socket != null && _picoAddress != null) {
      _emergencyStop = true;
      _sendCommand();
      await Future.delayed(const Duration(milliseconds: 100));
    }

    _socket?.close();
    _socket = null;
    _isConnected = false;
    _emergencyStop = false;
    _lastX = 0;
    _lastY = 0;
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

  /// Trigger emergency stop — sends zero and locks controls.
  void emergencyStop() {
    _emergencyStop = true;
    _lastX = 0;
    _lastY = 0;
    _sendCommand();
  }

  /// Release emergency stop.
  void releaseEmergencyStop() {
    _emergencyStop = false;
  }

  void _sendCommand() {
    if (_socket == null || _picoAddress == null) return;

    double x = _emergencyStop ? 0 : _lastX;
    double y = _emergencyStop ? 0 : (_lastY * _speedLimit);

    final data = jsonEncode({
      'x': double.parse(x.toStringAsFixed(3)),
      'y': double.parse(y.toStringAsFixed(3)),
      'e': _emergencyStop ? 1 : 0,
    });

    try {
      _socket!.send(
        utf8.encode(data),
        _picoAddress!,
        _picoPort,
      );
    } catch (_) {}
  }

  void _handleIncoming(Datagram datagram) {
    try {
      final raw = utf8.decode(datagram.data);
      final json = jsonDecode(raw) as Map<String, dynamic>;
      telemetry = PicoTelemetry.fromJson(json);
      _lastReceived = DateTime.now();

      if (!_isConnected) {
        _isConnected = true;
        onConnectionChanged?.call(true);
      }

      onTelemetryUpdate?.call(telemetry);
    } catch (_) {}
  }
}
