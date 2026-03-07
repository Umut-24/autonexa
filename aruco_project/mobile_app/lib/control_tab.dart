import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'joystick_widget.dart';
import 'pico_udp_service.dart';

/// The Control Tab for driving the Ackermann chassis via a virtual joystick.
/// Has two modes:
///   - Normal mode: fixed layout (NO scrolling), connection bar, joystick, telemetry, e-stop
///   - Fullscreen mode: landscape, fullscreen, joystick + minimal HUD only
class ControlTab extends StatefulWidget {
  const ControlTab({super.key});

  @override
  State<ControlTab> createState() => _ControlTabState();
}

class _ControlTabState extends State<ControlTab> {
  final PicoUdpService _udpService = PicoUdpService();
  final TextEditingController _ipController = TextEditingController(text: '192.168.1.100');
  final TextEditingController _portController = TextEditingController(text: '4210');

  bool _isConnected = false;
  bool _emergencyStopped = false;
  bool _fullscreenMode = false;
  double _speedLimit = 0.5;
  double _currentX = 0;
  double _currentY = 0;
  PicoTelemetry _telemetry = const PicoTelemetry();

  @override
  void initState() {
    super.initState();
    _udpService.onTelemetryUpdate = (t) {
      if (mounted) setState(() => _telemetry = t);
    };
    _udpService.onConnectionChanged = (c) {
      if (mounted) setState(() => _isConnected = c);
    };
    _udpService.setSpeedLimit(_speedLimit);
  }

  @override
  void dispose() {
    _exitFullscreen();
    _udpService.disconnect();
    _ipController.dispose();
    _portController.dispose();
    super.dispose();
  }

  void _toggleConnection() async {
    if (_isConnected) {
      await _udpService.disconnect();
    } else {
      final ip = _ipController.text.trim();
      final port = int.tryParse(_portController.text.trim()) ?? 4210;
      if (ip.isNotEmpty) {
        await _udpService.connect(ip, port: port);
      }
    }
  }

  void _toggleEmergencyStop() {
    setState(() {
      _emergencyStopped = !_emergencyStopped;
      if (_emergencyStopped) {
        _udpService.emergencyStop();
      } else {
        _udpService.releaseEmergencyStop();
      }
    });
  }

  void _enterFullscreen() {
    setState(() => _fullscreenMode = true);
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }

  void _exitFullscreen() {
    setState(() => _fullscreenMode = false);
    SystemChrome.setEnabledSystemUIMode(SystemUiMode.edgeToEdge);
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }

  void _onJoystickMove(double x, double y) {
    setState(() {
      _currentX = x;
      _currentY = y;
    });
    _udpService.updateJoystick(x, y);
  }

  void _onJoystickRelease(double x, double y) {
    setState(() {
      _currentX = 0;
      _currentY = 0;
    });
  }

  @override
  Widget build(BuildContext context) {
    if (_fullscreenMode) {
      return _buildFullscreenMode(context);
    }
    return _buildNormalMode(context);
  }

  // ==================== FULLSCREEN LANDSCAPE MODE ====================
  Widget _buildFullscreenMode(BuildContext context) {
    final screenHeight = MediaQuery.of(context).size.height;
    final joystickSize = screenHeight * 0.75;

    return Scaffold(
      backgroundColor: const Color(0xFF0A0A12),
      body: SafeArea(
        child: Stack(
          children: [
            // Main content: Joystick centered
            Center(
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  // Left: Telemetry HUD
                  SizedBox(
                    width: 120,
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        _hudItem('STEER', _currentX.toStringAsFixed(2)),
                        const SizedBox(height: 12),
                        _hudItem('THROTTLE', _currentY.toStringAsFixed(2)),
                        const SizedBox(height: 12),
                        _hudItem('L SPD', _telemetry.leftRPM.toStringAsFixed(0)),
                        const SizedBox(height: 12),
                        _hudItem('R SPD', _telemetry.rightRPM.toStringAsFixed(0)),
                        const SizedBox(height: 12),
                        _hudItem('SERVO', '${_telemetry.servoAngle.toStringAsFixed(0)}°'),
                      ],
                    ),
                  ),

                  // Center: Joystick
                  VirtualJoystick(
                    size: joystickSize,
                    onMove: _onJoystickMove,
                    onRelease: _onJoystickRelease,
                  ),

                  // Right: Speed + E-Stop
                  SizedBox(
                    width: 120,
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        _hudItem('SPEED', '${(_speedLimit * 100).toInt()}%'),
                        const SizedBox(height: 16),
                        Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            _speedButton(Icons.remove, () {
                              setState(() => _speedLimit = (_speedLimit - 0.1).clamp(0.1, 1.0));
                              _udpService.setSpeedLimit(_speedLimit);
                            }),
                            const SizedBox(width: 8),
                            _speedButton(Icons.add, () {
                              setState(() => _speedLimit = (_speedLimit + 0.1).clamp(0.1, 1.0));
                              _udpService.setSpeedLimit(_speedLimit);
                            }),
                          ],
                        ),
                        const SizedBox(height: 24),
                        SizedBox(
                          width: 100,
                          height: 48,
                          child: ElevatedButton(
                            onPressed: _toggleEmergencyStop,
                            style: ElevatedButton.styleFrom(
                              backgroundColor: _emergencyStopped
                                  ? Colors.orange.shade800
                                  : const Color(0xFFB71C1C),
                              shape: RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(10),
                              ),
                              padding: EdgeInsets.zero,
                            ),
                            child: Text(
                              _emergencyStopped ? 'RESUME' : 'E-STOP',
                              style: const TextStyle(
                                fontSize: 12,
                                fontWeight: FontWeight.bold,
                                letterSpacing: 1.2,
                              ),
                            ),
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // Connection indicator (top-left)
            Positioned(
              top: 8,
              left: 12,
              child: Container(
                padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                decoration: BoxDecoration(
                  color: Colors.black54,
                  borderRadius: BorderRadius.circular(6),
                ),
                child: Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Container(
                      width: 8,
                      height: 8,
                      decoration: BoxDecoration(
                        shape: BoxShape.circle,
                        color: _isConnected ? Colors.green : Colors.red,
                      ),
                    ),
                    const SizedBox(width: 6),
                    Text(
                      _isConnected ? 'LINKED' : 'NO LINK',
                      style: TextStyle(
                        fontSize: 10,
                        color: _isConnected ? Colors.green : Colors.red,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ],
                ),
              ),
            ),

            // Exit fullscreen button (top-right)
            Positioned(
              top: 8,
              right: 12,
              child: GestureDetector(
                onTap: _exitFullscreen,
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                  decoration: BoxDecoration(
                    color: Colors.white.withValues(alpha: 0.1),
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: Colors.white.withValues(alpha: 0.2)),
                  ),
                  child: const Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.fullscreen_exit, size: 18, color: Colors.white70),
                      SizedBox(width: 4),
                      Text(
                        'EXIT',
                        style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.bold,
                          color: Colors.white70,
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _hudItem(String label, String value) {
    return Column(
      children: [
        Text(
          label,
          style: TextStyle(
            fontSize: 9,
            fontWeight: FontWeight.w600,
            letterSpacing: 1.2,
            color: Colors.grey[600],
          ),
        ),
        const SizedBox(height: 2),
        Text(
          value,
          style: const TextStyle(
            fontSize: 16,
            fontWeight: FontWeight.bold,
            fontFamily: 'monospace',
            color: Colors.white,
          ),
        ),
      ],
    );
  }

  Widget _speedButton(IconData icon, VoidCallback onTap) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: Colors.white.withValues(alpha: 0.08),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.white.withValues(alpha: 0.15)),
        ),
        child: Icon(icon, size: 20, color: Colors.white70),
      ),
    );
  }

  // ==================== NORMAL MODE (NO SCROLL) ====================
  Widget _buildNormalMode(BuildContext context) {
    return SafeArea(
      child: Column(
        children: [
          // Connection bar (fixed at top, compact)
          _buildConnectionBar(),

          // Main content — NO scroll, flex layout
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 12),
              child: Column(
                children: [
                  const SizedBox(height: 4),

                  // Joystick area with fullscreen button
                  Expanded(
                    flex: 5,
                    child: _buildJoystickArea(),
                  ),

                  const SizedBox(height: 8),

                  // Speed limiter
                  _buildSpeedLimiter(),

                  const SizedBox(height: 8),

                  // Telemetry row (compact)
                  _buildTelemetryRow(),

                  const SizedBox(height: 8),

                  // E-Stop button
                  _buildEmergencyStop(),

                  const SizedBox(height: 8),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildConnectionBar() {
    return Container(
      margin: const EdgeInsets.fromLTRB(8, 4, 8, 0),
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.05),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(
          color: _isConnected
              ? Colors.green.withValues(alpha: 0.4)
              : Colors.white.withValues(alpha: 0.1),
        ),
      ),
      child: Row(
        children: [
          Container(
            width: 8,
            height: 8,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: _isConnected ? Colors.green : Colors.grey,
              boxShadow: _isConnected
                  ? [BoxShadow(color: Colors.green.withValues(alpha: 0.5), blurRadius: 6)]
                  : null,
            ),
          ),
          const SizedBox(width: 6),
          Expanded(
            flex: 3,
            child: SizedBox(
              height: 32,
              child: TextField(
                controller: _ipController,
                style: const TextStyle(fontSize: 12),
                decoration: InputDecoration(
                  hintText: 'Pico IP',
                  hintStyle: TextStyle(fontSize: 11, color: Colors.grey[600]),
                  filled: true,
                  fillColor: Colors.white.withValues(alpha: 0.05),
                  contentPadding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
                  border: OutlineInputBorder(
                    borderRadius: BorderRadius.circular(6),
                    borderSide: BorderSide.none,
                  ),
                ),
              ),
            ),
          ),
          const SizedBox(width: 4),
          SizedBox(
            width: 52,
            height: 32,
            child: TextField(
              controller: _portController,
              style: const TextStyle(fontSize: 12),
              keyboardType: TextInputType.number,
              decoration: InputDecoration(
                hintText: 'Port',
                hintStyle: TextStyle(fontSize: 11, color: Colors.grey[600]),
                filled: true,
                fillColor: Colors.white.withValues(alpha: 0.05),
                contentPadding: const EdgeInsets.symmetric(horizontal: 8, vertical: 6),
                border: OutlineInputBorder(
                  borderRadius: BorderRadius.circular(6),
                  borderSide: BorderSide.none,
                ),
              ),
            ),
          ),
          const SizedBox(width: 4),
          SizedBox(
            height: 32,
            child: ElevatedButton(
              onPressed: _toggleConnection,
              style: ElevatedButton.styleFrom(
                backgroundColor: _isConnected
                    ? Colors.red.withValues(alpha: 0.8)
                    : const Color(0xFF0F3460),
                padding: const EdgeInsets.symmetric(horizontal: 12),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(6),
                ),
              ),
              child: Text(
                _isConnected ? 'Stop' : 'Link',
                style: const TextStyle(fontSize: 11, fontWeight: FontWeight.bold),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildJoystickArea() {
    return Container(
      width: double.infinity,
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.02),
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: Colors.white.withValues(alpha: 0.05)),
      ),
      child: Column(
        children: [
          // Header row with label + fullscreen button
          Padding(
            padding: const EdgeInsets.fromLTRB(14, 8, 8, 0),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Row(
                  children: [
                    Text(
                      'X: ${_currentX.toStringAsFixed(2)}',
                      style: TextStyle(fontSize: 10, fontFamily: 'monospace', color: Colors.grey[500]),
                    ),
                    const SizedBox(width: 12),
                    Text(
                      'Y: ${_currentY.toStringAsFixed(2)}',
                      style: TextStyle(fontSize: 10, fontFamily: 'monospace', color: Colors.grey[500]),
                    ),
                  ],
                ),
                GestureDetector(
                  onTap: _enterFullscreen,
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
                    decoration: BoxDecoration(
                      color: const Color(0xFFE94560).withValues(alpha: 0.15),
                      borderRadius: BorderRadius.circular(6),
                      border: Border.all(
                        color: const Color(0xFFE94560).withValues(alpha: 0.4),
                      ),
                    ),
                    child: const Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Icon(Icons.fullscreen, size: 14, color: Color(0xFFE94560)),
                        SizedBox(width: 3),
                        Text(
                          'FULLSCREEN',
                          style: TextStyle(
                            fontSize: 9,
                            fontWeight: FontWeight.bold,
                            letterSpacing: 0.8,
                            color: Color(0xFFE94560),
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
          // Joystick — fills remaining space
          Expanded(
            child: Center(
              child: LayoutBuilder(
                builder: (context, constraints) {
                  final joystickSize = constraints.maxHeight * 0.85;
                  return VirtualJoystick(
                    size: joystickSize.clamp(140, 240),
                    onMove: _onJoystickMove,
                    onRelease: _onJoystickRelease,
                  );
                },
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSpeedLimiter() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.white.withValues(alpha: 0.03),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: Colors.white.withValues(alpha: 0.05)),
      ),
      child: Row(
        children: [
          Text(
            'SPEED',
            style: TextStyle(
              fontSize: 10,
              fontWeight: FontWeight.w600,
              letterSpacing: 1.2,
              color: Colors.grey[500],
            ),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: SliderTheme(
              data: SliderTheme.of(context).copyWith(
                activeTrackColor: const Color(0xFFE94560),
                inactiveTrackColor: Colors.white.withValues(alpha: 0.08),
                thumbColor: const Color(0xFFE94560),
                overlayColor: const Color(0xFFE94560).withValues(alpha: 0.15),
                trackHeight: 3,
                thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 7),
              ),
              child: Slider(
                value: _speedLimit,
                min: 0.1,
                max: 1.0,
                divisions: 9,
                onChanged: (v) {
                  setState(() => _speedLimit = v);
                  _udpService.setSpeedLimit(v);
                },
              ),
            ),
          ),
          SizedBox(
            width: 36,
            child: Text(
              '${(_speedLimit * 100).toInt()}%',
              textAlign: TextAlign.right,
              style: const TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.bold,
                color: Color(0xFFE94560),
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTelemetryRow() {
    return Row(
      children: [
        _telemetryChip('L', _telemetry.leftRPM.toStringAsFixed(0), Icons.rotate_left),
        const SizedBox(width: 6),
        _telemetryChip('R', _telemetry.rightRPM.toStringAsFixed(0), Icons.rotate_right),
        const SizedBox(width: 6),
        _telemetryChip('°', _telemetry.servoAngle.toStringAsFixed(0), Icons.compass_calibration),
        const SizedBox(width: 6),
        _telemetryChip('V', _telemetry.battery.toStringAsFixed(1), Icons.battery_charging_full),
      ],
    );
  }

  Widget _telemetryChip(String label, String value, IconData icon) {
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 6),
        decoration: BoxDecoration(
          color: Colors.white.withValues(alpha: 0.03),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: Colors.white.withValues(alpha: 0.04)),
        ),
        child: Column(
          children: [
            Icon(icon, size: 14, color: const Color(0xFF0F3460)),
            const SizedBox(height: 2),
            Text(
              value,
              style: const TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.bold,
                fontFamily: 'monospace',
              ),
            ),
            Text(label, style: TextStyle(fontSize: 9, color: Colors.grey[600])),
          ],
        ),
      ),
    );
  }

  Widget _buildEmergencyStop() {
    return SizedBox(
      width: double.infinity,
      height: 48,
      child: ElevatedButton(
        onPressed: _toggleEmergencyStop,
        style: ElevatedButton.styleFrom(
          backgroundColor: _emergencyStopped
              ? Colors.orange.shade800
              : const Color(0xFFB71C1C),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(10),
          ),
          elevation: _emergencyStopped ? 0 : 4,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(_emergencyStopped ? Icons.play_arrow : Icons.emergency, size: 20),
            const SizedBox(width: 6),
            Text(
              _emergencyStopped ? 'RELEASE E-STOP' : 'EMERGENCY STOP',
              style: const TextStyle(
                fontSize: 14,
                fontWeight: FontWeight.bold,
                letterSpacing: 1.2,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
