import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'main.dart' show AppColors;
import 'joystick_widget.dart';
import 'pico_udp_service.dart';

/// Driving interface with virtual joystick, speed limiter, telemetry, and E-STOP.
/// Two modes: normal (portrait) and fullscreen (landscape).
class ControlTab extends StatefulWidget {
  final String? bridgeUrl;
  final bool micropythonMode;

  const ControlTab({super.key, this.bridgeUrl, this.micropythonMode = false});

  @override
  State<ControlTab> createState() => _ControlTabState();
}

class _ControlTabState extends State<ControlTab> {
  final PicoUdpService _controlService = PicoUdpService();

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
    _controlService.onTelemetryUpdate = (t) {
      if (mounted) setState(() => _telemetry = t);
    };
    _controlService.onConnectionChanged = (c) {
      if (mounted) setState(() => _isConnected = c);
    };
    _controlService.setSpeedLimit(_speedLimit);
    _autoConnect();
  }

  String? get _effectiveUrl {
    final url = widget.bridgeUrl;
    if (url == null || url.isEmpty) return null;
    if (!widget.micropythonMode) return url;
    // In MicroPython mode, replace port with 5001
    try {
      final uri = Uri.parse(url);
      return uri.replace(port: 5001).toString();
    } catch (_) {
      return url;
    }
  }

  void _autoConnect() async {
    final url = _effectiveUrl;
    if (url != null && url.isNotEmpty) {
      await _controlService.connect(url);
    }
  }

  @override
  void didUpdateWidget(covariant ControlTab oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.bridgeUrl != oldWidget.bridgeUrl ||
        widget.micropythonMode != oldWidget.micropythonMode) {
      _controlService.disconnect().then((_) => _autoConnect());
    }
  }

  @override
  void dispose() {
    _exitFullscreen();
    _controlService.disconnect();
    super.dispose();
  }

  void _toggleConnection() async {
    if (_isConnected) {
      await _controlService.disconnect();
    } else if (widget.bridgeUrl != null && widget.bridgeUrl!.isNotEmpty) {
      await _controlService.connect(widget.bridgeUrl!);
    }
  }

  void _toggleEmergencyStop() {
    setState(() {
      _emergencyStopped = !_emergencyStopped;
      if (_emergencyStopped) {
        _controlService.emergencyStop();
      } else {
        _controlService.releaseEmergencyStop();
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
    _controlService.updateJoystick(x, y);
  }

  void _onJoystickRelease(double x, double y) {
    setState(() {
      _currentX = 0;
      _currentY = 0;
    });
  }

  @override
  Widget build(BuildContext context) {
    if (_fullscreenMode) return _buildFullscreenMode(context);
    return _buildNormalMode(context);
  }

  // ═══════════════════════════════════════════════════════════════════════
  //  FULLSCREEN LANDSCAPE MODE
  // ═══════════════════════════════════════════════════════════════════════
  Widget _buildFullscreenMode(BuildContext context) {
    final screenHeight = MediaQuery.of(context).size.height;
    final joystickSize = screenHeight * 0.75;

    return Scaffold(
      backgroundColor: AppColors.background,
      body: SafeArea(
        child: Stack(
          children: [
            Center(
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  // Left HUD
                  SizedBox(
                    width: 120,
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        _hudItem('STEER', _currentX.toStringAsFixed(2)),
                        const SizedBox(height: 14),
                        _hudItem('THROTTLE', _currentY.toStringAsFixed(2)),
                        const SizedBox(height: 14),
                        _hudItem('L VEL', _telemetry.leftVel.toStringAsFixed(2)),
                        const SizedBox(height: 14),
                        _hudItem('R VEL', _telemetry.rightVel.toStringAsFixed(2)),
                        const SizedBox(height: 14),
                        _hudItem('Vx', _telemetry.odomVx.toStringAsFixed(2)),
                      ],
                    ),
                  ),
                  // Joystick
                  VirtualJoystick(
                    size: joystickSize,
                    onMove: _onJoystickMove,
                    onRelease: _onJoystickRelease,
                  ),
                  // Right HUD
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
                            _speedBtn(Icons.remove, () {
                              setState(() => _speedLimit = (_speedLimit - 0.1).clamp(0.1, 1.0));
                              _controlService.setSpeedLimit(_speedLimit);
                            }),
                            const SizedBox(width: 8),
                            _speedBtn(Icons.add, () {
                              setState(() => _speedLimit = (_speedLimit + 0.1).clamp(0.1, 1.0));
                              _controlService.setSpeedLimit(_speedLimit);
                            }),
                          ],
                        ),
                        const SizedBox(height: 24),
                        _estopButton(compact: true),
                      ],
                    ),
                  ),
                ],
              ),
            ),
            // Connection badge (top-left)
            Positioned(
              top: 8,
              left: 12,
              child: _connectionBadge(),
            ),
            // Exit button (top-right)
            Positioned(
              top: 8,
              right: 12,
              child: GestureDetector(
                onTap: _exitFullscreen,
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                  decoration: BoxDecoration(
                    color: AppColors.surfaceLight,
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: AppColors.border),
                  ),
                  child: const Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.fullscreen_exit_rounded, size: 16, color: AppColors.textSecondary),
                      SizedBox(width: 4),
                      Text('EXIT',
                          style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600,
                              color: AppColors.textSecondary, letterSpacing: 1)),
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
        Text(label,
            style: const TextStyle(fontSize: 9, fontWeight: FontWeight.w600,
                letterSpacing: 1.2, color: AppColors.textSecondary)),
        const SizedBox(height: 2),
        Text(value,
            style: const TextStyle(fontSize: 16, fontWeight: FontWeight.bold,
                fontFamily: 'monospace', color: AppColors.textPrimary)),
      ],
    );
  }

  Widget _speedBtn(IconData icon, VoidCallback onTap) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: AppColors.surfaceLight,
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: AppColors.border),
        ),
        child: Icon(icon, size: 20, color: AppColors.textSecondary),
      ),
    );
  }

  Widget _connectionBadge() {
    final modeLabel = widget.micropythonMode ? 'MPY' : 'ROS2';
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: AppColors.border),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 7,
            height: 7,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: _isConnected ? AppColors.success : AppColors.error,
            ),
          ),
          const SizedBox(width: 6),
          Text(
            _isConnected ? 'LINKED' : 'NO LINK',
            style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600,
                color: _isConnected ? AppColors.success : AppColors.error),
          ),
          const SizedBox(width: 6),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 5, vertical: 1),
            decoration: BoxDecoration(
              color: widget.micropythonMode
                  ? AppColors.accent.withValues(alpha: 0.15)
                  : AppColors.accentDim.withValues(alpha: 0.3),
              borderRadius: BorderRadius.circular(4),
            ),
            child: Text(
              modeLabel,
              style: TextStyle(fontSize: 8, fontWeight: FontWeight.w700,
                  letterSpacing: 0.5,
                  color: widget.micropythonMode ? AppColors.accent : AppColors.textSecondary),
            ),
          ),
          if (widget.micropythonMode && _telemetry.picoState.isNotEmpty) ...[
            const SizedBox(width: 6),
            Text(
              _telemetry.picoState,
              style: TextStyle(fontSize: 9, fontWeight: FontWeight.w600,
                  color: _telemetry.picoState == 'IDLE'
                      ? AppColors.success
                      : _telemetry.picoState == 'FAILED'
                          ? AppColors.error
                          : AppColors.warning),
            ),
          ],
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════════════════
  //  NORMAL PORTRAIT MODE
  // ═══════════════════════════════════════════════════════════════════════
  Widget _buildNormalMode(BuildContext context) {
    return SafeArea(
      child: Column(
        children: [
          // Header
          Padding(
            padding: const EdgeInsets.fromLTRB(20, 12, 20, 4),
            child: Row(
              children: [
                const Text('Control',
                    style: TextStyle(fontSize: 22, fontWeight: FontWeight.w700,
                        color: AppColors.textPrimary, letterSpacing: -0.5)),
                const Spacer(),
                _connectionBadge(),
                const SizedBox(width: 8),
                _linkButton(),
              ],
            ),
          ),

          // Main content
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: Column(
                children: [
                  const SizedBox(height: 6),

                  // Joystick area
                  Expanded(flex: 5, child: _buildJoystickArea()),
                  const SizedBox(height: 10),

                  // Speed limiter
                  _buildSpeedLimiter(),
                  const SizedBox(height: 10),

                  // Telemetry
                  _buildTelemetryRow(),
                  const SizedBox(height: 10),

                  // E-STOP
                  _estopButton(compact: false),
                  const SizedBox(height: 10),

                  // MicroPython telemetry extras
                  if (widget.micropythonMode && _telemetry.picoState.isNotEmpty) ...[
                    _buildMicroPythonTelemetry(),
                    const SizedBox(height: 10),
                  ],

                  // Goal buttons (mode-dependent)
                  if (widget.bridgeUrl != null) ...[
                    if (widget.micropythonMode)
                      _buildMicroPythonGoalButtons()
                    else
                      _buildNavGoalButton(),
                    const SizedBox(height: 10),
                  ],
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _linkButton() {
    final hasUrl = widget.bridgeUrl != null && widget.bridgeUrl!.isNotEmpty;
    if (!hasUrl) return const SizedBox.shrink();
    return GestureDetector(
      onTap: _toggleConnection,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
        decoration: BoxDecoration(
          color: _isConnected
              ? AppColors.error.withValues(alpha: 0.15)
              : AppColors.accentDim.withValues(alpha: 0.3),
          borderRadius: BorderRadius.circular(8),
          border: Border.all(
            color: _isConnected
                ? AppColors.error.withValues(alpha: 0.4)
                : AppColors.accentDim.withValues(alpha: 0.5),
          ),
        ),
        child: Text(
          _isConnected ? 'Stop' : 'Link',
          style: TextStyle(
            fontSize: 11,
            fontWeight: FontWeight.w600,
            color: _isConnected ? AppColors.error : AppColors.textPrimary,
          ),
        ),
      ),
    );
  }

  Widget _buildJoystickArea() {
    return Container(
      width: double.infinity,
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: AppColors.border),
      ),
      child: Column(
        children: [
          // Header with XY values + fullscreen
          Padding(
            padding: const EdgeInsets.fromLTRB(14, 10, 10, 0),
            child: Row(
              children: [
                Text('X: ${_currentX.toStringAsFixed(2)}',
                    style: const TextStyle(fontSize: 11, fontFamily: 'monospace',
                        color: AppColors.textSecondary)),
                const SizedBox(width: 14),
                Text('Y: ${_currentY.toStringAsFixed(2)}',
                    style: const TextStyle(fontSize: 11, fontFamily: 'monospace',
                        color: AppColors.textSecondary)),
                const Spacer(),
                GestureDetector(
                  onTap: _enterFullscreen,
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                    decoration: BoxDecoration(
                      color: AppColors.accent.withValues(alpha: 0.12),
                      borderRadius: BorderRadius.circular(7),
                      border: Border.all(color: AppColors.accent.withValues(alpha: 0.3)),
                    ),
                    child: const Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Icon(Icons.fullscreen_rounded, size: 14, color: AppColors.accent),
                        SizedBox(width: 4),
                        Text('FULLSCREEN',
                            style: TextStyle(fontSize: 9, fontWeight: FontWeight.w700,
                                letterSpacing: 0.8, color: AppColors.accent)),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
          // Joystick
          Expanded(
            child: Center(
              child: LayoutBuilder(
                builder: (context, constraints) {
                  final joystickSize = constraints.maxHeight * 0.85;
                  return VirtualJoystick(
                    size: joystickSize.clamp(140, 260),
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
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: AppColors.border),
      ),
      child: Row(
        children: [
          const Text('SPEED',
              style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600,
                  letterSpacing: 1.2, color: AppColors.textSecondary)),
          const SizedBox(width: 10),
          Expanded(
            child: SliderTheme(
              data: SliderTheme.of(context).copyWith(
                activeTrackColor: AppColors.accent,
                inactiveTrackColor: AppColors.surfaceLight,
                thumbColor: AppColors.accent,
                overlayColor: AppColors.accent.withValues(alpha: 0.15),
                trackHeight: 4,
                thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 8),
              ),
              child: Slider(
                value: _speedLimit,
                min: 0.1,
                max: 1.0,
                divisions: 9,
                onChanged: (v) {
                  setState(() => _speedLimit = v);
                  _controlService.setSpeedLimit(v);
                },
              ),
            ),
          ),
          SizedBox(
            width: 40,
            child: Text(
              '${(_speedLimit * 100).toInt()}%',
              textAlign: TextAlign.right,
              style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w700,
                  color: AppColors.accent),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTelemetryRow() {
    return Row(
      children: [
        _telemetryChip('L', _telemetry.leftVel.toStringAsFixed(2), Icons.rotate_left_rounded),
        const SizedBox(width: 6),
        _telemetryChip('R', _telemetry.rightVel.toStringAsFixed(2), Icons.rotate_right_rounded),
        const SizedBox(width: 6),
        _telemetryChip('Vx', _telemetry.odomVx.toStringAsFixed(2), Icons.speed_rounded),
        const SizedBox(width: 6),
        _telemetryChip('Wz', _telemetry.odomWz.toStringAsFixed(2), Icons.rotate_90_degrees_ccw_rounded),
      ],
    );
  }

  Widget _telemetryChip(String label, String value, IconData icon) {
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 4),
        decoration: BoxDecoration(
          color: AppColors.surface,
          borderRadius: BorderRadius.circular(10),
          border: Border.all(color: AppColors.border),
        ),
        child: Column(
          children: [
            Icon(icon, size: 14, color: AppColors.accentDim),
            const SizedBox(height: 3),
            Text(value,
                style: const TextStyle(fontSize: 13, fontWeight: FontWeight.w700,
                    fontFamily: 'monospace', color: AppColors.textPrimary)),
            const SizedBox(height: 1),
            Text(label, style: const TextStyle(fontSize: 9, color: AppColors.textSecondary)),
          ],
        ),
      ),
    );
  }

  Widget _estopButton({required bool compact}) {
    final isActive = _emergencyStopped;
    return SizedBox(
      width: compact ? 100 : double.infinity,
      height: compact ? 48 : 52,
      child: ElevatedButton(
        onPressed: _toggleEmergencyStop,
        style: ElevatedButton.styleFrom(
          backgroundColor: isActive ? AppColors.warning : const Color(0xFFB71C1C),
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          elevation: 0,
          padding: compact ? EdgeInsets.zero : null,
        ),
        child: compact
            ? Text(
                isActive ? 'RESUME' : 'E-STOP',
                style: const TextStyle(fontSize: 11, fontWeight: FontWeight.w700, letterSpacing: 1.2),
              )
            : Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(isActive ? Icons.play_arrow_rounded : Icons.emergency_rounded, size: 22),
                  const SizedBox(width: 8),
                  Text(
                    isActive ? 'RELEASE E-STOP' : 'EMERGENCY STOP',
                    style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w700, letterSpacing: 1),
                  ),
                ],
              ),
      ),
    );
  }

  Widget _buildNavGoalButton() {
    return SizedBox(
      width: double.infinity,
      height: 44,
      child: ElevatedButton.icon(
        onPressed: _showNavGoalDialog,
        icon: const Icon(Icons.navigation_rounded, size: 18),
        label: const Text('Send Nav2 Goal', style: TextStyle(fontWeight: FontWeight.w600)),
        style: ElevatedButton.styleFrom(
          backgroundColor: AppColors.accentDim,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        ),
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════════════════
  //  MICROPYTHON MODE WIDGETS
  // ═══════════════════════════════════════════════════════════════════════

  Widget _buildMicroPythonTelemetry() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
      decoration: BoxDecoration(
        color: AppColors.surface,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: AppColors.border),
      ),
      child: Row(
        children: [
          _microTelemetryItem('STATE', _telemetry.picoState),
          _microTelemetryItem('DIST', '${_telemetry.odomX.toStringAsFixed(3)}m'),
          _microTelemetryItem('HDG', '${_telemetry.headingDeg.toStringAsFixed(1)}°'),
          _microTelemetryItem('L TK', '${_telemetry.leftTicks}'),
          _microTelemetryItem('R TK', '${_telemetry.rightTicks}'),
        ],
      ),
    );
  }

  Widget _microTelemetryItem(String label, String value) {
    return Expanded(
      child: Column(
        children: [
          Text(value,
              textAlign: TextAlign.center,
              style: const TextStyle(fontSize: 11, fontWeight: FontWeight.w700,
                  fontFamily: 'monospace', color: AppColors.textPrimary)),
          const SizedBox(height: 1),
          Text(label,
              style: const TextStyle(fontSize: 8, letterSpacing: 0.5,
                  color: AppColors.textSecondary)),
        ],
      ),
    );
  }

  Widget _buildMicroPythonGoalButtons() {
    return Column(
      children: [
        Row(
          children: [
            Expanded(
              child: _goalButton('DRIVE', Icons.arrow_forward_rounded, AppColors.accentDim, () {
                _showDriveDialog();
              }),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: _goalButton('TURN', Icons.rotate_right_rounded, AppColors.accentDim, () {
                _showTurnDialog();
              }),
            ),
          ],
        ),
        const SizedBox(height: 8),
        Row(
          children: [
            Expanded(
              child: _goalButton('STOP', Icons.stop_rounded, AppColors.error.withValues(alpha: 0.7), () {
                _sendGoalCommand({'cmd': 'STOP'});
              }),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: _goalButton('RESET ODOM', Icons.restart_alt_rounded, AppColors.surfaceLight, () {
                _sendGoalCommand({'cmd': 'RESET_ODOM'});
              }),
            ),
          ],
        ),
      ],
    );
  }

  Widget _goalButton(String label, IconData icon, Color bg, VoidCallback onTap) {
    return SizedBox(
      height: 42,
      child: ElevatedButton.icon(
        onPressed: onTap,
        icon: Icon(icon, size: 16),
        label: Text(label, style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w600)),
        style: ElevatedButton.styleFrom(
          backgroundColor: bg,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
          elevation: 0,
          padding: const EdgeInsets.symmetric(horizontal: 8),
        ),
      ),
    );
  }

  Future<void> _sendGoalCommand(Map<String, dynamic> cmd) async {
    final url = _effectiveUrl;
    if (url == null) return;
    try {
      await http.post(
        Uri.parse('$url/api/goal'),
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode(cmd),
      ).timeout(const Duration(seconds: 3));
    } catch (_) {}
  }

  void _showDriveDialog() {
    final distCtrl = TextEditingController(text: '0.5');
    final speedCtrl = TextEditingController(text: '0.20');
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: AppColors.surface,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: const Text('Drive Command'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: distCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
              decoration: const InputDecoration(labelText: 'Distance (m)', hintText: 'Negative = reverse'),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: speedCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              decoration: const InputDecoration(labelText: 'Speed (m/s)'),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancel', style: TextStyle(color: AppColors.textSecondary)),
          ),
          ElevatedButton(
            onPressed: () {
              final dist = double.tryParse(distCtrl.text);
              final speed = double.tryParse(speedCtrl.text);
              if (dist == null || speed == null) return;
              Navigator.pop(ctx);
              _sendGoalCommand({'cmd': 'DRIVE', 'distance_m': dist, 'speed': speed});
            },
            child: const Text('Send'),
          ),
        ],
      ),
    );
  }

  void _showTurnDialog() {
    final angleCtrl = TextEditingController(text: '15');
    final speedCtrl = TextEditingController(text: '0.20');
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: AppColors.surface,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: const Text('Turn Command'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: angleCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
              decoration: const InputDecoration(labelText: 'Angle (degrees)', hintText: 'Positive = right, Negative = left'),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: speedCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true),
              decoration: const InputDecoration(labelText: 'Speed (m/s)'),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancel', style: TextStyle(color: AppColors.textSecondary)),
          ),
          ElevatedButton(
            onPressed: () {
              final angle = double.tryParse(angleCtrl.text);
              final speed = double.tryParse(speedCtrl.text);
              if (angle == null || speed == null) return;
              Navigator.pop(ctx);
              _sendGoalCommand({'cmd': 'TURN', 'angle_deg': angle, 'speed': speed});
            },
            child: const Text('Send'),
          ),
        ],
      ),
    );
  }

  void _showNavGoalDialog() {
    final xCtrl = TextEditingController();
    final yCtrl = TextEditingController();
    final yawCtrl = TextEditingController(text: '0');

    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: AppColors.surface,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
        title: const Text('Send Nav2 Goal'),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
              controller: xCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
              decoration: const InputDecoration(labelText: 'X (meters)'),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: yCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
              decoration: const InputDecoration(labelText: 'Y (meters)'),
            ),
            const SizedBox(height: 10),
            TextField(
              controller: yawCtrl,
              keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
              decoration: const InputDecoration(labelText: 'Yaw (radians, default 0)'),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancel', style: TextStyle(color: AppColors.textSecondary)),
          ),
          ElevatedButton(
            onPressed: () async {
              final x = double.tryParse(xCtrl.text);
              final y = double.tryParse(yCtrl.text);
              final yaw = double.tryParse(yawCtrl.text) ?? 0.0;
              if (x == null || y == null) return;
              Navigator.pop(ctx);
              try {
                await http.post(
                  Uri.parse('${_effectiveUrl}/api/nav_goal'),
                  headers: {'Content-Type': 'application/json'},
                  body: jsonEncode({'x': x, 'y': y, 'yaw': yaw}),
                ).timeout(const Duration(seconds: 3));
              } catch (_) {}
            },
            child: const Text('Send Goal'),
          ),
        ],
      ),
    );
  }
}
