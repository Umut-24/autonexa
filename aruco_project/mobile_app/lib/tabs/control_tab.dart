import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../widgets/joystick_widget.dart';
import '../widgets/connection_indicator.dart';
import '../widgets/nav_goal_dialog.dart';

/// Driving interface with virtual joystick, speed limiter, telemetry, and E-STOP.
class ControlTab extends StatefulWidget {
  const ControlTab({super.key});

  @override
  State<ControlTab> createState() => _ControlTabState();
}

class _ControlTabState extends State<ControlTab> {
  bool _fullscreenMode = false;
  double _currentX = 0;
  double _currentY = 0;

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

  @override
  void dispose() {
    if (_fullscreenMode) _exitFullscreen();
    super.dispose();
  }

  void _onJoystickMove(double x, double y) {
    setState(() { _currentX = x; _currentY = y; });
    context.read<ConnectionService>().updateJoystick(x, y);
    HapticFeedback.selectionClick();
  }

  void _onJoystickRelease(double x, double y) {
    setState(() { _currentX = 0; _currentY = 0; });
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    if (_fullscreenMode) return _buildFullscreenMode(context, colors);
    return _buildNormalMode(context, colors);
  }

  // ═══════════════════════════════════════════════════════════════════════
  //  FULLSCREEN LANDSCAPE MODE
  // ═══════════════════════════════════════════════════════════════════════
  Widget _buildFullscreenMode(BuildContext context, ResolvedColors colors) {
    final conn = context.watch<ConnectionService>();
    final telemetry = conn.telemetry;
    final screenHeight = MediaQuery.of(context).size.height;
    final joystickSize = screenHeight * 0.75;

    return Scaffold(
      backgroundColor: colors.background,
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
                        _hudItem('STEER', _currentX.toStringAsFixed(2), colors),
                        const SizedBox(height: 14),
                        _hudItem('THROTTLE', _currentY.toStringAsFixed(2), colors),
                        const SizedBox(height: 14),
                        _hudItem('L VEL', telemetry.leftVel.toStringAsFixed(2), colors),
                        const SizedBox(height: 14),
                        _hudItem('R VEL', telemetry.rightVel.toStringAsFixed(2), colors),
                        const SizedBox(height: 14),
                        _hudItem('Vx', telemetry.odomVx.toStringAsFixed(2), colors),
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
                        _hudItem('SPEED', '${(conn.speedLimit * 100).toInt()}%', colors),
                        const SizedBox(height: 16),
                        Row(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            _speedBtn(Icons.remove, () {
                              conn.setSpeedLimit(conn.speedLimit - 0.1);
                            }, colors),
                            const SizedBox(width: 8),
                            _speedBtn(Icons.add, () {
                              conn.setSpeedLimit(conn.speedLimit + 0.1);
                            }, colors),
                          ],
                        ),
                        const SizedBox(height: 24),
                        _estopButton(conn, compact: true),
                      ],
                    ),
                  ),
                ],
              ),
            ),
            // Connection badge
            Positioned(
              top: 8,
              left: 12,
              child: _connectionBadge(conn, colors),
            ),
            // Exit button
            Positioned(
              top: 8,
              right: 12,
              child: GestureDetector(
                onTap: _exitFullscreen,
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                  decoration: BoxDecoration(
                    color: colors.surfaceLight,
                    borderRadius: BorderRadius.circular(8),
                    border: Border.all(color: colors.border),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.fullscreen_exit_rounded, size: 16, color: colors.textSecondary),
                      const SizedBox(width: 4),
                      Text('EXIT',
                          style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600,
                              color: colors.textSecondary, letterSpacing: 1)),
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

  // ═══════════════════════════════════════════════════════════════════════
  //  NORMAL PORTRAIT MODE
  // ═══════════════════════════════════════════════════════════════════════
  Widget _buildNormalMode(BuildContext context, ResolvedColors colors) {
    final conn = context.watch<ConnectionService>();
    final telemetry = conn.telemetry;

    return SafeArea(
      child: Column(
        children: [
          // Header
          Padding(
            padding: const EdgeInsets.fromLTRB(20, 12, 20, 4),
            child: Row(
              children: [
                Text('Control',
                    style: TextStyle(fontSize: 22, fontWeight: FontWeight.w700,
                        color: colors.textPrimary, letterSpacing: -0.5)),
                const Spacer(),
                _connectionBadge(conn, colors),
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
                  Expanded(flex: 5, child: _buildJoystickArea(conn, colors)),
                  const SizedBox(height: 10),
                  // Speed limiter
                  _buildSpeedLimiter(conn, colors),
                  const SizedBox(height: 10),
                  // Telemetry
                  _buildTelemetryRow(telemetry, colors),
                  const SizedBox(height: 8),
                  // Odometry position
                  _buildOdometryRow(telemetry, colors),
                  const SizedBox(height: 10),
                  // E-STOP
                  _estopButton(conn, compact: false),
                  const SizedBox(height: 10),
                  // Nav2 Goal
                  if (conn.isConnected) ...[
                    _buildNavGoalButton(context, conn, colors),
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

  Widget _hudItem(String label, String value, ResolvedColors colors) {
    return Column(
      children: [
        Text(label,
            style: TextStyle(fontSize: 9, fontWeight: FontWeight.w600,
                letterSpacing: 1.2, color: colors.textSecondary)),
        const SizedBox(height: 2),
        Text(value,
            style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold,
                fontFamily: 'monospace', color: colors.textPrimary)),
      ],
    );
  }

  Widget _speedBtn(IconData icon, VoidCallback onTap, ResolvedColors colors) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: colors.surfaceLight,
          borderRadius: BorderRadius.circular(8),
          border: Border.all(color: colors.border),
        ),
        child: Icon(icon, size: 20, color: colors.textSecondary),
      ),
    );
  }

  Widget _connectionBadge(ConnectionService conn, ResolvedColors colors) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: colors.surface,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: colors.border),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          ConnectionIndicator(status: conn.status),
          const SizedBox(width: 6),
          Text(
            conn.isConnected ? 'LINKED' : 'NO LINK',
            style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600,
                color: conn.isConnected ? AppColors.success : AppColors.danger),
          ),
        ],
      ),
    );
  }

  Widget _buildJoystickArea(ConnectionService conn, ResolvedColors colors) {
    return Container(
      width: double.infinity,
      decoration: BoxDecoration(
        color: colors.surface,
        borderRadius: BorderRadius.circular(14),
        border: Border.all(color: colors.border),
      ),
      child: Column(
        children: [
          Padding(
            padding: const EdgeInsets.fromLTRB(14, 10, 10, 0),
            child: Row(
              children: [
                Text('X: ${_currentX.toStringAsFixed(2)}',
                    style: TextStyle(fontSize: 11, fontFamily: 'monospace',
                        color: colors.textSecondary)),
                const SizedBox(width: 14),
                Text('Y: ${_currentY.toStringAsFixed(2)}',
                    style: TextStyle(fontSize: 11, fontFamily: 'monospace',
                        color: colors.textSecondary)),
                const Spacer(),
                GestureDetector(
                  onTap: _enterFullscreen,
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                    decoration: BoxDecoration(
                      color: colors.accentSurface,
                      borderRadius: BorderRadius.circular(7),
                      border: Border.all(color: colors.accent.withValues(alpha: 0.3)),
                    ),
                    child: Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Icon(Icons.fullscreen_rounded, size: 14, color: colors.accent),
                        const SizedBox(width: 4),
                        Text('FULLSCREEN',
                            style: TextStyle(fontSize: 9, fontWeight: FontWeight.w700,
                                letterSpacing: 0.8, color: colors.accent)),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
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

  Widget _buildSpeedLimiter(ConnectionService conn, ResolvedColors colors) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
      decoration: BoxDecoration(
        color: colors.surface,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: colors.border),
      ),
      child: Row(
        children: [
          Text('SPEED',
              style: TextStyle(fontSize: 10, fontWeight: FontWeight.w600,
                  letterSpacing: 1.2, color: colors.textSecondary)),
          const SizedBox(width: 10),
          Expanded(
            child: SliderTheme(
              data: SliderTheme.of(context).copyWith(
                activeTrackColor: colors.accent,
                inactiveTrackColor: colors.surfaceLight,
                thumbColor: colors.accent,
                overlayColor: colors.accent.withValues(alpha: 0.15),
                trackHeight: 4,
                thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 8),
              ),
              child: Slider(
                value: conn.speedLimit,
                min: 0.1,
                max: 1.0,
                divisions: 9,
                onChanged: (v) => conn.setSpeedLimit(v),
              ),
            ),
          ),
          SizedBox(
            width: 40,
            child: Text(
              '${(conn.speedLimit * 100).toInt()}%',
              textAlign: TextAlign.right,
              style: TextStyle(fontSize: 14, fontWeight: FontWeight.w700,
                  color: colors.accent),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildTelemetryRow(telemetry, ResolvedColors colors) {
    return Row(
      children: [
        _telemetryChip('L', telemetry.leftVel.toStringAsFixed(2), Icons.rotate_left_rounded, colors),
        const SizedBox(width: 6),
        _telemetryChip('R', telemetry.rightVel.toStringAsFixed(2), Icons.rotate_right_rounded, colors),
        const SizedBox(width: 6),
        _telemetryChip('Vx', telemetry.odomVx.toStringAsFixed(2), Icons.speed_rounded, colors),
        const SizedBox(width: 6),
        _telemetryChip('Wz', telemetry.odomWz.toStringAsFixed(2), Icons.rotate_90_degrees_ccw_rounded, colors),
      ],
    );
  }

  Widget _buildOdometryRow(telemetry, ResolvedColors colors) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
      decoration: BoxDecoration(
        color: colors.surface,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: colors.border),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          _odomValue('X', '${telemetry.odomX.toStringAsFixed(3)}m', colors),
          _odomValue('Y', '${telemetry.odomY.toStringAsFixed(3)}m', colors),
          _odomValue('Yaw', '${(telemetry.odomYaw * 57.2958).toStringAsFixed(1)}°', colors),
        ],
      ),
    );
  }

  Widget _odomValue(String label, String value, ResolvedColors colors) {
    return Column(
      children: [
        Text(label, style: TextStyle(fontSize: 9, color: colors.textTertiary,
            fontWeight: FontWeight.w600, letterSpacing: 1)),
        Text(value, style: TextStyle(fontSize: 12, fontFamily: 'monospace',
            fontWeight: FontWeight.w600, color: colors.textSecondary)),
      ],
    );
  }

  Widget _telemetryChip(String label, String value, IconData icon, ResolvedColors colors) {
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 8, horizontal: 4),
        decoration: BoxDecoration(
          color: colors.surface,
          borderRadius: BorderRadius.circular(10),
          border: Border.all(color: colors.border),
        ),
        child: Column(
          children: [
            Icon(icon, size: 14, color: colors.accentDim),
            const SizedBox(height: 3),
            Text(value,
                style: TextStyle(fontSize: 13, fontWeight: FontWeight.w700,
                    fontFamily: 'monospace', color: colors.textPrimary)),
            const SizedBox(height: 1),
            Text(label, style: TextStyle(fontSize: 9, color: colors.textSecondary)),
          ],
        ),
      ),
    );
  }

  Widget _estopButton(ConnectionService conn, {required bool compact}) {
    final isActive = conn.emergencyStopped;
    return SizedBox(
      width: compact ? 100 : double.infinity,
      height: compact ? 48 : 52,
      child: ElevatedButton(
        onPressed: () {
          HapticFeedback.heavyImpact();
          if (isActive) {
            conn.releaseEmergencyStop();
          } else {
            conn.emergencyStop();
          }
        },
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

  Widget _buildNavGoalButton(BuildContext context, ConnectionService conn, ResolvedColors colors) {
    return SizedBox(
      width: double.infinity,
      height: 44,
      child: ElevatedButton.icon(
        onPressed: () => NavGoalDialog.show(context, conn),
        icon: const Icon(Icons.navigation_rounded, size: 18),
        label: const Text('Send Nav2 Goal', style: TextStyle(fontWeight: FontWeight.w600)),
        style: ElevatedButton.styleFrom(
          backgroundColor: colors.accentDim,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
        ),
      ),
    );
  }
}
