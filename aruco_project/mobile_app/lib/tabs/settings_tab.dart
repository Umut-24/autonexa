import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../services/preferences_service.dart';
import '../widgets/glass_card.dart';
import '../widgets/connection_indicator.dart';
import '../widgets/autonexa_logo.dart';
import '../widgets/calibration_dialog.dart';
import '../widgets/robot_dimensions_dialog.dart';

/// Settings page with server connection, theme toggle, control prefs, and about info.
class SettingsTab extends StatefulWidget {
  const SettingsTab({super.key});

  @override
  State<SettingsTab> createState() => _SettingsTabState();
}

class _SettingsTabState extends State<SettingsTab> {
  final TextEditingController _serverController = TextEditingController();
  bool _showServerHistory = false;
  // Local cache for the Nav2 speed slider — fetched lazily so we don't add
  // another connection round-trip on tab open if the user never touches it.
  double? _nav2MaxSpeed;
  bool _nav2SpeedFetching = false;
  // Local cache for the path-planner mode ('standard' | 'multipoint'),
  // fetched lazily on first render of the Path Planner card.
  String? _plannerMode;
  bool _plannerModeFetching = false;

  @override
  void initState() {
    super.initState();
    final prefs = context.read<PreferencesService>();
    _serverController.text = prefs.lastServer ?? '';
  }

  @override
  void dispose() {
    _serverController.dispose();
    super.dispose();
  }

  Future<void> _connect() async {
    final text = _serverController.text.trim();
    if (text.isEmpty) return;

    final conn = context.read<ConnectionService>();
    final prefs = context.read<PreferencesService>();

    final success = await conn.connect(text);
    if (success) {
      await prefs.addServer(text);
      await prefs.setLastServer(text);
      setState(() => _showServerHistory = false);
    }
  }

  Future<void> _disconnect() async {
    await context.read<ConnectionService>().disconnect();
  }

  Future<void> _ensureNav2Speed() async {
    if (_nav2MaxSpeed != null || _nav2SpeedFetching) return;
    final conn = context.read<ConnectionService>();
    if (!conn.isConnected) return;
    _nav2SpeedFetching = true;
    final v = await conn.getNav2MaxSpeed();
    if (!mounted) return;
    setState(() {
      _nav2MaxSpeed = v ?? 0.30;
      _nav2SpeedFetching = false;
    });
  }

  Future<void> _ensurePlannerMode() async {
    if (_plannerMode != null || _plannerModeFetching) return;
    final conn = context.read<ConnectionService>();
    if (!conn.isConnected) return;
    _plannerModeFetching = true;
    final m = await conn.getPlannerMode();
    if (!mounted) return;
    setState(() {
      _plannerMode = m ?? 'standard';
      _plannerModeFetching = false;
    });
  }

  @override
  Widget build(BuildContext context) {
    final conn = context.watch<ConnectionService>();
    final prefs = context.read<PreferencesService>();
    final colors = context.watch<ThemeProvider>().colors;
    final themeProvider = context.watch<ThemeProvider>();

    return Scaffold(
      backgroundColor: colors.background,
      appBar: AppBar(
        backgroundColor: colors.surface,
        title: const Text('Settings',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600)),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_rounded),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.symmetric(vertical: 8),
          children: [
            // ── Server Connection ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('SERVER CONNECTION',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Row(
                    children: [
                      Expanded(
                        child: TextField(
                          controller: _serverController,
                          decoration: InputDecoration(
                            hintText: 'e.g. 192.168.1.5:5000',
                            prefixIcon: Icon(Icons.dns_rounded,
                                size: 20, color: colors.textSecondary),
                            suffixIcon: prefs.savedServers.isNotEmpty
                                ? IconButton(
                                    icon: Icon(
                                      _showServerHistory
                                          ? Icons.expand_less
                                          : Icons.expand_more,
                                      size: 20,
                                      color: colors.textSecondary,
                                    ),
                                    onPressed: () => setState(() =>
                                        _showServerHistory =
                                            !_showServerHistory),
                                  )
                                : null,
                          ),
                        ),
                      ),
                    ],
                  ),
                  if (_showServerHistory && prefs.savedServers.isNotEmpty)
                    Container(
                      margin: const EdgeInsets.only(top: 6),
                      decoration: BoxDecoration(
                        color: colors.surfaceLight,
                        borderRadius: BorderRadius.circular(10),
                        border: Border.all(color: colors.border),
                      ),
                      child: Column(
                        children: prefs.savedServers.map((server) {
                          return ListTile(
                            dense: true,
                            title: Text(server,
                                style: const TextStyle(
                                    fontSize: 13, fontFamily: 'monospace')),
                            trailing: IconButton(
                              icon: Icon(Icons.close,
                                  size: 16, color: colors.textTertiary),
                              onPressed: () {
                                prefs.removeServer(server);
                                setState(() {});
                              },
                            ),
                            onTap: () {
                              _serverController.text = server;
                              setState(() => _showServerHistory = false);
                            },
                          );
                        }).toList(),
                      ),
                    ),
                  const SizedBox(height: 14),
                  Row(
                    children: [
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: !conn.isConnected ? _connect : null,
                          icon: const Icon(Icons.link_rounded, size: 18),
                          label: const Text('Connect'),
                          style: ElevatedButton.styleFrom(
                            backgroundColor: AppColors.brand,
                            disabledBackgroundColor: colors.surfaceLight,
                          ),
                        ),
                      ),
                      const SizedBox(width: 10),
                      Expanded(
                        child: ElevatedButton.icon(
                          onPressed: conn.isConnected ? _disconnect : null,
                          icon: const Icon(Icons.link_off_rounded, size: 18),
                          label: const Text('Disconnect'),
                          style: ElevatedButton.styleFrom(
                            backgroundColor:
                                AppColors.danger.withValues(alpha: 0.8),
                            disabledBackgroundColor: colors.surfaceLight,
                          ),
                        ),
                      ),
                    ],
                  ),
                  if (conn.isConnected) ...[
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        ConnectionIndicator(
                            status: conn.status, showLabel: true),
                        const Spacer(),
                        Text(conn.baseUrl ?? '',
                            style: TextStyle(
                                fontSize: 11,
                                fontFamily: 'monospace',
                                color: colors.textSecondary)),
                      ],
                    ),
                  ],
                ],
              ),
            ),

            // ── Appearance ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('APPEARANCE',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Row(
                    children: [
                      Icon(themeProvider.themeIcon,
                          size: 20, color: colors.textSecondary),
                      const SizedBox(width: 12),
                      Text('Theme',
                          style: TextStyle(
                              fontSize: 13, color: colors.textSecondary)),
                      const Spacer(),
                      _themeToggle(themeProvider, colors),
                    ],
                  ),
                  if (themeProvider.isDark) ...[
                    const SizedBox(height: 12),
                    _switchRow(
                        'AMOLED black mode', themeProvider.amoled, colors, (v) {
                      themeProvider.setAmoled(v);
                    }),
                  ],
                ],
              ),
            ),

            // ── Communication ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('COMMUNICATION',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  _infoRow('Protocol', 'HTTP (Flask bridge)', colors),
                  const SizedBox(height: 8),
                  _infoRow('Control rate', '20 Hz (50ms)', colors),
                  _infoRow('Telemetry rate', '5 Hz (200ms)', colors),
                  _infoRow('Status rate', '2 Hz (500ms)', colors),
                  const SizedBox(height: 12),
                  Container(
                    padding: const EdgeInsets.all(12),
                    decoration: BoxDecoration(
                      color: AppColors.info.withValues(alpha: 0.08),
                      borderRadius: BorderRadius.circular(10),
                      border: Border.all(
                          color: AppColors.info.withValues(alpha: 0.15)),
                    ),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Row(
                          children: [
                            Icon(Icons.tips_and_updates_rounded,
                                size: 14, color: AppColors.info),
                            SizedBox(width: 6),
                            Text('Upgrade tip',
                                style: TextStyle(
                                    fontSize: 11,
                                    fontWeight: FontWeight.w700,
                                    color: AppColors.info)),
                          ],
                        ),
                        const SizedBox(height: 6),
                        Text(
                          'For lower latency (5-15ms vs 20-80ms) and reduced RPi5 CPU load, '
                          'consider switching to WebSocket via Foxglove Bridge. '
                          'Install: sudo apt install ros-jazzy-foxglove-bridge',
                          style: TextStyle(
                              fontSize: 11, color: colors.textSecondary),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),

            // ── Control Preferences ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('CONTROL',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Row(
                    children: [
                      Text('Default Speed Limit',
                          style: TextStyle(
                              fontSize: 13, color: colors.textSecondary)),
                      const Spacer(),
                      Text('${(prefs.defaultSpeedLimit * 100).toInt()}%',
                          style: const TextStyle(
                              fontSize: 13,
                              fontWeight: FontWeight.w600,
                              color: AppColors.brand)),
                    ],
                  ),
                  SliderTheme(
                    data: SliderTheme.of(context).copyWith(
                      activeTrackColor: AppColors.brand,
                      inactiveTrackColor: colors.surfaceLight,
                      thumbColor: AppColors.brand,
                      trackHeight: 4,
                      thumbShape:
                          const RoundSliderThumbShape(enabledThumbRadius: 8),
                    ),
                    child: Slider(
                      value: prefs.defaultSpeedLimit,
                      min: 0.1,
                      max: 1.0,
                      divisions: 9,
                      onChanged: (v) {
                        prefs.setDefaultSpeedLimit(v);
                        setState(() {});
                      },
                    ),
                  ),
                  const SizedBox(height: 8),
                  _switchRow('Auto-reconnect', prefs.autoReconnect, colors,
                      (v) {
                    prefs.setAutoReconnect(v);
                    setState(() {});
                  }),
                  const SizedBox(height: 8),
                  _switchRow('Haptic feedback', prefs.hapticEnabled, colors,
                      (v) {
                    prefs.setHapticEnabled(v);
                    setState(() {});
                  }),
                ],
              ),
            ),

            // ── Summon Point ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('SUMMON POINT',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  if (prefs.summonPose != null) ...[
                    Row(
                      children: [
                        const Icon(Icons.hail_rounded,
                            size: 18, color: AppColors.brand),
                        const SizedBox(width: 10),
                        Expanded(
                          child: Text(
                            'X: ${prefs.summonPose!['x']!.toStringAsFixed(2)}  '
                            'Y: ${prefs.summonPose!['y']!.toStringAsFixed(2)}  '
                            'Yaw: ${prefs.summonPose!['yaw']!.toStringAsFixed(2)}',
                            style: TextStyle(
                                fontSize: 12,
                                fontFamily: 'monospace',
                                color: colors.textSecondary),
                          ),
                        ),
                        IconButton(
                          icon: const Icon(Icons.delete_outline,
                              size: 20, color: AppColors.danger),
                          onPressed: () {
                            prefs.clearSummonPose();
                            setState(() {});
                          },
                        ),
                      ],
                    ),
                  ] else
                    Text(
                      'No summon point set. Tap "Summon" on the Home tab to set one.',
                      style:
                          TextStyle(fontSize: 12, color: colors.textTertiary),
                    ),
                ],
              ),
            ),

            // ── Display ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('DISPLAY',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Row(
                    children: [
                      Text('Map Refresh Rate',
                          style: TextStyle(
                              fontSize: 13, color: colors.textSecondary)),
                      const Spacer(),
                      DropdownButton<int>(
                        value: prefs.mapRefreshMs,
                        dropdownColor: colors.surface,
                        style:
                            TextStyle(fontSize: 13, color: colors.textPrimary),
                        underline: const SizedBox.shrink(),
                        items: const [
                          DropdownMenuItem(value: 1000, child: Text('1s')),
                          DropdownMenuItem(value: 2000, child: Text('2s')),
                          DropdownMenuItem(value: 5000, child: Text('5s')),
                        ],
                        onChanged: (v) {
                          if (v != null) prefs.setMapRefreshMs(v);
                          setState(() {});
                        },
                      ),
                    ],
                  ),
                ],
              ),
            ),

            // ── Calibration ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('CALIBRATION',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Text(
                    'If a Nav2 goal makes the robot drive the wrong way, '
                    'use this to flip vx_polarity (forward/back) or '
                    'servo_polarity (all left/right). Reverse-only steering '
                    'is in Diagnostics → Param Tuner. Values persist on disk.',
                    style: TextStyle(fontSize: 12, color: colors.textSecondary),
                  ),
                  const SizedBox(height: 10),
                  SizedBox(
                    width: double.infinity,
                    child: ElevatedButton.icon(
                      onPressed: conn.isConnected
                          ? () => CalibrationDialog.show(context)
                          : null,
                      icon: const Icon(Icons.compare_arrows_rounded, size: 18),
                      label: const Text('Calibrate Direction'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: AppColors.brand,
                        disabledBackgroundColor: colors.surfaceLight,
                      ),
                    ),
                  ),
                ],
              ),
            ),

            // ── Robot Dimensions ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('ROBOT DIMENSIONS',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Text(
                    'Edit chassis length / width, LiDAR mount offset and '
                    'footprint padding. The bridge regenerates the URDF '
                    'live (RViz RobotModel + Nav2 costmap footprints '
                    'update without restart) and persists values to disk.',
                    style: TextStyle(fontSize: 12, color: colors.textSecondary),
                  ),
                  const SizedBox(height: 10),
                  SizedBox(
                    width: double.infinity,
                    child: ElevatedButton.icon(
                      onPressed: conn.isConnected
                          ? () => RobotDimensionsDialog.show(context)
                          : null,
                      icon: const Icon(Icons.crop_din_rounded, size: 18),
                      label: const Text('Edit Robot Dimensions'),
                      style: ElevatedButton.styleFrom(
                        backgroundColor: AppColors.brand,
                        disabledBackgroundColor: colors.surfaceLight,
                      ),
                    ),
                  ),
                ],
              ),
            ),

            // ── Nav2 Speed ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('NAV2 MAX SPEED',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Builder(builder: (_) {
                    if (!conn.isConnected) {
                      return Text('Connect to adjust Nav2 speed.',
                          style: TextStyle(
                              fontSize: 12, color: colors.textTertiary));
                    }
                    _ensureNav2Speed();
                    final value = _nav2MaxSpeed ?? 0.30;
                    return Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Row(children: [
                          Text('target speed',
                              style: TextStyle(
                                  fontSize: 13, color: colors.textSecondary)),
                          const Spacer(),
                          Text('${value.toStringAsFixed(2)} m/s',
                              style: const TextStyle(
                                  fontSize: 13,
                                  fontWeight: FontWeight.w700,
                                  color: AppColors.brand)),
                        ]),
                        SliderTheme(
                          data: SliderTheme.of(context).copyWith(
                            activeTrackColor: AppColors.brand,
                            inactiveTrackColor: colors.surfaceLight,
                            thumbColor: AppColors.brand,
                            trackHeight: 4,
                          ),
                          child: Slider(
                            value: value.clamp(0.05, 0.50),
                            min: 0.05,
                            max: 0.50,
                            divisions: 45,
                            onChanged: (v) => setState(() => _nav2MaxSpeed = v),
                            onChangeEnd: (v) async {
                              final messenger = ScaffoldMessenger.of(context);
                              final ok = await conn.setNav2MaxSpeed(v);
                              if (!ok && mounted) {
                                messenger.showSnackBar(
                                  const SnackBar(
                                      content:
                                          Text('Failed to update Nav2 speed')),
                                );
                              }
                            },
                          ),
                        ),
                        Text(
                          'Applies to RPP FollowPath.desired_linear_vel and velocity_smoother in lockstep. '
                          'Persists across relaunches.',
                          style: TextStyle(
                              fontSize: 11, color: colors.textTertiary),
                        ),
                      ],
                    );
                  }),
                ],
              ),
            ),

            // ── Path Planner ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('PATH PLANNER',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Builder(builder: (_) {
                    if (!conn.isConnected) {
                      return Text('Connect to choose the path planner.',
                          style: TextStyle(
                              fontSize: 12, color: colors.textTertiary));
                    }
                    _ensurePlannerMode();
                    final multipoint =
                        (_plannerMode ?? 'standard') == 'multipoint';
                    return Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        _switchRow('Multi-point planner', multipoint, colors,
                            (v) async {
                          final messenger = ScaffoldMessenger.of(context);
                          final mode = v ? 'multipoint' : 'standard';
                          setState(() => _plannerMode = mode);
                          final ok = await conn.setPlannerMode(mode);
                          if (!ok && mounted) {
                            setState(() =>
                                _plannerMode = v ? 'standard' : 'multipoint');
                            messenger.showSnackBar(const SnackBar(
                                content:
                                    Text('Failed to update planner mode')));
                          }
                        }),
                        const SizedBox(height: 6),
                        Text(
                          'Standard: a single Nav2 goal. Multi-point: if a '
                          'goal fails to plan, the robot is routed via an '
                          'intermediate waypoint and then on to the final '
                          'goal. Persists across relaunches.',
                          style: TextStyle(
                              fontSize: 11, color: colors.textTertiary),
                        ),
                      ],
                    );
                  }),
                ],
              ),
            ),

            // ── About ──
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('ABOUT',
                      style: TextStyle(
                          fontSize: 11,
                          fontWeight: FontWeight.w700,
                          letterSpacing: 1.2,
                          color: colors.textTertiary)),
                  const SizedBox(height: 14),
                  Row(
                    children: [
                      const AutoNexaLogo(size: 40),
                      const SizedBox(width: 14),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text('AutoNexa Mobile Controller',
                                style: TextStyle(
                                    fontSize: 15,
                                    fontWeight: FontWeight.w600,
                                    color: colors.textPrimary)),
                            const SizedBox(height: 4),
                            Text('Version 2.1.0',
                                style: TextStyle(
                                    fontSize: 12, color: colors.textSecondary)),
                          ],
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 12),
                  Text(
                    'Intelligent Parking and Vehicle Recall System. '
                    'Control your autonomous parking robot via the RPi5 ROS2 bridge. '
                    'Features: joystick driving, SLAM map with path trail, '
                    'ArUco parking spot detection, autonomous parking, '
                    'vehicle summoning, mission planning, battery monitoring, '
                    'obstacle proximity alerts, and multi-theme UI.',
                    style: TextStyle(fontSize: 12, color: colors.textTertiary),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 80),
          ],
        ),
      ),
    );
  }

  Widget _themeToggle(ThemeProvider themeProvider, ResolvedColors colors) {
    return Container(
      decoration: BoxDecoration(
        color: colors.surfaceLight,
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: colors.border),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          _themeBtn(Icons.dark_mode_rounded, 'Dark',
              themeProvider.isDark && !themeProvider.amoled, colors, () {
            themeProvider.setAmoled(false);
            themeProvider.setMode(ThemeMode.dark);
          }),
          _themeBtn(
              Icons.light_mode_rounded, 'Light', !themeProvider.isDark, colors,
              () {
            themeProvider.setMode(ThemeMode.light);
          }),
        ],
      ),
    );
  }

  Widget _themeBtn(IconData icon, String label, bool active,
      ResolvedColors colors, VoidCallback onTap) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
        decoration: BoxDecoration(
          color: active ? AppColors.brandSurface : Colors.transparent,
          borderRadius: BorderRadius.circular(8),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(icon,
                size: 14,
                color: active ? AppColors.brand : colors.textTertiary),
            const SizedBox(width: 4),
            Text(label,
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: active ? AppColors.brand : colors.textTertiary,
                )),
          ],
        ),
      ),
    );
  }

  Widget _infoRow(String label, String value, ResolvedColors colors) {
    return Row(
      children: [
        Text(label,
            style: TextStyle(fontSize: 12, color: colors.textSecondary)),
        const Spacer(),
        Text(value,
            style: TextStyle(
                fontSize: 12,
                fontFamily: 'monospace',
                fontWeight: FontWeight.w600,
                color: colors.textPrimary)),
      ],
    );
  }

  Widget _switchRow(String label, bool value, ResolvedColors colors,
      ValueChanged<bool> onChanged) {
    return Row(
      children: [
        Text(label,
            style: TextStyle(fontSize: 13, color: colors.textSecondary)),
        const Spacer(),
        Switch(
          value: value,
          onChanged: onChanged,
          activeTrackColor: AppColors.brand,
        ),
      ],
    );
  }
}
