import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../services/preferences_service.dart';
import '../widgets/glass_card.dart';
import '../widgets/stat_tile.dart';
import '../widgets/marker_chip.dart';
import '../widgets/connection_indicator.dart';
import '../widgets/nav_goal_dialog.dart';
import '../widgets/autonexa_logo.dart';
import '../widgets/battery_indicator.dart';
import '../widgets/obstacle_alert.dart';

class HomeTab extends StatelessWidget {
  const HomeTab({super.key});

  @override
  Widget build(BuildContext context) {
    final conn = context.watch<ConnectionService>();
    final colors = context.watch<ThemeProvider>().colors;
    final status = conn.robotStatus;
    final telemetry = conn.telemetry;
    final markers = status.markers.values.toList();

    return SafeArea(
      child: Column(
        children: [
          _header(context, conn, colors),
          Expanded(
            child: ListView(
              padding: const EdgeInsets.only(bottom: 80),
              children: [
                _connectionCard(context, conn, colors),

                // Obstacle proximity alert
                if (conn.isConnected && conn.telemetry.obstacleWarning)
                  ObstacleAlert(
                    distanceM: conn.telemetry.minObstacleDistance,
                    critical: conn.telemetry.obstacleCritical,
                  ),

                // Battery warning
                if (conn.isConnected &&
                    conn.telemetry.estimatedPercent >= 0 &&
                    conn.telemetry.estimatedPercent <= 15)
                  Container(
                    margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                    padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
                    decoration: BoxDecoration(
                      color: AppColors.warning.withValues(alpha: 0.12),
                      borderRadius: BorderRadius.circular(12),
                      border: Border.all(color: AppColors.warning.withValues(alpha: 0.3)),
                    ),
                    child: Row(
                      children: [
                        const Icon(Icons.battery_alert_rounded, color: AppColors.warning, size: 20),
                        const SizedBox(width: 10),
                        Expanded(
                          child: Text(
                            'Low battery: ${conn.telemetry.estimatedPercent}%  —  '
                            'Consider charging soon',
                            style: const TextStyle(fontSize: 12, color: AppColors.warning),
                          ),
                        ),
                      ],
                    ),
                  ),

                const SizedBox(height: 4),

                _sectionLabel('ROBOT STATUS', colors),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Row(
                    children: [
                      Expanded(child: StatTile(
                        label: 'Position',
                        value: '${status.pose.x.toStringAsFixed(2)}, ${status.pose.y.toStringAsFixed(2)}',
                        icon: Icons.my_location_rounded,
                        iconColor: AppColors.info,
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: StatTile(
                        label: 'Velocity',
                        value: '${telemetry.odomVx.toStringAsFixed(2)} m/s',
                        icon: Icons.speed_rounded,
                        iconColor: AppColors.brand,
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: StatTile(
                        label: 'Heading',
                        value: '${(status.pose.yaw * 57.2958).toStringAsFixed(0)}\u00B0',
                        icon: Icons.explore_rounded,
                        iconColor: AppColors.warning,
                      )),
                    ],
                  ),
                ),
                const SizedBox(height: 12),

                _sectionLabel('MOTOR TELEMETRY', colors),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Row(
                    children: [
                      Expanded(child: StatTile(
                        label: 'Left Wheel',
                        value: telemetry.leftVel.toStringAsFixed(2),
                        icon: Icons.rotate_left_rounded,
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: StatTile(
                        label: 'Right Wheel',
                        value: telemetry.rightVel.toStringAsFixed(2),
                        icon: Icons.rotate_right_rounded,
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: StatTile(
                        label: 'Steering',
                        value: '${(telemetry.steerPos * 57.2958).toStringAsFixed(0)}\u00B0',
                        icon: Icons.swap_horiz_rounded,
                      )),
                    ],
                  ),
                ),
                const SizedBox(height: 12),

                if (markers.isNotEmpty) ...[
                  _sectionLabel('DETECTED MARKERS', colors),
                  SizedBox(
                    height: 68,
                    child: ListView.separated(
                      scrollDirection: Axis.horizontal,
                      padding: const EdgeInsets.symmetric(horizontal: 16),
                      itemCount: markers.length,
                      separatorBuilder: (_, __) => const SizedBox(width: 8),
                      itemBuilder: (_, i) => MarkerChip(
                        marker: markers[i],
                        onTap: () => NavGoalDialog.show(context, conn),
                      ),
                    ),
                  ),
                  const SizedBox(height: 12),
                ],

                _sectionLabel('QUICK ACTIONS', colors),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Row(
                    children: [
                      Expanded(child: _actionButton(
                        colors, 'E-STOP',
                        Icons.stop_rounded,
                        conn.emergencyStopped ? AppColors.warning : AppColors.danger,
                        () {
                          if (conn.emergencyStopped) {
                            conn.releaseEmergencyStop();
                          } else {
                            conn.emergencyStop();
                          }
                        },
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: _actionButton(
                        colors, 'Go Home',
                        Icons.home_rounded,
                        AppColors.info,
                        conn.isConnected ? () => conn.sendNavGoal(0, 0, 0) : null,
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: _actionButton(
                        colors, 'Summon',
                        Icons.hail_rounded,
                        AppColors.brand,
                        conn.isConnected ? () => _summonVehicle(context, conn, colors) : null,
                      )),
                    ],
                  ),
                ),
                const SizedBox(height: 8),
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  child: Row(
                    children: [
                      Expanded(child: _actionButton(
                        colors, 'Nav Goal',
                        Icons.navigation_rounded,
                        AppColors.info,
                        conn.isConnected
                            ? () => NavGoalDialog.show(context, conn)
                            : null,
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: _actionButton(
                        colors, 'Pick Park Spot',
                        Icons.local_parking_rounded,
                        AppColors.success,
                        conn.isConnected
                            ? () => _chooseParkSpot(context, conn, colors)
                            : null,
                      )),
                      const SizedBox(width: 8),
                      Expanded(child: _actionButton(
                        colors, 'Full Stop',
                        Icons.pan_tool_rounded,
                        colors.textSecondary,
                        conn.isConnected
                            ? () { conn.updateJoystick(0, 0); }
                            : null,
                      )),
                    ],
                  ),
                ),

                const SizedBox(height: 16),
                _sectionLabel('SYSTEM', colors),
                GlassCard(
                  child: Column(
                    children: [
                      _infoRow('Pose Source', status.pose.source, colors),
                      _infoRow('Scan Points', '${status.scan.count}', colors),
                      _infoRow('Map', status.mapInfo != null
                          ? '${status.mapInfo!.width}x${status.mapInfo!.height}'
                          : 'No map', colors),
                      _infoRow('Latency', '${conn.latencyMs}ms', colors),
                      _infoRow('Commands', '${conn.commandsSent}', colors),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Future<void> _summonVehicle(BuildContext context, ConnectionService conn, ResolvedColors colors) async {
    // Prefer the bridge-side waypoint store (shared with Manual Spots tab,
    // map overlay, etc.) — that way "summon" is the same point regardless of
    // whether you defined it from this button or from Parking → Manual Spots.
    // Fall back to the legacy phone-only prefs.summonPose so existing users
    // don't lose their saved point.
    final bridgeWps = await conn.listNamedWaypoints();
    final bridgeSummon = bridgeWps.where((w) =>
        w.name == 'summon' || w.kind == 'summon').toList();

    if (bridgeSummon.isNotEmpty) {
      final wp = bridgeSummon.first;
      final ok = await conn.navigateToNamedWaypoint(wp.name);
      if (!context.mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        content: Text(ok
            ? 'Summoning to "${wp.name}" (${wp.x.toStringAsFixed(2)}, ${wp.y.toStringAsFixed(2)})'
            : 'Summon failed'),
      ));
      return;
    }

    final prefs = context.read<PreferencesService>();
    final pose = prefs.summonPose;

    if (pose == null) {
      final status = conn.robotStatus;
      if (!context.mounted) return;
      showDialog(
        context: context,
        builder: (ctx) => AlertDialog(
          title: const Text('Set Summon Point'),
          content: Text(
            'Save current position (${status.pose.x.toStringAsFixed(2)}, '
            '${status.pose.y.toStringAsFixed(2)}) as the summon destination?\n\n'
            'Stored on the robot so it appears on the map and can be used '
            'from any phone.',
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(ctx),
              child: Text('Cancel', style: TextStyle(color: colors.textSecondary)),
            ),
            ElevatedButton(
              onPressed: () async {
                Navigator.pop(ctx);
                // Save to the bridge so subsequent summons / the map overlay
                // see it. Mirror to phone prefs as a redundant local backup.
                final wp = await conn.saveNamedWaypoint(
                  name: 'summon',
                  kind: 'summon',
                  x: status.pose.x,
                  y: status.pose.y,
                  yaw: status.pose.yaw,
                );
                prefs.setSummonPose(status.pose.x, status.pose.y, status.pose.yaw);
                if (!context.mounted) return;
                ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                  content: Text(wp != null
                      ? 'Summon point saved'
                      : 'Saved locally only (bridge offline)'),
                ));
              },
              child: const Text('Save'),
            ),
          ],
        ),
      );
    } else {
      // Legacy fallback: prefs.summonPose exists but no bridge waypoint —
      // best-effort upload it so it shows on the map next time, then navigate.
      // ignore: unawaited_futures
      conn.saveNamedWaypoint(
        name: 'summon', kind: 'summon',
        x: pose['x']!, y: pose['y']!, yaw: pose['yaw']!,
      );
      conn.sendNavGoal(pose['x']!, pose['y']!, pose['yaw']!);
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text(
          'Summoning to (${pose['x']!.toStringAsFixed(2)}, ${pose['y']!.toStringAsFixed(2)})'
        )),
      );
    }
  }

  Future<void> _chooseParkSpot(
      BuildContext context, ConnectionService conn, ResolvedColors colors) async {
    // Use the cached waypoint list (refreshed every 5 s by ConnectionService).
    // Filter to kind == 'park'. Empty -> nudge the user to add one in Parking.
    final spots = conn.namedWaypoints.where((w) => w.kind == 'park').toList();
    if (spots.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
        content: Text('No park spots saved — add one in the Parking tab.'),
      ));
      return;
    }
    final picked = await showModalBottomSheet<NamedWaypoint>(
      context: context,
      backgroundColor: colors.surface,
      isScrollControlled: true,
      builder: (ctx) => SafeArea(
        child: Padding(
          padding: const EdgeInsets.fromLTRB(16, 16, 16, 8),
          child: Column(mainAxisSize: MainAxisSize.min, children: [
            Row(children: [
              Icon(Icons.local_parking_rounded, color: AppColors.success),
              const SizedBox(width: 8),
              const Text('Choose park spot',
                  style: TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
            ]),
            const SizedBox(height: 12),
            Flexible(
              child: ListView.separated(
                shrinkWrap: true,
                itemCount: spots.length,
                separatorBuilder: (_, __) => const SizedBox(height: 6),
                itemBuilder: (_, i) {
                  final wp = spots[i];
                  return ListTile(
                    leading: Icon(Icons.location_on_rounded,
                        color: wp.stale ? colors.textTertiary : AppColors.success),
                    title: Text(wp.name,
                        style: const TextStyle(fontWeight: FontWeight.w600)),
                    subtitle: Text(
                      '(${wp.x.toStringAsFixed(2)}, ${wp.y.toStringAsFixed(2)})'
                      '  yaw ${(wp.yaw * 57.2958).toStringAsFixed(0)}°'
                      '${wp.stale ? '  • stale (map changed)' : ''}',
                      style: TextStyle(fontSize: 11, fontFamily: 'monospace'),
                    ),
                    enabled: !wp.stale,
                    onTap: wp.stale ? null : () => Navigator.pop(ctx, wp),
                  );
                },
              ),
            ),
            const SizedBox(height: 4),
            TextButton(
              onPressed: () => Navigator.pop(ctx),
              child: const Text('Cancel'),
            ),
          ]),
        ),
      ),
    );
    if (picked == null) return;
    final ok = await conn.navigateToNamedWaypoint(picked.name);
    if (!context.mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      content: Text(ok ? 'Parking at "${picked.name}"' : 'Navigate failed'),
    ));
  }

  Widget _header(BuildContext context, ConnectionService conn, ResolvedColors colors) {
    final themeProvider = context.watch<ThemeProvider>();

    return Padding(
      padding: const EdgeInsets.fromLTRB(20, 12, 20, 8),
      child: Row(
        children: [
          const AutoNexaLogo(size: 32),
          const SizedBox(width: 10),
          Text(
            'AutoNexa',
            style: TextStyle(
              fontSize: 22,
              fontWeight: FontWeight.w700,
              color: colors.textPrimary,
              letterSpacing: -0.5,
            ),
          ),
          const Spacer(),
          // Battery indicator (compact)
          if (conn.isConnected && conn.telemetry.estimatedPercent >= 0) ...[
            BatteryIndicator(
              percent: conn.telemetry.estimatedPercent,
              compact: true,
            ),
            const SizedBox(width: 10),
          ],
          // Theme toggle button
          GestureDetector(
            onTap: () => themeProvider.cycleTheme(),
            child: Container(
              padding: const EdgeInsets.all(6),
              decoration: BoxDecoration(
                color: colors.surfaceLight,
                borderRadius: BorderRadius.circular(8),
              ),
              child: Icon(
                themeProvider.themeIcon,
                size: 18,
                color: colors.textSecondary,
              ),
            ),
          ),
          const SizedBox(width: 8),
          if (conn.isConnected)
            Container(
              padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
              decoration: BoxDecoration(
                color: colors.surfaceLight,
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                '${conn.latencyMs}ms',
                style: TextStyle(
                  fontSize: 11,
                  fontFamily: 'monospace',
                  fontWeight: FontWeight.w600,
                  color: colors.textSecondary,
                ),
              ),
            ),
          const SizedBox(width: 8),
          ConnectionIndicator(status: conn.status),
        ],
      ),
    );
  }

  Widget _connectionCard(BuildContext context, ConnectionService conn, ResolvedColors colors) {
    return GlassCard(
      child: Row(
        children: [
          Container(
            width: 44,
            height: 44,
            decoration: BoxDecoration(
              color: conn.isConnected
                  ? AppColors.success.withValues(alpha: 0.15)
                  : colors.surfaceLight,
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(
              conn.isConnected ? Icons.link_rounded : Icons.link_off_rounded,
              color: conn.isConnected ? AppColors.success : colors.textTertiary,
              size: 22,
            ),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  conn.isConnected ? 'Connected' : 'Not Connected',
                  style: TextStyle(
                    fontSize: 14,
                    fontWeight: FontWeight.w600,
                    color: conn.isConnected ? AppColors.success : colors.textSecondary,
                  ),
                ),
                if (conn.baseUrl != null)
                  Text(
                    conn.baseUrl!,
                    style: TextStyle(fontSize: 12, color: colors.textSecondary),
                    overflow: TextOverflow.ellipsis,
                  )
                else
                  Text(
                    'Go to Settings to connect',
                    style: TextStyle(fontSize: 12, color: colors.textTertiary),
                  ),
              ],
            ),
          ),
          if (conn.connectedSince != null)
            Text(
              _formatUptime(DateTime.now().difference(conn.connectedSince!)),
              style: TextStyle(
                fontSize: 11,
                fontFamily: 'monospace',
                color: colors.textTertiary,
              ),
            ),
        ],
      ),
    );
  }

  Widget _sectionLabel(String title, ResolvedColors colors) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(20, 4, 20, 8),
      child: Text(
        title,
        style: TextStyle(
          fontSize: 11,
          fontWeight: FontWeight.w700,
          letterSpacing: 1.2,
          color: colors.textTertiary,
        ),
      ),
    );
  }

  Widget _actionButton(
    ResolvedColors colors,
    String label,
    IconData icon,
    Color color,
    VoidCallback? onTap,
  ) {
    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(vertical: 14),
        decoration: BoxDecoration(
          color: onTap != null ? color.withValues(alpha: 0.15) : colors.surfaceLight,
          borderRadius: BorderRadius.circular(14),
          border: Border.all(
            color: onTap != null ? color.withValues(alpha: 0.3) : colors.border,
          ),
        ),
        child: Column(
          children: [
            Icon(icon, size: 24, color: onTap != null ? color : colors.textTertiary),
            const SizedBox(height: 6),
            Text(
              label,
              style: TextStyle(
                fontSize: 11,
                fontWeight: FontWeight.w600,
                color: onTap != null ? color : colors.textTertiary,
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _infoRow(String label, String value, ResolvedColors colors) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 5),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: TextStyle(fontSize: 13, color: colors.textSecondary)),
          Text(
            value,
            style: TextStyle(
              fontSize: 13,
              fontWeight: FontWeight.w600,
              fontFamily: 'monospace',
              color: colors.textPrimary,
            ),
          ),
        ],
      ),
    );
  }

  static String _formatUptime(Duration d) {
    if (d.inHours > 0) return '${d.inHours}h ${d.inMinutes % 60}m';
    if (d.inMinutes > 0) return '${d.inMinutes}m ${d.inSeconds % 60}s';
    return '${d.inSeconds}s';
  }
}
