import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../services/preferences_service.dart';
import '../services/event_logger.dart';
import '../models/robot_state.dart';
import '../models/mission.dart';
import '../widgets/glass_card.dart';
import '../widgets/nav_goal_dialog.dart';

/// Parking spot manager with detected markers and waypoint mission planner.
class ParkingTab extends StatefulWidget {
  const ParkingTab({super.key});

  @override
  State<ParkingTab> createState() => _ParkingTabState();
}

class _ParkingTabState extends State<ParkingTab>
    with SingleTickerProviderStateMixin {
  late TabController _tabController;

  // Mission state
  final List<Waypoint> _waypoints = [];
  MissionExecutionState _missionState = MissionExecutionState.idle;
  int _currentWaypointIndex = 0;
  Timer? _missionTimer;

  // Manual waypoints loaded from the bridge.
  List<NamedWaypoint> _manualWaypoints = [];
  bool _manualLoading = false;
  String _lastFingerprint = '';

  ConnectionService? _conn;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 3, vsync: this);
    WidgetsBinding.instance
        .addPostFrameCallback((_) => _refreshManualWaypoints());
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final conn = context.read<ConnectionService>();
    if (!identical(conn, _conn)) {
      _conn?.removeListener(_onConnChange);
      _conn = conn;
      _conn!.addListener(_onConnChange);
    }
  }

  /// Surface the bridge's short-lived AI-staging outcome as a toast (the user
  /// asked to be notified when AI staging falls back to the deterministic pose).
  void _onConnChange() {
    final notice = _conn?.consumeParkAiNotice() ?? '';
    if (notice == 'fallback' && mounted) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        if (!mounted) return;
        ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
            content: Text('AI staging unavailable — used default staging')));
      });
    }
  }

  @override
  void dispose() {
    _conn?.removeListener(_onConnChange);
    _tabController.dispose();
    _missionTimer?.cancel();
    super.dispose();
  }

  Future<void> _refreshManualWaypoints() async {
    final conn = context.read<ConnectionService>();
    if (!conn.isConnected) return;
    setState(() => _manualLoading = true);
    final wps = await conn.listNamedWaypoints();
    if (!mounted) return;
    setState(() {
      _manualWaypoints = wps;
      _manualLoading = false;
      _lastFingerprint = conn.mapFingerprint;
    });
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;

    return SafeArea(
      child: Column(
        children: [
          // Header
          Padding(
            padding: const EdgeInsets.fromLTRB(20, 12, 20, 8),
            child: Align(
              alignment: Alignment.centerLeft,
              child: Text('Parking',
                  style: TextStyle(
                      fontSize: 22,
                      fontWeight: FontWeight.w700,
                      color: colors.textPrimary,
                      letterSpacing: -0.5)),
            ),
          ),

          // AI staging toggle (applies to the two-leg park staging selection).
          _buildAiStagingBar(colors),

          // Tab bar
          Container(
            margin: const EdgeInsets.symmetric(horizontal: 16),
            decoration: BoxDecoration(
              color: colors.surface,
              borderRadius: BorderRadius.circular(12),
            ),
            child: TabBar(
              controller: _tabController,
              indicator: BoxDecoration(
                color: colors.accentSurface,
                borderRadius: BorderRadius.circular(10),
              ),
              indicatorSize: TabBarIndicatorSize.tab,
              dividerColor: Colors.transparent,
              labelColor: colors.accent,
              unselectedLabelColor: colors.textSecondary,
              labelStyle:
                  const TextStyle(fontSize: 13, fontWeight: FontWeight.w600),
              tabs: const [
                Tab(text: 'Parking Spots'),
                Tab(text: 'Manual Spots'),
                Tab(text: 'Missions'),
              ],
            ),
          ),
          const SizedBox(height: 8),

          Expanded(
            child: TabBarView(
              controller: _tabController,
              children: [
                _buildParkingSpotsView(colors),
                _buildManualSpotsView(colors),
                _buildMissionsView(colors),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  AI STAGING TOGGLE
  // ═══════════════════════════════════════════════════════════════════════════

  Widget _buildAiStagingBar(ResolvedColors colors) {
    final conn = context.watch<ConnectionService>();
    final on = conn.parkAiMode;
    return Container(
      margin: const EdgeInsets.fromLTRB(16, 0, 16, 8),
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: colors.surface,
        borderRadius: BorderRadius.circular(12),
        border: Border.all(
            color: on ? colors.accent.withValues(alpha: 0.6) : colors.border),
      ),
      child: Row(children: [
        Icon(on ? Icons.auto_awesome : Icons.auto_awesome_outlined,
            size: 18, color: on ? colors.accent : colors.textSecondary),
        const SizedBox(width: 8),
        Text('AI STAGING',
            style: TextStyle(
                fontSize: 10,
                fontWeight: FontWeight.w700,
                letterSpacing: 1.2,
                color: colors.textSecondary)),
        const SizedBox(width: 6),
        Expanded(
          child: Text(
            on
                ? 'LLM picks the park staging pose (validated, falls back).'
                : 'Deterministic staging pose (default).',
            style: TextStyle(fontSize: 11, color: colors.textSecondary),
            overflow: TextOverflow.ellipsis,
          ),
        ),
        Switch.adaptive(
          value: on,
          onChanged: conn.isConnected
              ? (v) => conn.setParkAiMode(v)
              : null,
        ),
      ]),
    );
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  PARKING SPOTS TAB
  // ═══════════════════════════════════════════════════════════════════════════

  Widget _buildParkingSpotsView(ResolvedColors colors) {
    final conn = context.watch<ConnectionService>();
    final markers = conn.robotStatus.markers.values.toList()
      ..sort((a, b) => a.id.compareTo(b.id));

    if (!conn.isConnected) {
      return _emptyState(
        Icons.link_off_rounded,
        'Not Connected',
        'Connect to the robot to see detected parking spots',
        colors,
      );
    }

    if (markers.isEmpty) {
      return _emptyState(
        Icons.qr_code_2_rounded,
        'No Markers Detected',
        'Ensure the camera has line-of-sight to ArUco markers',
        colors,
      );
    }

    return ListView.builder(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
      itemCount: markers.length,
      itemBuilder: (_, i) => _markerCard(markers[i], conn, colors),
    );
  }

  Widget _markerCard(
      MarkerInfo marker, ConnectionService conn, ResolvedColors colors) {
    final Color statusColor;
    final String statusLabel;
    switch (marker.status) {
      case MarkerStatus.live:
        statusColor = AppColors.success;
        statusLabel = 'Live';
        break;
      case MarkerStatus.stale:
        statusColor = AppColors.warning;
        statusLabel = 'Stale';
        break;
      case MarkerStatus.lost:
        statusColor = colors.textTertiary;
        statusLabel = 'Lost';
        break;
    }

    return GlassCard(
      margin: const EdgeInsets.only(bottom: 8),
      child: Row(
        children: [
          // ID badge
          Container(
            width: 48,
            height: 48,
            decoration: BoxDecoration(
              color: colors.accentDim.withValues(alpha: 0.3),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Center(
              child: Text(
                '${marker.id}',
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.w800,
                  color: colors.textPrimary,
                ),
              ),
            ),
          ),
          const SizedBox(width: 14),
          // Info
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(
                  children: [
                    Text('Spot #${marker.id}',
                        style: const TextStyle(
                            fontSize: 14, fontWeight: FontWeight.w600)),
                    const SizedBox(width: 8),
                    Container(
                      padding: const EdgeInsets.symmetric(
                          horizontal: 8, vertical: 2),
                      decoration: BoxDecoration(
                        color: statusColor.withValues(alpha: 0.15),
                        borderRadius: BorderRadius.circular(6),
                      ),
                      child: Text(statusLabel,
                          style: TextStyle(
                              fontSize: 10,
                              fontWeight: FontWeight.w600,
                              color: statusColor)),
                    ),
                  ],
                ),
                const SizedBox(height: 4),
                Text(
                  '${marker.distance.toStringAsFixed(2)}m away  ·  ${marker.bearing.toStringAsFixed(0)}° bearing',
                  style: TextStyle(
                      fontSize: 12,
                      fontFamily: 'monospace',
                      color: colors.textSecondary),
                ),
              ],
            ),
          ),
          // Navigate button
          GestureDetector(
            onTap: () => NavGoalDialog.show(context, conn),
            child: Container(
              padding: const EdgeInsets.all(10),
              decoration: BoxDecoration(
                color: AppColors.info.withValues(alpha: 0.15),
                borderRadius: BorderRadius.circular(10),
              ),
              child: const Icon(Icons.navigation_rounded,
                  size: 20, color: AppColors.info),
            ),
          ),
        ],
      ),
    );
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  MANUAL SPOTS TAB — user-defined waypoints with no ArUco needed.
  // ═══════════════════════════════════════════════════════════════════════════

  Widget _buildManualSpotsView(ResolvedColors colors) {
    final conn = context.watch<ConnectionService>();
    if (!conn.isConnected) {
      return _emptyState(Icons.link_off_rounded, 'Not Connected',
          'Connect to manage manual waypoints.', colors);
    }
    // If the bridge fingerprint changed since we loaded, refresh on next frame
    // so stale flags update without manual pull-to-refresh.
    if (_lastFingerprint.isNotEmpty &&
        conn.mapFingerprint.isNotEmpty &&
        _lastFingerprint != conn.mapFingerprint) {
      WidgetsBinding.instance
          .addPostFrameCallback((_) => _refreshManualWaypoints());
    }
    return RefreshIndicator(
      onRefresh: _refreshManualWaypoints,
      child: ListView(
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
        children: [
          GlassCard(
            child:
                Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              Text('SAVE CURRENT POSE',
                  style: TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w700,
                      letterSpacing: 1.2,
                      color: colors.textTertiary)),
              const SizedBox(height: 8),
              Text(
                'Drive the robot to the spot, then tap a kind to save its current map pose '
                '(${conn.robotStatus.pose.x.toStringAsFixed(2)}, '
                '${conn.robotStatus.pose.y.toStringAsFixed(2)}, '
                'yaw ${(conn.robotStatus.pose.yaw * 57.2958).toStringAsFixed(0)}°).',
                style: TextStyle(fontSize: 12, color: colors.textSecondary),
              ),
              const SizedBox(height: 10),
              Row(children: [
                Expanded(
                    child: _saveKindBtn('park', Icons.local_parking_rounded,
                        AppColors.brand, conn)),
                const SizedBox(width: 6),
                Expanded(
                    child: _saveKindBtn(
                        'summon', Icons.hail_rounded, AppColors.info, conn)),
                const SizedBox(width: 6),
                Expanded(
                    child: _saveKindBtn(
                        'home', Icons.home_rounded, AppColors.success, conn)),
              ]),
            ]),
          ),
          if (_manualLoading)
            const Padding(
              padding: EdgeInsets.all(16),
              child: Center(child: CircularProgressIndicator()),
            )
          else if (_manualWaypoints.isEmpty)
            Padding(
              padding: const EdgeInsets.symmetric(vertical: 32),
              child: Center(
                child: Text('No manual spots yet. Save one above.',
                    style: TextStyle(fontSize: 13, color: colors.textTertiary)),
              ),
            )
          else
            ..._manualWaypoints.map((wp) => _manualWpCard(wp, conn, colors)),
        ],
      ),
    );
  }

  Widget _saveKindBtn(
      String kind, IconData icon, Color color, ConnectionService conn) {
    return ElevatedButton.icon(
      icon: Icon(icon, size: 16),
      label: Text(kind, style: const TextStyle(fontSize: 12)),
      style: ElevatedButton.styleFrom(
        backgroundColor: color.withValues(alpha: 0.85),
        padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 10),
      ),
      onPressed: () => _promptAndSave(kind, conn),
    );
  }

  Future<void> _promptAndSave(String kind, ConnectionService conn) async {
    final ctrl = TextEditingController(
        text: '${kind}_${DateTime.now().millisecondsSinceEpoch % 10000}');
    final ok = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: Text('Name this $kind spot'),
        content: TextField(
          controller: ctrl,
          autofocus: true,
          decoration: const InputDecoration(labelText: 'Name'),
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx, false),
              child: const Text('Cancel')),
          ElevatedButton(
              onPressed: () => Navigator.pop(ctx, true),
              child: const Text('Save')),
        ],
      ),
    );
    if (ok != true) return;
    final pose = conn.robotStatus.pose;
    final wp = await conn.saveNamedWaypoint(
        name: ctrl.text.trim(),
        kind: kind,
        x: pose.x,
        y: pose.y,
        yaw: pose.yaw);
    if (wp != null) await _refreshManualWaypoints();
  }

  Widget _manualWpCard(
      NamedWaypoint wp, ConnectionService conn, ResolvedColors colors) {
    Color kindColor;
    switch (wp.kind) {
      case 'park':
        kindColor = AppColors.brand;
        break;
      case 'summon':
        kindColor = AppColors.info;
        break;
      case 'home':
        kindColor = AppColors.success;
        break;
      default:
        kindColor = colors.textTertiary;
    }
    return GlassCard(
      margin: const EdgeInsets.only(bottom: 8),
      child: Row(children: [
        Container(
          width: 44,
          height: 44,
          decoration: BoxDecoration(
            color: kindColor.withValues(alpha: 0.18),
            borderRadius: BorderRadius.circular(10),
          ),
          child: Icon(
            wp.kind == 'park'
                ? Icons.local_parking_rounded
                : wp.kind == 'summon'
                    ? Icons.hail_rounded
                    : wp.kind == 'home'
                        ? Icons.home_rounded
                        : Icons.location_on_rounded,
            color: kindColor,
          ),
        ),
        const SizedBox(width: 12),
        Expanded(
          child:
              Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Row(children: [
              Text(wp.name,
                  style: const TextStyle(
                      fontSize: 14, fontWeight: FontWeight.w600)),
              const SizedBox(width: 8),
              if (wp.stale)
                Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                  decoration: BoxDecoration(
                    color: AppColors.warning.withValues(alpha: 0.18),
                    borderRadius: BorderRadius.circular(4),
                  ),
                  child: const Text('stale',
                      style: TextStyle(
                          fontSize: 9,
                          fontWeight: FontWeight.w700,
                          color: AppColors.warning,
                          letterSpacing: 0.6)),
                ),
            ]),
            const SizedBox(height: 2),
            Text(
              '(${wp.x.toStringAsFixed(2)}, ${wp.y.toStringAsFixed(2)})  '
              'yaw ${(wp.yaw * 57.2958).toStringAsFixed(0)}°',
              style: TextStyle(
                  fontSize: 11,
                  fontFamily: 'monospace',
                  color: colors.textSecondary),
            ),
          ]),
        ),
        IconButton(
          tooltip: 'Navigate',
          onPressed: () async {
            final ok = await conn.navigateToNamedWaypoint(wp.name);
            if (!mounted) return;
            ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                content:
                    Text(ok ? 'Going to "${wp.name}"' : 'Navigate failed')));
          },
          icon: Icon(Icons.navigation_rounded,
              color: wp.stale ? AppColors.warning : AppColors.info),
        ),
        IconButton(
          tooltip: 'Rename',
          onPressed: () => _renameManualWaypoint(wp, conn),
          icon: const Icon(Icons.edit_rounded, color: AppColors.brand),
        ),
        IconButton(
          tooltip: 'Delete',
          onPressed: () async {
            await conn.deleteNamedWaypoint(wp.name);
            await _refreshManualWaypoints();
          },
          icon:
              const Icon(Icons.delete_outline_rounded, color: AppColors.danger),
        ),
      ]),
    );
  }

  /// Rename a saved waypoint. The bridge keys waypoints by `name`, so the
  /// rename is implemented as POST new + DELETE old. POST-first means a
  /// failure leaves the original entry intact; a delete failure after a
  /// successful post will produce a transient duplicate until the next
  /// 5 s waypoint poll reconciles.
  Future<void> _renameManualWaypoint(
      NamedWaypoint wp, ConnectionService conn) async {
    final ctrl = TextEditingController(text: wp.name);
    final newName = await showDialog<String>(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Rename waypoint'),
        content: TextField(
          controller: ctrl,
          autofocus: true,
          decoration: const InputDecoration(labelText: 'Name'),
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx), child: const Text('Cancel')),
          ElevatedButton(
            onPressed: () => Navigator.pop(ctx, ctrl.text.trim()),
            child: const Text('Save'),
          ),
        ],
      ),
    );
    if (newName == null || newName.isEmpty || newName == wp.name) return;
    final saved = await conn.saveNamedWaypoint(
      name: newName,
      kind: wp.kind,
      x: wp.x,
      y: wp.y,
      yaw: wp.yaw,
    );
    if (saved == null) {
      if (!mounted) return;
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
          content: Text('Rename failed: could not save new name')));
      return;
    }
    await conn.deleteNamedWaypoint(wp.name);
    await _refreshManualWaypoints();
    if (!mounted) return;
    ScaffoldMessenger.of(context)
        .showSnackBar(SnackBar(content: Text('Renamed to "$newName"')));
  }

  // ═══════════════════════════════════════════════════════════════════════════
  //  MISSIONS TAB
  // ═══════════════════════════════════════════════════════════════════════════

  Widget _buildMissionsView(ResolvedColors colors) {
    final conn = context.watch<ConnectionService>();

    return Column(
      children: [
        // Waypoint list
        Expanded(
          child: _waypoints.isEmpty
              ? _emptyState(
                  Icons.route_rounded,
                  'No Waypoints',
                  'Add waypoints to create a navigation mission',
                  colors,
                )
              : ReorderableListView.builder(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                  itemCount: _waypoints.length,
                  onReorder: (oldIndex, newIndex) {
                    setState(() {
                      if (newIndex > oldIndex) newIndex--;
                      final item = _waypoints.removeAt(oldIndex);
                      _waypoints.insert(newIndex, item);
                    });
                  },
                  itemBuilder: (_, i) => _waypointTile(i, colors),
                ),
        ),

        // Mission progress
        if (_missionState == MissionExecutionState.running)
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16),
            child: Column(
              children: [
                LinearProgressIndicator(
                  value: _waypoints.isEmpty
                      ? 0
                      : _currentWaypointIndex / _waypoints.length,
                  backgroundColor: colors.surfaceLight,
                  valueColor: const AlwaysStoppedAnimation(AppColors.info),
                ),
                const SizedBox(height: 4),
                Text(
                  'Waypoint ${_currentWaypointIndex + 1} of ${_waypoints.length}',
                  style: TextStyle(fontSize: 11, color: colors.textSecondary),
                ),
              ],
            ),
          ),

        // Action buttons
        Padding(
          padding: const EdgeInsets.fromLTRB(16, 8, 16, 16),
          child: Row(
            children: [
              // Add waypoint
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () => _showAddWaypointDialog(context, colors),
                  icon: const Icon(Icons.add_location_rounded, size: 18),
                  label: const Text('Add',
                      style: TextStyle(fontWeight: FontWeight.w600)),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: colors.accentDim,
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12)),
                  ),
                ),
              ),
              const SizedBox(width: 8),
              // Execute / Stop
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: conn.isConnected && _waypoints.isNotEmpty
                      ? (_missionState == MissionExecutionState.running
                          ? _stopMission
                          : _executeMission)
                      : null,
                  icon: Icon(
                    _missionState == MissionExecutionState.running
                        ? Icons.stop_rounded
                        : Icons.play_arrow_rounded,
                    size: 18,
                  ),
                  label: Text(
                    _missionState == MissionExecutionState.running
                        ? 'Stop'
                        : 'Execute',
                    style: const TextStyle(fontWeight: FontWeight.w600),
                  ),
                  style: ElevatedButton.styleFrom(
                    backgroundColor:
                        _missionState == MissionExecutionState.running
                            ? AppColors.danger
                            : AppColors.success.withValues(alpha: 0.8),
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12)),
                  ),
                ),
              ),
              const SizedBox(width: 8),
              // Save
              IconButton(
                onPressed: _waypoints.isNotEmpty
                    ? () => _saveMission(context, colors)
                    : null,
                icon: const Icon(Icons.save_rounded, size: 22),
                color: colors.textSecondary,
              ),
              // Load
              IconButton(
                onPressed: () => _loadMission(context, colors),
                icon: const Icon(Icons.folder_open_rounded, size: 22),
                color: colors.textSecondary,
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _waypointTile(int index, ResolvedColors colors) {
    final wp = _waypoints[index];
    final isCurrent = _missionState == MissionExecutionState.running &&
        index == _currentWaypointIndex;

    return GlassCard(
      key: ValueKey('wp_$index'),
      margin: const EdgeInsets.only(bottom: 6),
      color: isCurrent ? AppColors.info.withValues(alpha: 0.1) : null,
      child: Row(
        children: [
          // Drag handle
          Icon(Icons.drag_handle_rounded, size: 20, color: colors.textTertiary),
          const SizedBox(width: 10),
          // Index
          Container(
            width: 28,
            height: 28,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: isCurrent ? AppColors.info : colors.surfaceLight,
            ),
            child: Center(
              child: Text('${index + 1}',
                  style: TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w700,
                      color: isCurrent ? Colors.white : colors.textSecondary)),
            ),
          ),
          const SizedBox(width: 12),
          // Coords
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                if (wp.label != null)
                  Text(wp.label!,
                      style: const TextStyle(
                          fontSize: 12, fontWeight: FontWeight.w600)),
                Text(
                  'X: ${wp.x.toStringAsFixed(2)}  Y: ${wp.y.toStringAsFixed(2)}  Yaw: ${wp.yaw.toStringAsFixed(2)}',
                  style: TextStyle(
                      fontSize: 11,
                      fontFamily: 'monospace',
                      color: colors.textSecondary),
                ),
              ],
            ),
          ),
          // Delete
          IconButton(
            onPressed: () => setState(() => _waypoints.removeAt(index)),
            icon:
                Icon(Icons.close_rounded, size: 18, color: colors.textTertiary),
            padding: EdgeInsets.zero,
            constraints: const BoxConstraints(minWidth: 32, minHeight: 32),
          ),
        ],
      ),
    );
  }

  void _showAddWaypointDialog(BuildContext context, ResolvedColors colors) {
    final xCtrl = TextEditingController();
    final yCtrl = TextEditingController();
    final yawCtrl = TextEditingController(text: '0');
    final labelCtrl = TextEditingController();

    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Add Waypoint', style: TextStyle(fontSize: 18)),
        content: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextField(
                controller: labelCtrl,
                decoration:
                    const InputDecoration(labelText: 'Label (optional)')),
            const SizedBox(height: 10),
            TextField(
                controller: xCtrl,
                keyboardType: const TextInputType.numberWithOptions(
                    decimal: true, signed: true),
                decoration: const InputDecoration(labelText: 'X (meters)')),
            const SizedBox(height: 10),
            TextField(
                controller: yCtrl,
                keyboardType: const TextInputType.numberWithOptions(
                    decimal: true, signed: true),
                decoration: const InputDecoration(labelText: 'Y (meters)')),
            const SizedBox(height: 10),
            TextField(
                controller: yawCtrl,
                keyboardType: const TextInputType.numberWithOptions(
                    decimal: true, signed: true),
                decoration: const InputDecoration(labelText: 'Yaw (radians)')),
          ],
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx),
              child: Text('Cancel',
                  style: TextStyle(color: colors.textSecondary))),
          ElevatedButton(
            onPressed: () {
              final x = double.tryParse(xCtrl.text);
              final y = double.tryParse(yCtrl.text);
              if (x == null || y == null) return;
              setState(() => _waypoints.add(Waypoint(
                    x: x,
                    y: y,
                    yaw: double.tryParse(yawCtrl.text) ?? 0,
                    label: labelCtrl.text.isEmpty ? null : labelCtrl.text,
                  )));
              Navigator.pop(ctx);
            },
            child: const Text('Add'),
          ),
        ],
      ),
    );
  }

  void _executeMission() {
    if (_waypoints.isEmpty) return;
    final conn = context.read<ConnectionService>();
    final logger = context.read<EventLogger>();

    setState(() {
      _missionState = MissionExecutionState.running;
      _currentWaypointIndex = 0;
    });

    logger.info('Mission started with ${_waypoints.length} waypoints',
        LogCategory.navigation);
    _sendCurrentWaypoint(conn, logger);
  }

  void _sendCurrentWaypoint(ConnectionService conn, EventLogger logger) {
    if (_currentWaypointIndex >= _waypoints.length) {
      setState(() => _missionState = MissionExecutionState.completed);
      logger.success('Mission completed!', LogCategory.navigation);
      return;
    }

    final wp = _waypoints[_currentWaypointIndex];
    conn.sendNavGoal(wp.x, wp.y, wp.yaw);

    // Poll for arrival (simple: advance after 10s delay — real impl would check nav status)
    _missionTimer?.cancel();
    _missionTimer = Timer(const Duration(seconds: 10), () {
      if (_missionState != MissionExecutionState.running) return;
      setState(() => _currentWaypointIndex++);
      _sendCurrentWaypoint(conn, logger);
    });
  }

  void _stopMission() {
    _missionTimer?.cancel();
    setState(() => _missionState = MissionExecutionState.idle);
    context
        .read<EventLogger>()
        .warn('Mission stopped by user', LogCategory.navigation);
  }

  void _saveMission(BuildContext context, ResolvedColors colors) {
    final nameCtrl = TextEditingController();
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Save Mission'),
        content: TextField(
          controller: nameCtrl,
          decoration: const InputDecoration(labelText: 'Mission name'),
          autofocus: true,
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx),
              child: Text('Cancel',
                  style: TextStyle(color: colors.textSecondary))),
          ElevatedButton(
            onPressed: () {
              if (nameCtrl.text.isEmpty) return;
              final mission = Mission(
                  name: nameCtrl.text, waypoints: List.from(_waypoints));
              context
                  .read<PreferencesService>()
                  .saveMission(nameCtrl.text, mission.toJsonString());
              Navigator.pop(ctx);
              ScaffoldMessenger.of(context).showSnackBar(
                SnackBar(content: Text('Mission "${nameCtrl.text}" saved')),
              );
            },
            child: const Text('Save'),
          ),
        ],
      ),
    );
  }

  void _loadMission(BuildContext context, ResolvedColors colors) {
    final prefs = context.read<PreferencesService>();
    final missions = prefs.savedMissions;

    if (missions.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(content: Text('No saved missions')),
      );
      return;
    }

    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Load Mission'),
        content: SizedBox(
          width: double.maxFinite,
          child: ListView.builder(
            shrinkWrap: true,
            itemCount: missions.length,
            itemBuilder: (_, i) {
              final mission = Mission.fromJsonString(missions[i]);
              return ListTile(
                title: Text(mission.name),
                subtitle: Text('${mission.waypoints.length} waypoints'),
                trailing: IconButton(
                  icon: const Icon(Icons.delete_outline,
                      size: 20, color: AppColors.danger),
                  onPressed: () {
                    prefs.deleteMission(mission.name);
                    Navigator.pop(ctx);
                    _loadMission(context, colors);
                  },
                ),
                onTap: () {
                  setState(() {
                    _waypoints.clear();
                    _waypoints.addAll(mission.waypoints);
                  });
                  Navigator.pop(ctx);
                },
              );
            },
          ),
        ),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx),
              child:
                  Text('Close', style: TextStyle(color: colors.textSecondary))),
        ],
      ),
    );
  }

  Widget _emptyState(
      IconData icon, String title, String subtitle, ResolvedColors colors) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 48, color: colors.textTertiary),
          const SizedBox(height: 14),
          Text(title,
              style: TextStyle(
                  fontSize: 16,
                  fontWeight: FontWeight.w600,
                  color: colors.textSecondary)),
          const SizedBox(height: 6),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 40),
            child: Text(subtitle,
                textAlign: TextAlign.center,
                style: TextStyle(fontSize: 13, color: colors.textTertiary)),
          ),
        ],
      ),
    );
  }
}
