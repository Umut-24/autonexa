import 'dart:async';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:path_provider/path_provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../services/event_logger.dart';
import '../widgets/glass_card.dart';
import '../widgets/param_tuner_dialog.dart';

/// Event log, network stats, and system diagnostics.
/// Accessed via the "More" bottom sheet.
class DiagnosticsTab extends StatefulWidget {
  const DiagnosticsTab({super.key});

  @override
  State<DiagnosticsTab> createState() => _DiagnosticsTabState();
}

class _DiagnosticsTabState extends State<DiagnosticsTab> {
  LogCategory? _filterCategory;
  Timer? _healthTimer;
  List<HealthRow> _health = [];

  @override
  void initState() {
    super.initState();
    _healthTimer = Timer.periodic(const Duration(seconds: 2), (_) => _refreshHealth());
    WidgetsBinding.instance.addPostFrameCallback((_) => _refreshHealth());
  }

  @override
  void dispose() {
    _healthTimer?.cancel();
    super.dispose();
  }

  Future<void> _refreshHealth() async {
    final conn = context.read<ConnectionService>();
    if (!conn.isConnected) return;
    final h = await conn.getHealth();
    if (!mounted) return;
    setState(() => _health = h);
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final logger = context.watch<EventLogger>();
    final conn = context.watch<ConnectionService>();

    final entries = _filterCategory == null
        ? logger.entries
        : logger.filtered(category: _filterCategory);

    return Scaffold(
      backgroundColor: colors.background,
      appBar: AppBar(
        backgroundColor: colors.surface,
        title: const Text('Diagnostics', style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600)),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_rounded),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          IconButton(
            icon: const Icon(Icons.delete_outline_rounded, size: 22),
            onPressed: () {
              logger.clear();
              ScaffoldMessenger.of(context).showSnackBar(
                const SnackBar(content: Text('Log cleared')),
              );
            },
          ),
          IconButton(
            icon: const Icon(Icons.file_download_outlined, size: 22),
            onPressed: () => _exportLog(context, logger),
          ),
        ],
      ),
      body: SafeArea(
        child: Column(
          children: [
            // Network stats
            GlassCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text('NETWORK', style: TextStyle(fontSize: 11, fontWeight: FontWeight.w700,
                      letterSpacing: 1.2, color: colors.textTertiary)),
                  const SizedBox(height: 10),
                  Row(
                    children: [
                      Expanded(child: _statItem('Latency', '${conn.latencyMs}ms',
                          conn.latencyMs < 50 ? AppColors.success : AppColors.warning, colors)),
                      Expanded(child: _statItem('Status',
                          conn.isConnected ? 'Connected' : 'Offline',
                          conn.isConnected ? AppColors.success : colors.textTertiary, colors)),
                      Expanded(child: _statItem('Commands',
                          '${conn.commandsSent}', colors.textPrimary, colors)),
                    ],
                  ),
                  if (conn.connectedSince != null) ...[
                    const SizedBox(height: 8),
                    Text(
                      'Uptime: ${_formatUptime(DateTime.now().difference(conn.connectedSince!))}',
                      style: TextStyle(fontSize: 11, color: colors.textSecondary),
                    ),
                  ],
                ],
              ),
            ),

            // Topic health
            GlassCard(
              child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Row(children: [
                  Text('TOPIC HEALTH', style: TextStyle(fontSize: 11, fontWeight: FontWeight.w700,
                      letterSpacing: 1.2, color: colors.textTertiary)),
                  const Spacer(),
                  IconButton(
                    iconSize: 16, padding: EdgeInsets.zero,
                    visualDensity: VisualDensity.compact,
                    onPressed: _refreshHealth,
                    icon: Icon(Icons.refresh_rounded, color: colors.textSecondary),
                  ),
                ]),
                if (_health.isEmpty)
                  Padding(
                    padding: const EdgeInsets.symmetric(vertical: 6),
                    child: Text(
                      conn.isConnected ? 'No data yet' : 'Connect to see topic rates',
                      style: TextStyle(fontSize: 11, color: colors.textTertiary),
                    ),
                  )
                else
                  ..._health.map((row) => _healthRow(row, colors)),
              ]),
            ),

            // Live param tuner
            GlassCard(
              child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Text('LIVE PARAMETERS', style: TextStyle(fontSize: 11, fontWeight: FontWeight.w700,
                    letterSpacing: 1.2, color: colors.textTertiary)),
                const SizedBox(height: 8),
                Text(
                  'Tune Nav2 / bridge parameters at runtime. Numeric edits persist on disk.',
                  style: TextStyle(fontSize: 12, color: colors.textSecondary),
                ),
                const SizedBox(height: 8),
                SizedBox(
                  width: double.infinity,
                  child: ElevatedButton.icon(
                    icon: const Icon(Icons.tune_rounded, size: 18),
                    label: const Text('Open param tuner'),
                    onPressed: conn.isConnected
                        ? () => ParamTunerDialog.show(context)
                        : null,
                  ),
                ),
              ]),
            ),

            // Filter chips
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
              child: Row(
                children: [
                  _filterChip('All', null, colors),
                  _filterChip('Connection', LogCategory.connection, colors),
                  _filterChip('Control', LogCategory.control, colors),
                  _filterChip('Nav', LogCategory.navigation, colors),
                ],
              ),
            ),

            // Event log
            Expanded(
              child: entries.isEmpty
                  ? Center(
                      child: Text('No events yet',
                          style: TextStyle(color: colors.textTertiary)),
                    )
                  : ListView.builder(
                      reverse: true,
                      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                      itemCount: entries.length,
                      itemBuilder: (_, i) {
                        final entry = entries[entries.length - 1 - i];
                        return _logEntry(entry, colors);
                      },
                    ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _statItem(String label, String value, Color color, ResolvedColors colors) {
    return Column(
      children: [
        Text(value, style: TextStyle(fontSize: 16, fontWeight: FontWeight.w700,
            fontFamily: 'monospace', color: color)),
        const SizedBox(height: 2),
        Text(label, style: TextStyle(fontSize: 10, color: colors.textSecondary)),
      ],
    );
  }

  Widget _filterChip(String label, LogCategory? category, ResolvedColors colors) {
    final active = _filterCategory == category;
    return Padding(
      padding: const EdgeInsets.only(right: 6),
      child: GestureDetector(
        onTap: () => setState(() => _filterCategory = category),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
          decoration: BoxDecoration(
            color: active ? colors.accentSurface : colors.surfaceLight,
            borderRadius: BorderRadius.circular(8),
            border: Border.all(color: active ? colors.accent.withValues(alpha: 0.3) : colors.border),
          ),
          child: Text(label,
              style: TextStyle(fontSize: 11, fontWeight: FontWeight.w600,
                  color: active ? colors.accent : colors.textSecondary)),
        ),
      ),
    );
  }

  Widget _logEntry(LogEntry entry, ResolvedColors colors) {
    final Color levelColor;
    switch (entry.level) {
      case LogLevel.info:
        levelColor = colors.textSecondary;
        break;
      case LogLevel.success:
        levelColor = AppColors.success;
        break;
      case LogLevel.warning:
        levelColor = AppColors.warning;
        break;
      case LogLevel.error:
        levelColor = AppColors.danger;
        break;
    }

    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Container(
            width: 6,
            height: 6,
            margin: const EdgeInsets.only(top: 5, right: 8),
            decoration: BoxDecoration(shape: BoxShape.circle, color: levelColor),
          ),
          Text(
            '${entry.time.hour.toString().padLeft(2, '0')}:'
            '${entry.time.minute.toString().padLeft(2, '0')}:'
            '${entry.time.second.toString().padLeft(2, '0')}',
            style: TextStyle(fontSize: 10, fontFamily: 'monospace',
                color: colors.textTertiary),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Text(entry.message,
                style: TextStyle(fontSize: 12, color: levelColor)),
          ),
        ],
      ),
    );
  }

  Future<void> _exportLog(BuildContext context, EventLogger logger) async {
    try {
      final dir = await getApplicationDocumentsDirectory();
      final file = File('${dir.path}/autonexa_log_${DateTime.now().millisecondsSinceEpoch}.txt');
      await file.writeAsString(logger.export());
      if (context.mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Log exported to ${file.path}')),
        );
      }
    } catch (e) {
      if (context.mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Export failed: $e')),
        );
      }
    }
  }

  String _formatUptime(Duration d) {
    if (d.inHours > 0) return '${d.inHours}h ${d.inMinutes % 60}m';
    if (d.inMinutes > 0) return '${d.inMinutes}m ${d.inSeconds % 60}s';
    return '${d.inSeconds}s';
  }

  Widget _healthRow(HealthRow row, ResolvedColors colors) {
    final color = row.ok
        ? AppColors.success
        : (row.lastAgeS == null || row.lastAgeS! > 5.0
            ? AppColors.danger
            : AppColors.warning);
    final age = row.lastAgeS == null ? '—' : '${row.lastAgeS!.toStringAsFixed(1)}s';
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(children: [
        Container(width: 8, height: 8,
            decoration: BoxDecoration(shape: BoxShape.circle, color: color)),
        const SizedBox(width: 8),
        Expanded(
          flex: 3,
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Text(row.label, style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w600)),
            Text(row.topic, style: TextStyle(fontSize: 10, fontFamily: 'monospace', color: colors.textTertiary)),
          ]),
        ),
        Expanded(
          flex: 2,
          child: Text('${row.rateHz.toStringAsFixed(1)} Hz',
              textAlign: TextAlign.right,
              style: TextStyle(fontSize: 11, fontFamily: 'monospace', color: colors.textSecondary)),
        ),
        const SizedBox(width: 8),
        SizedBox(
          width: 50,
          child: Text(age,
              textAlign: TextAlign.right,
              style: TextStyle(fontSize: 11, fontFamily: 'monospace', color: colors.textTertiary)),
        ),
      ]),
    );
  }
}
