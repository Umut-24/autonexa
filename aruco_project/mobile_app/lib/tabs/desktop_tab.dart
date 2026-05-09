import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';

/// Desktop screenshot mirror — polls /api/desktop_version every 500 ms and
/// fetches /api/desktop_shot when the bridge has a newer frame. Lets the
/// user see RViz / dev windows on the Pi without a separate VNC client.
///
/// Bridge captures at ~1 Hz via gnome-screenshot (Wayland-friendly).
class DesktopTab extends StatefulWidget {
  const DesktopTab({super.key});

  @override
  State<DesktopTab> createState() => _DesktopTabState();
}

class _DesktopTabState extends State<DesktopTab> {
  Timer? _pollTimer;
  Uint8List? _bytes;
  int _lastVersion = -1;
  DateTime? _lastUpdate;
  bool _busy = false;
  String? _error;

  @override
  void initState() {
    super.initState();
    // 500 ms version probe → fetch on change. Bridge produces at 1 Hz so
    // this gives <1 s end-to-end latency without spamming the network.
    _pollTimer = Timer.periodic(const Duration(milliseconds: 500), (_) => _tick());
    WidgetsBinding.instance.addPostFrameCallback((_) => _tick());
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
    super.dispose();
  }

  Future<void> _tick() async {
    if (_busy) return;
    final conn = context.read<ConnectionService>();
    if (!conn.isConnected) return;
    _busy = true;
    try {
      final v = await conn.fetchDesktopVersion();
      if (v == null) {
        if (mounted) setState(() => _error = 'Bridge not reachable');
        return;
      }
      if (v == _lastVersion && _bytes != null) {
        if (mounted && _error != null) setState(() => _error = null);
        return;
      }
      final bytes = await conn.fetchDesktopShot();
      if (bytes == null) {
        // 503 = no shot yet (gnome-screenshot may still be initializing).
        if (mounted) setState(() => _error = 'Waiting for first capture…');
        return;
      }
      if (!mounted) return;
      setState(() {
        _bytes = bytes;
        _lastVersion = v;
        _lastUpdate = DateTime.now();
        _error = null;
      });
    } finally {
      _busy = false;
    }
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final conn = context.watch<ConnectionService>();
    final age = _lastUpdate == null
        ? null
        : DateTime.now().difference(_lastUpdate!).inMilliseconds / 1000.0;

    return Scaffold(
      backgroundColor: colors.background,
      appBar: AppBar(
        backgroundColor: colors.surface,
        title: const Text('Desktop',
            style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600)),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_rounded),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh_rounded, size: 22),
            tooltip: 'Force refresh',
            onPressed: () {
              setState(() => _lastVersion = -1);
              _tick();
            },
          ),
        ],
      ),
      body: SafeArea(
        child: Column(children: [
          _statusStrip(conn, age, colors),
          Expanded(child: _body(conn, colors)),
        ]),
      ),
    );
  }

  Widget _statusStrip(
      ConnectionService conn, double? age, ResolvedColors colors) {
    Color dot;
    String label;
    if (!conn.isConnected) {
      dot = AppColors.danger;
      label = 'No connection';
    } else if (age == null) {
      dot = AppColors.warning;
      label = 'Waiting for first frame…';
    } else if (age > 3.0) {
      dot = AppColors.warning;
      label = 'Stale (${age.toStringAsFixed(1)} s old)';
    } else {
      dot = AppColors.success;
      label = 'Live · ${age.toStringAsFixed(1)} s ago · v$_lastVersion';
    }
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 8),
      color: colors.surface,
      child: Row(children: [
        Container(width: 8, height: 8,
            decoration: BoxDecoration(shape: BoxShape.circle, color: dot)),
        const SizedBox(width: 8),
        Expanded(
          child: Text(label,
              style: TextStyle(fontSize: 12, color: colors.textSecondary)),
        ),
        if (_error != null)
          Text(_error!,
              style: const TextStyle(fontSize: 11, color: AppColors.warning)),
      ]),
    );
  }

  Widget _body(ConnectionService conn, ResolvedColors colors) {
    if (!conn.isConnected) {
      return Center(
        child: Padding(
          padding: const EdgeInsets.all(24),
          child: Text(
            'Connect to the bridge first to mirror the Pi desktop.',
            textAlign: TextAlign.center,
            style: TextStyle(color: colors.textTertiary),
          ),
        ),
      );
    }
    if (_bytes == null) {
      return Center(
        child: Column(mainAxisSize: MainAxisSize.min, children: [
          const CircularProgressIndicator(),
          const SizedBox(height: 16),
          Text('Waiting for first desktop frame…',
              style: TextStyle(color: colors.textTertiary)),
          const SizedBox(height: 6),
          Text(
            'On the Pi: `sudo apt install gnome-screenshot` if missing.',
            style: TextStyle(fontSize: 11, color: colors.textTertiary,
                fontFamily: 'monospace'),
          ),
        ]),
      );
    }
    // Pinch to zoom + pan; gapless playback so version flips don't flash.
    return InteractiveViewer(
      minScale: 0.5,
      maxScale: 5.0,
      child: Center(
        child: Image.memory(
          _bytes!,
          gaplessPlayback: true,
          fit: BoxFit.contain,
          filterQuality: FilterQuality.medium,
        ),
      ),
    );
  }
}
