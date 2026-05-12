import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';

import '../models/robot_config.dart';
import '../services/connection_service.dart';
import '../theme/app_colors.dart';

/// Settings -> Robot Dimensions editor.
///
/// Fetches the current dimensions from the bridge on open, lets the operator
/// edit chassis L/W/H, wheelbase, LiDAR x/y/z, and footprint padding (values
/// in metres with 4 decimal places). Save POSTs to /api/robot_config; the
/// bridge regenerates the URDF, syncs Nav2 costmap footprints, and persists
/// the values to ~/.autonexa/robot_dimensions.yaml.
class RobotDimensionsDialog extends StatefulWidget {
  const RobotDimensionsDialog({super.key});

  static Future<void> show(BuildContext context) {
    return showDialog(
      context: context,
      builder: (_) => const RobotDimensionsDialog(),
    );
  }

  @override
  State<RobotDimensionsDialog> createState() => _RobotDimensionsDialogState();
}

class _RobotDimensionsDialogState extends State<RobotDimensionsDialog> {
  late RobotConfig _initial;
  late Map<String, TextEditingController> _controllers;
  bool _busy = false;
  String _status = '';

  static const _fields = <_Field>[
    _Field('chassis_length', 'Chassis length (m)', 'X extent of base_link'),
    _Field('chassis_width', 'Chassis width (m)', 'Y extent of base_link'),
    _Field('chassis_height', 'Chassis height (m)', 'Z extent of base_link'),
    _Field('wheelbase', 'Wheelbase (m)', 'Front-axle to rear-axle'),
    _Field('lidar_x', 'LiDAR x (m)', 'Forward offset from chassis center'),
    _Field('lidar_y', 'LiDAR y (m)', 'Lateral offset (+ = left)'),
    _Field('lidar_z', 'LiDAR z (m)', 'Height above chassis floor'),
    _Field('footprint_padding', 'Footprint padding (m)',
        'Extra ring around footprint for Nav2 collision'),
  ];

  @override
  void initState() {
    super.initState();
    _initial = context.read<ConnectionService>().robotConfig;
    _controllers = {
      for (final f in _fields)
        f.key: TextEditingController(
            text: _readField(_initial, f.key).toStringAsFixed(4)),
    };
    _loadFresh();
  }

  Future<void> _loadFresh() async {
    final conn = context.read<ConnectionService>();
    final fresh = await conn.fetchRobotConfig();
    if (!mounted || fresh == null) return;
    setState(() {
      _initial = fresh;
      for (final f in _fields) {
        _controllers[f.key]!.text = _readField(fresh, f.key).toStringAsFixed(4);
      }
    });
  }

  @override
  void dispose() {
    for (final c in _controllers.values) {
      c.dispose();
    }
    super.dispose();
  }

  Map<String, double> _diff() {
    final out = <String, double>{};
    for (final f in _fields) {
      final parsed = double.tryParse(_controllers[f.key]!.text);
      if (parsed == null) continue;
      final current = _readField(_initial, f.key);
      if ((parsed - current).abs() > 1e-6) {
        out[f.key] = parsed;
      }
    }
    return out;
  }

  Future<void> _save() async {
    final overrides = _diff();
    if (overrides.isEmpty) {
      setState(() => _status = 'No changes to apply.');
      return;
    }
    setState(() {
      _busy = true;
      _status = 'Applying ${overrides.length} change(s)…';
    });
    final ok = await context.read<ConnectionService>().setRobotConfig(overrides);
    if (!mounted) return;
    setState(() {
      _busy = false;
      _status = ok ? 'Saved. URDF + costmaps updated.' : 'Save failed.';
    });
    if (ok) {
      Navigator.of(context).pop();
    }
  }

  Future<void> _resetToDefaults() async {
    final defaults = RobotConfig.defaults.toJson();
    final overrides = <String, double>{
      for (final entry in defaults.entries)
        if (entry.value is num) entry.key: (entry.value as num).toDouble(),
    };
    setState(() {
      _busy = true;
      _status = 'Resetting to defaults…';
      for (final f in _fields) {
        _controllers[f.key]!.text = (overrides[f.key] ?? 0).toStringAsFixed(4);
      }
    });
    final ok = await context.read<ConnectionService>().setRobotConfig(overrides);
    if (!mounted) return;
    setState(() {
      _busy = false;
      _status = ok ? 'Defaults applied.' : 'Reset failed.';
    });
    if (ok) Navigator.of(context).pop();
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: const Text('Robot Dimensions'),
      content: SizedBox(
        width: 420,
        child: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              _FootprintPreview(
                controllers: _controllers,
              ),
              const SizedBox(height: 12),
              for (final f in _fields) _buildField(f),
              if (_status.isNotEmpty)
                Padding(
                  padding: const EdgeInsets.only(top: 8),
                  child: Text(
                    _status,
                    style: TextStyle(
                        color: _status.contains('fail') ||
                                _status.contains('failed')
                            ? AppColors.danger
                            : AppColors.success),
                  ),
                ),
            ],
          ),
        ),
      ),
      actions: [
        TextButton(
          onPressed: _busy ? null : _resetToDefaults,
          child: const Text('Reset to defaults'),
        ),
        TextButton(
          onPressed: _busy ? null : () => Navigator.of(context).pop(),
          child: const Text('Cancel'),
        ),
        FilledButton(
          onPressed: _busy ? null : _save,
          child: Text(_busy ? 'Saving…' : 'Save'),
        ),
      ],
    );
  }

  Widget _buildField(_Field f) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: TextField(
        controller: _controllers[f.key],
        keyboardType: const TextInputType.numberWithOptions(
            decimal: true, signed: true),
        inputFormatters: [
          FilteringTextInputFormatter.allow(RegExp(r'[-0-9.]')),
        ],
        onChanged: (_) => setState(() {}),
        decoration: InputDecoration(
          labelText: f.label,
          helperText: f.help,
          isDense: true,
          border: const OutlineInputBorder(),
        ),
      ),
    );
  }

  static double _readField(RobotConfig c, String key) {
    switch (key) {
      case 'chassis_length':
        return c.chassisLength;
      case 'chassis_width':
        return c.chassisWidth;
      case 'chassis_height':
        return c.chassisHeight;
      case 'wheelbase':
        return c.wheelbase;
      case 'lidar_x':
        return c.lidarX;
      case 'lidar_y':
        return c.lidarY;
      case 'lidar_z':
        return c.lidarZ;
      case 'footprint_padding':
        return c.footprintPadding;
      default:
        return 0;
    }
  }
}

class _Field {
  final String key;
  final String label;
  final String help;
  const _Field(this.key, this.label, this.help);
}

/// Live preview rectangle: redraws as the operator edits the chassis length /
/// width / LiDAR offset fields, so they can see the new footprint without
/// committing first.
class _FootprintPreview extends StatelessWidget {
  final Map<String, TextEditingController> controllers;
  const _FootprintPreview({required this.controllers});

  @override
  Widget build(BuildContext context) {
    final length = double.tryParse(controllers['chassis_length']!.text) ?? 0.30;
    final width = double.tryParse(controllers['chassis_width']!.text) ?? 0.20;
    final lidarX = double.tryParse(controllers['lidar_x']!.text) ?? 0.01;
    final lidarY = double.tryParse(controllers['lidar_y']!.text) ?? 0.00;
    return Container(
      height: 140,
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.04),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.black12),
      ),
      child: CustomPaint(
        painter: _FootprintPainter(
          chassisLength: length,
          chassisWidth: width,
          lidarX: lidarX,
          lidarY: lidarY,
        ),
      ),
    );
  }
}

class _FootprintPainter extends CustomPainter {
  final double chassisLength;
  final double chassisWidth;
  final double lidarX;
  final double lidarY;
  _FootprintPainter({
    required this.chassisLength,
    required this.chassisWidth,
    required this.lidarX,
    required this.lidarY,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final cx = size.width / 2;
    final cy = size.height / 2;
    // Scale so the chassis fills ~75% of the smaller dimension.
    final usable = math.min(size.width, size.height) * 0.75;
    final maxExtent = math.max(chassisLength, chassisWidth);
    final scale = maxExtent > 0 ? usable / maxExtent : 1.0;
    final halfL = (chassisLength * scale) / 2.0;
    final rect = Rect.fromCenter(
      center: Offset(cx, cy),
      width: chassisLength * scale,
      height: chassisWidth * scale,
    );
    final fill = Paint()..color = AppColors.brand.withValues(alpha: 0.15);
    final stroke = Paint()
      ..color = Colors.black.withValues(alpha: 0.6)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;
    canvas.drawRect(rect, fill);
    canvas.drawRect(rect, stroke);
    // Forward arrow
    final arrow = Paint()
      ..color = AppColors.brand
      ..strokeWidth = 2
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(
      Offset(cx, cy),
      Offset(cx + halfL * 0.8, cy),
      arrow,
    );
    // LiDAR marker
    final lidar = Paint()..color = AppColors.danger;
    canvas.drawCircle(
      Offset(cx + lidarX * scale, cy - lidarY * scale),
      3.5,
      lidar,
    );
    // Axis label "F" at the front
    final textPainter = TextPainter(
      text: const TextSpan(
        text: 'FRONT',
        style: TextStyle(fontSize: 10, color: Colors.black54),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    textPainter.paint(canvas, Offset(cx + halfL * 0.85, cy - 7));
    // Dimensions label
    final dimsText = TextPainter(
      text: TextSpan(
        text:
            '${(chassisLength * 100).toStringAsFixed(1)} × ${(chassisWidth * 100).toStringAsFixed(1)} cm',
        style: const TextStyle(fontSize: 11, color: Colors.black87),
      ),
      textDirection: TextDirection.ltr,
    )..layout();
    dimsText.paint(canvas, Offset(8, size.height - 18));
  }

  @override
  bool shouldRepaint(covariant _FootprintPainter old) {
    return old.chassisLength != chassisLength ||
        old.chassisWidth != chassisWidth ||
        old.lidarX != lidarX ||
        old.lidarY != lidarY;
  }
}
