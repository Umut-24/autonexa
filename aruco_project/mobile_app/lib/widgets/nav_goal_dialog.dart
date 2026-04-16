import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';

/// Dialog for entering a Nav2 goal (X, Y, Yaw).
class NavGoalDialog extends StatefulWidget {
  final ConnectionService connection;
  final double? initialX;
  final double? initialY;
  final double? initialYaw;

  const NavGoalDialog({
    super.key,
    required this.connection,
    this.initialX,
    this.initialY,
    this.initialYaw,
  });

  static Future<bool?> show(
    BuildContext context,
    ConnectionService connection, {
    double? initialX,
    double? initialY,
    double? initialYaw,
  }) {
    return showDialog<bool>(
      context: context,
      builder: (_) => NavGoalDialog(
        connection: connection,
        initialX: initialX,
        initialY: initialY,
        initialYaw: initialYaw,
      ),
    );
  }

  @override
  State<NavGoalDialog> createState() => _NavGoalDialogState();
}

class _NavGoalDialogState extends State<NavGoalDialog> {
  late final TextEditingController _xCtrl;
  late final TextEditingController _yCtrl;
  late final TextEditingController _yawCtrl;
  bool _sending = false;

  @override
  void initState() {
    super.initState();
    _xCtrl = TextEditingController(text: widget.initialX?.toStringAsFixed(2) ?? '');
    _yCtrl = TextEditingController(text: widget.initialY?.toStringAsFixed(2) ?? '');
    _yawCtrl = TextEditingController(text: (widget.initialYaw ?? 0).toStringAsFixed(2));
  }

  @override
  void dispose() {
    _xCtrl.dispose();
    _yCtrl.dispose();
    _yawCtrl.dispose();
    super.dispose();
  }

  Future<void> _send() async {
    final x = double.tryParse(_xCtrl.text);
    final y = double.tryParse(_yCtrl.text);
    final yaw = double.tryParse(_yawCtrl.text) ?? 0.0;
    if (x == null || y == null) return;

    setState(() => _sending = true);
    final ok = await widget.connection.sendNavGoal(x, y, yaw);
    if (mounted) Navigator.pop(context, ok);
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;

    return AlertDialog(
      title: const Row(
        children: [
          Icon(Icons.navigation_rounded, color: AppColors.info, size: 22),
          SizedBox(width: 10),
          Text('Send Nav2 Goal', style: TextStyle(fontSize: 18)),
        ],
      ),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          TextField(
            controller: _xCtrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(
              labelText: 'X (meters)',
              prefixIcon: Icon(Icons.arrow_right_alt, size: 20),
            ),
            autofocus: true,
          ),
          const SizedBox(height: 12),
          TextField(
            controller: _yCtrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(
              labelText: 'Y (meters)',
              prefixIcon: Icon(Icons.arrow_upward, size: 20),
            ),
          ),
          const SizedBox(height: 12),
          TextField(
            controller: _yawCtrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(
              labelText: 'Yaw (radians)',
              prefixIcon: Icon(Icons.rotate_right, size: 20),
            ),
          ),
        ],
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context, false),
          child: Text('Cancel', style: TextStyle(color: colors.textSecondary)),
        ),
        ElevatedButton.icon(
          onPressed: _sending ? null : _send,
          icon: _sending
              ? const SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2))
              : const Icon(Icons.send_rounded, size: 16),
          label: const Text('Send'),
        ),
      ],
    );
  }
}
