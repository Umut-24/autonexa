import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';

/// Compact battery indicator with icon, percentage, and voltage.
class BatteryIndicator extends StatelessWidget {
  final int percent;         // 0-100 or -1 for unknown
  final double voltage;      // 0 for unknown
  final bool compact;

  const BatteryIndicator({
    super.key,
    required this.percent,
    this.voltage = 0,
    this.compact = false,
  });

  Color _color() {
    if (percent < 0) return AppColors.darkTextTertiary;
    if (percent <= 10) return AppColors.danger;
    if (percent <= 25) return AppColors.warning;
    return AppColors.success;
  }

  IconData _icon() {
    if (percent < 0) return Icons.battery_unknown_rounded;
    if (percent <= 10) return Icons.battery_alert_rounded;
    if (percent <= 30) return Icons.battery_2_bar_rounded;
    if (percent <= 50) return Icons.battery_3_bar_rounded;
    if (percent <= 70) return Icons.battery_4_bar_rounded;
    if (percent <= 90) return Icons.battery_5_bar_rounded;
    return Icons.battery_full_rounded;
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final color = _color();

    if (compact) {
      return Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(_icon(), size: 16, color: color),
          const SizedBox(width: 3),
          Text(
            percent >= 0 ? '$percent%' : '--',
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.w600,
              fontFamily: 'monospace',
              color: color,
            ),
          ),
        ],
      );
    }

    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.12),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(color: color.withValues(alpha: 0.25)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(_icon(), size: 20, color: color),
          const SizedBox(width: 8),
          Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(
                percent >= 0 ? '$percent%' : 'Unknown',
                style: TextStyle(
                  fontSize: 14,
                  fontWeight: FontWeight.w700,
                  color: color,
                ),
              ),
              if (voltage > 0)
                Text(
                  '${voltage.toStringAsFixed(1)}V',
                  style: TextStyle(
                    fontSize: 10,
                    fontFamily: 'monospace',
                    color: colors.textSecondary,
                  ),
                ),
            ],
          ),
        ],
      ),
    );
  }
}
