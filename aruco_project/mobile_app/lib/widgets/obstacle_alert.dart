import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../theme/app_colors.dart';

/// Proximity warning banner that appears when obstacles are detected too close.
class ObstacleAlert extends StatelessWidget {
  final double distanceM;
  final bool critical;

  const ObstacleAlert({
    super.key,
    required this.distanceM,
    this.critical = false,
  });

  @override
  Widget build(BuildContext context) {
    if (distanceM >= 0.15) return const SizedBox.shrink();

    final color = critical ? AppColors.danger : AppColors.warning;
    final label = critical ? 'COLLISION WARNING' : 'OBSTACLE NEARBY';
    final dist = '${(distanceM * 100).toStringAsFixed(0)}cm';

    // Haptic on critical
    if (critical) {
      HapticFeedback.heavyImpact();
    }

    return Container(
      margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(
        color: color.withValues(alpha: 0.15),
        borderRadius: BorderRadius.circular(12),
        border: Border.all(color: color.withValues(alpha: 0.5), width: 1.5),
      ),
      child: Row(
        children: [
          Icon(
            critical ? Icons.warning_rounded : Icons.radar_rounded,
            color: color,
            size: 22,
          ),
          const SizedBox(width: 10),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  label,
                  style: TextStyle(
                    fontSize: 11,
                    fontWeight: FontWeight.w800,
                    letterSpacing: 0.8,
                    color: color,
                  ),
                ),
                Text(
                  'Nearest obstacle at $dist',
                  style: TextStyle(
                    fontSize: 12,
                    color: color.withValues(alpha: 0.8),
                  ),
                ),
              ],
            ),
          ),
          Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
            decoration: BoxDecoration(
              color: color.withValues(alpha: 0.2),
              borderRadius: BorderRadius.circular(8),
            ),
            child: Text(
              dist,
              style: TextStyle(
                fontSize: 16,
                fontWeight: FontWeight.w800,
                fontFamily: 'monospace',
                color: color,
              ),
            ),
          ),
        ],
      ),
    );
  }
}
