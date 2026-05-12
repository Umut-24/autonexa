import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../models/robot_state.dart';

/// Compact chip showing ArUco marker info with status badge.
class MarkerChip extends StatelessWidget {
  final MarkerInfo marker;
  final VoidCallback? onTap;

  const MarkerChip({super.key, required this.marker, this.onTap});

  Color _statusColor(ResolvedColors colors) {
    switch (marker.status) {
      case MarkerStatus.live:
        return AppColors.success;
      case MarkerStatus.stale:
        return AppColors.warning;
      case MarkerStatus.lost:
        return colors.textTertiary;
    }
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;

    return GestureDetector(
      onTap: onTap,
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
        decoration: BoxDecoration(
          color: colors.surfaceLight,
          borderRadius: BorderRadius.circular(12),
          border: Border.all(color: colors.border),
        ),
        child: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Container(
              width: 8,
              height: 8,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                color: _statusColor(colors),
              ),
            ),
            const SizedBox(width: 8),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              mainAxisSize: MainAxisSize.min,
              children: [
                Text(
                  'ID ${marker.id}',
                  style: TextStyle(
                    fontSize: 12,
                    fontWeight: FontWeight.w700,
                    color: colors.textPrimary,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  '${marker.distance.toStringAsFixed(2)}m / ${marker.bearing.toStringAsFixed(0)}°',
                  style: TextStyle(
                    fontSize: 10,
                    fontFamily: 'monospace',
                    color: colors.textSecondary,
                  ),
                ),
              ],
            ),
            if (onTap != null) ...[
              const SizedBox(width: 8),
              const Icon(Icons.navigation_rounded, size: 14, color: AppColors.info),
            ],
          ],
        ),
      ),
    );
  }
}
