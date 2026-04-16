import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';

/// Animated connection status dot with optional label.
class ConnectionIndicator extends StatelessWidget {
  final ConnectionStatus status;
  final bool showLabel;

  const ConnectionIndicator({
    super.key,
    required this.status,
    this.showLabel = false,
  });

  Color _color(ResolvedColors colors) {
    switch (status) {
      case ConnectionStatus.connected:
        return AppColors.success;
      case ConnectionStatus.connecting:
        return AppColors.warning;
      case ConnectionStatus.error:
        return AppColors.danger;
      case ConnectionStatus.disconnected:
        return colors.textTertiary;
    }
  }

  String get _label {
    switch (status) {
      case ConnectionStatus.connected:
        return 'Connected';
      case ConnectionStatus.connecting:
        return 'Connecting...';
      case ConnectionStatus.error:
        return 'Error';
      case ConnectionStatus.disconnected:
        return 'Offline';
    }
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final color = _color(colors);

    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 10,
          height: 10,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: color,
            boxShadow: status == ConnectionStatus.connected
                ? [BoxShadow(color: color.withValues(alpha: 0.5), blurRadius: 8)]
                : null,
          ),
        ),
        if (showLabel) ...[
          const SizedBox(width: 8),
          Text(
            _label,
            style: TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w600,
              color: color,
            ),
          ),
        ],
      ],
    );
  }
}
