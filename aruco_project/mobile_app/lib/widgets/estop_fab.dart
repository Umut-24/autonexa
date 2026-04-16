import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../theme/app_colors.dart';
import '../services/connection_service.dart';

/// Global floating Emergency Stop button, visible on all tabs.
class EstopFab extends StatefulWidget {
  final ConnectionService connection;

  const EstopFab({super.key, required this.connection});

  @override
  State<EstopFab> createState() => _EstopFabState();
}

class _EstopFabState extends State<EstopFab> with SingleTickerProviderStateMixin {
  late AnimationController _pulseController;

  @override
  void initState() {
    super.initState();
    _pulseController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1000),
    )..repeat(reverse: true);
    widget.connection.addListener(_onChanged);
  }

  @override
  void didUpdateWidget(covariant EstopFab oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.connection != widget.connection) {
      oldWidget.connection.removeListener(_onChanged);
      widget.connection.addListener(_onChanged);
    }
  }

  @override
  void dispose() {
    widget.connection.removeListener(_onChanged);
    _pulseController.dispose();
    super.dispose();
  }

  void _onChanged() {
    if (mounted) setState(() {});
  }

  void _toggle() {
    HapticFeedback.heavyImpact();
    if (widget.connection.emergencyStopped) {
      widget.connection.releaseEmergencyStop();
    } else {
      widget.connection.emergencyStop();
    }
  }

  @override
  Widget build(BuildContext context) {
    final active = widget.connection.emergencyStopped;
    final connected = widget.connection.isConnected;

    if (!connected) return const SizedBox.shrink();

    return AnimatedBuilder(
      animation: _pulseController,
      builder: (context, _) {
        final pulseValue = active ? _pulseController.value : 0.3;
        return GestureDetector(
          onTap: _toggle,
          child: Container(
            width: 64,
            height: 64,
            decoration: BoxDecoration(
              shape: BoxShape.circle,
              color: active ? AppColors.warning : AppColors.danger,
              boxShadow: [
                BoxShadow(
                  color: (active ? AppColors.warning : AppColors.danger)
                      .withValues(alpha: 0.3 + pulseValue * 0.3),
                  blurRadius: 12 + pulseValue * 8,
                  spreadRadius: pulseValue * 4,
                ),
              ],
            ),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(
                  active ? Icons.play_arrow_rounded : Icons.stop_rounded,
                  size: 28,
                  color: Colors.white,
                ),
                Text(
                  active ? 'GO' : 'STOP',
                  style: const TextStyle(
                    fontSize: 8,
                    fontWeight: FontWeight.w800,
                    color: Colors.white,
                    letterSpacing: 1,
                  ),
                ),
              ],
            ),
          ),
        );
      },
    );
  }
}
