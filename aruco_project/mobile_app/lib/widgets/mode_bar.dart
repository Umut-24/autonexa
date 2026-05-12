import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/connection_service.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';

/// Persistent bar shown across every primary tab. Surfaces:
///   1. The control mode state machine (AUTO / MANUAL / ESTOP).
///   2. The Nav2 NavigateToPose action status so the user always knows
///      whether a goal is planning, executing, succeeded, or aborted.
///
/// This is the formal AUTO/MANUAL/ESTOP UX called for in
/// CLAUDE.md "Known Open Items" (control-source arbitration).
class ModeBar extends StatelessWidget {
  const ModeBar({super.key});

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final conn = context.watch<ConnectionService>();
    if (!conn.isConnected) return const SizedBox.shrink();

    return Container(
      decoration: BoxDecoration(
        color: colors.surface,
        border: Border(bottom: BorderSide(color: colors.border)),
      ),
      child: SafeArea(
        bottom: false,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(12, 6, 12, 6),
          child: Row(
            children: [
              _segmented(context, conn, colors),
              const SizedBox(width: 12),
              Expanded(child: _navStatusChip(conn, colors)),
            ],
          ),
        ),
      ),
    );
  }

  Widget _segmented(BuildContext context, ConnectionService conn, ResolvedColors colors) {
    final current = conn.mode;
    Widget seg(String label, ControlMode m, Color activeColor) {
      final active = current == m;
      return GestureDetector(
        onTap: () => conn.setMode(m),
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 150),
          padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
          decoration: BoxDecoration(
            color: active ? activeColor : Colors.transparent,
            borderRadius: BorderRadius.circular(7),
          ),
          child: Text(label,
              style: TextStyle(
                fontSize: 11,
                fontWeight: FontWeight.w700,
                letterSpacing: 0.5,
                color: active ? Colors.white : colors.textSecondary,
              )),
        ),
      );
    }

    return Container(
      padding: const EdgeInsets.all(2),
      decoration: BoxDecoration(
        color: colors.surfaceLight,
        borderRadius: BorderRadius.circular(9),
        border: Border.all(color: colors.border),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          seg('AUTO', ControlMode.auto, const Color(0xFF1E88E5)),
          seg('MANUAL', ControlMode.manual, const Color(0xFF43A047)),
          seg('E-STOP', ControlMode.estop, const Color(0xFFE53935)),
        ],
      ),
    );
  }

  Widget _navStatusChip(ConnectionService conn, ResolvedColors colors) {
    final status = conn.navStatus;
    final palette = _statusPalette(status);
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: palette.bg,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: palette.fg.withValues(alpha: 0.4)),
      ),
      child: Row(
        children: [
          Container(
            width: 7, height: 7,
            decoration: BoxDecoration(color: palette.fg, shape: BoxShape.circle),
          ),
          const SizedBox(width: 8),
          Text('Nav2',
              style: TextStyle(
                fontSize: 9,
                fontWeight: FontWeight.w700,
                letterSpacing: 0.6,
                color: colors.textTertiary,
              )),
          const SizedBox(width: 6),
          Flexible(
            child: Text(
              status,
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: TextStyle(
                fontSize: 11,
                fontFamily: 'monospace',
                fontWeight: FontWeight.w700,
                color: palette.fg,
              ),
            ),
          ),
        ],
      ),
    );
  }

  _StatusPalette _statusPalette(String status) {
    switch (status) {
      case 'EXECUTING':
        return const _StatusPalette(Color(0xFF1E88E5), Color(0x1A1E88E5));
      case 'PLANNING':
        return const _StatusPalette(Color(0xFFFB8C00), Color(0x1AFB8C00));
      case 'SUCCEEDED':
        return const _StatusPalette(Color(0xFF43A047), Color(0x1A43A047));
      case 'CANCELED':
      case 'CANCELING':
        return const _StatusPalette(Color(0xFF757575), Color(0x1A757575));
      case 'ABORTED':
        return const _StatusPalette(Color(0xFFE53935), Color(0x1AE53935));
      default:
        return const _StatusPalette(Color(0xFF9E9E9E), Color(0x149E9E9E));
    }
  }
}

class _StatusPalette {
  final Color fg;
  final Color bg;
  const _StatusPalette(this.fg, this.bg);
}
