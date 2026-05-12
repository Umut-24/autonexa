import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import '../services/connection_service.dart';
import '../widgets/glass_card.dart';
import '../widgets/marker_chip.dart';

/// Live camera feed with ArUco marker overlay.
/// Accessed via the "More" bottom sheet.
class CameraTab extends StatelessWidget {
  const CameraTab({super.key});

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final conn = context.watch<ConnectionService>();
    final markers = conn.robotStatus.markers.values.toList();

    return Scaffold(
      backgroundColor: colors.background,
      appBar: AppBar(
        backgroundColor: colors.surface,
        title: const Text('Camera', style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600)),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_rounded),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: SafeArea(
        child: Column(
          children: [
            // Camera feed
            Expanded(
              child: Container(
                margin: const EdgeInsets.all(16),
                decoration: BoxDecoration(
                  borderRadius: BorderRadius.circular(16),
                  border: Border.all(color: colors.border),
                  color: colors.surface,
                ),
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(16),
                  child: conn.isConnected
                      ? Image.network(
                          conn.videoFeedUrl,
                          fit: BoxFit.contain,
                          errorBuilder: (_, __, ___) => _placeholder('Camera stream unavailable', colors),
                          loadingBuilder: (_, child, loadingProgress) {
                            if (loadingProgress == null) return child;
                            return Center(
                              child: CircularProgressIndicator(color: colors.accent),
                            );
                          },
                        )
                      : _placeholder('Connect to view camera feed', colors),
                ),
              ),
            ),

            // Detected markers
            if (markers.isNotEmpty) ...[
              Padding(
                padding: const EdgeInsets.fromLTRB(20, 0, 20, 8),
                child: Align(
                  alignment: Alignment.centerLeft,
                  child: Text(
                    'DETECTED MARKERS',
                    style: TextStyle(
                      fontSize: 11,
                      fontWeight: FontWeight.w700,
                      letterSpacing: 1.2,
                      color: colors.textTertiary,
                    ),
                  ),
                ),
              ),
              SizedBox(
                height: 68,
                child: ListView.separated(
                  scrollDirection: Axis.horizontal,
                  padding: const EdgeInsets.symmetric(horizontal: 16),
                  itemCount: markers.length,
                  separatorBuilder: (_, __) => const SizedBox(width: 8),
                  itemBuilder: (_, i) => MarkerChip(marker: markers[i]),
                ),
              ),
              const SizedBox(height: 8),
            ],

            // Telemetry bar
            GlassCard(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
              child: Row(
                children: [
                  _miniStat('ID', markers.isNotEmpty ? '${markers.first.id}' : '-', colors),
                  _miniStat('Dist', markers.isNotEmpty
                      ? '${markers.first.distance.toStringAsFixed(2)}m' : '-', colors),
                  _miniStat('Bearing', markers.isNotEmpty
                      ? '${markers.first.bearing.toStringAsFixed(0)}°' : '-', colors),
                ],
              ),
            ),
            const SizedBox(height: 8),
          ],
        ),
      ),
    );
  }

  Widget _miniStat(String label, String value, ResolvedColors colors) {
    return Expanded(
      child: Column(
        children: [
          Text(label, style: TextStyle(fontSize: 10, color: colors.textSecondary)),
          const SizedBox(height: 2),
          Text(value,
              style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600,
                  fontFamily: 'monospace')),
        ],
      ),
    );
  }

  Widget _placeholder(String message, ResolvedColors colors) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(Icons.videocam_off_rounded, size: 48,
              color: colors.textSecondary.withValues(alpha: 0.4)),
          const SizedBox(height: 14),
          Text(message,
              style: TextStyle(fontSize: 14, color: colors.textSecondary),
              textAlign: TextAlign.center),
        ],
      ),
    );
  }
}
