import 'dart:math' as math;
import 'package:flutter/material.dart';
import '../theme/app_colors.dart';

/// Custom-painted AutoNexa logo accurately matching the brand identity:
/// Red crescent (left), black crescent (right), white inner circle,
/// triangular node network with "A" in the center.
class AutoNexaLogo extends StatelessWidget {
  final double size;
  final bool showText;

  const AutoNexaLogo({super.key, this.size = 48, this.showText = false});

  @override
  Widget build(BuildContext context) {
    final isDark = Theme.of(context).brightness == Brightness.dark;

    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        CustomPaint(
          size: Size(size, size),
          painter: _LogoPainter(isDark: isDark),
        ),
        if (showText) ...[
          SizedBox(height: size * 0.12),
          Text(
            'AutoNexa',
            style: TextStyle(
              fontSize: size * 0.24,
              fontWeight: FontWeight.w800,
              color: isDark ? AppColors.darkTextPrimary : AppColors.lightTextPrimary,
              letterSpacing: -0.5,
            ),
          ),
        ],
      ],
    );
  }
}

class _LogoPainter extends CustomPainter {
  final bool isDark;

  _LogoPainter({required this.isDark});

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final r = size.width / 2;

    // ── Outer ring (thick arc crescents) ──
    final ringWidth = r * 0.22;
    final outerR = r - ringWidth / 2;

    // Red crescent on the left (~200° arc)
    final redPaint = Paint()
      ..color = AppColors.brand
      ..style = PaintingStyle.stroke
      ..strokeWidth = ringWidth
      ..strokeCap = StrokeCap.round;

    canvas.drawArc(
      Rect.fromCircle(center: center, radius: outerR),
      math.pi * 0.72,   // start angle
      math.pi * 1.1,    // sweep angle (~200°)
      false,
      redPaint,
    );

    // Black/dark crescent on the right (~170° arc)
    final darkPaint = Paint()
      ..color = isDark ? const Color(0xFF303030) : const Color(0xFF1A1A1A)
      ..style = PaintingStyle.stroke
      ..strokeWidth = ringWidth
      ..strokeCap = StrokeCap.round;

    canvas.drawArc(
      Rect.fromCircle(center: center, radius: outerR),
      -math.pi * 0.30,  // start angle
      math.pi * 0.95,   // sweep angle (~170°)
      false,
      darkPaint,
    );

    // ── Inner white/light circle ──
    final innerR = r * 0.60;
    final innerPaint = Paint()
      ..color = isDark ? const Color(0xFFEEEEEE) : Colors.white
      ..style = PaintingStyle.fill;
    canvas.drawCircle(center, innerR, innerPaint);

    // Subtle ring border
    final innerRingPaint = Paint()
      ..color = (isDark ? Colors.white : Colors.black).withValues(alpha: 0.06)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 0.5;
    canvas.drawCircle(center, innerR, innerRingPaint);

    // ── Network triangle (3 nodes + connections) ──
    final nodeR = r * 0.08;
    final triR = innerR * 0.52;
    final triCenter = Offset(center.dx - r * 0.01, center.dy + r * 0.01);

    // Triangle vertices (slightly CCW rotated to match logo)
    final nodes = <Offset>[];
    for (int i = 0; i < 3; i++) {
      final angle = -math.pi / 2 + (i * 2 * math.pi / 3) - 0.10;
      nodes.add(Offset(
        triCenter.dx + triR * math.cos(angle),
        triCenter.dy + triR * math.sin(angle),
      ));
    }

    // Connection lines
    final linePaint = Paint()
      ..color = isDark ? const Color(0xFF1A1A1A) : const Color(0xFF222222)
      ..strokeWidth = r * 0.055
      ..strokeCap = StrokeCap.round
      ..style = PaintingStyle.stroke;

    for (int i = 0; i < 3; i++) {
      for (int j = i + 1; j < 3; j++) {
        canvas.drawLine(nodes[i], nodes[j], linePaint);
      }
    }

    // Node circles (filled)
    final nodePaint = Paint()
      ..color = isDark ? const Color(0xFF1A1A1A) : const Color(0xFF222222)
      ..style = PaintingStyle.fill;

    for (final node in nodes) {
      canvas.drawCircle(node, nodeR, nodePaint);
    }

    // ── "A" letter in the triangle center ──
    final textPainter = TextPainter(
      text: TextSpan(
        text: 'A',
        style: TextStyle(
          fontSize: r * 0.34,
          fontWeight: FontWeight.w900,
          color: isDark ? const Color(0xFF1A1A1A) : const Color(0xFF222222),
          height: 1.0,
        ),
      ),
      textDirection: TextDirection.ltr,
    );
    textPainter.layout();
    textPainter.paint(
      canvas,
      Offset(
        triCenter.dx - textPainter.width / 2 + r * 0.01,
        triCenter.dy - textPainter.height / 2 + r * 0.05,
      ),
    );
  }

  @override
  bool shouldRepaint(covariant _LogoPainter oldDelegate) =>
      oldDelegate.isDark != isDark;
}
