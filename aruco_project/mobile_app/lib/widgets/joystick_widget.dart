import 'dart:math';
import 'package:flutter/material.dart';

/// Callback signature for joystick movement.
/// [x] ranges from -1.0 (full left) to 1.0 (full right).
/// [y] ranges from -1.0 (full backward) to 1.0 (full forward).
typedef JoystickCallback = void Function(double x, double y);

/// A virtual joystick widget with a circular base and a draggable knob.
/// Touch-optimized: responds to taps anywhere in the base area, not just on the knob.
class VirtualJoystick extends StatefulWidget {
  final double size;
  final JoystickCallback onMove;
  final JoystickCallback? onRelease;
  final Color baseColor;
  final Color knobColor;
  final Color accentColor;

  const VirtualJoystick({
    super.key,
    this.size = 220,
    required this.onMove,
    this.onRelease,
    this.baseColor = const Color(0xFF1A1A2E),
    this.knobColor = const Color(0xFFE94560),
    this.accentColor = const Color(0xFF0F3460),
  });

  @override
  State<VirtualJoystick> createState() => _VirtualJoystickState();
}

class _VirtualJoystickState extends State<VirtualJoystick>
    with SingleTickerProviderStateMixin {
  double _knobX = 0;
  double _knobY = 0;
  bool _isDragging = false;
  late AnimationController _returnController;
  late Animation<Offset> _returnAnimation;

  double get _radius => widget.size / 2;
  double get _knobRadius => widget.size * 0.18;

  @override
  void initState() {
    super.initState();
    _returnController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 120),
    );
    _returnController.addListener(() {
      setState(() {
        _knobX = _returnAnimation.value.dx;
        _knobY = _returnAnimation.value.dy;
      });
    });
  }

  @override
  void dispose() {
    _returnController.dispose();
    super.dispose();
  }

  void _handleDrag(Offset localPosition) {
    final center = Offset(_radius, _radius);
    final delta = localPosition - center;
    final maxDist = _radius - _knobRadius;

    double dist = delta.distance;
    double clampedDist = dist.clamp(0, maxDist);

    double angle = atan2(delta.dy, delta.dx);
    _knobX = clampedDist * cos(angle);
    _knobY = clampedDist * sin(angle);

    // Normalize to [-1, 1], invert Y so up = positive
    double normX = _knobX / maxDist;
    double normY = -_knobY / maxDist;

    setState(() {});
    widget.onMove(normX, normY);
  }

  void _handleRelease() {
    _returnAnimation = Tween<Offset>(
      begin: Offset(_knobX, _knobY),
      end: Offset.zero,
    ).animate(CurvedAnimation(
      parent: _returnController,
      curve: Curves.easeOutCubic,
    ));
    _returnController.forward(from: 0);

    _isDragging = false;
    widget.onRelease?.call(0, 0);
    widget.onMove(0, 0);
  }

  @override
  Widget build(BuildContext context) {
    // Use a Listener for immediate pointer events (no gesture arena delay)
    return SizedBox(
      width: widget.size,
      height: widget.size,
      child: Listener(
        behavior: HitTestBehavior.opaque,
        onPointerDown: (event) {
          _isDragging = true;
          _returnController.stop();
          _handleDrag(event.localPosition);
        },
        onPointerMove: (event) {
          if (_isDragging) {
            _handleDrag(event.localPosition);
          }
        },
        onPointerUp: (_) => _handleRelease(),
        onPointerCancel: (_) => _handleRelease(),
        child: CustomPaint(
          painter: _JoystickPainter(
            knobX: _knobX,
            knobY: _knobY,
            baseColor: widget.baseColor,
            knobColor: widget.knobColor,
            accentColor: widget.accentColor,
            knobRadius: _knobRadius,
            isDragging: _isDragging,
          ),
        ),
      ),
    );
  }
}

class _JoystickPainter extends CustomPainter {
  final double knobX;
  final double knobY;
  final Color baseColor;
  final Color knobColor;
  final Color accentColor;
  final double knobRadius;
  final bool isDragging;

  _JoystickPainter({
    required this.knobX,
    required this.knobY,
    required this.baseColor,
    required this.knobColor,
    required this.accentColor,
    required this.knobRadius,
    required this.isDragging,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2;

    // Outer ring glow
    final glowPaint = Paint()
      ..color = knobColor.withOpacity(0.15)
      ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 20);
    canvas.drawCircle(center, radius, glowPaint);

    // Base circle
    final basePaint = Paint()
      ..shader = RadialGradient(
        colors: [
          baseColor.withOpacity(0.9),
          baseColor,
        ],
      ).createShader(Rect.fromCircle(center: center, radius: radius));
    canvas.drawCircle(center, radius, basePaint);

    // Outer ring border
    final borderPaint = Paint()
      ..color = accentColor.withOpacity(0.6)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2.5;
    canvas.drawCircle(center, radius - 1, borderPaint);

    // Crosshair lines
    final crossPaint = Paint()
      ..color = accentColor.withOpacity(0.25)
      ..strokeWidth = 1;
    canvas.drawLine(
      Offset(center.dx, center.dy - radius * 0.6),
      Offset(center.dx, center.dy + radius * 0.6),
      crossPaint,
    );
    canvas.drawLine(
      Offset(center.dx - radius * 0.6, center.dy),
      Offset(center.dx + radius * 0.6, center.dy),
      crossPaint,
    );

    // Center dot
    canvas.drawCircle(
        center, 4, Paint()..color = accentColor.withOpacity(0.4));

    // Direction arrows
    _drawDirectionArrow(canvas, center, radius, 0, accentColor);
    _drawDirectionArrow(canvas, center, radius, pi, accentColor);
    _drawDirectionArrow(canvas, center, radius, -pi / 2, accentColor);
    _drawDirectionArrow(canvas, center, radius, pi / 2, accentColor);

    // Knob position
    final knobCenter = Offset(center.dx + knobX, center.dy + knobY);

    // Knob shadow
    canvas.drawCircle(
      knobCenter + const Offset(2, 2),
      knobRadius,
      Paint()
        ..color = Colors.black.withOpacity(0.4)
        ..maskFilter = const MaskFilter.blur(BlurStyle.normal, 8),
    );

    // Knob gradient
    canvas.drawCircle(
      knobCenter,
      knobRadius,
      Paint()
        ..shader = RadialGradient(
          colors: [
            isDragging ? knobColor : knobColor.withOpacity(0.85),
            isDragging
                ? knobColor.withOpacity(0.8)
                : knobColor.withOpacity(0.65),
          ],
          center: const Alignment(-0.3, -0.3),
        ).createShader(Rect.fromCircle(center: knobCenter, radius: knobRadius)),
    );

    // Knob inner highlight
    canvas.drawCircle(
      knobCenter,
      knobRadius * 0.7,
      Paint()
        ..color = Colors.white.withOpacity(isDragging ? 0.25 : 0.15)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 1.5,
    );

    // Knob outer ring
    canvas.drawCircle(
      knobCenter,
      knobRadius,
      Paint()
        ..color = knobColor.withOpacity(isDragging ? 1.0 : 0.6)
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2,
    );
  }

  void _drawDirectionArrow(
      Canvas canvas, Offset center, double radius, double angle, Color color) {
    final arrowDist = radius * 0.82;
    final arrowSize = 5.0;
    final arrowCenter = Offset(
      center.dx + arrowDist * cos(angle),
      center.dy + arrowDist * sin(angle),
    );

    final path = Path();
    path.moveTo(
      arrowCenter.dx + arrowSize * cos(angle),
      arrowCenter.dy + arrowSize * sin(angle),
    );
    path.lineTo(
      arrowCenter.dx + arrowSize * cos(angle + 2.4),
      arrowCenter.dy + arrowSize * sin(angle + 2.4),
    );
    path.lineTo(
      arrowCenter.dx + arrowSize * cos(angle - 2.4),
      arrowCenter.dy + arrowSize * sin(angle - 2.4),
    );
    path.close();

    canvas.drawPath(path, Paint()..color = color.withOpacity(0.5));
  }

  @override
  bool shouldRepaint(covariant _JoystickPainter oldDelegate) {
    return oldDelegate.knobX != knobX ||
        oldDelegate.knobY != knobY ||
        oldDelegate.isDragging != isDragging;
  }
}
