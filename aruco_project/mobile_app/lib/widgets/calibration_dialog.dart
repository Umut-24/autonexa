import 'package:flutter/material.dart';
import '../services/connection_service.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import 'package:provider/provider.dart';

/// Two-step direction calibration:
///   1. press FORWARD -> bridge drives a tiny vx; user answers "did the
///      robot go forward?". A "no" flips vx_polarity.
///   2. press LEFT -> small wz; "no" flips servo_polarity.
/// Each accepted answer is persisted by the bridge to
/// ~/.autonexa/runtime_overrides.yaml so it survives a relaunch.
class CalibrationDialog extends StatefulWidget {
  const CalibrationDialog({super.key});

  static Future<void> show(BuildContext context) {
    return showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => const CalibrationDialog(),
    );
  }

  @override
  State<CalibrationDialog> createState() => _CalibrationDialogState();
}

enum _Step { intro, forwardPulse, forwardAsk, turnPulse, turnAsk, done }

class _CalibrationDialogState extends State<CalibrationDialog> {
  _Step _step = _Step.intro;
  int? _vxPolarity;
  int? _servoPolarity;
  String _status = '';

  Future<void> _loadCurrent() async {
    final conn = context.read<ConnectionService>();
    final cur = await conn.getCalibration();
    if (!mounted) return;
    var vxPol = cur['vx_polarity'];
    final servoPol = cur['servo_polarity'];
    // If a previous wizard run accidentally flipped vx_polarity to -1 (the
    // user said "didn't move" when the real problem was insufficient PWM,
    // not wrong polarity), reset to a known-good baseline of +1. Manual
    // joystick already works on this chassis with vx_polarity=+1, so this
    // is the safe default. The user can still flip it during the wizard.
    if (vxPol == -1) {
      final result = await conn.calibrateDirection(vxPolarity: 1);
      if (result['vx_polarity'] == true) {
        vxPol = 1;
      }
    }
    if (!mounted) return;
    setState(() {
      _vxPolarity = vxPol;
      _servoPolarity = servoPol;
    });
  }

  @override
  void initState() {
    super.initState();
    _loadCurrent();
  }

  Future<void> _runForwardPulse() async {
    setState(() {
      _step = _Step.forwardPulse;
      _status = 'Driving forward briefly…';
    });
    // 0.20 m/s for 1.2 s — robust against the L298N 60 % PWM deadband on
    // most floor surfaces. See ConnectionService.calibrationPulse comment.
    await context.read<ConnectionService>().calibrationPulse(
        vx: 0.20, wz: 0.0, durationMs: 1200);
    if (!mounted) return;
    setState(() {
      _step = _Step.forwardAsk;
      _status = '';
    });
  }

  Future<void> _answerForward(bool wentForward) async {
    final conn = context.read<ConnectionService>();
    if (!wentForward) {
      // Flip vx polarity.
      final newPol = (_vxPolarity ?? 1) * -1;
      setState(() => _status = 'Flipping vx_polarity → $newPol …');
      final results = await conn.calibrateDirection(vxPolarity: newPol);
      if (results['vx_polarity'] == true) {
        _vxPolarity = newPol;
      } else {
        if (!mounted) return;
        setState(() => _status = 'Failed to apply vx_polarity flip');
        return;
      }
    }
    if (!mounted) return;
    setState(() {
      _step = _Step.turnPulse;
      _status = '';
    });
  }

  Future<void> _runTurnPulse() async {
    setState(() {
      _step = _Step.turnPulse;
      _status = 'Turning left briefly…';
    });
    // Slight forward + wz>0 so Ackermann actually steers (pure pivot is
    // undefined for wheeled-Ackermann; the bridge handles vx≈0 by going
    // to max steer, which is what we want for a calibration check).
    // Bumped to vx=0.18 / wz=0.6 / 1.2 s — enough motion to see a clear
    // left-vs-right delta despite the L298N deadband floor.
    await context.read<ConnectionService>().calibrationPulse(
        vx: 0.18, wz: 0.6, durationMs: 1200);
    if (!mounted) return;
    setState(() {
      _step = _Step.turnAsk;
      _status = '';
    });
  }

  Future<void> _answerTurn(bool wentLeft) async {
    final conn = context.read<ConnectionService>();
    if (!wentLeft) {
      final newPol = (_servoPolarity ?? -1) * -1;
      setState(() => _status = 'Flipping servo_polarity → $newPol …');
      final results = await conn.calibrateDirection(servoPolarity: newPol);
      if (results['servo_polarity'] == true) {
        _servoPolarity = newPol;
      } else {
        if (!mounted) return;
        setState(() => _status = 'Failed to apply servo_polarity flip');
        return;
      }
    }
    if (!mounted) return;
    setState(() {
      _step = _Step.done;
      _status = 'Saved. Polarities will persist across relaunches.';
    });
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    return AlertDialog(
      backgroundColor: colors.surface,
      title: const Text('Calibrate Direction'),
      content: ConstrainedBox(
        constraints: const BoxConstraints(maxWidth: 360, minWidth: 280),
        child: SingleChildScrollView(child: _content(colors)),
      ),
      actions: [
        if (_step == _Step.done)
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Done'),
          )
        else
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Cancel'),
          ),
      ],
    );
  }

  Widget _content(ResolvedColors colors) {
    final pol = 'vx=${_vxPolarity ?? '?'}  servo=${_servoPolarity ?? '?'}';
    final polChip = Padding(
      padding: const EdgeInsets.only(bottom: 8),
      child: Text(pol, style: TextStyle(fontFamily: 'monospace', fontSize: 11, color: colors.textTertiary)),
    );
    switch (_step) {
      case _Step.intro:
        return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          polChip,
          const Text(
            'Two short pulses verify forward direction and steering direction. '
            'Make sure the area is clear; the robot will move a few cm.',
            style: TextStyle(fontSize: 13),
          ),
          const SizedBox(height: 12),
          ElevatedButton.icon(
            icon: const Icon(Icons.play_arrow_rounded),
            label: const Text('Start step 1: forward'),
            style: ElevatedButton.styleFrom(backgroundColor: AppColors.brand),
            onPressed: _runForwardPulse,
          ),
        ]);
      case _Step.forwardPulse:
      case _Step.turnPulse:
        return Column(children: [
          polChip,
          const SizedBox(height: 8),
          const CircularProgressIndicator(),
          const SizedBox(height: 12),
          Text(_status, textAlign: TextAlign.center),
        ]);
      case _Step.forwardAsk:
        return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          polChip,
          const Text('Did the robot move FORWARD?', style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
          const SizedBox(height: 6),
          Text(_status, style: TextStyle(fontSize: 11, color: colors.textTertiary)),
          const SizedBox(height: 14),
          Row(children: [
            Expanded(
              child: ElevatedButton.icon(
                icon: const Icon(Icons.check_rounded),
                label: const Text('Yes'),
                style: ElevatedButton.styleFrom(backgroundColor: AppColors.success),
                onPressed: () => _answerForward(true),
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: ElevatedButton.icon(
                icon: const Icon(Icons.swap_horiz_rounded),
                label: const Text('No, flip'),
                style: ElevatedButton.styleFrom(backgroundColor: AppColors.warning),
                onPressed: () => _answerForward(false),
              ),
            ),
          ]),
          const SizedBox(height: 8),
          // "Didn't move at all" is a deliberately separate path from "No,
          // flip". If you flip vx_polarity when the real issue is insufficient
          // PWM (e.g. carpet), you'll break manual control.
          Row(children: [
            Expanded(
              child: TextButton.icon(
                onPressed: _runForwardPulse,
                icon: const Icon(Icons.replay_rounded, size: 16),
                label: const Text('Didn\'t move — repeat'),
              ),
            ),
          ]),
        ]);
      case _Step.turnAsk:
        return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          polChip,
          const Text('Did the robot turn LEFT?', style: TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
          const SizedBox(height: 6),
          Text(_status, style: TextStyle(fontSize: 11, color: colors.textTertiary)),
          const SizedBox(height: 14),
          Row(children: [
            Expanded(
              child: ElevatedButton.icon(
                icon: const Icon(Icons.check_rounded),
                label: const Text('Yes'),
                style: ElevatedButton.styleFrom(backgroundColor: AppColors.success),
                onPressed: () => _answerTurn(true),
              ),
            ),
            const SizedBox(width: 8),
            Expanded(
              child: ElevatedButton.icon(
                icon: const Icon(Icons.swap_horiz_rounded),
                label: const Text('No, flip'),
                style: ElevatedButton.styleFrom(backgroundColor: AppColors.warning),
                onPressed: () => _answerTurn(false),
              ),
            ),
          ]),
          const SizedBox(height: 8),
          // Same rationale as the forward step — separate "didn't move" from
          // "moved the wrong way" to avoid accidental servo_polarity flips.
          Row(children: [
            Expanded(
              child: TextButton.icon(
                onPressed: _runTurnPulse,
                icon: const Icon(Icons.replay_rounded, size: 16),
                label: const Text('Didn\'t move — repeat'),
              ),
            ),
          ]),
        ]);
      case _Step.done:
        return Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          polChip,
          Row(children: [
            Icon(Icons.check_circle_rounded, color: AppColors.success, size: 28),
            const SizedBox(width: 8),
            const Expanded(child: Text('Calibration complete.')),
          ]),
          const SizedBox(height: 8),
          Text(_status, style: TextStyle(fontSize: 12, color: colors.textTertiary)),
        ]);
    }
  }
}
