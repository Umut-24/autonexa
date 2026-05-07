import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/connection_service.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';

/// Generic parameter tuner — pick one of the whitelisted ROS2 nodes, see its
/// declared parameters, edit numeric / bool ones inline, and POST changes via
/// the bridge's /api/params endpoint. Successful edits persist to
/// ~/.autonexa/runtime_overrides.yaml.
class ParamTunerDialog extends StatefulWidget {
  const ParamTunerDialog({super.key});

  static Future<void> show(BuildContext context) =>
      showDialog(context: context, builder: (_) => const ParamTunerDialog());

  @override
  State<ParamTunerDialog> createState() => _ParamTunerDialogState();
}

const _whitelist = <String, String>{
  '/nav2_pico_bridge': 'Pico bridge',
  '/controller_server': 'Nav2 Controller (DWB)',
  '/planner_server': 'Nav2 Planner',
  '/velocity_smoother': 'Velocity Smoother',
  '/global_costmap/global_costmap': 'Global Costmap',
  '/local_costmap/local_costmap': 'Local Costmap',
};

// Common params worth editing per node — not exhaustive; "show all" toggle
// reveals the full list returned by ListParameters. Filter is just to keep
// the small-screen UX usable.
const _quickParams = <String, List<String>>{
  '/nav2_pico_bridge': [
    'vx_polarity', 'servo_polarity',
    'max_vx_mps', 'max_wz_radps', 'max_steer_rate_radps',
    'min_vx_creep',
  ],
  '/controller_server': [
    'FollowPath.max_vel_x', 'FollowPath.max_vel_theta',
    'FollowPath.min_vel_x',
  ],
  '/velocity_smoother': ['max_velocity', 'max_accel'],
  '/global_costmap/global_costmap': [
    'inflation_layer.inflation_radius',
    'inflation_layer.cost_scaling_factor',
  ],
  '/local_costmap/local_costmap': [
    'inflation_layer.inflation_radius',
    'inflation_layer.cost_scaling_factor',
  ],
  '/planner_server': [],
};

class _ParamTunerDialogState extends State<ParamTunerDialog> {
  String _node = '/nav2_pico_bridge';
  bool _loading = false;
  bool _showAll = false;
  Map<String, dynamic> _values = {};
  List<String> _allNames = [];
  String? _error;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => _refresh());
  }

  Future<void> _refresh() async {
    setState(() {
      _loading = true;
      _error = null;
    });
    final conn = context.read<ConnectionService>();
    final res = await conn.listParams(_node);
    if (!mounted) return;
    if (res.isEmpty) {
      setState(() {
        _loading = false;
        _error = 'Node not reachable or has no parameters.';
        _values = {};
        _allNames = [];
      });
      return;
    }
    setState(() {
      _values = (res['params'] as Map?)?.cast<String, dynamic>() ?? {};
      _allNames = ((res['names'] as List?) ?? []).map((e) => e.toString()).toList();
      _loading = false;
    });
  }

  Future<void> _setOne(String name, dynamic value) async {
    final conn = context.read<ConnectionService>();
    final res = await conn.setParams(_node, {name: value});
    if (!mounted) return;
    if (res[name] == true) {
      setState(() => _values[name] = value);
      ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('$name = $value applied')));
    } else {
      ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Failed to apply $name')));
    }
  }

  List<String> _visibleNames() {
    if (_showAll) return _allNames;
    final quick = _quickParams[_node] ?? const <String>[];
    return [for (final n in quick) if (_values.containsKey(n)) n];
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    return Dialog(
      backgroundColor: colors.surface,
      child: ConstrainedBox(
        constraints: const BoxConstraints(maxWidth: 480, maxHeight: 600),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 14, 8, 4),
              child: Row(children: [
                Text('Parameter tuner',
                    style: TextStyle(fontSize: 16, fontWeight: FontWeight.w700,
                        color: colors.textPrimary)),
                const Spacer(),
                IconButton(
                  icon: const Icon(Icons.close_rounded),
                  onPressed: () => Navigator.pop(context),
                ),
              ]),
            ),
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16),
              child: DropdownButtonFormField<String>(
                initialValue: _node,
                isExpanded: true,
                items: _whitelist.entries
                    .map((e) => DropdownMenuItem(value: e.key, child: Text(e.value)))
                    .toList(),
                onChanged: (v) {
                  if (v == null) return;
                  setState(() => _node = v);
                  _refresh();
                },
              ),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 4, 16, 4),
              child: Row(children: [
                TextButton.icon(
                  onPressed: _refresh,
                  icon: const Icon(Icons.refresh_rounded, size: 16),
                  label: const Text('Refresh'),
                ),
                const Spacer(),
                Switch(
                  value: _showAll,
                  onChanged: (v) => setState(() => _showAll = v),
                ),
                Text('show all (${_allNames.length})',
                    style: TextStyle(fontSize: 11, color: colors.textTertiary)),
              ]),
            ),
            const Divider(height: 1),
            Expanded(
              child: _loading
                  ? const Center(child: CircularProgressIndicator())
                  : _error != null
                      ? Center(child: Padding(
                          padding: const EdgeInsets.all(24),
                          child: Text(_error!, textAlign: TextAlign.center,
                              style: TextStyle(color: colors.textTertiary))))
                      : ListView(
                          children: _visibleNames().map((n) => _paramRow(n, colors)).toList(),
                        ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _paramRow(String name, ResolvedColors colors) {
    final v = _values[name];
    if (v == null) {
      return ListTile(
        title: Text(name, style: const TextStyle(fontSize: 12, fontFamily: 'monospace')),
        subtitle: Text('(unset on this node)', style: TextStyle(fontSize: 11, color: colors.textTertiary)),
      );
    }
    if (v is bool) {
      return SwitchListTile(
        title: Text(name, style: const TextStyle(fontSize: 12, fontFamily: 'monospace')),
        value: v,
        onChanged: (newV) => _setOne(name, newV),
      );
    }
    if (v is num) {
      // Use a simple TextFormField — sliders need bounds we don't have a-priori.
      final ctrl = TextEditingController(text: v.toString());
      return ListTile(
        title: Text(name, style: const TextStyle(fontSize: 12, fontFamily: 'monospace')),
        subtitle: Row(children: [
          Expanded(child: TextField(
            controller: ctrl,
            keyboardType: const TextInputType.numberWithOptions(decimal: true, signed: true),
            decoration: const InputDecoration(isDense: true, contentPadding: EdgeInsets.symmetric(vertical: 6)),
          )),
          IconButton(
            icon: const Icon(Icons.check_rounded, size: 18),
            onPressed: () {
              final parsed = v is int ? int.tryParse(ctrl.text) : double.tryParse(ctrl.text);
              if (parsed != null) _setOne(name, parsed);
            },
          ),
        ]),
      );
    }
    if (v is List) {
      return ListTile(
        title: Text(name, style: const TextStyle(fontSize: 12, fontFamily: 'monospace')),
        subtitle: Text(v.toString(),
            style: TextStyle(fontSize: 11, fontFamily: 'monospace', color: colors.textSecondary)),
        trailing: Text('list',
            style: TextStyle(fontSize: 10, color: colors.textTertiary)),
      );
    }
    return ListTile(
      title: Text(name, style: const TextStyle(fontSize: 12, fontFamily: 'monospace')),
      subtitle: Text(v.toString(),
          style: TextStyle(fontSize: 11, fontFamily: 'monospace', color: colors.textSecondary)),
    );
  }
}
