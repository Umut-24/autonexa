import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../services/connection_service.dart';
import '../theme/app_colors.dart';
import '../theme/theme_provider.dart';
import 'param_descriptions.dart';

/// Generic parameter tuner — pick one of the whitelisted ROS2 nodes, see its
/// declared parameters grouped by category with Turkish descriptions, edit
/// numeric / bool / list parameters inline, and POST changes via the bridge's
/// /api/params endpoint. Successful edits persist to
/// ~/.autonexa/runtime_overrides.yaml.
///
/// UX:
///   - Header dropdown: which node to tune
///   - Search filter: free-text match on param names + Turkish labels
///   - "Show all" switch: reveal everything ListParameters returns (advanced)
///   - Each card: Türkçe etiket, monospace teknik adı, editor, açıklama,
///     etki ipucu ve tipik aralık.
class ParamTunerDialog extends StatefulWidget {
  const ParamTunerDialog({super.key});

  static Future<void> show(BuildContext context) =>
      showDialog(context: context, builder: (_) => const ParamTunerDialog());

  @override
  State<ParamTunerDialog> createState() => _ParamTunerDialogState();
}

const _whitelist = <String, String>{
  '/nav2_pico_bridge': 'Pico bridge',
  '/controller_server': 'Nav2 Controller (RPP)',
  '/planner_server': 'Nav2 Planner',
  '/velocity_smoother': 'Velocity Smoother',
  '/global_costmap/global_costmap': 'Global Costmap',
  '/local_costmap/local_costmap': 'Local Costmap',
};

// "Quick" params — bunlar her zaman gösterilir (varsayılan view).
// "Show all" toggle ListParameters'tan dönen tüm parametreleri açar.
// Listeyi paramMetadata'daki tüm anahtarlarla genişlettim — tuning'de
// önemli her parametre buradan erişilebilir olsun.
const _quickParams = <String, List<String>>{
  '/nav2_pico_bridge': [
    'vx_polarity',
    'servo_polarity',
    'reverse_steer_polarity',
    'max_vx_mps',
    'max_wz_radps',
    'max_ax_mps2',
    'max_aw_radps2',
    'max_steer_rate_radps',
    'min_vx_creep',
    'servo_center_us',
    'servo_us_min',
    'servo_us_max',
  ],
  '/controller_server': [
    'FollowPath.desired_linear_vel',
    'FollowPath.lookahead_dist',
    'FollowPath.min_lookahead_dist',
    'FollowPath.max_lookahead_dist',
    'FollowPath.lookahead_time',
    'FollowPath.curvature_lookahead_dist',
    'FollowPath.regulated_linear_scaling_min_speed',
    'FollowPath.regulated_linear_scaling_min_radius',
    'FollowPath.cost_scaling_dist',
    'FollowPath.cost_scaling_gain',
    'FollowPath.max_allowed_time_to_collision_up_to_carrot',
    'FollowPath.approach_velocity_scaling_dist',
    'FollowPath.min_approach_linear_velocity',
    'FollowPath.use_rotate_to_heading',
    'FollowPath.allow_reversing',
    'general_goal_checker.xy_goal_tolerance',
    'general_goal_checker.yaw_goal_tolerance',
  ],
  '/planner_server': [
    'GridBased.minimum_turning_radius',
    'GridBased.reverse_penalty',
    'GridBased.change_penalty',
    'GridBased.non_straight_penalty',
    'GridBased.cost_penalty',
    'GridBased.analytic_expansion_ratio',
    'GridBased.max_planning_time',
  ],
  '/velocity_smoother': [
    'max_velocity',
    'min_velocity',
    'max_accel',
    'max_decel',
  ],
  '/global_costmap/global_costmap': [
    'inflation_layer.inflation_radius',
    'inflation_layer.cost_scaling_factor',
  ],
  '/local_costmap/local_costmap': [
    'inflation_layer.inflation_radius',
    'inflation_layer.cost_scaling_factor',
  ],
};

class _ParamTunerDialogState extends State<ParamTunerDialog> {
  String _node = '/nav2_pico_bridge';
  bool _loading = false;
  bool _showAll = false;
  String _filter = '';
  Map<String, dynamic> _values = {};
  List<String> _allNames = [];
  String? _error;
  // Param adı → controller. Kart yeniden inşa edildiğinde controller'ı
  // koruyalım — kullanıcının yazdığı henüz commit edilmemiş metin gitmesin.
  final Map<String, TextEditingController> _controllers = {};

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) => _refresh());
  }

  @override
  void dispose() {
    for (final c in _controllers.values) {
      c.dispose();
    }
    super.dispose();
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
        _error = 'Node erişilemiyor veya hiç parametre yok.';
        _values = {};
        _allNames = [];
      });
      return;
    }
    final newValues =
        (res['params'] as Map?)?.cast<String, dynamic>() ?? <String, dynamic>{};
    // Controller'ı sadece yeni gelen değer eski controller'ın text'inden
    // farklıysa update et — kullanıcı düzenlemekte olduğu inputu kaybetmesin.
    for (final entry in newValues.entries) {
      final v = entry.value;
      if (v is num || v is List) {
        final newText = v is List ? _listToCsv(v) : v.toString();
        final existing = _controllers[entry.key];
        if (existing == null) {
          _controllers[entry.key] = TextEditingController(text: newText);
        } else if (existing.text.isEmpty || _looksLikeServerValue(existing.text, _values[entry.key])) {
          // Kullanıcı düzenlemiyorsa server'dan gelen değeri yansıt.
          existing.text = newText;
        }
      }
    }
    setState(() {
      _values = newValues;
      _allNames =
          ((res['names'] as List?) ?? []).map((e) => e.toString()).toList();
      _loading = false;
    });
  }

  /// Heuristic: controller'daki text, daha önce server'dan gelen değere
  /// eşitse "kullanıcı düzenlemiyor" diye varsay; yeni server değerini güvenle
  /// yansıtabiliriz. Aksi halde kullanıcının düzenlemesini bekleyelim.
  bool _looksLikeServerValue(String text, dynamic prevValue) {
    if (prevValue == null) return true;
    final prevText = prevValue is List ? _listToCsv(prevValue) : prevValue.toString();
    return text.trim() == prevText.trim();
  }

  Future<void> _setOne(String name, dynamic value) async {
    final conn = context.read<ConnectionService>();
    final res = await conn.setParams(_node, {name: value});
    if (!mounted) return;
    final r = res[name];
    if (r != null && r.ok) {
      setState(() => _values[name] = value);
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        backgroundColor: Colors.green.shade700,
        duration: const Duration(milliseconds: 1500),
        content: Text('$name = $value uygulandı'),
      ));
    } else {
      // Show the bridge's actual rejection reason instead of a generic
      // failure message — typical reasons: "parameter not declared",
      // "Modifying parameter is not allowed", "out of range".
      final reason = (r?.reason.isNotEmpty ?? false)
          ? r!.reason
          : '(no reason returned)';
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        backgroundColor: Colors.red.shade700,
        duration: const Duration(seconds: 4),
        content: Text('Uygulanamadı: $name → $reason'),
      ));
    }
  }

  /// Hangi parametreleri göstermeli? quickParams ∪ filter sonucu, sırasıyla
  /// kategoriye göre gruplanmış olarak döndürür.
  Map<String, List<String>> _groupedVisibleNames() {
    final names = _showAll
        ? _allNames
        : [
            for (final n in _quickParams[_node] ?? const <String>[])
              if (_values.containsKey(n)) n
          ];
    final filter = _filter.trim().toLowerCase();
    final filtered = filter.isEmpty
        ? names
        : names.where((n) {
            if (n.toLowerCase().contains(filter)) return true;
            final m = metaFor(_node, n);
            if (m == null) return false;
            return m.label.toLowerCase().contains(filter) ||
                m.description.toLowerCase().contains(filter) ||
                m.category.toLowerCase().contains(filter);
          }).toList();

    // Kategoriye göre grupla. Metadata yoksa "Diğer" altında topla.
    final groups = <String, List<String>>{};
    for (final n in filtered) {
      final cat = metaFor(_node, n)?.category ?? 'Diğer';
      groups.putIfAbsent(cat, () => []).add(n);
    }
    return groups;
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final groups = _groupedVisibleNames();
    final totalShown = groups.values.fold<int>(0, (s, l) => s + l.length);
    return Dialog(
      backgroundColor: colors.surface,
      child: ConstrainedBox(
        constraints: const BoxConstraints(maxWidth: 540, maxHeight: 720),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            // ── Header ─────────────────────────────────────────────────
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 14, 8, 4),
              child: Row(children: [
                Icon(Icons.tune_rounded, size: 20, color: colors.accent),
                const SizedBox(width: 8),
                Text('Parametre Tuning',
                    style: TextStyle(
                        fontSize: 17,
                        fontWeight: FontWeight.w700,
                        color: colors.textPrimary)),
                const Spacer(),
                IconButton(
                  icon: const Icon(Icons.close_rounded),
                  onPressed: () => Navigator.pop(context),
                ),
              ]),
            ),
            // ── Node selector ──────────────────────────────────────────
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 0, 16, 6),
              child: DropdownButtonFormField<String>(
                initialValue: _node,
                isExpanded: true,
                decoration: const InputDecoration(
                  labelText: 'Node',
                  isDense: true,
                  border: OutlineInputBorder(),
                  contentPadding: EdgeInsets.symmetric(
                      horizontal: 12, vertical: 8),
                ),
                items: _whitelist.entries
                    .map((e) => DropdownMenuItem(
                          value: e.key,
                          child: Text('${e.value}  (${e.key})',
                              style: const TextStyle(fontSize: 13)),
                        ))
                    .toList(),
                onChanged: (v) {
                  if (v == null) return;
                  setState(() {
                    _node = v;
                    _filter = '';
                  });
                  _refresh();
                },
              ),
            ),
            // ── Search + show-all toggle ───────────────────────────────
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 4, 16, 4),
              child: Row(children: [
                Expanded(
                  child: TextField(
                    decoration: const InputDecoration(
                      hintText: 'Filtrele (ad / etiket / açıklama)',
                      prefixIcon: Icon(Icons.search_rounded, size: 18),
                      isDense: true,
                      border: OutlineInputBorder(),
                      contentPadding: EdgeInsets.symmetric(
                          horizontal: 8, vertical: 6),
                    ),
                    style: const TextStyle(fontSize: 13),
                    onChanged: (v) => setState(() => _filter = v),
                  ),
                ),
                const SizedBox(width: 8),
                IconButton(
                  tooltip: 'Yenile',
                  icon: const Icon(Icons.refresh_rounded, size: 20),
                  onPressed: _refresh,
                ),
              ]),
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 0, 16, 4),
              child: Row(children: [
                Text(
                    _loading
                        ? '...'
                        : '$totalShown parametre gösteriliyor (toplam ${_allNames.length})',
                    style:
                        TextStyle(fontSize: 11, color: colors.textTertiary)),
                const Spacer(),
                Text('Tümünü göster',
                    style:
                        TextStyle(fontSize: 11, color: colors.textTertiary)),
                Switch(
                  value: _showAll,
                  onChanged: (v) => setState(() => _showAll = v),
                ),
              ]),
            ),
            const Divider(height: 1),
            // ── Param list ─────────────────────────────────────────────
            Expanded(
              child: _loading
                  ? const Center(child: CircularProgressIndicator())
                  : _error != null
                      ? Center(
                          child: Padding(
                              padding: const EdgeInsets.all(24),
                              child: Text(_error!,
                                  textAlign: TextAlign.center,
                                  style:
                                      TextStyle(color: colors.textTertiary))))
                      : totalShown == 0
                          ? Center(
                              child: Padding(
                                  padding: const EdgeInsets.all(24),
                                  child: Text(
                                      _filter.isEmpty
                                          ? 'Bu node için parametre bulunamadı.'
                                          : 'Filtreyle eşleşen parametre yok.',
                                      textAlign: TextAlign.center,
                                      style: TextStyle(
                                          color: colors.textTertiary))))
                          : ListView(
                              padding: const EdgeInsets.symmetric(
                                  horizontal: 12, vertical: 4),
                              children: _buildGroupedList(groups, colors),
                            ),
            ),
            // ── Footer hint ───────────────────────────────────────────
            Container(
              padding: const EdgeInsets.fromLTRB(16, 6, 16, 10),
              decoration: BoxDecoration(
                color: colors.accentSurface,
                border: Border(top: BorderSide(color: colors.accentDim)),
              ),
              child: Row(children: [
                Icon(Icons.info_outline_rounded,
                    size: 14, color: colors.textTertiary),
                const SizedBox(width: 6),
                Expanded(
                    child: Text(
                        'Onaylanan değerler ~/.autonexa/runtime_overrides.yaml\'a yazılır ve relaunch\'tan sonra korunur.',
                        style: TextStyle(
                            fontSize: 11, color: colors.textTertiary))),
              ]),
            ),
          ],
        ),
      ),
    );
  }

  List<Widget> _buildGroupedList(
      Map<String, List<String>> groups, ResolvedColors colors) {
    final widgets = <Widget>[];
    // Stabil kategori sırası: paramMetadata'da ilk geçen sırayla.
    final sortedGroups = groups.entries.toList()
      ..sort((a, b) {
        // "Diğer" en sonda.
        if (a.key == 'Diğer' && b.key != 'Diğer') return 1;
        if (b.key == 'Diğer' && a.key != 'Diğer') return -1;
        return a.key.compareTo(b.key);
      });
    for (final entry in sortedGroups) {
      widgets.add(_categoryHeader(entry.key, entry.value.length, colors));
      for (final n in entry.value) {
        widgets.add(_paramCard(n, colors));
      }
    }
    widgets.add(const SizedBox(height: 8));
    return widgets;
  }

  Widget _categoryHeader(String name, int count, ResolvedColors colors) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(4, 10, 4, 4),
      child: Row(children: [
        Text(name.toUpperCase(),
            style: TextStyle(
                fontSize: 11,
                fontWeight: FontWeight.w700,
                letterSpacing: 0.8,
                color: colors.accent)),
        const SizedBox(width: 6),
        Text('$count',
            style: TextStyle(fontSize: 10, color: colors.textTertiary)),
        const SizedBox(width: 8),
        Expanded(child: Container(height: 1, color: colors.accentDim)),
      ]),
    );
  }

  Widget _paramCard(String name, ResolvedColors colors) {
    final v = _values[name];
    final meta = metaFor(_node, name);

    return Container(
      margin: const EdgeInsets.symmetric(vertical: 3),
      decoration: BoxDecoration(
        color: colors.surfaceLight,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: colors.accentDim.withValues(alpha: 0.4)),
      ),
      child: Padding(
        padding: const EdgeInsets.fromLTRB(10, 8, 10, 8),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            // Header satırı: Türkçe etiket + (varsa birim)
            Row(
              crossAxisAlignment: CrossAxisAlignment.center,
              children: [
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        meta?.label ?? name.split('.').last,
                        style: TextStyle(
                            fontSize: 13,
                            fontWeight: FontWeight.w600,
                            color: colors.textPrimary),
                      ),
                      Text(name,
                          style: TextStyle(
                              fontSize: 10,
                              fontFamily: 'monospace',
                              color: colors.textTertiary)),
                    ],
                  ),
                ),
                if (meta?.unit != null)
                  Container(
                    margin: const EdgeInsets.only(left: 6),
                    padding: const EdgeInsets.symmetric(
                        horizontal: 6, vertical: 2),
                    decoration: BoxDecoration(
                      color: colors.accentDim,
                      borderRadius: BorderRadius.circular(4),
                    ),
                    child: Text(meta!.unit!,
                        style: TextStyle(
                            fontSize: 10,
                            fontWeight: FontWeight.w600,
                            color: colors.accent)),
                  ),
              ],
            ),
            const SizedBox(height: 6),
            // Editör (TextField / Switch / List editor / read-only)
            _buildEditor(name, v, meta, colors),
            // Açıklama (varsa)
            if (meta != null) ...[
              const SizedBox(height: 8),
              Text(meta.description,
                  style: TextStyle(
                      fontSize: 11.5,
                      height: 1.35,
                      color: colors.textSecondary)),
              if (meta.effect != null) ...[
                const SizedBox(height: 4),
                Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
                  Icon(Icons.trending_up_rounded,
                      size: 12, color: colors.accent),
                  const SizedBox(width: 4),
                  Expanded(
                      child: Text(meta.effect!,
                          style: TextStyle(
                              fontSize: 11,
                              fontStyle: FontStyle.italic,
                              color: colors.accent))),
                ]),
              ],
              if (meta.hasRange) ...[
                const SizedBox(height: 3),
                Text(
                    'Tipik aralık: ${_fmt(meta.typicalMin!)} – ${_fmt(meta.typicalMax!)}${meta.unit != null ? " ${meta.unit}" : ""}',
                    style: TextStyle(
                        fontSize: 10.5, color: colors.textTertiary)),
              ],
            ],
          ],
        ),
      ),
    );
  }

  Widget _buildEditor(
      String name, dynamic v, ParamMeta? meta, ResolvedColors colors) {
    if (v == null) {
      return Text('(bu node\'da tanımlı değil)',
          style: TextStyle(
              fontSize: 11,
              fontStyle: FontStyle.italic,
              color: colors.textTertiary));
    }
    if (v is bool) {
      return Row(children: [
        Switch(value: v, onChanged: (newV) => _setOne(name, newV)),
        const SizedBox(width: 8),
        Text(v ? 'aktif (true)' : 'pasif (false)',
            style:
                TextStyle(fontSize: 12, color: colors.textSecondary)),
      ]);
    }
    if (v is num) {
      final ctrl = _controllers[name]!;
      return Row(children: [
        Expanded(
          child: TextField(
            controller: ctrl,
            keyboardType: const TextInputType.numberWithOptions(
                decimal: true, signed: true),
            style: const TextStyle(fontSize: 13, fontFamily: 'monospace'),
            decoration: InputDecoration(
              isDense: true,
              filled: true,
              fillColor: colors.surface,
              border: const OutlineInputBorder(
                  borderRadius: BorderRadius.all(Radius.circular(6))),
              contentPadding: const EdgeInsets.symmetric(
                  horizontal: 10, vertical: 8),
            ),
            onSubmitted: (text) => _commitNumeric(name, text, v),
          ),
        ),
        const SizedBox(width: 6),
        FilledButton.icon(
          style: FilledButton.styleFrom(
            padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
            minimumSize: const Size(0, 36),
            backgroundColor: colors.accent,
          ),
          icon: const Icon(Icons.check_rounded, size: 16),
          label: const Text('Uygula', style: TextStyle(fontSize: 12)),
          onPressed: () => _commitNumeric(name, ctrl.text, v),
        ),
      ]);
    }
    if (v is List) {
      // CSV-style list editing for numeric arrays (max_velocity, max_accel...).
      final ctrl = _controllers[name]!;
      return Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          TextField(
            controller: ctrl,
            inputFormatters: [
              FilteringTextInputFormatter.allow(RegExp(r'[0-9eE\-\+\.\, \[\]]'))
            ],
            style: const TextStyle(fontSize: 13, fontFamily: 'monospace'),
            decoration: InputDecoration(
              isDense: true,
              filled: true,
              fillColor: colors.surface,
              hintText: 'örn: 0.15, 0.0, 0.5',
              border: const OutlineInputBorder(
                  borderRadius: BorderRadius.all(Radius.circular(6))),
              contentPadding: const EdgeInsets.symmetric(
                  horizontal: 10, vertical: 8),
            ),
            onSubmitted: (text) => _commitList(name, text, v),
          ),
          const SizedBox(height: 4),
          Row(children: [
            Text('${v.length} eleman',
                style: TextStyle(
                    fontSize: 10, color: colors.textTertiary)),
            const Spacer(),
            FilledButton.icon(
              style: FilledButton.styleFrom(
                padding: const EdgeInsets.symmetric(
                    horizontal: 12, vertical: 6),
                minimumSize: const Size(0, 32),
                backgroundColor: colors.accent,
              ),
              icon: const Icon(Icons.check_rounded, size: 14),
              label: const Text('Uygula', style: TextStyle(fontSize: 11)),
              onPressed: () => _commitList(name, ctrl.text, v),
            ),
          ]),
        ],
      );
    }
    // Bilinmeyen tip: read-only metin.
    return Text(v.toString(),
        style: TextStyle(
            fontSize: 12,
            fontFamily: 'monospace',
            color: colors.textSecondary));
  }

  void _commitNumeric(String name, String text, num prev) {
    final s = text.trim();
    if (s.isEmpty) return;
    final parsed = prev is int ? int.tryParse(s) : double.tryParse(s);
    if (parsed == null) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        backgroundColor: Colors.orange.shade800,
        content: Text('Sayısal değer parse edilemedi: "$s"'),
      ));
      return;
    }
    _setOne(name, parsed);
  }

  void _commitList(String name, String text, List prev) {
    // "0.15, 0.0, 0.5" veya "[0.15, 0.0, 0.5]" formatlarını parse et.
    final clean = text.replaceAll('[', '').replaceAll(']', '').trim();
    if (clean.isEmpty) return;
    final parts = clean.split(',').map((p) => p.trim()).where((p) => p.isNotEmpty);
    final wantInt = prev.isNotEmpty && prev.first is int;
    final parsed = <num>[];
    for (final p in parts) {
      final v = wantInt ? int.tryParse(p) : double.tryParse(p);
      if (v == null) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(
          backgroundColor: Colors.orange.shade800,
          content: Text('Liste parse hatası: "$p" geçerli sayı değil'),
        ));
        return;
      }
      parsed.add(v);
    }
    if (parsed.length != prev.length) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(
        backgroundColor: Colors.orange.shade800,
        content: Text(
            'Liste uzunluğu uyumsuz: ${parsed.length} verildi, ${prev.length} bekleniyor'),
      ));
      return;
    }
    _setOne(name, parsed);
  }

  String _listToCsv(List v) {
    return v.map((e) => e is double ? _fmt(e) : e.toString()).join(', ');
  }

  String _fmt(num x) {
    if (x is int) return x.toString();
    final d = x.toDouble();
    if (d.abs() >= 100) return d.toStringAsFixed(0);
    if (d.abs() >= 10) return d.toStringAsFixed(1);
    if (d.abs() >= 1) return d.toStringAsFixed(2);
    return d.toStringAsFixed(3);
  }
}
