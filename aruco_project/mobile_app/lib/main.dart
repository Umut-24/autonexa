import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:webview_flutter/webview_flutter.dart';
import 'package:google_fonts/google_fonts.dart';
import 'control_tab.dart';
import 'lidar_map_view.dart';

// ─── Color Palette ──────────────────────────────────────────────────────────
class AppColors {
  static const background = Color(0xFF0D0D14);
  static const surface = Color(0xFF14141E);
  static const surfaceLight = Color(0xFF1C1C2A);
  static const border = Color(0xFF2A2A3A);
  static const accent = Color(0xFFE94560);
  static const accentDim = Color(0xFF0F3460);
  static const textPrimary = Color(0xFFF0F0F0);
  static const textSecondary = Color(0xFF8888A0);
  static const success = Color(0xFF2ECC71);
  static const warning = Color(0xFFE67E22);
  static const error = Color(0xFFE74C3C);
}

void main() {
  runApp(const AutoNexaApp());
}

class AutoNexaApp extends StatelessWidget {
  const AutoNexaApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'AutoNexa',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        brightness: Brightness.dark,
        scaffoldBackgroundColor: AppColors.background,
        colorScheme: const ColorScheme.dark(
          primary: AppColors.accent,
          surface: AppColors.surface,
        ),
        textTheme: GoogleFonts.interTextTheme(ThemeData.dark().textTheme),
        cardTheme: CardThemeData(
          color: AppColors.surface,
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(14),
            side: const BorderSide(color: AppColors.border, width: 1),
          ),
          elevation: 0,
          margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
        ),
        elevatedButtonTheme: ElevatedButtonThemeData(
          style: ElevatedButton.styleFrom(
            backgroundColor: AppColors.accentDim,
            foregroundColor: Colors.white,
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
            elevation: 0,
            padding: const EdgeInsets.symmetric(vertical: 14, horizontal: 20),
          ),
        ),
        inputDecorationTheme: InputDecorationTheme(
          filled: true,
          fillColor: AppColors.surfaceLight,
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(10),
            borderSide: const BorderSide(color: AppColors.border),
          ),
          enabledBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(10),
            borderSide: const BorderSide(color: AppColors.border),
          ),
          focusedBorder: OutlineInputBorder(
            borderRadius: BorderRadius.circular(10),
            borderSide: const BorderSide(color: AppColors.accent, width: 1.5),
          ),
          contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 14),
          hintStyle: const TextStyle(color: AppColors.textSecondary, fontSize: 14),
        ),
      ),
      home: const HomePage(),
    );
  }
}

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  final TextEditingController _serverController = TextEditingController();
  String? _baseUrl;
  WebViewController? _webViewController;
  Timer? _pollTimer;
  Map<String, dynamic> _state = {};
  int _currentTab = 3; // Start on Control tab
  bool _micropythonMode = false;

  @override
  void dispose() {
    _pollTimer?.cancel();
    super.dispose();
  }

  void _connect() async {
    final text = _serverController.text.trim();
    if (text.isEmpty) return;
    var url = text;
    if (!url.startsWith('http')) url = 'http://$url';
    url = url.replaceAll(RegExp(r'/*\z'), '');

    setState(() => _baseUrl = url);

    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(const Duration(milliseconds: 500), (_) async {
      if (_baseUrl == null) return;
      try {
        final r = await http.get(Uri.parse('$_baseUrl/api/status'))
            .timeout(const Duration(seconds: 1));
        if (r.statusCode == 200) {
          setState(() => _state = _jsonDecodeSafe(r.body));
        }
      } catch (_) {}
    });

    // Camera feed via webview
    if (_baseUrl != null) {
      try {
        final videoHtml = 'data:text/html,<html><body style="margin:0;background:black">'
            '<img src="$_baseUrl/video_feed" style="width:100%;height:100%;object-fit:contain" />'
            '</body></html>';
        if (_webViewController == null) {
          final c = WebViewController();
          c.setJavaScriptMode(JavaScriptMode.unrestricted);
          c.loadRequest(Uri.parse(videoHtml));
          _webViewController = c;
        } else {
          _webViewController!.loadRequest(Uri.parse(videoHtml));
        }
      } catch (_) {}
    }
  }

  void _disconnect() {
    _pollTimer?.cancel();
    setState(() {
      _baseUrl = null;
      _state = {};
      _webViewController = null;
    });
  }

  static Map<String, dynamic> _jsonDecodeSafe(String s) {
    try {
      return s.isEmpty ? {} : Map<String, dynamic>.from(json.decode(s));
    } catch (_) {
      return {};
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: IndexedStack(
        index: _currentTab,
        children: [
          _buildCameraTab(),
          _buildDashboardTab(),
          _buildMapTab(),
          ControlTab(bridgeUrl: _baseUrl, micropythonMode: _micropythonMode),
          _buildSettingsTab(),
        ],
      ),
      bottomNavigationBar: _buildBottomBar(),
    );
  }

  // ─── Bottom Navigation ──────────────────────────────────────────────────
  Widget _buildBottomBar() {
    return Container(
      decoration: const BoxDecoration(
        color: AppColors.surface,
        border: Border(top: BorderSide(color: AppColors.border, width: 1)),
      ),
      child: SafeArea(
        top: false,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 6),
          child: Row(
            children: [
              _navItem(0, Icons.videocam_rounded, 'Camera'),
              _navItem(1, Icons.dashboard_rounded, 'Status'),
              _navItem(2, Icons.map_rounded, 'Map'),
              _navItem(3, Icons.gamepad_rounded, 'Control'),
              _navItem(4, Icons.settings_rounded, 'Settings'),
            ],
          ),
        ),
      ),
    );
  }

  Widget _navItem(int index, IconData icon, String label) {
    final selected = _currentTab == index;
    return Expanded(
      child: GestureDetector(
        behavior: HitTestBehavior.opaque,
        onTap: () => setState(() => _currentTab = index),
        child: AnimatedContainer(
          duration: const Duration(milliseconds: 200),
          padding: const EdgeInsets.symmetric(vertical: 8),
          margin: const EdgeInsets.symmetric(horizontal: 2),
          decoration: BoxDecoration(
            color: selected ? AppColors.accent.withValues(alpha: 0.12) : Colors.transparent,
            borderRadius: BorderRadius.circular(10),
          ),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(
                icon,
                size: 22,
                color: selected ? AppColors.accent : AppColors.textSecondary,
              ),
              const SizedBox(height: 3),
              Text(
                label,
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: selected ? FontWeight.w600 : FontWeight.w400,
                  color: selected ? AppColors.accent : AppColors.textSecondary,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  // ─── Header Widget ──────────────────────────────────────────────────────
  Widget _header(String title, {Widget? trailing}) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(20, 12, 20, 8),
      child: Row(
        children: [
          Text(
            title,
            style: const TextStyle(
              fontSize: 22,
              fontWeight: FontWeight.w700,
              color: AppColors.textPrimary,
              letterSpacing: -0.5,
            ),
          ),
          const Spacer(),
          if (trailing != null) trailing,
          _connectionDot(),
        ],
      ),
    );
  }

  Widget _connectionDot() {
    return Container(
      width: 10,
      height: 10,
      margin: const EdgeInsets.only(left: 10),
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        color: _baseUrl != null ? AppColors.success : AppColors.textSecondary,
        boxShadow: _baseUrl != null
            ? [BoxShadow(color: AppColors.success.withValues(alpha: 0.5), blurRadius: 8)]
            : null,
      ),
    );
  }

  // ═════════════════════════════════════════════════════════════════════════
  //  TAB: CAMERA
  // ═════════════════════════════════════════════════════════════════════════
  Widget _buildCameraTab() {
    return SafeArea(
      child: Column(
        children: [
          _header('Camera'),
          Expanded(
            child: Container(
              margin: const EdgeInsets.fromLTRB(16, 0, 16, 8),
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(14),
                border: Border.all(color: AppColors.border),
                color: AppColors.surface,
              ),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(14),
                child: _baseUrl == null
                    ? _placeholder('Connect from Settings to view camera feed', Icons.videocam_off_rounded)
                    : (_webViewController == null
                        ? const Center(child: CircularProgressIndicator(color: AppColors.accent))
                        : WebViewWidget(controller: _webViewController!)),
              ),
            ),
          ),
          // Telemetry bar
          Container(
            margin: const EdgeInsets.fromLTRB(16, 0, 16, 12),
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
            decoration: BoxDecoration(
              color: AppColors.surface,
              borderRadius: BorderRadius.circular(12),
              border: Border.all(color: AppColors.border),
            ),
            child: Row(
              children: [
                _miniStat('ID', '${_state['target_id'] ?? '-'}'),
                _miniStat('Dist', '${_state['distance_cm'] ?? '-'}cm'),
                _miniStat('Bearing', '${_state['bearing'] ?? '-'}°'),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ═════════════════════════════════════════════════════════════════════════
  //  TAB: DASHBOARD
  // ═════════════════════════════════════════════════════════════════════════
  Widget _buildDashboardTab() {
    return SafeArea(
      child: Column(
        children: [
          _header('Status'),
          Expanded(
            child: ListView(
              padding: const EdgeInsets.symmetric(horizontal: 0, vertical: 4),
              children: [
                // System Status
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(18),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        _sectionTitle('System'),
                        const SizedBox(height: 14),
                        _statusRow('Connection', _baseUrl ?? 'Not Connected',
                            color: _baseUrl != null ? AppColors.success : AppColors.textSecondary),
                        _statusRow('Pose Source', _state['pose']?['source']?.toString() ?? '-'),
                        _statusRow('Robot X', '${(_state['pose']?['x_m'] as num?)?.toStringAsFixed(3) ?? '-'} m'),
                        _statusRow('Robot Y', '${(_state['pose']?['y_m'] as num?)?.toStringAsFixed(3) ?? '-'} m'),
                        _statusRow('Scan Points', _state['scan']?['count']?.toString() ?? '-'),
                        _statusRow('Map',
                            _state['map'] != null ? '${_state['map']?['width']}x${_state['map']?['height']}' : 'No map'),
                      ],
                    ),
                  ),
                ),
                // Visible Markers
                if (_state['markers'] != null && (_state['markers'] as Map).isNotEmpty)
                  Card(
                    child: Padding(
                      padding: const EdgeInsets.all(18),
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          _sectionTitle('Visible Markers'),
                          const SizedBox(height: 14),
                          ...(_state['markers'] as Map).entries.map((e) =>
                              _statusRow(
                                'ID ${e.key}',
                                '${(e.value['distance_m'] as num?)?.toStringAsFixed(2) ?? '-'}m / ${(e.value['bearing_deg'] as num?)?.toStringAsFixed(0) ?? '-'}°',
                              ),
                          ),
                        ],
                      ),
                    ),
                  ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ═════════════════════════════════════════════════════════════════════════
  //  TAB: MAP
  // ═════════════════════════════════════════════════════════════════════════
  Widget _buildMapTab() {
    return SafeArea(
      child: Column(
        children: [
          _header('Map'),
          Expanded(
            child: _baseUrl == null
                ? _placeholder('Connect from Settings to view LIDAR map', Icons.map_rounded)
                : LidarMapView(baseUrl: _baseUrl!),
          ),
        ],
      ),
    );
  }

  // ═════════════════════════════════════════════════════════════════════════
  //  TAB: SETTINGS
  // ═════════════════════════════════════════════════════════════════════════
  Widget _buildSettingsTab() {
    return SafeArea(
      child: Column(
        children: [
          _header('Settings'),
          Expanded(
            child: ListView(
              padding: const EdgeInsets.symmetric(vertical: 4),
              children: [
                // Connection
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(18),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        _sectionTitle('Server Connection'),
                        const SizedBox(height: 14),
                        TextField(
                          controller: _serverController,
                          decoration: const InputDecoration(
                            hintText: 'e.g. 192.168.1.5:5000',
                            prefixIcon: Icon(Icons.dns_rounded, size: 20, color: AppColors.textSecondary),
                          ),
                        ),
                        const SizedBox(height: 14),
                        Row(
                          children: [
                            Expanded(
                              child: ElevatedButton.icon(
                                onPressed: _baseUrl == null ? _connect : null,
                                icon: const Icon(Icons.link_rounded, size: 18),
                                label: const Text('Connect'),
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: AppColors.accentDim,
                                  disabledBackgroundColor: AppColors.surfaceLight,
                                ),
                              ),
                            ),
                            const SizedBox(width: 10),
                            Expanded(
                              child: ElevatedButton.icon(
                                onPressed: _baseUrl != null ? _disconnect : null,
                                icon: const Icon(Icons.link_off_rounded, size: 18),
                                label: const Text('Disconnect'),
                                style: ElevatedButton.styleFrom(
                                  backgroundColor: AppColors.error.withValues(alpha: 0.8),
                                  disabledBackgroundColor: AppColors.surfaceLight,
                                ),
                              ),
                            ),
                          ],
                        ),
                        if (_baseUrl != null) ...[
                          const SizedBox(height: 12),
                          Row(
                            children: [
                              Container(
                                width: 8,
                                height: 8,
                                decoration: const BoxDecoration(
                                  shape: BoxShape.circle,
                                  color: AppColors.success,
                                ),
                              ),
                              const SizedBox(width: 8),
                              Expanded(
                                child: Text(
                                  _baseUrl!,
                                  style: const TextStyle(fontSize: 12, color: AppColors.success),
                                  overflow: TextOverflow.ellipsis,
                                ),
                              ),
                            ],
                          ),
                        ],
                      ],
                    ),
                  ),
                ),
                // Control Mode
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(18),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        _sectionTitle('Control Mode'),
                        const SizedBox(height: 10),
                        SwitchListTile(
                          contentPadding: EdgeInsets.zero,
                          title: Text(
                            _micropythonMode ? 'MicroPython Direct' : 'ROS2 Bridge',
                            style: const TextStyle(fontSize: 14, color: AppColors.textPrimary),
                          ),
                          subtitle: Text(
                            _micropythonMode
                                ? 'Lightweight bridge on port 5001 (no ROS2 needed)'
                                : 'Full Nav2 stack via ros2_mobile_bridge',
                            style: const TextStyle(fontSize: 11, color: AppColors.textSecondary),
                          ),
                          value: _micropythonMode,
                          activeTrackColor: AppColors.accent,
                          onChanged: (val) {
                            setState(() => _micropythonMode = val);
                          },
                        ),
                        if (_micropythonMode)
                          Container(
                            margin: const EdgeInsets.only(top: 8),
                            padding: const EdgeInsets.all(10),
                            decoration: BoxDecoration(
                              color: AppColors.accentDim.withValues(alpha: 0.3),
                              borderRadius: BorderRadius.circular(8),
                            ),
                            child: const Row(
                              children: [
                                Icon(Icons.developer_board, size: 16, color: AppColors.accent),
                                SizedBox(width: 8),
                                Expanded(
                                  child: Text(
                                    'Run micropython_bridge.py on RPi5. '
                                    'Pico must be connected via USB.',
                                    style: TextStyle(fontSize: 11, color: AppColors.textSecondary),
                                  ),
                                ),
                              ],
                            ),
                          ),
                      ],
                    ),
                  ),
                ),
                // About
                Card(
                  child: Padding(
                    padding: const EdgeInsets.all(18),
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        _sectionTitle('About'),
                        const SizedBox(height: 14),
                        const Text(
                          'AutoNexa Mobile Controller',
                          style: TextStyle(fontSize: 14, color: AppColors.textPrimary),
                        ),
                        const SizedBox(height: 4),
                        const Text(
                          'Connects to the RPi5 ROS2 bridge for joystick control, '
                          'LIDAR map visualization, and system telemetry.',
                          style: TextStyle(fontSize: 12, color: AppColors.textSecondary),
                        ),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ─── Shared Widgets ─────────────────────────────────────────────────────
  Widget _sectionTitle(String title) {
    return Text(
      title,
      style: const TextStyle(
        fontSize: 15,
        fontWeight: FontWeight.w600,
        color: AppColors.textPrimary,
        letterSpacing: -0.3,
      ),
    );
  }

  Widget _statusRow(String label, String value, {Color? color}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 5),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label, style: const TextStyle(fontSize: 13, color: AppColors.textSecondary)),
          Text(
            value,
            style: TextStyle(
              fontSize: 13,
              fontWeight: FontWeight.w600,
              color: color ?? AppColors.textPrimary,
              fontFamily: 'monospace',
            ),
          ),
        ],
      ),
    );
  }

  Widget _miniStat(String label, String value) {
    return Expanded(
      child: Column(
        children: [
          Text(label, style: const TextStyle(fontSize: 10, color: AppColors.textSecondary)),
          const SizedBox(height: 2),
          Text(
            value,
            style: const TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w600,
              fontFamily: 'monospace',
            ),
          ),
        ],
      ),
    );
  }

  Widget _placeholder(String message, IconData icon) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 48, color: AppColors.textSecondary.withValues(alpha: 0.4)),
          const SizedBox(height: 14),
          Text(
            message,
            style: const TextStyle(fontSize: 14, color: AppColors.textSecondary),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }
}
