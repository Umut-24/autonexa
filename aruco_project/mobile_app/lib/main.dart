import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'package:webview_flutter/webview_flutter.dart';
import 'package:google_fonts/google_fonts.dart';
import 'control_tab.dart';
import 'lidar_map_view.dart';

void main() {
  runApp(const AutoNexaApp());
}

class AutoNexaApp extends StatelessWidget {
  const AutoNexaApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'AutoNexa Mobile',
      theme: ThemeData(
        brightness: Brightness.dark,
        primarySwatch: Colors.red,
        textTheme: GoogleFonts.interTextTheme(ThemeData.dark().textTheme),
        tabBarTheme: const TabBarThemeData(
          indicatorSize: TabBarIndicatorSize.tab,
          indicatorColor: Colors.red,
          labelColor: Colors.white,
          unselectedLabelColor: Colors.grey,
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

class _HomePageState extends State<HomePage> with TickerProviderStateMixin {
  final TextEditingController _serverController = TextEditingController();
  final TextEditingController _calibController = TextEditingController();
  String? _baseUrl; // e.g. http://192.168.1.5:5000
  WebViewController? _webViewController;
  Timer? _pollTimer;
  Map<String, dynamic> _state = {};
  bool _allIdsMode = false;
  Map<int, Map<String, dynamic>> _allIdsTelemetry = {}; // store telemetry for all IDs
  int? _preSelectedId; // pre-selected ID before connecting
  late TabController _tabController;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 5, vsync: this);
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
    _tabController.dispose();
    super.dispose();
  }

  void _connect() async {
    final text = _serverController.text.trim();
    if (text.isEmpty) return;
    var url = text;
    if (!url.startsWith('http')) {
      url = 'http://$url';
    }
    // ensure no trailing slash
    url = url.replaceAll(RegExp(r'/*\z'), '');

    setState(() {
      _baseUrl = url;
    });

    // if pre-selected ID, set it on connect
    if (_preSelectedId != null && _baseUrl != null) {
      _send('/set_id/$_preSelectedId');
    }

    // start polling state from ROS2 bridge
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(const Duration(milliseconds: 500), (_) async {
      if (_baseUrl == null) return;
      try {
        final r = await http.get(Uri.parse('$_baseUrl/api/status')).timeout(const Duration(seconds: 1));
        if (r.statusCode == 200) {
          final newState = Map<String, dynamic>.from(jsonDecodeSafe(r.body));
          setState(() {
            _state = newState;
            // if all-IDs mode, store telemetry for each ID
            if (_allIdsMode && newState['target_id'] != null) {
              final id = newState['target_id'] as int;
              _allIdsTelemetry[id] = {
                'distance_cm': newState['distance_cm'],
                'bearing': newState['bearing'],
                'timestamp': DateTime.now().millisecondsSinceEpoch,
              };
            }
          });
        }
      } catch (_) {}
    });

    // Load camera feed via webview (MJPEG from bridge /video_feed)
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

  void _send(String path) async {
    if (_baseUrl == null) return;
    try {
      await http.get(Uri.parse('$_baseUrl$path')).timeout(const Duration(seconds: 1));
    } catch (e) {
      // ignore
    }
  }

  void _setId(int id) {
    _send('/set_id/$id');
    // clear all-IDs telemetry when switching to single ID
    if (_allIdsMode) {
      setState(() {
        _allIdsTelemetry.clear();
        _allIdsMode = false;
      });
    }
  }

  void _nextId() => _send('/next_id');
  void _prevId() => _send('/prev_id');
  void _quit() => _send('/quit');

  void _calibrate() {
    final v = _calibController.text.trim();
    if (v.isEmpty || _baseUrl == null) return;
    _send('/calibrate?distance=${Uri.encodeComponent(v)}');
  }

  void _toggleAllIdsMode() {
    setState(() {
      _allIdsMode = !_allIdsMode;
      if (_allIdsMode) {
        _allIdsTelemetry.clear();
        // start scanning all IDs (optional: cycle through them)
        _send('/set_id/0');
      } else {
        _allIdsTelemetry.clear();
      }
    });
  }

  void _showPreSelectDialog() {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('Pre-select Detection ID'),
        content: SizedBox(
          width: double.maxFinite,
          child: GridView.count(
            crossAxisCount: 4,
            childAspectRatio: 1.5,
            children: List.generate(16, (i) {
              final selected = _preSelectedId == i;
              return Padding(
                padding: const EdgeInsets.all(4.0),
                child: ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: selected ? Colors.green : null,
                  ),
                  onPressed: () {
                    setState(() => _preSelectedId = i);
                    Navigator.pop(ctx);
                  },
                  child: Text('$i'),
                ),
              );
            }),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Cancel'),
          ),
        ],
      ),
    );
  }

  void _showAllIdsTelemetry() {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('All-IDs Telemetry'),
        content: SizedBox(
          width: double.maxFinite,
          child: ListView.builder(
            itemCount: _allIdsTelemetry.length,
            itemBuilder: (c, i) {
              final id = _allIdsTelemetry.keys.elementAt(i);
              final telemetry = _allIdsTelemetry[id]!;
              return ListTile(
                title: Text('ID $id'),
                subtitle: Text(
                  'Distance: ${telemetry['distance_cm']} cm, Bearing: ${telemetry['bearing']}°',
                ),
                trailing: ElevatedButton(
                  onPressed: () {
                    _setId(id);
                    Navigator.pop(ctx);
                  },
                  child: const Text('Lock'),
                ),
              );
            },
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx),
            child: const Text('Close'),
          ),
        ],
      ),
    );
  }

  static Map<String, dynamic> jsonDecodeSafe(String s) {
    try {
      return s.isEmpty ? {} : Map<String, dynamic>.from(json.decode(s));
    } catch (_) {
      return {};
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        elevation: 0,
        backgroundColor: Colors.transparent,
        title: Row(
          children: [
            SizedBox(
              height: 40,
              width: 40,
              child: ClipRRect(
                borderRadius: BorderRadius.circular(8),
                child: Image.asset(
                  'assets/logo.png',
                  fit: BoxFit.contain,
                  errorBuilder: (c, e, s) => Container(
                    color: Colors.white12,
                    alignment: Alignment.center,
                    child: const Text('A', style: TextStyle(fontSize: 18)),
                  ),
                ),
              ),
            ),
            const SizedBox(width: 12),
            Text('AutoNexa', style: Theme.of(context).textTheme.titleLarge),
            const Spacer(),
            // Connection status indicator
            Container(
              width: 12,
              height: 12,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                color: _baseUrl != null ? Colors.green : Colors.grey,
              ),
            ),
            const SizedBox(width: 8),
          ],
        ),
        bottom: TabBar(
          controller: _tabController,
          isScrollable: true,
          tabAlignment: TabAlignment.center,
          tabs: const [
            Tab(icon: Icon(Icons.videocam), text: 'Camera'),
            Tab(icon: Icon(Icons.dashboard), text: 'Dashboard'),
            Tab(icon: Icon(Icons.map), text: 'Map'),
            Tab(icon: Icon(Icons.gamepad), text: 'Control'),
            Tab(icon: Icon(Icons.settings), text: 'Settings'),
          ],
        ),
      ),
      body: TabBarView(
        controller: _tabController,
        children: [
          // ========== TAB 1: CAMERA FEED ==========
          _buildCameraTab(),
          
          // ========== TAB 2: DASHBOARD ==========
          _buildDashboardTab(),
          
          // ========== TAB 3: MAP VIEW ==========
          _buildMapTab(),
          
          // ========== TAB 4: CONTROL (ACKERMANN) ==========
          ControlTab(bridgeUrl: _baseUrl),
          
          // ========== TAB 5: SETTINGS ==========
          _buildSettingsTab(),
        ],
      ),
    );
  }

  // ==================== TAB: CAMERA FEED ====================
  Widget _buildCameraTab() {
    return SafeArea(
      child: Column(
        children: [
          // Camera feed (WebView)
          Expanded(
            child: Container(
              margin: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(12),
                border: Border.all(color: Colors.white12),
              ),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(12),
                child: _baseUrl == null
                    ? const Center(child: Text('Connect from Settings tab'))
                    : (_webViewController == null
                        ? const Center(child: CircularProgressIndicator())
                        : WebViewWidget(controller: _webViewController!)),
              ),
            ),
          ),

          // Telemetry display
          Container(
            margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.white10,
              borderRadius: BorderRadius.circular(12),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Live Telemetry',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                const SizedBox(height: 8),
                Text(
                  'ID: ${_state['target_id'] ?? '-'}  |  Distance: ${_state['distance_cm'] ?? '-'} cm  |  Bearing: ${_state['bearing'] ?? '-'}°',
                  style: const TextStyle(fontSize: 14, fontWeight: FontWeight.bold),
                ),
                if (_allIdsMode && _allIdsTelemetry.isNotEmpty) ...[
                  const SizedBox(height: 6),
                  Text(
                    'All-IDs Mode: ${_allIdsTelemetry.length} markers tracked',
                    style: const TextStyle(fontSize: 12, color: Colors.greenAccent),
                  ),
                ],
              ],
            ),
          ),

          // Quick controls
          Padding(
            padding: const EdgeInsets.all(12),
            child: Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _prevId,
                    icon: const Icon(Icons.arrow_back),
                    label: const Text('Prev'),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: _nextId,
                    icon: const Icon(Icons.arrow_forward),
                    label: const Text('Next'),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: ElevatedButton(
                    style: ElevatedButton.styleFrom(
                      backgroundColor: _allIdsMode ? Colors.green : Colors.grey,
                    ),
                    onPressed: _toggleAllIdsMode,
                    child: Text(_allIdsMode ? 'All-IDs ✓' : 'All-IDs'),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  // ==================== TAB: DASHBOARD ====================
  Widget _buildDashboardTab() {
    return SafeArea(
      child: SingleChildScrollView(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // ROS2 Bridge Status card
            Card(
              color: Colors.white10,
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'System Status',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Container(
                          width: 16,
                          height: 16,
                          decoration: BoxDecoration(
                            shape: BoxShape.circle,
                            color: _baseUrl != null ? Colors.green : Colors.grey,
                          ),
                        ),
                        const SizedBox(width: 8),
                        Text(
                          _baseUrl ?? 'Not Connected',
                          style: TextStyle(
                            fontSize: 14,
                            color: _baseUrl != null ? Colors.green : Colors.grey,
                          ),
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    _buildTelemetryRow('Pose Source', _state['pose']?['source']?.toString() ?? '-'),
                    _buildTelemetryRow('Robot X', '${(_state['pose']?['x_m'] as num?)?.toStringAsFixed(2) ?? '-'} m'),
                    _buildTelemetryRow('Robot Y', '${(_state['pose']?['y_m'] as num?)?.toStringAsFixed(2) ?? '-'} m'),
                    _buildTelemetryRow('Scan Points', _state['scan']?['count']?.toString() ?? '-'),
                    _buildTelemetryRow('Map', _state['map'] != null ? '${_state['map']?['width']}x${_state['map']?['height']}' : 'No map'),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 12),

            // ArUco Detection card
            Card(
              color: Colors.white10,
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'ArUco Detection',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                    const SizedBox(height: 12),
                    GridView.count(
                      crossAxisCount: 4,
                      shrinkWrap: true,
                      physics: const NeverScrollableScrollPhysics(),
                      childAspectRatio: 1.2,
                      mainAxisSpacing: 8,
                      crossAxisSpacing: 8,
                      children: List.generate(16, (i) {
                        final chosen = _state['target_id'] == i;
                        final tracked = _allIdsTelemetry.containsKey(i);
                        return ElevatedButton(
                          style: ElevatedButton.styleFrom(
                            backgroundColor: chosen
                                ? Colors.green
                                : (tracked ? Colors.orange : Colors.grey),
                          ),
                          onPressed: () => _setId(i),
                          child: Text('$i', style: const TextStyle(fontSize: 16)),
                        );
                      }),
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Expanded(
                          child: OutlinedButton(
                            onPressed: _showPreSelectDialog,
                            child: Text(
                              _preSelectedId != null
                                  ? 'Pre-select: $_preSelectedId'
                                  : 'Pre-select ID',
                            ),
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 12),

            // Markers card
            if (_state['markers'] != null && (_state['markers'] as Map).isNotEmpty)
              Card(
                color: Colors.white10,
                child: Padding(
                  padding: const EdgeInsets.all(16),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text(
                        'Visible Markers',
                        style: Theme.of(context).textTheme.titleMedium,
                      ),
                      const SizedBox(height: 12),
                      ...(_state['markers'] as Map).entries.map((e) =>
                        _buildTelemetryRow(
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
    );
  }

  // ==================== TAB: MAP VIEW ====================
  Widget _buildMapTab() {
    if (_baseUrl == null) {
      return const SafeArea(
        child: Center(child: Text('Connect from Settings tab')),
      );
    }
    return LidarMapView(baseUrl: _baseUrl!);
  }

  // ==================== TAB: SETTINGS ====================
  Widget _buildSettingsTab() {
    return SafeArea(
      child: SingleChildScrollView(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // Connection card
            Card(
              color: Colors.white10,
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Server Connection',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                    const SizedBox(height: 12),
                    TextField(
                      controller: _serverController,
                      decoration: InputDecoration(
                        hintText: 'e.g. 192.168.1.5:5000',
                        filled: true,
                        fillColor: Colors.white10,
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(8),
                          borderSide: const BorderSide(color: Colors.white12),
                        ),
                        contentPadding: const EdgeInsets.symmetric(
                          horizontal: 12,
                          vertical: 14,
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    ElevatedButton(
                      onPressed: _connect,
                      child: const Padding(
                        padding: EdgeInsets.symmetric(vertical: 12),
                        child: Text('Connect to Server'),
                      ),
                    ),
                    const SizedBox(height: 8),
                    if (_baseUrl != null)
                      Text(
                        'Connected to: $_baseUrl',
                        style: const TextStyle(fontSize: 12, color: Colors.green),
                      ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 12),

            // Calibration card
            Card(
              color: Colors.white10,
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Calibration',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                    const SizedBox(height: 12),
                    TextField(
                      controller: _calibController,
                      keyboardType: TextInputType.number,
                      decoration: InputDecoration(
                        hintText: 'Distance in cm (e.g. 50)',
                        filled: true,
                        fillColor: Colors.white10,
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(8),
                          borderSide: const BorderSide(color: Colors.white12),
                        ),
                        contentPadding: const EdgeInsets.symmetric(
                          horizontal: 12,
                          vertical: 14,
                        ),
                      ),
                    ),
                    const SizedBox(height: 12),
                    ElevatedButton(
                      onPressed: _calibrate,
                      child: const Padding(
                        padding: EdgeInsets.symmetric(vertical: 12),
                        child: Text('Calibrate Distance'),
                      ),
                    ),
                    const SizedBox(height: 8),
                    const Text(
                      'Place your car at a known distance and calibrate for accurate measurements.',
                      style: TextStyle(fontSize: 12, color: Colors.grey),
                    ),
                  ],
                ),
              ),
            ),
            const SizedBox(height: 12),

            // Control card
            Card(
              color: Colors.white10,
              child: Padding(
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      'Server Control',
                      style: Theme.of(context).textTheme.titleMedium,
                    ),
                    const SizedBox(height: 12),
                    ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: Colors.red,
                      ),
                      onPressed: _quit,
                      child: const Padding(
                        padding: EdgeInsets.symmetric(vertical: 12),
                        child: Text('Stop Server'),
                      ),
                    ),
                    const SizedBox(height: 8),
                    const Text(
                      'Gracefully stop the server process.',
                      style: TextStyle(fontSize: 12, color: Colors.grey),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildTelemetryRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            label,
            style: const TextStyle(fontSize: 13, color: Colors.grey),
          ),
          Text(
            value,
            style: const TextStyle(
              fontSize: 13,
              fontWeight: FontWeight.bold,
              color: Colors.white,
            ),
          ),
        ],
      ),
    );
  }

}
