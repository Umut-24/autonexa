import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'theme/app_theme.dart';
import 'theme/app_colors.dart';
import 'theme/theme_provider.dart';
import 'services/connection_service.dart';
import 'services/event_logger.dart';
import 'services/preferences_service.dart';
import 'state/app_state.dart';
import 'widgets/estop_fab.dart';
import 'widgets/mode_bar.dart';

import 'tabs/home_tab.dart';
import 'tabs/control_tab.dart';
import 'tabs/map_tab.dart';
import 'tabs/parking_tab.dart';
import 'tabs/camera_tab.dart';
import 'tabs/desktop_tab.dart';
import 'tabs/diagnostics_tab.dart';
import 'tabs/settings_tab.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();

  final prefs = PreferencesService();
  await prefs.init();

  final logger = EventLogger();
  final connection = ConnectionService(logger: logger);

  // Apply persisted speed limit
  connection.setSpeedLimit(prefs.defaultSpeedLimit);

  final themeProvider = ThemeProvider(prefs);

  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => AppState(
          connection: connection,
          logger: logger,
          prefs: prefs,
        )),
        ChangeNotifierProvider.value(value: connection),
        ChangeNotifierProvider.value(value: logger),
        ChangeNotifierProvider.value(value: themeProvider),
        Provider.value(value: prefs),
      ],
      child: const AutoNexaApp(),
    ),
  );
}

class AutoNexaApp extends StatelessWidget {
  const AutoNexaApp({super.key});

  @override
  Widget build(BuildContext context) {
    final themeProvider = context.watch<ThemeProvider>();

    return MaterialApp(
      title: 'AutoNexa',
      debugShowCheckedModeBanner: false,
      theme: AppTheme.forVariant(themeProvider.variant),
      darkTheme: AppTheme.forVariant(themeProvider.variant),
      themeMode: themeProvider.mode,
      home: const MainShell(),
    );
  }
}

// ─── Main Navigation Shell ────────────────────────────────────────────────────

class MainShell extends StatefulWidget {
  const MainShell({super.key});

  @override
  State<MainShell> createState() => _MainShellState();
}

class _MainShellState extends State<MainShell> {
  int _currentTab = 0;

  static const _primaryTabs = [
    _TabDef(icon: Icons.space_dashboard_rounded, label: 'Home'),
    _TabDef(icon: Icons.gamepad_rounded, label: 'Control'),
    _TabDef(icon: Icons.map_rounded, label: 'Map'),
    _TabDef(icon: Icons.local_parking_rounded, label: 'Parking'),
    _TabDef(icon: Icons.more_horiz_rounded, label: 'More'),
  ];

  void _showMoreSheet() {
    final colors = context.read<ThemeProvider>().colors;
    showModalBottomSheet(
      context: context,
      backgroundColor: colors.surface,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      builder: (ctx) => _MoreSheet(
        onNavigate: (page) {
          Navigator.pop(ctx);
          Navigator.push(context, MaterialPageRoute(builder: (_) => page));
        },
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final connection = context.watch<ConnectionService>();

    return Scaffold(
      body: Column(
        children: [
          const ModeBar(),
          Expanded(
            child: IndexedStack(
              index: _currentTab.clamp(0, 3),
              children: const [
                HomeTab(),
                ControlTab(),
                MapTab(),
                ParkingTab(),
              ],
            ),
          ),
        ],
      ),
      bottomNavigationBar: _buildBottomBar(),
      floatingActionButton: EstopFab(connection: connection),
      floatingActionButtonLocation: FloatingActionButtonLocation.miniEndFloat,
    );
  }

  Widget _buildBottomBar() {
    final colors = context.watch<ThemeProvider>().colors;

    return Container(
      decoration: BoxDecoration(
        color: colors.surface,
        border: Border(top: BorderSide(color: colors.border, width: 1)),
      ),
      child: SafeArea(
        top: false,
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 4, vertical: 6),
          child: Row(
            children: List.generate(_primaryTabs.length, (i) {
              final tab = _primaryTabs[i];
              final selected = _currentTab == i && i < 4;
              return Expanded(
                child: GestureDetector(
                  behavior: HitTestBehavior.opaque,
                  onTap: () {
                    if (i == 4) {
                      _showMoreSheet();
                    } else {
                      setState(() => _currentTab = i);
                    }
                  },
                  child: AnimatedContainer(
                    duration: const Duration(milliseconds: 200),
                    padding: const EdgeInsets.symmetric(vertical: 8),
                    margin: const EdgeInsets.symmetric(horizontal: 2),
                    decoration: BoxDecoration(
                      color: selected ? colors.accentSurface : Colors.transparent,
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Column(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Icon(
                          tab.icon,
                          size: 22,
                          color: selected ? colors.accent : colors.textSecondary,
                        ),
                        const SizedBox(height: 3),
                        Text(
                          tab.label,
                          style: TextStyle(
                            fontSize: 10,
                            fontWeight: selected ? FontWeight.w600 : FontWeight.w400,
                            color: selected ? colors.accent : colors.textSecondary,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              );
            }),
          ),
        ),
      ),
    );
  }
}

class _TabDef {
  final IconData icon;
  final String label;
  const _TabDef({required this.icon, required this.label});
}

// ─── "More" Bottom Sheet ──────────────────────────────────────────────────────

class _MoreSheet extends StatelessWidget {
  final void Function(Widget page) onNavigate;

  const _MoreSheet({required this.onNavigate});

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;

    return Padding(
      padding: const EdgeInsets.fromLTRB(20, 12, 20, 24),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 40,
            height: 4,
            decoration: BoxDecoration(
              color: colors.textTertiary,
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const SizedBox(height: 20),
          _moreItem(context, colors,
            Icons.videocam_rounded,
            'Camera',
            'Live video feed with ArUco detection',
            () => onNavigate(const CameraTab()),
          ),
          _moreItem(context, colors,
            Icons.desktop_windows_rounded,
            'Desktop',
            'Mirror the Pi desktop (RViz, terminals) at 1 Hz',
            () => onNavigate(const DesktopTab()),
          ),
          _moreItem(context, colors,
            Icons.monitor_heart_rounded,
            'Diagnostics',
            'Event log, network stats, system info',
            () => onNavigate(const DiagnosticsTab()),
          ),
          _moreItem(context, colors,
            Icons.settings_rounded,
            'Settings',
            'Server connection, preferences, about',
            () => onNavigate(const SettingsTab()),
          ),
        ],
      ),
    );
  }

  Widget _moreItem(BuildContext context, ResolvedColors colors,
      IconData icon, String title, String subtitle, VoidCallback onTap) {
    return ListTile(
      leading: Container(
        width: 40,
        height: 40,
        decoration: BoxDecoration(
          color: colors.surfaceLight,
          borderRadius: BorderRadius.circular(10),
        ),
        child: Icon(icon, size: 20, color: colors.textSecondary),
      ),
      title: Text(title, style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
      subtitle: Text(subtitle, style: TextStyle(fontSize: 12, color: colors.textSecondary)),
      trailing: Icon(Icons.chevron_right_rounded, color: colors.textTertiary),
      onTap: onTap,
      contentPadding: const EdgeInsets.symmetric(horizontal: 4, vertical: 2),
    );
  }
}
