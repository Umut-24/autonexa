import 'package:flutter/material.dart';
import '../services/preferences_service.dart';
import 'app_colors.dart';

/// Manages theme mode (dark/light/system) and variant (AMOLED) with persistence.
class ThemeProvider extends ChangeNotifier {
  final PreferencesService _prefs;
  ThemeMode _mode;
  bool _amoled;

  ThemeProvider(this._prefs)
      : _mode = _prefs.themeMode,
        _amoled = _prefs.amoledEnabled;

  ThemeMode get mode => _mode;
  bool get amoled => _amoled;

  bool get isDark => _mode == ThemeMode.dark ||
      (_mode == ThemeMode.system); // default to dark for system

  ThemeVariant get variant {
    if (!isDark) return ThemeVariant.light;
    return _amoled ? ThemeVariant.amoled : ThemeVariant.dark;
  }

  ResolvedColors get colors {
    switch (variant) {
      case ThemeVariant.light: return const ResolvedColors.light();
      case ThemeVariant.amoled: return const ResolvedColors.amoled();
      case ThemeVariant.dark: return const ResolvedColors.dark();
    }
  }

  void setMode(ThemeMode mode) {
    _mode = mode;
    _prefs.setThemeMode(mode);
    notifyListeners();
  }

  void setAmoled(bool enabled) {
    _amoled = enabled;
    _prefs.setAmoledEnabled(enabled);
    notifyListeners();
  }

  void toggle() {
    setMode(isDark ? ThemeMode.light : ThemeMode.dark);
  }

  /// Cycle through: Dark → AMOLED → Light → Dark
  void cycleTheme() {
    switch (variant) {
      case ThemeVariant.dark:
        setAmoled(true);
        break;
      case ThemeVariant.amoled:
        setAmoled(false);
        setMode(ThemeMode.light);
        break;
      case ThemeVariant.light:
        setMode(ThemeMode.dark);
        break;
    }
  }

  IconData get themeIcon {
    switch (variant) {
      case ThemeVariant.dark: return Icons.dark_mode_rounded;
      case ThemeVariant.amoled: return Icons.brightness_2_rounded;
      case ThemeVariant.light: return Icons.light_mode_rounded;
    }
  }

  String get themeLabel {
    switch (variant) {
      case ThemeVariant.dark: return 'Dark';
      case ThemeVariant.amoled: return 'AMOLED';
      case ThemeVariant.light: return 'Light';
    }
  }
}
