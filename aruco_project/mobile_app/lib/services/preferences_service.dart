import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';

/// Persists app settings via SharedPreferences.
class PreferencesService {
  static const _keyServers = 'saved_servers';
  static const _keyLastServer = 'last_server';
  static const _keyAutoReconnect = 'auto_reconnect';
  static const _keySpeedLimit = 'default_speed_limit';
  static const _keyMapRefreshMs = 'map_refresh_ms';
  static const _keyMissions = 'saved_missions';
  static const _keyThemeMode = 'theme_mode';
  static const _keySummonPose = 'summon_pose';
  static const _keyHapticEnabled = 'haptic_enabled';
  static const _keyShowBatteryWarning = 'show_battery_warning';
  static const _keyAmoledEnabled = 'amoled_enabled';
  static const _keyProtocol = 'comm_protocol';
  static const _keyBatteryCapacity = 'battery_capacity_mah';

  late final SharedPreferences _prefs;

  Future<void> init() async {
    _prefs = await SharedPreferences.getInstance();
  }

  // --- Server history ---

  List<String> get savedServers {
    return _prefs.getStringList(_keyServers) ?? [];
  }

  Future<void> addServer(String ip) async {
    final list = savedServers;
    list.remove(ip);
    list.insert(0, ip);
    if (list.length > 10) list.removeLast();
    await _prefs.setStringList(_keyServers, list);
  }

  Future<void> removeServer(String ip) async {
    final list = savedServers;
    list.remove(ip);
    await _prefs.setStringList(_keyServers, list);
  }

  String? get lastServer => _prefs.getString(_keyLastServer);
  Future<void> setLastServer(String ip) => _prefs.setString(_keyLastServer, ip);

  // --- Auto-reconnect ---

  bool get autoReconnect => _prefs.getBool(_keyAutoReconnect) ?? false;
  Future<void> setAutoReconnect(bool v) => _prefs.setBool(_keyAutoReconnect, v);

  // --- Speed limit ---

  double get defaultSpeedLimit => _prefs.getDouble(_keySpeedLimit) ?? 0.5;
  Future<void> setDefaultSpeedLimit(double v) => _prefs.setDouble(_keySpeedLimit, v);

  // --- Map refresh ---

  int get mapRefreshMs => _prefs.getInt(_keyMapRefreshMs) ?? 2000;
  Future<void> setMapRefreshMs(int ms) => _prefs.setInt(_keyMapRefreshMs, ms);

  // --- Theme mode ---

  ThemeMode get themeMode {
    final v = _prefs.getString(_keyThemeMode);
    switch (v) {
      case 'light':
        return ThemeMode.light;
      case 'dark':
        return ThemeMode.dark;
      default:
        return ThemeMode.dark;
    }
  }

  Future<void> setThemeMode(ThemeMode mode) {
    final s = mode == ThemeMode.light ? 'light' : 'dark';
    return _prefs.setString(_keyThemeMode, s);
  }

  // --- Haptic feedback ---

  bool get hapticEnabled => _prefs.getBool(_keyHapticEnabled) ?? true;
  Future<void> setHapticEnabled(bool v) => _prefs.setBool(_keyHapticEnabled, v);

  // --- Battery warning ---

  bool get showBatteryWarning => _prefs.getBool(_keyShowBatteryWarning) ?? true;
  Future<void> setShowBatteryWarning(bool v) => _prefs.setBool(_keyShowBatteryWarning, v);

  // --- Summon pose (saved user position for vehicle recall) ---

  Map<String, double>? get summonPose {
    final s = _prefs.getString(_keySummonPose);
    if (s == null) return null;
    try {
      final json = jsonDecode(s) as Map<String, dynamic>;
      return {
        'x': (json['x'] ?? 0).toDouble(),
        'y': (json['y'] ?? 0).toDouble(),
        'yaw': (json['yaw'] ?? 0).toDouble(),
      };
    } catch (_) {
      return null;
    }
  }

  Future<void> setSummonPose(double x, double y, double yaw) {
    return _prefs.setString(_keySummonPose, jsonEncode({'x': x, 'y': y, 'yaw': yaw}));
  }

  Future<void> clearSummonPose() => _prefs.remove(_keySummonPose);

  // --- Missions ---

  List<String> get savedMissions => _prefs.getStringList(_keyMissions) ?? [];

  Future<void> saveMission(String name, String jsonString) async {
    final list = savedMissions;
    // Replace existing with same name
    list.removeWhere((s) {
      try {
        final m = jsonDecode(s) as Map<String, dynamic>;
        return m['name'] == name;
      } catch (_) {
        return false;
      }
    });
    list.add(jsonString);
    await _prefs.setStringList(_keyMissions, list);
  }

  Future<void> deleteMission(String name) async {
    final list = savedMissions;
    list.removeWhere((s) {
      try {
        final m = jsonDecode(s) as Map<String, dynamic>;
        return m['name'] == name;
      } catch (_) {
        return false;
      }
    });
    await _prefs.setStringList(_keyMissions, list);
  }

  // --- AMOLED theme ---

  bool get amoledEnabled => _prefs.getBool(_keyAmoledEnabled) ?? false;
  Future<void> setAmoledEnabled(bool v) => _prefs.setBool(_keyAmoledEnabled, v);

  // --- Communication protocol ---

  String get protocol => _prefs.getString(_keyProtocol) ?? 'http';
  Future<void> setProtocol(String v) => _prefs.setString(_keyProtocol, v);

  // --- Battery capacity (mAh) ---

  int get batteryCapacityMah => _prefs.getInt(_keyBatteryCapacity) ?? 5000;
  Future<void> setBatteryCapacityMah(int v) => _prefs.setInt(_keyBatteryCapacity, v);
}
