import 'package:flutter/foundation.dart';
import '../services/connection_service.dart';
import '../services/event_logger.dart';
import '../services/preferences_service.dart';

/// Top-level app state container. Provides access to all services.
///
/// This is the single source of truth for the app. Tabs read from
/// ConnectionService (robotStatus, telemetry, etc.) and write commands
/// through it. EventLogger and PreferencesService are also accessible here.
class AppState extends ChangeNotifier {
  final ConnectionService connection;
  final EventLogger logger;
  final PreferencesService prefs;

  // Navigation state for the "More" bottom sheet
  int currentTab = 0;

  AppState({
    required this.connection,
    required this.logger,
    required this.prefs,
  });

  void setTab(int index) {
    currentTab = index;
    notifyListeners();
  }
}
