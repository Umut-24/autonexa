import 'dart:collection';
import 'package:flutter/foundation.dart';

enum LogLevel { info, success, warning, error }
enum LogCategory { connection, control, navigation, system }

class LogEntry {
  final DateTime time;
  final String message;
  final LogLevel level;
  final LogCategory category;

  const LogEntry({
    required this.time,
    required this.message,
    this.level = LogLevel.info,
    this.category = LogCategory.system,
  });

  String get formatted =>
      '[${time.hour.toString().padLeft(2, '0')}:'
      '${time.minute.toString().padLeft(2, '0')}:'
      '${time.second.toString().padLeft(2, '0')}] '
      '${level.name.toUpperCase()} $message';
}

/// In-memory ring buffer event logger.
class EventLogger extends ChangeNotifier {
  static const int maxEntries = 500;
  final _entries = ListQueue<LogEntry>(maxEntries);

  List<LogEntry> get entries => _entries.toList();

  void log(String message, {
    LogLevel level = LogLevel.info,
    LogCategory category = LogCategory.system,
  }) {
    if (_entries.length >= maxEntries) _entries.removeFirst();
    _entries.addLast(LogEntry(
      time: DateTime.now(),
      message: message,
      level: level,
      category: category,
    ));
    notifyListeners();
  }

  void info(String msg, [LogCategory cat = LogCategory.system]) =>
      log(msg, level: LogLevel.info, category: cat);
  void success(String msg, [LogCategory cat = LogCategory.system]) =>
      log(msg, level: LogLevel.success, category: cat);
  void warn(String msg, [LogCategory cat = LogCategory.system]) =>
      log(msg, level: LogLevel.warning, category: cat);
  void error(String msg, [LogCategory cat = LogCategory.system]) =>
      log(msg, level: LogLevel.error, category: cat);

  List<LogEntry> filtered({LogLevel? level, LogCategory? category}) {
    return _entries.where((e) {
      if (level != null && e.level != level) return false;
      if (category != null && e.category != category) return false;
      return true;
    }).toList();
  }

  String export() => _entries.map((e) => e.formatted).join('\n');

  void clear() {
    _entries.clear();
    notifyListeners();
  }
}
