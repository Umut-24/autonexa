import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as ws_status;
import 'package:xterm/xterm.dart';

import '../theme/theme_provider.dart';
import '../theme/app_colors.dart';
import '../services/connection_service.dart';

/// Full web terminal: a real PTY shell on the robot, streamed both ways over
/// `/ws/terminal`. Lists every active session (multiple browsers can attach to
/// the same shell), lets you spawn new ones and kill them. Gated server-side by
/// the `enable_web_terminal` launch flag — when off, the bridge 403s and this
/// page shows a disabled notice.
///
/// SECURITY: this is arbitrary shell as the robot user with no auth — only
/// reachable on the LAN-trusted testbed network, per the deployment decision.
class TerminalsTab extends StatefulWidget {
  const TerminalsTab({super.key});

  @override
  State<TerminalsTab> createState() => _TerminalsTabState();
}

class _TerminalsTabState extends State<TerminalsTab> {
  final Terminal _terminal = Terminal(maxLines: 10000);
  WebSocketChannel? _channel;
  StreamSubscription? _sub;
  List<Map<String, dynamic>> _sessions = [];
  String? _activeId;
  bool _connecting = false;
  String _statusLine = '';

  @override
  void initState() {
    super.initState();
    _terminal.onOutput = (data) =>
        _send({'op': 'input', 'data': data});
    _terminal.onResize = (w, h, pw, ph) =>
        _send({'op': 'resize', 'rows': h, 'cols': w});
    WidgetsBinding.instance.addPostFrameCallback((_) => _refresh());
  }

  @override
  void dispose() {
    _detach();
    super.dispose();
  }

  void _send(Map<String, dynamic> msg) {
    try {
      _channel?.sink.add(jsonEncode(msg));
    } catch (_) {}
  }

  void _detach() {
    _sub?.cancel();
    _sub = null;
    try {
      _channel?.sink.close(ws_status.normalClosure);
    } catch (_) {}
    _channel = null;
  }

  Future<void> _refresh() async {
    final conn = context.read<ConnectionService>();
    final list = await conn.listTerminals();
    if (!mounted) return;
    setState(() => _sessions = list);
  }

  /// Open a WS and either attach to [sessionId] or spawn a fresh shell.
  void _connect({String? sessionId}) {
    final conn = context.read<ConnectionService>();
    final base = conn.wsBaseUrl;
    if (base == null) return;
    _detach();
    setState(() {
      _connecting = true;
      _statusLine = sessionId == null ? 'Starting shell…' : 'Attaching…';
    });
    // Reset the on-screen buffer so a re-attach doesn't stack old output.
    _terminal.buffer.clear();
    _terminal.buffer.setCursor(0, 0);
    try {
      final ch = WebSocketChannel.connect(Uri.parse('$base/ws/terminal'));
      _channel = ch;
      _sub = ch.stream.listen(
        _onWsMessage,
        onError: (e) => _onWsClosed('error: $e'),
        onDone: () => _onWsClosed('disconnected'),
        cancelOnError: true,
      );
      if (sessionId == null) {
        _send({'op': 'spawn', 'rows': 24, 'cols': 80});
      } else {
        _send({'op': 'attach', 'session_id': sessionId});
      }
    } catch (e) {
      _onWsClosed('connect failed: $e');
    }
  }

  void _onWsMessage(dynamic raw) {
    Map<String, dynamic> msg;
    try {
      msg = jsonDecode(raw as String) as Map<String, dynamic>;
    } catch (_) {
      return;
    }
    switch (msg['type']) {
      case 'attached':
        setState(() {
          _activeId = (msg['session_id'] ?? '').toString();
          _connecting = false;
          _statusLine = 'attached to ${_activeId!}';
        });
        _refresh();
        break;
      case 'output':
        final data = (msg['data'] ?? '').toString();
        if (data.isNotEmpty) {
          // PTY emits raw bytes (base64 on the wire); decode leniently so a
          // split UTF-8 sequence at a chunk boundary doesn't throw.
          _terminal.write(utf8.decode(base64.decode(data), allowMalformed: true));
        }
        break;
      case 'exit':
        setState(() => _statusLine = 'shell exited (code ${msg['code']})');
        _terminal.write('\r\n[process exited: code ${msg['code']}]\r\n');
        _refresh();
        break;
      case 'error':
        setState(() {
          _connecting = false;
          _statusLine = 'error: ${msg['text']}';
        });
        break;
    }
  }

  void _onWsClosed(String why) {
    if (!mounted) return;
    setState(() {
      _connecting = false;
      _statusLine = why;
    });
  }

  Future<void> _killActive() async {
    final id = _activeId;
    if (id == null) return;
    final conn = context.read<ConnectionService>();
    await conn.killTerminal(id);
    _detach();
    if (!mounted) return;
    setState(() {
      _activeId = null;
      _statusLine = 'killed $id';
    });
    _refresh();
  }

  @override
  Widget build(BuildContext context) {
    final colors = context.watch<ThemeProvider>().colors;
    final conn = context.watch<ConnectionService>();

    return Scaffold(
      backgroundColor: colors.background,
      appBar: AppBar(
        title: const Text('Terminal'),
        backgroundColor: colors.surface,
        actions: [
          IconButton(
            tooltip: 'Refresh sessions',
            icon: const Icon(Icons.refresh_rounded),
            onPressed: conn.isConnected ? _refresh : null,
          ),
        ],
      ),
      body: !conn.isConnected
          ? _notice('Not connected to the robot.', colors)
          : !conn.webTerminalEnabled
              ? _notice(
                  'Web terminal is disabled on the bridge '
                  '(enable_web_terminal:=false).',
                  colors)
              : Column(
                  children: [
                    _sessionBar(context, conn, colors),
                    Expanded(child: _terminalArea(colors)),
                    _statusBar(colors),
                  ],
                ),
    );
  }

  Widget _notice(String text, ResolvedColors colors) => Center(
        child: Padding(
          padding: const EdgeInsets.all(24),
          child: Text(text,
              textAlign: TextAlign.center,
              style: TextStyle(color: colors.textSecondary)),
        ),
      );

  Widget _sessionBar(
      BuildContext context, ConnectionService conn, ResolvedColors colors) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      color: colors.surface,
      child: Row(
        children: [
          Expanded(
            child: SingleChildScrollView(
              scrollDirection: Axis.horizontal,
              child: Row(
                children: [
                  for (final s in _sessions)
                    Padding(
                      padding: const EdgeInsets.only(right: 6),
                      child: ChoiceChip(
                        label: Text(
                          '${s['title'] ?? s['id']}'
                          '${(s['clients'] ?? 0) is int && (s['clients'] as int) > 1 ? ' •${s['clients']}' : ''}',
                          style: const TextStyle(fontSize: 12),
                        ),
                        selected: _activeId == s['id'],
                        onSelected: (_) =>
                            _connect(sessionId: (s['id']).toString()),
                      ),
                    ),
                  if (_sessions.isEmpty)
                    Text('No active shells',
                        style: TextStyle(
                            fontSize: 12, color: colors.textTertiary)),
                ],
              ),
            ),
          ),
          const SizedBox(width: 8),
          IconButton(
            tooltip: 'New shell',
            icon: const Icon(Icons.add_circle_outline_rounded,
                color: AppColors.info),
            onPressed: _connecting ? null : () => _connect(),
          ),
          IconButton(
            tooltip: 'Kill active shell',
            icon: const Icon(Icons.delete_outline_rounded,
                color: AppColors.danger),
            onPressed: _activeId == null ? null : _killActive,
          ),
        ],
      ),
    );
  }

  Widget _terminalArea(ResolvedColors colors) {
    if (_activeId == null && !_connecting) {
      return Center(
        child: Text(
          'Tap a session above, or "+" to start a new shell.',
          style: TextStyle(color: colors.textTertiary),
        ),
      );
    }
    return Container(
      color: Colors.black,
      padding: const EdgeInsets.all(6),
      child: TerminalView(
        _terminal,
        autofocus: true,
        backgroundOpacity: 0,
        textStyle: const TerminalStyle(fontSize: 13),
      ),
    );
  }

  Widget _statusBar(ResolvedColors colors) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      color: colors.surface,
      child: Text(
        _statusLine.isEmpty ? 'idle' : _statusLine,
        style: TextStyle(
            fontSize: 11, fontFamily: 'monospace', color: colors.textSecondary),
      ),
    );
  }
}
