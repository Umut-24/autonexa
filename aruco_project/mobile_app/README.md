# AutoNexa Mobile App (Flutter)

AutoNexa Mobile is a Flutter control client for the AutoNexa robot stack. It targets Android first (APK), and can connect to the Raspberry Pi bridge in two runtime modes:

1. **ROS2 Bridge mode** (default): connect to `ros2_mobile_bridge` (typically port `5000`).
2. **MicroPython Direct mode**: connect to `micropython_bridge.py` (typically port `5001`) for direct Pico control without ROS2.

---

## Current app structure

Main tabs:
- **Camera**: embedded WebView video stream from `/video_feed`.
- **Status**: live system and marker telemetry from `/api/status`.
- **Map**: LIDAR map visualization.
- **Control**: joystick + speed limiter + E-STOP + telemetry HUD.
- **Settings**: server connection and control mode selection.

Core files:
- `lib/main.dart`: theme, navigation, connection lifecycle, mode switch.
- `lib/control_tab.dart`: joystick UI, fullscreen driving mode, telemetry display.
- `lib/pico_udp_service.dart`: command/telemetry HTTP client and health watchdog.

---

## Bridge/API contract used by mobile app

The mobile app expects these endpoints on the selected bridge URL:

- `GET /api/status`
- `POST /api/control`
- `GET /api/telemetry`
- `POST /api/estop`

In MicroPython mode, the app keeps the same host/IP and automatically uses **port 5001**.

---

## MicroPython direct bridge setup (RPi)

From repository root:

```bash
python3 test/pc/micropython_bridge.py --port /dev/ttyACM0 --http-port 5001
```

This serves the same mobile endpoints as ROS2 bridge and forwards control packets to Pico over serial.

## Pico firmware upload bundle (for mobile app tests)

Use the ready bundle in `test/pico/mobile_app_bundle/`:

```bash
./test/pico/upload_mobile_bundle.sh /dev/ttyACM0
```

Detailed test flow:

- `docs/MICROPYTHON_MOBILE_APP_TEST_GUIDE_2026-03-29.md`

---

## Build APK (release)

Prerequisites:
- Flutter SDK installed and available in `PATH`
- Android SDK + build tools installed
- Java 17+ recommended for modern Android Gradle setups

Commands:

```bash
cd aruco_project/mobile_app
flutter pub get
flutter build apk --release
```

APK output:

```text
aruco_project/mobile_app/build/app/outputs/flutter-apk/app-release.apk
```

Optional debug APK:

```bash
flutter build apk --debug
```

---

## Practical usage flow

1. Launch either bridge on RPi (`5000` for ROS2 bridge, `5001` for MicroPython bridge).
2. In app **Settings**, enter server host/IP (e.g. `192.168.1.5:5000`).
3. Toggle **Control Mode**:
   - OFF: ROS2 Bridge mode.
   - ON: MicroPython Direct mode (app switches to port `5001`).
4. Use **Control** tab joystick and monitor telemetry.
5. Use E-STOP if needed; bridge watchdog also zeroes stale commands.

---

## Notes

- Android app requires internet permission (`android.permission.INTERNET`).
- Control transport is HTTP polling + periodic command push (not native UDP).
- Connection health is monitored periodically and surfaced in the UI.
