# AutoNexa Mobile App (Flutter)

AutoNexa Mobile is a Flutter control client for the robot. The **primary path** is now:

- **Pico W Direct Wi-Fi mode** (recommended): phone talks directly to Pico W HTTP API on port `5001`.

A legacy ROS2 bridge mode remains available, but it is optional.

---

## App structure

Main tabs:
- **Camera**
- **Status**
- **Map**
- **Control**
- **Settings**

Core files:
- `lib/main.dart`
- `lib/control_tab.dart`
- `lib/pico_udp_service.dart`

---

## API contract used by Control tab

- `GET /api/status`
- `POST /api/control`
- `GET /api/telemetry`
- `POST /api/estop`
- `POST /api/goal` (optional, used by goal buttons in MicroPython mode)

In MicroPython mode, default control port is `5001`.

---

## Pico W direct Wi-Fi setup (no RPi)

Use firmware bundle:

- `test/pico/pico_w_wifi_bundle/`

Upload script:

```bash
./test/pico/upload_pico_w_wifi_bundle.sh /dev/ttyACM0
```

Default AP mode from `config.py`:

- SSID: `AutoNexa-PicoW`
- Password: `autonexa123`
- Pico IP: `192.168.4.1`
- API URL: `http://192.168.4.1:5001`

Detailed setup + validation guide:

- `docs/PICO_W_DIRECT_WIFI_CONTROL_GUIDE_2026-03-29.md`

---

## Build APK

```bash
cd aruco_project/mobile_app
flutter pub get
flutter build apk --release
```

Output:

- `aruco_project/mobile_app/build/app/outputs/flutter-apk/app-release.apk`

---

## Quick usage flow (Pico W direct)

1. Flash Pico W with `pico_w_wifi_bundle`.
2. Connect phone to Pico W Wi-Fi (`AutoNexa-PicoW`).
3. In app **Settings**:
   - Enable **Pico W Direct Wi-Fi** mode.
   - Server: `192.168.4.1:5001`
   - Tap **Connect**.
4. Open **Control** tab and drive.

---

## Notes

- Android app needs internet permission (already set).
- Transport is HTTP command push + telemetry polling.
- If you use legacy ROS2 bridge mode, set server to that host/port instead.
