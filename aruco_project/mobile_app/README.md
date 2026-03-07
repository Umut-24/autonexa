AutoNexa Mobile (Flutter)

This is a minimal Flutter Android app that acts as a dedicated mobile interface for the AutoNexa server.

Features:
- Connect to your PC server by IP (e.g. `http://192.168.1.5:5000`).
- Shows the server UI (via an embedded WebView) and native controls.
- Native buttons to set target ID (0-15), next/prev, calibrate, and quit.
- Polls `/state` every 500ms to show current telemetry and highlight selected ID.

Build & install (on your machine with Flutter SDK installed):

1. Open a terminal in this folder:

```powershell
cd "c:\Users\Anıl\OneDrive\Masaüstü\aruco project\mobile_app"
```

# AutoNexa Mobile (Flutter)

This is a lightweight Flutter Android app that connects to your AutoNexa server (PC now, Raspberry Pi later).

## Features

- **Connect to server** by IP (e.g. `http://192.168.1.5:5000`)
- **Embedded WebView** showing video/telemetry from server UI
- **Native controls** with full detection ID management:
  - Single ID mode: Select and lock on a specific ID (0-15) via buttons or grid
  - **All-IDs mode**: Track telemetry for all detected markers simultaneously
  - **Pre-select ID**: Choose an ID before connecting; auto-selects on connection
  - **Custom calibration distance**: Input any distance in cm (no hardcoded 50cm)
  - **On-the-road ID switching**: Lock onto a new ID or cycle through detected targets

## Quick Setup

1. Navigate to this folder in terminal:

```powershell
cd "C:\aruco_project\mobile_app"
```

2. Get dependencies:

```powershell
flutter pub get
```

## Add the Logo Asset

Place your logo at `mobile_app/assets/logo.png`. The app shows a fallback if missing.

## Build & Install (Release APK)

```powershell
flutter pub get
flutter build apk --release
```

The APK will be at: `build\app\outputs\flutter-apk\app-release.apk`

Optional AAB build:
```powershell
flutter build appbundle --release
```

## Usage Guide

### Single ID Mode (Default)
- **ID Grid**: Tap any button 0-15 to lock on that marker
- **Prev/Next**: Cycle through IDs
- **Calibrate**: Enter distance in cm and tap Calibrate

### All-IDs Mode
- **All-IDs Button**: Toggle on to track all detected markers
- Telemetry for each detected ID is stored in memory
- **Tracked Counter**: Shows how many IDs have been observed
- **Telemetry Popup**: View all tracked IDs with distance/bearing and lock onto any one

### Pre-select ID (Before Connecting)
- **Pre-select ID Button**: Choose an ID before connecting to server
- On successful connection, that ID is automatically set as target
- Useful for knowing which marker to focus on before going out

### Custom Calibration
- Replace hardcoded 50cm with any distance
- Enter value in cm, tap **Calibrate**
- Sent to server via `/calibrate?distance=<value>`

### On-the-Road ID Switching
- All-IDs mode tracks multiple targets as you scan
- **Lock button** in telemetry popup instantly switches focus to that ID
- Use **Prev/Next** to manually cycle if needed

## Raspberry Pi (Server) Optimization

- The mobile app is intentionally lightweight — minimal animations, simple widgets.
- When deploying to Raspberry Pi 5 (4GB), optimize the server:
  - Reduce camera resolution (e.g. 640x360)
  - Limit FPS to 10–15
  - Avoid heavy per-frame processing
  - Stream compressed frames to mobile app

## Android Permissions

Verify `android/app/src/main/AndroidManifest.xml` includes:

```xml
<uses-permission android:name="android.permission.INTERNET" />
```

## Dependencies

- `webview_flutter: ^4.0.7` — Embedded web UI
- `google_fonts: ^6.0.0` — Clean typography
- `http: ^1.1.0` — Server communication
- `cupertino_icons: ^1.0.2` — iOS-style icons

## Notes

- State polling: 500ms interval to keep telemetry fresh
- All-IDs telemetry is cleared when switching to single ID mode
- Pre-selected ID persists until connection; auto-sets on connect

