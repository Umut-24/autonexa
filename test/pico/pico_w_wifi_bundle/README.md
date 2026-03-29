# Pico W Direct Wi-Fi Control Bundle

This bundle lets the mobile app control **Pico W directly over Wi-Fi** (no RPi5 bridge).

## What this firmware provides

- Pico W creates or joins Wi-Fi (configurable in `config.py`).
- HTTP API on port `5001`:
  - `GET /api/status`
  - `GET /api/telemetry`
  - `POST /api/control`
  - `POST /api/goal`
  - `POST /api/estop`
- Motor + servo + encoder control using the same pin map as existing tests.

## Default Wi-Fi mode

Default is AP mode in `config.py`:

- SSID: `AutoNexa-PicoW`
- Password: `autonexa123`
- Pico IP: `192.168.4.1`
- API port: `5001`

So mobile app control URL is:

- `http://192.168.4.1:5001`

## Files

- `boot.py`
- `config.py`
- `main.py`

## Upload

From repo root:

```bash
./test/pico/upload_pico_w_wifi_bundle.sh /dev/ttyACM0
```

Or manually with `mpremote`:

```bash
mpremote connect /dev/ttyACM0 fs cp test/pico/pico_w_wifi_bundle/boot.py :boot.py
mpremote connect /dev/ttyACM0 fs cp test/pico/pico_w_wifi_bundle/config.py :config.py
mpremote connect /dev/ttyACM0 fs cp test/pico/pico_w_wifi_bundle/main.py :main.py
mpremote connect /dev/ttyACM0 reset
```

## App usage

1. Connect phone to Pico W AP (`AutoNexa-PicoW`).
2. In app Settings, server = `192.168.4.1:5001`.
3. Enable MicroPython mode.
4. Open Control tab and drive.

## Pin mapping

- Left motor: `GP2`=`IN1`, `GP3`=`IN2`, `GP4`=`ENA PWM`
- Right motor: `GP6`=`IN3`, `GP7`=`IN4`, `GP8`=`ENB PWM`
- Servo: `GP15`
- Left encoder: `GP10` (A), `GP11` (B)
- Right encoder: `GP12` (A), `GP13` (B)

Keep common GND between Pico and motor driver power domain.
