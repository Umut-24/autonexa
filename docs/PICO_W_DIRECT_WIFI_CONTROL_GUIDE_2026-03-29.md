# Pico W Direct Wi-Fi Control Guide (No RPi5)

Date: March 29, 2026

This guide switches your control architecture to:

`Mobile App -> Pico W Wi-Fi API -> Motor Driver`

No `ros2_mobile_bridge.py`, no `micropython_bridge.py`, and no RPi5 required for manual drive control.

---

## 1) Prerequisites

- Raspberry Pi Pico W (Wi-Fi model)
- MicroPython firmware installed on Pico W
- Motor wiring completed (L298N + motors + common GND)
- Phone with AutoNexa app build that includes latest changes
- `mpremote` on your dev machine

Install `mpremote` if needed:

```bash
python3 -m pip install --user mpremote
```

---

## 2) Firmware files

Use this bundle:

- `test/pico/pico_w_wifi_bundle/boot.py`
- `test/pico/pico_w_wifi_bundle/config.py`
- `test/pico/pico_w_wifi_bundle/main.py`

### Default Wi-Fi config

In `config.py` default values are:

- `WIFI_MODE = "AP"`
- `AP_SSID = "AutoNexa-PicoW"`
- `AP_PASSWORD = "autonexa123"`
- `AP_IP = "192.168.4.1"`
- `HTTP_PORT = 5001`

That means Pico W creates hotspot and app connects to:

- `http://192.168.4.1:5001`

---

## 3) Upload to Pico W

From repo root:

```bash
./test/pico/upload_pico_w_wifi_bundle.sh /dev/ttyACM0
```

If your port differs, replace `/dev/ttyACM0`.

Manual upload alternative:

```bash
mpremote connect /dev/ttyACM0 fs cp test/pico/pico_w_wifi_bundle/boot.py :boot.py
mpremote connect /dev/ttyACM0 fs cp test/pico/pico_w_wifi_bundle/config.py :config.py
mpremote connect /dev/ttyACM0 fs cp test/pico/pico_w_wifi_bundle/main.py :main.py
mpremote connect /dev/ttyACM0 reset
```

---

## 4) Confirm Pico W API is alive

Connect your laptop/phone to Pico Wi-Fi AP (`AutoNexa-PicoW`) and check:

```bash
curl http://192.168.4.1:5001/api/status
curl http://192.168.4.1:5001/api/telemetry
```

Expected:

- `pico_connected: true`
- valid JSON response from both endpoints

---

## 5) Mobile app setup (Pico-only mode)

1. Connect phone Wi-Fi to `AutoNexa-PicoW`.
2. Open app -> `Settings`.
3. Enable **Pico W Direct Wi-Fi** mode.
4. Server field: `192.168.4.1:5001`.
5. Tap `Connect`.
6. Open `Control` tab.

Expected on Control tab:

- Top badge: `LINKED MPY`
- State: `IDLE` when joystick centered
- On joystick move: state transitions to `TRACKING_PATH`

---

## 6) Functional test sequence

Run this exact order with wheels lifted first.

1. **Link test**:
   - App connects and stays linked for >30s.
2. **Joystick forward**:
   - Hold forward 2s.
   - Observe non-zero wheel velocity values.
3. **Turn test**:
   - Hold left/right turn.
   - Observe opposite-sign wheel behavior.
4. **Release joystick**:
   - State returns to `IDLE`.
5. **E-STOP**:
   - Press E-STOP.
   - PWM/velocity drops to zero quickly.
6. **Goal test (optional)**:
   - Use `DRIVE` and `TURN` buttons.

---

## 7) Wiring reference

- Left motor: `GP2`=`IN1`, `GP3`=`IN2`, `GP4`=`ENA PWM`
- Right motor: `GP6`=`IN3`, `GP7`=`IN4`, `GP8`=`ENB PWM`
- Servo: `GP15`
- Left encoder: `GP10` (A), `GP11` (B)
- Right encoder: `GP12` (A), `GP13` (B)

Critical:

- Common ground between Pico and motor driver power domain.
- Motor power supply connected to driver (USB alone is not enough).

---

## 8) Common troubleshooting

## App shows `NO LINK`

- Ensure phone is connected to Pico AP, not another Wi-Fi.
- Re-check server as `192.168.4.1:5001`.
- Verify `curl /api/status` works from another device on same AP.

## App shows `LINKED` but no movement

- Check motor power source and common GND.
- Confirm E-STOP is released.
- Move joystick far enough (speed limit >= 0.4 recommended).
- Verify telemetry wheel velocities change during joystick hold.

## Connection drops intermittently

- Keep phone close to Pico W.
- Avoid aggressive AP power saving modes.
- Reboot Pico W and reconnect Wi-Fi.

---

## 9) Optional: STA mode instead of AP mode

If you want Pico W and phone on your existing router:

1. Edit `config.py`:
   - `WIFI_MODE = "STA"`
   - Set `STA_SSID` and `STA_PASSWORD`
2. Re-upload files.
3. Read Pico serial output to find assigned IP.
4. Use `<router-assigned-ip>:5001` in app.

---

## 10) Architecture summary

New control path:

- `App -> Pico W (HTTP API) -> motor control`

Removed from manual drive path:

- RPi5
- ROS2 bridge
- USB serial bridge process
