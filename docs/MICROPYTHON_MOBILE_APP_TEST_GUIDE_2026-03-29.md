# MicroPython Mobile Control Test Guide

Date: March 29, 2026

This guide validates end-to-end control and telemetry:

`Mobile App -> micropython_bridge.py -> Pico (MicroPython) -> telemetry back to Mobile App`

## 1. Flash Pico with ready bundle

From repo root:

```bash
./test/pico/upload_mobile_bundle.sh /dev/ttyACM0
```

Check Pico output:

```bash
mpremote connect /dev/ttyACM0 repl
```

You should see:

- `PICO_READY`
- repeating telemetry CSV lines

## 2. Start MicroPython HTTP bridge on RPi/PC

Install dependencies once:

```bash
python3 -m pip install pyserial flask
```

Run bridge:

```bash
python3 test/pc/micropython_bridge.py --port /dev/ttyACM0 --http-port 5001
```

Quick API sanity check:

```bash
curl http://127.0.0.1:5001/api/status
curl http://127.0.0.1:5001/api/telemetry
```

## 3. Connect mobile app in MicroPython mode

1. Open app `Settings` tab.
2. Enter host and base port, example: `192.168.1.5:5000`.
3. Press `Connect`.
4. Enable `MicroPython Direct` mode.
5. Open `Control` tab.

Note: in MicroPython mode, the app automatically switches control API calls to port `5001`.

## 4. Execute control tests and observe app changes

## Test A: Link health

- Expected:
  - Control badge shows `LINKED` and mode `MPY`.
  - Micro state appears (`IDLE`, `DRIVE`, `TURN`, or `FAILED`).

## Test B: Joystick live control

- Move joystick forward and turn.
- Expected live changes in app:
  - `L` and `R` wheel values change from near `0.00`.
  - `DIST` and encoder ticks (`L TK`, `R TK`) update continuously.
  - `HDG` changes while turning.

## Test C: Goal commands

- Press `DRIVE` and send `0.5 m`.
- Press `TURN` and send `15 deg`.
- Expected:
  - `STATE` changes to `DRIVE` or `TURN` during motion.
  - Returns to `IDLE` when goal completes.

## Test D: Safety

- Press `E-STOP`.
- Press `STOP` goal button.
- Expected:
  - Wheel values drop near zero quickly.
  - `STATE` becomes `IDLE` or `FAILED` depending on command path.

## 5. Troubleshooting

- If app stays `NO LINK` in MicroPython mode:
  - Ensure bridge is running on `5001`.
  - Ensure phone and RPi are on same network.
  - Check firewall allows `5001/tcp`.

- If bridge starts but Pico not detected:
  - Confirm serial port path (`/dev/ttyACM0`, `/dev/ttyUSB0`, `COMx`).
  - Close Thonny/serial monitor that may lock the port.

- If motors do not move but telemetry updates:
  - Verify L298N power rail and common GND.
  - Verify pin mapping in `test/pico/mobile_app_bundle/main.py`.
