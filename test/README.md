# Nav2 -> Pico Test Package

This folder contains an end-to-end starter kit for testing processed navigation packets from a PC/RPi5 into a Raspberry Pi Pico running MicroPython.

## Folder structure

- `pico/main.py`: Pico firmware (UART JSON parser + L298N motor + servo + watchdog)
- `pc/pc_sender.py`: PC sender/logger for manual bench scenarios
- `rpi/nav2_to_pico_bridge.py`: ROS2 bridge (Nav2 topics -> JSON packets over serial)
- `scenarios/`: static scenario definitions for reporting and reuse
- `report/report_template.md`: report skeleton with test matrix
- `report/results_table.csv`: CSV header template for logs
- `logs/.gitkeep`: placeholder for generated logs

## Safety first

1. Start with wheels lifted from the ground.
2. Use a low PWM cap and low linear/angular command limits first.
3. Keep hardware emergency stop accessible.
4. Verify wiring and common ground before applying motor supply.

## Hardware wiring (example)

### Pico -> L298N (edit in `pico/main.py` if different)

- Left motor:
  - `GP2 -> IN1`
  - `GP3 -> IN2`
  - `GP4 (PWM) -> ENA`
- Right motor:
  - `GP6 -> IN3`
  - `GP7 -> IN4`
  - `GP8 (PWM) -> ENB`
- Servo:
  - `GP15 (PWM) -> servo signal`

Also ensure:
- Common GND between Pico, L298N, and power source.
- Motor supply connected correctly to L298N.

## Software prerequisites

### PC/RPi side

```bash
python3 -m pip install pyserial
```

### RPi5 Nav2 bridge

Requires ROS2 + Nav2 environment where these topics exist:
- `/cmd_vel`
- `/amcl_pose`
- `/goal_pose`
- `/plan`

## Flash/run Pico firmware

1. Copy `test/pico/main.py` to Pico as `main.py`.
2. Reboot Pico.
3. Confirm it prints `PICO_READY`.

## Run manual bench scenarios from PC

```bash
python3 test/pc/pc_sender.py --port /dev/ttyACM0 --baud 115200 --rate 10 --scenario all
```

Scenario options:
- `basic`
- `obstacle`
- `all`

Expected output: sender prints `PICO:` log lines from Pico.

## Run real Nav2 point-to-point test on RPi5

1. Launch localization + Nav2.
2. Run bridge:

```bash
python3 test/rpi/nav2_to_pico_bridge.py
```

3. Set initial pose and goal in RViz.
4. Execute at least 5 trials per route type:
   - straight
   - 90-degree turn
   - obstacle detour

## What to measure

- Packet rate, parse errors, duplicate/old packet drops
- PWM commands (left/right), direction behavior
- Emergency stop and watchdog stop latency
- Goal reach success rate and error metrics in real Nav2 tests

Use `test/report/report_template.md` to record results.

## Notes

- Default serial ports vary by OS (`/dev/ttyACM0`, `/dev/ttyUSB0`, `COMx`).
- If using USB CDC instead of UART pins on Pico, adapt the receive method in `main.py`.
- `rpi/nav2_to_pico_bridge.py` currently uses placeholder obstacle distances unless you connect obstacle topic processing.
