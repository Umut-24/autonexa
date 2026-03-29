# Pico MicroPython Upload Bundle (Mobile App Testing)

This folder contains the exact files to flash to a Raspberry Pi Pico for the AutoNexa mobile app MicroPython control flow.

## Files

- `boot.py`: boot-time USB + LED startup indicator.
- `main.py`: control firmware (serial JSON commands, motor/servo control, telemetry CSV output).

## Upload (recommended)

From repo root:

```bash
./test/pico/upload_mobile_bundle.sh /dev/ttyACM0
```

If your Pico uses another port, replace `/dev/ttyACM0`.

## Upload manually with mpremote

```bash
mpremote connect /dev/ttyACM0 fs cp test/pico/mobile_app_bundle/boot.py :boot.py
mpremote connect /dev/ttyACM0 fs cp test/pico/mobile_app_bundle/main.py :main.py
mpremote connect /dev/ttyACM0 reset
```

## Upload manually with Thonny

1. Open Thonny.
2. Interpreter: `MicroPython (Raspberry Pi Pico)`.
3. Open `boot.py`, then `Save as...` to `Raspberry Pi Pico` root.
4. Open `main.py`, then `Save as...` to `Raspberry Pi Pico` root.
5. Press Reset on Pico.

## Expected startup output

Open serial REPL/monitor and confirm:

- `PICO_READY`
- periodic telemetry lines:
  `t_ms,cmd,left_ticks,right_ticks,dist_m,heading_deg,state,left_pwm,right_pwm`

## Pin mapping used by firmware

- Left motor: `GP2`=`IN1`, `GP3`=`IN2`, `GP4`=`ENA PWM`
- Right motor: `GP6`=`IN3`, `GP7`=`IN4`, `GP8`=`ENB PWM`
- Servo: `GP15`
- Left encoder: `GP10` (A), `GP11` (B)
- Right encoder: `GP12` (A), `GP13` (B)

Make sure Pico/L298N/power share common GND.
