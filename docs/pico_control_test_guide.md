# Pico Control Testing Guide — RPi5 to Pico

Step-by-step guide for configuring the Pico firmware, connecting it to the RPi5, and testing the full control chain with manual commands.

## 1. Hardware Setup

### Connections

```
RPi5  ←── USB cable ──→  Pico (USB-C / micro-USB)
                         Pico GPIO 0 (SDA) ──→ Hiwonder motor driver board (I2C)
                         Pico GPIO 1 (SCL) ──→ Hiwonder motor driver board (I2C)
                         Pico GPIO 12 (PWM) ──→ LD-1501MG steering servo (signal)
                         Hiwonder board ←── 12V power ──→ Battery
                         Servo ←── 6-8.4V ──→ Power supply / BEC
```

### Pico Wiring Summary

| Pico GPIO | Function | Target |
|-----------|----------|--------|
| 0 | I2C0 SDA | Motor driver board SDA |
| 1 | I2C0 SCL | Motor driver board SCL |
| 12 | PWM (50 Hz) | Steering servo signal |
| 25 | LED heartbeat | On-board LED |
| USB | Serial (115200 baud) | RPi5 USB port |

### Motor Driver Board (I2C address: 0x34)

| Motor Channel | Wheel |
|---------------|-------|
| M2 | Left rear |
| M4 | Right rear |
| M1, M3 | Unused (set to 0) |

## 2. Building and Flashing the Pico Firmware

### Prerequisites (on RPi5 or build machine)

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

If not already done, clone the Pico SDK:
```bash
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init
export PICO_SDK_PATH=~/pico-sdk
```

### Build — Serial CLI mode (for manual testing)

```bash
cd ~/intelligent_parking_ws/src/autonexa/pico_firmware
mkdir -p build && cd build
cmake -DPICO_SDK_PATH=$PICO_SDK_PATH ..
make -j4
```

Output: `autonexa_pico.uf2`

### Build — micro-ROS mode (for ROS2 integration)

Requires `micro_ros_sdk/libmicroros/libmicroros.a` to be present.

```bash
cd ~/intelligent_parking_ws/src/autonexa/pico_firmware
mkdir -p build_uros && cd build_uros
cmake -DPICO_SDK_PATH=$PICO_SDK_PATH -DUSE_MICRO_ROS=ON ..
make -j4
```

Output: `autonexa_pico_uros.uf2`

### Flash the Pico

1. Hold the **BOOTSEL** button on the Pico.
2. While holding, plug the USB cable into the RPi5. The Pico mounts as a USB drive.
3. Copy the UF2 file:
   ```bash
   cp autonexa_pico.uf2 /media/$USER/RPI-RP2/
   ```
4. The Pico reboots automatically. The on-board LED blinks 4 times on boot.

## 3. Connecting from RPi5 — Serial CLI Mode

### Find the serial port

```bash
ls /dev/ttyACM*
```

Typically `/dev/ttyACM0`. If multiple devices, unplug/replug and check `dmesg | tail`.

### Open serial terminal

```bash
# Using minicom:
sudo apt install minicom
minicom -b 115200 -D /dev/ttyACM0

# Or using screen:
screen /dev/ttyACM0 115200

# Or using Python:
python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
ser.write(b'HELP\n')
time.sleep(0.5)
print(ser.read(ser.in_waiting).decode())
ser.close()
"
```

You should see the firmware banner:
```
╔═══════════════════════════════════════════╗
║  AUTONEXA Pico Firmware v2.0              ║
║  Hiwonder Ackermann Chassis               ║
║  Phase 1 — Bench Test (Serial CLI)        ║
║  Control freq: 50 Hz                      ║
╚═══════════════════════════════════════════╝
Type HELP for commands.

[HW_DRV] Motor driver board detected at 0x34
[HW_DRV] Motor type=JGB37, encoder polarity=default
```

If you see `Motor driver board NOT detected`, check I2C wiring and power.

## 4. Serial CLI Command Reference

### Servo (Steering)

| Command | Description | Example |
|---------|-------------|---------|
| `SERVO_CENTER` | Center steering (1500 us) | `SERVO_CENTER` |
| `SERVO_ANGLE <rad>` | Set steering angle (±0.52 rad = ±30 deg) | `SERVO_ANGLE 0.26` |
| `SERVO_PWM <us>` | Raw PWM pulse width (500-2500 us) | `SERVO_PWM 1500` |

### Motor Control (Closed-Loop PID)

| Command | Description | Example |
|---------|-------------|---------|
| `ENABLE` | Enable motors (required before any motion) | `ENABLE` |
| `DISABLE` | Disable motors (stops and locks out) | `DISABLE` |
| `SPEED <val>` | Both motors, closed-loop (±30 pulses/10ms) | `SPEED 10` |
| `SPEED_L <val>` | Left motor only | `SPEED_L 15` |
| `SPEED_R <val>` | Right motor only | `SPEED_R 15` |
| `STOP` | Stop all motors + center steering | `STOP` |

### Ackermann Velocity

| Command | Description | Example |
|---------|-------------|---------|
| `VEL <vx> <wz>` | Ackermann cmd: linear (m/s) + angular (rad/s) | `VEL 0.1 0.0` |

### Encoder & Diagnostics

| Command | Description | Example |
|---------|-------------|---------|
| `ENC_READ` | Read accumulated encoder ticks (L, R) | `ENC_READ` |
| `STATUS` | Full state: enable, estop, timeout, speed, steer, odom | `STATUS` |
| `I2C_SCAN` | Scan I2C bus for devices | `I2C_SCAN` |

### Safety

| Command | Description | Example |
|---------|-------------|---------|
| `ESTOP` | Emergency stop (latching — motors locked) | `ESTOP` |
| `ESTOP_CLEAR` | Clear E-STOP (resumes normal operation) | `ESTOP_CLEAR` |

### Low-Level Debug (bypass motor_control layer)

| Command | Description | Example |
|---------|-------------|---------|
| `RAW_PWM <m1> <m2> <m3> <m4>` | Open-loop PWM to register 0x1F (-100..100) | `RAW_PWM 0 30 0 30` |
| `RAW_PID <m1> <m2> <m3> <m4>` | Closed-loop to register 0x33 (pulses/10ms) | `RAW_PID 0 10 0 10` |
| `I2C_WRITE <reg> <val>` | Raw I2C register write | `I2C_WRITE 20 3` |
| `I2C_READ <reg> <len>` | Raw I2C register read (hex output) | `I2C_READ 60 16` |

### Telemetry Output (automatic, 10 Hz)

The firmware prints telemetry every 100ms while running:

```
TEL <ms>,<speed_L>,<speed_R>,<steer_rad>,<enc_L>,<enc_R>,<odom_x>,<odom_y>,<odom_yaw>,<odom_vx>,<odom_wz>,<estop>,<timeout>
```

Example:
```
TEL 12340,10,10,0.000,4521,4530,0.142,0.001,0.01,0.105,0.002,0,0
```

`odom_vx` [m/s] and `odom_wz` [rad/s] were added 2026-05-16 — 13 CSV fields
total. `enc_L`/`enc_R` are signed 4x-quadrature counts; `odom_*` is the
differential-drive odometry integrated from the two wheel encoders.

## 5. Test Procedures

### Test 1: Verify I2C Connection

```
I2C_SCAN
```

Expected output:
```
I2C scan on bus 0:
  Found device at 0x34
Scan complete.
```

If 0x34 is not found: check wiring (SDA→GPIO0, SCL→GPIO1), check power to driver board.

### Test 2: Servo Steering

```
SERVO_CENTER
SERVO_ANGLE 0.26
SERVO_ANGLE -0.26
SERVO_ANGLE 0.0
SERVO_PWM 1000
SERVO_PWM 2000
SERVO_PWM 1500
```

Verify:
- Center command → wheels point straight
- ±0.26 rad (±15 deg) → wheels turn visibly left/right
- PWM 1000/2000 → near-max steering in each direction

### Test 3: Motor — Closed-Loop Speed

```
ENABLE
SPEED 10
```

Wait 2 seconds, then:
```
ENC_READ
```

Wait 2 more seconds:
```
ENC_READ
STOP
```

Verify:
- Both wheels spin forward at constant speed
- Encoder counts increase steadily between reads
- `STOP` halts both wheels and centers steering

### Test 4: Individual Wheel Control

```
ENABLE
SPEED_L 10
```

Verify only the left wheel spins. Then:

```
SPEED_L 0
SPEED_R 10
```

Verify only the right wheel spins. Then:

```
STOP
```

If the wrong wheel spins, swap `MOTOR_CHANNEL_LEFT` and `MOTOR_CHANNEL_RIGHT` in `config.h`.

### Test 5: Ackermann Velocity Command

Straight line:
```
ENABLE
VEL 0.1 0.0
```

Expected response:
```
OK VEL vx=0.100 wz=0.000 → steer=0.000 L=6 R=6
```

Verify: both wheels spin equally, steering centered.

Right turn:
```
VEL 0.1 0.3
```

Verify: steering turns right, outer wheel faster than inner.

Left turn:
```
VEL 0.1 -0.3
```

Verify: steering turns left.

Stop:
```
STOP
```

### Test 6: Encoder Odometry

```
ENABLE
VEL 0.1 0.0
```

Wait 5 seconds, then:
```
STATUS
```

Check `odom x=...` — should show ~0.5m of forward travel (0.1 m/s × 5s). Then:

```
STOP
```

### Test 7: Safety — Command Timeout

```
ENABLE
SPEED 10
```

Now **do not send any command for 200ms**. The firmware should auto-stop (watchdog timeout). Verify:

```
STATUS
```

Should show `timeout=1`. Motors should have stopped. Send any new command to resume:

```
VEL 0.1 0.0
STATUS
```

Should show `timeout=0`.

### Test 8: Safety — E-STOP

```
ENABLE
SPEED 15
ESTOP
```

Verify: motors stop immediately, LED blinks fast (5 Hz). Try to move:

```
SPEED 10
```

Should have no effect (E-STOP is latching). Clear it:

```
ESTOP_CLEAR
ENABLE
SPEED 10
```

Motors should resume.

### Test 9: Encoder Direction Verification

```
ENABLE
SPEED 10
ENC_READ
```

Wait 2 seconds:
```
ENC_READ
STOP
```

Both encoder values should **increase** when driving forward. If one decreases, its encoder polarity is reversed. Fix by changing `ENCODER_POLARITY_DEFAULT` to `1` for that channel in the init code, or swap the motor wires.

### Test 10: Closed-Loop vs Open-Loop Comparison

Open-loop (raw PWM, no PID):
```
RAW_PWM 0 30 0 30
```

Wait 3 seconds, read encoders, then stop:
```
ENC_READ
RAW_PWM 0 0 0 0
```

Closed-loop (board PID):
```
RAW_PID 0 10 0 10
```

Wait 3 seconds, read encoders, then stop:
```
RAW_PID 0 0 0 0
ENC_READ
```

The closed-loop mode should give more consistent encoder deltas per interval, especially under varying load.

## 6. Connecting via micro-ROS (ROS2 Mode)

Flash `autonexa_pico_uros.uf2` instead of the serial CLI build.

### Start micro-ROS agent on RPi5

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

The Pico LED blinks fast while searching for the agent, then stops when connected.

### Verify Pico topics appear

```bash
ros2 topic list | grep pico
```

Expected:
```
/pico/control_cmd
/pico/enable
/pico/odom
/pico/joint_feedback
```

### Enable motors

```bash
ros2 topic pub --once /pico/enable std_msgs/msg/Bool "{data: true}"
```

### Send velocity command

```bash
ros2 topic pub /pico/control_cmd geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}" \
  --rate 10
```

The `--rate 10` keeps the watchdog alive (sends at 10 Hz, timeout is 200ms).

### Monitor odometry

```bash
ros2 topic echo /pico/odom
```

### Monitor joint feedback

```bash
ros2 topic echo /pico/joint_feedback
```

### E-STOP via service

```bash
# Activate E-STOP:
ros2 service call /pico/estop std_srvs/srv/SetBool "{data: true}"

# Clear E-STOP:
ros2 service call /pico/estop std_srvs/srv/SetBool "{data: false}"
```

### Stop (disable motors)

```bash
ros2 topic pub --once /pico/enable std_msgs/msg/Bool "{data: false}"
```

## 7. micro-ROS Topic Reference

| Topic | Type | Direction | Rate | Description |
|-------|------|-----------|------|-------------|
| `/pico/control_cmd` | `TwistStamped` | RPi5 → Pico | ≥5 Hz | Velocity command (vx m/s, wz rad/s) |
| `/pico/enable` | `Bool` | RPi5 → Pico | Once | Motor enable/disable |
| `/pico/odom` | `Odometry` | Pico → RPi5 | 20 Hz | Wheel odometry (x, y, yaw, vx, wz) |
| `/pico/joint_feedback` | `JointState` | Pico → RPi5 | 10 Hz | Wheel velocities (rad/s) + steering angle |
| `/pico/estop` | `SetBool` (srv) | RPi5 → Pico | On demand | E-STOP activate (true) / clear (false) |

## 8. Key Configuration Parameters

All tunable values are in `pico_firmware/include/config.h`:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `WHEELBASE_M` | 0.25 | Front-to-rear axle distance (m) |
| `WHEEL_RADIUS_M` | 0.033 | Wheel radius (m) |
| `TRACK_WIDTH_M` | 0.20 | Left-to-right wheel distance (m) |
| `MAX_STEERING_RAD` | 0.5236 | Max steering angle (±30 deg) |
| `MOTOR_DRIVER_ADDR` | 0x34 | I2C address of motor driver board |
| `MOTOR_CHANNEL_LEFT` | 2 (M2) | Left rear motor channel |
| `MOTOR_CHANNEL_RIGHT` | 4 (M4) | Right rear motor channel |
| `MOTOR_TYPE_JGB37` | 3 | Motor type for board init (JGB37-520) |
| `MOTOR_SPEED_MAX` | 30 | Max closed-loop speed (pulses/10ms) |
| `SERVO_PIN` | 12 | Steering servo GPIO |
| `CONTROL_FREQ_HZ` | 50 | Main loop frequency |
| `CMD_TIMEOUT_MS` | 200 | Watchdog timeout (ms) |

### Changing max speed

Edit `MOTOR_SPEED_MAX` in `config.h`. The velocity scale in `motor_control.c` (`VEL_TO_SPEED_SCALE = 63.7`) maps 0.3 m/s to ~19 pulses/10ms. If your motors have different specs, recalculate:

```
pulses_per_10ms = (speed_mps / wheel_circumference) * ENCODER_EDGES_PER_REV / 100
```

## 9. Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Motor driver board NOT detected` | I2C wiring or no power to board | Check SDA/SCL, check 12V power |
| Motors don't spin after `SPEED 10` | Motors not enabled | Send `ENABLE` first |
| Motors stop after ~200ms | Watchdog timeout (no command refresh) | Send commands at ≥5 Hz, or use `VEL` which the loop keeps alive |
| Wrong wheel spins | Channel mapping reversed | Swap `MOTOR_CHANNEL_LEFT`/`RIGHT` in `config.h` |
| Encoder counts decrease on forward | Encoder polarity wrong | Change polarity in `hiwonder_driver_init()` or swap motor wires |
| Steering reversed | Servo mapping inverted | Adjust `SERVO_PWM_MIN_US`/`MAX_US` or negate angle in `servo_set_angle()` |
| `ESTOP` won't clear | Must explicitly call `ESTOP_CLEAR` | Send `ESTOP_CLEAR` then `ENABLE` |
| No serial output on RPi5 | Wrong port or baud rate | Check `ls /dev/ttyACM*`, use 115200 baud |
| micro-ROS topics don't appear | Agent not running or Pico not connected | Start agent first: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200` |

## 10. Quick-Start Cheat Sheet

```bash
# === Serial CLI Mode ===
# 1. Flash autonexa_pico.uf2 via BOOTSEL
# 2. Connect from RPi5:
minicom -b 115200 -D /dev/ttyACM0

# 3. Basic test sequence:
I2C_SCAN                    # Verify driver board at 0x34
SERVO_CENTER                # Center steering
ENABLE                      # Enable motors
VEL 0.1 0.0                # Drive straight at 0.1 m/s
STATUS                      # Check odometry
STOP                        # Stop everything

# === micro-ROS Mode ===
# 1. Flash autonexa_pico_uros.uf2 via BOOTSEL
# 2. Start agent:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200

# 3. Enable + drive:
ros2 topic pub --once /pico/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub /pico/control_cmd geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}" --rate 10

# 4. Monitor:
ros2 topic echo /pico/odom
ros2 topic echo /pico/joint_feedback

# 5. Stop:
ros2 topic pub --once /pico/enable std_msgs/msg/Bool "{data: false}"
```
