# Debug Session — 2026-04-26

End-to-end bringup attempt on the bench. Started: "servo doesn't move." Ended:
servo confirmed damaged, DC motor + driver chain isolated to the
"between driver outputs and motors" stage with the direct-12 V bench test
still pending verification.

## Quick status

| Subsystem | Status at end of session |
|---|---|
| Pico Pico WH hardware | ✅ Verified working |
| Pico C firmware (`autonexa_pico.uf2`) | ✅ Verified working — `SERVO_PWM`, `RAW_PWM`, `I2C_SCAN`, `I2C_WRITE`, `I2C_READ`, `ENC_READ` all correct |
| GP15 servo PWM output | ✅ 3.3 V open-circuit confirmed; SG90 swept full range |
| Buck converter (XL4016, 300 W / 8 A) | ✅ Working — CC limit ~5.4 A, voltage 5–8 V tested |
| LD-1501MG steering servo | ❌ Damaged — replace |
| SG90 (stand-in) | ✅ Works at 5 V buck for steering-logic testing only (no torque) |
| I2C bus to motor driver | ⚠ Chronic — needs external pull-ups |
| Motor driver MOTOR_TYPE config | ⚠ Wiped by brownout — workaround in standalone test, real fix pending |
| DC motors (M1, M2) | ⚠ Inconclusive — direct-12 V bench test still pending |
| 12 V battery / power chain | ⚠ Sag to 0–11 V under load — wire gauge or battery IR suspect |

## Issues found, by category

### 1. Wrong firmware on Pico at session start

- **Symptom:** servo not moving with any command.
- **Cause:** Pico had a leftover MicroPython sketch (11 501 bytes) that tried to drive the servo over I2C as if it were a Hiwonder bus servo at `0x32`. The LD-1501MG is a standard 3-wire PWM hobby servo — no I2C interface.
- **Fix:** flashed `pico_firmware/build/autonexa_pico.uf2`. Confirmed `SERVO_SWEEP` produces 12 clean pulse-width changes on GP15.
- **Lesson:** always verify which firmware is on the Pico before debugging "servo doesn't respond" — `lsusb` ID `2e8a:000a` = C firmware, `2e8a:0005` = MicroPython.

### 2. LD-1501MG steering servo damaged

- **Symptom progression:**
  - At 6.5 V buck: constant ~10 Hz ticking, no rotation
  - At 6.96 V: 1–2° rotation then stuck
  - At 8 V: no movement at all (degraded further)
- **Diagnostic gauntlet:**
  - Pico GP15 reads 3.3 V open-circuit, drops to **2.45 V** with servo connected → signal input has ~350 Ω to GND (should be MΩ — partially damaged input stage)
  - Buck CC limit measured 5.4 A — not current-limited
  - Buck voltage held rock-steady 7 V during motion attempts — no power sag
  - Servo horn detached / free → same behavior, not mechanical
  - **SG90 swapped onto identical wiring at 5 V** → swept full range cleanly → eliminates Pico, firmware, buck, wiring
- **Probable cause:** earlier session running at 6.5 V (below 6.6 V spec floor) for several minutes while buck CC was uncalibrated. Repeated brownout/restart cycles likely cooked the servo's internal driver IC + input protection.
- **Status:** **dead, replace.** Order LD-1501MG (or LD-3015MG / equivalent 17–20 kg·cm metal-gear digital servo, 6.6–7.4 V, 500–2500 µs PWM @ 50 Hz).
- **For next time:** when the new servo arrives, **set buck to 7.0 V before plugging it in**. Never below 6.6 V.

### 3. I2C bus reliability issues

- **Symptom:** intermittent `ret=-1` failures on I2C reads/writes to the Hiwonder motor driver at `0x34`.
- **Failure rate during 90 s standalone test:** 42 % at start, **75–80 % by end** — bus degrades over time.
- **Pattern:** writes work more reliably than reads. Classic missing-pull-up signature: SDA can't rise fast enough between bytes during a read transaction.
- **Quick fix mid-session:** reseating the I2C wires made the bus much more reliable.
- **Permanent fix needed:** add external **4.7 kΩ pull-ups** from SDA (GP0) and SCL (GP1) to 3.3 V. The RP2040's internal `gpio_pull_up()` (~50 kΩ) is borderline at 100 kHz with breadboard wiring.
- **Code reference:** `pico_firmware/src/hiwonder_driver.c:102-103` only enables internal pulls.

### 4. Motor driver MOTOR_TYPE config wiped by brownout

- **Symptom:** I2C writes succeed (`ret=5`), but motors don't respond to any command.
- **Discovery:** read register 20 (`MOTOR_TYPE`) directly → returned `0x00` (= `MOTOR_TYPE_WITHOUT_ENCODER`), not the expected `0x03` (`MOTOR_TYPE_JGB37`).
- **Why:** `hiwonder_driver_init()` writes MOTOR_TYPE only at boot. If I2C is flaky at the boot moment (which it was, because of the loose wire in #3), the init write is corrupted and the driver retains its default of 0. No re-write happens for the rest of the session.
- **Workaround in `dc_motor_test.c`:** verify-after-write, re-write at the start of every cycle.
- **Permanent fix needed in main firmware:** in `hiwonder_driver.c`, periodically (e.g., once per second from `safety.c`) re-write MOTOR_TYPE. A single dropped frame should not permanently break motor control.

### 5. Closed-loop runaway on right motor

- **Symptom:** with closed-loop `SPEED` commands active, the right motor (M2) spun fast back-and-forth even after commands stopped.
- **Cause:** wrong encoder polarity (`MOTOR_ENCODER_POLARITY` register 21) on the M2 channel. Per Hiwonder docs: *"If the motor speed is completely out of control, reset this address."*
- **Mitigation in `dc_motor_test.c`:** OPEN-LOOP PWM ONLY (register 0x1F). Never sends closed-loop SPEED commands, so polarity inversion can't cause oscillation.
- **Permanent fix needed:** verify encoder polarity per channel before enabling closed-loop. Either probe each motor's encoder direction at init and adjust, or expose a one-time calibration command.

### 6. DC motors barely moving on any commanded PWM

- **Test:** 90 s standalone open-loop firmware, individual motors at ±30 %, four 2.5 s phases per cycle, 5 cycles.
- **Result:** **encoders read M1=154 M2=1 unchanged** across the entire 90 s test. Net delta = 0 for every phase except cycle 1's misleading "delta=154" artifact (failed start-read returned 0, end-read returned the persistent 154).
- **Visual observation:** both motors twitched briefly in sync with successful commands but never sustained rotation.
- **What's eliminated:**
  - ❌ Software bug — brand-new minimal firmware, registers verified, motor type re-written every cycle
  - ❌ Closed-loop polarity oscillation — open-loop only
  - ❌ Wrong register addresses — verified against Hiwonder reference C code shared by user
- **Remaining suspects:**
  - 12 V power not reaching the H-bridges (driver runs on logic 3.3 V from I2C → acks commands but can't drive motors)
  - Driver's H-bridge silicon dead (especially M1)
  - Motors themselves damaged (worn brushes, partial winding short)
  - 12 V battery internal resistance too high → collapses under load
  - Wires between battery → driver too thin → voltage drops in transit
- **Pending decisive test (not yet done):** disconnect motor wires from driver, touch directly to 12 V battery for 1 s. Healthy motor = fast spin. Tick or no spin = motor itself damaged.

### 7. 12 V battery sag under load

- **Observed during sustained `RAW_PWM 30 30 0 0`:** voltage at driver's 12 V input terminals fluctuated `0 V / 2 V / 9 V / 11 V` rapidly — not steady sag, but oscillation.
- **No-load reading:** reportedly 12 V at battery terminals.
- **Driver's red power LED:** blinking continuously throughout — UVLO active during every motion attempt.
- **Likely contributors (rank uncertain):**
  - Battery internal resistance after repeated stress
  - Wire gauge between battery and driver
  - Loose connector at battery → driver junction
  - Driver pulling rail down on each H-bridge switching event
- **Test to disambiguate:** measure voltage **at the battery terminals** during load — if battery terminals also collapse, battery is the issue; if battery stays steady but driver input collapses, the wire or connector is.

## Files created / modified today

| File | What |
|---|---|
| `pico_firmware/src/dc_motor_test.c` | NEW — standalone open-loop motor diagnostic, auto-stops after 90 s |
| `pico_firmware/CMakeLists.txt` | NEW build target `dc_motor_test` (produces its own UF2) |
| `pico_firmware/build/dc_motor_test.uf2` | Built artifact — flash to test motors without main firmware |
| `~/.claude/.../memory/MEMORY.md` | Updated servo memory entry pointer |
| `~/.claude/.../memory/project_servo_dead_2026_04_26.md` | Updated with corrected diagnosis (severely damaged, not totally dead) |

## Action items for next session

### Hardware (in order)

1. **Direct-12 V bench test on each motor** — touch each motor's leads to the 12 V battery for 1 s. Spin = motor good; tick or silence = motor damaged. **This is the one test that ends the ambiguity.**
2. **Order replacement LD-1501MG** (or LD-3015MG / equivalent ≥17 kg·cm metal-gear digital servo).
3. **Add external 4.7 kΩ pull-ups** on I2C SDA (GP0) and SCL (GP1) to 3.3 V. Will fix the chronic bus failure rate.
4. **Battery health check** — measure voltage at battery terminals (not just driver input) during load. If sag, replace or recharge to full 12.6 V.
5. **Possibly replace 12 V wires** between battery and driver with thicker gauge if existing are < 18 AWG.

### Firmware

1. **Periodic re-write of `MOTOR_TYPE`** in `pico_firmware/src/hiwonder_driver.c` or `safety.c` — once per second is enough. Prevents a single bad I2C frame from permanently disabling motors.
2. **Encoder polarity verification at startup** — small routine that nudges each motor briefly and checks encoder direction matches expected. Logs warning if polarity needs flipping.
3. **Keep `dc_motor_test.c`** as a debug tool. The auto-stop + isolation pattern is useful for any future motor debugging.

### Out of scope until hardware is sorted

- TEST.md Stage 3 onward (motor + steering)
- Any Nav2 / SLAM end-to-end integration
- micro-ROS bridge testing

## Key diagnostic techniques learned this session

1. **GP15 collapse test for servo input damage:** probe Pico GPIO during steady-HIGH (`SERVO_PIN_HOLD`) with servo disconnected vs connected. Healthy servo: 3.3 V both. Shorted/loaded input: drops to ≤2.5 V or 0 V.
2. **DMM cannot show servo PWM:** at 5–10 % duty cycle, average DC = 0.16–0.33 V → DMM rounds to 0. Use `SERVO_PIN_HOLD` (steady GPIO) for DMM verification.
3. **Direct register read for driver state:** `I2C_READ 20 1` (MOTOR_TYPE), `I2C_READ 0 1` (battery ADC). Don't trust higher-level commands — go to the registers when stuck.
4. **Open-loop only when debugging:** any closed-loop test risks runaway if encoder polarity is wrong. Always validate open-loop motion first, then enable closed-loop after polarity is confirmed.
5. **`ret=` values in firmware logs are gold:** `ret=5` = 5 bytes written (success), `ret=-1` = `PICO_ERROR_GENERIC` (slave didn't ACK). Always log them; failure rate over time tells you bus health.
