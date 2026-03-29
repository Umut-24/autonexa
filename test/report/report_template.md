# Nav2-to-Pico Integration Test Report Template

## 1) System under test
- Controller: Raspberry Pi Pico (MicroPython)
- Motor driver: L298N
- Actuators: 2x DC motors + 1x servo
- Navigation source: PC manual packets and/or Nav2 bridge from RPi5

## 2) Test setup
- Date:
- Operator:
- Battery voltage:
- Wheel diameter:
- Track width:
- PWM frequency:
- Watchdog timeout:
- Max PWM cap:

## 3) Acceptance criteria
- Emergency stop reaction time < 100 ms
- Watchdog stop reaction time < 300 ms
- Safe handling of malformed packets (no unintended motion)
- Goal reach success >= 80% (4/5) for each route in real navigation test
- Final position error < 0.20 m
- Final heading error < 10 deg

## 4) Test cases and results

| ID | Objective | Input | Expected behavior | Observed behavior | Measurements | Result |
|---|---|---|---|---|---|---|
| TC01 | Valid packet parsing @10Hz | Well-formed JSON stream | No parse errors, stable control |  | rate=, parse_err= |  |
| TC02 | Malformed JSON handling | Bad JSON syntax | Packet rejected, safe motion |  | parse_err= |  |
| TC03 | Missing-field handling | Missing required key | Packet rejected |  | parse_err= |  |
| TC04 | Duplicate/out-of-order seq | seq: 10,11,11,9,12 | Old/duplicate dropped |  | drop_old= |  |
| TC05 | Straight forward mapping | v_lin>0, v_ang=0 | Both wheels forward |  | pwm_l=, pwm_r= |  |
| TC06 | Reverse mapping | v_lin<0, v_ang=0 | Both wheels reverse |  | pwm_l=, pwm_r= |  |
| TC07 | In-place rotate mapping | v_lin=0, v_ang!=0 | Opposite wheel directions |  | yaw_rate= |  |
| TC08 | Curved motion mapping | v_lin>0, v_ang!=0 | Differential PWM |  | pwm_diff= |  |
| TC09 | Saturation/clamp behavior | Excessive command values | PWM safely clamped |  | max_pwm= |  |
| TC10 | Emergency stop behavior | obstacle.emergency_stop=true | Immediate stop |  | t_stop_ms= |  |
| TC11 | Watchdog behavior | Stop packets > timeout | Motors stop |  | t_stop_ms= |  |
| TC12 | Obstacle speed policy | front_m: 0.5 -> 0.2 | Speed scale then stop |  | front_m_vs_pwm= |  |
| TC13 | Servo state behavior | TRACKING/RECOVERY/etc. | Servo target/sweep respected |  | angle= |  |
| TC14 | Endurance stability | 30-60 min run | No freeze or unsafe event |  | parse_err=, watchdog= |  |

## 5) Real navigation point-to-point (A->B)

### Route types
1. Straight segment (1-2 m)
2. 90-degree turn route
3. Obstacle detour route

### Trial results table
| Route | Trial | Success (Y/N) | Time to goal (s) | Final pos error (m) | Final heading error (deg) | Notes |
|---|---:|---|---:|---:|---:|---|
| Straight | 1 |  |  |  |  |  |
| Straight | 2 |  |  |  |  |  |
| Straight | 3 |  |  |  |  |  |
| Straight | 4 |  |  |  |  |  |
| Straight | 5 |  |  |  |  |  |
| 90-turn | 1 |  |  |  |  |  |
| 90-turn | 2 |  |  |  |  |  |
| 90-turn | 3 |  |  |  |  |  |
| 90-turn | 4 |  |  |  |  |  |
| 90-turn | 5 |  |  |  |  |  |
| Obstacle | 1 |  |  |  |  |  |
| Obstacle | 2 |  |  |  |  |  |
| Obstacle | 3 |  |  |  |  |  |
| Obstacle | 4 |  |  |  |  |  |
| Obstacle | 5 |  |  |  |  |  |

## 6) Summary
- Overall pass/fail:
- Major issues found:
- Parameter changes made:
- Next actions:
