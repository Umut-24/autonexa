#ifndef L298N_DRIVER_H
#define L298N_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * L298N dual H-bridge motor driver — direct-control interface.
 *
 * The L298N has two independent H-bridges (Motor A, Motor B). Each
 * H-bridge takes:
 *   - 2 GPIO direction pins (IN1/IN2 for A, IN3/IN4 for B)
 *   - 1 PWM enable pin (ENA for A, ENB for B), 10 kHz
 *
 * Direction encoding for one H-bridge:
 *   IN_A  IN_B   Output
 *    0    0      coast (free-wheeling)
 *    1    0      forward
 *    0    1      reverse
 *    1    1      brake (both motor terminals tied high)
 *
 * Pin assignments and channel numbering are in `config.h`:
 *   Channel 1 = LEFT  (OUT3-OUT4 / IN3 / IN4 / ENB)
 *   Channel 2 = RIGHT (OUT1-OUT2 / IN1 / IN2 / ENA)
 *
 * Speed values use the same `[-MOTOR_SPEED_MAX, +MOTOR_SPEED_MAX]`
 * range the rest of the firmware uses. Internally the driver maps
 * that to a 0..100% PWM duty cycle on the enable pin and sets the
 * direction pins per the sign of the speed.
 *
 * No encoder feedback — this driver only commands.
 */

/** Init GPIOs and PWM for both L298N channels. Always returns true. */
bool l298n_driver_init(void);

/**
 * Set speed on a single channel (1 = left, 2 = right).
 * @param channel  1 or 2
 * @param speed    Signed value in [-MOTOR_SPEED_MAX, +MOTOR_SPEED_MAX]
 * @return         true if the channel was valid.
 */
bool l298n_set_speed(uint8_t channel, int8_t speed);

/** Set both channels in one call (matches old hiwonder_set_speeds API). */
bool l298n_set_speeds(int8_t speed_left, int8_t speed_right);

/**
 * Open-loop PWM duty injection. Used by the CLI `RAW_PWM` verb's first
 * two arguments (m1 = channel 1, m2 = channel 2). Values 3 and 4 are
 * accepted for protocol compatibility but ignored — L298N has no
 * channels 3 / 4.
 * @param duty_pct  Per-channel duty in [-100, +100].
 */
bool l298n_set_raw_pwm(int8_t m1_pct, int8_t m2_pct);

/** Coast both motors (IN_A = IN_B = 0, EN = 0). */
bool l298n_stop_all(void);

#endif /* L298N_DRIVER_H */
