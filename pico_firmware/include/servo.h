#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/**
 * Steering servo driver.
 *
 * Drives a standard hobby servo at 50 Hz PWM.
 * Provides both raw PWM control (for calibration) and angle-based control
 * using a linear map between steering angle and pulse width.
 */

/** Initialize servo PWM on the configured GPIO pin. */
void servo_init(void);

/**
 * Set servo to a steering angle in radians.
 * Clamped to [-MAX_STEERING_RAD, +MAX_STEERING_RAD].
 *   negative = right turn
 *   positive = left turn
 */
void servo_set_angle(float angle_rad);

/**
 * Set servo pulse width directly (for calibration).
 * @param pulse_us  Pulse width in microseconds [SERVO_PWM_MIN_US .. SERVO_PWM_MAX_US]
 */
void servo_set_pwm_us(uint16_t pulse_us);

/** Center the steering (set to 0 rad / center PWM). */
void servo_center(void);

/** Get the current commanded angle in radians. */
float servo_get_angle(void);

#endif /* SERVO_H */
