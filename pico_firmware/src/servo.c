#include "servo.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"

/* ── Internal state ──────────────────────────────────────────── */
static uint   servo_slice;
static uint   servo_channel;
static float  current_angle_rad = 0.0f;

/* ── Helpers ─────────────────────────────────────────────────── */

/**
 * Clamp pulse width to safe range.
 * The slice runs at 1 MHz (125 MHz / 125) so 1 count = 1 µs.
 * Wrap is set to SERVO_PERIOD_US - 1 (19 999) for a 50 Hz period.
 */
static inline uint16_t us_to_level(uint16_t us)
{
    if (us < SERVO_PWM_MIN_US) us = SERVO_PWM_MIN_US;
    if (us > SERVO_PWM_MAX_US) us = SERVO_PWM_MAX_US;
    return us;
}

/**
 * Linear map: angle_rad → pulse width in µs.
 *
 * LD-1501MG servo: 500 µs = 0°, 1500 µs = 90° (center), 2500 µs = 180°
 *
 * angle = -MAX_STEERING_RAD  →  SERVO_PWM_MIN_US   (full right)
 * angle =  0                 →  SERVO_PWM_CENTER_US (straight)
 * angle = +MAX_STEERING_RAD  →  SERVO_PWM_MAX_US   (full left)
 */
static uint16_t angle_to_us(float angle_rad)
{
    /* Clamp */
    if (angle_rad >  MAX_STEERING_RAD) angle_rad =  MAX_STEERING_RAD;
    if (angle_rad < -MAX_STEERING_RAD) angle_rad = -MAX_STEERING_RAD;

    /* Normalise to [-1, +1] */
    float norm = angle_rad / MAX_STEERING_RAD;

    /* Map to PWM range */
    float half_range = (float)(SERVO_PWM_MAX_US - SERVO_PWM_MIN_US) / 2.0f;
    float center     = (float)SERVO_PWM_CENTER_US;
    uint16_t us      = (uint16_t)(center + norm * half_range);

    return us;
}

/* ── Public API ──────────────────────────────────────────────── */

void servo_init(void)
{
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    servo_slice   = pwm_gpio_to_slice_num(SERVO_PIN);
    servo_channel = pwm_gpio_to_channel(SERVO_PIN);

    /*
     * Clock divider:  125 MHz / 125 = 1 MHz  → 1 µs per count.
     * Wrap value:     20 000 - 1 = 19 999     → 20 ms period (50 Hz).
     */
    pwm_set_clkdiv(servo_slice, 125.0f);
    pwm_set_wrap(servo_slice, SERVO_PERIOD_US - 1);

    /* Start at center */
    pwm_set_chan_level(servo_slice, servo_channel, SERVO_PWM_CENTER_US);
    pwm_set_enabled(servo_slice, true);

    current_angle_rad = 0.0f;
}

void servo_set_angle(float angle_rad)
{
    uint16_t pulse = angle_to_us(angle_rad);
    pwm_set_chan_level(servo_slice, servo_channel, us_to_level(pulse));
    current_angle_rad = angle_rad;
}

void servo_set_pwm_us(uint16_t pulse_us)
{
    pwm_set_chan_level(servo_slice, servo_channel, us_to_level(pulse_us));
    /* Back-calculate approximate angle for telemetry */
    float half_range = (float)(SERVO_PWM_MAX_US - SERVO_PWM_MIN_US) / 2.0f;
    float norm = ((float)pulse_us - (float)SERVO_PWM_CENTER_US) / half_range;
    current_angle_rad = norm * MAX_STEERING_RAD;
}

void servo_center(void)
{
    servo_set_angle(0.0f);
}

float servo_get_angle(void)
{
    return current_angle_rad;
}
