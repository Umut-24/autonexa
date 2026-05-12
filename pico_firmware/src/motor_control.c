#include "motor_control.h"
#include "config.h"
#include "ackermann.h"
#include "servo.h"
#include "l298n_driver.h"
#include "safety.h"

#include <math.h>

/* ── State ──────────────────────────────────────────────────── */
static float   target_steer_rad = 0.0f;
static int8_t  speed_left       = 0;
static int8_t  speed_right      = 0;
static bool    motors_enabled   = false;

/* ── Velocity-to-speed scale factor ─────────────────────────── */
/* Open-loop on the L298N — without encoder feedback we can't tie a
 * commanded m/s to an actual rev/s. The scale is a heuristic so that
 * `vx = 0.30 m/s → SPEED 30 → 100% PWM duty`. Refine empirically once
 * encoders are connected. Battery voltage and load both affect actual
 * speed at any given duty.
 *
 *   speed_cli = round(vx_mps * VEL_TO_SPEED_SCALE)        (this file)
 *   pwm_pct   = (speed_cli * 100) / MOTOR_SPEED_MAX       (l298n_driver.c)
 */
#define VEL_TO_SPEED_SCALE  (100.0f)

/* ── API ────────────────────────────────────────────────────── */

void motor_control_set_velocity(float vx, float wz)
{
    if (safety_is_estopped()) {
        speed_left = 0;
        speed_right = 0;
        return;
    }

    float steer, v_l, v_r;
    ackermann_inverse(vx, wz, &steer, &v_l, &v_r);

    /* Convert m/s to driver board speed units (closed-loop pulses/10ms).
     * Round to the nearest integer rather than truncating, so that small
     * velocities (e.g. final parking approach at 0.03 m/s ~ 1.9 units)
     * actually produce a non-zero command instead of collapsing to 0. */
    int8_t sl = (int8_t)roundf(v_l * VEL_TO_SPEED_SCALE);
    int8_t sr = (int8_t)roundf(v_r * VEL_TO_SPEED_SCALE);

    if (sl >  MOTOR_SPEED_MAX) sl =  MOTOR_SPEED_MAX;
    if (sl <  MOTOR_SPEED_MIN) sl =  MOTOR_SPEED_MIN;
    if (sr >  MOTOR_SPEED_MAX) sr =  MOTOR_SPEED_MAX;
    if (sr <  MOTOR_SPEED_MIN) sr =  MOTOR_SPEED_MIN;

    speed_left       = sl;
    speed_right      = sr;
    target_steer_rad = steer;

    servo_set_angle(steer);
}

static int8_t clamp_speed(int8_t s)
{
    if (s > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
    if (s < MOTOR_SPEED_MIN) return MOTOR_SPEED_MIN;
    return s;
}

void motor_control_set_speeds(int8_t left, int8_t right)
{
    speed_left  = clamp_speed(left);
    speed_right = clamp_speed(right);
}

void motor_control_set_speed_left(int8_t left)
{
    speed_left = clamp_speed(left);
}

void motor_control_set_speed_right(int8_t right)
{
    speed_right = clamp_speed(right);
}

void motor_control_enable(bool enable)
{
    if (enable && safety_is_estopped()) {
        motors_enabled = false;
        speed_left = 0;
        speed_right = 0;
        hiwonder_stop_all();
        servo_center();
        return;
    }

    motors_enabled = enable;
    if (enable) {
        safety_feed_watchdog();
    } else {
        speed_left  = 0;
        speed_right = 0;
        l298n_stop_all();
    }
}

void motor_control_stop(void)
{
    speed_left  = 0;
    speed_right = 0;
    target_steer_rad = 0.0f;
    hiwonder_stop_all();
    servo_center();
}

void motor_control_emergency_stop(void)
{
    motors_enabled = false;
    motor_control_stop();
}

bool motor_control_is_enabled(void)
{
    return motors_enabled;
}

void motor_control_apply(void)
{
    if (motors_enabled && safety_is_ok()) {
        hiwonder_set_speeds(speed_left, speed_right);
    } else {
        hiwonder_stop_all();
    }
}

int8_t motor_control_get_speed_left(void)
{
    return speed_left;
}

int8_t motor_control_get_speed_right(void)
{
    return speed_right;
}

float motor_control_get_steer_rad(void)
{
    return target_steer_rad;
}
