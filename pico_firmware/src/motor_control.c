#include "motor_control.h"
#include "config.h"
#include "ackermann.h"
#include "servo.h"
#include "hiwonder_driver.h"
#include "safety.h"

#include <math.h>

/* ── State ──────────────────────────────────────────────────── */
static float   target_steer_rad = 0.0f;
static int8_t  speed_left       = 0;
static int8_t  speed_right      = 0;
static bool    motors_enabled   = false;

/* ── Velocity-to-speed scale factor ─────────────────────────── */
/* Closed-loop speed register (0x33) uses pulses per 10ms.
 * Wheel circumference = 2π × 0.033m = 0.2073m
 * At 0.3 m/s: 0.3/0.2073 = 1.447 rev/s × 1320 edges = 1910 pulses/s
 *           = ~19.1 pulses per 10ms
 * Scale: 19.1 / 0.3 ≈ 63.7 */
#define VEL_TO_SPEED_SCALE  (63.7f)

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

    /* Convert m/s to driver board speed units */
    int8_t sl = (int8_t)(v_l * VEL_TO_SPEED_SCALE);
    int8_t sr = (int8_t)(v_r * VEL_TO_SPEED_SCALE);

    if (sl >  MOTOR_SPEED_MAX) sl =  MOTOR_SPEED_MAX;
    if (sl <  MOTOR_SPEED_MIN) sl =  MOTOR_SPEED_MIN;
    if (sr >  MOTOR_SPEED_MAX) sr =  MOTOR_SPEED_MAX;
    if (sr <  MOTOR_SPEED_MIN) sr =  MOTOR_SPEED_MIN;

    speed_left       = sl;
    speed_right      = sr;
    target_steer_rad = steer;

    servo_set_angle(steer);
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
        hiwonder_stop_all();
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
