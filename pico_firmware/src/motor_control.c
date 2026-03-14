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
/* max speed ≈ 0.3 m/s → scale to 100 */
#define VEL_TO_SPEED_SCALE  (100.0f / 0.3f)

/* ── API ────────────────────────────────────────────────────── */

void motor_control_set_velocity(float vx, float wz)
{
    float steer, v_l, v_r;
    ackermann_inverse(vx, wz, &steer, &v_l, &v_r);

    /* Convert m/s to driver board speed units */
    int8_t sl = (int8_t)(v_l * VEL_TO_SPEED_SCALE);
    int8_t sr = (int8_t)(v_r * VEL_TO_SPEED_SCALE);

    if (sl >  100) sl =  100;
    if (sl < -100) sl = -100;
    if (sr >  100) sr =  100;
    if (sr < -100) sr = -100;

    speed_left       = sl;
    speed_right      = sr;
    target_steer_rad = steer;

    servo_set_angle(steer);
}

void motor_control_enable(bool enable)
{
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
    hiwonder_stop_all();
    servo_center();
}

bool motor_control_is_enabled(void)
{
    return motors_enabled;
}

void motor_control_apply(void)
{
    if (motors_enabled && safety_is_ok()) {
        hiwonder_set_speeds(speed_left, speed_right);
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
