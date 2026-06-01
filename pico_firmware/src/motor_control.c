#include "motor_control.h"
#include "config.h"
#include "ackermann.h"
#include "servo.h"
#include "l298n_driver.h"
#include "safety.h"

#include "pico/stdlib.h"   /* to_ms_since_boot / get_absolute_time */
#include <math.h>

/* ── State ──────────────────────────────────────────────────── */
static float   target_steer_rad = 0.0f;
static int8_t  speed_left       = 0;
static int8_t  speed_right      = 0;
static bool    motors_enabled   = false;

/* ── Encoder-aware kick-start state ─────────────────────────── */
/* Updated each tick by motor_control_update_feedback() from the measured
 * encoder deltas; read by motor_control_apply()'s kick logic. */
static bool     wheel_moving_left   = false;
static bool     wheel_moving_right  = false;
/* Absolute-time (ms) at which the current kick pulse ends. 0 = no kick
 * armed (re-armed the next time the wheel is found stopped with a command). */
static uint32_t kick_until_left_ms  = 0;
static uint32_t kick_until_right_ms = 0;

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
    l298n_stop_all();
    servo_center();
}

bool motor_control_is_enabled(void)
{
    return motors_enabled;
}

void motor_control_update_feedback(int32_t d_enc_left, int32_t d_enc_right)
{
    int32_t al = (d_enc_left  < 0) ? -d_enc_left  : d_enc_left;
    int32_t ar = (d_enc_right < 0) ? -d_enc_right : d_enc_right;
    wheel_moving_left  = (al > MOTOR_ENC_MOVING_EDGES);
    wheel_moving_right = (ar > MOTOR_ENC_MOVING_EDGES);
}

/* Encoder-aware kick-start for one channel. Converts the commanded speed to a
 * signed duty %, then:
 *   - SPEED 0       -> 0 (coast/stop), disarm the kick so the next start kicks.
 *   - wheel rolling -> proportional duty, floored to MOTOR_MIN_RUN_PCT; disarm
 *                      the kick.
 *   - wheel stopped -> fire ONE kick pulse (MOTOR_KICK_PCT for MOTOR_KICK_MS,
 *                      used only as a floor so a higher command isn't reduced)
 *                      to break static friction. After the pulse, if the wheel
 *                      still hasn't rolled, BACK OFF to the commanded
 *                      proportional duty (floored to MOTOR_MIN_RUN_PCT) — never
 *                      a sustained high duty.
 *
 * L298N safety: a blocked wheel must not sit at high duty (the L298N has no
 * current limit and overheats). After the single pulse the duty drops to the
 * low commanded value — strictly cooler than the firmware's old permanent 60%
 * floor. A genuine block is then handled upstream (collision_monitor zeroes the
 * command; the progress checker triggers Nav2 recovery). The kick re-arms only
 * once the wheel actually rolls again (or the command returns to 0). */
static int kick_duty(int8_t speed, bool wheel_moving, uint32_t *kick_until_ms)
{
    if (speed == 0) {
        *kick_until_ms = 0;
        return 0;
    }

    int target = ((int)speed * 100) / MOTOR_SPEED_MAX;   /* signed duty % */
    int sign   = (target < 0) ? -1 : 1;
    int mag    = (target < 0) ? -target : target;
    if (mag > 100) mag = 100;

    if (wheel_moving) {
        /* Rolling: proportional duty with a sustain floor; disarm the kick. */
        *kick_until_ms = 0;
        if (mag < MOTOR_MIN_RUN_PCT) mag = MOTOR_MIN_RUN_PCT;
        return sign * mag;
    }

    /* Stopped with a command: fire a single kick pulse to break stiction. */
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (*kick_until_ms == 0) {
        *kick_until_ms = now + MOTOR_KICK_MS;   /* arm one pulse */
    }
    if (now < *kick_until_ms) {
        /* During the pulse: kick floor — never reduce an already-higher cmd. */
        return sign * (mag > MOTOR_KICK_PCT ? mag : MOTOR_KICK_PCT);
    }
    /* Pulse over and still not rolling (likely blocked): back off to the low
     * commanded duty. Do NOT re-arm here — the kick re-arms via the rolling
     * branch (wheel started) or the SPEED 0 branch (command cleared). */
    if (mag < MOTOR_MIN_RUN_PCT) mag = MOTOR_MIN_RUN_PCT;
    return sign * mag;
}

void motor_control_apply(void)
{
    if (motors_enabled && safety_is_ok()) {
        int dl = kick_duty(speed_left,  wheel_moving_left,  &kick_until_left_ms);
        int dr = kick_duty(speed_right, wheel_moving_right, &kick_until_right_ms);
        /* l298n_set_raw_pwm applies literal duty (m1 = LEFT, m2 = RIGHT),
         * bypassing the driver's old deadband floor. */
        l298n_set_raw_pwm((int8_t)dl, (int8_t)dr);
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
