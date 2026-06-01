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

/* ── Closed-loop velocity-PI state ──────────────────────────── */
/* Measured per-wheel linear speed (m/s, signed), low-pass filtered. Updated each
 * tick by motor_control_update_feedback() from the encoder-derived velocities. */
static float    v_meas_left   = 0.0f;
static float    v_meas_right  = 0.0f;
/* PI integrator per wheel (carries duty %; holds the steady-state duty a load
 * needs so err -> 0 at the target speed). */
static float    integ_left    = 0.0f;
static float    integ_right   = 0.0f;
/* started_* latches the in-progress move so the one-shot start feedforward (kick)
 * fires once per move from rest; cleared on SPEED 0. */
static bool     started_left  = false;
static bool     started_right = false;
static uint32_t start_since_left_ms  = 0;
static uint32_t start_since_right_ms = 0;
/* Absolute-time (ms) the current kick feedforward pulse ends. */
static uint32_t kick_until_left_ms  = 0;
static uint32_t kick_until_right_ms = 0;
/* Absolute-time (ms) the current stall began (commanded + past kick + measured
 * not-moving). 0 = not stalling. Past MOTOR_STALL_CUTOFF_MS the duty is cut to 0
 * to protect the motor + current-limitless L298N from stall heat. */
static uint32_t stall_since_left_ms  = 0;
static uint32_t stall_since_right_ms = 0;
/* Live-tunable PI gains + deadband feedforward (init from config.h, overridable
 * at runtime via motor_control_set_pi() / the CLI "PI" verb so the loop can be
 * tuned on the bench without reflashing; bake the winners back into config.h). */
static float    vel_kp        = MOTOR_VEL_KP;
static float    vel_ki        = MOTOR_VEL_KI;
static float    vel_ff_offset = MOTOR_FF_OFFSET_PCT;
static motor_debug_t latest_debug = {0};

/* ── Velocity-to-speed scale factor ─────────────────────────── */
/* Keep VEL/micro-ROS semantics aligned with the ASCII bridge:
 *   speed_cli = round(vx_mps * 30 / MOTOR_V_MAX_MPS)
 *   target    = speed_cli / 30 * MOTOR_V_MAX_MPS
 */
#define VEL_TO_SPEED_SCALE  ((float)MOTOR_SPEED_MAX / MOTOR_V_MAX_MPS)

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

void motor_control_update_feedback(float v_left_mps, float v_right_mps)
{
    const float a = MOTOR_VEL_LPF_ALPHA;
    v_meas_left  = a * v_left_mps  + (1.0f - a) * v_meas_left;
    v_meas_right = a * v_right_mps + (1.0f - a) * v_meas_right;
}

/* Closed-loop velocity PI for one wheel. Returns signed duty % in [-100, 100].
 *
 *   speed  : commanded SPEED unit (-MOTOR_SPEED_MAX..+MOTOR_SPEED_MAX), mapped to
 *            a TARGET wheel speed: target = speed/MOTOR_SPEED_MAX * MOTOR_V_MAX_MPS.
 *   v_meas : measured (filtered) wheel linear speed [m/s], signed.
 *   integ  : PI integrator (duty %), persists across ticks.
 *   started/kick_until_ms : one-shot start feedforward (kick) state.
 *   stall_since_ms        : stall-cutoff timer.
 *
 * SPEED 0 coasts and resets the loop. From rest a kick pulse breaks stiction;
 * then a PI regulates duty to hit the target speed (integral carries the
 * load-dependent steady-state duty, with conditional anti-windup). If commanded
 * but the encoder reports not-moving past the kick for > MOTOR_STALL_CUTOFF_MS
 * (PI saturated), the duty is cut to 0 to protect the motor + L298N from stall
 * heat; it re-arms on the next SPEED 0. */
static int velocity_pi(int8_t speed, float v_meas, float *integ, bool *started,
                       uint32_t *start_since_ms, uint32_t *kick_until_ms,
                       uint32_t *stall_since_ms, float *target_out,
                       bool *stall_out, bool *cutoff_out)
{
    *target_out = 0.0f;
    *stall_out = false;
    *cutoff_out = false;

    if (speed == 0) {
        *integ          = 0.0f;
        *started        = false;
        *start_since_ms = 0;
        *kick_until_ms  = 0;
        *stall_since_ms = 0;
        return 0;   /* coast */
    }

    float target = ((float)speed / (float)MOTOR_SPEED_MAX) * MOTOR_V_MAX_MPS; /* signed m/s */
    int   tsign  = (target < 0.0f) ? -1 : 1;
    float err    = target - v_meas;
    float moving_toward_mps = (float)tsign * v_meas;
    bool  moving_toward = moving_toward_mps > MOTOR_VEL_MOVING_MPS;
    float target_abs = fabsf(target);
    *target_out = target;

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (!*started) {
        *started        = true;
        *start_since_ms = now;
        *kick_until_ms  = now + MOTOR_KICK_MS;
        *integ          = 0.0f;
        *stall_since_ms = now;
    }

    /* Start feedforward: full kick toward the target until the wheel is confirmed
     * moving or the pulse elapses. Integral stays frozen during the kick. */
    if (now < *kick_until_ms && !moving_toward) {
        return tsign * MOTOR_KICK_PCT;
    }

    /* Stall cutoff: commanded, past the kick, still not moving -> protect HW. */
    if (!moving_toward) {
        if (*stall_since_ms == 0) *stall_since_ms = now;
        *stall_out = true;
        if (now - *stall_since_ms > MOTOR_STALL_CUTOFF_MS) {
            *integ = 0.0f;
            *cutoff_out = true;
            return 0;   /* coast until SPEED 0 re-arms */
        }
    } else {
        *stall_since_ms = 0;
    }

    /* PI + deadband feedforward. dt = 1 / control-loop rate. The FF offset puts
     * the duty near the sustain so the integral only trims the residual. */
    const float dt = 1.0f / (float)CONTROL_FREQ_HZ;
    float ff     = (float)tsign * vel_ff_offset;
    float p      = vel_kp * err;
    float duty_f = ff + p + *integ;

    /* Conditional anti-windup: only integrate when not pushing further into a
     * saturated rail in the same direction as the error. */
    bool sat_hi = (duty_f >=  100.0f);
    bool sat_lo = (duty_f <= -100.0f);
    if (!((sat_hi && err > 0.0f) || (sat_lo && err < 0.0f))) {
        *integ += vel_ki * err * dt;
        if (*integ >  100.0f) *integ =  100.0f;
        if (*integ < -100.0f) *integ = -100.0f;
    }

    duty_f = ff + p + *integ;
    if (duty_f >  100.0f) duty_f =  100.0f;
    if (duty_f < -100.0f) duty_f = -100.0f;

    bool in_start_assist = (now - *start_since_ms) < MOTOR_START_ASSIST_MS;
    bool still_ramping = moving_toward_mps < (0.60f * target_abs);
    if (in_start_assist && still_ramping) {
        float min_start_duty = (float)tsign * (float)MOTOR_START_MIN_DUTY_PCT;
        if (tsign > 0 && duty_f < min_start_duty) duty_f = min_start_duty;
        if (tsign < 0 && duty_f > min_start_duty) duty_f = min_start_duty;
    }

    return (int)duty_f;
}

void motor_control_set_pi(float kp, float ki, float ff_offset)
{
    vel_kp        = kp;
    vel_ki        = ki;
    vel_ff_offset = ff_offset;
    integ_left    = 0.0f;   /* reset integrators so a retune starts clean */
    integ_right   = 0.0f;
}

void motor_control_get_pi(float *kp, float *ki, float *ff_offset)
{
    if (kp)        *kp        = vel_kp;
    if (ki)        *ki        = vel_ki;
    if (ff_offset) *ff_offset = vel_ff_offset;
}

void motor_control_get_debug(motor_debug_t *debug)
{
    if (debug) *debug = latest_debug;
}

void motor_control_apply(void)
{
    if (motors_enabled && safety_is_ok()) {
        bool stall_l, stall_r, cutoff_l, cutoff_r;
        float target_l, target_r;
        int dl = velocity_pi(speed_left,  v_meas_left,  &integ_left,
                             &started_left,  &start_since_left_ms,
                             &kick_until_left_ms, &stall_since_left_ms,
                             &target_l, &stall_l, &cutoff_l);
        int dr = velocity_pi(speed_right, v_meas_right, &integ_right,
                             &started_right, &start_since_right_ms,
                             &kick_until_right_ms, &stall_since_right_ms,
                             &target_r, &stall_r, &cutoff_r);
        latest_debug.target_left_mps = target_l;
        latest_debug.target_right_mps = target_r;
        latest_debug.measured_left_mps = v_meas_left;
        latest_debug.measured_right_mps = v_meas_right;
        latest_debug.duty_left_pct = (int8_t)dl;
        latest_debug.duty_right_pct = (int8_t)dr;
        latest_debug.started_left = started_left;
        latest_debug.started_right = started_right;
        latest_debug.stall_left = stall_l;
        latest_debug.stall_right = stall_r;
        latest_debug.cutoff_left = cutoff_l;
        latest_debug.cutoff_right = cutoff_r;
        /* l298n_set_raw_pwm applies literal duty (m1 = LEFT, m2 = RIGHT),
         * bypassing the driver's old deadband floor. */
        l298n_set_raw_pwm((int8_t)dl, (int8_t)dr);
    } else {
        latest_debug.duty_left_pct = 0;
        latest_debug.duty_right_pct = 0;
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
