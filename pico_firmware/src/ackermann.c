#include "ackermann.h"
#include "config.h"

#include <math.h>
#include <string.h>

/* ── Inverse Kinematics ──────────────────────────────────────── */

void ackermann_inverse(float vx, float wz,
                       float *steer_rad,
                       float *v_left, float *v_right)
{
    /*
     * For an Ackermann vehicle:
     *   steering_angle = atan(L * wz / vx)
     *
     * When vx ≈ 0, the car is stationary or pivoting.
     * We handle this by clamping the steering angle.
     */
    float steer = 0.0f;

    if (fabsf(vx) > 0.001f) {
        steer = atanf(WHEELBASE_M * wz / vx);
    } else if (fabsf(wz) > 0.001f) {
        /* Pure rotation request but vx ≈ 0: steer to max */
        steer = (wz > 0.0f) ? MAX_STEERING_RAD : -MAX_STEERING_RAD;
    }

    /* Clamp steering */
    if (steer >  MAX_STEERING_RAD) steer =  MAX_STEERING_RAD;
    if (steer < -MAX_STEERING_RAD) steer = -MAX_STEERING_RAD;

    *steer_rad = steer;

    /*
     * Differential wheel speeds for inner / outer wheels:
     *   Turning radius R = L / tan(steer)      (when steer ≠ 0)
     *   V_left  = vx * (R - W/2) / R
     *   V_right = vx * (R + W/2) / R
     *
     * When steer ≈ 0 (straight), V_left = V_right = vx.
     */
    if (fabsf(steer) > 0.001f) {
        float R = WHEELBASE_M / tanf(steer);
        float half_track = TRACK_WIDTH_M / 2.0f;
        *v_left  = vx * (R - half_track) / R;
        *v_right = vx * (R + half_track) / R;
    } else {
        *v_left  = vx;
        *v_right = vx;
    }
}

/* ── Forward Kinematics / Odometry Integration ───────────────── */

void ackermann_forward(float v_left, float v_right, float steer,
                       float dt, odom_state_t *odom)
{
    /*
     * NOTE: 'steer' is the commanded servo angle from servo_get_angle(),
     * not a measured angle (no servo feedback sensor). This is the only
     * option without feedback hardware, but is a known source of odometry
     * drift at large steering angles where servo nonlinearity is worst.
     *
     * Average velocity and yaw rate from bicycle model:
     *   v_avg  = (v_left + v_right) / 2
     *   wz     = v_avg * tan(steer) / L
     *
     * Then integrate:
     *   yaw    += wz * dt
     *   x      += v_avg * cos(yaw) * dt
     *   y      += v_avg * sin(yaw) * dt
     */
    float v_avg = (v_left + v_right) / 2.0f;
    float wz    = 0.0f;

    if (fabsf(steer) > 0.001f) {
        wz = v_avg * tanf(steer) / WHEELBASE_M;
    }

    odom->yaw += wz * dt;

    /* Normalise yaw to [-π, π] */
    while (odom->yaw >  (float)M_PI) odom->yaw -= 2.0f * (float)M_PI;
    while (odom->yaw < -(float)M_PI) odom->yaw += 2.0f * (float)M_PI;

    odom->x += v_avg * cosf(odom->yaw) * dt;
    odom->y += v_avg * sinf(odom->yaw) * dt;

    odom->vx = v_avg;
    odom->wz = wz;
}

void ackermann_odom_reset(odom_state_t *odom)
{
    memset(odom, 0, sizeof(odom_state_t));
}
