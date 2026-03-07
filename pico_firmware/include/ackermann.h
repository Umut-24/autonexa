#ifndef ACKERMANN_H
#define ACKERMANN_H

/**
 * Ackermann kinematics (Phase 2).
 *
 * Inverse: (vx, wz) → (steering_angle, V_left, V_right)
 * Forward: encoder ticks → (dx, dy, dyaw) integrated odometry
 */

/** Accumulated odometry state. */
typedef struct {
    float x;      /* [m]     */
    float y;      /* [m]     */
    float yaw;    /* [rad]   */
    float vx;     /* [m/s]   */
    float wz;     /* [rad/s] */
} odom_state_t;

/**
 * Inverse kinematics: body velocity → actuator setpoints.
 * @param vx              Longitudinal speed [m/s].
 * @param wz              Yaw rate [rad/s].
 * @param[out] steer_rad  Required steering angle [rad].
 * @param[out] v_left     Left wheel linear speed [m/s].
 * @param[out] v_right    Right wheel linear speed [m/s].
 */
void ackermann_inverse(float vx, float wz,
                       float *steer_rad,
                       float *v_left, float *v_right);

/**
 * Forward kinematics: integrate encoder-measured wheel speeds into odometry.
 * @param v_left   Left wheel speed [m/s].
 * @param v_right  Right wheel speed [m/s].
 * @param steer    Steering angle [rad].
 * @param dt       Time step [s].
 * @param odom     Odometry state to update in-place.
 */
void ackermann_forward(float v_left, float v_right, float steer,
                       float dt, odom_state_t *odom);

/** Reset odometry to origin. */
void ackermann_odom_reset(odom_state_t *odom);

#endif /* ACKERMANN_H */
