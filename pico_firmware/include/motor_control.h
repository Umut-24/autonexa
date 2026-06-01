#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float target_left_mps;
    float target_right_mps;
    float measured_left_mps;
    float measured_right_mps;
    int8_t duty_left_pct;
    int8_t duty_right_pct;
    bool started_left;
    bool started_right;
    bool stall_left;
    bool stall_right;
    bool cutoff_left;
    bool cutoff_right;
} motor_debug_t;

/**
 * Motor control abstraction layer.
 *
 * Provides a unified API for setting velocity commands via Ackermann
 * inverse kinematics. Used by both the ASCII serial CLI and micro-ROS
 * transport layers.
 */

/**
 * Set target body velocity. Computes Ackermann inverse kinematics
 * internally and updates motor speeds and steering angle.
 * @param vx  Longitudinal speed [m/s]
 * @param wz  Yaw rate [rad/s]
 */
void motor_control_set_velocity(float vx, float wz);

/**
 * Directly set the persistent motor speed targets that
 * motor_control_apply() writes to hardware every control loop. Used by
 * the CLI SPEED / SPEED_L / SPEED_R verbs which command motors but do
 * not change steering. Bypasses Ackermann IK.
 *
 * Both arguments are clamped to [MOTOR_SPEED_MIN, MOTOR_SPEED_MAX].
 */
void motor_control_set_speeds(int8_t left, int8_t right);

/** Set only the left motor's persistent speed target. */
void motor_control_set_speed_left(int8_t left);

/** Set only the right motor's persistent speed target. */
void motor_control_set_speed_right(int8_t right);

/**
 * Enable or disable motor output.
 * When disabled, motors are stopped and speeds zeroed.
 * @param enable  true to enable, false to disable
 */
void motor_control_enable(bool enable);

/** Stop all motors and center steering. Keeps enabled state unchanged. */
void motor_control_stop(void);

/** Returns true if motors are currently enabled. */
bool motor_control_is_enabled(void);

/**
 * Feed the measured per-wheel linear speed (m/s, signed) from the encoders so
 * the closed-loop velocity PI in motor_control_apply() can regulate each wheel
 * to its target. Internally low-pass filtered (MOTOR_VEL_LPF_ALPHA). Call once
 * per control loop iteration BEFORE motor_control_apply().
 * @param v_left_mps   left-wheel measured linear speed [m/s] (signed)
 * @param v_right_mps  right-wheel measured linear speed [m/s] (signed)
 */
void motor_control_update_feedback(float v_left_mps, float v_right_mps);

/**
 * Live-set the closed-loop velocity-PI gains + deadband feedforward (bench
 * tuning without reflashing; resets the integrators). Bake winners into config.h.
 * @param kp         P gain (duty% per m/s error)
 * @param ki         I gain (duty% per m/s*s error)
 * @param ff_offset  static deadband feedforward duty % (sign of target)
 */
void motor_control_set_pi(float kp, float ki, float ff_offset);

/** Read the current velocity-PI gains (any pointer may be NULL). */
void motor_control_get_pi(float *kp, float *ki, float *ff_offset);

/** Read latest closed-loop debug state for serial telemetry. */
void motor_control_get_debug(motor_debug_t *debug);

/**
 * Apply current motor commands to hardware.
 * Call once per control loop iteration. Only actuates if enabled and
 * safety is OK; otherwise stops motors.
 */
void motor_control_apply(void);

/** Get current left motor speed command [-100..100]. */
int8_t motor_control_get_speed_left(void);

/** Get current right motor speed command [-100..100]. */
int8_t motor_control_get_speed_right(void);

/** Get current target steering angle [rad]. */
float motor_control_get_steer_rad(void);

#endif /* MOTOR_CONTROL_H */
