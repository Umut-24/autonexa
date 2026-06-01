#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

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
 * Feed measured per-tick encoder deltas so motor_control_apply() knows
 * whether each wheel is actually rolling. Drives the encoder-aware
 * kick-start (break static friction from rest, then honor proportional
 * duty). Call once per control loop iteration BEFORE motor_control_apply().
 * @param d_enc_left   left-wheel encoder edges since last tick (signed)
 * @param d_enc_right  right-wheel encoder edges since last tick (signed)
 */
void motor_control_update_feedback(int32_t d_enc_left, int32_t d_enc_right);

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
