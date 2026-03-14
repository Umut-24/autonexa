#ifndef UROS_TRANSPORT_H
#define UROS_TRANSPORT_H

#ifdef USE_MICRO_ROS

#include <stdbool.h>
#include "ackermann.h"

/**
 * micro-ROS transport layer for Pico.
 *
 * Creates a ROS2 node ("pico_controller") with:
 *   - Subscriber: geometry_msgs/TwistStamped on /pico/control_cmd
 *   - Subscriber: std_msgs/Bool on /pico/enable
 *   - Publisher:  nav_msgs/Odometry on /pico/odom
 *   - Publisher:  sensor_msgs/JointState on /pico/joint_feedback
 *   - Service:    std_srvs/SetBool on /pico/estop
 *
 * All message buffers are statically allocated (no malloc).
 */

/**
 * Initialize micro-ROS node, subscribers, publishers, service, and executor.
 * @return true if initialization succeeded, false otherwise.
 */
bool uros_init(void);

/**
 * Non-blocking executor spin. Processes any pending callbacks.
 * Call once per control loop iteration.
 */
void uros_spin_some(void);

/**
 * Publish odometry message.
 * @param odom  Current odometry state from Ackermann forward kinematics.
 */
void uros_publish_odom(const odom_state_t *odom);

/**
 * Publish joint state feedback (diagnostic).
 * @param vl_rads   Left wheel angular velocity [rad/s]
 * @param vr_rads   Right wheel angular velocity [rad/s]
 * @param steer_rad Current steering angle [rad]
 */
void uros_publish_joint_state(float vl_rads, float vr_rads, float steer_rad);

/**
 * Check if the micro-ROS agent is reachable.
 * @return true if agent responds to ping.
 */
bool uros_is_agent_connected(void);

#endif /* USE_MICRO_ROS */
#endif /* UROS_TRANSPORT_H */
