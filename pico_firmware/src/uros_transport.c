#ifdef USE_MICRO_ROS

#include "uros_transport.h"
#include "config.h"
#include "motor_control.h"
#include "safety.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/set_bool.h>

#include <math.h>
#include <string.h>

/* ── Static allocations ─────────────────────────────────────── */

static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;

/* Subscribers */
static rcl_subscription_t control_cmd_sub;
static rcl_subscription_t enable_sub;
static geometry_msgs__msg__TwistStamped control_cmd_msg;
static std_msgs__msg__Bool              enable_msg;

/* Publishers */
static rcl_publisher_t odom_pub;
static rcl_publisher_t joint_pub;
static nav_msgs__msg__Odometry            odom_msg;
static sensor_msgs__msg__JointState       joint_msg;

/* Service */
static rcl_service_t   estop_srv;
static std_srvs__srv__SetBool_Request  estop_req;
static std_srvs__srv__SetBool_Response estop_res;

/* JointState static string buffers */
#define JOINT_NAME_MAX_LEN 24
#define JOINT_COUNT        3

static char joint_name_buf[JOINT_COUNT][JOINT_NAME_MAX_LEN];
static rosidl_runtime_c__String joint_names[JOINT_COUNT];
static double joint_positions[JOINT_COUNT];
static double joint_velocities[JOINT_COUNT];

/* ── Helpers ────────────────────────────────────────────────── */

#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) return false; }

/* ── Callbacks ──────────────────────────────────────────────── */

static void control_cmd_callback(const void *msgin)
{
    const geometry_msgs__msg__TwistStamped *msg =
        (const geometry_msgs__msg__TwistStamped *)msgin;

    motor_control_set_velocity(
        (float)msg->twist.linear.x,
        (float)msg->twist.angular.z
    );
    safety_feed_watchdog();
}

static void enable_callback(const void *msgin)
{
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    motor_control_enable(msg->data);
}

static void estop_service_callback(const void *reqin, void *resin)
{
    const std_srvs__srv__SetBool_Request *req =
        (const std_srvs__srv__SetBool_Request *)reqin;
    std_srvs__srv__SetBool_Response *res =
        (std_srvs__srv__SetBool_Response *)resin;

    if (req->data) {
        safety_estop_activate();
        res->success = true;
        /* res->message is left empty — static allocation, no dynamic string */
    } else {
        safety_estop_clear();
        res->success = true;
    }
}

/* ── Init ───────────────────────────────────────────────────── */

static void init_joint_msg(void)
{
    /* Set up joint names */
    strncpy(joint_name_buf[0], "left_wheel_joint",  JOINT_NAME_MAX_LEN);
    strncpy(joint_name_buf[1], "right_wheel_joint", JOINT_NAME_MAX_LEN);
    strncpy(joint_name_buf[2], "steering_joint",    JOINT_NAME_MAX_LEN);

    for (int i = 0; i < JOINT_COUNT; i++) {
        joint_names[i].data     = joint_name_buf[i];
        joint_names[i].size     = strlen(joint_name_buf[i]);
        joint_names[i].capacity = JOINT_NAME_MAX_LEN;
    }

    joint_msg.name.data     = joint_names;
    joint_msg.name.size     = JOINT_COUNT;
    joint_msg.name.capacity = JOINT_COUNT;

    joint_msg.position.data     = joint_positions;
    joint_msg.position.size     = JOINT_COUNT;
    joint_msg.position.capacity = JOINT_COUNT;

    joint_msg.velocity.data     = joint_velocities;
    joint_msg.velocity.size     = JOINT_COUNT;
    joint_msg.velocity.capacity = JOINT_COUNT;

    /* effort not used */
    joint_msg.effort.data     = NULL;
    joint_msg.effort.size     = 0;
    joint_msg.effort.capacity = 0;
}

bool uros_init(void)
{
    allocator = rcl_get_default_allocator();

    /* Wait for agent connection */
    RCCHECK(rmw_uros_ping_agent(1000, 10));

    /* Create support and node */
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, UROS_NODE_NAME, "", &support));

    /* --- Subscribers --- */

    RCCHECK(rclc_subscription_init_default(
        &control_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        "/pico/control_cmd"));

    RCCHECK(rclc_subscription_init_default(
        &enable_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/pico/enable"));

    /* --- Publishers --- */

    RCCHECK(rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/pico/odom"));

    RCCHECK(rclc_publisher_init_default(
        &joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/pico/joint_feedback"));

    /* --- Service --- */

    RCCHECK(rclc_service_init_default(
        &estop_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
        "/pico/estop"));

    /* --- Executor (2 subs + 1 service = 3 handles) --- */

    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &control_cmd_sub, &control_cmd_msg,
        &control_cmd_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(
        &executor, &enable_sub, &enable_msg,
        &enable_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_service(
        &executor, &estop_srv, &estop_req, &estop_res,
        &estop_service_callback));

    /* Init static JointState message */
    init_joint_msg();

    /* Init odom message frames */
    /* Using static string approach — frame_id fields are set via
       the rosidl_runtime_c__String struct in the message */
    static char odom_frame[] = "odom";
    static char base_frame[] = "base_link";
    odom_msg.header.frame_id.data     = odom_frame;
    odom_msg.header.frame_id.size     = strlen(odom_frame);
    odom_msg.header.frame_id.capacity = sizeof(odom_frame);
    odom_msg.child_frame_id.data      = base_frame;
    odom_msg.child_frame_id.size      = strlen(base_frame);
    odom_msg.child_frame_id.capacity  = sizeof(base_frame);

    /* JointState frame */
    static char js_frame[] = "base_link";
    joint_msg.header.frame_id.data     = js_frame;
    joint_msg.header.frame_id.size     = strlen(js_frame);
    joint_msg.header.frame_id.capacity = sizeof(js_frame);

    return true;
}

/* ── Spin ───────────────────────────────────────────────────── */

void uros_spin_some(void)
{
    /* Zero timeout = non-blocking */
    rclc_executor_spin_some(&executor, 0);
}

/* ── Publishers ─────────────────────────────────────────────── */

void uros_publish_odom(const odom_state_t *odom)
{
    /* Timestamp */
    int64_t now_ns = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    /* Pose */
    odom_msg.pose.pose.position.x = (double)odom->x;
    odom_msg.pose.pose.position.y = (double)odom->y;
    odom_msg.pose.pose.position.z = 0.0;

    /* Quaternion from yaw */
    double half_yaw = (double)odom->yaw * 0.5;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(half_yaw);
    odom_msg.pose.pose.orientation.w = cos(half_yaw);

    /* Twist */
    odom_msg.twist.twist.linear.x  = (double)odom->vx;
    odom_msg.twist.twist.angular.z = (double)odom->wz;

    rcl_publish(&odom_pub, &odom_msg, NULL);
}

void uros_publish_joint_state(float vl_rads, float vr_rads, float steer_rad)
{
    /* Timestamp */
    int64_t now_ns = rmw_uros_epoch_nanos();
    joint_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    joint_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    joint_positions[0]  = 0.0;                  /* left wheel pos (unused)  */
    joint_positions[1]  = 0.0;                  /* right wheel pos (unused) */
    joint_positions[2]  = (double)steer_rad;    /* steering angle           */

    joint_velocities[0] = (double)vl_rads;      /* left wheel rad/s         */
    joint_velocities[1] = (double)vr_rads;      /* right wheel rad/s        */
    joint_velocities[2] = 0.0;                  /* steering velocity        */

    rcl_publish(&joint_pub, &joint_msg, NULL);
}

/* ── Agent connectivity check ───────────────────────────────── */

bool uros_is_agent_connected(void)
{
    return rmw_uros_ping_agent(100, 1) == RCL_RET_OK;
}

#endif /* USE_MICRO_ROS */
