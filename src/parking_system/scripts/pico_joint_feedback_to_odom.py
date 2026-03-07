#!/usr/bin/env python3
"""
RPi5 Team-B odometry integration node:
- Input: sensor_msgs/JointState from Pico (/pico/joint_feedback)
  Expected joint names:
    * left_wheel_joint
    * right_wheel_joint
    * steering_joint
  and velocities in rad/s for wheel joints, position in rad for steering joint.
- Output: nav_msgs/Odometry (/pico/odom)

Kinematic model:
- Rear-wheel average speed -> body longitudinal speed vx
- Ackermann yaw rate wz = vx * tan(delta) / wheelbase
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class PicoJointFeedbackToOdom(Node):
    def __init__(self) -> None:
        super().__init__('pico_joint_feedback_to_odom')

        self.declare_parameter('joint_feedback_topic', '/pico/joint_feedback')
        self.declare_parameter('odom_topic', '/pico/odom')

        self.declare_parameter('left_wheel_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint_name', 'right_wheel_joint')
        self.declare_parameter('steering_joint_name', 'steering_joint')

        self.declare_parameter('wheel_radius_m', 0.0325)
        self.declare_parameter('wheelbase_m', 0.20)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.feedback_topic = self.get_parameter('joint_feedback_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.left_name = self.get_parameter('left_wheel_joint_name').value
        self.right_name = self.get_parameter('right_wheel_joint_name').value
        self.steer_name = self.get_parameter('steering_joint_name').value

        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None

        self.sub = self.create_subscription(JointState, self.feedback_topic, self.on_feedback, 50)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 50)

        self.get_logger().info(
            f'Odom node started: {self.feedback_topic} -> {self.odom_topic} '
            f'(wheel_radius={self.wheel_radius:.4f}, wheelbase={self.wheelbase:.4f})'
        )

    def _index(self, names, key: str) -> int:
        for i, n in enumerate(names):
            if n == key:
                return i
        return -1

    def on_feedback(self, msg: JointState) -> None:
        il = self._index(msg.name, self.left_name)
        ir = self._index(msg.name, self.right_name)
        isr = self._index(msg.name, self.steer_name)

        if il < 0 or ir < 0 or isr < 0:
            self.get_logger().warn(
                'JointState missing required names. '
                f'Needed: {self.left_name}, {self.right_name}, {self.steer_name}',
                throttle_duration_sec=5.0,
            )
            return

        if il >= len(msg.velocity) or ir >= len(msg.velocity) or isr >= len(msg.position):
            self.get_logger().warn('JointState does not contain required velocity/position arrays.', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        wl = msg.velocity[il]  # rad/s
        wr = msg.velocity[ir]  # rad/s
        steer = msg.position[isr]  # rad

        vl = wl * self.wheel_radius
        vr = wr * self.wheel_radius
        vx = 0.5 * (vl + vr)
        wz = vx * math.tan(steer) / self.wheelbase if abs(self.wheelbase) > 1e-6 else 0.0

        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw += wz * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PicoJointFeedbackToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
