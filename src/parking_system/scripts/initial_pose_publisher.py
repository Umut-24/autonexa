#!/usr/bin/env python3
"""One-shot AMCL initial-pose seeder.

AMCL's launch-time `set_initial_pose` parameter is applied only once, at
activation, and is frequently missed — leaving AMCL at its (0,0,0) default.
On a small saved map whose world bounds do not include the origin (e.g. the
2x2 m testbed: X span [-2.389, -0.309]), that renders the robot visibly OFF
the map, and there is no monitor / second terminal in the field to nudge it.

The critical detail: AMCL **ignores /initialpose until it is in the ACTIVE
lifecycle state** ("Received initial pose request, but AMCL is not yet in the
active state"). Publishing during configuring is silently dropped. So this
node polls AMCL's lifecycle state via /amcl/get_state and only publishes the
pose once AMCL reports ACTIVE — then republishes a couple more times for
good measure and exits. Fully launch-driven: no operator action, no second
terminal, and it tolerates an arbitrarily slow AMCL activation.

Parameters:
    x, y, yaw         target pose in the map frame (yaw in radians)
    frame_id          pose frame (default 'map')
    topic             output topic (default '/initialpose')
    amcl_node         AMCL node name for the get_state service (default 'amcl')
    publish_count     how many times to (re)publish once ACTIVE (default 3)
    publish_period_s  spacing between republishes (default 1.0)
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/initialpose')
        self.declare_parameter('amcl_node', 'amcl')
        self.declare_parameter('publish_count', 3)
        self.declare_parameter('publish_period_s', 1.0)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('topic').value)
        amcl_node = str(self.get_parameter('amcl_node').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.publish_period = float(self.get_parameter('publish_period_s').value)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.topic, 10)
        self._state_cli = self.create_client(
            GetState, f'/{amcl_node}/get_state')
        self._active = False
        self._pending = None
        self._sent = 0
        self._pub_timer = None
        # Poll AMCL's lifecycle state until ACTIVE, then publish.
        self._poll_timer = self.create_timer(1.0, self._poll_state)
        self.get_logger().info(
            f'Waiting for AMCL ({amcl_node}) to reach ACTIVE before seeding '
            f'{self.topic} = (x={self.x:.3f}, y={self.y:.3f}, '
            f'yaw={self.yaw:.3f}).')

    def _poll_state(self) -> None:
        """Async-poll /<amcl>/get_state; trip self._active when ACTIVE."""
        if self._active:
            return
        if not self._state_cli.service_is_ready():
            return
        if self._pending is not None and not self._pending.done():
            return
        self._pending = self._state_cli.call_async(GetState.Request())
        self._pending.add_done_callback(self._on_state)

    def _on_state(self, future) -> None:
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001 - log + retry next tick
            self.get_logger().warning(f'get_state call failed: {exc}')
            return
        if resp.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE:
            self.get_logger().info('AMCL is ACTIVE — seeding initial pose.')
            self._active = True
            self._poll_timer.cancel()
            self._publish()
            self._sent = 1
            self._pub_timer = self.create_timer(
                self.publish_period, self._pub_tick)

    def _pub_tick(self) -> None:
        if self._sent >= self.publish_count:
            self._pub_timer.cancel()
            self.get_logger().info('Initial pose seeded — seeder exiting.')
            rclpy.shutdown()
            return
        self._publish()
        self._sent += 1

    def _publish(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        # Same covariance RViz's "2D Pose Estimate" uses: confident but not
        # zero, so AMCL keeps a sane particle spread around the seed.
        cov = [0.0] * 36
        cov[0] = 0.25          # x variance  [m^2]
        cov[7] = 0.25          # y variance  [m^2]
        cov[35] = 0.06853892   # yaw variance [rad^2] (~15 deg 1-sigma)
        msg.pose.covariance = cov
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published initial pose ({self.x:.3f}, {self.y:.3f}, '
            f'{self.yaw:.3f}) [{self._sent + 1}/{self.publish_count}]')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
