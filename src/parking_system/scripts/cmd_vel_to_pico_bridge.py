#!/usr/bin/env python3
"""
RPi5 bridge node:
- Consumes Nav2 velocity commands (/cmd_vel)
- Applies output rate + acceleration limiting + timeout safety
- Publishes normalized control command for Pico as:
  * geometry_msgs/TwistStamped (/pico/control_cmd)
  * std_msgs/Bool enable state (/pico/enable)
  * std_msgs/Bool heartbeat (/pico/heartbeat)

The Pico subscribes to these topics directly via micro-ROS (XRCE-DDS over USB serial).
"""

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool


@dataclass
class MotionState:
    vx: float = 0.0
    wz: float = 0.0


class CmdVelToPicoBridge(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_pico_bridge')

        # Topics
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_cmd_topic', '/pico/control_cmd')
        self.declare_parameter('enable_topic', '/pico/enable')
        self.declare_parameter('heartbeat_topic', '/pico/heartbeat')

        # Timing and safety
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('command_timeout_s', 0.20)

        # Limits for Ackermann platform command stream
        self.declare_parameter('max_vx_mps', 0.35)
        self.declare_parameter('max_wz_radps', 0.8)
        self.declare_parameter('max_ax_mps2', 0.8)
        self.declare_parameter('max_aw_radps2', 1.2)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_cmd_topic = self.get_parameter('control_cmd_topic').value
        self.enable_topic = self.get_parameter('enable_topic').value
        self.heartbeat_topic = self.get_parameter('heartbeat_topic').value

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.command_timeout = float(self.get_parameter('command_timeout_s').value)

        self.max_vx = float(self.get_parameter('max_vx_mps').value)
        self.max_wz = float(self.get_parameter('max_wz_radps').value)
        self.max_ax = float(self.get_parameter('max_ax_mps2').value)
        self.max_aw = float(self.get_parameter('max_aw_radps2').value)

        self.target = MotionState()
        self.output = MotionState()
        self.last_cmd_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)
        self.cmd_pub = self.create_publisher(TwistStamped, self.control_cmd_topic, 20)
        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 20)
        self.heartbeat_pub = self.create_publisher(Bool, self.heartbeat_topic, 5)

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(dt, self.on_timer)

        self.get_logger().info(
            f'Bridge started: {self.cmd_vel_topic} -> {self.control_cmd_topic}, '
            f'rate={self.publish_rate_hz:.1f}Hz timeout={self.command_timeout:.3f}s'
        )

    def clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def on_cmd_vel(self, msg: Twist) -> None:
        self.target.vx = self.clamp(msg.linear.x, -self.max_vx, self.max_vx)
        self.target.wz = self.clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self.last_cmd_time = self.get_clock().now()

    def apply_rate_limit(self, current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = 1.0 / max(1.0, self.publish_rate_hz)

        command_stale = (now - self.last_cmd_time) > Duration(seconds=self.command_timeout)
        desired = MotionState(0.0, 0.0) if command_stale else self.target

        self.output.vx = self.apply_rate_limit(self.output.vx, desired.vx, self.max_ax * dt)
        self.output.wz = self.apply_rate_limit(self.output.wz, desired.wz, self.max_aw * dt)

        # Publish TwistStamped control command
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = now.to_msg()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.twist.linear.x = self.output.vx
        cmd_msg.twist.angular.z = self.output.wz
        self.cmd_pub.publish(cmd_msg)

        # Publish enable state
        enable_msg = Bool()
        enable_msg.data = not command_stale
        self.enable_pub.publish(enable_msg)

        # Publish heartbeat
        hb = Bool()
        hb.data = True
        self.heartbeat_pub.publish(hb)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToPicoBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Tests often stop this node using timeout/signals; guard shutdown to avoid
        # "rcl_shutdown already called" exceptions on repeated runs.
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
