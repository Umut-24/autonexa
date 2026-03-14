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

import fcntl
import os
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
        self.declare_parameter('enforce_single_publisher', True)
        self.declare_parameter('bridge_lock_file', '/tmp/cmd_vel_to_pico_bridge.lock')
        self.declare_parameter('duplicate_check_period_s', 1.0)
        self.declare_parameter('duplicate_check_startup_delay_s', 3.0)

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
        self.enforce_single_publisher = bool(self.get_parameter('enforce_single_publisher').value)
        self.bridge_lock_file = str(self.get_parameter('bridge_lock_file').value)
        self.duplicate_check_period_s = float(self.get_parameter('duplicate_check_period_s').value)
        self.duplicate_check_startup_delay_s = float(
            self.get_parameter('duplicate_check_startup_delay_s').value
        )

        self.max_vx = float(self.get_parameter('max_vx_mps').value)
        self.max_wz = float(self.get_parameter('max_wz_radps').value)
        self.max_ax = float(self.get_parameter('max_ax_mps2').value)
        self.max_aw = float(self.get_parameter('max_aw_radps2').value)

        self.target = MotionState()
        self.output = MotionState()
        self.last_cmd_time = self.get_clock().now()
        self.started_at = self.get_clock().now()
        self._lock_file_handle = None
        self._shutdown_requested = False

        if self.enforce_single_publisher:
            self._acquire_bridge_lock()

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 20)
        self.cmd_pub = self.create_publisher(TwistStamped, self.control_cmd_topic, 20)
        self.enable_pub = self.create_publisher(Bool, self.enable_topic, 20)
        self.heartbeat_pub = self.create_publisher(Bool, self.heartbeat_topic, 5)

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(dt, self.on_timer)
        if self.enforce_single_publisher:
            self.duplicate_timer = self.create_timer(
                max(0.2, self.duplicate_check_period_s),
                self.check_for_duplicate_publishers
            )

        self.get_logger().info(
            f'Bridge started: {self.cmd_vel_topic} -> {self.control_cmd_topic}, '
            f'rate={self.publish_rate_hz:.1f}Hz timeout={self.command_timeout:.3f}s '
            f'single_pub_guard={self.enforce_single_publisher}'
        )

    def clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def _acquire_bridge_lock(self) -> None:
        # Host-local process lock to prevent launching multiple bridge instances.
        try:
            self._lock_file_handle = open(self.bridge_lock_file, 'w', encoding='utf-8')
            fcntl.flock(self._lock_file_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_file_handle.write(f'pid={os.getpid()}\n')
            self._lock_file_handle.flush()
        except OSError as exc:
            raise RuntimeError(
                f'Another cmd_vel_to_pico_bridge instance is already running '
                f'(lock: {self.bridge_lock_file}): {exc}'
            ) from exc

    def _release_bridge_lock(self) -> None:
        if self._lock_file_handle is None:
            return
        try:
            fcntl.flock(self._lock_file_handle.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            self._lock_file_handle.close()
        except OSError:
            pass
        self._lock_file_handle = None

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

    def publish_safe_stop_once(self) -> None:
        now = self.get_clock().now()

        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = now.to_msg()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.twist.linear.x = 0.0
        cmd_msg.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd_msg)

        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)

    def check_for_duplicate_publishers(self) -> None:
        if self._shutdown_requested:
            return
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if elapsed < self.duplicate_check_startup_delay_s:
            return

        watched_topics = [self.control_cmd_topic, self.enable_topic, self.heartbeat_topic]
        duplicates = []
        for topic in watched_topics:
            pub_infos = self.get_publishers_info_by_topic(topic)
            if len(pub_infos) > 1:
                publishers = [f'{info.node_namespace}{info.node_name}' for info in pub_infos]
                duplicates.append((topic, publishers))

        if duplicates:
            for topic, publishers in duplicates:
                self.get_logger().error(
                    f'Duplicate publishers detected on {topic}: {publishers}'
                )
            self.get_logger().error(
                'Single-publisher guard triggered. Publishing safe stop and shutting down bridge.'
            )
            self.publish_safe_stop_once()
            self._shutdown_requested = True
            rclpy.shutdown()

    def on_timer(self) -> None:
        if self._shutdown_requested:
            return

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
    node = None
    try:
        node = CmdVelToPicoBridge()
        rclpy.spin(node)
    except RuntimeError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'[cmd_vel_to_pico_bridge] {exc}', flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Tests often stop this node using timeout/signals; guard shutdown to avoid
        # "rcl_shutdown already called" exceptions on repeated runs.
        if node is not None:
            try:
                node._release_bridge_lock()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
