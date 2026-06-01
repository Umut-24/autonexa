#!/usr/bin/env python3
"""Drop non-finite Odometry messages before they reach robot_localization.

laser_scan_matcher can emit a NaN /odom_icp sample during cold start. One bad
sample is enough to poison the EKF state, so this relay republishes only finite
messages from /odom_icp_raw to /odom_icp.
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _finite(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)


class OdomNanFilter(Node):
    def __init__(self) -> None:
        super().__init__('odom_nan_filter')
        self.declare_parameter('input_topic', '/odom_icp_raw')
        self.declare_parameter('output_topic', '/odom_icp')
        self.declare_parameter('warn_period_s', 2.0)
        self.declare_parameter('use_sim_time', False)

        self.warn_period = float(self.get_parameter('warn_period_s').value)
        self._last_warn = 0.0
        self.pub = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 10)
        self.sub = self.create_subscription(
            Odometry, str(self.get_parameter('input_topic').value), self._cb, 10)

        self.get_logger().info(
            f"Filtering {self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}")

    def _cb(self, msg: Odometry) -> None:
        if self._is_finite(msg):
            self.pub.publish(msg)
            return
        now = time.monotonic()
        if now - self._last_warn >= self.warn_period:
            self.get_logger().warning('Dropped non-finite odometry sample')
            self._last_warn = now

    @staticmethod
    def _is_finite(msg: Odometry) -> bool:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        lv = msg.twist.twist.linear
        av = msg.twist.twist.angular
        values = [
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            lv.x, lv.y, lv.z,
            av.x, av.y, av.z,
            *msg.pose.covariance,
            *msg.twist.covariance,
        ]
        return _finite(values)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
