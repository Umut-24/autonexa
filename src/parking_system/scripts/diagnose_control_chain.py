#!/usr/bin/env python3
"""
Diagnose end-to-end control chain health for Nav2 -> Pico integration.

Checks:
1) Required topics exist in ROS graph with expected message types
2) Optional message flow/freshness checks over a fixed observation window

Examples:
  ros2 run parking_system diagnose_control_chain.py
  ros2 run parking_system diagnose_control_chain.py --ros-args -p expect_pico_bridge:=true
  ros2 run parking_system diagnose_control_chain.py --ros-args -p expect_pico_bridge:=true -p require_flow:=true -p window_s:=12.0
"""

import sys
import time
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


@dataclass
class TopicSpec:
    msg_type: object
    ros_type: str
    required_nav: bool
    required_pico: bool
    require_flow: bool


@dataclass
class TopicState:
    count: int = 0
    last_msg_time: float = 0.0


TOPIC_SPECS: Dict[str, TopicSpec] = {
    '/cmd_vel': TopicSpec(Twist, 'geometry_msgs/msg/Twist', True, False, True),
    '/cmd_vel_smoothed': TopicSpec(Twist, 'geometry_msgs/msg/Twist', True, False, True),
    '/cmd_vel_safe': TopicSpec(Twist, 'geometry_msgs/msg/Twist', True, False, True),
    '/pico/control_cmd': TopicSpec(TwistStamped, 'geometry_msgs/msg/TwistStamped', False, True, True),
    '/pico/enable': TopicSpec(Bool, 'std_msgs/msg/Bool', False, True, False),
    '/pico/heartbeat': TopicSpec(Bool, 'std_msgs/msg/Bool', False, True, True),
    '/pico/odom': TopicSpec(Odometry, 'nav_msgs/msg/Odometry', False, False, False),
    '/pico/joint_feedback': TopicSpec(JointState, 'sensor_msgs/msg/JointState', False, False, False),
}


class ControlChainDiagnoser(Node):
    def __init__(self):
        super().__init__('diagnose_control_chain')

        self.declare_parameter('expect_pico_bridge', False)
        self.declare_parameter('require_flow', False)
        self.declare_parameter('require_single_pico_publisher', True)
        self.declare_parameter('window_s', 10.0)
        self.declare_parameter('freshness_timeout_s', 1.5)
        self.declare_parameter('status_period_s', 1.0)

        self.expect_pico_bridge = bool(self.get_parameter('expect_pico_bridge').value)
        self.require_flow = bool(self.get_parameter('require_flow').value)
        self.require_single_pico_publisher = bool(
            self.get_parameter('require_single_pico_publisher').value
        )
        self.window_s = float(self.get_parameter('window_s').value)
        self.freshness_timeout_s = float(self.get_parameter('freshness_timeout_s').value)
        self.status_period_s = float(self.get_parameter('status_period_s').value)

        self.start_time = time.monotonic()
        self.done = False
        self.exit_code = 0

        self.topic_state: Dict[str, TopicState] = {
            topic: TopicState() for topic in TOPIC_SPECS
        }

        for topic, spec in TOPIC_SPECS.items():
            self.create_subscription(spec.msg_type, topic, self._make_cb(topic), 20)

        self.create_timer(self.status_period_s, self._status_tick)

        self.get_logger().info(
            'Control chain diagnosis started '
            f'(expect_pico_bridge={self.expect_pico_bridge}, '
            f'require_flow={self.require_flow}, '
            f'require_single_pico_publisher={self.require_single_pico_publisher}, '
            f'window_s={self.window_s:.1f})'
        )
        if self.require_flow:
            self.get_logger().info(
                'Flow mode enabled. Send a goal or teleop command during the observation window.'
            )

    def _make_cb(self, topic_name):
        def _cb(_msg):
            state = self.topic_state[topic_name]
            state.count += 1
            state.last_msg_time = time.monotonic()
        return _cb

    def _required_topics(self) -> List[str]:
        required = []
        for topic, spec in TOPIC_SPECS.items():
            if spec.required_nav:
                required.append(topic)
            if self.expect_pico_bridge and spec.required_pico:
                required.append(topic)
        return required

    def _status_tick(self):
        now = time.monotonic()
        elapsed = now - self.start_time
        graph = dict(self.get_topic_names_and_types())
        required_topics = self._required_topics()
        single_pub_errors = self._single_publisher_errors()

        missing_topics = []
        type_mismatches = []
        for topic in required_topics:
            spec = TOPIC_SPECS[topic]
            types = graph.get(topic, [])
            if not types:
                missing_topics.append(topic)
                continue
            if spec.ros_type not in types:
                type_mismatches.append((topic, spec.ros_type, types))

        self.get_logger().info(
            f'Elapsed {elapsed:.1f}s/{self.window_s:.1f}s | '
            f'missing_required={len(missing_topics)} type_mismatches={len(type_mismatches)} '
            f'single_pub_errors={len(single_pub_errors)}'
        )

        for topic in required_topics:
            state = self.topic_state[topic]
            hz = state.count / elapsed if elapsed > 0.0 else 0.0
            age = (now - state.last_msg_time) if state.last_msg_time > 0.0 else -1.0
            self.get_logger().info(
                f'  {topic:<20} count={state.count:<4} hz~{hz:>5.2f} age={age:>5.2f}s'
            )

        if elapsed >= self.window_s:
            self._finish(graph, missing_topics, type_mismatches, single_pub_errors, now)

    def _single_publisher_errors(self):
        if not (self.expect_pico_bridge and self.require_single_pico_publisher):
            return []
        errors = []
        for topic in ('/pico/control_cmd', '/pico/enable', '/pico/heartbeat'):
            count = self.count_publishers(topic)
            if count != 1:
                errors.append((topic, count))
        return errors

    def _finish(self, _graph, missing_topics, type_mismatches, single_pub_errors, now):
        required_topics = self._required_topics()
        flow_missing = []
        flow_stale = []

        if self.require_flow:
            for topic in required_topics:
                spec = TOPIC_SPECS[topic]
                if not spec.require_flow:
                    continue
                state = self.topic_state[topic]
                if state.count == 0:
                    flow_missing.append(topic)
                    continue
                age = now - state.last_msg_time
                if age > self.freshness_timeout_s:
                    flow_stale.append((topic, age))

        self.get_logger().info('=== Control Chain Diagnosis Summary ===')
        if missing_topics:
            self.get_logger().error(f'Missing required topics: {missing_topics}')
        if type_mismatches:
            for topic, expected, found in type_mismatches:
                self.get_logger().error(
                    f'Type mismatch on {topic}: expected {expected}, found {found}'
                )
        if flow_missing:
            self.get_logger().error(f'No messages seen on required flow topics: {flow_missing}')
        if flow_stale:
            pretty = [f'{topic}({age:.2f}s)' for topic, age in flow_stale]
            self.get_logger().error(
                f'Stale required flow topics (>{self.freshness_timeout_s:.2f}s): {pretty}'
            )
        if single_pub_errors:
            pretty = [f'{topic}(publishers={count})' for topic, count in single_pub_errors]
            self.get_logger().error(
                f'Single-publisher requirement failed: {pretty}'
            )

        passed = (
            not missing_topics and
            not type_mismatches and
            not flow_missing and
            not flow_stale and
            not single_pub_errors
        )
        if passed:
            self.get_logger().info('PASS: control chain checks succeeded.')
            self.exit_code = 0
        else:
            self.get_logger().error('FAIL: control chain checks failed.')
            self.exit_code = 1

        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = ControlChainDiagnoser()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.2)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(node.exit_code)


if __name__ == '__main__':
    main()
