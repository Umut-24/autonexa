#!/usr/bin/env python3
"""
RPi5 ↔ Pico bridge launch (micro-ROS):
- micro_ros_agent: XRCE-DDS agent over USB serial
- cmd_vel_to_pico_bridge: Nav2 cmd_vel -> rate-limited /pico/control_cmd + /pico/enable

The Pico runs micro-ROS firmware and subscribes/publishes directly.
No serial transceiver or odom conversion nodes are needed.

Usage:
  ros2 launch parking_system rpi5_pico_bridge.launch.py
  ros2 launch parking_system rpi5_pico_bridge.launch.py pico_serial_port:=/dev/ttyACM1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')
    control_cmd_topic_arg = DeclareLaunchArgument('control_cmd_topic', default_value='/pico/control_cmd')
    serial_port_arg = DeclareLaunchArgument('pico_serial_port', default_value='/dev/ttyACM0')

    # micro-ROS agent — bridges XRCE-DDS over USB serial to ROS2 DDS
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', LaunchConfiguration('pico_serial_port'),
            '-b', '115200',
        ],
        output='screen',
    )

    # Bridge: rate/accel limits on Nav2 cmd_vel, publishes TwistStamped + Bool enable
    bridge = Node(
        package='parking_system',
        executable='cmd_vel_to_pico_bridge.py',
        name='cmd_vel_to_pico_bridge',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'control_cmd_topic': LaunchConfiguration('control_cmd_topic'),
            'publish_rate_hz': 30.0,
            'command_timeout_s': 0.20,
            'max_vx_mps': 0.35,
            'max_wz_radps': 0.8,
            'max_ax_mps2': 0.8,
            'max_aw_radps2': 1.2,
        }],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        control_cmd_topic_arg,
        serial_port_arg,
        micro_ros_agent,
        bridge,
    ])
