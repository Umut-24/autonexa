#!/usr/bin/env python3
"""
RPi5 ↔ Pico bridge launch (micro-ROS):
- micro_ros_agent: XRCE-DDS agent over USB serial
- cmd_vel_to_pico_bridge: Nav2 cmd_vel -> rate-limited /pico/control_cmd + /pico/enable
- ros2_mobile_bridge (optional): HTTP server for Flutter app joystick control

The Pico runs micro-ROS firmware and subscribes/publishes directly.
No serial transceiver or odom conversion nodes are needed.

Usage:
  ros2 launch parking_system rpi5_pico_bridge.launch.py
  ros2 launch parking_system rpi5_pico_bridge.launch.py pico_serial_port:=/dev/ttyACM1
  ros2 launch parking_system rpi5_pico_bridge.launch.py use_mobile_bridge:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')
    control_cmd_topic_arg = DeclareLaunchArgument('control_cmd_topic', default_value='/pico/control_cmd')
    serial_port_arg = DeclareLaunchArgument('pico_serial_port', default_value='/dev/ttyACM0')
    enforce_single_pub_arg = DeclareLaunchArgument(
        'enforce_single_publisher',
        default_value='true',
        description='Stop bridge if duplicate publishers are detected on /pico command topics'
    )
    bridge_lock_file_arg = DeclareLaunchArgument(
        'bridge_lock_file',
        default_value='/tmp/cmd_vel_to_pico_bridge.lock'
    )
    use_mobile_bridge_arg = DeclareLaunchArgument(
        'use_mobile_bridge', default_value='true',
        description='Launch the HTTP bridge for Flutter app joystick control'
    )

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
            'enforce_single_publisher': LaunchConfiguration('enforce_single_publisher'),
            'bridge_lock_file': LaunchConfiguration('bridge_lock_file'),
        }],
    )

    # HTTP bridge for Flutter mobile app (joystick, telemetry, map, etc.)
    mobile_bridge = Node(
        package='parking_system',
        executable='ros2_mobile_bridge.py',
        name='mobile_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mobile_bridge')),
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        control_cmd_topic_arg,
        serial_port_arg,
        enforce_single_pub_arg,
        bridge_lock_file_arg,
        use_mobile_bridge_arg,
        micro_ros_agent,
        bridge,
        mobile_bridge,
    ])
