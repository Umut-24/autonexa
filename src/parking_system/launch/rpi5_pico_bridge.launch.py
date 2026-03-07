#!/usr/bin/env python3
"""
Team-B integration launch:
- cmd_vel_to_pico_bridge: Nav2 cmd stream -> Pico command stream
- pico_joint_feedback_to_odom: Pico joint feedback -> odometry topic

Usage:
  ros2 launch parking_system rpi5_pico_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')
    control_cmd_topic_arg = DeclareLaunchArgument('control_cmd_topic', default_value='/pico/control_cmd')
    control_cmd_json_topic_arg = DeclareLaunchArgument('control_cmd_json_topic', default_value='/pico/control_cmd_json')
    joint_feedback_topic_arg = DeclareLaunchArgument('joint_feedback_topic', default_value='/pico/joint_feedback')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/pico/odom')
    
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0', description='USB serial port of Pico')
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='115200', description='Baud rate for Pico serial')

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

    feedback_to_odom = Node(
        package='parking_system',
        executable='pico_joint_feedback_to_odom.py',
        name='pico_joint_feedback_to_odom',
        output='screen',
        parameters=[{
            'joint_feedback_topic': LaunchConfiguration('joint_feedback_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'wheel_radius_m': 0.033,  # updated to match config.h 66mm diameter
            'wheelbase_m': 0.25,      # updated to match config.h 0.25m
        }],
    )

    transceiver = Node(
        package='parking_system',
        executable='pico_serial_transceiver.py',
        name='pico_serial_transceiver',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'control_cmd_json_topic': LaunchConfiguration('control_cmd_json_topic'),
            'joint_feedback_topic': LaunchConfiguration('joint_feedback_topic'),
        }],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        control_cmd_topic_arg,
        control_cmd_json_topic_arg,
        joint_feedback_topic_arg,
        odom_topic_arg,
        serial_port_arg,
        baud_rate_arg,
        bridge,
        transceiver,
        feedback_to_odom,
    ])
