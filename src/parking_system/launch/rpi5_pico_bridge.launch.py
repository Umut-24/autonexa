#!/usr/bin/env python3
"""
RPi5 <-> Pico bridge launch.

Two mutually-exclusive backends:
  use_micropython_bridge:=false (default)
    Legacy: micro_ros_agent + cmd_vel_to_pico_bridge (for C/micro-ROS firmware)
  use_micropython_bridge:=true
    New: pico_serial_bridge (for pico_micropython/main.py)

Usage:
  ros2 launch parking_system rpi5_pico_bridge.launch.py
  ros2 launch parking_system rpi5_pico_bridge.launch.py use_micropython_bridge:=true
  ros2 launch parking_system rpi5_pico_bridge.launch.py pico_serial_port:=/dev/ttyACM1
  ros2 launch parking_system rpi5_pico_bridge.launch.py use_mobile_bridge:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


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
        default_value='/tmp/cmd_vel_to_pico_bridge.lock',
        description='fcntl lock path for the legacy bridge (ignored for MicroPython)'
    )
    micropython_lock_file_arg = DeclareLaunchArgument(
        'micropython_lock_file',
        default_value='/tmp/pico_serial_bridge.lock',
        description='fcntl lock path for the MicroPython bridge'
    )
    use_mobile_bridge_arg = DeclareLaunchArgument(
        'use_mobile_bridge', default_value='true',
        description='Launch the HTTP bridge for Flutter app joystick control'
    )
    enable_web_terminal_arg = DeclareLaunchArgument(
        'enable_web_terminal', default_value='true',
        description='Expose the full web terminal (PTY) in the app. SECURITY: '
                    'arbitrary shell as the robot user for anyone on the LAN — '
                    'set false to disable without a code change.'
    )
    use_micropython_bridge_arg = DeclareLaunchArgument(
        'use_micropython_bridge', default_value='false',
        description='Use the new MicroPython serial bridge (replaces micro_ros_agent + cmd_vel_to_pico_bridge)'
    )

    use_legacy = NotSubstitution(LaunchConfiguration('use_micropython_bridge'))
    use_mp     = LaunchConfiguration('use_micropython_bridge')

    # Legacy path — micro-ROS agent bridges XRCE-DDS over USB serial to ROS2 DDS.
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', LaunchConfiguration('pico_serial_port'),
            '-b', '115200',
        ],
        output='screen',
        condition=IfCondition(use_legacy),
    )

    legacy_bridge = Node(
        package='parking_system',
        executable='cmd_vel_to_pico_bridge.py',
        name='cmd_vel_to_pico_bridge',
        output='screen',
        condition=IfCondition(use_legacy),
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

    # New path — one node speaks the MicroPython line protocol on USB serial
    # and publishes all the /pico/* topics directly.
    micropython_bridge = Node(
        package='parking_system',
        executable='pico_serial_bridge.py',
        name='pico_serial_bridge',
        output='screen',
        condition=IfCondition(use_mp),
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'control_cmd_topic': LaunchConfiguration('control_cmd_topic'),
            'serial_port': LaunchConfiguration('pico_serial_port'),
            'serial_baud': 115200,
            'publish_rate_hz': 30.0,
            'command_timeout_s': 0.20,
            'max_vx_mps': 0.35,
            'max_wz_radps': 0.8,
            'max_ax_mps2': 0.8,
            'max_aw_radps2': 1.2,
            'enforce_single_publisher': LaunchConfiguration('enforce_single_publisher'),
            'bridge_lock_file': LaunchConfiguration('micropython_lock_file'),
        }],
    )

    mobile_bridge = Node(
        package='parking_system',
        executable='ros2_mobile_bridge.py',
        name='mobile_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mobile_bridge')),
        parameters=[{
            'active_map_yaml': '',
            'enable_web_terminal': ParameterValue(
                LaunchConfiguration('enable_web_terminal'), value_type=bool),
        }],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        control_cmd_topic_arg,
        serial_port_arg,
        enforce_single_pub_arg,
        bridge_lock_file_arg,
        micropython_lock_file_arg,
        use_mobile_bridge_arg,
        enable_web_terminal_arg,
        use_micropython_bridge_arg,
        micro_ros_agent,
        legacy_bridge,
        micropython_bridge,
        mobile_bridge,
    ])
