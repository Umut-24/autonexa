#!/usr/bin/env python3
"""
Nav2 -> Pico ASCII serial bridge launch (standalone).

Brings up only the bridge node. Compose with Nav2/SLAM separately:

    Terminal 1 — Nav2 + SLAM + LiDAR (no Pico bridge of its own):
        ros2 launch parking_system nav2_live_slam.launch.py use_pico_bridge:=false

    Terminal 2 — this bridge:
        ros2 launch parking_system nav2_pico_serial.launch.py

Or for first-cut bench testing without Nav2:

    Terminal 1: ros2 launch parking_system nav2_pico_serial.launch.py
    Terminal 2: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                  "{linear: {x: 0.1}, angular: {z: 0.0}}" --rate 10

Mutually exclusive with test/pico_gui.py — both want /dev/ttyACM0.
Close one before starting the other.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('serial_port',         default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('serial_baud',         default_value='115200'),
        DeclareLaunchArgument('cmd_vel_topic',       default_value='/cmd_vel'),
        DeclareLaunchArgument('publish_rate_hz',     default_value='30.0'),
        DeclareLaunchArgument('command_timeout_s',   default_value='0.20'),
        DeclareLaunchArgument('max_vx_mps',          default_value='0.25',
                              description='Linear speed clamp (m/s).'),
        DeclareLaunchArgument('max_wz_radps',        default_value='0.8',
                              description='Angular rate clamp (rad/s).'),
        DeclareLaunchArgument('max_ax_mps2',         default_value='0.60'),
        DeclareLaunchArgument('max_aw_radps2',       default_value='0.50'),
        DeclareLaunchArgument('min_vx_creep',        default_value='0.10',
                              description='|vx| below this -> SPEEDS 0 0.'),
        DeclareLaunchArgument('wheelbase_m',         default_value='0.25'),
        DeclareLaunchArgument('track_width_m',       default_value='0.20'),
        DeclareLaunchArgument('servo_center_us',     default_value='1650',
                              description='Calibrated servo neutral pulse width (us).'),
        DeclareLaunchArgument('servo_us_min',        default_value='1150',
                              description='Hard min pulse width (us); never sent below this. Symmetric ±500 µs around 1650 center.'),
        DeclareLaunchArgument('servo_us_max',        default_value='2150',
                              description='Hard max pulse width (us); never sent above this. Symmetric ±500 µs around 1650 center.'),
        DeclareLaunchArgument('servo_polarity',      default_value='+1',
                              description='+1 for this chassis: verified on hardware that ROS-positive wz (left turn) needs servo us < center. Was -1; flipped 2026-06-01 after observing reversed steering in both joystick and Nav2.'),
        DeclareLaunchArgument('reverse_steer_polarity', default_value='-1',
                              description='Flip steering sign only while reversing.'),
        DeclareLaunchArgument('max_steer_rate_radps', default_value='3.0',
                              description='Servo slew-rate cap (rad/s).'),
        DeclareLaunchArgument('auto_enable',         default_value='true',
                              description='Send ENABLE to Pico on launch.'),
        DeclareLaunchArgument('dry_run',             default_value='false',
                              description='Skip serial open; log would-be commands only.'),
    ]

    bridge = Node(
        package='parking_system',
        executable='nav2_pico_bridge.py',
        name='nav2_pico_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port':       LaunchConfiguration('serial_port'),
            'serial_baud':       LaunchConfiguration('serial_baud'),
            'cmd_vel_topic':     LaunchConfiguration('cmd_vel_topic'),
            'publish_rate_hz':   LaunchConfiguration('publish_rate_hz'),
            'command_timeout_s': LaunchConfiguration('command_timeout_s'),
            'max_vx_mps':        LaunchConfiguration('max_vx_mps'),
            'max_wz_radps':      LaunchConfiguration('max_wz_radps'),
            'max_ax_mps2':       LaunchConfiguration('max_ax_mps2'),
            'max_aw_radps2':     LaunchConfiguration('max_aw_radps2'),
            'min_vx_creep':      LaunchConfiguration('min_vx_creep'),
            'wheelbase_m':       LaunchConfiguration('wheelbase_m'),
            'track_width_m':     LaunchConfiguration('track_width_m'),
            'servo_center_us':   LaunchConfiguration('servo_center_us'),
            'servo_us_min':      LaunchConfiguration('servo_us_min'),
            'servo_us_max':      LaunchConfiguration('servo_us_max'),
            'servo_polarity':    LaunchConfiguration('servo_polarity'),
            'reverse_steer_polarity': LaunchConfiguration('reverse_steer_polarity'),
            'max_steer_rate_radps': LaunchConfiguration('max_steer_rate_radps'),
            'auto_enable':       LaunchConfiguration('auto_enable'),
            'dry_run':           LaunchConfiguration('dry_run'),
        }],
    )

    return LaunchDescription(args + [bridge])
