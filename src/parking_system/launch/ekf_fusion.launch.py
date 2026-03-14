#!/usr/bin/env python3
"""
Optional EKF fusion bringup (no IMU required yet).

Initial use:
- Fuse Pico wheel odometry (/pico/odom) into /odometry/filtered
- Keep publish_tf disabled to avoid conflicting with scan-matcher odom TF

Future:
- Add IMU source and enable TF publication when fusion replaces scan-matcher odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = FindPackageShare('parking_system').find('parking_system')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'ekf_2d_no_imu.yaml'])
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        ekf,
    ])
