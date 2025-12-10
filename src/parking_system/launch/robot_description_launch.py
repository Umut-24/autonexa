#!/usr/bin/env python3
"""
Helper launch file for robot description
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = FindPackageShare('parking_system').find('parking_system')
    
    urdf_file = PathJoinSubstitution([
        pkg_dir,
        'urdf',
        'robot.urdf'
    ])
    
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher,
    ])

