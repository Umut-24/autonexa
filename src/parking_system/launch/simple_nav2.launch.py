#!/usr/bin/env python3
"""
COMPLETE NAV2 LAUNCH - Everything in one file!
Just run this and click goals in RViz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_dir = get_package_share_directory('parking_system')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Files
    map_file = '/home/autonexa/intelligent_parking_ws/maps/emre.yaml'
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation.rviz')
    
    return LaunchDescription([
        # Set log level
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Launch Nav2 bringup (includes map_server, amcl, planner, controller, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()
        ),
        
        # Launch RViz with navigation config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
