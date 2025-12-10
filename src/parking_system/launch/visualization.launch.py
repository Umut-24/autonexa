#!/usr/bin/env python3
"""
Launch file for basic robot visualization in RViz
Starts only robot_state_publisher and RViz - no sensors or navigation required
Useful for visualizing the robot model without running the full system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = FindPackageShare('parking_system').find('parking_system')
    
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    rviz_config_path = PathJoinSubstitution([
        pkg_dir,
        'rviz',
        'mapping.rviz'
    ])
    
    # Robot description via robot_state_publisher
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
    robot_description_content = ''
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_description_content = f.read()
    else:
        print(f"Warning: URDF file not found at {urdf_path}")
    
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
    
    # Static transform from map to odom (for visualization without SLAM)
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Static transform from odom to base_link (for visualization)
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    # Static transform from laser to laser_link (LIDAR frame mapping)
    # The sllidar publishes with frame "laser" but robot URDF uses "laser_link"
    static_tf_laser_to_laser_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser_to_laser_link',
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'laser_link'],
        output='screen'
    )
    
    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher,
        static_tf_map_to_odom,
        static_tf_odom_to_base,
        static_tf_laser_to_laser_link,
        rviz_node,
    ])

