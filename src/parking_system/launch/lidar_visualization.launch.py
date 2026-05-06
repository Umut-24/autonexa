#!/usr/bin/env python3
"""
Launch file for LIDAR and robot visualization in RViz
Starts LIDAR driver, robot model, and RViz with proper transform chain
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to launch LIDAR driver'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LIDAR'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',  # Default for Slamtec C1
        description='Serial baudrate for LIDAR'
    )
    
    rviz_config_path = PathJoinSubstitution([
        pkg_dir,
        'rviz',
        'visualization.rviz'  # Use visualization config (no map)
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
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen'
    )
    
    # Static transform from odom to base_link (for visualization)
    static_tf_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_base',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', '--frame-id', 'odom', '--child-frame-id', 'base_link'],
        output='screen'
    )
    
    # Note: LIDAR is configured to publish with 'laser_link' frame directly
    # No transform from 'laser' to 'laser_link' is needed
    
    # LIDAR launch (sllidar_ros2) — resolve via FindPackageShare so the launch
    # works regardless of which workspace built the driver.
    sllidar_share = FindPackageShare('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_share, 'launch', 'sllidar_c1_launch.py'])
        ),
        launch_arguments={
            'frame_id': 'laser_link',  # Override to use laser_link to match robot URDF
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_lidar'))
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
        use_lidar_arg,
        serial_port_arg,
        serial_baudrate_arg,
        robot_state_publisher,
        static_tf_map_to_odom,
        static_tf_odom_to_base,
        lidar_launch,
        rviz_node,
    ])

