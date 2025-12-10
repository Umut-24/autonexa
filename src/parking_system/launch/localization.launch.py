#!/usr/bin/env python3
"""
Localization launch file for LiDAR-only robot (no encoders)
Loads a saved map and localizes the robot within it
TF Tree: map -> odom -> base_link -> laser_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package directory for config files
    pkg_dir = FindPackageShare('parking_system').find('parking_system')
    
    # ============================================
    # NODE 1: STATIC TRANSFORM PUBLISHER
    # ============================================
    # Role: Connects robot body to sensor (base_link -> laser_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # ============================================
    # NODE 2: LASER SCAN MATCHER (ODOMETRY SOURCE)
    # ============================================
    # Role: Calculates movement from laser scans (odom -> base_link)
    laser_scan_matcher = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'laser_frame': 'laser_link',
            'publish_tf': True,
            'publish_odom': '/odom',
            'use_sim_time': False,
        }],
        remappings=[
            ('scan', '/scan'),
        ]
    )
    
    # ============================================
    # NODE 3: SLAM TOOLBOX (LOCALIZATION MODE)
    # ============================================
    # Role: Loads saved map and localizes robot (map -> odom)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            # Frame configuration
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            
            # LOCALIZATION MODE
            'mode': 'localization',
            'map_file_name': LaunchConfiguration('map_file'),
            
            # TF settings
            'provide_odom_frame': False,
            'publish_tf': True,
            'use_sim_time': False,
            
            # Laser settings (match your LIDAR)
            'max_laser_range': 16.0,
            
            # Responsiveness
            'transform_timeout': 0.5,
            'tf_buffer_duration': 30.0,
            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.05,
        }]
    )
    
    # ============================================
    # LIDAR DRIVER (needed to provide /scan topic)
    # ============================================
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/autonexa/ws_lidar/install/sllidar_ros2/share/sllidar_ros2/launch/sllidar_c1_launch.py'
        ),
        launch_arguments={
            'frame_id': 'laser_link',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
        }.items()
    )
    
    # Lifecycle manager (required to activate SLAM Toolbox)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 10.0,  # Longer timeout for map loading
            'node_names': ['slam_toolbox']
        }]
    )
    
    # RViz for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='460800'),
        DeclareLaunchArgument('map_file', default_value='/home/autonexa/intelligent_parking_ws/maps/parking_map'),
        
        # NODE 1: Static TF (base_link -> laser_link)
        static_tf,
        
        # LIDAR driver (provides /scan)
        lidar,
        
        # NODE 2: Laser Scan Matcher (odom -> base_link)
        laser_scan_matcher,
        
        # NODE 3: SLAM Toolbox in LOCALIZATION mode (map -> odom)
        slam_toolbox,
        lifecycle_manager,
        
        # Visualization
        rviz,
    ])
