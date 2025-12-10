#!/usr/bin/env python3
"""
AMCL Localization - TRUE READ-ONLY localization
Loads a saved map and estimates robot position using particle filter
The map is NEVER modified - only position is estimated
TF Tree: map -> odom -> base_link -> laser_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    pkg_dir = FindPackageShare('parking_system').find('parking_system')
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'localization.rviz'])
    # Use small-scale config optimized for 2x2m testbed
    amcl_config = PathJoinSubstitution([pkg_dir, 'config', 'amcl_small_scale.yaml'])
    
    # ============================================
    # NODE 0: MAP BOOTSTRAP (Temporary map -> odom)
    # ============================================
    # Publishes dynamic map->odom until AMCL takes over
    # This ensures TF tree is connected immediately
    map_bootstrap = Node(
        package='parking_system',
        executable='map_bootstrap.py',
        name='map_bootstrap',
        output='screen'
    )
    
    # ============================================
    # NODE 1: STATIC TRANSFORM (base_link -> laser_link)
    # ============================================
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # ============================================
    # NODE 2: LASER SCAN MATCHER (odom -> base_link)
    # ============================================
    # Role: Calculates movement from laser scans (odom -> base_link)
    # MUST match the working mapping.launch.py exactly!
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
    # NODE 3: MAP SERVER (Loads and publishes the map)
    # ============================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_yaml'),
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': False,
        }]
    )
    
    # ============================================
    # NODE 4: AMCL (Particle Filter Localization)
    # ============================================
    # Uses optimized config for GLOBAL LOCALIZATION
    # High particle count, frequent updates, tuned for 2x2m testbed
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': False}
        ]
    )
    
    # ============================================
    # LIDAR DRIVER
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
    
    # ============================================
    # LIFECYCLE MANAGER
    # ============================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # ============================================
    # RVIZ with pre-loaded displays
    # ============================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='460800'),
        
        # MAP FILE - Update this to your map path!
        DeclareLaunchArgument(
            'map_yaml',
            default_value='/home/autonexa/intelligent_parking_ws/maps/my_map.yaml',
            description='Full path to map YAML file'
        ),
        
        # Launch nodes in order
        map_bootstrap,       # FIRST! Temporary map -> odom until AMCL ready
        static_tf,           # base_link -> laser_link
        lidar,               # /scan topic
        laser_scan_matcher,  # odom -> base_link
        map_server,          # Publishes /map
        amcl,                # map -> odom (localization) - will override bootstrap
        lifecycle_manager,   # Activates map_server and amcl
        rviz,                # Visualization
    ])
