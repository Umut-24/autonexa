#!/usr/bin/env python3
"""
Localization-only launch file for LiDAR-only robot
Uses localization_slam_toolbox_node to locate robot in a PRE-SAVED map
The map will NOT be modified - only robot position is tracked
TF Tree: map -> odom -> base_link -> laser_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get package directory
    pkg_dir = FindPackageShare('parking_system').find('parking_system')
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'localization.rviz'])
    
    # ============================================
    # NODE 1: STATIC TRANSFORM PUBLISHER
    # ============================================
    # Connects robot body to sensor (base_link -> laser_link)
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
    # Calculates movement from laser scans (odom -> base_link)
    # SAME AS MAPPING - we still need odometry!
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
    # NODE 3: SLAM TOOLBOX (LOCALIZATION MODE - READ ONLY)
    # ============================================
    # Uses localization_slam_toolbox_node for pure localization
    # Map will NOT be modified - only position estimation
    slam_toolbox_localization = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',  # LOCALIZATION-ONLY NODE
        name='slam_toolbox',
        output='screen',
        parameters=[{
            # Frame configuration
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            
            # LOCALIZATION MODE - CRITICAL
            'mode': 'localization',
            
            # MAP FILE PATH
            'map_file_name': LaunchConfiguration('map_file'),
            
            # Start position
            'map_start_at_dock': True,
            
            # TF settings
            'provide_odom_frame': False,
            'publish_tf': True,
            'use_sim_time': False,
            
            # Laser settings
            'max_laser_range': 16.0,
            
            # ============================================
            # DISABLE MAP UPDATES - READ ONLY MODE
            # ============================================
            'do_loop_closing': False,           # Don't close loops (would modify map)
            'enable_interactive_mode': False,   # No interactive editing
            
            # These prevent new scans from being added to the map
            'link_match_minimum_response_fine': 1.0,  # Very high threshold = reject new features
            'link_scan_maximum_distance': 0.0,        # Don't link new scans
            'loop_match_minimum_response_fine': 1.0,  # Very high threshold
            'loop_match_minimum_response_coarse': 1.0,
            
            # Localization settings
            'transform_timeout': 0.5,
            'tf_buffer_duration': 30.0,
            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.05,
            'use_scan_matching': True,
            'resolution': 0.05,
        }]
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
    
    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': ['slam_toolbox']
        }]
    )
    
    # RViz with pre-configured displays
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
        
        # MAP FILE PATH - DEFAULT VALUE
        # Change this to your saved map path (without .posegraph extension)
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/autonexa/intelligent_parking_ws/maps/parking_map',
            description='Path to saved map file (without .posegraph extension)'
        ),
        
        # NODE 1: Static TF (base_link -> laser_link)
        static_tf,
        
        # LIDAR driver (provides /scan)
        lidar,
        
        # NODE 2: Laser Scan Matcher (odom -> base_link)
        laser_scan_matcher,
        
        # NODE 3: SLAM Toolbox LOCALIZATION (map -> odom)
        slam_toolbox_localization,
        lifecycle_manager,
        
        # RViz with auto-loaded displays
        rviz,
    ])

