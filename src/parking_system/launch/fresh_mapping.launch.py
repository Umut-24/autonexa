#!/usr/bin/env python3
"""
FRESH MAPPING - Starts completely from scratch
No config files, no cached data, no old maps
Everything is defined inline to ensure clean start
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # ============================================
    # NODE 1: STATIC TRANSFORM (base_link -> laser_link)
    # ============================================
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen'
    )
    
    # ============================================
    # NODE 2: LASER SCAN MATCHER (odom -> base_link)
    # ============================================
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
        remappings=[('scan', '/scan')]
    )
    
    # ============================================
    # NODE 3: SLAM TOOLBOX - FRESH MAPPING
    # ============================================
    # ALL PARAMETERS INLINE - No config file!
    # This ensures absolutely no old data is loaded
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            # ========== FRESH START ==========
            'mode': 'mapping',
            'map_file_name': '',              # EMPTY - no map to load
            'map_start_at_dock': False,       # Don't look for saved position
            'map_start_pose': [0.0, 0.0, 0.0],  # Start at origin
            
            # ========== FRAME CONFIG ==========
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            
            # ========== ODOMETRY ==========
            'provide_odom_frame': False,      # laser_scan_matcher provides this
            'publish_tf': True,
            'use_sim_time': False,
            
            # ========== MAP SETTINGS ==========
            'resolution': 0.05,               # 5cm per pixel
            'max_laser_range': 16.0,          # Match LIDAR
            'minimum_travel_distance': 0.1,   # Update every 10cm
            'minimum_travel_heading': 0.1,    # Update every ~6 degrees
            'map_update_interval': 1.0,       # Update map every second
            
            # ========== SCAN MATCHING ==========
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_time_interval': 0.0,
            'transform_timeout': 0.5,
            'tf_buffer_duration': 30.0,
            'scan_buffer_size': 20,
            
            # ========== CORRELATION ==========
            'correlation_search_space_dimension': 0.5,
            'correlation_search_space_resolution': 0.01,
            'correlation_search_space_smear_deviation': 0.03,
            
            # ========== LOOP CLOSURE ==========
            'do_loop_closing': True,
            'loop_search_space_dimension': 3.0,
            'loop_search_space_resolution': 0.05,
            'loop_search_space_smear_deviation': 0.03,
            'loop_search_maximum_distance': 3.0,
            
            # ========== PUBLISHING ==========
            'publish_map_updates': True,
            'enable_interactive_mode': False,
        }]
    )
    
    # ============================================
    # NODE 4: MARKER MAPPER (records markers during SLAM)
    # ============================================
    marker_mapper = Node(
        package='parking_system',
        executable='marker_mapper.py',
        name='marker_mapper',
        output='screen',
        parameters=[{
            'map_name': 'parking_map',  # Will create maps/parking_map_markers.yaml
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
            'node_names': ['slam_toolbox']
        }]
    )
    
    # ============================================
    # RVIZ
    # ============================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='460800'),
        
        static_tf,
        lidar,
        laser_scan_matcher,
        slam_toolbox,
        marker_mapper,
        lifecycle_manager,
        rviz,
    ])

