#!/usr/bin/env python3
"""
Simple navigation launch WITHOUT Nav2 (keeps existing Nav2 untouched).
Uses:
- map_server + AMCL for localization (already stable in your setup)
- road_mask_publisher to constrain navigation to drawn roads
- simple_nav.py (A* planner + pure-pursuit follower)
- laser_scan_matcher for odom->base_link
- sllidar driver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_dir = FindPackageShare('parking_system').find('parking_system')

    nav_params = PathJoinSubstitution([pkg_dir, 'config', 'nav2_navigation_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'navigation.rviz'])

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR serial port'
    )
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='LiDAR baud'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/autonexa/intelligent_parking_ws/maps/emre.yaml',
        description='Map YAML'
    )
    road_mask_arg = DeclareLaunchArgument(
        'road_mask',
        default_value='/home/autonexa/intelligent_parking_ws/maps/emre_roads.yaml',
        description='Road mask YAML'
    )
    spots_file_arg = DeclareLaunchArgument(
        'spots_file',
        default_value='/home/autonexa/intelligent_parking_ws/maps/parking_spots.yaml',
        description='Parking spots YAML'
    )

    # Map server (for map topic)
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

    # AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav_params, {'use_sim_time': False}]
    )

    # LiDAR driver
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser_link',
        }]
    )

    # Laser scan matcher for odom->base_link
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

    # Static TF base->laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # Map bootstrap (temporary map->odom until AMCL active)
    map_bootstrap = Node(
        package='parking_system',
        executable='map_bootstrap.py',
        name='map_bootstrap',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Road mask publisher
    road_mask_publisher = Node(
        package='parking_system',
        executable='road_mask_publisher.py',
        name='road_mask_publisher',
        output='screen',
        parameters=[{
            'mask_yaml': LaunchConfiguration('road_mask'),
            'use_sim_time': False
        }]
    )

    # Simple road-constrained navigator (A* + pure pursuit)
    simple_nav = Node(
        package='parking_system',
        executable='simple_nav.py',
        name='simple_nav',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'road_mask_topic': '/road_mask',
            'spots_file': LaunchConfiguration('spots_file'),
            'cmd_vel_topic': '/cmd_vel',
            'use_sim_time': False
        }]
    )

    # Auto-activate map_server and amcl (they're lifecycle nodes)
    nav_activator = TimerAction(
        period=3.0,  # Wait 3 seconds for nodes to start
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', '''
                    sleep 1
                    ros2 lifecycle set /map_server configure 2>/dev/null || true
                    ros2 lifecycle set /map_server activate 2>/dev/null || true
                    sleep 0.5
                    ros2 lifecycle set /amcl configure 2>/dev/null || true
                    ros2 lifecycle set /amcl activate 2>/dev/null || true
                    echo "=== Map server and AMCL activated ==="
                '''],
                output='screen'
            )
        ]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        map_yaml_arg,
        road_mask_arg,
        spots_file_arg,

        # TF first
        static_tf_base_to_laser,
        map_bootstrap,

        # Sensors
        lidar,

        # Odometry
        laser_scan_matcher,

        # Map + localization
        map_server,
        amcl,

        # Road mask + simple nav
        road_mask_publisher,
        simple_nav,

        # Auto-activate lifecycle nodes
        nav_activator,

        # Visualization
        rviz,
    ])

