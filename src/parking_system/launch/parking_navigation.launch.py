#!/usr/bin/env python3
"""
Parking Navigation Launch File
Complete navigation system for the intelligent parking system

Features:
- LiDAR-based localization (AMCL)
- Path planning to parking spots
- Spot visualization in RViz
- Interactive spot selection

TF Tree: map -> odom -> base_link -> laser_link

Usage:
  ros2 launch parking_system parking_navigation.launch.py map_yaml:=/path/to/map.yaml

  Then navigate to spots:
    ros2 topic pub --once /navigate_to_spot std_msgs/String "data: 'spot_1'"
  
  Or run interactive mode:
    ros2 run parking_system spot_navigator.py --interactive
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    pkg_dir = FindPackageShare('parking_system').find('parking_system')
    
    # Configuration files
    nav2_params = PathJoinSubstitution([pkg_dir, 'config', 'nav2_navigation_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'navigation.rviz'])
    
    # ============================================
    # ARGUMENTS
    # ============================================
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR serial port'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='LiDAR baud rate'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/autonexa/intelligent_parking_ws/maps/mapppp.yaml',
        description='Full path to map YAML file'
    )
    
    spots_file_arg = DeclareLaunchArgument(
        'spots_file',
        default_value='/home/autonexa/intelligent_parking_ws/maps/parking_spots.yaml',
        description='Full path to parking spots YAML file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # ============================================
    # TF SETUP
    # ============================================
    
    # Map bootstrap - provides temporary map->odom until AMCL takes over
    map_bootstrap = Node(
        package='parking_system',
        executable='map_bootstrap.py',
        name='map_bootstrap',
        output='screen'
    )
    
    # Static transform: base_link -> laser_link
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    # ============================================
    # SENSORS
    # ============================================
    
    # LiDAR driver
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
    # ODOMETRY
    # ============================================
    
    # Laser scan matcher for odom -> base_link
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
    # NAV2 STACK
    # ============================================
    
    # Map server
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
    
    # AMCL localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}]
    )
    
    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Behavior server (replaces recoveries_server in newer Nav2)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Velocity smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Auto-activate Nav2 nodes using bash script
    nav2_activator = TimerAction(
        period=5.0,  # Wait 5 seconds for nodes to start
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', '''
                    sleep 2
                    for node in map_server amcl controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do
                        ros2 lifecycle set /$node configure 2>/dev/null
                        sleep 0.3
                        ros2 lifecycle set /$node activate 2>/dev/null
                        echo "Activated $node"
                    done
                    echo "=== All Nav2 nodes activated ==="
                '''],
                output='screen'
            )
        ]
    )
    
    # ============================================
    # PARKING SYSTEM NODES
    # ============================================
    
    # Road mask publisher - publishes road constraints for costmap
    road_mask_publisher = Node(
        package='parking_system',
        executable='road_mask_publisher.py',
        name='road_mask_publisher',
        output='screen',
        parameters=[{
            'mask_yaml': '/home/autonexa/intelligent_parking_ws/maps/mapppp_roads.yaml'
        }]
    )
    
    # Spot navigator - handles navigation to parking spots
    spot_navigator = Node(
        package='parking_system',
        executable='spot_navigator.py',
        name='spot_navigator',
        output='screen',
        parameters=[{
            'spots_file': LaunchConfiguration('spots_file')
        }]
    )
    
    # ============================================
    # VISUALIZATION
    # ============================================
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # ============================================
    # LAUNCH
    # ============================================
    
    return LaunchDescription([
        # Arguments
        serial_port_arg,
        serial_baudrate_arg,
        map_yaml_arg,
        spots_file_arg,
        use_rviz_arg,
        
        # TF setup (FIRST!)
        map_bootstrap,
        static_tf_base_to_laser,
        
        # Sensors
        lidar,
        
        # Odometry
        laser_scan_matcher,
        
        # Nav2 stack
        map_server,
        amcl,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        nav2_activator,
        
        # Parking system
        road_mask_publisher,
        spot_navigator,
        
        # Visualization
        rviz,
    ])

