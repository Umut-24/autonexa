#!/usr/bin/env python3
"""
Parking Navigation Launch File
Read-only Nav2 navigation stack that reuses an existing map.

This launch file intentionally avoids any SLAM mapping node so stored maps are
never modified while navigating.
TF Tree: map -> odom -> base_link -> laser_link
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = FindPackageShare('parking_system').find('parking_system')

    # Configuration files
    nav2_params_file = PathJoinSubstitution([pkg_dir, 'config', 'nav2_navigation_params.yaml'])
    ekf_params_file = PathJoinSubstitution([pkg_dir, 'config', 'ekf.yaml'])
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'navigation.rviz'])

    default_map_yaml = os.path.join(os.getcwd(), 'maps', 'parking_map.yaml')
    default_road_mask_yaml = os.path.join(os.getcwd(), 'maps', 'parking_map_roads.yaml')
    default_spots_yaml = os.path.join(os.getcwd(), 'maps', 'parking_spots.yaml')

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
        default_value=default_map_yaml,
        description='Full path to existing map YAML file (read-only in navigation mode)'
    )

    road_mask_yaml_arg = DeclareLaunchArgument(
        'road_mask_yaml',
        default_value=default_road_mask_yaml,
        description='Full path to road mask YAML file'
    )

    spots_file_arg = DeclareLaunchArgument(
        'spots_file',
        default_value=default_spots_yaml,
        description='Full path to parking spots YAML file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    use_road_mask_arg = DeclareLaunchArgument(
        'use_road_mask',
        default_value='false',
        description='Enable road mask constraints in global costmap'
    )

    use_spot_navigator_arg = DeclareLaunchArgument(
        'use_spot_navigator',
        default_value='false',
        description='Enable optional parking-spot navigator helper node'
    )

    # Keepout layer must be disabled when road mask is not used.
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.keepout_filter.enabled': LaunchConfiguration('use_road_mask'),
        },
        convert_types=True,
    )

    # ============================================
    # TF SETUP
    # ============================================
    map_bootstrap = Node(
        package='parking_system',
        executable='map_bootstrap.py',
        name='map_bootstrap',
        output='screen'
    )

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
    laser_scan_matcher = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'laser_frame': 'laser_link',
            'publish_tf': False,  # EKF publishes odom -> base_link
            'publish_odom': '/laser_odom',
            'use_sim_time': False,
        }],
        remappings=[('scan', '/scan')]
    )

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file]
    )

    # ============================================
    # NAV2 STACK (READ-ONLY LOCALIZATION + NAVIGATION)
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

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_nav2_params, {'use_sim_time': False}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_nav2_params]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_nav2_params]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_nav2_params]
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
        }]
    )

    # ============================================
    # OPTIONAL PARKING HELPERS
    # ============================================
    road_mask_publisher = Node(
        package='parking_system',
        executable='road_mask_publisher.py',
        name='road_mask_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_road_mask')),
        parameters=[{
            'mask_yaml': LaunchConfiguration('road_mask_yaml')
        }]
    )

    spot_navigator = Node(
        package='parking_system',
        executable='spot_navigator.py',
        name='spot_navigator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_spot_navigator')),
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

    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        map_yaml_arg,
        road_mask_yaml_arg,
        spots_file_arg,
        use_rviz_arg,
        use_road_mask_arg,
        use_spot_navigator_arg,
        map_bootstrap,
        static_tf_base_to_laser,
        lidar,
        laser_scan_matcher,
        ekf_filter_node,
        map_server,
        amcl,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,
        road_mask_publisher,
        spot_navigator,
        rviz,
    ])
