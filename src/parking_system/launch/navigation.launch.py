#!/usr/bin/env python3
"""
Launch file for navigation mode
Enables navigation to selected parking slots with path planning and monitoring
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
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
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/autonexa/intelligent_parking_ws/maps/parking_map.yaml',
        description='Path to the map yaml file'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to launch the LIDAR driver and laser odometry'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Slamtec C1'
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='Serial baudrate for Slamtec C1'
    )
    
    nav2_params_path = PathJoinSubstitution([
        pkg_dir,
        'config',
        'nav2_params.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        pkg_dir,
        'rviz',
        'navigation.rviz'
    ])
    
    # Robot state publisher
    urdf_path = os.path.join(pkg_dir, 'urdf', 'robot.urdf')
    robot_description_content = ''
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as f:
            robot_description_content = f.read()
    
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

    # Temporary map->odom until AMCL publishes
    map_bootstrap_node = Node(
        package='parking_system',
        executable='map_bootstrap.py',
        name='map_bootstrap',
        output='screen'
    )

    # LIDAR driver (Slamtec C1 via sllidar_ros2)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser_link',
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Standard',
        }],
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )

    # Laser scan matcher (provides /odom and odom->base_link)
    laser_odom_config_path = PathJoinSubstitution([
        pkg_dir,
        'config',
        'laser_scan_matcher.yaml'
    ])

    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[
            laser_odom_config_path,
            {
                'use_sim_time': False,
                'publish_tf': True,
                'publish_odom': '/odom',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'laser_frame': 'laser_link',
            }
        ],
        remappings=[
            ('scan', '/scan'),
        ],
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map_file')
        }]
    )
    
    # AMCL for localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_path]
    )

    # Nav2 Smoother Server (required by default BT)
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # Nav2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # Nav2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # Nav2 Behavior Server (recovery behaviors used by the BT)
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # Nav2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # Nav2 Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # Nav2 Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_path]
    )
    
    # Nav2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'smoother_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]}
        ]
    )
    
    # Parking Slot Selector Node
    parking_slot_selector = Node(
        package='parking_system',
        executable='parking_slot_selector.py',
        name='parking_slot_selector',
        output='screen'
    )
    
    # Path Monitor Node
    path_monitor = Node(
        package='parking_system',
        executable='path_monitor.py',
        name='path_monitor',
        output='screen'
    )
    
    # Path Analyzer Node
    path_analyzer = Node(
        package='parking_system',
        executable='path_analyzer.py',
        name='path_analyzer',
        output='screen'
    )
    
    # ArUco Detector Node
    aruco_detector = Node(
        package='parking_system',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen'
    )
    
    # Marker Map Manager Node
    marker_map_manager = Node(
        package='parking_system',
        executable='marker_map_manager.py',
        name='marker_map_manager',
        output='screen',
        parameters=[{
            'map_name': 'parking_map',
            'auto_load_map_markers': True,
        }]
    )
    
    # Enhanced Sensor Fusion Node
    enhanced_sensor_fusion = Node(
        package='parking_system',
        executable='enhanced_sensor_fusion.py',
        name='enhanced_sensor_fusion',
        output='screen'
    )
    
    # ArUco Navigation Node
    aruco_navigation = Node(
        package='parking_system',
        executable='aruco_navigation.py',
        name='aruco_navigation',
        output='screen'
    )
    
    # Marker Selector Node
    marker_selector = Node(
        package='parking_system',
        executable='marker_selector.py',
        name='marker_selector',
        output='screen'
    )
    
    # Parking Coordinator Node
    parking_coordinator = Node(
        package='parking_system',
        executable='parking_coordinator.py',
        name='parking_coordinator',
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
        use_lidar_arg,
        serial_port_arg,
        serial_baudrate_arg,
        map_file_arg,
        robot_state_publisher,
        map_bootstrap_node,
        lidar_node,
        laser_scan_matcher_node,
        map_server_node,
        amcl_node,
        planner_server,
        smoother_server,
        controller_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        parking_slot_selector,
        path_monitor,
        path_analyzer,
        aruco_detector,
        marker_map_manager,
        enhanced_sensor_fusion,
        aruco_navigation,
        marker_selector,
        parking_coordinator,
        rviz_node,
    ])

