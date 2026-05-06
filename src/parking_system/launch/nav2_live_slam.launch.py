#!/usr/bin/env python3
"""
Live Nav2 + SLAM launch (LiDAR-only)

What this provides:
- Start RViz + Nav2 directly
- Build map online while driving/exploring (no pre-saved map required)
- Click goal in RViz and navigate to it
- Obstacle avoidance through Nav2 costmaps + controller

TF Tree: map -> odom -> base_link -> laser_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = FindPackageShare('parking_system').find('parking_system')

    nav2_params_file = PathJoinSubstitution([pkg_dir, 'config', 'nav2_navigation_params.yaml'])
    slam_params_file = PathJoinSubstitution([pkg_dir, 'config', 'slam_toolbox_mapping.yaml'])
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'navigation.rviz'])

    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    serial_baudrate_arg = DeclareLaunchArgument('serial_baudrate', default_value='460800')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    # Enable the Pico path by default so Nav2 commands reach the motors out of
    # the box. Pass use_pico_bridge:=false to run a headless simulation without
    # hardware.
    use_pico_bridge_arg = DeclareLaunchArgument('use_pico_bridge', default_value='true')
    pico_serial_port_arg = DeclareLaunchArgument('pico_serial_port', default_value='/dev/ttyACM0')
    # Flask HTTP bridge for the Flutter mobile app (joystick, telemetry, map).
    # Publishes joystick to /cmd_vel so the manual path shares the autonomous
    # safety chain: /cmd_vel -> velocity_smoother -> collision_monitor
    # -> /cmd_vel_safe -> cmd_vel_to_pico_bridge -> /pico/control_cmd.
    use_mobile_bridge_arg = DeclareLaunchArgument(
        'use_mobile_bridge',
        default_value='true',
        description='Launch the Flask HTTP bridge (ros2_mobile_bridge) for the Flutter app'
    )
    enforce_single_pub_arg = DeclareLaunchArgument(
        'enforce_single_publisher',
        default_value='true',
        description='Stop bridge if duplicate publishers are detected on /pico command topics'
    )
    bridge_lock_file_arg = DeclareLaunchArgument(
        'bridge_lock_file',
        default_value='/tmp/cmd_vel_to_pico_bridge.lock'
    )
    bridge_cmd_vel_topic_arg = DeclareLaunchArgument(
        'bridge_cmd_vel_topic',
        default_value='/cmd_vel_safe',
        description='Final velocity topic consumed by the Pico bridge'
    )
    use_micropython_bridge_arg = DeclareLaunchArgument(
        'use_micropython_bridge',
        default_value='false',
        description='Use the new MicroPython serial bridge instead of micro_ros_agent + cmd_vel_to_pico_bridge'
    )
    micropython_lock_file_arg = DeclareLaunchArgument(
        'micropython_lock_file',
        default_value='/tmp/pico_serial_bridge.lock',
    )

    # In live SLAM mode there is no road-mask topic by default.
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.keepout_filter.enabled': 'false',
        },
        convert_types=True,
    )

    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_link'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

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

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'mode': 'mapping',
                'map_file_name': '',
                'map_start_at_dock': False,
                'use_sim_time': False,
            }
        ]
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

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[configured_nav2_params]
    )

    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox']
        }]
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
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ]
        }]
    )

    use_legacy_bridge = AndSubstitution(
        LaunchConfiguration('use_pico_bridge'),
        NotSubstitution(LaunchConfiguration('use_micropython_bridge')),
    )
    use_mp_bridge = AndSubstitution(
        LaunchConfiguration('use_pico_bridge'),
        LaunchConfiguration('use_micropython_bridge'),
    )

    # Legacy: micro-ROS agent + cmd_vel_to_pico_bridge (used with C firmware)
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', LaunchConfiguration('pico_serial_port'),
            '-b', '115200',
        ],
        output='screen',
        condition=IfCondition(use_legacy_bridge),
    )

    cmd_vel_to_pico_bridge = Node(
        package='parking_system',
        executable='cmd_vel_to_pico_bridge.py',
        name='cmd_vel_to_pico_bridge',
        output='screen',
        condition=IfCondition(use_legacy_bridge),
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('bridge_cmd_vel_topic'),
            'control_cmd_topic': '/pico/control_cmd',
            'enable_topic': '/pico/enable',
            'heartbeat_topic': '/pico/heartbeat',
            'publish_rate_hz': 30.0,
            'command_timeout_s': 0.20,
            'max_vx_mps': 0.35,
            'max_wz_radps': 0.8,
            'max_ax_mps2': 0.8,
            'max_aw_radps2': 1.2,
            'enforce_single_publisher': LaunchConfiguration('enforce_single_publisher'),
            'bridge_lock_file': LaunchConfiguration('bridge_lock_file'),
        }],
    )

    # New: MicroPython serial bridge (replaces micro_ros_agent + cmd_vel_to_pico_bridge).
    pico_serial_bridge = Node(
        package='parking_system',
        executable='pico_serial_bridge.py',
        name='pico_serial_bridge',
        output='screen',
        condition=IfCondition(use_mp_bridge),
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('bridge_cmd_vel_topic'),
            'control_cmd_topic': '/pico/control_cmd',
            'enable_topic': '/pico/enable',
            'heartbeat_topic': '/pico/heartbeat',
            'odom_topic': '/pico/odom',
            'joint_topic': '/pico/joint_feedback',
            'estop_service': '/pico/estop',
            'serial_port': LaunchConfiguration('pico_serial_port'),
            'serial_baud': 115200,
            'publish_rate_hz': 30.0,
            'command_timeout_s': 0.20,
            'max_vx_mps': 0.35,
            'max_wz_radps': 0.8,
            'max_ax_mps2': 0.8,
            'max_aw_radps2': 1.2,
            'enforce_single_publisher': LaunchConfiguration('enforce_single_publisher'),
            'bridge_lock_file': LaunchConfiguration('micropython_lock_file'),
        }],
    )

    # HTTP bridge for Flutter mobile app: /api/control (joystick), /api/status,
    # /api/map, /api/nav_goal, /api/estop, /video_feed. Joystick input becomes
    # a Twist on /cmd_vel so it flows through the same safety chain as Nav2.
    mobile_bridge = Node(
        package='parking_system',
        executable='ros2_mobile_bridge.py',
        name='mobile_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mobile_bridge')),
    )

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
        use_rviz_arg,
        use_pico_bridge_arg,
        pico_serial_port_arg,
        enforce_single_pub_arg,
        bridge_lock_file_arg,
        bridge_cmd_vel_topic_arg,
        use_micropython_bridge_arg,
        micropython_lock_file_arg,
        use_mobile_bridge_arg,
        static_tf_base_to_laser,
        lidar,
        laser_scan_matcher,
        slam_toolbox,
        lifecycle_manager_slam,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_navigation,
        micro_ros_agent,
        cmd_vel_to_pico_bridge,
        pico_serial_bridge,
        mobile_bridge,
        rviz,
    ])
