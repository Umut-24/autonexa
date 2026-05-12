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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

# Render the robot description from the xacro template + any persisted
# dimension overrides at launch time. The bridge can later re-publish
# /robot_description live via SetParameters on robot_state_publisher.
import sys as _sys
_scripts_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, 'scripts'
)
if _scripts_dir not in _sys.path:
    _sys.path.insert(0, _scripts_dir)
try:
    from parking_system.build_urdf import render as _render_urdf
except Exception:  # fallback to the static URDF if the renderer is unavailable
    _render_urdf = None


def generate_launch_description():
    pkg_dir = FindPackageShare('parking_system').find('parking_system')

    if _render_urdf is not None:
        robot_description_content, _rendered_footprint, _rendered_dims = _render_urdf()
    else:
        _legacy_urdf = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), os.pardir, 'urdf', 'robot.urdf'
        )
        with open(_legacy_urdf, 'r') as _f:
            robot_description_content = _f.read()

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
        description='Use the (now-removed) MicroPython serial bridge. Dead code; kept only for backward arg compatibility.'
    )
    micropython_lock_file_arg = DeclareLaunchArgument(
        'micropython_lock_file',
        default_value='/tmp/pico_serial_bridge.lock',
    )
    use_serial_bridge_arg = DeclareLaunchArgument(
        'use_serial_bridge',
        default_value='true',
        description='Use the new ASCII Nav2 bridge (nav2_pico_bridge.py) over USB serial to the L298N CLI firmware. Default true; set false to fall back to the legacy micro-ROS path.'
    )
    serial_bridge_lock_file_arg = DeclareLaunchArgument(
        'serial_bridge_lock_file',
        default_value='/tmp/nav2_pico_bridge.lock',
    )
    min_vx_creep_arg = DeclareLaunchArgument(
        'min_vx_creep',
        default_value='0.02',
        description='|vx| below this -> SPEED 0 in the ASCII bridge (firmware deadband workaround).'
    )
    servo_center_us_arg = DeclareLaunchArgument('servo_center_us', default_value='1650')
    # Symmetric ±500 µs around the 1650 center — the previous 1100/1900
    # defaults gave 550 µs travel left vs 250 µs right, which translated
    # to ~2× more steering on left turns than right.
    servo_us_min_arg = DeclareLaunchArgument('servo_us_min', default_value='1150')
    servo_us_max_arg = DeclareLaunchArgument('servo_us_max', default_value='2150')
    servo_polarity_arg = DeclareLaunchArgument('servo_polarity', default_value='-1')
    reverse_steer_polarity_arg = DeclareLaunchArgument(
        'reverse_steer_polarity',
        default_value='-1',
        description='Flip steering sign only while reversing. -1 matches this chassis reverse maneuvering.')
    # +1 = ROS-positive vx -> chassis forward (standard). The mobile app's
    # Calibrate Direction wizard flips this at runtime via SetParameters and
    # persists the chosen value to ~/.autonexa/runtime_overrides.yaml.
    vx_polarity_arg = DeclareLaunchArgument('vx_polarity', default_value='1')
    max_steer_rate_arg = DeclareLaunchArgument(
        'max_steer_rate_radps', default_value='3.0',
        description='Servo slew-rate cap (rad/s). Smooths Nav2 wz step changes.')

    # In live SLAM mode there is no road-mask topic by default.
    # The Ackermann-aware BT XML at config/bt_navigate_to_pose_ackermann.xml
    # is kept in the package for future opt-in; bt_navigator currently uses
    # the stock minimal tree configured in nav2_navigation_params.yaml,
    # which never invokes Spin so the Ackermann constraint is already met.
    # If we successfully rendered the URDF, push the matching footprint into
    # both costmaps so Nav2 starts up consistent with any user dimension
    # overrides. The bridge can further override at runtime via SetParameters.
    _footprint_overrides = {}
    if _render_urdf is not None:
        _footprint_overrides = {
            'global_costmap.global_costmap.ros__parameters.footprint': _rendered_footprint,
            'global_costmap.global_costmap.ros__parameters.footprint_padding': str(_rendered_dims['footprint_padding']),
            'local_costmap.local_costmap.ros__parameters.footprint': _rendered_footprint,
            'local_costmap.local_costmap.ros__parameters.footprint_padding': str(_rendered_dims['footprint_padding']),
        }

    # Point bt_navigator at the package's Ackermann BT (Spin-stripped + 1 Hz
    # periodic replan) instead of the stock invalidation-only tree.
    _bt_xml_path = os.path.join(pkg_dir, 'config', 'bt_navigate_to_pose_ackermann.xml')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.keepout_filter.enabled': 'false',
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': _bt_xml_path,
            **_footprint_overrides,
        },
        convert_types=True,
    )

    # base_link -> laser_link is now driven by robot_state_publisher from the
    # rendered URDF (laser_joint origin honors the LiDAR's actual mount offset,
    # ~1 cm forward of chassis center, plus the yaw=π for rear-facing mount).
    # The earlier static_transform_publisher placed the LiDAR at base_link
    # origin which made SLAM/costmaps mis-register obstacles by the URDF offset
    # amount (was masked because the old URDF had a wrong 15 cm offset too).
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content,
        }],
    )

    # sllidar_ros2 lives in this workspace under src/sllidar_ros2; resolve via
    # FindPackageShare so the launch is portable and not tied to a separate
    # ws_lidar workspace.
    sllidar_share = FindPackageShare('sllidar_ros2')
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_share, 'launch', 'sllidar_c1_launch.py'])
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

    # Three-way mutually exclusive backend selection. Priority: serial > mp > legacy.
    # If use_serial_bridge:=true (default), the ASCII Nav2 bridge runs and the
    # micro-ROS legacy path is suppressed. The MicroPython script is dead so
    # use_micropython_bridge:=true currently fails to find an executable.
    use_serial_bridge_active = AndSubstitution(
        LaunchConfiguration('use_pico_bridge'),
        LaunchConfiguration('use_serial_bridge'),
    )
    use_legacy_bridge = AndSubstitution(
        LaunchConfiguration('use_pico_bridge'),
        AndSubstitution(
            NotSubstitution(LaunchConfiguration('use_serial_bridge')),
            NotSubstitution(LaunchConfiguration('use_micropython_bridge')),
        ),
    )
    use_mp_bridge = AndSubstitution(
        LaunchConfiguration('use_pico_bridge'),
        AndSubstitution(
            LaunchConfiguration('use_micropython_bridge'),
            NotSubstitution(LaunchConfiguration('use_serial_bridge')),
        ),
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
            'max_vx_mps': 0.22,
            'max_wz_radps': 0.8,
            'max_ax_mps2': 0.8,
            'max_aw_radps2': 1.2,
            'enforce_single_publisher': LaunchConfiguration('enforce_single_publisher'),
            'bridge_lock_file': LaunchConfiguration('bridge_lock_file'),
        }],
    )

    # New (default): ASCII Nav2 bridge — /cmd_vel_safe -> SPEED + SERVO_PWM
    # over USB serial to the L298N CLI firmware (autonexa_pico.uf2). Mutually
    # exclusive with the legacy micro-ROS path via use_serial_bridge_active.
    nav2_pico_bridge = Node(
        package='parking_system',
        executable='nav2_pico_bridge.py',
        name='nav2_pico_bridge',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_serial_bridge_active),
        parameters=[{
            'cmd_vel_topic':     LaunchConfiguration('bridge_cmd_vel_topic'),
            'manual_cmd_vel_topic': '/cmd_vel_manual',
            'serial_port':       LaunchConfiguration('pico_serial_port'),
            'serial_baud':       115200,
            'publish_rate_hz':   30.0,
            'command_timeout_s': 0.20,
            'max_vx_mps':        0.30,
            'max_wz_radps':      0.8,
            'max_ax_mps2':       0.8,
            'max_aw_radps2':     1.2,
            'min_vx_creep':      LaunchConfiguration('min_vx_creep'),
            'wheelbase_m':       0.25,
            'servo_center_us':   LaunchConfiguration('servo_center_us'),
            'servo_us_min':      LaunchConfiguration('servo_us_min'),
            'servo_us_max':      LaunchConfiguration('servo_us_max'),
            'servo_polarity':    LaunchConfiguration('servo_polarity'),
            'reverse_steer_polarity': LaunchConfiguration('reverse_steer_polarity'),
            'vx_polarity':       LaunchConfiguration('vx_polarity'),
            'max_steer_rate_radps': LaunchConfiguration('max_steer_rate_radps'),
            'auto_enable':       True,
            'bridge_lock_file':  LaunchConfiguration('serial_bridge_lock_file'),
            'dry_run':           False,
        }],
    )

    # Legacy MicroPython serial bridge — script removed in earlier cleanup; this
    # node will fail to launch if use_micropython_bridge:=true. Kept only so
    # existing launch arg compatibility doesn't break.
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
        use_serial_bridge_arg,
        serial_bridge_lock_file_arg,
        min_vx_creep_arg,
        servo_center_us_arg,
        servo_us_min_arg,
        servo_us_max_arg,
        servo_polarity_arg,
        reverse_steer_polarity_arg,
        vx_polarity_arg,
        max_steer_rate_arg,
        use_mobile_bridge_arg,
        robot_state_publisher,
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
        nav2_pico_bridge,
        pico_serial_bridge,
        mobile_bridge,
        rviz,
    ])
