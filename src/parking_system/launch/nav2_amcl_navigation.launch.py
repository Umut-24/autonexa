#!/usr/bin/env python3
"""
Nav2 + AMCL launch (pre-saved map, LiDAR-only)

What this provides:
- Load a previously saved occupancy grid via nav2_map_server
- Localize against it with nav2_amcl (particle filter, map -> odom)
- Full Nav2 stack (planner / controller / smoother / behaviors / BT /
  waypoint follower / velocity smoother / collision monitor)
- Same scan filter chain, Pico bridge selection, mobile bridge, RViz as
  nav2_live_slam.launch.py — the only differences are SLAM Toolbox is
  swapped out for map_server + amcl, and the user supplies an initial
  pose at launch time.

Build the map first with nav2_live_slam.launch.py, then persist it via
the mobile bridge (`curl -X POST http://localhost:5000/api/lock_map`)
into ~/.autonexa/maps/garage_<ts>.{pgm,yaml}.

TF Tree: map -> odom -> base_link -> laser_link
         (amcl)  (laser_scan_matcher)  (robot_state_publisher)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

import sys as _sys
_scripts_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, 'scripts'
)
if _scripts_dir not in _sys.path:
    _sys.path.insert(0, _scripts_dir)
try:
    from parking_system.build_urdf import render as _render_urdf
except Exception:
    _render_urdf = None


def _persisted_use_ekf() -> str:
    """Default for `use_ekf`, read from the app-written ~/.autonexa/use_ekf.txt
    flag ('true'/'false', default 'false'). Toggle via /api/ekf_mode + relaunch."""
    try:
        path = os.path.expanduser('~/.autonexa/use_ekf.txt')
        with open(path, 'r', encoding='utf-8') as fh:
            on = fh.read().strip().lower() in ('1', 'true', 'yes', 'on')
        return 'true' if on else 'false'
    except OSError:
        return 'false'


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
    laser_scan_matcher_params_file = PathJoinSubstitution([pkg_dir, 'config', 'laser_scan_matcher.yaml'])
    rviz_config = PathJoinSubstitution([pkg_dir, 'rviz', 'navigation.rviz'])

    # Default map path matches what /api/lock_map writes (~/.autonexa/maps).
    # The endpoint timestamps each save (garage_<YYYYMMDD_HHMMSS>.yaml) so users
    # pass an explicit map_yaml:= for production; this default lets a quick
    # "ros2 launch ..." pick up a manually placed garage.yaml.
    default_map_yaml = os.path.join(
        os.path.expanduser('~'), '.autonexa', 'maps', 'garage.yaml'
    )

    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    serial_baudrate_arg = DeclareLaunchArgument('serial_baudrate', default_value='460800')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    controller_arg = DeclareLaunchArgument(
        'controller', default_value='mppi',
        description="Local controller plugin: 'mppi' (default, obstacle-aware "
                    "sampling MPC) or 'rpp' (Regulated Pure Pursuit fallback). "
                    "Switchable at launch with no rebuild.")
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Absolute path to the .yaml saved by /api/lock_map (map_server input).',
    )
    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='0.0')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    initial_pose_yaw_arg = DeclareLaunchArgument('initial_pose_yaw', default_value='0.0')

    use_pico_bridge_arg = DeclareLaunchArgument('use_pico_bridge', default_value='true')
    pico_serial_port_arg = DeclareLaunchArgument('pico_serial_port', default_value='/dev/ttyACM0')
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
        default_value='0.10',
        description='|vx| below this -> SPEEDS 0 0 in the ASCII bridge. Matches live-SLAM and lets '
                    'the Pico start-assist kick engage before the old 0.15 gate would pass motion.'
    )
    servo_center_us_arg = DeclareLaunchArgument('servo_center_us', default_value='1650')
    servo_us_min_arg = DeclareLaunchArgument('servo_us_min', default_value='1150')
    servo_us_max_arg = DeclareLaunchArgument('servo_us_max', default_value='2150')
    servo_polarity_arg = DeclareLaunchArgument('servo_polarity', default_value='+1')
    reverse_steer_polarity_arg = DeclareLaunchArgument(
        'reverse_steer_polarity',
        default_value='-1',
        description='Flip steering sign only while reversing. -1 matches this chassis reverse maneuvering.')
    vx_polarity_arg = DeclareLaunchArgument('vx_polarity', default_value='1')
    max_steer_rate_arg = DeclareLaunchArgument(
        'max_steer_rate_radps', default_value='3.0',
        description='Servo slew-rate cap (rad/s). Smooths Nav2 wz step changes.')
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value=_persisted_use_ekf(),
        description='Fuse wheel odom (/pico/odom) + scan-match odom via a '
                    'robot_localization EKF that owns odom->base_link TF. '
                    'Default from ~/.autonexa/use_ekf.txt (app toggle); false = '
                    'scan-matcher-owned TF. Relaunch to change.')

    _footprint_overrides = {}
    if _render_urdf is not None:
        _footprint_overrides = {
            'global_costmap.global_costmap.ros__parameters.footprint': _rendered_footprint,
            'global_costmap.global_costmap.ros__parameters.footprint_padding': str(_rendered_dims['footprint_padding']),
            'local_costmap.local_costmap.ros__parameters.footprint': _rendered_footprint,
            'local_costmap.local_costmap.ros__parameters.footprint_padding': str(_rendered_dims['footprint_padding']),
        }

    _bt_xml_path = os.path.join(pkg_dir, 'config', 'bt_navigate_to_pose_ackermann.xml')

    # AMCL initial-pose rewrites are pushed into the same Nav2 YAML so the
    # parameter source stays single. set_initial_pose=true + initial_pose.*
    # means amcl seeds at our coords instead of waiting for /initialpose.
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.keepout_filter.enabled': 'false',
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': '/scan',
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': '/scan',
            'collision_monitor.ros__parameters.scan.topic': '/scan',
            # "Unsafe navigation" (operator choice): disable collision_monitor's
            # stop/approach polygons so AUTO nav drives to the goal without
            # halting near walls (behaves like the manual OFF/bypass chain).
            # collision_monitor still republishes cmd_vel_smoothed -> cmd_vel_safe
            # unchanged. Bridge clamps + 200 ms watchdog + E-STOP remain. Flip
            # these back to 'true' to restore wall-stop safety.
            'collision_monitor.ros__parameters.DirectionalStop.enabled': 'false',
            'collision_monitor.ros__parameters.FootprintApproach.enabled': 'false',
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': _bt_xml_path,
            'amcl.ros__parameters.scan_topic': '/scan',
            'amcl.ros__parameters.set_initial_pose': 'true',
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('initial_pose_x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('initial_pose_y'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('initial_pose_yaw'),
            **_footprint_overrides,
        },
        convert_types=True,
    )

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

    # Scan filtering removed 2026-05-20: SLAM/AMCL run on the driver's raw
    # /scan (the map was built on raw /scan too, so this stays consistent).
    # laser_scan_matcher still owns odom -> base_link (AMCL only publishes
    # map -> odom, so the two TF producers don't fight) — UNLESS use_ekf is
    # on, in which case the EKF owns odom -> base_link and the matcher feeds
    # /odom_icp_raw through odom_nan_filter before EKF fusion.
    laser_scan_matcher = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_ekf')),
        parameters=[
            laser_scan_matcher_params_file,
            {'use_sim_time': False},
        ],
        remappings=[('scan', '/scan')]
    )

    laser_scan_matcher_ekf = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[
            laser_scan_matcher_params_file,
            # Override the yaml: don't own TF, publish raw ICP odom for the
            # NaN filter to clean before EKF fusion.
            {'use_sim_time': False, 'publish_tf': False, 'publish_odom': '/odom_icp_raw'},
        ],
        remappings=[('scan', '/scan')]
    )

    odom_nan_filter = Node(
        package='parking_system',
        executable='odom_nan_filter.py',
        name='odom_nan_filter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[{
            'input_topic': '/odom_icp_raw',
            'output_topic': '/odom_icp',
            'use_sim_time': False,
        }],
    )

    ekf_params_file = PathJoinSubstitution([pkg_dir, 'config', 'ekf_2d_no_imu.yaml'])
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[ekf_params_file, {'publish_tf': True, 'use_sim_time': False}],
        remappings=[('odometry/filtered', '/odom')],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map_yaml'),
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_nav2_params]
    )

    # Switchable local controller. Exactly one of these launches (mutually
    # exclusive via the 'controller' arg). Both keep name='controller_server'
    # so the single lifecycle_manager node_names list needs no change.
    #   - RPP: uses the FollowPath block in nav2_navigation_params.yaml as-is.
    #   - MPPI: layers controller_mppi.yaml AFTER the base so its
    #     controller_server block overrides the RPP one (the base file's
    #     RPP FollowPath.* keys remain but MPPI ignores undeclared params).
    use_mppi = EqualsSubstitution(LaunchConfiguration('controller'), 'mppi')
    use_rpp = EqualsSubstitution(LaunchConfiguration('controller'), 'rpp')
    mppi_params_file = PathJoinSubstitution([pkg_dir, 'config', 'controller_mppi.yaml'])

    controller_server_rpp = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        condition=IfCondition(use_rpp),
        parameters=[configured_nav2_params]
    )

    controller_server_mppi = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        condition=IfCondition(use_mppi),
        parameters=[configured_nav2_params, mppi_params_file]
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

    # Single lifecycle manager owns map_server + amcl + the Nav2 servers.
    # Order matters: map_server before amcl so the map is published when
    # amcl tries to subscribe to it during configure.
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
                'collision_monitor',
            ]
        }]
    )

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
            'max_vx_mps':        0.25,
            'max_wz_radps':      0.8,
            'max_ax_mps2':       0.60,
            'max_aw_radps2':     0.50,
            'min_vx_creep':      LaunchConfiguration('min_vx_creep'),
            'wheelbase_m':       0.25,
            'track_width_m':     0.20,
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
        controller_arg,
        map_yaml_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
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
        use_ekf_arg,
        use_mobile_bridge_arg,
        robot_state_publisher,
        lidar,
        laser_scan_matcher,
        laser_scan_matcher_ekf,
        odom_nan_filter,
        ekf_node,
        map_server,
        amcl,
        planner_server,
        controller_server_rpp,
        controller_server_mppi,
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
