#!/usr/bin/env python3
"""
Minimal, safest Nav2 bringup for this project (no ArUco nodes).

Keeps the existing TF logic used by mapping/localization:
  map -> odom  (AMCL, with a temporary bootstrap identity)
  odom -> base_link (laser_scan_matcher)
  base_link -> laser_link (static)

This is intended for your 2x2m testbed where you want to:
  1) Load a saved map
  2) Localize with AMCL
  3) Click a goal in RViz and see the planned path (and optionally execute)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = FindPackageShare("parking_system").find("parking_system")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz",
    )

    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="/home/autonexa/intelligent_parking_ws/maps/parking_map.yaml",
        description="Absolute path to the map yaml file",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Whether to launch the LIDAR driver and laser odometry",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Slamtec C1",
    )

    serial_baudrate_arg = DeclareLaunchArgument(
        "serial_baudrate",
        default_value="460800",
        description="Serial baudrate for Slamtec C1",
    )

    nav2_params_path = PathJoinSubstitution([pkg_dir, "config", "nav2_params.yaml"])
    laser_odom_config_path = PathJoinSubstitution([pkg_dir, "config", "laser_scan_matcher.yaml"])
    rviz_config_path = PathJoinSubstitution([pkg_dir, "rviz", "navigation.rviz"])

    static_tf_base_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser_link"],
        parameters=[{"use_sim_time": False}],
    )

    map_bootstrap_node = Node(
        package="parking_system",
        executable="map_bootstrap.py",
        name="map_bootstrap",
        output="screen",
    )

    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": LaunchConfiguration("serial_port"),
                "serial_baudrate": LaunchConfiguration("serial_baudrate"),
                "frame_id": "laser_link",
                "inverted": "false",
                "angle_compensate": "true",
                "scan_mode": "Standard",
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_lidar")),
    )

    laser_scan_matcher_node = Node(
        package="ros2_laser_scan_matcher",
        executable="laser_scan_matcher",
        name="laser_scan_matcher",
        output="screen",
        parameters=[
            laser_odom_config_path,
            {
                "use_sim_time": False,
                "publish_tf": True,
                "publish_odom": "/odom",
                "base_frame": "base_link",
                "odom_frame": "odom",
                "laser_frame": "laser_link",
            },
        ],
        remappings=[("scan", "/scan")],
        condition=IfCondition(LaunchConfiguration("use_lidar")),
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params_path,
            {
                "use_sim_time": False,
                "yaml_filename": LaunchConfiguration("map_file"),
            },
        ],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params_path],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_path],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_path],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params_path],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params_path],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "smoother_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ]
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            map_file_arg,
            use_lidar_arg,
            serial_port_arg,
            serial_baudrate_arg,
            static_tf_base_to_laser,
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
            rviz_node,
        ]
    )

