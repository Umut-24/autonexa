#!/usr/bin/env python3
"""
Launch file for SLAM mapping with laser-based odometry
GOLDEN STANDARD TF TREE:
    map (SLAM Toolbox) -> odom (Laser Scan Matcher) -> base_link (Robot Center) -> laser_link (Sensor)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to launch LIDAR driver'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for LIDAR'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='460800',
        description='Serial baudrate for LIDAR'
    )
    
    # Configuration file paths
    slam_config_path = PathJoinSubstitution([
        pkg_dir,
        'config',
        'slam_toolbox_mapping_with_odom.yaml'
    ])
    
    scan_filter_config_path = PathJoinSubstitution([
        pkg_dir,
        'config',
        'scan_filter.yaml'
    ])
    
    laser_odom_config_path = PathJoinSubstitution([
        pkg_dir,
        'config',
        'laser_scan_matcher.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        pkg_dir,
        'rviz',
        'mapping.rviz'
    ])
    
    # Robot description via robot_state_publisher
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
    
    # ============================================
    # TF TREE LINK #3: base_link -> laser_link
    # ============================================
    # NOTE: This transform is already defined in the URDF (robot.urdf)
    # The robot_state_publisher will automatically publish base_link -> laser_link
    # from the URDF joint definition (xyz="0.15 0.0 0.12")
    # NO STATIC TRANSFORM NEEDED - robot_state_publisher handles it
    
    # ============================================
    # LIDAR DRIVER
    # ============================================
    # Publishes /scan topic with frame_id='laser_link'
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'frame_id': 'laser_link',  # Must match static transform child frame
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Standard',
        }],
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # ============================================
    # SCAN FILTER (Optional - improves scan quality)
    # ============================================
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[scan_filter_config_path],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered'),
        ],
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # ============================================
    # TF TREE LINK #2: odom -> base_link
    # ============================================
    # CRITICAL: Laser Scan Matcher calculates movement and publishes odom->base_link
    # This is the "odometry source" - calculates how much base_link moved
    # MUST receive /scan topic and find base_link->laser_link transform to initialize
    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[
            laser_odom_config_path,  # Load config file first
            {
                # CRITICAL PARAMETERS - Set explicitly to ensure TF publishing
                'use_sim_time': False,
                'publish_tf': True,  # MANDATORY: Must publish odom->base_link transform
                'publish_odom': '/odom',  # Topic name for odometry (string, not boolean)
                'publish_pose': True,  # Publish pose for debugging
                # Frame definitions - explicit to avoid defaults
                'base_frame': 'base_link',  # Robot center frame
                'odom_frame': 'odom',  # Odometry frame (fixed frame)
                'laser_frame': 'laser_link',  # Must match LIDAR frame_id from URDF
            }
        ],
        remappings=[
            ('scan', '/scan'),  # CRITICAL: Subscribe to /scan topic (raw LIDAR data)
        ],
        condition=IfCondition(LaunchConfiguration('use_lidar'))
    )
    
    # ============================================
    # TF TREE LINK #1: map -> odom
    # ============================================
    # SLAM Toolbox corrects drift and publishes map->odom
    # Uses external odometry from laser_scan_matcher
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path],  # Config should have base_frame='base_link', odom_frame='odom'
        remappings=[
            ('/scan', '/scan_filtered'),  # Use filtered scan for better quality
        ]
    )
    
    # ============================================
    # SLAM LIFECYCLE MANAGER
    # ============================================
    slam_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='slam_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    )
    
    # ============================================
    # MAP BOOTSTRAP (Temporary until SLAM starts)
    # ============================================
    # Publishes temporary map->odom until SLAM Toolbox takes over
    map_bootstrap_node = Node(
        package='parking_system',
        executable='map_bootstrap.py',
        name='map_bootstrap',
        output='screen'
    )
    
    # ============================================
    # SLAM INITIALIZER
    # ============================================
    # Automatically sets initial pose for SLAM
    slam_initializer = ExecuteProcess(
        cmd=['ros2', 'run', 'parking_system', 'slam_initializer.py'],
        name='slam_initializer',
        output='screen'
    )
    
    # ============================================
    # RVIZ VISUALIZATION
    # ============================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # ============================================
    # LAUNCH DESCRIPTION
    # ============================================
    # CRITICAL LAUNCH ORDER:
    # 1. Static transforms FIRST (foundation: base_link -> laser_link)
    # 2. Robot description
    # 3. Sensors (LIDAR publishes /scan)
    # 4. Odometry (Laser Scan Matcher publishes odom -> base_link)
    # 5. SLAM (SLAM Toolbox publishes map -> odom)
    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_lidar_arg,
        serial_port_arg,
        serial_baudrate_arg,
        
        # Robot description (publishes base_link -> laser_link from URDF)
        robot_state_publisher,  # Publishes base_link -> laser_link (LINK #3) from URDF
        
        # Temporary bootstrap (until SLAM starts)
        map_bootstrap_node,  # Temporary map -> odom
        
        # Sensors
        lidar_node,  # Publishes /scan with frame_id='laser_link'
        scan_filter_node,  # Optional: filters /scan -> /scan_filtered
        
        # Odometry (calculates movement)
        laser_scan_matcher_node,  # odom -> base_link (LINK #2) - CRITICAL!
        
        # SLAM (corrects drift)
        slam_toolbox_node,  # map -> odom (LINK #1)
        slam_lifecycle_manager,  # Auto-activate SLAM
        
        # Initialization
        slam_initializer,  # Auto-set initial pose
        
        # Visualization
        rviz_node,
    ])
