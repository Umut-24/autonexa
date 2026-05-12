#!/usr/bin/env python3
"""Helper launch for robot_state_publisher only.

Reads ~/.autonexa/robot_dimensions.yaml via build_urdf.render() so it
matches the live system's view of the robot.
"""

import os
import sys as _sys

from launch import LaunchDescription
from launch_ros.actions import Node

_scripts_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, 'scripts'
)
if _scripts_dir not in _sys.path:
    _sys.path.insert(0, _scripts_dir)

from parking_system.build_urdf import render as _render_urdf


def generate_launch_description():
    urdf_xml, _footprint, _dims = _render_urdf()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': urdf_xml,
        }],
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
