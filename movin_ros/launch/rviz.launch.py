"""
Start rviz2 with the MOVIN skeleton configuration.

``rviz/mocap.rviz`` uses ``world`` as the fixed frame, shows the TF tree
(the skeleton is published as TF by ``mocap_node``) and has a disabled
MarkerArray display for ``/movin/skeleton_markers`` (enable it together with
``publish_markers:=true``).  Included by ``mocap.launch.py`` when
``rviz:=true``; standalone use::

    ros2 launch movin_ros rviz.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for rviz2."""
    default_config = os.path.join(
        get_package_share_directory('movin_ros'), 'rviz', 'mocap.rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', default_value=default_config,
            description='RViz configuration file.'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
