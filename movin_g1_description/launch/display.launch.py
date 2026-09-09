"""
Publish the Unitree G1 robot description (and optionally show it in RViz).

Starts ``robot_state_publisher`` inside ``namespace`` (default ``movin``) so that it
subscribes to ``/movin/joint_states`` published by ``movin_retarget`` and publishes
``/movin/robot_description`` plus the URDF link transforms on ``/tf`` / ``/tf_static``.
The URDF is read from ``urdf_file`` at launch time (no xacro), so a custom robot
description can be passed without rebuilding the package.

Included by ``movin_retarget/launch/retarget.launch.py``; standalone use::

    ros2 launch movin_g1_description display.launch.py rviz:=true
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for robot_state_publisher and the optional RViz."""
    share_dir = get_package_share_directory('movin_g1_description')
    default_urdf = os.path.join(share_dir, 'urdf', 'g1_custom_collision_29dof.urdf')
    default_rviz_config = os.path.join(share_dir, 'rviz', 'g1_mocap.rviz')

    urdf_file = LaunchConfiguration('urdf_file')
    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    declared_arguments = [
        DeclareLaunchArgument(
            'urdf_file',
            default_value=default_urdf,
            description='Absolute path of the URDF loaded into robot_description.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='movin',
            description='Namespace of robot_state_publisher (must match movin_retarget so that '
                        '<namespace>/joint_states is consumed).',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Also start rviz2 with rviz_config (requires the rviz2 package).',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz configuration file used when rviz:=true.',
        ),
    ]

    # The file content becomes the parameter value; value_type=str keeps launch from trying
    # to parse the XML as YAML.
    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declared_arguments + [robot_state_publisher_node, rviz_node])
