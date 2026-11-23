"""
Start ``retarget_node`` together with the Unitree G1 ``robot_state_publisher``.

Both run inside ``namespace`` (default ``movin``) so that the node subscribes
``/movin/mocap_frame`` from ``mocap_node`` and ``robot_state_publisher``
consumes ``/movin/joint_states``.  The robot description comes from
``movin_g1_description/launch/display.launch.py`` (``urdf_file`` is read at
launch time, no xacro).  Included by ``movin_ros/launch/mocap.launch.py`` when
``robot:=unitree_g1`` or ``robot:=unitree_g1_with_hands``; standalone use::

    ros2 launch movin_retarget retarget.launch.py forward_mode:=isaac_world rviz:=true

``forward_mode``, ``world_frame`` and ``frame_qos_reliable`` must match
``mocap_node``; ``robot_offset`` is the x offset of the robot root in meters
(the node parameter of the same name is the full ``[x, 0, 0]`` translation).
"""

from __future__ import annotations

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for retarget_node and the G1 description."""
    g1_share = get_package_share_directory('movin_g1_description')
    default_urdf = os.path.join(g1_share, 'urdf', 'g1_custom_collision_29dof.urdf')
    default_rviz_config = os.path.join(g1_share, 'rviz', 'g1_mocap.rviz')

    robot = LaunchConfiguration('robot')
    human_height = LaunchConfiguration('human_height')
    source_preset = LaunchConfiguration('source_preset')
    forward_mode = LaunchConfiguration('forward_mode')
    world_frame = LaunchConfiguration('world_frame')
    robot_root_frame = LaunchConfiguration('robot_root_frame')
    robot_offset = LaunchConfiguration('robot_offset')
    offset_to_ground = LaunchConfiguration('offset_to_ground')
    max_rate_hz = LaunchConfiguration('max_rate_hz')
    actor = LaunchConfiguration('actor')
    frame_qos_reliable = LaunchConfiguration('frame_qos_reliable')
    urdf_file = LaunchConfiguration('urdf_file')
    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    log_level = LaunchConfiguration('log_level')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot', default_value='unitree_g1',
            description='Retargeter robot type: unitree_g1 or unitree_g1_with_hands.'),
        DeclareLaunchArgument(
            'human_height', default_value='auto',
            description="Body size: auto (measured from the streamed bones) or the actor's "
                        'standing height in meters (body scale = 1.8 / human_height).'),
        DeclareLaunchArgument(
            'source_preset', default_value='auto',
            description='Source skeleton preset: auto (from the message), movinman, movinman_v3.'),
        DeclareLaunchArgument(
            'forward_mode', default_value='isaac_world',
            description='coord_equivalent or isaac_world; must match mocap_node.'),
        DeclareLaunchArgument(
            'world_frame', default_value='world',
            description='Parent frame of the robot root transform (same as mocap_node).'),
        DeclareLaunchArgument(
            'robot_root_frame', default_value='pelvis',
            description='Child frame of the root transform (URDF root link).'),
        DeclareLaunchArgument(
            'robot_offset', default_value='0.0',
            description='X offset of the robot root in world_frame, meters.'),
        DeclareLaunchArgument(
            'offset_to_ground', default_value='false',
            description='Shift the skeleton so the lowest foot touches the ground.'),
        DeclareLaunchArgument(
            'max_rate_hz', default_value='60.0',
            description='Upper bound of the average retargeting rate; 0 = every frame.'),
        DeclareLaunchArgument(
            'actor', default_value='',
            description='Actor name to retarget; empty = first actor seen.'),
        DeclareLaunchArgument(
            'frame_qos_reliable', default_value='true',
            description='Subscribe mocap_frame RELIABLE (true) or BEST_EFFORT (false); must '
                        'match the mocap_node publisher.'),
        DeclareLaunchArgument(
            'urdf_file', default_value=default_urdf,
            description='Absolute path of the URDF loaded into robot_description.'),
        DeclareLaunchArgument(
            'namespace', default_value='movin',
            description='Namespace of retarget_node and robot_state_publisher.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also start rviz2 with rviz_config (robot model + skeleton TF).'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='RViz configuration used when rviz:=true.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='rclpy log level of retarget_node.'),
    ]

    # The node parameter is a double[3]; the launch argument is the x component so that
    # mocap.launch.py can forward a single number.  float() keeps yaml from producing an int.
    robot_offset_xyz = ParameterValue(
        PythonExpression(['[float(', robot_offset, '), 0.0, 0.0]']),
        value_type=List[float],
    )

    retarget_node = Node(
        package='movin_retarget',
        executable='retarget_node',
        name='retarget_node',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot': ParameterValue(robot, value_type=str),
            # 'auto' or meters; passed as a string, retarget_node parses it (see its description).
            'human_height': ParameterValue(human_height, value_type=str),
            'source_preset': ParameterValue(source_preset, value_type=str),
            'forward_mode': ParameterValue(forward_mode, value_type=str),
            'world_frame': ParameterValue(world_frame, value_type=str),
            'robot_root_frame': ParameterValue(robot_root_frame, value_type=str),
            'robot_offset': robot_offset_xyz,
            'offset_to_ground': ParameterValue(offset_to_ground, value_type=bool),
            'max_rate_hz': ParameterValue(max_rate_hz, value_type=float),
            'actor': ParameterValue(actor, value_type=str),
            'frame_qos_reliable': ParameterValue(frame_qos_reliable, value_type=bool),
        }],
        arguments=['--ros-args', '--log-level', log_level],
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g1_share, 'launch', 'display.launch.py')),
        launch_arguments={
            'urdf_file': urdf_file,
            'namespace': namespace,
            'rviz': rviz,
            'rviz_config': rviz_config,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [retarget_node, robot_description])
