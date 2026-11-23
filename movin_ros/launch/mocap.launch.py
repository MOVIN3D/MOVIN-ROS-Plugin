"""
Single entry point for MOVIN Studio motion capture in ROS 2.

Starts ``mocap_node`` (package ``movin_ros``) inside ``namespace`` (default
``movin``) with the defaults from ``config/mocap_node.yaml`` overridden by the
launch arguments below, and optionally RViz (``rviz:=true``), the Unitree G1
retargeting pipeline (``robot:=unitree_g1``, package ``movin_retarget``) and a
``ros2 bag`` recorder (``bag:=true bag_dir:=...``).  Examples::

    ros2 launch movin_ros mocap.launch.py                                 # receive only
    ros2 launch movin_ros mocap.launch.py host:=192.168.0.25 port:=12000  # Studio -> this IP:port
    ros2 launch movin_ros mocap.launch.py record:=auto                    # record from packet one
    ros2 launch movin_ros mocap.launch.py mode:=replay recording:=take.pkl replay_loop:=false
    ros2 launch movin_ros mocap.launch.py robot:=unitree_g1 rviz:=true    # needs [retargeting]

Every substitution parameter is wrapped in ``ParameterValue(..., value_type=...)``
so that ``port:=12000`` stays an int, ``record_segment_sec:=60`` becomes a
double and paths or actor names are never re-parsed as YAML numbers/booleans.
"""

from __future__ import annotations

import os

from ament_index_python import packages as ament_packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration
from launch.substitutions import NotEqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

BAG_TOPICS = ('mocap_frame', 'status')


def _default_urdf() -> str:
    """Return the default G1 URDF path, or '' when movin_g1_description is not installed."""
    try:
        share = ament_packages.get_package_share_directory('movin_g1_description')
    except ament_packages.PackageNotFoundError:
        return ''
    return os.path.join(share, 'urdf', 'g1_custom_collision_29dof.urdf')


def _declare_arguments(params_file_default: str, urdf_default: str) -> list:
    """Return the DeclareLaunchArgument actions (name, default, description)."""
    args = [
        ('mode', 'live', 'live (UDP from MOVIN Studio) or replay (SDK .pkl recording).'),
        ('host', '0.0.0.0', 'LOCAL bind address of the UDP socket (not the Studio IP).'),
        ('port', '11235', 'UDP port MOVIN Studio streams to; 0 = ephemeral port.'),
        ('recording', '', 'Replay input: SDK raw-OSC .pkl recording (mode:=replay).'),
        ('replay_loop', 'true', 'Restart the recording when it ends.'),
        ('replay_exit_on_end', 'false',
         'Exit with code 0 once a non-looping replay has finished.'),
        ('record', '',
         'Recording: "" = off, auto = auto-named file in record_dir, else a .pkl path.'),
        ('record_dir', '~/movin_recordings', 'Directory of auto-named recordings.'),
        ('record_segment_sec', '600.0',
         'Lossless segment rotation period, seconds; 0 = one file.'),
        ('preset', 'auto', 'Skeleton preset: auto, movinman or movinman_v3.'),
        ('forward_mode', 'isaac_world',
         'coord_equivalent (Unity +Z -> ROS -Y) or isaac_world (Unity +Z -> ROS +X).'),
        ('world_frame', 'world', 'Parent frame of the actor anchors.'),
        ('tf_prefix', 'movin', 'TF prefix: <tf_prefix>/<actor_id>/<bone>.'),
        ('publish_every_frame', 'true', 'Publish every assembled frame (false = latest only).'),
        ('poll_rate_hz', '120.0', 'Receiver polling rate.'),
        ('publish_markers', 'false', 'Publish skeleton_markers (MarkerArray) for RViz.'),
        ('frame_qos_reliable', 'true',
         'mocap_frame QoS of mocap_node AND retarget_node: true = RELIABLE, false = best effort.'),
        ('robot', 'none', 'Retargeting target: none, unitree_g1 or unitree_g1_with_hands.'),
        ('human_height', 'auto',
         "Retargeting body size: auto (measured from the stream) or the actor's height (m)."),
        ('robot_offset', '0.0',
         'X offset of the robot root in world_frame, meters (retargeting).'),
        ('urdf_file', urdf_default, 'URDF loaded by robot_state_publisher (retargeting).'),
        ('rviz', 'false', 'Also start rviz2: rviz/mocap.rviz (TF skeleton), or with robot:=... '
         'movin_g1_description/rviz/g1_mocap.rviz (robot model + skeleton).'),
        ('bag', 'false', 'Also run ros2 bag record on the mocap topics and TF.'),
        ('bag_dir', '', 'Output directory of the bag ("" = rosbag2 default name).'),
        ('max_frames', '0', 'Exit 0 after publishing this many frames (0 = run forever).'),
        ('namespace', 'movin', 'Namespace of mocap_node (topics become /<namespace>/...).'),
        ('params_file', params_file_default, 'YAML file with the node defaults.'),
        ('log_level', 'info', 'rclpy log level of mocap_node.'),
    ]
    return [DeclareLaunchArgument(name, default_value=default, description=description)
            for name, default, description in args]


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for mocap_node and its optional companions."""
    share = ament_packages.get_package_share_directory('movin_ros')
    params_file_default = os.path.join(share, 'config', 'mocap_node.yaml')

    def cfg(name):
        return LaunchConfiguration(name)

    def typed(name, value_type):
        return ParameterValue(LaunchConfiguration(name), value_type=value_type)

    namespace = cfg('namespace')
    robot = cfg('robot')
    bag_dir = cfg('bag_dir')

    mocap_node = Node(
        package='movin_ros',
        executable='mocap_node',
        name='mocap_node',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[
            cfg('params_file'),
            {
                'mode': typed('mode', str),
                'host': typed('host', str),
                'port': typed('port', int),
                'recording': typed('recording', str),
                'replay_loop': typed('replay_loop', bool),
                'replay_exit_on_end': typed('replay_exit_on_end', bool),
                'record': typed('record', str),
                'record_dir': typed('record_dir', str),
                'record_segment_sec': typed('record_segment_sec', float),
                'preset': typed('preset', str),
                'forward_mode': typed('forward_mode', str),
                'world_frame': typed('world_frame', str),
                'tf_prefix': typed('tf_prefix', str),
                'publish_every_frame': typed('publish_every_frame', bool),
                'poll_rate_hz': typed('poll_rate_hz', float),
                'publish_markers': typed('publish_markers', bool),
                'frame_qos_reliable': typed('frame_qos_reliable', bool),
                'max_frames': typed('max_frames', int),
            },
        ],
        arguments=['--ros-args', '--log-level', cfg('log_level')],
    )

    # Skeleton-only RViz config; with a robot the retarget launch starts RViz with the
    # movin_g1_description config (RobotModel + TF) instead, so the robot mesh is visible.
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rviz.launch.py')),
        condition=IfCondition(AndSubstitution(cfg('rviz'), EqualsSubstitution(robot, 'none'))),
    )

    # Resolved lazily (FindPackageShare) so a workspace without movin_retarget still launches.
    retarget = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('movin_retarget'), '/launch/retarget.launch.py']),
        condition=IfCondition(NotEqualsSubstitution(robot, 'none')),
        launch_arguments={
            'robot': robot,
            'human_height': cfg('human_height'),
            'robot_offset': cfg('robot_offset'),
            'forward_mode': cfg('forward_mode'),
            'world_frame': cfg('world_frame'),
            'urdf_file': cfg('urdf_file'),
            'namespace': namespace,
            'frame_qos_reliable': cfg('frame_qos_reliable'),
            'rviz': cfg('rviz'),
        }.items(),
    )

    bag_topics = [['/', namespace, '/', topic] for topic in BAG_TOPICS] + ['/tf', '/tf_static']
    bag = GroupAction(
        condition=IfCondition(cfg('bag')),
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', bag_dir] + bag_topics,
                output='screen',
                condition=IfCondition(NotEqualsSubstitution(bag_dir, '')),
            ),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record'] + bag_topics,
                output='screen',
                condition=IfCondition(EqualsSubstitution(bag_dir, '')),
            ),
        ],
    )

    return LaunchDescription(
        _declare_arguments(params_file_default, _default_urdf())
        + [mocap_node, rviz, retarget, bag]
    )
