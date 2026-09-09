"""
RViz markers for a MOVIN skeleton (spheres at joints, cylinders along bones).

Every marker is expressed in the TF frames published by ``mocap_node``, so
RViz composes the poses itself and the module stays numpy-free: a joint
sphere sits at the origin of its bone frame and a bone cylinder is drawn in
the PARENT bone frame from the origin to the child's translation
(:func:`movin_ros.convert.cylinder_pose`).  Marker ids are the stable
``bone_index`` values and every marker lives 0.2 s, so a stopped stream
leaves no ghosts; when the topology changes a ``DELETEALL`` marker per
namespace clears stale ids first.
"""

from __future__ import annotations

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from movin_ros import convert
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

JOINT_COLOR = (1.0, 0.2, 0.2, 1.0)
BONE_COLOR = (1.0, 0.5, 0.0, 1.0)
LIFETIME_NANOSEC = 200_000_000  # 0.2 s
MIN_BONE_LENGTH = 1e-6


def joints_namespace(tf_prefix: str, actor_id: str) -> str:
    """Return the marker namespace of the joint spheres: ``<tf_prefix>/<actor_id>/joints``."""
    return f'{tf_prefix}/{actor_id}/joints'


def bones_namespace(tf_prefix: str, actor_id: str) -> str:
    """Return the marker namespace of the bone cylinders: ``<tf_prefix>/<actor_id>/bones``."""
    return f'{tf_prefix}/{actor_id}/bones'


def _lifetime() -> Duration:
    return Duration(sec=0, nanosec=LIFETIME_NANOSEC)


def _color(rgba) -> ColorRGBA:
    return ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))


def _delete_all(namespace: str, stamp) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = 0
    marker.action = Marker.DELETEALL
    return marker


def _sphere(frame_id: str, namespace: str, marker_id: int, stamp, radius: float) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = Pose(position=Point(x=0.0, y=0.0, z=0.0),
                       orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    diameter = 2.0 * float(radius)
    marker.scale = Vector3(x=diameter, y=diameter, z=diameter)
    marker.color = _color(JOINT_COLOR)
    marker.lifetime = _lifetime()
    return marker


def _cylinder(frame_id: str, namespace: str, marker_id: int, stamp, t_xyz, radius: float):
    center, q_xyzw, length = convert.cylinder_pose(t_xyz)
    if length < MIN_BONE_LENGTH:
        return None
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose = Pose(
        position=Point(x=center[0], y=center[1], z=center[2]),
        orientation=Quaternion(x=q_xyzw[0], y=q_xyzw[1], z=q_xyzw[2], w=q_xyzw[3]))
    diameter = 2.0 * float(radius)
    marker.scale = Vector3(x=diameter, y=diameter, z=float(length))
    marker.color = _color(BONE_COLOR)
    marker.lifetime = _lifetime()
    return marker


def build_marker_array(frame_edges, *, tf_prefix: str, actor_id: str, stamp,
                       joint_radius: float = 0.03, bone_radius: float = 0.015,
                       topology_changed: bool = False) -> MarkerArray:
    """
    Return the ``MarkerArray`` for one actor's frame.

    ``frame_edges`` are the :class:`movin_ros.convert.Edge` tuples of the frame
    (``local_edges``).  Joint spheres (``id = bone_index``) are placed at the
    origin of each bone frame; bone cylinders (``id = bone_index`` of the
    child) are drawn in the parent bone frame, or in the actor anchor frame
    for root edges, and skipped when the edge has zero length.  With
    ``topology_changed`` one ``DELETEALL`` marker per namespace is prepended.
    """
    joints_ns = joints_namespace(tf_prefix, actor_id)
    bones_ns = bones_namespace(tf_prefix, actor_id)
    anchor = convert.anchor_frame(tf_prefix, actor_id)
    array = MarkerArray()
    if topology_changed:
        array.markers.append(_delete_all(joints_ns, stamp))
        array.markers.append(_delete_all(bones_ns, stamp))
    for edge in frame_edges:
        bone_index, name, parent_name, t_xyz, _q = edge
        child_frame = convert.bone_frame(tf_prefix, actor_id, name)
        array.markers.append(_sphere(child_frame, joints_ns, bone_index, stamp, joint_radius))
        if parent_name is None:
            parent_frame = anchor
        else:
            parent_frame = convert.bone_frame(tf_prefix, actor_id, parent_name)
        cylinder = _cylinder(parent_frame, bones_ns, bone_index, stamp, t_xyz, bone_radius)
        if cylinder is not None:
            array.markers.append(cylinder)
    return array
