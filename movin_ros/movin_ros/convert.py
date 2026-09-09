"""
Pure-Python conversion of MOVIN Studio bones to ROS conventions.

MOVIN Studio streams every bone as a local transform in Unity's left-handed,
Y-up frame: ``p`` is the joint offset in the parent's frame and ``q`` (SDK
order ``(w, x, y, z)``) is the full local rotation.  This module decomposes
the SDK's ``compute_forward_kinematics`` so that tf2 can do the composition:

1. :func:`mirror_local` mirrors each local transform through the X axis
   (``diag(-1, 1, 1)`` change of basis, identical to the SDK's
   ``unity_to_opengl_vec`` / ``unity_to_opengl_quat``) and publishes it as a
   parent->child TF edge.
2. One static anchor rotation per actor (:data:`ANCHOR_QUAT_XYZW`) maps the
   mirrored, still Y-up tree into a Z-up ROS world.  ``coord_equivalent`` is
   the SDK/Isaac default (Unity forward +Z -> ROS -Y); ``isaac_world`` adds a
   +90 degree yaw so Unity forward faces ROS +X (REP-103).

Composing the edges under the anchor reproduces the SDK's forward kinematics
to floating-point precision, provided bones are streamed parents-first
(``parent_index < bone_index``) with unique names.  Quaternions in this module
are ``(x, y, z, w)`` unless a name says ``wxyz``.  Only the standard library
is used: no numpy, no ROS message types.
"""

from __future__ import annotations

import math
import re
import typing
import zlib

# Static anchor rotations world_frame -> <tf_prefix>/<actor_id>, quaternion (x, y, z, w).
ANCHOR_QUAT_XYZW = {
    # R_x(+90 deg): Unity +Z (forward) -> ROS -Y, Unity +Y (up) -> ROS +Z.
    'coord_equivalent': (0.7071067811865476, 0.0, 0.0, 0.7071067811865476),
    # R_z(+90 deg) * R_x(+90 deg): Unity +Z -> ROS +X, actor's left -> ROS +Y (REP-103).
    'isaac_world': (0.5, 0.5, 0.5, 0.5),
}
FORWARD_MODES = ('coord_equivalent', 'isaac_world')

# +90 degree yaw about Z (SDK isaac_lab_utils.Q_ISAAC_WORLD_FORWARD), both orders.
YAW_QUAT_WXYZ = (0.7071067811865476, 0.0, 0.0, 0.7071067811865476)
YAW_QUAT_XYZW = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)

IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


class Edge(typing.NamedTuple):
    """
    One parent->child TF edge derived from a streamed bone.

    ``parent_name`` is ``None`` for bones that hang directly under the actor
    anchor (``parent_index < 0`` or a parent that is not part of the frame).
    """

    bone_index: int
    name: str
    parent_name: str | None
    t_xyz: Vec3
    q_xyzw: Quat


# ---------------------------------------------------------------------------
# quaternion helpers (x, y, z, w)
# ---------------------------------------------------------------------------

def quat_normalize_xyzw(q) -> Quat:
    """Return ``q`` scaled to unit length; identity when the norm is ~0 (SDK behaviour)."""
    x, y, z, w = (float(c) for c in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8:
        return IDENTITY_XYZW
    return (x / norm, y / norm, z / norm, w / norm)


def quat_mul_xyzw(a, b) -> Quat:
    """Return the Hamilton product ``a * b`` (apply ``b`` first, then ``a``)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate_xyzw(q, v) -> Vec3:
    """Rotate vector ``v`` by the unit quaternion ``q`` (Rodrigues form, as in the SDK)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_conjugate_xyzw(q) -> Quat:
    """Return the conjugate (inverse for unit quaternions) of ``q``."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def anchor_quat(forward_mode: str) -> Quat:
    """Return the static anchor quaternion (x, y, z, w) for ``forward_mode``."""
    try:
        return ANCHOR_QUAT_XYZW[forward_mode]
    except KeyError:
        raise ValueError(
            f'unknown forward_mode {forward_mode!r}; expected one of {FORWARD_MODES}'
        ) from None


# ---------------------------------------------------------------------------
# per-bone conversion
# ---------------------------------------------------------------------------

def mirror_local(p, q_wxyz) -> tuple[Vec3, Quat]:
    """
    Convert a Unity left-handed local transform to a ROS-convention TF edge.

    ``t = (-p[0], p[1], p[2])`` and ``q_xyzw = normalize((q[1], -q[2], -q[3], q[0]))``,
    i.e. the SDK's ``unity_to_opengl_vec`` / ``unity_to_opengl_quat`` followed by
    the (w, x, y, z) -> (x, y, z, w) reorder.  The result is a proper rotation
    (det +1), so it can be published as a single ``TransformStamped``.
    """
    t = (-float(p[0]), float(p[1]), float(p[2]))
    q = quat_normalize_xyzw((q_wxyz[1], -q_wxyz[2], -q_wxyz[3], q_wxyz[0]))
    return t, q


def sanitize_actor_id(actor: str) -> str:
    """Return ``actor`` with every character outside ``[A-Za-z0-9_]`` replaced, or ``'actor0'``."""
    return re.sub(r'[^A-Za-z0-9_]', '_', actor or '') or 'actor0'


def assign_actor_id(actor: str, registry: dict) -> str:
    """
    Return a deterministic TF-safe id for ``actor`` and remember it in ``registry``.

    The id is :func:`sanitize_actor_id`; when that id is already mapped to a
    DIFFERENT actor, ``'_' + crc32(actor) & 0xffff`` (4 hex digits) is appended so
    two actors never share a TF subtree.  Ids never change once assigned.
    """
    known = registry.get(actor)
    if known is not None:
        return known
    actor_id = sanitize_actor_id(actor)
    taken = {aid: name for name, aid in registry.items()}
    if actor_id in taken and taken[actor_id] != actor:
        actor_id += '_' + format(zlib.crc32(actor.encode('utf-8')) & 0xffff, '04x')
    registry[actor] = actor_id
    return actor_id


def bone_frame(tf_prefix: str, actor_id: str, bone_name: str) -> str:
    """Return the TF frame id of one bone: ``<tf_prefix>/<actor_id>/<bone_name>``."""
    return f'{tf_prefix}/{actor_id}/{bone_name}'


def anchor_frame(tf_prefix: str, actor_id: str) -> str:
    """Return the TF frame id of the actor anchor: ``<tf_prefix>/<actor_id>``."""
    return f'{tf_prefix}/{actor_id}'


def local_edges(bones) -> list:
    """
    Convert the bones of one SDK frame into :class:`Edge` tuples (frame order kept).

    A bone's ``parent_name`` is ``None`` when ``parent_index < 0``, when the
    parent index is not present in this frame, or when a bone lists itself as
    parent; such bones hang directly under the actor anchor.  The root bone is
    found through ``parent_index`` only, never by name (``Root``/``RootBone``).
    """
    names = {}
    for bone in bones:
        names[bone['bone_index']] = bone['bone_name']
    edges = []
    for bone in bones:
        index = int(bone['bone_index'])
        parent_index = bone.get('parent_index', -1)
        parent_name = None
        if parent_index is not None and parent_index >= 0 and parent_index != index:
            parent_name = names.get(parent_index)
        t, q = mirror_local(bone['p'], bone['q'])
        edges.append(Edge(index, bone['bone_name'], parent_name, t, q))
    return edges


# ---------------------------------------------------------------------------
# composition (what tf2 does at runtime; used by tests and the retargeting node)
# ---------------------------------------------------------------------------

def forward_kinematics(edges, anchor_q_xyzw, anchor_t_xyz=(0.0, 0.0, 0.0)) -> dict:
    """
    Compose ``edges`` under the anchor and return ``{name: (t_xyz, q_xyzw)}`` in the world frame.

    This is the tf2-equivalent chain ``world -> anchor -> ... -> bone``.  Edges
    whose parent is missing (or would form a cycle) attach to the anchor.  Any
    edge order is accepted; the result equals the SDK's
    ``compute_forward_kinematics`` (left-multiplied by ``Q_ISAAC_WORLD_FORWARD``
    for the ``isaac_world`` anchor) when bones are parents-first with unique names.
    """
    by_name = {}
    for edge in edges:
        by_name[edge[1]] = edge
    anchor = (
        tuple(float(c) for c in anchor_t_xyz),
        quat_normalize_xyzw(anchor_q_xyzw),
    )
    world = {}

    def solve(name, visiting):
        if name in world:
            return world[name]
        _index, _name, parent_name, t_local, q_local = by_name[name]
        if (
            parent_name is None
            or parent_name == name
            or parent_name not in by_name
            or parent_name in visiting
        ):
            parent_t, parent_q = anchor
        else:
            parent_t, parent_q = solve(parent_name, visiting | {name})
        rotated = quat_rotate_xyzw(parent_q, t_local)
        t_world = (parent_t[0] + rotated[0], parent_t[1] + rotated[1], parent_t[2] + rotated[2])
        q_world = quat_normalize_xyzw(quat_mul_xyzw(parent_q, q_local))
        world[name] = (t_world, q_world)
        return world[name]

    for edge in edges:
        solve(edge[1], frozenset())
    return world


def cylinder_pose(p_xyz) -> tuple[Vec3, Quat, float]:
    """
    Return ``(center_xyz, q_xyzw, length)`` of a cylinder from the origin to ``p``.

    RViz cylinders extend along their local Z axis, so ``q`` rotates +Z onto
    the direction of ``p``.  The anti-parallel case (``p`` along -Z) uses a 180
    degree rotation about X; a zero-length ``p`` yields the identity and length 0.
    """
    x, y, z = (float(c) for c in p_xyz)
    length = math.sqrt(x * x + y * y + z * z)
    center = (0.5 * x, 0.5 * y, 0.5 * z)
    if length < 1e-12:
        return center, IDENTITY_XYZW, 0.0
    dx, dy, dz = x / length, y / length, z / length
    if 1.0 + dz < 1e-9:
        return center, (1.0, 0.0, 0.0, 0.0), length
    # Shortest rotation from a=(0,0,1) to d: q = (a x d, 1 + a.d), normalized.
    return center, quat_normalize_xyzw((-dy, dx, 0.0, 1.0 + dz)), length


def yaw_qpos(qpos_root_pos_xyz, qpos_root_quat_wxyz, forward_mode, offset_xyz=(0.0, 0.0, 0.0)):
    """
    Convert the Retargeter root pose (``qpos[:3]``, ``qpos[3:7]`` wxyz) to a world pose.

    The Retargeter works in the ``coord_equivalent`` world.  For ``isaac_world``
    the same +90 degree yaw as the skeleton anchor is applied
    (``pos = R(qY) * pos``, ``quat = qY * quat``) so robot and skeleton face the
    same way; ``offset_xyz`` is then added.  Returns ``(pos_xyz, quat_xyzw)``.
    """
    if forward_mode not in FORWARD_MODES:
        raise ValueError(
            f'unknown forward_mode {forward_mode!r}; expected one of {FORWARD_MODES}'
        )
    pos = tuple(float(c) for c in qpos_root_pos_xyz)
    w, x, y, z = (float(c) for c in qpos_root_quat_wxyz)
    quat = quat_normalize_xyzw((x, y, z, w))
    if forward_mode == 'isaac_world':
        pos = quat_rotate_xyzw(YAW_QUAT_XYZW, pos)
        quat = quat_normalize_xyzw(quat_mul_xyzw(YAW_QUAT_XYZW, quat))
    pos = (
        pos[0] + float(offset_xyz[0]),
        pos[1] + float(offset_xyz[1]),
        pos[2] + float(offset_xyz[2]),
    )
    return pos, quat
