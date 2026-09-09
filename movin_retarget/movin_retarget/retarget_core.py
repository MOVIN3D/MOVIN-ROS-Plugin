"""
ROS-free helpers of the retargeting node.

Everything here can be exercised with plain pytest (no rclpy): rebuilding SDK
bone dicts from a ``movin_msgs/MocapFrame`` (or any duck-typed equivalent),
enumerating the hinge joints of the Retargeter's MuJoCo model in model order,
running one retargeting step and converting the resulting root pose into the
selected ``forward_mode`` world.  numpy, mujoco and ``movin_sdk_python``'s
optional ``[retargeting]`` extra are imported lazily inside the functions that
need them, so importing this module never pulls them in.

Quaternions are ``(w, x, y, z)`` when the name says ``wxyz`` (SDK order) and
``(x, y, z, w)`` when it says ``xyzw`` (ROS order).
"""

from __future__ import annotations

import functools
import math
import os
import time
from typing import NamedTuple

ROBOT_TYPES = ('unitree_g1', 'unitree_g1_with_hands')
SOURCE_PRESETS = ('auto', 'movinman', 'movinman_v3')
DEFAULT_SOURCE_PRESET = 'movinman_v3'
FORWARD_MODES = ('coord_equivalent', 'isaac_world')
# The only bone convention movin_msgs/MocapFrame carries (MocapFrame.CONVENTION_UNITY_LH_YUP).
CONVENTION_UNITY_LH_YUP = 0

PIP_HINT = 'pip install "movin_sdk_python[retargeting]"'

# +90 degree yaw about Z (SDK isaac_lab_utils.Q_ISAAC_WORLD_FORWARD) as (x, y, z, w).
_YAW_QUAT_XYZW = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)


class MissingBonesError(ValueError):
    """Raised by :func:`run_retarget` when bones required by the IK config are absent."""

    def __init__(self, missing):
        self.missing = tuple(missing)
        super().__init__(
            f'{len(self.missing)} bone(s) required by the retargeting config are missing '
            f'from the frame: {", ".join(self.missing)}'
        )


class RetargetResult(NamedTuple):
    """Split ``qpos`` of one retargeting step (plain Python numbers)."""

    root_pos: tuple
    root_quat_wxyz: tuple
    joint_positions: list


# ---------------------------------------------------------------------------
# message -> SDK bone dicts
# ---------------------------------------------------------------------------

def _field(obj, name):
    """Read ``name`` from a mapping or from an object attribute (ROS message or dict)."""
    if isinstance(obj, dict):
        return obj[name]
    return getattr(obj, name)


def _vec3(v) -> tuple:
    return (float(_field(v, 'x')), float(_field(v, 'y')), float(_field(v, 'z')))


def _quat_wxyz(q) -> tuple:
    """Return a geometry_msgs-style (x, y, z, w) quaternion as an SDK (w, x, y, z) tuple."""
    return (
        float(_field(q, 'w')), float(_field(q, 'x')), float(_field(q, 'y')), float(_field(q, 'z'))
    )


def bone_from_message(bone) -> dict:
    """
    Rebuild one SDK bone dict from a ``movin_msgs/Bone`` (or duck-typed equivalent).

    The message stores quaternions as ``(x, y, z, w)``; the SDK expects
    ``rq``/``q`` as ``(w, x, y, z)``.  Positions and scales are copied as-is,
    nothing is mirrored (the bones stay in Unity's left-handed Y-up frame, which
    is what ``Retargeter.process_mocap_frame`` consumes).
    """
    return {
        'bone_index': int(_field(bone, 'bone_index')),
        'parent_index': int(_field(bone, 'parent_index')),
        'bone_name': str(_field(bone, 'name')),
        'p': _vec3(_field(bone, 'position')),
        'rq': _quat_wxyz(_field(bone, 'rest_rotation')),
        'q': _quat_wxyz(_field(bone, 'rotation')),
        's': _vec3(_field(bone, 'scale')),
    }


def bones_from_message(msg) -> list:
    """Return the SDK bone dicts of a ``movin_msgs/MocapFrame`` in message order."""
    return [bone_from_message(bone) for bone in _field(msg, 'bones')]


def frame_from_message(msg) -> dict:
    """Rebuild the complete SDK frame dict (``timestamp``, ``actor``, ``frame_idx``, ``bones``)."""
    return {
        'timestamp': str(_field(msg, 'source_timestamp')),
        'actor': str(_field(msg, 'actor')),
        'frame_idx': int(_field(msg, 'frame_idx')),
        'bones': bones_from_message(msg),
    }


def check_convention(msg) -> None:
    """Raise ``ValueError`` unless the frame uses ``CONVENTION_UNITY_LH_YUP``."""
    convention = int(_field(msg, 'convention'))
    if convention != CONVENTION_UNITY_LH_YUP:
        raise ValueError(
            f'unsupported bone convention {convention} (expected '
            f'{CONVENTION_UNITY_LH_YUP} = CONVENTION_UNITY_LH_YUP)'
        )


# ---------------------------------------------------------------------------
# preset / dependency helpers
# ---------------------------------------------------------------------------

def detect_source_preset(bone_names) -> str:
    """Return the SDK preset name for ``bone_names`` (``''`` without names or on failure)."""
    names = list(bone_names)
    if not names:
        return ''
    try:
        from movin_sdk_python.utils.skeleton_presets import detect_preset_from_bone_names
        return detect_preset_from_bone_names(names).name
    except Exception:
        return ''


def resolve_source_preset(parameter: str, message_preset: str, bone_names=()) -> str:
    """
    Pick the Retargeter ``source_preset``.

    An explicit ``parameter`` (anything but ``'auto'``) wins; otherwise the
    ``preset`` field of the message is used, then bone-name detection, and
    finally the SDK default ``movinman_v3``.
    """
    parameter = (parameter or 'auto').strip()
    if parameter != 'auto':
        return parameter
    if message_preset:
        return message_preset
    return detect_source_preset(bone_names) or DEFAULT_SOURCE_PRESET


def missing_dependency_message(exc: BaseException) -> str:
    """Return the fatal message for a failed retargeting import (always with the pip hint)."""
    text = str(exc) or exc.__class__.__name__
    message = f'retargeting is unavailable: {text}'
    if 'pip install' not in text:
        message += f'. Install the optional dependencies with: {PIP_HINT}'
    return message


# ---------------------------------------------------------------------------
# MuJoCo model helpers (need mujoco / the SDK [retargeting] extra)
# ---------------------------------------------------------------------------

# MJCF assets shipped with the SDK (mirrors retargeter.ROBOT_XML_DICT, used when mink is absent).
_ROBOT_XML_FILES = {
    'unitree_g1': ('retargeter', 'assets', 'unitree_g1', 'g1_mocap_29dof.xml'),
    'unitree_g1_with_hands': (
        'retargeter', 'assets', 'unitree_g1', 'g1_mocap_29dof_with_hands.xml'
    ),
}


def robot_xml_path(robot_type: str) -> str:
    """
    Return the MJCF path the SDK Retargeter loads for ``robot_type``.

    The SDK's ``ROBOT_XML_DICT`` is authoritative; when importing it fails
    because the ``[retargeting]`` extra (mink) is missing, the asset path is
    resolved inside the installed ``movin_sdk_python`` package instead so the
    model can still be inspected with mujoco alone.
    """
    try:
        from movin_sdk_python.retargeter.retargeter import ROBOT_XML_DICT
    except ImportError:
        ROBOT_XML_DICT = None
    if ROBOT_XML_DICT is not None:
        if robot_type not in ROBOT_XML_DICT:
            raise ValueError(
                f'unknown robot type {robot_type!r}; expected one of {list(ROBOT_XML_DICT)}'
            )
        return str(ROBOT_XML_DICT[robot_type])
    if robot_type not in _ROBOT_XML_FILES:
        raise ValueError(
            f'unknown robot type {robot_type!r}; expected one of {list(_ROBOT_XML_FILES)}'
        )
    import movin_sdk_python
    package_dir = os.path.dirname(os.path.abspath(movin_sdk_python.__file__))
    path = os.path.join(package_dir, *_ROBOT_XML_FILES[robot_type])
    if not os.path.isfile(path):
        raise FileNotFoundError(f'SDK robot model not found: {path}')
    return path


def load_robot_model(robot_type: str):
    """Load the Retargeter's MuJoCo model for ``robot_type`` (``mujoco.MjModel``)."""
    import mujoco
    return mujoco.MjModel.from_xml_path(robot_xml_path(robot_type))


def hinge_joint_names(model) -> list:
    """
    Return the non-free joint names of a MuJoCo model in model order.

    This is the order of ``qpos[7:]`` returned by ``Retargeter.retarget`` (the
    free joint occupies ``qpos[:7]``): 29 names for ``unitree_g1`` (identical
    to the revolute order of the URDF in ``movin_g1_description``) and 43 for
    ``unitree_g1_with_hands`` (hand joints interleaved after each wrist).
    """
    import mujoco
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(model.njnt)
        if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE
    ]


def create_retargeter(robot_type: str, human_height: float, source_preset: str):
    """Construct the SDK ``Retargeter`` (raises ``ValueError`` for unsupported combinations)."""
    from movin_sdk_python import Retargeter
    return Retargeter(
        robot_type=robot_type,
        human_height=float(human_height),
        verbose=False,
        source_preset=source_preset,
    )


def run_retarget(retargeter, bones, offset_to_ground: bool = False) -> RetargetResult:
    """
    Retarget one frame of SDK bone dicts and split the resulting ``qpos``.

    Raises :class:`MissingBonesError` when the frame lacks bones the IK config
    requires (the Retargeter would raise a bare ``KeyError`` otherwise).
    """
    human = retargeter.process_mocap_frame(bones)
    required = retargeter.get_required_bones()
    missing = sorted(name for name in required if name not in human)
    if missing:
        raise MissingBonesError(missing)
    qpos = retargeter.retarget({name: human[name] for name in required}, offset_to_ground)
    values = [float(v) for v in qpos]
    if len(values) < 7:
        raise ValueError(f'unexpected qpos length {len(values)} (< 7)')
    return RetargetResult(tuple(values[:3]), tuple(values[3:7]), values[7:])


# ---------------------------------------------------------------------------
# root pose (prefers movin_ros.convert.yaw_qpos, same math as fallback)
# ---------------------------------------------------------------------------

def _quat_normalize_xyzw(q) -> tuple:
    x, y, z, w = (float(c) for c in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_mul_xyzw(a, b) -> tuple:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_rotate_xyzw(q, v) -> tuple:
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


def yaw_qpos_fallback(qpos_root_pos_xyz, qpos_root_quat_wxyz, forward_mode,
                      offset_xyz=(0.0, 0.0, 0.0)):
    """
    Convert the Retargeter root pose to a world pose (same math as ``movin_ros.convert.yaw_qpos``).

    The Retargeter works in the ``coord_equivalent`` world.  For
    ``isaac_world`` the +90 degree yaw of the skeleton anchor is applied
    (``pos = R(qY) * pos``, ``quat = qY * quat``) so robot and skeleton face
    the same way; ``offset_xyz`` is added afterwards.  Returns
    ``(pos_xyz, quat_xyzw)``.
    """
    if forward_mode not in FORWARD_MODES:
        raise ValueError(
            f'unknown forward_mode {forward_mode!r}; expected one of {FORWARD_MODES}'
        )
    pos = tuple(float(c) for c in qpos_root_pos_xyz)
    w, x, y, z = (float(c) for c in qpos_root_quat_wxyz)
    quat = _quat_normalize_xyzw((x, y, z, w))
    if forward_mode == 'isaac_world':
        pos = _quat_rotate_xyzw(_YAW_QUAT_XYZW, pos)
        quat = _quat_normalize_xyzw(_quat_mul_xyzw(_YAW_QUAT_XYZW, quat))
    pos = (
        pos[0] + float(offset_xyz[0]),
        pos[1] + float(offset_xyz[1]),
        pos[2] + float(offset_xyz[2]),
    )
    return pos, quat


@functools.lru_cache(maxsize=None)
def _yaw_qpos_impl():
    """Return ``movin_ros.convert.yaw_qpos`` when importable, else the local fallback."""
    try:
        from movin_ros.convert import yaw_qpos
    except ImportError:
        return yaw_qpos_fallback
    return yaw_qpos


def root_pose(root_pos_xyz, root_quat_wxyz, forward_mode: str, offset_xyz=(0.0, 0.0, 0.0)):
    """
    Return ``(pos_xyz, quat_xyzw)`` of the robot root in the ``forward_mode`` world.

    Prefers ``movin_ros.convert.yaw_qpos`` (the single definition shared with
    ``mocap_node``) and falls back to :func:`yaw_qpos_fallback` when
    ``movin_ros`` is not installed or its signature differs.
    """
    if forward_mode not in FORWARD_MODES:
        raise ValueError(
            f'unknown forward_mode {forward_mode!r}; expected one of {FORWARD_MODES}'
        )
    impl = _yaw_qpos_impl()
    if impl is not yaw_qpos_fallback:
        try:
            pos, quat = impl(root_pos_xyz, root_quat_wxyz, forward_mode, offset_xyz)
            if len(pos) == 3 and len(quat) == 4:
                return (tuple(float(c) for c in pos), tuple(float(c) for c in quat))
        except (TypeError, ValueError):
            pass
    return yaw_qpos_fallback(root_pos_xyz, root_quat_wxyz, forward_mode, offset_xyz)


# ---------------------------------------------------------------------------
# small stateful helpers used by the node
# ---------------------------------------------------------------------------

class RateLimiter:
    """Accept at most ``max_rate_hz`` events per second (``<= 0`` disables the limit)."""

    def __init__(self, max_rate_hz: float):
        self.min_interval = 1.0 / float(max_rate_hz) if float(max_rate_hz) > 0.0 else 0.0
        self._last = None

    def accept(self, now=None) -> bool:
        """Return True (and remember ``now``) when enough time passed since the last accept."""
        now = time.monotonic() if now is None else float(now)
        if self._last is not None and now - self._last < self.min_interval:
            return False
        self._last = now
        return True


class ActorSelector:
    """
    Select the single actor to retarget.

    With an explicit ``actor`` only frames of that actor pass.  With ``''`` the
    first actor seen is locked in and every other actor is ignored afterwards.
    """

    def __init__(self, actor: str = ''):
        self.selected = actor or None
        self.explicit = bool(actor)

    def accept(self, actor: str) -> bool:
        """Return True when frames of ``actor`` should be retargeted."""
        if self.selected is None:
            self.selected = actor
            return True
        return actor == self.selected
