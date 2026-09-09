"""
Generate ``test/data/movinman_legacy_short.pkl``: a small synthetic legacy-skeleton recording.

The file is a genuine SDK version-1 recording written through
``movin_sdk_python.OscRecorder`` from synthetic ``/MOVIN/Frame`` chunks, so it
exercises the same assembler path as a live stream.  The skeleton is the
51-body legacy ``movinman`` preset (``MOVINMAN_BODY_NAMES``) under a ``Root``
bone (index 0, parent -1) that carries the actor's global transform, exactly
like the streams of the legacy MOVINMan rig.  Standard library only (plus the
SDK core); run it from anywhere::

    python3 movin_ros/test/make_fixture.py [--frames 30] [--fps 60] [--output PATH]
"""

from __future__ import annotations

import argparse
import datetime
import math
import os
import struct
import sys

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(TEST_DIR))
SDK_DIR = os.path.join(REPO_DIR, 'movin_sdk_python_vendor', 'MOVIN-SDK-Python')
if os.path.isdir(SDK_DIR) and SDK_DIR not in sys.path:
    # The fixture must come from the pinned submodule, not from whatever SDK
    # copy the interpreter happens to see (there is none at all without ROS).
    sys.path.insert(0, SDK_DIR)

from movin_sdk_python import OscPlayer, OscRecorder, peek_first_frame  # noqa: E402
from movin_sdk_python.utils.skeleton_presets import MOVINMAN_BODY_NAMES  # noqa: E402

DEFAULT_OUTPUT = os.path.join(TEST_DIR, 'data', 'movinman_legacy_short.pkl')
MAX_BYTES = 300 * 1024
ACTOR = 'MOVINMan'
ROOT_NAME = 'Root'
FRAME_ADDRESS = '/MOVIN/Frame'
FIRST_FRAME_IDX = 1000
START_TIME = datetime.datetime(2026, 9, 9, 12, 0, 0)

IDENTITY_XYZW = (0.0, 0.0, 0.0, 1.0)
ONES = (1.0, 1.0, 1.0)

# Rest-pose offsets (Unity: +X = actor's right, +Y up, +Z forward, meters) of the
# legacy rig, keyed by body name; left-side bones mirror the right side in X.
_RIGHT_OFFSETS = {
    'Shoulder': (0.06, 0.12, 0.0),
    'Arm': (0.12, 0.0, 0.0),
    'ForeArm': (0.27, 0.0, 0.0),
    'Hand': (0.26, 0.0, 0.0),
    'HandThumb1': (0.03, -0.01, 0.03),
    'HandThumb2': (0.03, 0.0, 0.01),
    'HandThumb3': (0.025, 0.0, 0.005),
    'HandIndex1': (0.09, 0.0, 0.025),
    'HandIndex2': (0.035, 0.0, 0.0),
    'HandIndex3': (0.025, 0.0, 0.0),
    'HandMiddle1': (0.09, 0.0, 0.005),
    'HandMiddle2': (0.04, 0.0, 0.0),
    'HandMiddle3': (0.028, 0.0, 0.0),
    'HandRing1': (0.085, 0.0, -0.012),
    'HandRing2': (0.035, 0.0, 0.0),
    'HandRing3': (0.025, 0.0, 0.0),
    'HandPinky1': (0.08, 0.0, -0.03),
    'HandPinky2': (0.03, 0.0, 0.0),
    'HandPinky3': (0.02, 0.0, 0.0),
    'UpLeg': (0.09, -0.05, 0.0),
    'Leg': (0.0, -0.42, 0.0),
    'Foot': (0.0, -0.40, 0.0),
    'ToeBase': (0.0, -0.06, 0.14),
}
_CENTER_OFFSETS = {
    'Hips': (0.0, 0.87, 0.0),
    'Spine': (0.0, 0.10, 0.0),
    'Spine1': (0.0, 0.12, 0.0),
    'Neck': (0.0, 0.15, 0.0),
    'Head': (0.0, 0.10, 0.0),
}


def legacy_parent(name: str) -> str:
    """Return the parent body name of ``name`` in the legacy MOVINMan hierarchy."""
    center = {'Hips': ROOT_NAME, 'Spine': 'Hips', 'Spine1': 'Spine', 'Neck': 'Spine1',
              'Head': 'Neck'}
    if name in center:
        return center[name]
    for side in ('Left', 'Right'):
        if not name.startswith(side):
            continue
        part = name[len(side):]
        chain = {'Shoulder': 'Spine1', 'Arm': f'{side}Shoulder', 'ForeArm': f'{side}Arm',
                 'Hand': f'{side}ForeArm', 'UpLeg': 'Hips', 'Leg': f'{side}UpLeg',
                 'Foot': f'{side}Leg', 'ToeBase': f'{side}Foot'}
        if part in chain:
            return chain[part]
        if part.startswith('Hand') and part[-1] in '123':
            digit = int(part[-1])
            return f'{side}Hand' if digit == 1 else f'{name[:-1]}{digit - 1}'
    raise KeyError(f'unknown legacy bone {name!r}')


def rest_offset(name: str):
    """Return the rest-pose offset of ``name`` in its parent's frame (Unity axes)."""
    if name == ROOT_NAME:
        return (0.0, 0.0, 0.0)
    if name in _CENTER_OFFSETS:
        return _CENTER_OFFSETS[name]
    for side, sign in (('Left', -1.0), ('Right', 1.0)):
        if name.startswith(side):
            x, y, z = _RIGHT_OFFSETS[name[len(side):]]
            return (sign * x, y, z)
    raise KeyError(name)


def build_hierarchy() -> list:
    """Return ``[(bone_index, parent_index, name), ...]`` for Root + the 51 legacy bodies."""
    names = (ROOT_NAME,) + tuple(MOVINMAN_BODY_NAMES)
    index_of = {name: i for i, name in enumerate(names)}
    rows = [(0, -1, ROOT_NAME)]
    for name in MOVINMAN_BODY_NAMES:
        rows.append((index_of[name], index_of[legacy_parent(name)], name))
    for bone_index, parent_index, _name in rows:
        assert parent_index < bone_index, 'legacy DFS order must be parents-first'
    return rows


def axis_angle_xyzw(axis, angle: float):
    """Return the quaternion (x, y, z, w) rotating ``angle`` radians about unit ``axis``."""
    half = 0.5 * angle
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))


def quat_mul_xyzw(a, b):
    """Return the Hamilton product ``a * b`` of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def local_rotation_xyzw(name: str, t: float):
    """Return the synthetic local rotation of ``name`` at time ``t`` seconds (x, y, z, w)."""
    swing = math.sin(2.0 * math.pi * 0.8 * t)
    if name == 'Hips':
        yaw = axis_angle_xyzw((0.0, 1.0, 0.0), 0.15 * math.sin(math.pi * t))
        lean = axis_angle_xyzw((1.0, 0.0, 0.0), 0.05 * swing)
        return quat_mul_xyzw(yaw, lean)
    if name in ('Spine', 'Spine1'):
        return axis_angle_xyzw((0.0, 0.0, 1.0), 0.04 * swing)
    if name in ('Neck', 'Head'):
        return axis_angle_xyzw((0.0, 1.0, 0.0), 0.1 * swing)
    for side, sign in (('Left', 1.0), ('Right', -1.0)):
        if not name.startswith(side):
            continue
        part = name[len(side):]
        if part == 'Arm':
            return axis_angle_xyzw((1.0, 0.0, 0.0), sign * 0.5 * swing)
        if part == 'ForeArm':
            return axis_angle_xyzw((0.0, 1.0, 0.0), sign * 0.3 * (1.0 - swing))
        if part == 'UpLeg':
            return axis_angle_xyzw((1.0, 0.0, 0.0), -sign * 0.4 * swing)
        if part == 'Leg':
            return axis_angle_xyzw((1.0, 0.0, 0.0), 0.35 * (1.0 + sign * swing))
        if part == 'Foot':
            return axis_angle_xyzw((1.0, 0.0, 0.0), 0.1 * swing)
        if part.startswith('Hand') and part[-1] in '123':
            return axis_angle_xyzw((0.0, 0.0, 1.0), -sign * 0.3 * (1.0 + swing) * 0.5)
    return IDENTITY_XYZW


def float32(value: float) -> float:
    """Round ``value`` to the nearest float32 as an OSC float argument would."""
    return struct.unpack('>f', struct.pack('>f', value))[0]


def bone_args(bone_index: int, parent_index: int, name: str, t: float) -> list:
    """Return the 17 OSC arguments MOVIN Studio streams for one bone (quaternions xyzw)."""
    p = rest_offset(name)
    if name == 'Hips':
        p = (p[0], p[1] + 0.02 * math.sin(2.0 * math.pi * t), p[2])
    q = local_rotation_xyzw(name, t)
    args = [bone_index, parent_index, name]
    args.extend(float32(c) for c in p)
    args.extend(IDENTITY_XYZW)
    args.extend(float32(c) for c in q)
    args.extend(ONES)
    return args


def frame_messages(frame: int, fps: float, chunks: int, hierarchy) -> list:
    """Return ``[(wall_time, address, args), ...]`` for the ``chunks`` OSC messages of a frame."""
    t = frame / fps
    stamp = (START_TIME + datetime.timedelta(seconds=t)).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    total = len(hierarchy)
    per_chunk = -(-total // chunks)
    messages = []
    for chunk_idx in range(chunks):
        rows = hierarchy[chunk_idx * per_chunk:(chunk_idx + 1) * per_chunk]
        args = [stamp, ACTOR, FIRST_FRAME_IDX + frame, chunks, chunk_idx, total, len(rows)]
        for bone_index, parent_index, name in rows:
            args.extend(bone_args(bone_index, parent_index, name, t))
        messages.append((t + chunk_idx * 0.0002, FRAME_ADDRESS, args))
    return messages


def build_messages(frames: int = 30, fps: float = 60.0, chunks: int = 3) -> list:
    """Return the synthetic ``(wall_time, address, args)`` stream of the whole fixture."""
    hierarchy = build_hierarchy()
    messages = []
    for frame in range(frames):
        messages.extend(frame_messages(frame, fps, chunks, hierarchy))
    return messages


def write_recording(path: str, messages) -> dict:
    """Write ``messages`` through ``OscRecorder`` and return a summary of the result."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    recorder = OscRecorder(path, stream_type='auto')
    for wall_time, address, args in messages:
        recorder.record(address, args, wall_time=1_700_000_000.0 + wall_time)
    recorder.save()
    player = OscPlayer(path)
    first = peek_first_frame(path)
    return {
        'path': path,
        'bytes': os.path.getsize(path),
        'num_messages': player.num_messages,
        'duration_sec': player.duration_sec,
        'stream_type': player.stream_type,
        'bones': len(first['bones']) if first else 0,
    }


def main(argv=None) -> int:
    """Generate the fixture; return 1 when it exceeds the size budget."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--output', default=DEFAULT_OUTPUT)
    parser.add_argument('--frames', type=int, default=30)
    parser.add_argument('--fps', type=float, default=60.0)
    parser.add_argument('--chunks', type=int, default=3)
    args = parser.parse_args(argv)
    summary = write_recording(args.output, build_messages(args.frames, args.fps, args.chunks))
    print(f"[make_fixture] {summary['path']}: {summary['bytes']} bytes, "
          f"{summary['num_messages']} messages, {summary['duration_sec']:.3f} s, "
          f"stream_type={summary['stream_type']}, bones={summary['bones']}")
    if summary['bytes'] > MAX_BYTES:
        print(f'[make_fixture] ERROR: fixture exceeds {MAX_BYTES} bytes', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
