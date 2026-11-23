"""
ROS-free unit tests of :mod:`movin_retarget.retarget_core`.

The pure-Python parts (bone-dict rebuild, preset resolution, root pose,
rate/actor helpers) always run; joint-name checks need ``mujoco`` and the
end-to-end retargeting smoke test additionally needs ``mink`` (the SDK
``[retargeting]`` extra).  Missing optional dependencies skip those tests.
"""

from __future__ import annotations

import math
import os
import random
import subprocess
import sys
import types
import xml.etree.ElementTree as ElementTree

from movin_retarget import retarget_core as core
import pytest

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_DIR = os.path.dirname(os.path.dirname(_TEST_DIR))
_FIXTURE = os.path.join(_REPO_DIR, 'movin_ros', 'test', 'data', 'live_movinman_v3_1s.pkl')
_URDF = os.path.join(
    _REPO_DIR, 'movin_g1_description', 'urdf', 'g1_custom_collision_29dof.urdf'
)

# Old motion_player.py JOINT_NAMES == URDF revolute order == MuJoCo hinge order (29 DoF).
_G1_29DOF_JOINTS = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
]


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _synthetic_sdk_bones():
    """Return a tiny parents-first SDK frame with non-trivial quaternions."""
    return [
        {'bone_index': 0, 'parent_index': -1, 'bone_name': 'RootBone',
         'p': (0.0, 0.0, 0.0), 'rq': (1.0, 0.0, 0.0, 0.0), 'q': (1.0, 0.0, 0.0, 0.0),
         's': (1.0, 1.0, 1.0)},
        {'bone_index': 1, 'parent_index': 0, 'bone_name': 'Hips',
         'p': (0.25, 0.9, -0.1), 'rq': (0.9, 0.1, -0.2, 0.3), 'q': (0.5, -0.5, 0.5, -0.5),
         's': (1.0, 2.0, 3.0)},
        {'bone_index': 2, 'parent_index': 1, 'bone_name': 'Spine',
         'p': (0.0, 0.1, 0.02), 'rq': (0.0, 0.0, 0.0, 1.0), 'q': (0.6, 0.0, 0.8, 0.0),
         's': (1.0, 1.0, 1.0)},
    ]


def _fixture_frame():
    """Return the first frame of the real live recording (SDK core only, no numpy)."""
    pytest.importorskip('movin_sdk_python')
    if not os.path.exists(_FIXTURE):
        pytest.skip(f'fixture missing: {_FIXTURE}')
    from movin_sdk_python import peek_first_frame
    frame = peek_first_frame(_FIXTURE)
    assert frame is not None
    return frame


def _message_like_from_sdk(frame, as_objects=False, preset='movinman_v3'):
    """
    Build a MocapFrame-like dict (or attribute objects) the way mocap_node fills the message.

    Quaternions go from SDK ``(w, x, y, z)`` to geometry_msgs ``x=q[1], y=q[2], z=q[3], w=q[0]``.
    """
    def vec(v):
        return {'x': v[0], 'y': v[1], 'z': v[2]}

    def quat(q):
        return {'x': q[1], 'y': q[2], 'z': q[3], 'w': q[0]}

    bones = []
    for bone in frame['bones']:
        bones.append({
            'bone_index': bone['bone_index'],
            'parent_index': bone['parent_index'],
            'name': bone['bone_name'],
            'position': vec(bone['p']),
            'rest_rotation': quat(bone['rq']),
            'rotation': quat(bone['q']),
            'scale': vec(bone['s']),
        })
    msg = {
        'actor': frame['actor'],
        'actor_frame': 'movin/' + frame['actor'],
        'frame_idx': frame['frame_idx'],
        'source_timestamp': frame['timestamp'],
        'preset': preset,
        'convention': core.CONVENTION_UNITY_LH_YUP,
        'bones': bones,
    }
    if as_objects:
        return _to_objects(msg)
    return msg


def _to_objects(value):
    if isinstance(value, dict):
        return types.SimpleNamespace(**{k: _to_objects(v) for k, v in value.items()})
    if isinstance(value, list):
        return [_to_objects(v) for v in value]
    return value


def _yaw(q_xyzw) -> float:
    x, y, z, w = q_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _urdf_revolute_joints():
    if not os.path.exists(_URDF):
        pytest.skip(f'URDF missing: {_URDF}')
    root = ElementTree.parse(_URDF).getroot()
    return [j.get('name') for j in root.iter('joint') if j.get('type') == 'revolute']


# ---------------------------------------------------------------------------
# bone dict rebuild (pure Python)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('as_objects', [False, True])
def test_bone_dict_rebuild_round_trips_synthetic(as_objects):
    original = _synthetic_sdk_bones()
    frame = {'timestamp': 't', 'actor': 'A', 'frame_idx': 7, 'bones': original}
    msg = _message_like_from_sdk(frame, as_objects=as_objects)

    rebuilt = core.bones_from_message(msg)
    assert rebuilt == original
    assert core.frame_from_message(msg) == frame
    for bone in rebuilt:
        assert isinstance(bone['q'], tuple) and len(bone['q']) == 4
        assert isinstance(bone['rq'], tuple) and len(bone['rq']) == 4
        assert isinstance(bone['p'], tuple) and len(bone['p']) == 3


def test_bone_dict_rebuild_keeps_wxyz_order():
    bone = {
        'bone_index': 3, 'parent_index': 1, 'name': 'X',
        'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
        'rest_rotation': {'x': 0.1, 'y': 0.2, 'z': 0.3, 'w': 0.9},
        'rotation': {'x': -0.1, 'y': -0.2, 'z': -0.3, 'w': -0.9},
        'scale': {'x': 1.0, 'y': 1.0, 'z': 1.0},
    }
    sdk = core.bone_from_message(bone)
    assert sdk['rq'] == (0.9, 0.1, 0.2, 0.3)       # (w, x, y, z)
    assert sdk['q'] == (-0.9, -0.1, -0.2, -0.3)
    assert sdk['p'] == (1.0, 2.0, 3.0)
    assert sdk['bone_name'] == 'X'
    assert set(sdk) == {'bone_index', 'parent_index', 'bone_name', 'p', 'rq', 'q', 's'}


def test_bone_dict_rebuild_round_trips_live_fixture():
    frame = _fixture_frame()
    assert len(frame['bones']) == 55
    msg = _message_like_from_sdk(frame, as_objects=True)
    rebuilt = core.bones_from_message(msg)
    assert rebuilt == frame['bones']
    assert core.frame_from_message(msg) == frame


def test_check_convention():
    core.check_convention({'convention': 0})
    with pytest.raises(ValueError):
        core.check_convention({'convention': 1})


# ---------------------------------------------------------------------------
# preset / dependency message helpers
# ---------------------------------------------------------------------------

def test_resolve_source_preset():
    assert core.resolve_source_preset('movinman', 'movinman_v3') == 'movinman'
    assert core.resolve_source_preset('auto', 'movinman') == 'movinman'
    assert core.resolve_source_preset('', 'movinman_v3') == 'movinman_v3'
    assert core.resolve_source_preset('auto', '', []) == core.DEFAULT_SOURCE_PRESET


def test_resolve_source_preset_detects_from_bone_names():
    pytest.importorskip('movin_sdk_python')
    assert core.resolve_source_preset('auto', '', ['Hips', 'Spine3', 'Neck1']) == 'movinman_v3'
    assert core.resolve_source_preset('auto', '', ['Hips', 'Spine1']) == 'movinman'


def test_missing_dependency_message_has_pip_hint():
    plain = core.missing_dependency_message(ImportError("No module named 'mujoco'"))
    assert 'mujoco' in plain and core.PIP_HINT in plain

    sdk = pytest.importorskip('movin_sdk_python')
    exc = sdk.MissingOptionalDependencyError('Retargeting', 'retargeting', 'mink')
    text = core.missing_dependency_message(exc)
    assert str(exc) in text
    assert text.count('pip install') == 1


# ---------------------------------------------------------------------------
# root pose
# ---------------------------------------------------------------------------

def test_yaw_qpos_fallback_math():
    pos = (1.0, 2.0, 0.5)
    quat_wxyz = (1.0, 0.0, 0.0, 0.0)
    p, q = core.yaw_qpos_fallback(pos, quat_wxyz, 'coord_equivalent', (0.1, 0.2, 0.3))
    assert p == pytest.approx((1.1, 2.2, 0.8))
    assert q == pytest.approx((0.0, 0.0, 0.0, 1.0))

    p, q = core.yaw_qpos_fallback(pos, quat_wxyz, 'isaac_world')
    assert p == pytest.approx((-2.0, 1.0, 0.5))          # +90 deg yaw: (x, y) -> (-y, x)
    assert _yaw(q) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        core.yaw_qpos_fallback(pos, quat_wxyz, 'sideways')


def test_root_pose_matches_fallback_and_movin_ros():
    pos = (0.3, -0.4, 0.9)
    quat_wxyz = (0.8, 0.1, -0.3, 0.5)   # not normalized on purpose
    offset = (0.5, 0.0, 0.0)
    for mode in core.FORWARD_MODES:
        got = core.root_pose(pos, quat_wxyz, mode, offset)
        want = core.yaw_qpos_fallback(pos, quat_wxyz, mode, offset)
        assert got[0] == pytest.approx(want[0], abs=1e-12)
        assert got[1] == pytest.approx(want[1], abs=1e-12)
        assert math.hypot(*got[1]) == pytest.approx(1.0)
    with pytest.raises(ValueError):
        core.root_pose(pos, quat_wxyz, 'sideways')

    convert = pytest.importorskip('movin_ros.convert')
    for mode in core.FORWARD_MODES:
        ref = convert.yaw_qpos(pos, quat_wxyz, mode, offset)
        got = core.root_pose(pos, quat_wxyz, mode, offset)
        assert got[0] == pytest.approx(ref[0], abs=1e-12)
        assert got[1] == pytest.approx(ref[1], abs=1e-12)


# ---------------------------------------------------------------------------
# rate limiter / actor selector
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('value, expected', [
    ('auto', 'auto'), (' Auto ', 'auto'), ('1.66', 1.66), (' 1.8 ', 1.8), (1.66, 1.66), (2, 2.0),
])
def test_parse_human_height(value, expected):
    """'auto' or the actor's height in meters, as a string (launch) or a number (-p / YAML)."""
    assert core.parse_human_height(value) == expected


@pytest.mark.parametrize('value', ['', 'tall', '1.7m', 0, 0.0, -1.75, '-1', 'nan', 'inf',
                                   float('inf'), True, False, None, [1.7]])
def test_parse_human_height_rejects(value):
    """Anything but 'auto' or a finite positive height is a ValueError with the expected form."""
    with pytest.raises(ValueError, match="'auto' or the actor's height in meters"):
        core.parse_human_height(value)


def _accepted(limiter, times):
    """Return the arrival times ``limiter`` accepts, in order."""
    return [t for t in times if limiter.accept(now=t)]


def _most_in_window(times, window):
    """Return the most ``times`` that fall into any half-open interval of length ``window``."""
    end = 0
    best = 0
    for start, t in enumerate(times):
        while end < len(times) and times[end] < t + window:
            end += 1
        best = max(best, end - start)
    return best


def _jittered(rate_hz, count, max_delay, seed):
    """Return ``count`` in-order arrivals at ``rate_hz``, each delayed by up to ``max_delay``."""
    rng = random.Random(seed)
    times = []
    for i in range(count):
        arrival = i / rate_hz + rng.uniform(0.0, max_delay)
        times.append(max(arrival, times[-1]) if times else arrival)  # delivery keeps the order
    return times


def test_rate_limiter_passes_a_jittered_stream_at_the_limit():
    """At max_rate_hz = stream rate, early, late and paired frames all pass."""
    period = 1.0 / 60.0
    times = _jittered(60.0, 1200, 1.5 * period, seed=1)
    intervals = [b - a for a, b in zip(times, times[1:])]
    assert sum(dt < period for dt in intervals) > len(intervals) // 4  # the jitter is real
    assert min(intervals) < 0.1 * period
    assert _accepted(core.RateLimiter(60.0), times) == times


@pytest.mark.parametrize('jitter', [0.0, 0.5 / 120.0])
def test_rate_limiter_thins_a_faster_stream_to_the_limit(jitter):
    """A 120 Hz stream at max_rate_hz=60 comes out at 60 Hz on average and in every second."""
    times = _jittered(120.0, 1200, jitter, seed=2)  # 10 s
    accepted = _accepted(core.RateLimiter(60.0), times)
    assert abs(len(accepted) - 600) <= core.RateLimiter.BURST
    assert _most_in_window(accepted, 1.0) <= 60 + core.RateLimiter.BURST


def test_rate_limiter_spreads_frames_that_pile_up_after_a_pause():
    """Frames delivered together after a pause pass at most BURST at once, then 60 Hz resumes."""
    period = 1.0 / 60.0
    limiter = core.RateLimiter(60.0)
    before = [i * period for i in range(60)]
    pile = [2.0 + i * 1e-4 for i in range(30)]
    after = [2.0 + (i + 1) * period for i in range(120)]
    assert _accepted(limiter, before) == before
    assert len(_accepted(limiter, pile)) == core.RateLimiter.BURST
    assert len(_accepted(limiter, after)) >= len(after) - 1
    # the pause earns no extra credit: still at most 60 + BURST frames in any second
    everything = _accepted(core.RateLimiter(60.0), before + pile + after)
    assert _most_in_window(everything, 1.0) <= 60 + core.RateLimiter.BURST


@pytest.mark.parametrize('max_rate_hz', [0.0, -5.0])
def test_rate_limiter_without_a_limit_accepts_everything(max_rate_hz):
    """max_rate_hz <= 0 accepts every event, even many at the same instant."""
    unlimited = core.RateLimiter(max_rate_hz)
    assert all(unlimited.accept(now=0.0) for _ in range(100))
    assert all(unlimited.accept() for _ in range(10))


def test_actor_selector():
    first = core.ActorSelector('')
    assert first.accept('MOVINMan')
    assert not first.accept('Other')
    assert first.accept('MOVINMan')
    assert first.selected == 'MOVINMan' and not first.explicit

    explicit = core.ActorSelector('Bob')
    assert not explicit.accept('MOVINMan')
    assert explicit.accept('Bob')
    assert explicit.explicit


def test_core_import_is_numpy_free():
    code = (
        'import sys\n'
        'import movin_retarget.retarget_core\n'
        'bad = [m for m in ("numpy", "scipy", "mujoco", "mink", "rclpy") if m in sys.modules]\n'
        'print(bad)\n'
    )
    env = dict(os.environ)
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(core.__file__)))
    env['PYTHONPATH'] = package_dir + os.pathsep + env.get('PYTHONPATH', '')
    out = subprocess.run(
        [sys.executable, '-c', code], env=env, capture_output=True, text=True, check=True
    )
    assert out.stdout.strip() == '[]'


# ---------------------------------------------------------------------------
# MuJoCo joint order (needs mujoco)
# ---------------------------------------------------------------------------

def test_g1_hinge_joints_match_urdf_revolute_order():
    pytest.importorskip('mujoco')
    pytest.importorskip('movin_sdk_python')
    names = core.hinge_joint_names(core.load_robot_model('unitree_g1'))
    assert len(names) == 29
    assert names == _urdf_revolute_joints()
    assert names == _G1_29DOF_JOINTS


def test_g1_with_hands_hinge_joints():
    pytest.importorskip('mujoco')
    pytest.importorskip('movin_sdk_python')
    names = core.hinge_joint_names(core.load_robot_model('unitree_g1_with_hands'))
    assert len(names) == 43
    assert len(set(names)) == 43
    base = set(_G1_29DOF_JOINTS)
    # Hand joints are interleaved after each wrist: the 29 base joints keep their relative order
    # but are not a prefix, so qpos must be zipped with the names, never sliced positionally.
    assert [n for n in names if n in base] == _G1_29DOF_JOINTS
    assert names[:29] != _G1_29DOF_JOINTS
    assert all(n.startswith(('left_hand_', 'right_hand_')) for n in names if n not in base)


def test_robot_xml_path_rejects_unknown_robot():
    pytest.importorskip('movin_sdk_python')
    with pytest.raises(ValueError):
        core.robot_xml_path('unitree_h1')


# ---------------------------------------------------------------------------
# end-to-end retargeting smoke test (needs the full [retargeting] extra)
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def retargeter():
    pytest.importorskip('numpy')
    pytest.importorskip('mujoco')
    pytest.importorskip('mink')
    pytest.importorskip('movin_sdk_python')
    return core.create_retargeter('unitree_g1', 'auto', 'movinman_v3')


def test_human_height_auto_measures_the_stream(retargeter):
    """'auto' starts at the reference body and sizes the targets from the first frame."""
    fresh = core.create_retargeter('unitree_g1', 'auto', 'movinman_v3')
    assert fresh.human_height == core.HUMAN_HEIGHT_AUTO and fresh.body_scale == 1.0
    core.run_retarget(fresh, _fixture_frame()['bones'])
    assert 0.5 < fresh.body_scale < 2.0 and fresh.body_scale != 1.0


@pytest.mark.parametrize('height', [1.66, '1.66', 1.9])
def test_human_height_number_is_the_actor_height(retargeter, height):
    """A number is the actor's height: body_scale = 1.8 / h (taller actor, smaller scale)."""
    sized = core.create_retargeter('unitree_g1', height, 'movinman_v3')
    expected = core.REFERENCE_HEIGHT_M / float(height)
    assert sized.body_scale == pytest.approx(expected)
    core.run_retarget(sized, _fixture_frame()['bones'])   # not re-measured from the stream
    assert sized.body_scale == pytest.approx(expected)


def test_reference_height_matches_the_sdk_ik_configs(retargeter):
    """REFERENCE_HEIGHT_M (used in log messages) is the SDK IK tables' reference height."""
    import json
    from movin_sdk_python.retargeter.retargeter import IK_CONFIG_BY_PRESET
    for path in set(IK_CONFIG_BY_PRESET.values()):
        with open(path) as f:
            assert json.load(f)['human_height_assumption'] == core.REFERENCE_HEIGHT_M


def test_create_retargeter_rejects_bad_human_height(retargeter):
    """An invalid height fails before the Retargeter is built."""
    with pytest.raises(ValueError, match='human_height'):
        core.create_retargeter('unitree_g1', 'tall', 'movinman_v3')


def test_run_retarget_on_live_fixture(retargeter):
    frame = _fixture_frame()
    bones = core.bones_from_message(_message_like_from_sdk(frame, as_objects=True))
    names = core.hinge_joint_names(retargeter.model)

    result = core.run_retarget(retargeter, bones)
    assert len(result.joint_positions) == len(names) == 29
    assert all(math.isfinite(v) for v in result.joint_positions)
    assert all(isinstance(v, float) for v in result.joint_positions)
    assert math.hypot(*result.root_quat_wxyz) == pytest.approx(1.0, abs=1e-6)
    assert 0.2 < result.root_pos[2] < 1.5      # pelvis height in meters

    coord = core.root_pose(result.root_pos, result.root_quat_wxyz, 'coord_equivalent')
    isaac = core.root_pose(result.root_pos, result.root_quat_wxyz, 'isaac_world', (1.0, 0.0, 0.0))
    # isaac_world = coord_equivalent rotated by +90 deg about Z, then offset.
    assert isaac[0] == pytest.approx((-coord[0][1] + 1.0, coord[0][0], coord[0][2]), abs=1e-9)
    delta = (_yaw(isaac[1]) - _yaw(coord[1]) + math.pi) % (2.0 * math.pi) - math.pi
    assert delta == pytest.approx(math.pi / 2, abs=1e-6)

    again = core.run_retarget(retargeter, bones, offset_to_ground=True)
    assert len(again.joint_positions) == 29


def test_run_retarget_reports_missing_bones(retargeter):
    frame = _fixture_frame()
    bones = [b for b in frame['bones'] if b['bone_name'] != 'Spine3']
    with pytest.raises(core.MissingBonesError) as info:
        core.run_retarget(retargeter, bones)
    assert info.value.missing == ('Spine3',)
    assert 'Spine3' in str(info.value)


def test_legacy_preset_rejects_v3_only_bones_gracefully(retargeter):
    legacy = core.create_retargeter('unitree_g1', 'auto', 'movinman')
    frame = _fixture_frame()
    # The V3 stream also has Spine1, so the legacy mapping can consume it.
    result = core.run_retarget(legacy, frame['bones'])
    assert len(result.joint_positions) == 29
    with pytest.raises(ValueError):
        core.create_retargeter('unitree_g1', 'auto', 'movinman_v9')
