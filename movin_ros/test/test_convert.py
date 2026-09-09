"""Tests for the pure-Python coordinate conversion in ``movin_ros.convert``."""

from __future__ import annotations

import math
import random

from movin_ros import convert

import pytest

ANCHOR_QUAT_XYZW = convert.ANCHOR_QUAT_XYZW
FORWARD_MODES = convert.FORWARD_MODES
Edge = convert.Edge
anchor_frame = convert.anchor_frame
assign_actor_id = convert.assign_actor_id
bone_frame = convert.bone_frame
cylinder_pose = convert.cylinder_pose
forward_kinematics = convert.forward_kinematics
local_edges = convert.local_edges
mirror_local = convert.mirror_local
quat_mul_xyzw = convert.quat_mul_xyzw
quat_normalize_xyzw = convert.quat_normalize_xyzw
quat_rotate_xyzw = convert.quat_rotate_xyzw
sanitize_actor_id = convert.sanitize_actor_id
yaw_qpos = convert.yaw_qpos

IDENTITY_WXYZ = (1.0, 0.0, 0.0, 0.0)
PARITY_ATOL = 1e-9


def _bone(index, parent, name, p=(0.0, 0.0, 0.0), q=IDENTITY_WXYZ):
    return {
        'bone_index': index,
        'parent_index': parent,
        'bone_name': name,
        'p': tuple(p),
        'rq': IDENTITY_WXYZ,
        'q': tuple(q),
        's': (1.0, 1.0, 1.0),
    }


def _random_unit_wxyz(rng):
    q = [rng.gauss(0.0, 1.0) for _ in range(4)]
    norm = math.sqrt(sum(c * c for c in q))
    return tuple(c / norm for c in q)


def _random_tree(rng, count):
    """Random parents-first tree with unique names, root first (parent -1)."""
    bones = [_bone(0, -1, 'Root', p=[rng.uniform(-1, 1) for _ in range(3)],
                   q=_random_unit_wxyz(rng))]
    for index in range(1, count):
        parent = rng.randrange(0, index)
        bones.append(_bone(index, parent, f'bone{index}',
                           p=[rng.uniform(-0.5, 0.5) for _ in range(3)],
                           q=_random_unit_wxyz(rng)))
    return bones


def _close(a, b, atol=1e-12):
    return all(abs(float(x) - float(y)) <= atol for x, y in zip(a, b))


# ---------------------------------------------------------------- mirror/anchor


def test_mirror_local_formula():
    """Verify mirror local formula."""
    p = (0.1, 0.2, 0.3)
    q = (0.5, -0.5, 0.5, -0.5)  # w, x, y, z
    t, q_xyzw = mirror_local(p, q)
    assert t == (-0.1, 0.2, 0.3)
    assert _close(q_xyzw, (-0.5, -0.5, 0.5, 0.5))


def test_mirror_local_normalizes_and_is_proper_rotation():
    """Verify mirror local normalizes and is proper rotation."""
    _, q_xyzw = mirror_local((0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0))
    assert _close(q_xyzw, (0.0, 0.0, 0.0, 1.0))
    _, q_xyzw = mirror_local((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
    assert q_xyzw == (0.0, 0.0, 0.0, 1.0)
    rng = random.Random(1)
    for _ in range(20):
        _, q_xyzw = mirror_local((0, 0, 0), _random_unit_wxyz(rng))
        assert abs(math.sqrt(sum(c * c for c in q_xyzw)) - 1.0) < 1e-12


def test_anchor_constants():
    """Verify anchor constants."""
    assert FORWARD_MODES == ('coord_equivalent', 'isaac_world')
    assert set(ANCHOR_QUAT_XYZW) == set(FORWARD_MODES)
    s = 0.7071067811865476
    assert ANCHOR_QUAT_XYZW['coord_equivalent'] == (s, 0.0, 0.0, s)
    assert ANCHOR_QUAT_XYZW['isaac_world'] == (0.5, 0.5, 0.5, 0.5)
    # Unity up (+Y) becomes ROS +Z in both modes; Unity forward (+Z) becomes
    # ROS -Y (coord_equivalent) or ROS +X (isaac_world, REP-103).
    for mode, forward in (('coord_equivalent', (0, -1, 0)), ('isaac_world', (1, 0, 0))):
        q = ANCHOR_QUAT_XYZW[mode]
        assert _close(quat_rotate_xyzw(q, (0, 1, 0)), (0, 0, 1))
        assert _close(quat_rotate_xyzw(q, (0, 0, 1)), forward)
    # isaac_world = R_z(+90) * R_x(+90)
    yaw = (0.0, 0.0, 0.7071067811865476, 0.7071067811865476)
    assert _close(quat_mul_xyzw(yaw, ANCHOR_QUAT_XYZW['coord_equivalent']),
                  ANCHOR_QUAT_XYZW['isaac_world'])


def test_anchor_quat_lookup_rejects_unknown_mode():
    """Verify anchor quat lookup rejects unknown mode."""
    assert convert.anchor_quat('isaac_world') == (0.5, 0.5, 0.5, 0.5)
    with pytest.raises(ValueError):
        convert.anchor_quat('sideways')


# ------------------------------------------------------------------ actor ids


def test_sanitize_actor_id():
    """Verify sanitize actor id."""
    assert sanitize_actor_id('MOVINMan') == 'MOVINMan'
    assert sanitize_actor_id('Actor One/2') == 'Actor_One_2'
    assert sanitize_actor_id('') == 'actor0'
    assert sanitize_actor_id('한글') == '__'


def test_assign_actor_id_deterministic_and_collision_safe():
    """Verify assign actor id deterministic and collision safe."""
    registry = {}
    assert assign_actor_id('MOVINMan', registry) == 'MOVINMan'
    assert assign_actor_id('MOVINMan', registry) == 'MOVINMan'
    assert assign_actor_id('A B', registry) == 'A_B'
    collided = assign_actor_id('A-B', registry)
    assert collided.startswith('A_B_') and len(collided) == len('A_B_') + 4
    assert assign_actor_id('A-B', registry) == collided
    assert registry == {'MOVINMan': 'MOVINMan', 'A B': 'A_B', 'A-B': collided}
    # deterministic across registries
    assert assign_actor_id('A-B', {'A B': 'A_B'}) == collided
    # without a collision the raw id is used even for the other spelling
    assert assign_actor_id('A-B', {}) == 'A_B'


def test_frame_names():
    """Verify frame names."""
    assert bone_frame('movin', 'MOVINMan', 'Hips') == 'movin/MOVINMan/Hips'
    assert anchor_frame('movin', 'MOVINMan') == 'movin/MOVINMan'


# ---------------------------------------------------------------- local_edges


def test_local_edges_root_rule_and_shape():
    """Verify local edges root rule and shape."""
    bones = [
        _bone(0, -1, 'Root', p=(1.0, 2.0, 3.0)),
        _bone(1, 0, 'Hips', p=(0.0, 0.9, 0.0)),
        _bone(2, 1, 'Spine'),
        _bone(7, 99, 'Orphan'),   # parent index absent from the frame
        _bone(8, 8, 'Selfie'),    # self-parented
    ]
    edges = local_edges(bones)
    assert [e.name for e in edges] == ['Root', 'Hips', 'Spine', 'Orphan', 'Selfie']
    assert [e.parent_name for e in edges] == [None, 'Root', 'Hips', None, None]
    assert [e.bone_index for e in edges] == [0, 1, 2, 7, 8]
    assert edges[0].t_xyz == (-1.0, 2.0, 3.0)
    assert edges[0].q_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert isinstance(edges[0], Edge) and len(edges[0]) == 5
    assert edges[0] == (0, 'Root', None, (-1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))


def test_local_edges_root_found_by_parent_index_not_name(live_frames):
    """Verify local edges root found by parent index not name."""
    edges = local_edges(live_frames[0]['bones'])
    assert edges[0].name == 'RootBone' and edges[0].parent_name is None
    assert edges[1].name == 'Hips' and edges[1].parent_name == 'RootBone'
    assert len(edges) == 55
    assert sum(1 for e in edges if e.parent_name is None) == 1


# ------------------------------------------------------- quaternion helpers


def test_quat_helpers():
    """Verify quat helpers."""
    identity = (0.0, 0.0, 0.0, 1.0)
    rng = random.Random(7)
    for _ in range(10):
        w, x, y, z = _random_unit_wxyz(rng)
        q = (x, y, z, w)
        assert _close(quat_mul_xyzw(q, identity), q)
        assert _close(quat_mul_xyzw(identity, q), q)
        assert _close(quat_mul_xyzw(q, convert.quat_conjugate_xyzw(q)), identity, 1e-12)
        v = (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1))
        r = quat_rotate_xyzw(q, v)
        assert abs(math.dist(r, (0, 0, 0)) - math.dist(v, (0, 0, 0))) < 1e-12
    assert quat_normalize_xyzw((0, 0, 0, 0)) == identity
    assert _close(quat_normalize_xyzw((0, 0, 0, 3)), identity)


# ---------------------------------------------------------- forward_kinematics


def test_forward_kinematics_simple_chain():
    """Verify forward kinematics simple chain."""
    s = 0.7071067811865476
    bones = [
        _bone(0, -1, 'Root'),
        # Hips 1 m up, rotated +90 deg about Unity Y
        _bone(1, 0, 'Hips', p=(0.0, 1.0, 0.0), q=(s, 0.0, s, 0.0)),
        _bone(2, 1, 'Spine', p=(0.0, 0.0, 0.5)),
    ]
    world = forward_kinematics(local_edges(bones), ANCHOR_QUAT_XYZW['coord_equivalent'])
    assert set(world) == {'Root', 'Hips', 'Spine'}
    assert _close(world['Root'][0], (0, 0, 0))
    assert _close(world['Hips'][0], (0, 0, 1))
    # Unity: yaw +90 about Y maps local +Z to +X; mirrored -> ROS x = -unity x
    assert _close(world['Spine'][0], (-0.5, 0.0, 1.0), 1e-12)
    # anchor translation shifts everything
    shifted = forward_kinematics(local_edges(bones), ANCHOR_QUAT_XYZW['coord_equivalent'],
                                 anchor_t_xyz=(1.0, 2.0, 3.0))
    assert _close(shifted['Spine'][0], (0.5, 2.0, 4.0), 1e-12)


def test_forward_kinematics_accepts_any_edge_order_and_cycles():
    """Verify forward kinematics accepts any edge order and cycles."""
    bones = [_bone(0, -1, 'A', p=(1, 0, 0)), _bone(1, 0, 'B', p=(1, 0, 0))]
    edges = local_edges(bones)
    ordered = forward_kinematics(edges, (0, 0, 0, 1))
    reversed_ = forward_kinematics(list(reversed(edges)), (0, 0, 0, 1))
    assert ordered == reversed_
    assert _close(ordered['B'][0], (-2, 0, 0))
    cyc = [Edge(0, 'A', 'B', (1, 0, 0), (0, 0, 0, 1)), Edge(1, 'B', 'A', (1, 0, 0), (0, 0, 0, 1))]
    world = forward_kinematics(cyc, (0, 0, 0, 1))
    assert set(world) == {'A', 'B'}


def _sdk_reference(bones, mode):
    """SDK forward kinematics (numpy) in the requested forward mode; {name: (pos, quat_xyzw)}."""
    np = pytest.importorskip('numpy')
    from movin_sdk_python.utils.fk_utils import compute_forward_kinematics
    from movin_sdk_python.utils.isaac_lab_utils import Q_ISAAC_WORLD_FORWARD
    from movin_sdk_python.utils.quat_utils import quat_mul, quat_normalize, rotate_vec_by_quat

    result = compute_forward_kinematics(bones)
    reference = {}
    for name, (pos, quat) in result.items():
        pos = np.asarray(pos, dtype=np.float64)
        quat = np.asarray(quat, dtype=np.float64)
        if mode == 'isaac_world':
            pos = rotate_vec_by_quat(pos, Q_ISAAC_WORLD_FORWARD)
            quat = quat_normalize(quat_mul(Q_ISAAC_WORLD_FORWARD, quat))
        reference[name] = (tuple(pos), (quat[1], quat[2], quat[3], quat[0]))
    return reference


def _assert_parity(bones, mode):
    reference = _sdk_reference(bones, mode)
    world = forward_kinematics(local_edges(bones), ANCHOR_QUAT_XYZW[mode])
    assert set(world) == set(reference)
    for name, (pos, quat) in reference.items():
        t, q = world[name]
        assert _close(t, pos, PARITY_ATOL), (mode, name, t, pos)
        assert _close(q, quat, PARITY_ATOL), (mode, name, q, quat)


@pytest.mark.parametrize('mode', FORWARD_MODES)
def test_forward_kinematics_matches_sdk_on_random_trees(mode):
    """Verify forward kinematics matches sdk on random trees."""
    pytest.importorskip('numpy')
    rng = random.Random(1234)
    for count in (1, 2, 4, 6, 12, 55):
        for _ in range(4):
            _assert_parity(_random_tree(rng, count), mode)


@pytest.mark.parametrize('mode', FORWARD_MODES)
def test_forward_kinematics_matches_sdk_on_live_frames(mode, live_frames):
    """Verify forward kinematics matches sdk on live frames."""
    pytest.importorskip('numpy')
    for frame in live_frames[::5]:
        assert len(frame['bones']) == 55
        _assert_parity(frame['bones'], mode)


@pytest.mark.parametrize('mode', FORWARD_MODES)
def test_forward_kinematics_matches_sdk_on_legacy_frames(mode, legacy_frames):
    """Verify forward kinematics matches sdk on legacy frames."""
    pytest.importorskip('numpy')
    for frame in legacy_frames[::7]:
        assert len(frame['bones']) == 52
        _assert_parity(frame['bones'], mode)


def test_forward_kinematics_matches_sdk_with_unordered_parent(live_frames):
    """A missing parent index attaches to the anchor exactly like the SDK's root rule."""
    pytest.importorskip('numpy')
    bones = [dict(b) for b in live_frames[0]['bones']]
    bones[5]['parent_index'] = 999
    _assert_parity(bones, 'coord_equivalent')


# -------------------------------------------------------------------- yaw_qpos


def test_yaw_qpos_sanity():
    """Verify yaw qpos sanity."""
    pos, quat = yaw_qpos((1.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0), 'coord_equivalent', (0, 0, 1))
    assert _close(pos, (1.0, 0.0, 1.5)) and _close(quat, (0, 0, 0, 1))
    pos, quat = yaw_qpos((1.0, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0), 'isaac_world')
    assert _close(pos, (0.0, 1.0, 0.5), 1e-12)
    assert _close(quat, (0.0, 0.0, 0.7071067811865476, 0.7071067811865476))
    # a robot facing coord_equivalent -Y (the actor's forward) faces +X afterwards
    facing = quat_rotate_xyzw(quat, (0.0, -1.0, 0.0))
    assert _close(facing, (1.0, 0.0, 0.0), 1e-12)
    pos, quat = yaw_qpos((0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 2.0), 'isaac_world', (0.5, 0.0, 0.0))
    assert _close(pos, (-0.5, 0.0, 0.0), 1e-12)
    assert abs(math.sqrt(sum(c * c for c in quat)) - 1.0) < 1e-12
    with pytest.raises(ValueError):
        yaw_qpos((0, 0, 0), (1, 0, 0, 0), 'bogus')


def test_yaw_qpos_matches_sdk_forward_mode():
    """Verify yaw qpos matches sdk forward mode."""
    np = pytest.importorskip('numpy')
    from movin_sdk_python.utils.isaac_lab_utils import _apply_forward_mode_to_root

    rng = random.Random(99)
    for mode in FORWARD_MODES:
        for _ in range(5):
            pos = [rng.uniform(-2, 2) for _ in range(3)]
            quat = _random_unit_wxyz(rng)
            ref_pos, ref_quat = _apply_forward_mode_to_root(np.array(pos), np.array(quat), mode)
            got_pos, got_quat = yaw_qpos(pos, quat, mode)
            assert _close(got_pos, ref_pos, PARITY_ATOL)
            assert _close(got_quat, (ref_quat[1], ref_quat[2], ref_quat[3], ref_quat[0]),
                          PARITY_ATOL)


# --------------------------------------------------------------- cylinder_pose


def test_cylinder_pose_generic_and_degenerate():
    """Verify cylinder pose generic and degenerate."""
    center, q, length = cylinder_pose((0.0, 0.0, 2.0))
    assert _close(center, (0, 0, 1)) and _close(q, (0, 0, 0, 1)) and length == 2.0
    center, q, length = cylinder_pose((0.0, 0.0, 0.0))
    assert _close(center, (0, 0, 0)) and q == (0.0, 0.0, 0.0, 1.0) and length == 0.0
    rng = random.Random(3)
    for _ in range(25):
        p = tuple(rng.uniform(-1, 1) for _ in range(3))
        center, q, length = cylinder_pose(p)
        assert abs(length - math.dist(p, (0, 0, 0))) < 1e-12
        assert _close(center, tuple(0.5 * c for c in p))
        axis = quat_rotate_xyzw(q, (0.0, 0.0, length))
        assert _close(axis, p, 1e-9)


def test_cylinder_pose_anti_parallel():
    """Verify cylinder pose anti parallel."""
    center, q, length = cylinder_pose((0.0, 0.0, -3.0))
    assert _close(center, (0, 0, -1.5)) and length == 3.0
    assert abs(math.sqrt(sum(c * c for c in q)) - 1.0) < 1e-12
    assert _close(quat_rotate_xyzw(q, (0.0, 0.0, 3.0)), (0.0, 0.0, -3.0), 1e-12)
    # nearly anti-parallel stays finite and correct on both sides of the snap threshold
    center, q, length = cylinder_pose((1e-4, 0.0, -1.0))
    assert all(math.isfinite(c) for c in q)
    assert _close(quat_rotate_xyzw(q, (0.0, 0.0, length)), (1e-4, 0.0, -1.0), 1e-9)
    center, q, length = cylinder_pose((1e-7, 0.0, -1.0))
    assert all(math.isfinite(c) for c in q)
    assert _close(quat_rotate_xyzw(q, (0.0, 0.0, length)), (1e-7, 0.0, -1.0), 1e-6)
