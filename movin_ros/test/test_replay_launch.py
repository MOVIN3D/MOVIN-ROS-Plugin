"""
Launch test: replay the recorded fixture through ``mocap_node`` and verify its output.

Three nodes replay the installed fixture
``share/movin_ros/test/data/live_movinman_v3_1s.pkl``:

* ``/movin/mocap_node`` loops the recording and exits after ``max_frames``;
  its ``mocap_frame``, ``/tf`` and ``/tf_static`` output is checked, including
  a tf2 composition of ``world -> movin/MOVINMan/<bone>`` against
  :func:`movin_ros.convert.forward_kinematics`.
* ``/movin_end/mocap_node`` plays the recording once with
  ``replay_exit_on_end`` and must exit 0 on its own after logging
  ``replay finished``.
* ``/movin_keep/mocap_node`` loops silently until the harness sends SIGINT
  (launch_testing stops the launch as soon as every process has exited),
  which also checks the interrupt path in replay mode.
* ``/movin_fast/mocap_node`` replays once with ``replay_realtime: false``:
  every one of the 54 recorded frames must be published (lossless fast
  replay) before it exits 0 on its own.
* ``/movin_empty/mocap_node`` is given a recording without a complete frame
  and must exit 1 with a fatal message instead of hanging.

Only runs under ROS 2 (``launch_testing``); plain pytest on a ROS-free host skips it.
"""

from __future__ import annotations

import os
import sys
import tempfile
import threading
import time
import unittest

import pytest

# `launch_test <file>` outside pytest: make osc_helpers importable (conftest does it for pytest).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from ament_index_python.packages import get_package_share_directory
    from osc_helpers import LIVE_FIXTURE, wait_until
    import launch
    import launch_ros.actions
    import launch_testing
    import launch_testing.actions
    import launch_testing.asserts
    from movin_msgs.msg import MocapFrame, MocapStatus
    from movin_ros.convert import ANCHOR_QUAT_XYZW, forward_kinematics, local_edges
    import rclpy
    from rclpy import qos
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    import tf2_ros
    from movin_sdk_python import OscRecorder
except ImportError:  # ROS-free host: launch tests only run under colcon test
    pytest.skip('launch tests need a ROS 2 environment', allow_module_level=True)

MAX_FRAMES = 120
MIN_FRAMES = 30
FIXTURE_FRAMES = 54
ACTOR = 'MOVINMan'
ANCHOR = 'movin/MOVINMan'
BONE_COUNT = 55
STATIC_QOS = qos.QoSProfile(
    depth=10,
    reliability=qos.ReliabilityPolicy.RELIABLE,
    durability=qos.DurabilityPolicy.TRANSIENT_LOCAL)


def fixture_path() -> str:
    """Return the installed fixture path (share/movin_ros/test/data), else the source copy."""
    installed = os.path.join(
        get_package_share_directory('movin_ros'), 'test', 'data', 'live_movinman_v3_1s.pkl')
    return installed if os.path.exists(installed) else LIVE_FIXTURE


def empty_recording_path() -> str:
    """Write a valid SDK recording with zero messages (what an idle ``record:=auto`` produced)."""
    path = os.path.join(tempfile.mkdtemp(prefix='movin_replay_empty_'), 'empty.pkl')
    OscRecorder(path, stream_type='movin').save()
    return path


def _mocap_node(namespace: str, extra_params: dict) -> launch_ros.actions.Node:
    params = {
        'mode': 'replay',
        'recording': fixture_path(),
        'replay_realtime': True,
        'forward_mode': 'isaac_world',
        'poll_rate_hz': 120.0,
    }
    params.update(extra_params)
    return launch_ros.actions.Node(
        package='movin_ros',
        executable='mocap_node',
        name='mocap_node',
        namespace=namespace,
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[params],
    )


@pytest.mark.launch_test
def generate_test_description():
    """Launch the looping node (max_frames), the play-once node and the keep-alive node."""
    mocap_node = _mocap_node('movin', {'replay_loop': True, 'max_frames': MAX_FRAMES})
    end_node = _mocap_node('movin_end', {
        'replay_loop': False, 'replay_exit_on_end': True, 'publish_tf': False})
    keep_node = _mocap_node('movin_keep', {
        'replay_loop': True, 'publish_tf': False, 'publish_frame': False})
    fast_node = _mocap_node('movin_fast', {
        'replay_realtime': False, 'replay_loop': False, 'replay_exit_on_end': True,
        'publish_tf': False})
    empty_node = _mocap_node('movin_empty', {
        'recording': empty_recording_path(), 'replay_loop': True, 'publish_tf': False})
    return launch.LaunchDescription([
        mocap_node,
        end_node,
        keep_node,
        fast_node,
        empty_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mocap_node': mocap_node, 'end_node': end_node, 'keep_node': keep_node,
         'fast_node': fast_node, 'empty_node': empty_node}


class _Listener:
    """rclpy node (private context) spun on a background thread; stores received messages."""

    def __init__(self, name: str):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(name, context=self.context)
        self.lock = threading.Lock()
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)

    def subscribe(self, msg_type, topic, profile, store):
        def callback(msg):
            with self.lock:
                store.append(msg)
        return self.node.create_subscription(msg_type, topic, callback, profile)

    def start(self):
        self.thread.start()

    def stop(self):
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


def _output_text(proc_output, process) -> str:
    try:
        events = list(proc_output[process])
    except KeyError:
        return ''
    return ''.join(event.text.decode('utf-8', errors='replace') for event in events)


def _bones_from_message(msg) -> list:
    bones = []
    for bone in msg.bones:
        bones.append({
            'bone_index': bone.bone_index,
            'parent_index': bone.parent_index,
            'bone_name': bone.name,
            'p': (bone.position.x, bone.position.y, bone.position.z),
            'q': (bone.rotation.w, bone.rotation.x, bone.rotation.y, bone.rotation.z),
        })
    return bones


class TestReplay(unittest.TestCase):
    """Active tests: run while the nodes replay the fixture."""

    @classmethod
    def setUpClass(cls):
        """Create the listener and subscribe the mocap, TF and status topics."""
        cls.listener = _Listener('test_replay_listener')
        cls.frames = []
        cls.tf_msgs = []
        cls.static_msgs = []
        cls.status_msgs = []
        cls.listener.subscribe(MocapFrame, '/movin/mocap_frame', 10, cls.frames)
        cls.listener.subscribe(TFMessage, '/tf', 100, cls.tf_msgs)
        cls.listener.subscribe(TFMessage, '/tf_static', STATIC_QOS, cls.static_msgs)
        cls.listener.subscribe(MocapStatus, '/movin/status', 10, cls.status_msgs)
        cls.listener.start()

    @classmethod
    def tearDownClass(cls):
        """Stop the listener."""
        cls.listener.stop()

    def _snapshot(self, store):
        with self.listener.lock:
            return list(store)

    def test_01_frames(self):
        """At least MIN_FRAMES MocapFrame messages arrive with the expected content."""
        ok = wait_until(
            lambda: len(self._snapshot(self.frames)) >= MIN_FRAMES
            and len(self._snapshot(self.tf_msgs)) >= MIN_FRAMES, timeout=30.0)
        frames = self._snapshot(self.frames)
        self.assertTrue(ok, f'received only {len(frames)} frames')
        first_idx = None
        last_idx = None
        for msg in frames:
            self.assertEqual(msg.actor, ACTOR)
            self.assertEqual(msg.actor_frame, ANCHOR)
            self.assertEqual(msg.preset, 'movinman_v3')
            self.assertEqual(msg.convention, MocapFrame.CONVENTION_UNITY_LH_YUP)
            self.assertEqual(msg.header.frame_id, 'world')
            self.assertEqual(len(msg.bones), BONE_COUNT)
            self.assertEqual(msg.bones[0].parent_index, -1)
            self.assertEqual(msg.bones[1].name, 'Hips')
            self.assertTrue(msg.source_timestamp)
            if first_idx is None:
                first_idx = msg.frame_idx
            elif msg.frame_idx <= last_idx:
                # replay_loop restarts at the first recorded frame
                self.assertLessEqual(msg.frame_idx, first_idx)
            last_idx = msg.frame_idx

    def test_02_tf(self):
        """Every /tf message carries all bone edges; /tf_static carries the actor anchor."""
        tf_msgs = self._snapshot(self.tf_msgs)
        self.assertGreaterEqual(len(tf_msgs), MIN_FRAMES)
        for msg in tf_msgs:
            self.assertEqual(len(msg.transforms), BONE_COUNT)
            parents = {t.child_frame_id: t.header.frame_id for t in msg.transforms}
            self.assertEqual(parents[f'{ANCHOR}/RootBone'], ANCHOR)
            self.assertEqual(parents[f'{ANCHOR}/Hips'], f'{ANCHOR}/RootBone')
            stamps = {(t.header.stamp.sec, t.header.stamp.nanosec) for t in msg.transforms}
            self.assertEqual(len(stamps), 1)
        self.assertTrue(wait_until(lambda: self._snapshot(self.static_msgs), timeout=10.0))
        anchors = {}
        for msg in self._snapshot(self.static_msgs):
            for t in msg.transforms:
                anchors[t.child_frame_id] = t
        self.assertIn(ANCHOR, anchors)
        anchor = anchors[ANCHOR]
        self.assertEqual(anchor.header.frame_id, 'world')
        rotation = anchor.transform.rotation
        expected = ANCHOR_QUAT_XYZW['isaac_world']
        for got, want in zip((rotation.x, rotation.y, rotation.z, rotation.w), expected):
            self.assertAlmostEqual(got, want, places=12)

    def test_03_tf2_matches_forward_kinematics(self):
        """tf2 composition of world -> bone equals convert.forward_kinematics for every bone."""
        frames = self._snapshot(self.frames)
        tf_by_stamp = {}
        for msg in self._snapshot(self.tf_msgs):
            stamp = msg.transforms[0].header.stamp
            tf_by_stamp[(stamp.sec, stamp.nanosec)] = msg
        matches = [
            m for m in frames if (m.header.stamp.sec, m.header.stamp.nanosec) in tf_by_stamp]
        self.assertTrue(matches, 'no MocapFrame with a matching /tf message')
        frame_msg = matches[-1]
        stamp = frame_msg.header.stamp
        tf_msg = tf_by_stamp[(stamp.sec, stamp.nanosec)]

        buffer = tf2_ros.Buffer()
        for static in self._snapshot(self.static_msgs):
            for t in static.transforms:
                buffer.set_transform_static(t, 'test')
        for t in tf_msg.transforms:
            buffer.set_transform(t, 'test')

        expected = forward_kinematics(
            local_edges(_bones_from_message(frame_msg)), ANCHOR_QUAT_XYZW['isaac_world'])
        self.assertIn('LeftToeBase', expected)
        for name, (t_xyz, q_xyzw) in expected.items():
            got = buffer.lookup_transform('world', f'{ANCHOR}/{name}', Time.from_msg(stamp))
            tr = got.transform.translation
            rot = got.transform.rotation
            for a, b in zip((tr.x, tr.y, tr.z), t_xyz):
                self.assertAlmostEqual(a, b, delta=1e-6, msg=name)
            # quaternions may differ by sign
            dot = rot.x * q_xyzw[0] + rot.y * q_xyzw[1] + rot.z * q_xyzw[2] + rot.w * q_xyzw[3]
            self.assertAlmostEqual(abs(dot), 1.0, delta=1e-6, msg=name)

    def test_04_status(self):
        """Status reports replay mode, the actor, the preset and no dropped frames."""
        self.assertTrue(wait_until(
            lambda: any(m.frames_published > 0 for m in self._snapshot(self.status_msgs)),
            timeout=15.0))
        status = [m for m in self._snapshot(self.status_msgs) if m.frames_published > 0][-1]
        self.assertEqual(status.mode, 'replay')
        self.assertEqual(status.bind_address, '')
        self.assertEqual(status.error, '')
        self.assertEqual(status.frame_delivery, 'drain')
        self.assertEqual(status.frames_dropped, 0)
        self.assertEqual(list(status.actors), [ACTOR])
        self.assertEqual(status.preset, 'movinman_v3')
        self.assertFalse(status.recording)

    def test_05_nodes_exit_on_their_own(self, proc_info, proc_output, mocap_node, end_node):
        """The play-once node logs 'replay finished' and exits; the looping one stops at max."""
        proc_info.assertWaitForShutdown(process=end_node, timeout=40)
        proc_info.assertWaitForShutdown(process=mocap_node, timeout=40)
        self.assertTrue(
            wait_until(lambda: 'replay finished' in _output_text(proc_output, end_node),
                       timeout=10.0),
            'end node did not log "replay finished"')
        self.assertTrue(
            wait_until(
                lambda: f'max_frames={MAX_FRAMES} reached' in _output_text(
                    proc_output, mocap_node),
                timeout=10.0),
            'looping node did not log the max_frames exit')
        time.sleep(0.2)
        self.assertGreaterEqual(len(self._snapshot(self.frames)), MIN_FRAMES)

    def test_06_fast_replay_is_lossless(self, proc_info, proc_output, fast_node):
        """replay_realtime:=false publishes every recorded frame, then exits on its own."""
        proc_info.assertWaitForShutdown(process=fast_node, timeout=40)
        self.assertTrue(
            wait_until(
                lambda: f'replay finished ({FIXTURE_FRAMES} frames published)' in _output_text(
                    proc_output, fast_node),
                timeout=10.0),
            f'fast node output:\n{_output_text(proc_output, fast_node)}')
        self.assertIn('lossless', _output_text(proc_output, fast_node))

    def test_07_frameless_recording_is_fatal(self, proc_info, proc_output, empty_node):
        """A recording without a complete frame ends the node with a fatal message, no hang."""
        proc_info.assertWaitForShutdown(process=empty_node, timeout=40)
        self.assertTrue(
            wait_until(lambda: 'contains no complete frame' in _output_text(
                proc_output, empty_node), timeout=10.0),
            f'empty node output:\n{_output_text(proc_output, empty_node)}')


@launch_testing.post_shutdown_test()
class TestReplayExitCodes(unittest.TestCase):
    """Exit codes: 0 for the replaying nodes, 1 for the frame-less recording."""

    def test_exit_codes(self, proc_info, mocap_node, end_node, keep_node, fast_node, empty_node):
        """max_frames, replay_exit_on_end and SIGINT end with 0; the fatal startup with 1."""
        for process in (mocap_node, end_node, keep_node, fast_node):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=[0], process=process)
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[1], process=empty_node)
