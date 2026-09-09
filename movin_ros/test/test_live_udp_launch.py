"""
Launch test: feed ``mocap_node`` over UDP with the fixture re-sent at its recorded timing.

The node binds ``127.0.0.1:21235``; the test process re-encodes the OSC
messages of ``live_movinman_v3_1s.pkl`` and sends them with their original
timing.  Every frame must be published (no drops), the status must reflect
the stream, and the ``start_recording`` / ``split_recording`` /
``stop_recording`` services must round-trip with ``num_messages`` equal to
the packets sent while recording.

Only runs under ROS 2 (``launch_testing``); plain pytest on a ROS-free host skips it.
"""

from __future__ import annotations

import os
import socket
import sys
import tempfile
import threading
import time
import unittest

import pytest

# `launch_test <file>` outside pytest: make osc_helpers importable (conftest does it for pytest).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from osc_helpers import LIVE_FIXTURE, osc_message, wait_until
    import launch
    import launch_ros.actions
    import launch_testing
    import launch_testing.actions
    import launch_testing.asserts
    from movin_msgs.msg import MocapFrame, MocapStatus
    from movin_msgs.srv import StartRecording, StopRecording
    from movin_sdk_python import OscPlayer, peek_first_frame
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
except ImportError:  # ROS-free host: launch tests only run under colcon test
    pytest.skip('launch tests need a ROS 2 environment', allow_module_level=True)

HOST = '127.0.0.1'
PORT = 21235
ADDRESS = (HOST, PORT)
BONE_COUNT = 55
RECORD_DIR = tempfile.mkdtemp(prefix='movin_live_udp_')


@pytest.mark.launch_test
def generate_test_description():
    """Launch mocap_node in live mode on the loopback test port."""
    mocap_node = launch_ros.actions.Node(
        package='movin_ros',
        executable='mocap_node',
        name='mocap_node',
        namespace='movin',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
            'mode': 'live',
            'host': HOST,
            'port': PORT,
            'record_dir': RECORD_DIR,
            'record_segment_sec': 600.0,
            'poll_rate_hz': 120.0,
            'forward_mode': 'isaac_world',
        }],
    )
    return launch.LaunchDescription([
        mocap_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mocap_node': mocap_node}


def send_recording(address, frame_limit=None, frame_idx_offset=0) -> tuple:
    """
    Re-send the fixture's OSC messages to ``address`` at their recorded timing.

    Returns ``(packets_sent, frame_indices)``; ``frame_limit`` stops after that
    many frames and ``frame_idx_offset`` shifts every ``frame_idx`` so repeated
    sends look like a continuing stream.
    """
    player = OscPlayer(LIVE_FIXTURE)
    seen = []
    packets = 0
    start = time.monotonic()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        for t, addr, args in player.messages_with_timing():
            args = list(args)
            key = (args[1], int(args[2]))
            if not seen or seen[-1] != key:
                if frame_limit is not None and len(seen) >= frame_limit:
                    break
                seen.append(key)
            args[2] = int(args[2]) + frame_idx_offset
            delay = t - (time.monotonic() - start)
            if delay > 0:
                time.sleep(delay)
            sender.sendto(osc_message(addr, args), address)
            packets += 1
    return packets, [idx + frame_idx_offset for _actor, idx in seen]


class _Listener:
    """rclpy node (private context) spun on a background thread; topics and service calls."""

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

    def call(self, srv_type, name, request, timeout=15.0):
        client = self.node.create_client(srv_type, name)
        try:
            assert client.wait_for_service(timeout_sec=timeout), f'{name} unavailable'
            future = client.call_async(request)
            assert wait_until(future.done, timeout=timeout), f'{name} timed out'
            return future.result()
        finally:
            self.node.destroy_client(client)

    def start(self):
        self.thread.start()

    def stop(self):
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


class TestLiveUdp(unittest.TestCase):
    """Active tests against the live node (ordered by name)."""

    @classmethod
    def setUpClass(cls):
        """Create the listener and subscribe mocap_frame and status."""
        cls.listener = _Listener('test_live_udp_listener')
        cls.frames = []
        cls.status_msgs = []
        cls.listener.subscribe(MocapFrame, '/movin/mocap_frame', 10, cls.frames)
        cls.listener.subscribe(MocapStatus, '/movin/status', 10, cls.status_msgs)
        cls.listener.start()
        cls.expected_frames = 0

    @classmethod
    def tearDownClass(cls):
        """Stop the listener."""
        cls.listener.stop()

    def _snapshot(self, store):
        with self.listener.lock:
            return list(store)

    def _frame_count(self) -> int:
        return len(self._snapshot(self.frames))

    def _wait_frames(self, count: int, timeout: float = 15.0) -> None:
        self.assertTrue(wait_until(lambda: self._frame_count() >= count, timeout=timeout),
                        f'expected {count} frames, got {self._frame_count()}')
        time.sleep(0.3)  # let stragglers (if any) arrive before asserting an exact count
        self.assertEqual(self._frame_count(), count)

    def _latest_status(self, predicate, timeout: float = 5.0):
        self.assertTrue(wait_until(
            lambda: any(predicate(m) for m in self._snapshot(self.status_msgs)),
            timeout=timeout), 'no status message matched')
        return [m for m in self._snapshot(self.status_msgs) if predicate(m)][-1]

    def _send(self, frame_limit=None, frame_idx_offset=0) -> tuple:
        packets, indices = send_recording(ADDRESS, frame_limit, frame_idx_offset)
        type(self).expected_frames += len(indices)
        self._wait_frames(self.expected_frames)
        return packets, indices

    def test_01_node_is_bound(self):
        """The node reports its bind address and the subscriber is matched."""
        status = self._latest_status(lambda m: m.bind_address == f'{HOST}:{PORT}', timeout=30.0)
        self.assertEqual(status.mode, 'live')
        self.assertEqual(status.error, '')
        self.assertFalse(status.receiving)
        self.assertTrue(wait_until(
            lambda: self.listener.node.count_publishers('/movin/mocap_frame') >= 1,
            timeout=10.0))
        time.sleep(0.5)

    def test_02_every_frame_is_published(self):
        """All 54 frames of the fixture arrive in order with no drops."""
        packets, indices = self._send()
        self.assertEqual(packets, 162)
        self.assertEqual(len(indices), 54)
        frames = self._snapshot(self.frames)
        self.assertEqual([m.frame_idx for m in frames], indices)
        for msg in frames:
            self.assertEqual(msg.actor, 'MOVINMan')
            self.assertEqual(msg.preset, 'movinman_v3')
            self.assertEqual(len(msg.bones), BONE_COUNT)
        status = self._latest_status(lambda m: m.frames_published >= 54)
        self.assertEqual(status.frames_dropped, 0)
        self.assertEqual(status.frame_delivery, 'drain')
        self.assertEqual(list(status.actors), ['MOVINMan'])
        self.assertEqual(status.preset, 'movinman_v3')
        self.assertFalse(status.recording)
        self.assertGreater(self._latest_status(lambda m: m.receive_rate_hz > 0).receive_rate_hz, 0)

    def test_03_start_stop_recording(self):
        """start_recording / stop_recording round trip: num_messages == packets sent."""
        path = os.path.join(RECORD_DIR, 'take.pkl')
        started = self.listener.call(
            StartRecording, '/movin/start_recording',
            StartRecording.Request(path=path, stream_type=''))
        self.assertTrue(started.success, started.message)
        self.assertEqual(started.path, path)
        again = self.listener.call(
            StartRecording, '/movin/start_recording',
            StartRecording.Request(path='', stream_type=''))
        self.assertFalse(again.success)
        packets, _indices = self._send(frame_limit=20, frame_idx_offset=1000)
        status = self._latest_status(lambda m: m.recording and m.recording_messages >= packets)
        self.assertEqual(status.recording_path, path)
        self.assertEqual(status.recording_messages, packets)
        stopped = self.listener.call(
            StopRecording, '/movin/stop_recording', StopRecording.Request(save=True))
        self.assertTrue(stopped.success, stopped.message)
        self.assertEqual(stopped.path, path)
        self.assertEqual(stopped.num_messages, packets)
        self.assertGreater(stopped.duration_sec, 0.1)
        self.assertTrue(os.path.exists(path))
        player = OscPlayer(path)
        self.assertEqual(player.num_messages, packets)
        self.assertEqual(player.stream_type, 'movin')
        frame = peek_first_frame(path)
        self.assertEqual(len(frame['bones']), BONE_COUNT)
        self.assertEqual(frame['frame_idx'], 13899 + 1000)

    def test_04_split_recording(self):
        """split_recording swaps segments without losing a packet."""
        started = self.listener.call(
            StartRecording, '/movin/start_recording',
            StartRecording.Request(path='', stream_type=''))
        self.assertTrue(started.success, started.message)
        first_path = started.path
        self.assertTrue(first_path.startswith(RECORD_DIR))
        self.assertTrue(first_path.endswith('_000.pkl'))
        packets_a, _ = self._send(frame_limit=10, frame_idx_offset=2000)
        self._latest_status(lambda m: m.recording_messages >= packets_a)
        split = self.listener.call(Trigger, '/movin/split_recording', Trigger.Request())
        self.assertTrue(split.success, split.message)
        self.assertIn(first_path, split.message)
        packets_b, _ = self._send(frame_limit=10, frame_idx_offset=3000)
        status = self._latest_status(
            lambda m: m.recording_segment == 1 and m.recording_messages >= packets_b)
        second_path = status.recording_path
        self.assertNotEqual(second_path, first_path)
        self.assertTrue(second_path.endswith('_001.pkl'))
        stopped = self.listener.call(
            StopRecording, '/movin/stop_recording', StopRecording.Request(save=True))
        self.assertTrue(stopped.success, stopped.message)
        self.assertEqual(stopped.path, second_path)
        self.assertEqual(stopped.num_messages, packets_b)

        def first_saved():
            try:
                return OscPlayer(first_path).num_messages == packets_a
            except Exception:
                return False
        self.assertTrue(wait_until(first_saved, timeout=10.0), 'first segment not saved')
        self.assertEqual(OscPlayer(second_path).num_messages, packets_b)
        self.assertEqual(packets_a + packets_b, 60)

    def test_05_stop_when_idle_fails(self):
        """stop_recording / split_recording report failure when nothing is recording."""
        stopped = self.listener.call(
            StopRecording, '/movin/stop_recording', StopRecording.Request(save=True))
        self.assertFalse(stopped.success)
        self.assertIn('not recording', stopped.message)
        split = self.listener.call(Trigger, '/movin/split_recording', Trigger.Request())
        self.assertFalse(split.success)


@launch_testing.post_shutdown_test()
class TestLiveUdpExitCode(unittest.TestCase):
    """The node must exit cleanly when launch shuts it down."""

    def test_exit_code(self, proc_info, mocap_node):
        """SIGINT from launch ends the node with exit code 0."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0], process=mocap_node)
