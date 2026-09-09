"""
Launch test: ``record:=auto`` plus SIGINT leaves a loadable SDK recording behind.

``mocap_node`` starts in live mode with ``record: auto`` (recorder attached
before the socket is bound), the active test streams a few frames of the
fixture over UDP to ``127.0.0.1:21236`` and confirms they were counted but
not yet saved.  launch_testing then shuts the launch down, which sends SIGINT
to the node exactly like Ctrl-C on ``ros2 launch``; the post-shutdown test
checks the exit code and that the auto-named ``.pkl`` in ``record_dir`` opens
with ``OscPlayer`` / ``peek_first_frame``.

Only runs under ROS 2 (``launch_testing``); plain pytest on a ROS-free host skips it.
"""

from __future__ import annotations

import glob
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
    from movin_msgs.msg import MocapStatus
    from movin_sdk_python import OscPlayer, peek_first_frame
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
except ImportError:  # ROS-free host: launch tests only run under colcon test
    pytest.skip('launch tests need a ROS 2 environment', allow_module_level=True)

HOST = '127.0.0.1'
PORT = 21236
FRAMES_TO_SEND = 10
RECORD_DIR = tempfile.mkdtemp(prefix='movin_shutdown_')
# Filled by the active test, read by the post-shutdown test.
STATE = {'path': '', 'packets': 0}


@pytest.mark.launch_test
def generate_test_description():
    """Launch mocap_node in live mode with record:=auto into a temporary record_dir."""
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
            'record': 'auto',
            'record_dir': RECORD_DIR,
            'record_segment_sec': 600.0,
        }],
    )
    return launch.LaunchDescription([
        mocap_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mocap_node': mocap_node}


def send_frames(address, frame_limit: int) -> int:
    """Send the first ``frame_limit`` frames of the fixture at recorded timing; return packets."""
    player = OscPlayer(LIVE_FIXTURE)
    seen = []
    packets = 0
    start = time.monotonic()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        for t, addr, args in player.messages_with_timing():
            key = (args[1], int(args[2]))
            if not seen or seen[-1] != key:
                if len(seen) >= frame_limit:
                    break
                seen.append(key)
            delay = t - (time.monotonic() - start)
            if delay > 0:
                time.sleep(delay)
            sender.sendto(osc_message(addr, list(args)), address)
            packets += 1
    return packets


class TestRecordingBeforeShutdown(unittest.TestCase):
    """Stream frames into the auto-started recording (nothing is saved yet)."""

    @classmethod
    def setUpClass(cls):
        """Create a status listener (private rclpy context) spun on a background thread."""
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('test_shutdown_listener', context=cls.context)
        cls.lock = threading.Lock()
        cls.status_msgs = []

        def on_status(msg):
            with cls.lock:
                cls.status_msgs.append(msg)

        cls.subscription = cls.node.create_subscription(
            MocapStatus, '/movin/status', on_status, 10)
        cls.executor = SingleThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        """Stop the listener."""
        cls.executor.shutdown()
        cls.thread.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def _status(self, predicate, timeout: float):
        def matching():
            with self.lock:
                return [m for m in self.status_msgs if predicate(m)]
        self.assertTrue(wait_until(matching, timeout=timeout), 'no status message matched')
        return matching()[-1]

    def test_frames_are_recorded(self):
        """The recorder is active from startup and counts every packet sent."""
        status = self._status(
            lambda m: m.recording and m.bind_address == f'{HOST}:{PORT}', timeout=30.0)
        path = status.recording_path
        self.assertTrue(path.startswith(RECORD_DIR), path)
        self.assertRegex(os.path.basename(path), r'^movin_\d{8}_\d{6}_000\.pkl$')
        self.assertEqual(status.recording_segment, 0)
        self.assertEqual(status.recording_messages, 0)
        self.assertEqual(status.mode, 'live')

        packets = send_frames((HOST, PORT), FRAMES_TO_SEND)
        self.assertEqual(packets, 3 * FRAMES_TO_SEND)
        status = self._status(
            lambda m: m.recording_messages >= packets and m.frames_published >= FRAMES_TO_SEND,
            timeout=10.0)
        self.assertEqual(status.recording_messages, packets)
        self.assertEqual(status.frames_published, FRAMES_TO_SEND)
        self.assertEqual(status.frames_dropped, 0)
        self.assertFalse(os.path.exists(path), 'recording saved before shutdown')
        STATE['path'] = path
        STATE['packets'] = packets


@launch_testing.post_shutdown_test()
class TestRecordingSavedOnSigint(unittest.TestCase):
    """After the harness' SIGINT the recording must be on disk and loadable."""

    def test_exit_code(self, proc_info, mocap_node):
        """The interrupted node exits 0 after flushing the recording."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0], process=mocap_node)

    def test_recording_saved(self):
        """The auto-named .pkl exists, has every packet and yields a complete first frame."""
        path = STATE['path']
        packets = STATE['packets']
        self.assertTrue(path, 'active test did not record the path')
        self.assertTrue(wait_until(lambda: os.path.exists(path), timeout=10.0), path)
        self.assertEqual(glob.glob(os.path.join(RECORD_DIR, 'movin_*_000.pkl')), [path])
        player = OscPlayer(path)
        self.assertEqual(player.num_messages, packets)
        self.assertEqual(player.stream_type, 'movin')
        self.assertGreater(player.duration_sec, 0.0)
        frame = peek_first_frame(path)
        self.assertIsNotNone(frame)
        self.assertEqual(frame['actor'], 'MOVINMan')
        self.assertEqual(frame['frame_idx'], 13899)
        self.assertEqual(len(frame['bones']), 55)
