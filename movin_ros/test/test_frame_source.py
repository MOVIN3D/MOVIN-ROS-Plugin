"""Tests for ``movin_ros.frame_source`` against a real ``MocapReceiver`` fed over UDP."""

from __future__ import annotations

import os
import pickle
import time

from movin_ros import frame_source
from movin_ros.frame_source import (
    bound_address,
    build_receiver,
    detect_preset,
    drain_frames,
    FastReplayReceiver,
    first_complete_frame,
    receiver_thread_died,
    recording_summary,
    replay_finished,
    validate_frame,
    validate_recording,
)

from movin_sdk_python import MocapReceiver, OscPlayer, OscRecorder, ReplayMocapReceiver

from osc_helpers import frame_packets, ready_frame_count, send_packets, wait_until

import pytest

GapTracker = frame_source.GapTracker


def _send_frames(receiver, frames):
    """Send the chunk packets of every frame to ``receiver``; return packets sent."""
    packets = [packet for chunks in frames for packet in chunks]
    return send_packets(packets, receiver.sock.getsockname()[:2])


def _wait_ready(receiver, count, timeout=3.0):
    assert wait_until(lambda: ready_frame_count(receiver) >= count, timeout), (
        f'expected {count} ready frames, got {ready_frame_count(receiver)}')


# ------------------------------------------------------------- build_receiver


def test_build_receiver_live_is_not_started():
    """Verify build receiver live is not started."""
    receiver = build_receiver('live', host='127.0.0.1', port=0, recording='',
                              replay_realtime=True, replay_loop=True)
    assert isinstance(receiver, MocapReceiver)
    assert receiver.host == '127.0.0.1' and receiver.port == 0
    assert receiver.sock is None and receiver.thread is None
    assert receiver.recorder is None


def test_build_receiver_live_attaches_recorder():
    """Verify build receiver live attaches recorder."""
    marker = object()
    receiver = build_receiver('live', host='0.0.0.0', port=0, recording='',
                              replay_realtime=True, replay_loop=True, recorder=marker)
    assert receiver.recorder is marker


def test_build_receiver_replay(live_fixture_path):
    """Realtime replay uses the SDK receiver, non-realtime the lossless FastReplayReceiver."""
    receiver = build_receiver('replay', host='0.0.0.0', port=11235, recording=live_fixture_path,
                              replay_realtime=True, replay_loop=False)
    assert isinstance(receiver, ReplayMocapReceiver)
    assert receiver.realtime is True and receiver.loop is False
    assert receiver.thread is None
    assert receiver.player.num_messages == 162
    fast = build_receiver('replay', host='0.0.0.0', port=11235, recording=live_fixture_path,
                          replay_realtime=False, replay_loop=True)
    assert isinstance(fast, FastReplayReceiver)
    assert fast.loop is True and fast.running is False and fast.finished is False
    assert fast.player.num_messages == 162
    assert not hasattr(fast, 'recorder')


def test_build_receiver_replay_rejects_legacy_pickle(tmp_path):
    """A non-SDK pickle fails with the friendly message from build_receiver (both receivers)."""
    legacy = tmp_path / 'motion.pkl'
    with open(legacy, 'wb') as f:
        pickle.dump({'fps': 30, 'root_pos': [], 'root_rot': [], 'dof_pos': []}, f)
    for realtime in (True, False):
        with pytest.raises(ValueError) as excinfo:
            build_receiver('replay', host='0.0.0.0', port=0, recording=str(legacy),
                           replay_realtime=realtime, replay_loop=False)
        assert 'not a MOVIN SDK recording' in str(excinfo.value)


def test_build_receiver_errors(tmp_path):
    """Verify build receiver errors."""
    with pytest.raises(ValueError):
        build_receiver('bogus', host='0.0.0.0', port=0, recording='',
                       replay_realtime=True, replay_loop=True)
    with pytest.raises(ValueError):
        build_receiver('replay', host='0.0.0.0', port=0, recording='',
                       replay_realtime=True, replay_loop=True)
    with pytest.raises(FileNotFoundError):
        build_receiver('replay', host='0.0.0.0', port=0, recording=str(tmp_path / 'nope.pkl'),
                       replay_realtime=True, replay_loop=True)


# ---------------------------------------------------------- validate_recording


def test_validate_recording_live_fixture(live_fixture_path):
    """Verify validate recording live fixture."""
    info = validate_recording(live_fixture_path)
    assert info['num_messages'] == 162
    assert info['stream_type'] == 'movin'
    assert 0.9 < info['duration_sec'] < 1.1
    assert recording_summary(OscPlayer(live_fixture_path)) == info


def _empty_recording(path) -> str:
    """Write a valid version-1 SDK recording with zero messages (what an idle node produces)."""
    OscRecorder(str(path), stream_type='movin').save()
    return str(path)


def _partial_recording(path, live_messages) -> str:
    """Write a recording holding only the first chunk of a frame (no complete frame)."""
    recorder = OscRecorder(str(path), stream_type='movin')
    address, args = live_messages[0]
    recorder.record(address, args, wall_time=0.0)
    recorder.save()
    return str(path)


def test_first_complete_frame(live_fixture_path, live_frames, live_messages, tmp_path):
    """first_complete_frame equals peek_first_frame without re-reading the file; None if none."""
    frame = first_complete_frame(OscPlayer(live_fixture_path))
    assert frame['frame_idx'] == live_frames[0]['frame_idx'] == 13899
    assert len(frame['bones']) == 55 and detect_preset(frame) == 'movinman_v3'
    empty = _empty_recording(tmp_path / 'empty.pkl')
    assert validate_recording(empty)['num_messages'] == 0
    assert first_complete_frame(OscPlayer(empty)) is None
    partial = _partial_recording(tmp_path / 'partial.pkl', live_messages)
    assert validate_recording(partial)['num_messages'] == 1
    assert first_complete_frame(OscPlayer(partial)) is None


def test_validate_recording_legacy_fixture(legacy_fixture_path):
    """Verify validate recording legacy fixture."""
    info = validate_recording(legacy_fixture_path)
    assert info == {'num_messages': 90, 'duration_sec': pytest.approx(0.4837, abs=1e-3),
                    'stream_type': 'movin'}


def test_validate_recording_rejects_legacy_motion_player_pickle(tmp_path):
    """Verify validate recording rejects legacy motion player pickle."""
    legacy = tmp_path / 'motion.pkl'
    with open(legacy, 'wb') as f:
        pickle.dump({'fps': 30, 'root_pos': [[0.0, 0.0, 0.9]], 'root_rot': [[0, 0, 0, 1]],
                     'dof_pos': [[0.0] * 29]}, f)
    with pytest.raises(ValueError) as excinfo:
        validate_recording(str(legacy))
    message = str(excinfo.value)
    assert message.startswith(str(legacy))
    assert 'not a MOVIN SDK recording' in message
    assert 'legacy motion_player .pkl / BVH files are not supported' in message
    assert isinstance(excinfo.value.__cause__, ValueError)


def test_validate_recording_rejects_unpicklable_numpy_style_pickle(tmp_path):
    """Verify validate recording rejects unpicklable numpy style pickle."""
    # A pickle whose payload needs a module that is not importable, like the
    # numpy arrays of the legacy format on a host without numpy.
    bad = tmp_path / 'needs_missing_module.pkl'
    bad.write_bytes(b'cno_such_legacy_module\nMotion\n.')
    with pytest.raises(ValueError) as excinfo:
        validate_recording(str(bad))
    assert 'not a MOVIN SDK recording' in str(excinfo.value)
    assert isinstance(excinfo.value.__cause__, ModuleNotFoundError)


def test_validate_recording_rejects_garbage_and_missing_file(tmp_path):
    """Verify validate recording rejects garbage and missing file."""
    garbage = tmp_path / 'garbage.pkl'
    garbage.write_bytes(b'HIERARCHY\nROOT Hips\n')
    with pytest.raises(ValueError):
        validate_recording(str(garbage))
    (tmp_path / 'empty.pkl').write_bytes(b'')
    with pytest.raises(ValueError):
        validate_recording(str(tmp_path / 'empty.pkl'))
    with pytest.raises(FileNotFoundError):
        validate_recording(str(tmp_path / 'missing.pkl'))


def test_validate_recording_expands_user(monkeypatch, live_fixture_path):
    """Verify validate recording expands user."""
    monkeypatch.setenv('HOME', os.path.dirname(live_fixture_path))
    info = validate_recording('~/' + os.path.basename(live_fixture_path))
    assert info['num_messages'] == 162


# ---------------------------------------------------------------- drain_frames


@pytest.mark.parametrize('count', [1, 2, 3, 4])
def test_drain_frames_returns_all_frames_in_order(local_receiver, live_messages, count):
    """Verify drain frames returns all frames in order."""
    frames = frame_packets(live_messages, frame_count=count)
    assert all(len(chunks) == 3 for chunks in frames)
    _send_frames(local_receiver, frames)
    _wait_ready(local_receiver, count)
    drained, delivery = drain_frames(local_receiver)
    assert delivery == 'drain'
    assert len(drained) == count
    indices = [f['frame_idx'] for f in drained]
    assert indices == sorted(indices) and len(set(indices)) == count
    assert indices[0] == 13899
    assert all(f['actor'] == 'MOVINMan' and len(f['bones']) == 55 for f in drained)
    assert drain_frames(local_receiver) == ([], 'drain')


def test_drain_frames_sdk_queue_keeps_only_last_four(local_receiver, live_messages):
    """The SDK deque is maxlen=4: a 5th/6th frame before a poll evicts the oldest."""
    frames = frame_packets(live_messages, frame_count=6)
    expected_last = 13899 + 5
    _send_frames(local_receiver, frames)

    def last_is_sixth():
        """Return True once the 6th frame is the newest queued frame."""
        with local_receiver.lock:
            ready = local_receiver.assembler.ready_frames
            return bool(ready) and ready[-1]['frame_idx'] == expected_last

    assert wait_until(last_is_sixth)
    assert local_receiver.assembler.ready_frames.maxlen == frame_source.SDK_READY_QUEUE_DEPTH == 4
    drained, delivery = drain_frames(local_receiver)
    assert delivery == 'drain'
    assert [f['frame_idx'] for f in drained] == [13901, 13902, 13903, 13904]
    assert 'maxlen=4' in drain_frames.__doc__


def test_drain_frames_two_actor_interleave_keeps_both(local_receiver, live_messages):
    """Verify drain frames two actor interleave keeps both."""
    actor_a = frame_packets(live_messages, frame_count=2, actor='A')
    actor_b = frame_packets(live_messages, frame_count=2, actor='B', frame_idx_offset=100000)
    interleaved = [actor_a[0], actor_b[0], actor_a[1], actor_b[1]]
    _send_frames(local_receiver, interleaved)
    _wait_ready(local_receiver, 4)
    drained, delivery = drain_frames(local_receiver)
    assert delivery == 'drain'
    assert [(f['actor'], f['frame_idx']) for f in drained] == [
        ('A', 13899), ('B', 113899), ('A', 13900), ('B', 113900)]


def test_get_latest_frame_returns_only_newest(local_receiver, live_messages):
    """Verify get latest frame returns only newest."""
    frames = frame_packets(live_messages, frame_count=3)
    _send_frames(local_receiver, frames)
    _wait_ready(local_receiver, 3)
    frame = local_receiver.get_latest_frame()
    assert frame['frame_idx'] == 13901
    assert local_receiver.get_latest_frame() is None
    assert drain_frames(local_receiver) == ([], 'drain')


def test_drain_frames_rereads_assembler_after_reset(local_receiver, live_messages):
    """start()/stop()/reset() replace ``receiver.assembler``; draining must follow."""
    before = local_receiver.assembler
    local_receiver.reset()
    assert local_receiver.assembler is not before
    _send_frames(local_receiver, frame_packets(live_messages, frame_count=1))
    _wait_ready(local_receiver, 1)
    drained, _ = drain_frames(local_receiver)
    assert len(drained) == 1


def test_drain_frames_falls_back_to_latest_only():
    """Verify drain frames falls back to latest only."""
    class Minimal:
        def __init__(self):
            """Initialize the helper."""
            self.frames = [{'frame_idx': 1}, {'frame_idx': 2}]

        def get_latest_frame(self):
            """Pop the newest queued frame (lossy SDK API)."""
            return self.frames.pop() if self.frames else None

    receiver = Minimal()
    assert drain_frames(receiver) == ([{'frame_idx': 2}], 'latest_only')
    assert drain_frames(receiver) == ([{'frame_idx': 1}], 'latest_only')
    assert drain_frames(receiver) == ([], 'latest_only')


def test_drain_frames_uses_get_new_frames():
    """Any receiver with get_new_frames() is drained through it, never via its internals."""
    class Pulling:
        def __init__(self):
            """Initialize the helper."""
            self.batches = [[{'frame_idx': 1}, {'frame_idx': 2}], []]

        def get_new_frames(self):
            """Return the next batch (oldest first)."""
            return self.batches.pop(0)

        def get_latest_frame(self):
            """Must not be used when get_new_frames() exists."""
            raise AssertionError('get_latest_frame() called')

    receiver = Pulling()
    assert drain_frames(receiver) == ([{'frame_idx': 1}, {'frame_idx': 2}], 'drain')
    assert drain_frames(receiver) == ([], 'drain')


def test_drain_frames_counts_toward_studio_viewer_rate(local_receiver, live_messages):
    """Drained frames are what MOVIN Studio's status panel shows as the viewer rate."""
    def viewer(now):
        args = local_receiver.status.reply_args('0' * 32, 'MOVINMan', None, now)
        return args[5], args[8]  # viewer fps, viewer age (-1 = no frame handed out yet)

    frames = frame_packets(live_messages, frame_count=4)
    _send_frames(local_receiver, frames[:2])
    _wait_ready(local_receiver, 2)
    assert viewer(time.monotonic()) == (0.0, -1.0)
    assert len(drain_frames(local_receiver)[0]) == 2
    time.sleep(0.05)
    _send_frames(local_receiver, frames[2:])
    _wait_ready(local_receiver, 2)
    assert len(drain_frames(local_receiver)[0]) == 2
    fps, age = viewer(time.monotonic())
    assert fps > 0.0 and 0.0 <= age < 1.0


def test_drain_frames_on_sdk_replay_receiver_is_lossy(live_fixture_path):
    """The SDK ReplayMocapReceiver(realtime=False) bursts the file: only the last 4 frames stay."""
    receiver = ReplayMocapReceiver(live_fixture_path, realtime=False, loop=False)
    assert replay_finished(receiver) is False
    receiver.start()
    assert wait_until(lambda: replay_finished(receiver), timeout=5.0)
    drained, delivery = drain_frames(receiver)
    assert delivery == 'drain'
    assert 1 <= len(drained) <= 4
    assert drained[-1]['frame_idx'] == 13899 + 53
    receiver.stop()
    assert replay_finished(receiver) is False  # stop() clears the thread


# ---------------------------------------------------------- FastReplayReceiver


def _pump_all(receiver, frames_per_pump=frame_source.FAST_REPLAY_FRAMES_PER_POLL,
              max_polls=10000) -> list:
    """Poll like the node (pump then drain) until the replay finished; return every frame."""
    frames = []
    for _ in range(max_polls):
        receiver.pump(frames_per_pump)
        drained, delivery = drain_frames(receiver)
        assert delivery == 'drain'
        frames.extend(drained)
        if replay_finished(receiver):
            break
    return frames


def test_fast_replay_receiver_is_lossless(live_fixture_path, live_frames):
    """Non-realtime replay delivers every frame of the recording, in order, exactly once."""
    receiver = FastReplayReceiver(live_fixture_path, loop=False)
    assert receiver.pump() == 0                     # not started
    assert replay_finished(receiver) is False
    receiver.start()
    assert receiver.running and not receiver.finished
    frames = _pump_all(receiver)
    assert replay_finished(receiver) is True
    assert [f['frame_idx'] for f in frames] == [f['frame_idx'] for f in live_frames]
    assert len(frames) == 54 and frames[0]['frame_idx'] == 13899
    assert all(len(f['bones']) == 55 for f in frames)
    assert receiver.messages_replayed == 162 and receiver.loops_completed == 0
    assert receiver.pump() == 0                     # finished: nothing more
    assert drain_frames(receiver) == ([], 'drain')
    assert receiver.get_latest_frame() is None
    assert receiver.get_receive_rate() >= 0.0
    receiver.stop()
    assert receiver.running is False
    receiver.start()                                # start() rewinds
    assert not receiver.finished
    assert len(_pump_all(receiver)) == 54


def test_fast_replay_receiver_pump_budget(live_fixture_path):
    """pump(n) queues at most n frames per call and the queue depth bounds it."""
    receiver = FastReplayReceiver(live_fixture_path, loop=False, max_ready_frames=16)
    receiver.start()
    assert receiver.pump(3) > 0
    assert ready_frame_count(receiver) == 3
    assert receiver.pump(3) == 0                    # queue already holds 3
    drained, _ = drain_frames(receiver)
    assert [f['frame_idx'] for f in drained] == [13899, 13900, 13901]
    receiver.pump(100)                              # capped at max_ready_frames
    assert ready_frame_count(receiver) == 16
    assert receiver.assembler.ready_frames.maxlen == 16
    assert receiver.get_latest_frame()['frame_idx'] == 13899 + 3 + 15
    assert ready_frame_count(receiver) == 0


def test_fast_replay_receiver_loops(live_fixture_path):
    """With loop=True the file restarts at its end (frame_idx wraps) and never finishes."""
    receiver = FastReplayReceiver(live_fixture_path, loop=True)
    receiver.start()
    frames = []
    while len(frames) < 54 + 10:
        receiver.pump(8)
        frames.extend(drain_frames(receiver)[0])
    indices = [f['frame_idx'] for f in frames]
    assert indices[:54] == list(range(13899, 13899 + 54))
    assert indices[54:64] == list(range(13899, 13899 + 10))
    assert receiver.loops_completed == 1 and not receiver.finished
    assert replay_finished(receiver) is False
    receiver.stop()


def test_fast_replay_receiver_frameless_recordings_do_not_hang(tmp_path, live_messages):
    """Empty / partial recordings: pump() returns, non-loop finishes, loop rewinds once a call."""
    for path in (_empty_recording(tmp_path / 'e.pkl'),
                 _partial_recording(tmp_path / 'p.pkl', live_messages)):
        once = FastReplayReceiver(path, loop=False)
        once.start()
        assert once.pump(8) <= 1
        assert once.finished and replay_finished(once)
        assert drain_frames(once) == ([], 'drain')
        forever = FastReplayReceiver(path, loop=True)
        forever.start()
        for _ in range(3):
            assert forever.pump(8) <= 2            # one rewind per call
        assert not forever.finished and forever.loops_completed == 3


# ------------------------------------------------------------- detect/validate


def test_detect_preset(live_frames, legacy_frames):
    """Verify detect preset."""
    assert detect_preset(live_frames[0]) == 'movinman_v3'
    assert detect_preset(legacy_frames[0]) == 'movinman'
    assert detect_preset({'bones': [{'bone_name': 'Hips'}]}) == 'movinman'
    assert detect_preset({}) == ''
    assert detect_preset(None) == ''


def test_validate_frame_clean(live_frames, legacy_frames):
    """Verify validate frame clean."""
    assert validate_frame(live_frames[0]) == []
    assert validate_frame(legacy_frames[0]) == []


def test_validate_frame_reports_problems(live_frames):
    """Verify validate frame reports problems."""
    bones = [dict(b) for b in live_frames[0]['bones']]
    bones[3]['parent_index'] = 10          # child before parent
    bones[4]['parent_index'] = 4           # self-parented
    bones[6]['parent_index'] = 999         # absent parent
    bones[7]['bone_name'] = bones[8]['bone_name']  # duplicate name
    warnings = validate_frame({'bones': bones})
    assert len(warnings) == 3
    assert any('parents-first' in w for w in warnings)
    assert any('absent from the frame' in w for w in warnings)
    assert any('duplicate bone names' in w for w in warnings)
    assert validate_frame({}) == ['frame has no bones list']


# ------------------------------------------------------------------ GapTracker


def test_gap_tracker():
    """Verify gap tracker."""
    tracker = GapTracker()
    assert tracker.update('A', 10) == 0
    assert tracker.update('A', 11) == 0
    assert tracker.update('A', 14) == 2
    assert tracker.update('B', 5) == 0
    assert tracker.update('B', 4) == 0      # non-increasing: reset, no gap
    assert tracker.update('B', 6) == 1
    assert tracker.update('A', 14) == 0     # duplicate index
    assert tracker.frames_seen == 7
    assert tracker.frames_dropped == 3
    assert tracker.actors == ['A', 'B']
    assert tracker.last_frame_idx('A') == 14 and tracker.last_frame_idx('C') is None
    tracker.reset()
    assert tracker.frames_seen == 0 and tracker.frames_dropped == 0 and tracker.actors == []


# ------------------------------------------------------------- misc helpers


def test_bound_address(local_receiver):
    """Verify bound address."""
    host, port = local_receiver.sock.getsockname()[:2]
    assert bound_address(local_receiver) == f'{host}:{port}'
    assert host == '127.0.0.1' and port > 0
    assert bound_address(MocapReceiver(host='127.0.0.1', port=0)) == ''
    assert bound_address(object()) == ''


def test_replay_finished_is_false_for_live(local_receiver):
    """Verify replay finished is false for live."""
    assert replay_finished(local_receiver) is False


@pytest.mark.filterwarnings('ignore::pytest.PytestUnhandledThreadExceptionWarning')
def test_receiver_thread_died(local_receiver, live_messages, live_fixture_path):
    """A UDP thread that ends on its own is detected while the receiver still says running."""
    assert receiver_thread_died(local_receiver) is False
    assert receiver_thread_died(MocapReceiver(host='127.0.0.1', port=0)) is False
    assert receiver_thread_died(FastReplayReceiver(live_fixture_path)) is False

    class Exploding:
        def record(self, address, args, wall_time=None):
            """Fail like an out-of-memory recorder on the UDP thread."""
            raise MemoryError('simulated')

    local_receiver.recorder = Exploding()
    _send_frames(local_receiver, frame_packets(live_messages, frame_count=1))
    assert wait_until(lambda: receiver_thread_died(local_receiver), timeout=3.0)
    assert local_receiver.running is True and local_receiver.sock is None
    local_receiver.recorder = None
