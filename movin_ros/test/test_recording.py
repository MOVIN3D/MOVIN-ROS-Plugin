"""Tests for ``movin_ros.recording`` (CountingRecorder, auto paths, RecorderManager)."""

from __future__ import annotations

import datetime
import os
import pickle
import re

from movin_ros import recording
from movin_ros.frame_source import build_receiver

from movin_sdk_python import OscPlayer, OscRecorder, ReplayMocapReceiver
from movin_sdk_python.recording import peek_first_frame

from osc_helpers import frame_packets, ready_frame_count, send_packets, wait_until

import pytest

REPLAY_UNAVAILABLE = recording.REPLAY_UNAVAILABLE
NOTHING_RECORDED = recording.NOTHING_RECORDED
CountingRecorder = recording.CountingRecorder
RecorderManager = recording.RecorderManager
auto_recording_path = recording.auto_recording_path
ensure_writable = recording.ensure_writable
resolve_recording_path = recording.resolve_recording_path


class FakeClock:
    """Injectable monotonic clock."""

    def __init__(self, now=0.0):
        """Initialize the helper."""
        self.now = now

    def __call__(self):
        """Return the current fake time."""
        return self.now


class ListLogger:
    """Collects log calls as (level, message)."""

    def __init__(self):
        """Initialize the helper."""
        self.records = []

    def info(self, message):
        """Record an info message."""
        self.records.append(('info', message))

    def warning(self, message):
        """Record a warning message."""
        self.records.append(('warning', message))

    def error(self, message):
        """Record a error message."""
        self.records.append(('error', message))


def _live_receiver(recorder=None):
    return build_receiver('live', host='127.0.0.1', port=0, recording='',
                          replay_realtime=True, replay_loop=True, recorder=recorder)


def _send_frames(receiver, frames):
    packets = [packet for chunks in frames for packet in chunks]
    return send_packets(packets, receiver.sock.getsockname()[:2])


def _wait_recorded(recorder, count, timeout=3.0):
    assert wait_until(lambda: recorder.num_messages >= count, timeout), (
        f'expected {count} recorded messages, got {recorder.num_messages}')


@pytest.fixture
def started_receiver():
    """Start a loopback MocapReceiver for one test and stop it afterwards."""
    receiver = _live_receiver()
    receiver.start()
    try:
        yield receiver
    finally:
        receiver.stop()


# ------------------------------------------------------------ CountingRecorder


def test_counting_recorder_counts_and_saves(tmp_path, live_messages):
    """Verify counting recorder counts and saves."""
    path = str(tmp_path / 'count.pkl')
    recorder = CountingRecorder(path)
    assert isinstance(recorder, OscRecorder)
    assert recorder.num_messages == 0 and recorder.first_t is None and recorder.last_t is None
    assert recorder.duration_sec == 0.0
    for i, (address, args) in enumerate(live_messages[:6]):
        recorder.record(address, args, wall_time=100.0 + 0.5 * i)
    assert recorder.num_messages == 6
    assert recorder.first_t == 100.0 and recorder.last_t == 102.5
    assert recorder.duration_sec == pytest.approx(2.5)
    recorder.record('/MOVIN/Frame', live_messages[6][1])   # wall_time defaults to now
    assert recorder.num_messages == 7 and recorder.last_t > 102.5
    recorder.save()
    player = OscPlayer(path)
    assert player.num_messages == 7
    assert player.stream_type == 'movin'
    assert list(player.messages())[0][1] == list(live_messages[0][1])


# --------------------------------------------------------- auto_recording_path


def test_auto_recording_path(tmp_path):
    """Verify auto recording path."""
    when = datetime.datetime(2026, 9, 9, 16, 15, 0)
    record_dir = tmp_path / 'nested' / 'dir'
    path = auto_recording_path(str(record_dir), when=when)
    assert path == str(record_dir / 'movin_20260909_161500.pkl')
    assert record_dir.is_dir()
    assert auto_recording_path(str(record_dir), when=when, segment=3) == str(
        record_dir / 'movin_20260909_161500_003.pkl')
    now_path = auto_recording_path(str(record_dir))
    assert re.fullmatch(r'movin_\d{8}_\d{6}\.pkl', os.path.basename(now_path))


def test_auto_recording_path_expands_user(tmp_path, monkeypatch):
    """Verify auto recording path expands user."""
    monkeypatch.setenv('HOME', str(tmp_path))
    path = auto_recording_path('~/movin_recordings', when=datetime.datetime(2026, 1, 2, 3, 4, 5))
    assert path == str(tmp_path / 'movin_recordings' / 'movin_20260102_030405.pkl')
    assert (tmp_path / 'movin_recordings').is_dir()


# ---------------------------------------------- ensure_writable / resolve path


def test_ensure_writable_probes_without_leaving_a_file(tmp_path):
    """A writable path is returned expanded, parent created, and no file is left behind."""
    path = ensure_writable(str(tmp_path / 'new' / 'dir' / 'take.pkl'))
    assert path == str(tmp_path / 'new' / 'dir' / 'take.pkl')
    assert (tmp_path / 'new' / 'dir').is_dir() and not os.path.exists(path)
    existing = tmp_path / 'existing.pkl'
    existing.write_bytes(b'keep me')
    assert ensure_writable(str(existing)) == str(existing)
    assert existing.read_bytes() == b'keep me'          # append probe leaves the content


def test_ensure_writable_fails_early(tmp_path):
    """A directory as path or an unwritable directory raise OSError at start time."""
    with pytest.raises(IsADirectoryError):
        ensure_writable(str(tmp_path))
    if os.geteuid() == 0:
        pytest.skip('root ignores directory permissions')
    locked = tmp_path / 'locked'
    locked.mkdir()
    locked.chmod(0o500)
    try:
        with pytest.raises(PermissionError):
            ensure_writable(str(locked / 'take.pkl'))
    finally:
        locked.chmod(0o700)


def test_resolve_recording_path(tmp_path, monkeypatch):
    """'' / 'auto' select an auto name (segment suffix with rotation), else the explicit path."""
    monkeypatch.setenv('HOME', str(tmp_path))
    auto = resolve_recording_path('', str(tmp_path / 'rec'), 0.0)
    assert re.fullmatch(r'movin_\d{8}_\d{6}\.pkl', os.path.basename(auto))
    assert os.path.dirname(auto) == str(tmp_path / 'rec') and not os.path.exists(auto)
    rotating = resolve_recording_path('auto', str(tmp_path / 'rec'), 600.0)
    assert re.fullmatch(r'movin_\d{8}_\d{6}_000\.pkl', os.path.basename(rotating))
    explicit = resolve_recording_path('~/takes/t1.pkl', str(tmp_path / 'rec'), 600.0)
    assert explicit == str(tmp_path / 'takes' / 't1.pkl') and (tmp_path / 'takes').is_dir()
    with pytest.raises(IsADirectoryError):
        resolve_recording_path(str(tmp_path / 'rec'), str(tmp_path / 'rec'), 0.0)


# ------------------------------------------------------------ RecorderManager


def test_constructor_attached_recorder_keeps_first_packet(tmp_path, live_messages):
    """Attach-before-start: the recorder passed into MocapReceiver() sees packet #1."""
    path = str(tmp_path / 'first.pkl')
    recorder = CountingRecorder(path, stream_type='movin')
    receiver = _live_receiver(recorder=recorder)
    manager = RecorderManager(receiver, record_dir=str(tmp_path), segment_sec=0.0,
                              initial=(recorder, path))
    assert manager.is_recording and manager.current_path == path
    assert manager.segment_index == 0 and manager.messages == 0
    receiver.start()
    try:
        frames = frame_packets(live_messages, frame_count=2)
        sent = _send_frames(receiver, frames)
        assert sent == 6
        _wait_recorded(recorder, sent)
        assert manager.messages == sent
        assert wait_until(lambda: ready_frame_count(receiver) >= 2)
    finally:
        receiver.stop()
    stopped_path, count, duration = manager.stop()
    assert stopped_path == path and count == sent and duration >= 0.0
    assert not manager.is_recording and manager.current_path == ''
    assert receiver.recorder is None
    player = OscPlayer(path)
    assert player.num_messages == sent
    first_address, first_args = next(player.messages())
    assert first_address == '/MOVIN/Frame'
    assert first_args[:7] == list(live_messages[0][1][:7])   # header of frame 13899, chunk 0
    assert first_args[4] == 0


def test_start_stop_round_trip_and_peek(tmp_path, started_receiver, live_messages):
    """Verify start stop round trip and peek."""
    logger = ListLogger()
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0,
                              logger=logger)
    assert not manager.is_recording
    path = manager.start(str(tmp_path / 'sub' / 'take.pkl'))
    assert path == str(tmp_path / 'sub' / 'take.pkl')
    assert manager.is_recording and manager.current_path == path
    assert isinstance(started_receiver.recorder, CountingRecorder)
    with pytest.raises(RuntimeError):
        manager.start()
    sent = _send_frames(started_receiver, frame_packets(live_messages, frame_count=3))
    _wait_recorded(started_receiver.recorder, sent)
    assert manager.messages == sent and manager.duration_sec >= 0.0
    stopped_path, count, _ = manager.stop()
    assert stopped_path == path and count == sent
    assert started_receiver.recorder is None
    with pytest.raises(RuntimeError):
        manager.stop()
    frame = peek_first_frame(path)
    assert frame['actor'] == 'MOVINMan' and len(frame['bones']) == 55
    assert frame['frame_idx'] == 13899
    assert [level for level, _ in logger.records] == ['info', 'info']


def test_start_rejects_unwritable_path(tmp_path, started_receiver):
    """start() probes the path: a directory fails immediately and nothing is attached."""
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0)
    with pytest.raises(IsADirectoryError):
        manager.start(str(tmp_path))
    assert not manager.is_recording and started_receiver.recorder is None
    path = manager.start(str(tmp_path / 'ok.pkl'))
    assert manager.is_recording and path == str(tmp_path / 'ok.pkl')
    manager.stop(save=False)


def test_stop_never_writes_an_empty_recording(tmp_path, started_receiver):
    """A recorder that saw no packet is not written (logged as nothing recorded)."""
    logger = ListLogger()
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0,
                              logger=logger)
    path = manager.start(str(tmp_path / 'idle.pkl'))
    assert manager.stop(save=True) == (path, 0, 0.0)
    assert not os.path.exists(path)
    assert any(level == 'warning' and NOTHING_RECORDED in message and path in message
               for level, message in logger.records)
    manager.start(str(tmp_path / 'idle2.pkl'))
    manager.close()
    assert not os.path.exists(tmp_path / 'idle2.pkl')


def test_stop_keeps_the_take_when_saving_fails(tmp_path, started_receiver, live_messages):
    """A failed save re-attaches the recorder: stop again after fixing, or discard."""
    logger = ListLogger()
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0,
                              logger=logger)
    path = manager.start(str(tmp_path / 'blocked.pkl'))
    sent = _send_frames(started_receiver, frame_packets(live_messages, frame_count=3))
    recorder = started_receiver.recorder
    _wait_recorded(recorder, sent)
    os.mkdir(path)                                   # the path is now a directory: save fails
    with pytest.raises(IsADirectoryError):
        manager.stop(save=True)
    assert manager.is_recording and manager.current_path == path
    assert started_receiver.recorder is recorder and manager.messages == sent
    assert any(level == 'error' and 'stays active' in message for level, message in logger.records)
    with pytest.raises(RuntimeError):                # still recording: cannot start another
        manager.start(str(tmp_path / 'other.pkl'))
    # more packets keep being recorded into the same take
    sent += _send_frames(started_receiver, frame_packets(live_messages, frame_count=2))
    _wait_recorded(recorder, sent)
    os.rmdir(path)                                   # "fix the disk", then stop again
    assert manager.stop(save=True) == (path, sent, pytest.approx(recorder.duration_sec))
    assert OscPlayer(path).num_messages == sent
    assert started_receiver.recorder is None and not manager.is_recording


def test_close_falls_back_to_a_temp_file(tmp_path, started_receiver, live_messages,
                                         monkeypatch):
    """At shutdown a failed save is retried into a movin_unsaved_*.pkl temp file."""
    logger = ListLogger()
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0,
                              logger=logger)
    path = manager.start(str(tmp_path / 'doomed.pkl'))
    sent = _send_frames(started_receiver, frame_packets(live_messages, frame_count=2))
    _wait_recorded(started_receiver.recorder, sent)
    monkeypatch.setattr('tempfile.tempdir', str(tmp_path / 'fallback'))
    (tmp_path / 'fallback').mkdir()
    os.mkdir(path)                                   # primary save fails
    manager.close()
    assert not manager.is_recording and started_receiver.recorder is None
    fallbacks = [p for p in (tmp_path / 'fallback').iterdir()
                 if p.name.startswith('movin_unsaved_') and p.suffix == '.pkl']
    assert len(fallbacks) == 1
    assert OscPlayer(str(fallbacks[0])).num_messages == sent
    assert peek_first_frame(str(fallbacks[0]))['frame_idx'] == 13899
    assert any(level == 'warning' and str(fallbacks[0]) in message
               for level, message in logger.records)


def test_stop_without_save_discards(tmp_path, started_receiver, live_messages):
    """Verify stop without save discards."""
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0)
    path = manager.start(str(tmp_path / 'discard.pkl'))
    sent = _send_frames(started_receiver, frame_packets(live_messages, frame_count=1))
    _wait_recorded(started_receiver.recorder, sent)
    assert manager.stop(save=False) == (path, sent, pytest.approx(manager.duration_sec, abs=1.0))
    assert not os.path.exists(path)


def test_auto_path_naming(tmp_path, started_receiver):
    """Verify auto path naming."""
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path / 'rec'),
                              segment_sec=0.0)
    path = manager.start()
    assert os.path.dirname(path) == str(tmp_path / 'rec')
    assert re.fullmatch(r'movin_\d{8}_\d{6}\.pkl', os.path.basename(path))
    manager.stop(save=False)
    rotating = RecorderManager(started_receiver, record_dir=str(tmp_path / 'rec'),
                               segment_sec=600.0)
    path = rotating.start('auto')
    assert re.fullmatch(r'movin_\d{8}_\d{6}_000\.pkl', os.path.basename(path))
    rotating.stop(save=False)


def test_split_loses_no_packets(tmp_path, started_receiver, live_messages):
    """Verify split loses no packets."""
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0)
    path0 = manager.start(str(tmp_path / 'seg.pkl'))
    frames = frame_packets(live_messages, frame_count=8)
    sent_a = _send_frames(started_receiver, frames[:3])
    _wait_recorded(started_receiver.recorder, sent_a)
    old_path, new_path = manager.split()
    assert old_path == path0 and new_path == str(tmp_path / 'seg_001.pkl')
    assert manager.segment_index == 1 and manager.current_path == new_path
    assert manager.messages == 0
    sent_b = _send_frames(started_receiver, frames[3:6])
    _wait_recorded(started_receiver.recorder, sent_b)
    old_path2, new_path2 = manager.split()
    assert (old_path2, new_path2) == (new_path, str(tmp_path / 'seg_002.pkl'))
    sent_c = _send_frames(started_receiver, frames[6:])
    _wait_recorded(started_receiver.recorder, sent_c)
    manager.close()
    assert not manager.is_recording
    total = sum(OscPlayer(p).num_messages for p in (path0, new_path, new_path2))
    assert total == sent_a + sent_b + sent_c == 24
    assert OscPlayer(path0).num_messages == sent_a
    assert OscPlayer(new_path2).num_messages == sent_c
    # the segments are contiguous: frame indices continue across the boundary
    idx = [args[2] for p in (path0, new_path, new_path2) for _, args in OscPlayer(p).messages()]
    assert idx == sorted(idx) and idx[0] == 13899 and idx[-1] == 13899 + 7
    assert peek_first_frame(new_path)['frame_idx'] == 13899 + 3


def test_split_names_from_segment_zero_suffix(tmp_path, started_receiver, live_messages):
    """Verify split names from segment zero suffix."""
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=1.0)
    path0 = manager.start(str(tmp_path / 'take_000.pkl'))
    frames = frame_packets(live_messages, frame_count=2)
    _wait_recorded(started_receiver.recorder, _send_frames(started_receiver, frames[:1]))
    _, path1 = manager.split()
    assert path1 == str(tmp_path / 'take_001.pkl')
    _wait_recorded(started_receiver.recorder, _send_frames(started_receiver, frames[1:]))
    manager.close()
    assert os.path.exists(path0) and os.path.exists(path1)


def test_split_skips_empty_segments(tmp_path, started_receiver, live_messages):
    """An empty segment is not written; numbering continues; later segments are saved."""
    logger = ListLogger()
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=1.0,
                              logger=logger)
    path0 = manager.start(str(tmp_path / 'gap_000.pkl'))
    _, path1 = manager.split()                       # nothing recorded into _000
    sent = _send_frames(started_receiver, frame_packets(live_messages, frame_count=1))
    _wait_recorded(started_receiver.recorder, sent)
    manager.close()
    assert not os.path.exists(path0) and OscPlayer(path1).num_messages == sent
    assert any(NOTHING_RECORDED in message and path0 in message for _, message in logger.records)


def test_tick_auto_splits_on_segment_sec(tmp_path, started_receiver, live_messages):
    """Verify tick auto splits on segment sec."""
    clock = FakeClock(100.0)
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=5.0,
                              clock=clock)
    frames = frame_packets(live_messages, frame_count=3)
    manager.tick(1000.0)               # not recording: no-op
    path0 = manager.start()
    assert path0.endswith('_000.pkl')
    _wait_recorded(started_receiver.recorder, _send_frames(started_receiver, frames[:1]))
    manager.tick(104.9)
    assert manager.segment_index == 0 and manager.current_path == path0
    clock.now = 105.0
    manager.tick(105.0)
    assert manager.segment_index == 1
    assert manager.current_path == path0.replace('_000.pkl', '_001.pkl')
    _wait_recorded(started_receiver.recorder, _send_frames(started_receiver, frames[1:2]))
    manager.tick(109.9)
    assert manager.segment_index == 1
    manager.reset_segment_start(108.0)  # e.g. the receiver re-bound: period restarts
    manager.tick(110.0)
    assert manager.segment_index == 1
    manager.tick(113.0)
    assert manager.segment_index == 2
    _wait_recorded(started_receiver.recorder, _send_frames(started_receiver, frames[2:]))
    manager.close()
    for index in range(3):
        assert os.path.exists(path0.replace('_000.pkl', f'_{index:03d}.pkl'))
    idle = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=5.0)
    idle.reset_segment_start()         # not recording: no-op
    assert not idle.is_recording


def test_tick_never_splits_without_rotation(tmp_path, started_receiver):
    """Verify tick never splits without rotation."""
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0,
                              clock=FakeClock(0.0))
    manager.start(str(tmp_path / 'norot.pkl'))
    manager.tick(1e9)
    assert manager.segment_index == 0
    manager.close()


def test_close_saves_and_is_idempotent(tmp_path, started_receiver, live_messages):
    """Verify close saves and is idempotent."""
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0)
    path = manager.start(str(tmp_path / 'close.pkl'))
    sent = _send_frames(started_receiver, frame_packets(live_messages, frame_count=1))
    _wait_recorded(started_receiver.recorder, sent)
    manager.close()
    manager.close()
    assert not manager.is_recording
    assert peek_first_frame(path)['frame_idx'] == 13899
    assert OscPlayer(path).num_messages == sent


def test_replay_mode_cannot_record(tmp_path, live_fixture_path):
    """Verify replay mode cannot record."""
    receiver = ReplayMocapReceiver(live_fixture_path, realtime=False, loop=False)
    manager = RecorderManager(receiver, record_dir=str(tmp_path), segment_sec=0.0)
    with pytest.raises(RuntimeError) as excinfo:
        manager.start()
    assert str(excinfo.value) == REPLAY_UNAVAILABLE == 'recording unavailable in replay mode'
    assert not manager.is_recording
    with pytest.raises(RuntimeError):
        manager.stop()
    with pytest.raises(RuntimeError):
        manager.split()
    manager.close()


def test_start_refuses_foreign_recorder(tmp_path):
    """Verify start refuses foreign recorder."""
    receiver = _live_receiver(recorder=OscRecorder(str(tmp_path / 'foreign.pkl')))
    manager = RecorderManager(receiver, record_dir=str(tmp_path), segment_sec=0.0)
    with pytest.raises(RuntimeError):
        manager.start()


def test_legacy_fixture_is_a_version1_sdk_recording(legacy_fixture_path, legacy_frames):
    """The synthetic fixture built by make_fixture.py: 52 bones, Root above Hips, <= 300 KB."""
    assert os.path.getsize(legacy_fixture_path) <= 300 * 1024
    player = OscPlayer(legacy_fixture_path)
    assert player.stream_type == 'movin' and player.num_messages == 90
    assert len(legacy_frames) == 30
    frame = legacy_frames[0]
    assert frame['actor'] == 'MOVINMan' and frame['frame_idx'] == 1000
    bones = frame['bones']
    assert len(bones) == 52
    assert bones[0]['bone_name'] == 'Root' and bones[0]['parent_index'] == -1
    assert bones[1]['bone_name'] == 'Hips' and bones[1]['parent_index'] == 0
    assert all(b['parent_index'] < b['bone_index'] for b in bones)
    assert len({b['bone_name'] for b in bones}) == 52
    assert 'Spine3' not in {b['bone_name'] for b in bones}
    assert [f['frame_idx'] for f in legacy_frames] == list(range(1000, 1030))


def test_recording_info_cli(capsys, live_fixture_path, legacy_fixture_path, tmp_path):
    """Verify recording info cli."""
    from movin_ros import recording_info

    assert recording_info.main([live_fixture_path]) == 0
    out = capsys.readouterr().out
    assert f'recording:    {live_fixture_path}' in out
    assert 'version:      1' in out
    assert 'stream_type:  movin' in out
    assert 'num_messages: 162' in out
    assert 'duration_sec: 0.998' in out
    assert 'actor:        MOVINMan' in out
    assert 'frame_idx:    13899' in out
    assert 'bones:        55' in out
    assert 'preset:       movinman_v3' in out

    assert recording_info.main([legacy_fixture_path]) == 0
    out = capsys.readouterr().out
    assert 'bones:        52' in out and 'preset:       movinman' in out

    legacy = tmp_path / 'motion.pkl'
    with open(legacy, 'wb') as f:
        pickle.dump({'fps': 30, 'root_pos': [], 'root_rot': [], 'dof_pos': []}, f)
    assert recording_info.main([str(legacy)]) == 1
    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'not a MOVIN SDK recording' in captured.err
    assert 'legacy motion_player .pkl / BVH files are not supported' in captured.err

    assert recording_info.main([str(tmp_path / 'missing.pkl')]) == 1
    assert 'no such file' in capsys.readouterr().err

    # several files: all printed, exit 1 if any failed
    assert recording_info.main([live_fixture_path, str(legacy)]) == 1
    captured = capsys.readouterr()
    assert 'num_messages: 162' in captured.out and 'not a MOVIN SDK recording' in captured.err

    with pytest.raises(SystemExit):
        recording_info.main([])


def test_recording_info_describe(live_fixture_path):
    """Verify recording info describe."""
    from movin_ros.recording_info import describe_recording, format_info

    info = describe_recording(live_fixture_path)
    assert info['version'] == 1 and info['num_messages'] == 162
    assert info['bones'] == 55 and info['preset'] == 'movinman_v3'
    assert info['timestamp'].startswith('2026-09-09')
    text = format_info(info)
    assert text.splitlines()[0] == f'recording:    {live_fixture_path}'
    empty = format_info({'recording': 'x', 'duration_sec': 0, 'frame_idx': None, 'actor': ''})
    assert 'frame_idx:    -' in empty and 'actor:        -' in empty


def test_background_save_failure_is_logged(tmp_path, started_receiver, live_messages,
                                           monkeypatch):
    """Verify background save failure is logged."""
    logger = ListLogger()
    manager = RecorderManager(started_receiver, record_dir=str(tmp_path), segment_sec=0.0,
                              logger=logger)
    manager.start(str(tmp_path / 'fail.pkl'))
    sent = _send_frames(started_receiver, frame_packets(live_messages, frame_count=1))
    _wait_recorded(started_receiver.recorder, sent)
    monkeypatch.setattr(started_receiver.recorder, 'save',
                        lambda: (_ for _ in ()).throw(OSError('disk full')))
    manager.split()
    manager.close()
    assert any(level == 'error' and 'disk full' in message for level, message in logger.records)
