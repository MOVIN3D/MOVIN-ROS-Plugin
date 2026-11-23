"""
Pin the ``movin_sdk_python`` API and attributes that ``movin_ros`` relies on.

Frames are taken with ``get_new_frames()``; ``assembler.ready_frames`` (the
4-frame queue behind it, also used by ``FastReplayReceiver`` and the tests),
``receiver.thread``/``sock``/``recorder`` and the replay ``player`` are public
attributes but not documented API, so this module fails loudly when the
vendored SDK (git submodule) changes shape.
"""

from __future__ import annotations

import collections
import inspect
import threading

import movin_sdk_python
from movin_sdk_python import (
    MocapReceiver,
    MovinFrameAssembler,
    MovinSession,
    OscPlayer,
    OscRecorder,
    ReplayMocapReceiver,
)
from movin_sdk_python.utils.skeleton_presets import detect_preset_from_bone_names

import pytest


def test_sdk_version_pinned():
    """Verify sdk version pinned."""
    assert movin_sdk_python.__version__ == '0.1.0'


def test_mocap_receiver_shape():
    """Verify mocap receiver shape."""
    receiver = MocapReceiver(host='127.0.0.1', port=0)
    assert isinstance(receiver.lock, type(threading.Lock()))
    assert isinstance(receiver.assembler, MovinFrameAssembler)
    ready = receiver.assembler.ready_frames
    assert isinstance(ready, collections.deque) and ready.maxlen == 4
    assert receiver.thread is None and receiver.sock is None
    assert hasattr(receiver, 'get_latest_frame') and hasattr(receiver, 'get_receive_rate')
    assert receiver.get_new_frames() == []
    params = inspect.signature(MocapReceiver).parameters
    assert set(params) == {'port', 'recorder', 'host', 'reply_status'}
    assert params['host'].kind is inspect.Parameter.KEYWORD_ONLY
    # mocap_node relies on the default: MOVIN Studio's OSC status probe is answered.
    assert params['reply_status'].kind is inspect.Parameter.KEYWORD_ONLY
    assert params['reply_status'].default is True


def test_mocap_receiver_recorder_property_has_setter():
    """Verify mocap receiver recorder property has setter."""
    prop = MocapReceiver.recorder
    assert isinstance(prop, property) and prop.fset is not None
    receiver = MocapReceiver(host='127.0.0.1', port=0)
    marker = object()
    receiver.recorder = marker
    assert receiver.recorder is marker
    receiver.recorder = None
    assert receiver.recorder is None


def test_reset_replaces_assembler():
    """Verify reset replaces assembler."""
    receiver = MocapReceiver(host='127.0.0.1', port=0)
    before = receiver.assembler
    receiver.reset()
    assert receiver.assembler is not before
    assert receiver.assembler.ready_frames.maxlen == 4


def test_replay_receiver_shape(live_fixture_path):
    """Verify replay receiver shape."""
    receiver = ReplayMocapReceiver(live_fixture_path, realtime=False, loop=False)
    assert receiver.thread is None
    assert isinstance(receiver.lock, type(threading.Lock()))
    assert isinstance(receiver.assembler.ready_frames, collections.deque)
    assert receiver.assembler.ready_frames.maxlen == 4
    assert not hasattr(receiver, 'recorder')
    assert receiver.get_new_frames() == []
    assert isinstance(receiver.player, OscPlayer)
    params = inspect.signature(ReplayMocapReceiver).parameters
    assert list(params) == ['recording_path', 'realtime', 'loop']


def test_frame_assembler_defaults():
    """Verify frame assembler defaults."""
    params = inspect.signature(MovinFrameAssembler).parameters
    assert params['max_ready_frames'].default == 4
    assembler = MovinFrameAssembler()
    for i in range(6):
        assembler.ready_frames.append({'frame_idx': i})
    assert [f['frame_idx'] for f in assembler.ready_frames] == [2, 3, 4, 5]
    assert assembler.pop_latest_frame() == {'frame_idx': 5}
    assert len(assembler.ready_frames) == 0
    for i in range(3):
        assembler.ready_frames.append({'frame_idx': i})
    assert [f['frame_idx'] for f in assembler.pop_all_frames()] == [0, 1, 2]
    assert assembler.pop_all_frames() == []


def test_osc_recorder_and_player_api(tmp_path, live_fixture_path):
    """Verify osc recorder and player api."""
    params = inspect.signature(OscRecorder.record).parameters
    assert list(params) == ['self', 'address', 'args', 'wall_time']
    player = OscPlayer(live_fixture_path)
    assert player.num_messages == 162 and player.stream_type == 'movin'
    assert 0.9 < player.duration_sec < 1.1
    address, args = next(player.messages(realtime=False))
    assert address == '/MOVIN/Frame' and len(args) > 7
    t, _, _ = next(player.messages_with_timing())
    assert t == 0.0
    frame = movin_sdk_python.peek_first_frame(live_fixture_path)
    assert set(frame) == {'timestamp', 'actor', 'frame_idx', 'bones'}
    assert set(frame['bones'][0]) == {'bone_index', 'parent_index', 'bone_name', 'p', 'rq', 'q',
                                      's'}
    with pytest.raises(ValueError):
        OscPlayer(str(_write_pickle(tmp_path / 'v0.pkl', {'version': 0, 'messages': []})))


def test_session_api():
    """Verify session api."""
    params = inspect.signature(MovinSession).parameters
    for name in ('receiver', 'processors', 'sinks', 'on_extension_error', 'host', 'port'):
        assert name in params
    for name in ('start', 'stop', 'process_frame', 'get_latest_frame', 'start_recording',
                 'stop_recording'):
        assert callable(getattr(MovinSession, name))


def test_preset_detection_api():
    """Verify preset detection api."""
    assert detect_preset_from_bone_names(['Hips', 'Spine3']).name == 'movinman_v3'
    assert detect_preset_from_bone_names(['Hips', 'Spine1']).name == 'movinman'


def _write_pickle(path, payload):
    import pickle

    with open(path, 'wb') as f:
        pickle.dump(payload, f)
    return path
