"""
Receiver construction and frame-polling helpers on top of ``movin_sdk_python``.

Everything here is ROS-free (standard library plus the MOVIN SDK core) so the
node logic can be exercised with plain ``pytest``.  :func:`drain_frames` takes
every queued frame through the SDK's ``get_new_frames()``;
:mod:`test_sdk_compat` pins the SDK API relied upon.
"""

from __future__ import annotations

import os
import threading
import time

from movin_sdk_python import MocapReceiver, MovinFrameAssembler, OscPlayer, ReplayMocapReceiver
from movin_sdk_python.utils.skeleton_presets import detect_preset_from_bone_names

MODES = ('live', 'replay')

LEGACY_RECORDING_MESSAGE = (
    ' is not a MOVIN SDK recording '
    '(legacy motion_player .pkl / BVH files are not supported)'
)

# MovinFrameAssembler(max_ready_frames=4): complete frames the SDK keeps between polls.
SDK_READY_QUEUE_DEPTH = 4
# Ready-frame queue of FastReplayReceiver (must exceed the frames pulled per pump()).
FAST_REPLAY_QUEUE_DEPTH = 64
# Frames mocap_node pulls from a FastReplayReceiver per poll (bounded so services and the
# status timer still run between polls; 8 x 120 Hz = up to 960 frames/s).
FAST_REPLAY_FRAMES_PER_POLL = 8


class FastReplayReceiver:
    """
    Lossless, pull-based replay of an SDK recording (``replay_realtime=False``).

    The SDK ``ReplayMocapReceiver(realtime=False)`` pushes the whole file
    through its 4-slot ready queue from a background thread, so a poller only
    ever sees the last 4 frames.  This receiver has no thread: the consumer
    calls :meth:`pump` (``mocap_node`` does so from its poll timer) which
    ingests recorded messages until ``max_frames`` complete frames are queued
    or the recording ends, so every frame is delivered and the replay runs as
    fast as the consumer drains it.  It exposes the attributes the node relies
    on for ``ReplayMocapReceiver`` (``player``, ``lock``, ``assembler``,
    ``running``, ``start()``, ``stop()``, ``get_new_frames()``,
    ``get_latest_frame()``, ``get_receive_rate()``) plus :attr:`finished` and
    :attr:`loops_completed`; like the SDK replay receiver it has no ``recorder``.
    """

    def __init__(self, recording_path: str, loop: bool = True,
                 max_ready_frames: int = FAST_REPLAY_QUEUE_DEPTH):
        """Load ``recording_path`` with ``OscPlayer``; ``loop`` restarts the file at its end."""
        self.player = OscPlayer(recording_path)
        self.loop = bool(loop)
        self.lock = threading.Lock()
        self._max_ready_frames = max(1, int(max_ready_frames))
        self.assembler = MovinFrameAssembler(max_ready_frames=self._max_ready_frames)
        self.running = False
        self.finished = False
        self.loops_completed = 0
        self.messages_replayed = 0
        self.recv_rate_hz = 0.0
        self._messages = None
        self._rate_count = 0
        self._rate_t0 = time.time()

    def start(self) -> None:
        """Rewind the recording and accept :meth:`pump` calls."""
        with self.lock:
            self.assembler = MovinFrameAssembler(max_ready_frames=self._max_ready_frames)
        self._messages = self.player.messages(realtime=False)
        self.finished = False
        self.loops_completed = 0
        self.messages_replayed = 0
        self.recv_rate_hz = 0.0
        self._rate_count = 0
        self._rate_t0 = time.time()
        self.running = True

    def stop(self) -> None:
        """Stop replaying (a later ``start()`` rewinds)."""
        self.running = False
        self._messages = None

    def pump(self, max_frames: int = FAST_REPLAY_FRAMES_PER_POLL) -> int:
        """
        Ingest messages until ``max_frames`` complete frames are queued; return messages read.

        At the end of the file a looping receiver rewinds (at most once per
        call, so a recording without complete frames cannot spin forever) and a
        non-looping one sets :attr:`finished`.  Nothing happens before
        ``start()`` or after the replay finished.
        """
        if not self.running or self.finished or self._messages is None:
            return 0
        budget = max(1, min(int(max_frames), self._max_ready_frames))
        ingested = 0
        rewound = False
        now = time.time()
        with self.lock:
            while len(self.assembler.ready_frames) < budget:
                try:
                    address, args = next(self._messages)
                except StopIteration:
                    if not self.loop:
                        self.finished = True
                        break
                    if rewound:
                        break
                    rewound = True
                    self.loops_completed += 1
                    self._messages = self.player.messages(realtime=False)
                    continue
                self.assembler.ingest(address, args, now=now)
                ingested += 1
        self.messages_replayed += ingested
        self._rate_count += ingested
        dt = now - self._rate_t0
        if dt >= 1.0:
            self.recv_rate_hz = self._rate_count / dt
            self._rate_count = 0
            self._rate_t0 = now
        return ingested

    def get_new_frames(self) -> list:
        """Return every queued frame, oldest first, and clear the queue (SDK API)."""
        with self.lock:
            return self.assembler.pop_all_frames()

    def get_latest_frame(self):
        """Return the newest queued frame and drop the older ones (SDK API)."""
        with self.lock:
            return self.assembler.pop_latest_frame()

    def get_receive_rate(self) -> float:
        """Return the replayed message rate in messages per second of wall time."""
        return self.recv_rate_hz


def _open_player(path: str) -> OscPlayer:
    """Open ``path`` with ``OscPlayer``; OSError passes through, anything else is a ValueError."""
    try:
        return OscPlayer(path)
    except OSError:
        raise
    except Exception as exc:
        raise ValueError(f'{path}{LEGACY_RECORDING_MESSAGE}') from exc


def build_receiver(mode, *, host, port, recording, replay_realtime, replay_loop, recorder=None):
    """
    Create the SDK receiver for ``mode``.

    ``'live'`` returns ``MocapReceiver(port=port, recorder=recorder, host=host)``
    (``host`` is the LOCAL bind address).  ``'replay'`` returns
    ``ReplayMocapReceiver(recording, realtime=True, loop=replay_loop)`` when
    ``replay_realtime`` is true and the lossless :class:`FastReplayReceiver`
    otherwise (the SDK receiver's non-realtime mode keeps only 4 frames).
    Raises ``ValueError`` for an unknown mode, a replay without a recording or
    a file that is not an SDK recording (see :func:`validate_recording`);
    ``FileNotFoundError``/``OSError`` pass through.  Neither receiver is started.
    """
    if mode == 'live':
        return MocapReceiver(port=int(port), recorder=recorder, host=host)
    if mode == 'replay':
        if not recording:
            raise ValueError("mode 'replay' requires the 'recording' parameter (SDK .pkl path)")
        path = os.path.expanduser(recording)
        try:
            if replay_realtime:
                return ReplayMocapReceiver(path, realtime=True, loop=bool(replay_loop))
            return FastReplayReceiver(path, loop=bool(replay_loop))
        except OSError:
            raise
        except Exception as exc:
            raise ValueError(f'{path}{LEGACY_RECORDING_MESSAGE}') from exc
    raise ValueError(f'unknown mode {mode!r}; expected one of {MODES}')


def recording_summary(player) -> dict:
    """Return ``{'num_messages', 'duration_sec', 'stream_type'}`` of an ``OscPlayer``."""
    return {
        'num_messages': int(player.num_messages),
        'duration_sec': float(player.duration_sec),
        'stream_type': str(player.stream_type),
    }


def validate_recording(path: str) -> dict:
    """
    Open ``path`` with ``OscPlayer`` and return its summary.

    Returns ``{'num_messages', 'duration_sec', 'stream_type'}``.  File access
    errors (``FileNotFoundError`` and other ``OSError``) pass through.  Anything
    the SDK rejects, including ``ValueError('Unsupported recording version')``
    and unpickling failures such as the ``ModuleNotFoundError`` a legacy numpy
    pickle raises, becomes ``ValueError('<path> is not a MOVIN SDK recording ...')``.
    """
    return recording_summary(_open_player(os.path.expanduser(path)))


def first_complete_frame(player):
    """
    Return the first complete frame of an already loaded ``OscPlayer``, or ``None``.

    Same as ``movin_sdk_python.peek_first_frame`` but without reading the file
    again, so a replay receiver's ``player`` can be inspected at no extra cost.
    """
    assembler = MovinFrameAssembler(max_ready_frames=1)
    for address, args in player.messages(realtime=False):
        assembler.ingest(address, args, now=0.0)
        frame = assembler.pop_latest_frame()
        if frame is not None:
            return frame
    return None


def drain_frames(receiver) -> tuple[list, str]:
    """
    Return ``(frames_oldest_first, delivery)``: every complete frame queued since the last poll.

    ``delivery == 'drain'`` when the receiver has ``get_new_frames()``
    (``MocapReceiver``, ``ReplayMocapReceiver`` and :class:`FastReplayReceiver`
    do): it returns every frame completed since the last call, oldest first.
    For ``MocapReceiver`` every returned frame also counts toward the viewer
    rate it reports to MOVIN Studio.  A receiver without it gets the lossy
    fallback ``[receiver.get_latest_frame()]`` with ``delivery == 'latest_only'``.

    Queue-depth caveat: the SDK assembler stores ready frames in a
    ``collections.deque(maxlen=4)`` shared by all actors.  When a fifth frame
    completes before the next poll the OLDEST queued frame is silently dropped
    by the SDK itself, so draining is lossless only while at most 4 frames
    complete between two polls (e.g. a 60 Hz stream polled at >= 15 Hz; the
    node default of 120 Hz leaves an 8x margin).  Drops are visible afterwards
    as ``frame_idx`` gaps (see :class:`GapTracker`).  :class:`FastReplayReceiver`
    queues up to ``FAST_REPLAY_QUEUE_DEPTH`` frames and only fills the queue
    when pumped, so draining it is always lossless.
    """
    get_new_frames = getattr(receiver, 'get_new_frames', None)
    if get_new_frames is None:
        frame = receiver.get_latest_frame()
        return ([frame] if frame is not None else []), 'latest_only'
    return list(get_new_frames()), 'drain'


def detect_preset(frame) -> str:
    """Return the preset name (``movinman`` / ``movinman_v3``) of ``frame``, or ``''`` on error."""
    try:
        names = [bone['bone_name'] for bone in frame['bones']]
        return detect_preset_from_bone_names(names).name
    except Exception:
        return ''


def validate_frame(frame) -> list:
    """
    Return human-readable warnings about a frame's skeleton topology (empty when fine).

    Checked: bones that are not parents-first (``parent_index >= bone_index``,
    which also catches self-parenting), parent indices that are absent from the
    frame, and duplicate bone names.  TF publishing tolerates all of these, but
    parity with the SDK forward kinematics is only guaranteed without them.
    """
    warnings = []
    try:
        bones = list(frame['bones'])
    except Exception:
        return ['frame has no bones list']
    indices = {bone.get('bone_index') for bone in bones}
    seen = set()
    duplicates = []
    unordered = []
    dangling = []
    for bone in bones:
        name = bone.get('bone_name', '?')
        index = bone.get('bone_index', -1)
        parent = bone.get('parent_index', -1)
        if name in seen:
            duplicates.append(name)
        seen.add(name)
        if parent is not None and parent >= 0:
            if parent not in indices:
                dangling.append(name)
            elif parent >= index:
                unordered.append(name)
    if unordered:
        warnings.append(
            f'{len(unordered)} bone(s) not in parents-first order (parent_index >= bone_index): '
            f'{unordered[:8]}; TF stays valid but SDK forward-kinematics parity is not guaranteed'
        )
    if dangling:
        warnings.append(
            f'{len(dangling)} bone(s) reference a parent_index absent from the frame '
            f'(attached to the actor anchor): {dangling[:8]}'
        )
    if duplicates:
        warnings.append(f'duplicate bone names (later bones overwrite earlier TF frames): '
                        f'{sorted(set(duplicates))[:8]}')
    return warnings


class GapTracker:
    """
    Per-actor ``frame_idx`` gap accounting.

    ``frames_seen`` counts every update; ``frames_dropped`` accumulates the
    number of frame indices skipped between consecutive frames of the same
    actor.  A non-increasing index (replay loop restart, Studio restart) resets
    the actor's reference without counting a gap.
    """

    def __init__(self):
        """Start with zero frames seen/dropped and no actors."""
        self.frames_seen = 0
        self.frames_dropped = 0
        self._last = {}

    def update(self, actor, frame_idx) -> int:
        """Record ``frame_idx`` for ``actor``; return the gap added (0 on first/non-increasing)."""
        self.frames_seen += 1
        last = self._last.get(actor)
        self._last[actor] = frame_idx
        if last is None or frame_idx <= last:
            return 0
        gap = int(frame_idx - last - 1)
        self.frames_dropped += gap
        return gap

    @property
    def actors(self) -> list:
        """Return the actor names seen so far, sorted."""
        return sorted(self._last)

    def last_frame_idx(self, actor):
        """Return the last ``frame_idx`` seen for ``actor`` or ``None``."""
        return self._last.get(actor)

    def reset(self):
        """Forget every actor and zero both counters."""
        self.frames_seen = 0
        self.frames_dropped = 0
        self._last.clear()


def replay_finished(receiver) -> bool:
    """
    Return True when a non-looping replay receiver has played its last message.

    ``ReplayMocapReceiver`` gives no end-of-replay signal: its thread simply
    exits while ``running`` stays True, so the thread state is inspected.
    :class:`FastReplayReceiver` reports its ``finished`` flag.  Always False
    for live receivers and for a replay receiver that was never started.
    """
    if isinstance(receiver, FastReplayReceiver):
        return bool(receiver.finished)
    if not isinstance(receiver, ReplayMocapReceiver):
        return False
    thread = getattr(receiver, 'thread', None)
    return thread is not None and not thread.is_alive()


def receiver_thread_died(receiver) -> bool:
    """
    Return True when a started ``MocapReceiver``'s UDP thread has ended on its own.

    An exception escaping ``recorder.record()`` or ``assembler.ingest()`` ends
    ``_udp_server_loop`` (the socket is closed in its ``finally``) while
    ``receiver.running`` stays True and no frame is assembled any more; the
    receiver looks healthy otherwise, so the thread state has to be checked.
    """
    if not isinstance(receiver, MocapReceiver) or not receiver.running:
        return False
    thread = getattr(receiver, 'thread', None)
    return thread is not None and not thread.is_alive()


def bound_address(receiver) -> str:
    """Return ``'<host>:<port>'`` of the receiver's bound UDP socket, or ``''`` if unavailable."""
    sock = getattr(receiver, 'sock', None)
    if sock is None:
        return ''
    try:
        host, port = sock.getsockname()[:2]
    except (OSError, ValueError, TypeError):
        return ''
    return f'{host}:{port}'
