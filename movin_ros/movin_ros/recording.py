"""
Raw-OSC recording management on top of ``movin_sdk_python.OscRecorder``.

The SDK recorder is fed from the receiver's UDP thread BEFORE frame assembly,
so a recording is lossless regardless of how often the node polls.  This
module adds message/duration counters (:class:`CountingRecorder`), automatic
file naming (:func:`auto_recording_path`) and the start/stop/split lifecycle
used by the node's services and segment rotation (:class:`RecorderManager`).
Everything is ROS-free: standard library plus the MOVIN SDK core only.
"""

from __future__ import annotations

import datetime
import os
import tempfile
import threading
import time

from movin_sdk_python import OscRecorder

REPLAY_UNAVAILABLE = 'recording unavailable in replay mode'
NOTHING_RECORDED = 'nothing recorded'


class CountingRecorder(OscRecorder):
    """
    ``OscRecorder`` that also counts messages and tracks the first/last wall time.

    Only the public ``record()`` API of the SDK recorder is used; the counters
    are protected by their own lock because ``record()`` runs on the receiver's
    UDP thread while the node reads the counters from its timers.
    """

    def __init__(self, output_path: str, stream_type: str = 'auto'):
        """Create an ``OscRecorder`` for ``output_path`` with zeroed counters."""
        super().__init__(output_path, stream_type=stream_type)
        self.num_messages = 0
        self.first_t = None
        self.last_t = None
        self._count_lock = threading.Lock()

    @property
    def duration_sec(self) -> float:
        """Return seconds between the first and the last recorded message (0.0 when empty)."""
        with self._count_lock:
            if self.first_t is None or self.last_t is None:
                return 0.0
            return float(self.last_t - self.first_t)

    def record(self, address, args, wall_time=None):
        """Count the message, then append it through ``OscRecorder.record``."""
        now = time.time() if wall_time is None else wall_time
        with self._count_lock:
            self.num_messages += 1
            if self.first_t is None:
                self.first_t = now
            self.last_t = now
        super().record(address, args, wall_time=now)


def auto_recording_path(record_dir: str, when: datetime.datetime | None = None,
                        segment: int | None = None) -> str:
    """
    Return ``<record_dir>/movin_<YYYYmmdd_HHMMSS>[_<segment:03d>].pkl`` and create ``record_dir``.

    ``record_dir`` is user-expanded (``~``) and created with ``makedirs(exist_ok=True)``.
    ``when`` defaults to the current local time.
    """
    record_dir = os.path.expanduser(record_dir)
    os.makedirs(record_dir, exist_ok=True)
    stamp = (when or datetime.datetime.now()).strftime('%Y%m%d_%H%M%S')
    if segment is None:
        name = f'movin_{stamp}.pkl'
    else:
        name = f'movin_{stamp}_{int(segment):03d}.pkl'
    return os.path.join(record_dir, name)


def ensure_writable(path: str) -> str:
    """
    Expand ``path``, create its parent directory and check that the file can be written.

    The check opens the file for appending (so an existing file is left as it
    is) and removes it again when it did not exist before, which surfaces a
    directory given as path, a read-only or missing directory and permission
    problems as ``OSError`` at *start* time instead of at the end of the take.
    Returns the expanded path.
    """
    path = os.path.expanduser(path)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    existed = os.path.exists(path)
    with open(path, 'ab'):
        pass
    if not existed:
        try:
            os.remove(path)
        except OSError:
            pass
    return path


def resolve_recording_path(path: str, record_dir: str, segment_sec: float) -> str:
    """
    Return the output file for a recording request (``''``/``'auto'`` = auto-named).

    Auto-named files go to ``record_dir`` with a ``_000`` suffix when segment
    rotation (``segment_sec > 0``) is on; an explicit ``path`` is user-expanded.
    Either way the result passed :func:`ensure_writable`.
    """
    if path in ('', 'auto'):
        segment = 0 if float(segment_sec) > 0.0 else None
        path = auto_recording_path(record_dir, segment=segment)
    return ensure_writable(path)


class RecorderManager:
    """
    Start, stop, split and auto-rotate raw-OSC recordings on one receiver.

    The manager attaches a :class:`CountingRecorder` through the receiver's
    ``recorder`` setter (which the SDK serializes with in-flight ``record()``
    calls), so swapping recorders in :meth:`split` loses no packet.  Segment
    rotation is time based: :meth:`tick` must be called periodically with
    ``time.monotonic()`` seconds (the clock used internally, injectable through
    ``clock`` for tests) and splits once ``segment_sec`` has elapsed.

    ``initial=(recorder, path)`` adopts a recorder that was created before the
    receiver was started and passed into ``MocapReceiver(recorder=...)`` (the
    attach-before-start rule, which keeps the very first packet).
    """

    def __init__(self, receiver, *, record_dir: str, segment_sec: float,
                 stream_type: str = 'movin', logger=None, initial=None, clock=None):
        """Bind to ``receiver``; optionally adopt an already attached ``initial`` recorder."""
        self._receiver = receiver
        self._record_dir = record_dir
        self._segment_sec = float(segment_sec)
        self._stream_type = stream_type
        self._logger = logger
        self._clock = clock if clock is not None else time.monotonic
        self._recorder = None
        self._current_path = ''
        self._segment_index = 0
        self._segment_start = 0.0
        self._base_stem = ''
        self._base_ext = '.pkl'
        self._save_threads = []
        self._lock = threading.RLock()
        if initial is not None:
            recorder, path = initial
            if getattr(receiver, 'recorder', None) is not recorder:
                receiver.recorder = recorder
            self._adopt(recorder, path)

    # ------------------------------------------------------------------ state

    @property
    def is_recording(self) -> bool:
        """Return True while a recorder is attached to the receiver."""
        return self._recorder is not None

    @property
    def current_path(self) -> str:
        """Return the output path of the active segment ('' when not recording)."""
        return self._current_path

    @property
    def segment_index(self) -> int:
        """Return the index of the active segment (0 for the first file)."""
        return self._segment_index

    @property
    def messages(self) -> int:
        """Return the number of messages recorded into the active segment."""
        recorder = self._recorder
        return int(recorder.num_messages) if recorder is not None else 0

    @property
    def duration_sec(self) -> float:
        """Return the duration of the active segment in seconds."""
        recorder = self._recorder
        return float(recorder.duration_sec) if recorder is not None else 0.0

    @property
    def segment_sec(self) -> float:
        """Return the rotation period in seconds (0 = no rotation)."""
        return self._segment_sec

    # -------------------------------------------------------------- lifecycle

    def start(self, path: str = '', stream_type: str | None = None) -> str:
        """
        Attach a new recorder and return its output path.

        ``path`` ``''`` or ``'auto'`` selects :func:`auto_recording_path` in
        ``record_dir`` (with a ``_000`` suffix when segment rotation is on).
        Raises ``RuntimeError`` when the receiver cannot record (replay mode)
        or when a recording is already active.
        """
        with self._lock:
            if not hasattr(self._receiver, 'recorder'):
                raise RuntimeError(REPLAY_UNAVAILABLE)
            if self._recorder is not None:
                raise RuntimeError(f'already recording to {self._current_path}')
            if getattr(self._receiver, 'recorder', None) is not None:
                raise RuntimeError('the receiver already has a recorder attached')
            path = self._resolve_start_path(path)
            recorder = CountingRecorder(
                path, stream_type=stream_type if stream_type else self._stream_type)
            self._receiver.recorder = recorder
            self._adopt(recorder, path)
            self._log('info', f'recording to {path}')
            return path

    def stop(self, save: bool = True) -> tuple[str, int, float]:
        """
        Detach the active recorder, save it synchronously; return ``(path, messages, duration)``.

        With ``save=False`` the buffered messages are discarded.  A recording
        without any message is never written (logged as ``nothing recorded``).
        When the save fails (path is a directory, disk full, ...) the recorder
        is re-attached so the take stays in memory and the error is re-raised:
        fix the cause and stop again, or stop with ``save=False``.  Raises
        ``RuntimeError`` when nothing is being recorded.
        """
        with self._lock:
            recorder = self._recorder
            if recorder is None:
                raise RuntimeError('not recording')
            path = self._current_path
            segment_start = self._segment_start
            if getattr(self._receiver, 'recorder', None) is recorder:
                self._receiver.recorder = None
            self._recorder = None
            self._current_path = ''
            count = int(recorder.num_messages)
            duration = float(recorder.duration_sec)
            if not save:
                self._log('info', f'discarded {count} messages for {path}')
            elif count == 0:
                self._log('warning', f'{NOTHING_RECORDED}, {path} not written')
            else:
                try:
                    recorder.save()
                except Exception:
                    # keep the take: the caller can retry after fixing the path or the disk
                    self._recorder = recorder
                    self._current_path = path
                    self._segment_start = segment_start
                    if hasattr(self._receiver, 'recorder'):
                        self._receiver.recorder = recorder
                    self._log('error', f'saving {count} messages to {path} failed; the recording '
                                       'stays active, fix the path and stop again (or discard)')
                    raise
                self._log('info', f'saved {count} messages ({duration:.1f}s) to {path}')
            return path, count, duration

    def split(self) -> tuple[str, str]:
        """
        Swap in a new segment recorder and save the old one on a background thread.

        Returns ``(old_path, new_path)``.  The swap goes through the receiver's
        ``recorder`` setter, so no packet is lost at the boundary and the
        potentially large ``pickle.dump`` never blocks the caller.
        """
        with self._lock:
            old = self._recorder
            if old is None:
                raise RuntimeError('not recording')
            old_path = self._current_path
            new_path = self._segment_path(self._segment_index + 1)
            new = CountingRecorder(new_path, stream_type=old.stream_type)
            self._receiver.recorder = new
            self._recorder = new
            self._current_path = new_path
            self._segment_index += 1
            self._segment_start = self._clock()
            self._reap_save_threads()
            thread = threading.Thread(
                target=self._save_in_background, args=(old, old_path), daemon=False,
                name=f'movin-save-{os.path.basename(old_path)}')
            self._save_threads.append(thread)
            thread.start()
            self._log('info', f'split recording: {old_path} -> {new_path}')
            return old_path, new_path

    def tick(self, now: float) -> None:
        """Split the recording when ``segment_sec > 0`` and the active segment is old enough."""
        if self._recorder is None or self._segment_sec <= 0.0:
            return
        if now - self._segment_start >= self._segment_sec:
            self.split()

    def reset_segment_start(self, now: float | None = None) -> None:
        """
        Restart the rotation period of the active segment (no-op when not recording).

        The node calls this when its receiver (re)binds, so time spent waiting
        for the socket does not count against the first segment.
        """
        if self._recorder is not None:
            self._segment_start = self._clock() if now is None else float(now)

    def close(self) -> None:
        """
        Save the active recording (if any) and join every background save.

        When the save fails the messages are written to a temporary file
        (``movin_unsaved_*.pkl`` in the system temp directory) instead, so a
        take is never lost at shutdown because of a bad path or a full disk.
        """
        with self._lock:
            if self._recorder is not None:
                try:
                    self.stop(save=True)
                except Exception as exc:
                    self._log('error', f'saving the recording failed: {exc!r}')
                    self._save_to_fallback()
        for thread in list(self._save_threads):
            thread.join()
        self._save_threads = []

    # -------------------------------------------------------------- internals

    def _adopt(self, recorder, path: str) -> None:
        self._recorder = recorder
        self._current_path = path
        self._segment_index = 0
        self._segment_start = self._clock()
        stem, ext = os.path.splitext(path)
        if stem.endswith('_000'):
            stem = stem[:-4]
        self._base_stem = stem
        self._base_ext = ext or '.pkl'

    def _resolve_start_path(self, path: str) -> str:
        return resolve_recording_path(path, self._record_dir, self._segment_sec)

    def _segment_path(self, index: int) -> str:
        return f'{self._base_stem}_{index:03d}{self._base_ext}'

    def _save_in_background(self, recorder, path: str) -> None:
        if int(recorder.num_messages) == 0:
            self._log('warning', f'{NOTHING_RECORDED}, {path} not written')
            return
        try:
            recorder.save()
        except Exception as exc:  # never let a failed save kill the thread silently
            self._log('error', f'failed to save recording {path}: {exc}')

    def _save_to_fallback(self) -> None:
        """Retry the failed final save into a temporary file; discard if that fails too."""
        recorder = self._recorder
        if recorder is None:
            return
        original = self._current_path
        try:
            fd, fallback = tempfile.mkstemp(prefix='movin_unsaved_', suffix='.pkl')
            os.close(fd)
            recorder.output_path = fallback
            self._current_path = fallback
            self.stop(save=True)
            self._log('warning', f'{original} could not be written; the recording was saved '
                                 f'to {fallback} instead')
        except Exception as exc:
            self._log('error', f'fallback save failed too ({exc!r}); discarding the recording')
            self.stop(save=False)

    def _reap_save_threads(self) -> None:
        self._save_threads = [t for t in self._save_threads if t.is_alive()]

    def _log(self, level: str, message: str) -> None:
        logger = self._logger
        if logger is None:
            return
        method = getattr(logger, level, None)
        if callable(method):
            method(message)
