"""
ROS 2 node that receives, records and replays MOVIN Studio motion capture.

``mocap_node`` wraps the MOVIN SDK receiver (``MocapReceiver`` for the live
UDP/OSC stream, ``ReplayMocapReceiver`` for SDK ``.pkl`` recordings) in a
``MovinSession`` and turns every assembled frame into

* ``mocap_frame`` (``movin_msgs/MocapFrame``): the SDK bones, lossless;
* ``/tf``: one ``TFMessage`` per frame with every bone edge under a static
  per-actor anchor ``world_frame -> <tf_prefix>/<actor_id>`` on ``/tf_static``;
* ``status`` (``movin_msgs/MocapStatus``, 1 Hz) and optional
  ``skeleton_markers``.

Recording uses the SDK's raw-OSC recorder through
:class:`movin_ros.recording.RecorderManager` (``record`` parameter or the
``start_recording`` / ``stop_recording`` / ``split_recording`` services).
Everything is a ROS parameter; see ``config/mocap_node.yaml``.
"""

from __future__ import annotations

import logging
import os
import signal
import time

from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from movin_msgs.msg import Bone, MocapFrame, MocapStatus
from movin_msgs.srv import StartRecording, StopRecording
from movin_ros import convert, frame_source, markers, recording
import movin_sdk_python
from movin_sdk_python import MovinSession
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy import qos
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import MarkerArray

NODE_NAME = 'mocap_node'
# Console-only logger for messages after the context is gone; must never equal a node
# logger name (a bare ``ros2 run`` node is logged as ``mocap_node``), else rcl would
# route it to that node's already destroyed rosout publisher.
PROCESS_LOGGER_NAME = 'mocap_node.process'
PRESET_CHOICES = ('auto', 'movinman', 'movinman_v3')
STREAM_TYPES = ('movin', 'nova', 'auto')
WARN_THROTTLE_SEC = 2.0
STATUS_PERIOD_SEC = 1.0
SDK_LOGGER_NAME = 'movin_sdk_python'


class ExitRequested(Exception):
    """Raised out of a timer callback to leave ``rclpy.spin`` (``MocapNode.exit_code`` = code)."""


class RclpyLogHandler(logging.Handler):
    """Forward Python ``logging`` records (the SDK's) to an rclpy logger."""

    def __init__(self, ros_logger):
        """Create a handler that writes to ``ros_logger`` (rosout)."""
        super().__init__()
        self._ros_logger = ros_logger
        self.setFormatter(logging.Formatter('%(name)s: %(message)s'))

    def emit(self, record):
        """Map the record level to the rclpy logger; WARNING and above are throttled."""
        try:
            message = self.format(record)
            if record.levelno >= logging.ERROR:
                self._ros_logger.error(message, throttle_duration_sec=WARN_THROTTLE_SEC)
            elif record.levelno >= logging.WARNING:
                self._ros_logger.warning(message, throttle_duration_sec=WARN_THROTTLE_SEC)
            elif record.levelno >= logging.INFO:
                self._ros_logger.info(message)
            else:
                self._ros_logger.debug(message)
        except Exception:
            self.handleError(record)


class ShutdownSafeLogger:
    """
    Logger facade that survives an external shutdown.

    After ``ros2 launch`` Ctrl-C on Jazzy the context is already shut down while
    the node still saves its recording; the node logger would then warn that the
    rosout publisher is invalid, so a plain rcutils logger (console only) is used
    instead.  Exposes ``info``/``warning``/``error`` like an rclpy logger.
    """

    def __init__(self, node):
        """Bind to ``node`` (its logger and context)."""
        self._node = node

    def _target(self):
        if self._node.context.ok():
            return self._node.get_logger()
        return get_logger(PROCESS_LOGGER_NAME)

    def info(self, message: str) -> None:
        """Log at INFO level."""
        self._target().info(message)

    def warning(self, message: str) -> None:
        """Log at WARNING level."""
        self._target().warning(message)

    def error(self, message: str) -> None:
        """Log at ERROR level."""
        self._target().error(message)


class SignalGuard:
    """
    Defer SIGINT/SIGTERM while the node shuts down.

    A terminal Ctrl-C reaches every process of the foreground group and
    ``ros2 launch`` then signals each child again, so a node normally receives
    two SIGINTs.  rclpy turns the first one into the end of ``spin``; the second
    would surface as a ``KeyboardInterrupt`` in the middle of ``destroy_node``
    (for example while the last recording segment is being saved).  Inside the
    guard both signals are only counted and on exit they are ignored for the
    rest of the process (see :meth:`__exit__`); must be entered from the main
    thread (elsewhere it is a no-op).
    """

    def __init__(self):
        """Create an inactive guard."""
        self.received = []
        self._previous = {}

    def _record(self, signum, _frame) -> None:
        self.received.append(signum)

    def __enter__(self):
        """Install the recording handlers (main thread only)."""
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                self._previous[signum] = signal.signal(signum, self._record)
            except (ValueError, OSError):  # not the main thread / unsupported
                pass
        return self

    def __exit__(self, exc_type, exc, tb):
        """
        Ignore SIGINT/SIGTERM from now on: the process is exiting.

        ``signal.signal`` only ever reports Python's own handler table
        (``default_int_handler`` for SIGINT, ``SIG_DFL`` for SIGTERM) because
        rclpy installs its handler with ``sigaction`` behind Python's back, so
        "restoring" the previous handlers would re-arm exactly the behaviour
        the guard exists to prevent: a late SIGINT (``ros2 launch`` signals
        every child a second time) would abort the interpreter's finalization
        with a ``KeyboardInterrupt`` (exit code -2) and a SIGTERM arriving
        during the last ~100 ms of rmw teardown would kill the process with
        exit code 143 although the recording is already on disk.  SIGKILL
        (launch's escalation) still works.
        """
        for signum in self._previous:
            try:
                signal.signal(signum, signal.SIG_IGN)
            except (ValueError, OSError, TypeError):
                pass
        return False


def _vector3(v) -> Vector3:
    return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _quaternion_from_wxyz(q) -> Quaternion:
    return Quaternion(x=float(q[1]), y=float(q[2]), z=float(q[3]), w=float(q[0]))


def _transform(stamp, parent: str, child: str, t_xyz, q_xyzw) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = float(t_xyz[0])
    transform.transform.translation.y = float(t_xyz[1])
    transform.transform.translation.z = float(t_xyz[2])
    transform.transform.rotation.x = float(q_xyzw[0])
    transform.transform.rotation.y = float(q_xyzw[1])
    transform.transform.rotation.z = float(q_xyzw[2])
    transform.transform.rotation.w = float(q_xyzw[3])
    return transform


def bone_message(bone) -> Bone:
    """Convert one SDK bone dict to ``movin_msgs/Bone`` (quaternions reordered wxyz -> xyzw)."""
    return Bone(
        bone_index=int(bone['bone_index']),
        parent_index=int(bone.get('parent_index', -1)),
        name=str(bone['bone_name']),
        position=_vector3(bone['p']),
        rest_rotation=_quaternion_from_wxyz(bone.get('rq', (1.0, 0.0, 0.0, 0.0))),
        rotation=_quaternion_from_wxyz(bone['q']),
        scale=_vector3(bone.get('s', (1.0, 1.0, 1.0))),
    )


def _skeleton_signature(bones) -> tuple:
    return tuple(
        (b.get('bone_index'), b.get('parent_index'), b.get('bone_name')) for b in bones)


class FramePublisherSink:
    """
    SDK ``FrameSink`` that publishes every assembled frame (spec section 3.5).

    Runs on the node's executor thread (``MovinSession.process_frame`` is
    called from the poll timer), so it shares the node state without locks.
    """

    def __init__(self, node):
        """Bind to ``node`` (publishers, parameters, logger, clock)."""
        self._node = node
        self.actor_ids = {}          # actor name -> TF-safe actor id
        self.actor_info = {}         # actor name -> {'id', 'preset', 'signature', 'bones'}
        self.anchors = []            # every static anchor transform sent so far
        self.gaps = frame_source.GapTracker()
        self.frames_published = 0
        self.frames_filtered = 0
        self.last_preset = ''
        self._ignored_actors = set()

    # ---------------------------------------------------------------- sink API

    def on_frame(self, frame) -> None:
        """Publish ``frame`` (MocapFrame, TF, markers) and update the counters."""
        try:
            self._publish_frame(frame)
        except RuntimeError:
            # RCLError from a publisher whose context a SIGINT/SIGTERM shut down in the middle
            # of this callback; the process is exiting, so there is nothing left to publish to.
            if self._node.context.ok():
                raise

    def _publish_frame(self, frame) -> None:
        node = self._node
        stamp = node.get_clock().now().to_msg()
        actor = str(frame.get('actor', ''))
        if node.actor_filter and actor != node.actor_filter:
            self.frames_filtered += 1
            if actor not in self._ignored_actors:
                self._ignored_actors.add(actor)
                node.get_logger().info(
                    f'ignoring actor {actor!r} (actor_filter={node.actor_filter!r})')
            return
        bones = frame['bones']
        actor_id = convert.assign_actor_id(actor, self.actor_ids)
        signature = _skeleton_signature(bones)
        info = self.actor_info.get(actor)
        topology_changed = False
        if info is None:
            info = self._register_actor(frame, actor, actor_id, stamp, signature)
            topology_changed = True
        elif info['signature'] != signature:
            self._update_topology(frame, actor, info, signature)
            topology_changed = True
        self.gaps.update(actor, int(frame['frame_idx']))

        edges = convert.local_edges(bones)
        anchor = convert.anchor_frame(node.tf_prefix, actor_id)
        if node.publish_frame:
            node.frame_pub.publish(
                self._frame_message(frame, actor, anchor, info['preset'], stamp))
        if node.publish_tf:
            node.tf_broadcaster.sendTransform(self._transforms(edges, actor_id, anchor, stamp))
        if node.publish_markers:
            node.marker_pub.publish(markers.build_marker_array(
                edges, tf_prefix=node.tf_prefix, actor_id=actor_id, stamp=stamp,
                topology_changed=topology_changed))
        self.frames_published += 1
        if node.max_frames > 0 and self.frames_published >= node.max_frames:
            node.request_exit(f'max_frames={node.max_frames} reached')

    # ---------------------------------------------------------------- helpers

    def _register_actor(self, frame, actor, actor_id, stamp, signature) -> dict:
        node = self._node
        bones = frame['bones']
        preset = node.preset if node.preset != 'auto' else frame_source.detect_preset(frame)
        for warning in frame_source.validate_frame(frame):
            node.get_logger().warning(f'actor {actor!r}: {warning}')
        anchor = convert.anchor_frame(node.tf_prefix, actor_id)
        self.anchors.append(_transform(
            stamp, node.world_frame, anchor, node.skeleton_offset,
            convert.anchor_quat(node.forward_mode)))
        if node.publish_tf:
            # Humble publishes only the given list and Jazzy keeps the first transform per
            # child id, so the complete anchor list is re-sent for late subscribers.
            node.static_broadcaster.sendTransform(list(self.anchors))
        info = {'id': actor_id, 'preset': preset, 'signature': signature, 'bones': len(bones)}
        self.actor_info[actor] = info
        self.last_preset = preset
        node.get_logger().info(
            f'actor {actor!r} -> {anchor}: {len(bones)} bones, '
            f'preset {preset or "unknown"}, first frame_idx {frame.get("frame_idx")}')
        return info

    def _update_topology(self, frame, actor, info, signature) -> None:
        node = self._node
        bones = frame['bones']
        if node.preset == 'auto':
            info['preset'] = frame_source.detect_preset(frame)
            self.last_preset = info['preset']
        info['signature'] = signature
        info['bones'] = len(bones)
        for warning in frame_source.validate_frame(frame):
            node.get_logger().warning(f'actor {actor!r}: {warning}')
        node.get_logger().info(
            f'actor {actor!r}: skeleton changed, now {len(bones)} bones, '
            f'preset {info["preset"] or "unknown"}')

    def _frame_message(self, frame, actor, anchor, preset, stamp) -> MocapFrame:
        msg = MocapFrame()
        msg.header.stamp = stamp
        msg.header.frame_id = self._node.world_frame
        msg.actor = actor
        msg.actor_frame = anchor
        msg.frame_idx = int(frame['frame_idx'])
        msg.source_timestamp = str(frame.get('timestamp', ''))
        msg.preset = preset
        msg.convention = MocapFrame.CONVENTION_UNITY_LH_YUP
        msg.bones = [bone_message(bone) for bone in frame['bones']]
        return msg

    def _transforms(self, edges, actor_id, anchor, stamp) -> list:
        tf_prefix = self._node.tf_prefix
        transforms = []
        for _index, name, parent_name, t_xyz, q_xyzw in edges:
            if parent_name is None:
                parent = anchor
            else:
                parent = convert.bone_frame(tf_prefix, actor_id, parent_name)
            child = convert.bone_frame(tf_prefix, actor_id, name)
            transforms.append(_transform(stamp, parent, child, t_xyz, q_xyzw))
        return transforms


class MocapNode(Node):
    """``mocap_node``: MOVIN Studio receive/record/replay node (spec section 3.5)."""

    def __init__(self):
        """Declare parameters, create interfaces, build and start the SDK session."""
        super().__init__(NODE_NAME)
        self.startup_error = ''
        self.exit_code = 0
        self._exit_reason = ''
        self._recorder_closed = False
        self._summary_logged = False
        self._node_destroyed = False
        self._session = None
        self._receiver = None
        self._recorder_manager = None
        self._sink = None
        self._poll_timer = None
        self._status_timer = None
        self._retry_timer = None
        self._log_handler = None
        self._start_error = ''
        self._bind_address = ''
        self._delivery = ''
        self._replay_finished = False
        self._peeked_preset = ''
        self._last_frame_monotonic = None
        self._sink_errors = 0
        self.frame_pub = None
        self.status_pub = None
        self.marker_pub = None
        self.tf_broadcaster = None
        self.static_broadcaster = None
        self._safe_logger = ShutdownSafeLogger(self)

        self.get_logger().info(
            f'movin_sdk_python {getattr(movin_sdk_python, "__version__", "?")} from '
            f'{os.path.dirname(os.path.abspath(movin_sdk_python.__file__))}')
        try:
            self._declare_parameters()
            self._read_parameters()
        except (ParameterException, ValueError) as exc:
            self._fatal(str(exc))
            return
        self._create_interfaces()
        self._install_sdk_logging_bridge()
        try:
            self._create_session()
        except (OSError, ValueError) as exc:
            self._fatal(str(exc))
            return
        if not self._start_session():
            if self.retry_bind_sec <= 0.0:
                self._fatal(
                    f'cannot start the {self.mode} receiver on {self.host}:{self.port}: '
                    f'{self._start_error} (set retry_bind_sec > 0 to keep retrying)')
                return
            self.get_logger().warning(
                f'cannot bind {self.host}:{self.port} ({self._start_error}); '
                f'retrying every {self.retry_bind_sec:g}s')
            self._arm_retry_timer()
        self._poll_timer = self.create_timer(1.0 / self.poll_rate_hz, self._on_poll)
        self._status_timer = self.create_timer(STATUS_PERIOD_SEC, self._on_status)
        self.get_logger().info(
            f'mode={self.mode} forward_mode={self.forward_mode} world_frame={self.world_frame} '
            f'tf_prefix={self.tf_prefix} preset={self.preset} poll_rate_hz={self.poll_rate_hz:g} '
            f'publish_every_frame={self.publish_every_frame} publish_tf={self.publish_tf} '
            f'publish_frame={self.publish_frame} publish_markers={self.publish_markers} '
            f'max_frames={self.max_frames}')

    # ------------------------------------------------------------- parameters

    def _declare_parameters(self) -> None:
        """
        Declare every parameter (all read once at startup, hence ``read_only``).

        Floating-point parameters are declared with ``dynamic_typing`` so that an
        integer override (``-p poll_rate_hz:=60``, ``skeleton_offset: [0, 0, 0]``
        in YAML) is accepted and converted instead of aborting the node with
        ``InvalidParameterTypeException``.
        """
        def declare(name, default, description):
            dynamic = isinstance(default, float) or (
                isinstance(default, list) and all(isinstance(v, float) for v in default))
            self.declare_parameter(
                name, default, ParameterDescriptor(
                    description=description, read_only=True, dynamic_typing=dynamic))

        declare('mode', 'live', 'Frame source: live (UDP from MOVIN Studio) or replay '
                '(SDK .pkl recording given by the recording parameter).')
        declare('host', '0.0.0.0', 'LOCAL address the UDP socket binds to (not the Studio IP); '
                'MOVIN Studio must send to this machine.')
        declare('port', 11235, 'UDP port to bind; 0 = ephemeral (the actual port is logged '
                'and reported in status.bind_address).')
        declare('recording', '', 'Replay input: path of an SDK raw-OSC .pkl recording.')
        declare('replay_loop', True, 'Restart the recording when it ends (replay mode).')
        declare('replay_realtime', True,
                'true = reproduce the recorded timing (SDK ReplayMocapReceiver); false = replay '
                'as fast as the node publishes, losslessly, from the poll timer.')
        declare('replay_exit_on_end', False,
                'Exit with code 0 once a non-looping replay has played its last frame.')
        declare('record', '', 'Recording at startup: "" = off, "auto" = auto-named file in '
                'record_dir, anything else = explicit .pkl path.')
        declare('record_dir', '~/movin_recordings',
                'Directory of auto-named recordings (movin_<YYYYmmdd_HHMMSS>[_NNN].pkl).')
        declare('record_segment_sec', 600.0,
                'Lossless segment rotation period in seconds; 0 = never split.')
        declare('record_stream_type', 'movin',
                'stream_type written into recordings: movin, nova or auto.')
        declare('preset', 'auto',
                'Skeleton preset: auto (detected from the bone names), movinman or movinman_v3.')
        declare('poll_rate_hz', 120.0, 'Rate of the receiver polling timer; the SDK queues at '
                'most 4 frames, so keep it above 4x the stream frame rate.')
        declare('publish_every_frame', True, 'true = drain every assembled frame from the SDK '
                'queue, false = publish only the newest frame per poll.')
        declare('actor_filter', '', 'Publish only this actor name ("" = every actor).')
        declare('forward_mode', 'isaac_world', 'Static anchor rotation: coord_equivalent '
                '(Unity +Z -> ROS -Y) or isaac_world (Unity +Z -> ROS +X, REP-103).')
        declare('world_frame', 'world', 'Parent frame of the actor anchors.')
        declare('tf_prefix', 'movin',
                'TF prefix: anchor <tf_prefix>/<actor_id>, bones <tf_prefix>/<actor_id>/<bone>.')
        declare('skeleton_offset', [0.0, 0.0, 0.0],
                'Translation of every actor anchor in world_frame, meters (x, y, z).')
        declare('publish_tf', True, 'Publish the skeleton on /tf and the anchors on /tf_static.')
        declare('publish_frame', True, 'Publish mocap_frame (movin_msgs/MocapFrame).')
        declare('publish_markers', False,
                'Publish skeleton_markers (visualization_msgs/MarkerArray); requires publish_tf.')
        declare('frame_qos_reliable', True, 'mocap_frame QoS: true = RELIABLE/VOLATILE depth 10 '
                '(bag friendly), false = sensor data (best effort).')
        declare('stale_timeout_sec', 1.0,
                'status.receiving becomes false after this long without a frame.')
        declare('retry_bind_sec', 0.0, 'Bind failure policy: 0 = fatal (exit code 1), '
                '> 0 = stay alive and retry every N seconds (error shown in status).')
        declare('max_frames', 0, 'Exit with code 0 after publishing this many frames (0 = never).')

    def _read_parameters(self) -> None:
        def text(name):
            return str(self.get_parameter(name).value).strip()

        def flag(name):
            return bool(self.get_parameter(name).value)

        def number(name, minimum=None, strict=False):
            value = float(self.get_parameter(name).value)
            if minimum is not None and (value < minimum or (strict and value <= minimum)):
                bound = 'greater than' if strict else 'at least'
                raise ValueError(f'parameter {name} must be {bound} {minimum:g}, got {value:g}')
            return value

        self.mode = text('mode')
        if self.mode not in frame_source.MODES:
            raise ValueError(f'unknown mode {self.mode!r}; expected one of {frame_source.MODES}')
        self.host = text('host')
        self.port = int(self.get_parameter('port').value)
        if not 0 <= self.port <= 65535:
            raise ValueError(f'parameter port must be in 0..65535, got {self.port}')
        self.recording = text('recording')
        self.replay_loop = flag('replay_loop')
        self.replay_realtime = flag('replay_realtime')
        self.replay_exit_on_end = flag('replay_exit_on_end')
        self.record = text('record')
        self.record_dir = text('record_dir')
        self.record_segment_sec = number('record_segment_sec', 0.0)
        self.record_stream_type = text('record_stream_type')
        if self.record_stream_type not in STREAM_TYPES:
            raise ValueError(
                f'unknown record_stream_type {self.record_stream_type!r}; '
                f'expected one of {STREAM_TYPES}')
        self.preset = text('preset')
        if self.preset not in PRESET_CHOICES:
            raise ValueError(f'unknown preset {self.preset!r}; expected one of {PRESET_CHOICES}')
        self.poll_rate_hz = number('poll_rate_hz', 0.0, strict=True)
        self.publish_every_frame = flag('publish_every_frame')
        self.actor_filter = text('actor_filter')
        self.forward_mode = text('forward_mode')
        if self.forward_mode not in convert.FORWARD_MODES:
            raise ValueError(
                f'unknown forward_mode {self.forward_mode!r}; '
                f'expected one of {convert.FORWARD_MODES}')
        self.world_frame = text('world_frame')
        self.tf_prefix = text('tf_prefix').strip('/')
        if not self.world_frame or not self.tf_prefix:
            raise ValueError('parameters world_frame and tf_prefix must not be empty')
        offset = [float(c) for c in self.get_parameter('skeleton_offset').value]
        if len(offset) != 3:
            raise ValueError(f'parameter skeleton_offset must have 3 elements, got {offset}')
        self.skeleton_offset = tuple(offset)
        self.publish_tf = flag('publish_tf')
        self.publish_frame = flag('publish_frame')
        self.publish_markers = flag('publish_markers')
        if self.publish_markers and not self.publish_tf:
            self.get_logger().error('publish_markers requires publish_tf; markers disabled')
            self.publish_markers = False
        self.frame_qos_reliable = flag('frame_qos_reliable')
        self.stale_timeout_sec = number('stale_timeout_sec', 0.0)
        self.retry_bind_sec = number('retry_bind_sec', 0.0)
        self.max_frames = int(self.get_parameter('max_frames').value)
        if self.max_frames < 0:
            raise ValueError(f'parameter max_frames must be >= 0, got {self.max_frames}')
        if self.mode == 'replay' and not self.recording:
            raise ValueError("mode 'replay' requires the 'recording' parameter (SDK .pkl path)")

    # ------------------------------------------------------------- interfaces

    def _create_interfaces(self) -> None:
        if self.frame_qos_reliable:
            frame_qos = qos.QoSProfile(
                depth=10,
                reliability=qos.ReliabilityPolicy.RELIABLE,
                durability=qos.DurabilityPolicy.VOLATILE)
        else:
            frame_qos = qos.qos_profile_sensor_data
        if self.publish_frame:
            self.frame_pub = self.create_publisher(MocapFrame, 'mocap_frame', frame_qos)
        self.status_pub = self.create_publisher(MocapStatus, 'status', 10)
        if self.publish_markers:
            self.marker_pub = self.create_publisher(MarkerArray, 'skeleton_markers', 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
            self.static_broadcaster = StaticTransformBroadcaster(self)
        self.create_service(StartRecording, 'start_recording', self._on_start_recording)
        self.create_service(StopRecording, 'stop_recording', self._on_stop_recording)
        self.create_service(Trigger, 'split_recording', self._on_split_recording)

    def _install_sdk_logging_bridge(self) -> None:
        sdk_logger = logging.getLogger(SDK_LOGGER_NAME)
        self._log_handler = RclpyLogHandler(self.get_logger().get_child('sdk'))
        sdk_logger.addHandler(self._log_handler)
        sdk_logger.setLevel(logging.INFO)
        sdk_logger.propagate = False

    # ---------------------------------------------------------------- session

    def _create_session(self) -> None:
        recorder = None
        initial = None
        if self.mode == 'live' and self.record:
            # Attach-before-start: the recorder must see the very first packet.  The path
            # is probed for writability here so a bad record/record_dir fails at startup.
            path = recording.resolve_recording_path(
                self.record, self.record_dir, self.record_segment_sec)
            recorder = recording.CountingRecorder(path, stream_type=self.record_stream_type)
            initial = (recorder, path)
        elif self.record:
            self.get_logger().warning('record parameter ignored in replay mode')
        receiver = frame_source.build_receiver(
            self.mode, host=self.host, port=self.port, recording=self.recording,
            replay_realtime=self.replay_realtime, replay_loop=self.replay_loop,
            recorder=recorder)
        if self.mode == 'replay':
            self._inspect_replay(receiver)
        self._receiver = receiver
        self._sink = FramePublisherSink(self)
        self._session = MovinSession(
            receiver=receiver, sinks=[self._sink], on_extension_error=self._on_sdk_error)
        self._recorder_manager = recording.RecorderManager(
            receiver, record_dir=self.record_dir, segment_sec=self.record_segment_sec,
            stream_type=self.record_stream_type, logger=self._safe_logger, initial=initial)
        if initial is not None:
            self.get_logger().info(f'recording from the first packet to {initial[1]}')

    def _inspect_replay(self, receiver) -> None:
        """
        Log the recording summary and take the preset from its first frame (file read once).

        A recording without a complete frame is fatal: the SDK replay thread
        would busy-loop on it with ``replay_loop`` and the end of a non-looping
        replay could never be detected (no frame ever arrives).
        """
        info = frame_source.recording_summary(receiver.player)
        self.get_logger().info(
            f'replaying {self.recording}: {info["num_messages"]} messages, '
            f'{info["duration_sec"]:.1f}s, stream_type={info["stream_type"]}, '
            f'loop={self.replay_loop}, realtime={self.replay_realtime}'
            + ('' if self.replay_realtime else
               f' (lossless, up to {frame_source.FAST_REPLAY_FRAMES_PER_POLL} frames per poll)'))
        frame = frame_source.first_complete_frame(receiver.player)
        if frame is None:
            raise ValueError(
                f'{self.recording} contains no complete frame ({info["num_messages"]} messages); '
                'nothing to replay')
        preset = frame_source.detect_preset(frame)
        self.get_logger().info(
            f'first recorded frame: actor {frame.get("actor")!r}, {len(frame["bones"])} bones, '
            f'preset {preset or "unknown"}')
        if self.preset == 'auto':
            self._peeked_preset = preset

    def _start_session(self) -> bool:
        try:
            self._session.start()
        except OSError as exc:
            self._start_error = str(exc)
            self._bind_address = ''
            return False
        self._start_error = ''
        self._bind_address = frame_source.bound_address(self._receiver)
        if self._recorder_manager is not None:
            # segment rotation counts from the moment packets can actually arrive
            self._recorder_manager.reset_segment_start()
        if self.mode == 'live':
            self.get_logger().info(
                f'listening on UDP {self._bind_address} '
                "(MOVIN Studio destination: this machine's IP and this port)")
        return True

    def _arm_retry_timer(self) -> None:
        if self._retry_timer is None:
            self._retry_timer = self.create_timer(self.retry_bind_sec, self._on_retry_bind)
        else:
            self._retry_timer.reset()

    def _on_retry_bind(self) -> None:
        if self._start_session():
            self.get_logger().info('receiver started after retry')
            self._retry_timer.cancel()
            return
        self.get_logger().warning(
            f'bind {self.host}:{self.port} still failing ({self._start_error}); '
            f'retrying in {self.retry_bind_sec:g}s',
            throttle_duration_sec=max(WARN_THROTTLE_SEC, 5.0 * self.retry_bind_sec))

    def _on_receiver_died(self) -> None:
        """
        Handle the SDK UDP thread ending on its own (exception in record()/ingest()).

        The receiver keeps reporting ``running`` and the old bind address, so
        without this check the node would look healthy forever while nothing
        is received.  Policy follows ``retry_bind_sec``: fatal (exit code 1)
        when 0, otherwise the receiver is restarted through the bind-retry path.
        """
        error = 'receiver thread died (see the traceback above); no packets are processed'
        self.get_logger().error(error)
        try:
            self._session.stop()
        except Exception as exc:  # the socket is already closed; keep going
            self.get_logger().warning(f'receiver stop after thread death failed: {exc!r}')
        self._start_error = error
        self._bind_address = ''
        if self.retry_bind_sec <= 0.0:
            self.request_exit('receiver thread died', exit_code=1)
            return
        self.get_logger().warning(f'restarting the receiver in {self.retry_bind_sec:g}s')
        self._arm_retry_timer()

    def _on_sdk_error(self, extension, exc) -> None:
        self._sink_errors += 1
        self.get_logger().warning(
            f'frame sink error #{self._sink_errors}: {exc!r}',
            throttle_duration_sec=WARN_THROTTLE_SEC)

    # ----------------------------------------------------------------- timers

    def _on_poll(self) -> None:
        session = self._session
        if session is None or not session.is_running:
            return
        receiver = self._receiver
        if frame_source.receiver_thread_died(receiver):
            self._on_receiver_died()
            if self._exit_reason:
                raise ExitRequested(self._exit_reason)
            return
        pump = getattr(receiver, 'pump', None)
        if pump is not None:  # FastReplayReceiver: pull the next frames (lossless)
            pump(frame_source.FAST_REPLAY_FRAMES_PER_POLL)
        if self.publish_every_frame:
            frames, delivery = frame_source.drain_frames(receiver)
        else:
            frame = receiver.get_latest_frame()
            frames, delivery = ([frame] if frame is not None else []), 'latest_only'
        if delivery != self._delivery:
            self._delivery = delivery
            if delivery == 'latest_only' and self.publish_every_frame:
                self.get_logger().warning(
                    'SDK receiver has no get_new_frames(); falling back to '
                    'get_latest_frame() (intermediate frames may be skipped)')
        self._process_frames(frames)
        if (self.mode == 'replay' and not self._replay_finished
                and frame_source.replay_finished(receiver)):
            # The replay thread may have completed frames after the drain above.
            self._process_frames(frame_source.drain_frames(receiver)[0])
            self._replay_finished = True
            self.get_logger().info(
                f'replay finished ({self._sink.frames_published} frames published)')
            if self.replay_exit_on_end:
                self.request_exit('replay finished')
        if self._exit_reason:
            raise ExitRequested(self._exit_reason)

    def _process_frames(self, frames) -> None:
        for frame in frames:
            self._last_frame_monotonic = time.monotonic()
            try:
                self._session.process_frame(frame)
            except Exception as exc:  # MovinSession isolates sink errors; second safety net
                self.get_logger().warning(
                    f'frame processing failed: {exc!r}', throttle_duration_sec=WARN_THROTTLE_SEC)
            if self._exit_reason:
                break

    def _on_status(self) -> None:
        now = time.monotonic()
        manager = self._recorder_manager
        session = self._session
        if manager is not None and session is not None and session.is_running:
            # no rotation while the socket is not bound: nothing can be recorded yet
            try:
                manager.tick(now)
            except Exception as exc:  # a failed rotation must not stop the status timer
                self.get_logger().error(f'recording split failed: {exc!r}')
        self.status_pub.publish(self._status_message(now))

    def _status_message(self, now: float) -> MocapStatus:
        sink = self._sink
        manager = self._recorder_manager
        session = self._session
        running = session is not None and session.is_running
        msg = MocapStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.mode = self.mode
        last = self._last_frame_monotonic
        msg.receiving = last is not None and (now - last) < self.stale_timeout_sec
        msg.bind_address = self._bind_address if running else ''
        msg.error = self._start_error
        # The SDK only recomputes its packet rate while packets arrive, so the last value
        # would be republished forever after the stream stopped; report 0 when not receiving.
        msg.receive_rate_hz = (
            float(self._receiver.get_receive_rate()) if running and msg.receiving else 0.0)
        msg.frame_delivery = self._delivery
        if sink is not None:
            msg.frames_published = int(sink.frames_published)
            msg.frames_dropped = int(sink.gaps.frames_dropped)
            msg.actors = sorted(sink.actor_ids)
        msg.preset = self.current_preset()
        if manager is not None:
            msg.recording = bool(manager.is_recording)
            msg.recording_path = manager.current_path
            msg.recording_segment = int(manager.segment_index)
            msg.recording_messages = int(manager.messages)
            msg.recording_duration_sec = float(manager.duration_sec)
        msg.replay_finished = self._replay_finished
        return msg

    def current_preset(self) -> str:
        """Return the explicit preset, else the last detected, else the peeked replay preset."""
        if self.preset != 'auto':
            return self.preset
        if self._sink is not None and self._sink.last_preset:
            return self._sink.last_preset
        return self._peeked_preset

    def request_exit(self, reason: str, exit_code: int = 0) -> None:
        """Ask the node to leave ``rclpy.spin`` after the current poll with ``exit_code``."""
        if not self._exit_reason:
            self._exit_reason = reason
            self.exit_code = int(exit_code)
            self.get_logger().info(f'{reason}; shutting down')

    # --------------------------------------------------------------- services

    def _on_start_recording(self, request, response):
        try:
            path = self._recorder_manager.start(request.path, request.stream_type or None)
        except (RuntimeError, OSError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
            response.path = ''
            return response
        response.success = True
        response.message = f'recording to {path}'
        response.path = path
        return response

    def _on_stop_recording(self, request, response):
        manager = self._recorder_manager
        try:
            path, count, duration = manager.stop(save=bool(request.save))
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            return response
        except Exception as exc:  # save failed: the recording is still active (see stop())
            response.success = False
            response.path = manager.current_path
            response.num_messages = int(manager.messages)
            response.duration_sec = float(manager.duration_sec)
            response.message = (
                f'saving failed: {exc}; the recording is still active, fix the path or disk '
                'and call stop_recording again (save: false discards it)')
            return response
        response.success = True
        response.path = path
        response.num_messages = int(count)
        response.duration_sec = float(duration)
        if not request.save:
            response.message = f'discarded {count} messages ({duration:.1f}s) {path}'
        elif count == 0:
            response.message = f'{recording.NOTHING_RECORDED}, {path} not written'
        else:
            response.message = f'saved {count} messages ({duration:.1f}s) {path}'
        return response

    def _on_split_recording(self, request, response):
        try:
            old_path, new_path = self._recorder_manager.split()
        except (RuntimeError, OSError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = f'{old_path} -> {new_path}'
        return response

    # --------------------------------------------------------------- shutdown

    def destroy_node(self):
        """Stop the receiver, flush recordings, detach the SDK log bridge, destroy the node."""
        self._shutdown_sdk()
        if self._node_destroyed:
            return True
        self._node_destroyed = True
        return super().destroy_node()

    def _shutdown_sdk(self) -> None:
        """Release SDK resources; every step is skipped once done so a retry can resume."""
        logger = self._safe_logger
        for timer in (self._poll_timer, self._status_timer, self._retry_timer):
            if timer is not None:
                timer.cancel()
        if self._session is not None and self._session.is_running:
            try:
                self._session.stop()
            except Exception as exc:  # keep going: the recording must still be saved
                logger.error(f'receiver stop failed: {exc!r}')
        if self._recorder_manager is not None and not self._recorder_closed:
            try:
                self._recorder_manager.close()
            except Exception as exc:
                logger.error(f'saving the recording failed: {exc!r}')
            self._recorder_closed = True
        if self._log_handler is not None:
            logging.getLogger(SDK_LOGGER_NAME).removeHandler(self._log_handler)
            self._log_handler = None
        if self._sink is not None and not self._summary_logged:
            self._summary_logged = True
            logger.info(
                f'published {self._sink.frames_published} frames, '
                f'{self._sink.gaps.frames_dropped} dropped')

    def _fatal(self, message: str) -> None:
        self.startup_error = message
        self.get_logger().fatal(message)


def is_shutdown_race(exc: BaseException, context_ok: bool) -> bool:
    """
    Return True for the errors rclpy raises when a signal shut the context down mid-spin.

    rclpy's SIGINT/SIGTERM handler shuts the context down from its own thread.
    When that lands between two executor iterations (likely with a fast poll
    timer) ``rclpy.spin`` raises ``RCLError('failed to initialize wait set: the
    given context is not valid ...')`` from the wait set, or a pybind11
    ``RuntimeError('Unable to convert call argument ...')`` from a subscription
    take, instead of ``ExternalShutdownException`` (observed on Humble and
    Jazzy, about every second Ctrl-C).  ``RCLError`` is a ``RuntimeError``;
    any ``RuntimeError`` after the context is gone is an ordinary shutdown, not
    a failure, so ``main`` must not let it escape as exit code 1.
    """
    return not context_ok and isinstance(exc, RuntimeError)


def shutdown(node) -> None:
    """Destroy ``node`` and rclpy with further SIGINT/SIGTERM deferred (see SignalGuard)."""
    with SignalGuard() as guard:
        if node is not None:
            for _attempt in range(3):
                try:
                    node.destroy_node()
                    break
                except KeyboardInterrupt:  # signal that slipped in before the guard was armed
                    continue
        rclpy.try_shutdown()
    if guard.received:
        get_logger(PROCESS_LOGGER_NAME).info(
            f'{len(guard.received)} signal(s) received during shutdown were deferred')


def main(args=None) -> int:
    """Run ``mocap_node``; return 1 on a fatal startup error or a dead receiver, else 0."""
    rclpy.init(args=args)
    exit_code = 0
    node = None
    try:
        node = MocapNode()
        if node.startup_error:
            exit_code = 1
        else:
            rclpy.spin(node)
    except ExitRequested as exc:
        exit_code = node.exit_code
        get_logger(PROCESS_LOGGER_NAME).info(f'exiting: {exc} (exit code {exit_code})')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if not is_shutdown_race(exc, rclpy.ok()):
            raise
    finally:
        shutdown(node)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
