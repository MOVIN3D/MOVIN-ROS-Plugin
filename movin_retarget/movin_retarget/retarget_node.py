"""
Retarget MOVIN Studio mocap frames to a Unitree G1 and publish joint states.

``retarget_node`` subscribes ``mocap_frame`` (``movin_msgs/MocapFrame``,
normally under the ``/movin`` namespace, from live reception, replay or
``ros2 bag play``), runs the SDK ``Retargeter`` on every frame of the selected
actor and publishes ``joint_states`` (``sensor_msgs/JointState``, joint names
taken from the Retargeter's MuJoCo model) plus the ``world_frame ->
robot_root_frame`` transform.  ``robot_state_publisher`` (see
``launch/retarget.launch.py``) turns the joint states into the remaining
robot link transforms.

The Retargeter needs the SDK's optional ``[retargeting]`` extra; when it is
missing ``main()`` logs a fatal message with the pip hint and returns 1.
"""

from __future__ import annotations

import os
import signal

from geometry_msgs.msg import TransformStamped
from movin_msgs.msg import MocapFrame
from movin_retarget import retarget_core as core
import movin_sdk_python
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

try:  # Jazzy: rclpy.event_handler; Humble: rclpy.qos_event (deprecated alias on Jazzy)
    from rclpy.event_handler import SubscriptionEventCallbacks, UnsupportedEventTypeError
except ImportError:  # pragma: no cover - Humble
    from rclpy.qos_event import SubscriptionEventCallbacks, UnsupportedEventTypeError

WARN_THROTTLE_SEC = 2.0
# Console-only logger for messages after the context is gone (must not equal a node logger name).
PROCESS_LOGGER_NAME = 'retarget_node.process'


class RetargetNode(Node):
    """rclpy node wrapping :mod:`movin_retarget.retarget_core`."""

    def __init__(self):
        super().__init__('retarget_node')
        self.get_logger().info(
            f'movin_sdk_python {getattr(movin_sdk_python, "__version__", "?")} from '
            f'{os.path.dirname(os.path.abspath(movin_sdk_python.__file__))}')
        self._declare_parameters()

        self.robot = self._str('robot')
        self.human_height = core.parse_human_height(self.get_parameter('human_height').value)
        self.source_preset = self._str('source_preset')
        self.forward_mode = self._str('forward_mode')
        self.world_frame = self._str('world_frame')
        self.robot_root_frame = self._str('robot_root_frame')
        self.robot_offset = [float(c) for c in self.get_parameter('robot_offset').value]
        self.offset_to_ground = bool(self.get_parameter('offset_to_ground').value)
        self.max_rate_hz = self._float('max_rate_hz')
        self.actor = self._str('actor')
        frame_qos_reliable = bool(self.get_parameter('frame_qos_reliable').value)

        if self.robot not in core.ROBOT_TYPES:
            raise ValueError(f'unknown robot {self.robot!r}; expected one of {core.ROBOT_TYPES}')
        if self.source_preset not in core.SOURCE_PRESETS:
            raise ValueError(
                f'unknown source_preset {self.source_preset!r}; '
                f'expected one of {core.SOURCE_PRESETS}'
            )
        if self.forward_mode not in core.FORWARD_MODES:
            raise ValueError(
                f'unknown forward_mode {self.forward_mode!r}; expected one of {core.FORWARD_MODES}'
            )
        if len(self.robot_offset) != 3:
            raise ValueError(f'robot_offset must have 3 elements, got {self.robot_offset}')
        low, high = core.PLAUSIBLE_HUMAN_HEIGHT_M
        if self.human_height != core.HUMAN_HEIGHT_AUTO and not low <= self.human_height <= high:
            self.get_logger().warning(
                f"human_height={self.human_height:g} is the actor's standing height in meters "
                f'(body scale {core.REFERENCE_HEIGHT_M:g} / human_height); expected '
                f'{low:g}..{high:g} m. Use human_height:=auto to measure the actor.')

        self._retargeter = None
        self._joint_names = []
        self._disabled = False
        self._rate = core.RateLimiter(self.max_rate_hz)
        self._actors = core.ActorSelector(self.actor)
        self._ignored_actors = set()
        self._body_scale = None
        self.frames_received = 0
        self.frames_published = 0

        qos = QoSProfile(depth=10)
        qos.reliability = (
            ReliabilityPolicy.RELIABLE if frame_qos_reliable else ReliabilityPolicy.BEST_EFFORT
        )
        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._qos_name = 'RELIABLE' if frame_qos_reliable else 'BEST_EFFORT'
        try:
            self._subscription = self.create_subscription(
                MocapFrame, 'mocap_frame', self._on_frame, qos,
                event_callbacks=SubscriptionEventCallbacks(
                    incompatible_qos=self._on_incompatible_qos),
            )
        except UnsupportedEventTypeError:  # rmw without QoS events: no diagnostic, same data
            self._subscription = self.create_subscription(
                MocapFrame, 'mocap_frame', self._on_frame, qos
            )

        self.get_logger().info(
            f'robot={self.robot} human_height={self.human_height} '
            f'source_preset={self.source_preset} forward_mode={self.forward_mode} '
            f'{self.world_frame}->{self.robot_root_frame} robot_offset={self.robot_offset} '
            f'max_rate_hz={self.max_rate_hz:g} actor={self.actor or "<first seen>"}; '
            'waiting for mocap_frame'
        )

    # ------------------------------------------------------------------ params

    def _declare_parameters(self) -> None:
        """Declare every parameter; all are read once at startup, hence ``read_only``."""
        def declare(name, default, description, dynamic=None):
            if dynamic is None:
                dynamic = isinstance(default, float) or (
                    isinstance(default, list) and all(isinstance(v, float) for v in default))
            self.declare_parameter(
                name, default, ParameterDescriptor(
                    description=description, read_only=True, dynamic_typing=dynamic))

        declare('robot', 'unitree_g1',
                'Retargeter robot type: unitree_g1 (29 joints) or unitree_g1_with_hands (43).')
        # Dynamically typed: 'auto' is a string, a height may arrive as a double or an integer.
        declare('human_height', core.HUMAN_HEIGHT_AUTO,
                "Actor size for retargeting: 'auto' measures it from the streamed bones (and "
                "again when Studio streams new proportions); a number is the actor's standing "
                f'height in meters (body scale = {core.REFERENCE_HEIGHT_M:g} / human_height).',
                dynamic=True)
        declare('source_preset', 'auto',
                'Source skeleton preset: auto (from the MocapFrame.preset field), '
                'movinman or movinman_v3.')
        declare('forward_mode', 'isaac_world',
                'Must match mocap_node: coord_equivalent (Unity +Z -> ROS -Y) or '
                'isaac_world (Unity +Z -> ROS +X, REP-103).')
        declare('world_frame', 'world', 'Parent frame of the robot root transform.')
        declare('robot_root_frame', 'pelvis',
                'Child frame of the root transform (URDF root link of the robot).')
        declare('robot_offset', [0.0, 0.0, 0.0],
                'Translation added to the robot root position in world_frame, meters (x, y, z).')
        declare('offset_to_ground', False,
                'Let the Retargeter shift the skeleton so the lowest foot touches the ground.')
        declare('max_rate_hz', 60.0,
                'Upper bound of the average retargeting (IK) rate; surplus frames are skipped, '
                'short bursts of jittered frames pass. 0 = no limit.')
        declare('actor', '', 'Actor name to retarget; empty = lock onto the first actor seen.')
        declare('frame_qos_reliable', True,
                'Subscribe mocap_frame with RELIABLE (true) or BEST_EFFORT (false) QoS; '
                'must match the mocap_node publisher.')

    def _on_incompatible_qos(self, event) -> None:
        """Log a mocap_frame publisher whose QoS does not match this subscription."""
        self.get_logger().warning(
            f'a mocap_frame publisher is incompatible with this {self._qos_name} subscription '
            f'(policy {getattr(event, "last_policy_kind", "?")}); no frames will arrive from '
            'it. Set frame_qos_reliable on retarget_node to the value used by mocap_node '
            '(launch argument frame_qos_reliable sets both).',
            throttle_duration_sec=WARN_THROTTLE_SEC,
        )

    def _str(self, name: str) -> str:
        return str(self.get_parameter(name).value).strip()

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    # ---------------------------------------------------------------- callback

    def _on_frame(self, msg) -> None:
        self.frames_received += 1
        if self._disabled:
            return
        if not self._actors.accept(msg.actor):
            if msg.actor not in self._ignored_actors:
                self._ignored_actors.add(msg.actor)
                self.get_logger().info(
                    f'ignoring actor {msg.actor!r} (retargeting {self._actors.selected!r})'
                )
            return
        if not self._rate.accept():
            return
        try:
            core.check_convention(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=WARN_THROTTLE_SEC)
            return

        bones = core.bones_from_message(msg)
        if self._retargeter is None:
            self._create_retargeter(msg, bones)
            if self._retargeter is None:
                return

        try:
            result = core.run_retarget(self._retargeter, bones, self.offset_to_ground)
        except core.MissingBonesError as exc:
            self.get_logger().warning(
                f'{exc} (actor {msg.actor!r}, {len(bones)} bones, preset '
                f'{self._retargeter.source_preset!r}); check the source_preset parameter',
                throttle_duration_sec=WARN_THROTTLE_SEC,
            )
            return
        except Exception as exc:  # IK failures must not kill the node
            self.get_logger().warning(
                f'retargeting failed: {exc!r}', throttle_duration_sec=WARN_THROTTLE_SEC
            )
            return

        if len(result.joint_positions) != len(self._joint_names):
            self.get_logger().error(
                f'qpos has {len(result.joint_positions)} joint values but the model lists '
                f'{len(self._joint_names)} hinge joints; retargeting disabled'
            )
            self._disabled = True
            return

        self._report_body_scale()
        self._publish(result, msg)

    def _report_body_scale(self) -> None:
        """Log the Retargeter's body scale when it is first known and whenever it changes."""
        scale = getattr(self._retargeter, 'body_scale', None)
        if scale is None:
            return
        if self._body_scale is not None and abs(scale - self._body_scale) < 1e-4:
            return
        source = ('measured from the stream' if self.human_height == core.HUMAN_HEIGHT_AUTO
                  else f'from human_height={self.human_height:g} m')
        self.get_logger().info(
            f'body scale to the {core.REFERENCE_HEIGHT_M:g} m reference body: {scale:.4f} '
            f'({source})')
        self._body_scale = scale

    def _create_retargeter(self, msg, bones) -> None:
        """Build the Retargeter on the first accepted frame (preset from the message)."""
        preset = core.resolve_source_preset(
            self.source_preset, msg.preset, [b['bone_name'] for b in bones]
        )
        try:
            self._retargeter = core.create_retargeter(self.robot, self.human_height, preset)
            self._joint_names = core.hinge_joint_names(self._retargeter.model)
        except ValueError as exc:
            self._retargeter = None
            self._disabled = True
            self.get_logger().error(
                f'cannot retarget actor {msg.actor!r}: {exc}; retargeting disabled'
            )
            return
        self.get_logger().info(
            f'retargeting actor {msg.actor!r} ({len(bones)} bones, source_preset={preset}) '
            f'to {self.robot} with {len(self._joint_names)} joints'
        )

    def _publish(self, result, msg) -> None:
        pos, quat = core.root_pose(
            result.root_pos, result.root_quat_wxyz, self.forward_mode, self.robot_offset
        )
        stamp = self.get_clock().now().to_msg()

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(self._joint_names)
        joint_state.position = list(result.joint_positions)
        self._joint_pub.publish(joint_state)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.robot_root_frame
        transform.transform.translation.x = pos[0]
        transform.transform.translation.y = pos[1]
        transform.transform.translation.z = pos[2]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self._tf_broadcaster.sendTransform(transform)
        self.frames_published += 1

    def destroy_node(self):
        """Log the frame counters and destroy the node."""
        message = f'received {self.frames_received} frames, published {self.frames_published}'
        if self.context.ok():
            self.get_logger().info(message)
        else:  # context already shut down by the signal handler: console only, no rosout
            get_logger(PROCESS_LOGGER_NAME).info(message)
        return super().destroy_node()


class SignalGuard:
    """
    Defer SIGINT/SIGTERM while the node shuts down (same as ``movin_ros.mocap_node.SignalGuard``).

    A terminal Ctrl-C reaches every process of the foreground group and
    ``ros2 launch`` then signals each child again, so a node normally receives
    two SIGINTs.  rclpy turns the first one into the end of ``spin``; the second
    would surface as a ``KeyboardInterrupt`` in the middle of ``destroy_node``.
    Inside the guard both signals are only counted and on exit they are ignored
    for the rest of the process; must be entered from the main thread
    (elsewhere it is a no-op).
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

        ``signal.signal`` only reports Python's own handler table (rclpy installs
        its handler with ``sigaction`` behind Python's back), so restoring the
        "previous" handlers would re-arm ``default_int_handler`` / ``SIG_DFL``
        and a late SIGINT or SIGTERM during interpreter finalization would end
        the process with -2 / 143.  SIGKILL (launch's escalation) still works.
        """
        for signum in self._previous:
            try:
                signal.signal(signum, signal.SIG_IGN)
            except (ValueError, OSError, TypeError):
                pass
        return False


def is_shutdown_race(exc: BaseException, context_ok: bool) -> bool:
    """
    Return True for the errors rclpy raises when a signal shut the context down mid-spin.

    rclpy's SIGINT/SIGTERM handler shuts the context down from its own thread;
    when that lands between two executor iterations ``rclpy.spin`` raises
    ``RCLError('failed to initialize wait set: the given context is not valid
    ...')`` or a pybind11 ``RuntimeError('Unable to convert call argument
    ...')`` from the ``mocap_frame`` subscription take instead of
    ``ExternalShutdownException`` (Humble and Jazzy).  ``RCLError`` is a
    ``RuntimeError``; after the context is gone that is an ordinary shutdown,
    not a failure (same helper as ``movin_ros.mocap_node``).
    """
    return not context_ok and isinstance(exc, RuntimeError)


def _import_retargeting_dependencies() -> None:
    """Import the SDK Retargeter and mujoco (raises ImportError with the pip hint when absent)."""
    from movin_sdk_python import Retargeter  # noqa: F401
    import mujoco  # noqa: F401


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
    """Run ``retarget_node``; return 1 when the retargeting extra is missing."""
    rclpy.init(args=args)
    node = None
    try:
        logger = get_logger('retarget_node')
        try:
            _import_retargeting_dependencies()
        except ImportError as exc:
            logger.fatal(core.missing_dependency_message(exc))
            return 1
        try:
            node = RetargetNode()
        except (ParameterException, ValueError) as exc:
            logger.fatal(str(exc))
            return 1
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if not is_shutdown_race(exc, rclpy.ok()):
            raise
    finally:
        shutdown(node)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
