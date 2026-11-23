# MOVIN ROS Plugin reference

Detailed behaviour, every parameter, and the rationale behind the conventions. The
[README](../README.md) covers everyday use; this file is the long form.

## Docker details

`docker/Dockerfile` builds a development image on top of `ros:<distro>` (pip, pytest,
`launch_testing_ros`, `tf2_tools`; `--build-arg WITH_RVIZ=1` adds rviz2 + Xvfb,
`--build-arg WITH_RETARGETING=1` adds mujoco/mink/numpy/scipy). The repository is never copied into
the image; it is bind-mounted as the workspace source.

```bash
# Build the workspace and run the node as your own user, with the host network (UDP + DDS).
./docker/dev.sh jazzy colcon build                    # builds movin-ros-dev:jazzy on first use
./docker/dev.sh jazzy ros2 launch movin_ros mocap.launch.py record:=auto
./docker/dev.sh humble bash                           # interactive shell, Humble

# Same with Docker Compose (services: jazzy, humble; runs as MOVIN_UID:MOVIN_GID, default 1000:1000)
docker compose -f docker/compose.yaml run --rm jazzy colcon build
docker compose -f docker/compose.yaml run --rm jazzy ros2 launch movin_ros mocap.launch.py record:=auto

# Image variants
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble -t movin-ros-dev:humble .
docker build -f docker/Dockerfile --build-arg WITH_RETARGETING=1 -t movin-ros-dev:jazzy-retarget .
MOVIN_IMAGE=movin-ros-dev:jazzy-retarget ./docker/dev.sh jazzy ros2 launch movin_ros mocap.launch.py robot:=unitree_g1
```

`dev.sh` keeps the repository clean: colcon `build/`, `install/` and `log/` go to
`${MOVIN_WS_CACHE:-~/.cache/movin_ros_ws}/<distro>` (mounted at `/ws/.cache` and sourced
automatically on the next run), and `${MOVIN_RECORD_DIR:-~/movin_recordings}` is mounted as the
container's `~/movin_recordings`, the default `record_dir`. `docker/ci.sh <distro>
[--symlink-install]` reproduces the CI run locally (rosdep, build, tests, SDK unit tests) in a
throw-away container.

## Live reception, per frame

What happens per frame:

- The skeleton preset (`movinman` 51 bodies or `movinman_v3` 54 bodies + `RootBone`) is
  detected from the bone names of the first frame of each actor (`preset:=auto`), or forced with
  `preset:=movinman_v3`.
- Every actor gets its own id (`actor` sanitized to `[A-Za-z0-9_]`, collisions get a 4-hex-digit
  suffix), a static anchor transform `world -> movin/<actor_id>` on `/tf_static` and one
  `/tf` message per frame carrying all bone edges. Anchors are published once per actor and never
  change for the node's lifetime. `actor_filter:=<name>` publishes one actor only.
- `publish_every_frame:=true` (default) takes every frame assembled since the last poll with the
  SDK's `get_new_frames()` (`poll_rate_hz`, default 120 Hz) so no assembled frame is skipped. The
  SDK keeps at most 4 complete frames between polls, so delivery is lossless as long as the poll
  rate stays above a quarter of the frame rate; gaps in `frame_idx` are counted in
  `status.frames_dropped`. `publish_every_frame:=false` publishes only the newest frame per poll
  (`frame_delivery: latest_only`).
- The SDK receiver answers MOVIN Studio's OSC status probe (`/MOVIN/OSC/Status/Request`, about
  once a second) from the receiving socket, so Studio's Streaming panel shows `OSC connected`,
  the Motion fps (complete frames received for the character) and the Viewer fps (frames handed
  to `mocap_node`, i.e. published). Without an answer Studio shows `not connected` although
  frames arrive. Replay mode sends nothing.
- `status.receiving` turns `false` when no frame arrived for `stale_timeout_sec` (1 s), and
  `receive_rate_hz` reports `0` while not receiving (the SDK only updates its packet rate while
  packets arrive, so the last value is not repeated forever).
- A bind failure (port in use, bad `host`) is fatal (exit code 1) unless `retry_bind_sec > 0`, in
  which case the node stays up, reports `status.error` and retries. The same policy applies if the
  SDK receive thread ever dies (for example on `MemoryError` from a huge single-file recording):
  the node logs it, sets `status.error` and exits 1 or restarts the receiver.

Direct node invocation (all launch arguments are plain parameters):

```bash
ros2 run movin_ros mocap_node --ros-args -r __ns:=/movin -p mode:=live -p port:=11235 -p record:=auto
```

## Recording details

Recording captures the **raw OSC stream** with the SDK's `OscRecorder` on the UDP receive thread,
before frame assembly and independent of ROS polling, so a recording is complete even if the
ROS side lags. The output is the SDK's versioned pickle (`version: 1`, plain Python objects, no
NumPy), the same file the SDK examples and the Isaac plugin write and read.

- **Segments.** With `record_segment_sec` > 0 (default 600 s) the recording is rotated into
  numbered files (`movin_<timestamp>_000.pkl`, `_001.pkl`, ...). A split swaps in a new recorder
  under the receiver lock and saves the previous segment on a background thread: no packet is lost
  at the boundary and the 120 Hz poll timer is not blocked by `pickle.dump`. `split_recording`
  does the same on demand. Messages are kept in RAM until a segment is saved (roughly 25 KB per
  frame, about 1 GB per 10 minutes at 60 Hz with 55 bones; ~7 KB per frame on disk), which is why
  the default segment length is 10 minutes.
- **Status.** `/movin/status` reports `recording`, `recording_path`, `recording_segment`,
  `recording_messages` and `recording_duration_sec`; `StopRecording` answers with the saved path,
  `num_messages` and `duration_sec`. Message counts include every OSC message received, also
  Studio's status probes (about one per second, ignored by replay); `recording_info` prints the
  count per OSC address and the number of complete frames.
- **Nothing lost, nothing empty.** The output path is probed for writability when the recording
  starts (`record:=` at startup is fatal with exit 1 if the path is a directory or not writable;
  `start_recording` answers `success: false`). A recording that received no packet is never
  written (`nothing recorded, <path> not written`), so an idle node, a failed bind or an empty
  segment leaves no 0-message file behind and an existing file named by `record:=` is not
  truncated. If saving fails at `stop_recording` (disk full, path became a directory), the
  answer is `success: false` and the recording **stays active** in memory: fix the cause and call
  `stop_recording` again, or discard with `{save: false}`. If saving fails at shutdown, the take
  is written to `movin_unsaved_<random>.pkl` in the system temp directory instead.
- **`stop_recording` blocks the node while it writes.** The service saves synchronously so the
  file exists when it answers; `pickle.dump` takes about 0.4 s per 10 minutes of stream on a
  local disk, during which the poll timer does not run and frames completing beyond the SDK's
  4-frame queue are dropped from `mocap_frame`/`/tf` (not from the recording; they show up in
  `frames_dropped`). For a large take call `split_recording` first (the big segment is saved on a
  background thread) and then `stop_recording`, which then only writes the small last segment.
- **Shutdown.** `Ctrl-C` (SIGINT) or SIGTERM saves the active recording synchronously before the
  node exits (exit code 0); further SIGINT/SIGTERM during that save are deferred and ignored
  afterwards (only SIGKILL cuts it short). `ros2 launch` escalates SIGINT to SIGTERM after 5 s and
  to SIGKILL 5 s later (`--sigterm-timeout`, `--sigkill-timeout`); a very large final segment can
  take longer than that to write, so keep `record_segment_sec` moderate or raise the timeouts.
- **Interoperability.** A recording plays back in ROS 2 (`mode:=replay`), in the SDK
  (`ReplayMocapReceiver`, `OscPlayer`, `peek_first_frame`, `examples/receive_mocap.py`) and in
  the Isaac plugin (`mocap_to_isaaclab.py --mode replay --recording <file>`), and recordings made
  by those tools replay here.
- **Warning: legacy files are unrelated.** The `.pkl` files of the former `motion_player` package
  (`{'fps', 'root_pos', 'root_rot', 'dof_pos'}` NumPy arrays of retargeted robot joints) and BVH
  files are **not** MOVIN SDK recordings. `mode:=replay` rejects them with
  `<path> is not a MOVIN SDK recording (legacy motion_player .pkl / BVH files are not supported)`.
  Record with this node, the SDK or the Isaac plugin instead.

## Replay details

Replay feeds the recorded OSC messages through the same frame assembly and publishing path as
live reception, with the original timing (`replay_realtime: true`, the SDK `ReplayMocapReceiver`)
and looping by default (`replay_loop`). `replay_realtime: false` (parameter, e.g.
`ros2 run movin_ros mocap_node --ros-args -p mode:=replay -p recording:=take1.pkl -p replay_realtime:=false`)
replays **as fast as the node can publish and losslessly**: the node pulls up to 8 frames per poll
from its own frame assembler instead of the SDK replay thread (which would keep only the last 4
frames of a burst), so converting a take to a bag or checking a file takes seconds, not the
recording's length. The recording is validated before start (read once) and the preset is taken
from its first frame; a file without a complete frame (for example a 0-message take) is fatal
(`contains no complete frame`, exit 1) instead of looping on nothing. With `replay_loop:=false` the
node logs `replay finished (<n> frames published)`, sets `status.replay_finished` and, with
`replay_exit_on_end:=true`, exits with code 0. The recording services answer `success: false`
("recording unavailable in replay mode"). The one-second sample used by the tests is
`movin_ros/test/data/live_movinman_v3_1s.pkl` (MOVINMan, `movinman_v3`, 55 bones).

## ros2 bag details

`bag:=true` starts `ros2 bag record -o <bag_dir> /movin/mocap_frame /movin/status /tf /tf_static`
next to the node (`ros2bag`/`rosbag2` are exec dependencies of `movin_ros`, so `rosdep` installs
them on a `ros-core`-only machine). Because `MocapFrame` carries the unmodified Studio values, a bag is a complete
record and can be retargeted later by playing it into `retarget_node`; consumers only need
`movin_msgs`. Bags are not readable by the SDK or the Isaac plugin, so use the `.pkl` recording
(both can run at the same time) when the data must leave ROS 2. Humble writes sqlite3 bags by
default (`ros-humble-rosbag2-storage-mcap` for MCAP), Jazzy writes MCAP.

Two messages from `ros2 bag record` are expected: on Jazzy it warns that the positional topic
list is deprecated (Humble has no `--topics` option, so the launch file keeps the portable form),
and after a terminal `Ctrl-C` `ros2 launch` reports it with `exit code 2` because the process
receives two SIGINTs (the terminal's and the one launch forwards; ros2cli reports the second one
as `KeyboardInterrupt`). The bag is complete in both cases: `Recording stopped` is logged first and
`ros2 bag info` lists every message.

## Retargeting details

The retargeter needs the SDK's `[retargeting]` extra: NumPy, SciPy, MuJoCo and Mink.
`rosdep` installs `python3-numpy` and `python3-scipy`; `mujoco` and `mink` have no rosdep key and
come from `pip`. On Ubuntu 24.04 (Jazzy) the system interpreter is externally managed (PEP 668),
so use a virtualenv that still sees the ROS packages, activated **before `colcon build`** and
before launching (the SDK itself comes from the workspace, do not `pip install` it again); the virtualenv recipe is in the README.

On Humble (Ubuntu 22.04) plain `pip3 install --user mujoco mink numpy scipy` also works; in
Docker use the `WITH_RETARGETING=1` image (`pip --break-system-packages` on Jazzy, plain pip on
Humble). Without the extra, `retarget_node` exits with code 1 and prints the SDK's
`MissingOptionalDependencyError` message (`pip install "movin_sdk_python[retargeting]"`); the
core node is unaffected.

Notes:

- `human_height` sizes the actor to the reference body (1.8 m) the SDK IK tables were tuned for.
  `auto` (default) lets the SDK `Retargeter` measure the actor from the T-pose leg length of the
  streamed bones, and again whenever Studio streams new proportions (a recalibrated actor). A
  number is the actor's standing height in meters and turns the measurement off; the body scale
  is then `1.8 / human_height`. `retarget_node` logs the applied scale when it is first known and
  whenever it changes. The node parameter is dynamically typed so that `auto`, a launch argument
  string (`human_height:=1.66`) and a number (`-p human_height:=1.66`, YAML) all work; anything
  else is fatal, and a height outside 0.5 to 3 m is warned about.
- **Changed meaning of a numeric `human_height`.** Earlier versions (default `1.75`) scaled the
  targets by `human_height / 1.8`, the inverse of today's `1.8 / human_height`. To reproduce a
  result retargeted with an old value `h`, pass `3.24 / h` (`1.8² / h`; about `1.85` for `1.75`).
- `robot_offset` (launch argument: X offset in meters; node parameter `robot_offset`:
  `[x, y, z]`) places the robot next to the skeleton.
- The Retargeter, like the SDK and Isaac examples, scales the hips position and does not align
  the feet with the ground unless `offset_to_ground` is set, so the robot drifts away from the
  skeleton by about 10 cm per meter of travel; this is SDK behaviour and is not corrected here.
- `source_preset:=auto` uses the `preset` field of the incoming `MocapFrame`; legacy 51-body
  streams therefore retarget as `movinman` without extra configuration.
- `urdf_file` replaces the default `g1_custom_collision_29dof.urdf` (plain URDF read with `cat`,
  no xacro). The bundled URDF's fixed `left/right_toe_joint` origins (`0.1 0 -0.02`) equal the toe
  bodies of the SDK's MuJoCo model, which are the IK targets for `LeftToeBase`/`RightToeBase`, so the
  toe frames in RViz coincide with what the Retargeter solved for.
- `mocap_frame` QoS must match on both ends: the launch argument `frame_qos_reliable` (default
  `true`) sets the publisher in `mocap_node` and the subscription in `retarget_node` together; a
  mismatch (for example a best-effort publisher from a params file and a standalone
  `retarget.launch.py` with the default) is reported by `retarget_node` as
  `a mocap_frame publisher is incompatible with this RELIABLE subscription` instead of silently
  waiting for frames forever.

## RViz details

The skeleton is a TF tree, so the default RViz view is a TF display with fixed frame `world`.
`publish_markers:=true` adds `/movin/skeleton_markers` (namespaces `movin/<actor_id>/joints`, red
spheres, and `movin/<actor_id>/bones`, orange cylinders, drawn in the bone frames; the display is
present but disabled in `mocap.rviz`). With `robot:=...` the launch file starts RViz with
`movin_g1_description/rviz/g1_mocap.rviz` instead, which shows the RobotModel from
`/movin/robot_description` together with the TF tree and the markers; standalone:

```bash
rviz2 -d $(ros2 pkg prefix movin_g1_description)/share/movin_g1_description/rviz/g1_mocap.rviz
```

`rviz2` is an `exec_depend` guarded by `MOVIN_HEADLESS`; on machines without a display run
`MOVIN_HEADLESS=1 rosdep install ...` (or `--skip-keys rviz2`) and leave `rviz:=false`.

## Coordinate conventions

**`MocapFrame` is lossless and unconverted.** Each `Bone` carries the values as streamed by MOVIN
Studio: `position` = joint offset in the parent's frame (meters), `rotation` = full local rotation
(rest rotation included), `rest_rotation` (informational), `scale`, all in Unity's left-handed,
Y-up frame (`convention == CONVENTION_UNITY_LH_YUP`). SDK quaternions are `(w, x, y, z)`; the
message uses the `geometry_msgs/Quaternion` fields `x, y, z, w` with the same values. Consumers
that want the SDK view of the data (`compute_forward_kinematics`, `Retargeter`,
`isaac_lab_utils`) rebuild the SDK bone dicts from the message one-to-one; nothing is mirrored.

**TF does the conversion.** The SDK forward kinematics are decomposed so that tf2 composes them:

1. Per bone, the Unity local transform becomes a proper right-handed parent-to-child edge by a
   reflection of the X axis, identical to the SDK's `unity_to_opengl_vec/quat`:
   `translation = (-p.x, p.y, p.z)`, `rotation(x, y, z, w) = normalize(q.x, -q.y, -q.z, q.w)`.
2. One static rotation per actor, `world -> movin/<actor_id>`, turns the still Y-up tree into a
   Z-up ROS world. `forward_mode` selects it:

| `forward_mode` | Anchor rotation | Actor forward (Unity +Z) | Actor left | Up | Matches |
|---|---|---|---|---|---|
| `coord_equivalent` | `R_x(+90°)`, xyzw `(0.7071, 0, 0, 0.7071)` | ROS **-Y** | +X | +Z | SDK `compute_forward_kinematics`, Isaac default |
| `isaac_world` (default) | `R_z(+90°)·R_x(+90°)`, xyzw `(0.5, 0.5, 0.5, 0.5)` | ROS **+X** | +Y | +Z | REP-103; SDK `Q_ISAAC_WORLD_FORWARD` |

Composing the edges under the anchor reproduces `movin_sdk_python.utils.fk_utils.compute_forward_kinematics`
(and, for `isaac_world`, its `Q_ISAAC_WORLD_FORWARD` variant) to floating-point precision; the
tests check this against the SDK on the live fixture. Parity requires bones streamed
parents-first (`parent_index < bone_index`) with unique names, which MOVIN Studio does; the node
warns once if a stream violates it. Bones whose parent is absent (`parent_index < 0`, e.g.
`RootBone`) hang directly under the anchor; the root is determined by `parent_index` only, never
by name. Whether the actor really faces ROS +X depends on the root yaw of the MOVIN Studio
calibration; the mapping above is axis-level.

**Bone frame orientation.** Bone frames follow the SDK `fk_utils` orientation convention: the
world rotation of a bone is the anchor rotation composed with the mirrored local rotations
(`C ⊗ q_local...`), not a world-aligned rest frame. In the T-pose the axes of a bone frame (for
example `movin/<actor>/Hips`) therefore appear rotated by the anchor rotation with respect to
`world` in RViz. This is expected, not a bug: it is the frame convention the SDK `Retargeter`'s
IK offsets are calibrated for, and it keeps the TF tree equal to the SDK forward kinematics. The
world-aligned rest frames of `isaac_lab_utils` (`C q C⁻¹`) are deliberately not used.

**Robot.** The Retargeter's root pose (`qpos[:3]`, `qpos[3:7]`) lives in the `coord_equivalent`
world. For `forward_mode:=isaac_world` the node applies the same +90 degree yaw to the robot root
(`world -> pelvis`) so the robot and the skeleton face the same way, then adds `robot_offset`.

**Frames and stamps.** `world_frame` (`world`) → `<tf_prefix>/<actor_id>` (`movin/MOVINMan`) →
`<tf_prefix>/<actor_id>/<bone_name>` (`movin/MOVINMan/Hips`, ...). `MocapFrame.actor_frame` names
the anchor. `header.stamp` is the node clock at publish time; MOVIN Studio's own clock is kept as
the string `source_timestamp` and its counter as `frame_idx`.

## Launch arguments and parameters

`ros2 launch movin_ros mocap.launch.py` (node `mocap_node` in namespace `/movin`, parameters from
`movin_ros/config/mocap_node.yaml` overridden by the typed launch arguments):

| Launch argument | Default | Description |
|---|---|---|
| `mode` | `live` | `live` (UDP) or `replay` (recording file) |
| `host` | `0.0.0.0` | Local bind address (not the Studio address) |
| `port` | `11235` | UDP port; `0` = ephemeral (logged) |
| `recording` | `''` | Input `.pkl` for `mode:=replay` |
| `replay_loop` | `true` | Restart playback at the end |
| `replay_exit_on_end` | `false` | Exit 0 when the (non-looping) replay finishes |
| `record` | `''` | `''` off, `auto` auto-named file in `record_dir`, or an explicit path |
| `record_dir` | `~/movin_recordings` | Directory for auto-named recordings |
| `record_segment_sec` | `600.0` | Rotate the recording every N seconds; `0` = single file |
| `preset` | `auto` | `auto`, `movinman`, `movinman_v3` |
| `forward_mode` | `isaac_world` | `coord_equivalent` or `isaac_world` (see conventions) |
| `world_frame` | `world` | TF parent of the actor anchors |
| `tf_prefix` | `movin` | Prefix of actor and bone frames |
| `publish_every_frame` | `true` | Drain all assembled frames per poll (`false`: newest only) |
| `poll_rate_hz` | `120.0` | Receiver polling rate |
| `publish_markers` | `false` | Publish `skeleton_markers` (requires `publish_tf`) |
| `frame_qos_reliable` | `true` | `mocap_frame` QoS of `mocap_node` and `retarget_node`: reliable (`true`) or best effort |
| `robot` | `none` | `none`, `unitree_g1`, `unitree_g1_with_hands` (starts `movin_retarget`) |
| `human_height` | `auto` | Retargeting body size: `auto` (measured from the stream) or the actor's height in meters |
| `robot_offset` | `0.0` | Robot X offset in meters (retargeting) |
| `urdf_file` | G1 29-DoF URDF | URDF for `robot_state_publisher` (retargeting) |
| `rviz` | `false` | Start RViz: `movin_ros/rviz/mocap.rviz`, or with `robot:=...` `movin_g1_description/rviz/g1_mocap.rviz` |
| `bag` / `bag_dir` | `false` / `''` | Record `mocap_frame`, `status`, `/tf`, `/tf_static` with `ros2 bag` |
| `max_frames` | `0` | Exit 0 after N published frames (testing) |
| `namespace` | `movin` | Namespace of all nodes |
| `params_file` | `config/mocap_node.yaml` | Parameter file loaded before the launch arguments |
| `log_level` | `info` | `--log-level` of `mocap_node` |

`mocap_node` parameters (`ros2 param list /movin/mocap_node`; those not exposed as launch
arguments are set in `params_file` or with `ros2 run ... -p name:=value`). Every parameter is read
once at startup and declared **read-only**, so `ros2 param set` is rejected with a clear error
rather than accepted without effect; restart the node to change a value. Floating-point
parameters accept integer literals (`-p poll_rate_hz:=60`, `skeleton_offset: [0, 0, 0]`):

| Parameter | Type | Default | Description |
|---|---|---|---|
| `mode` | string | `live` | `live` or `replay` |
| `host` | string | `0.0.0.0` | Local bind address |
| `port` | int | `11235` | UDP port, `0` = ephemeral |
| `recording` | string | `''` | Replay input `.pkl` |
| `replay_loop` | bool | `true` | Loop the replay |
| `replay_realtime` | bool | `true` | Reproduce the recorded timing (`false`: as fast as the node publishes, lossless) |
| `replay_exit_on_end` | bool | `false` | Exit when a non-looping replay ends |
| `record` | string | `''` | `''`, `auto` or path |
| `record_dir` | string | `~/movin_recordings` | Auto-name directory |
| `record_segment_sec` | double | `600.0` | Segment length, `0` = no rotation |
| `record_stream_type` | string | `movin` | `stream_type` written into the recording |
| `preset` | string | `auto` | Skeleton preset override |
| `poll_rate_hz` | double | `120.0` | Poll timer rate |
| `publish_every_frame` | bool | `true` | Drain vs. latest-only delivery |
| `actor_filter` | string | `''` | Publish only this actor |
| `forward_mode` | string | `isaac_world` | Anchor rotation |
| `world_frame` | string | `world` | Parent of the anchors |
| `tf_prefix` | string | `movin` | Frame prefix |
| `skeleton_offset` | double[] | `[0, 0, 0]` | Translation of the actor anchors in `world_frame` |
| `publish_tf` | bool | `true` | Publish `/tf` and `/tf_static` |
| `publish_frame` | bool | `true` | Publish `mocap_frame` |
| `publish_markers` | bool | `false` | Publish `skeleton_markers` |
| `frame_qos_reliable` | bool | `true` | Reliable (`true`) or best-effort QoS for `mocap_frame` |
| `stale_timeout_sec` | double | `1.0` | `status.receiving` = false after this long without frames |
| `retry_bind_sec` | double | `0.0` | `0` = bind failure (or a dead receive thread) is fatal (exit 1); otherwise retry period |
| `max_frames` | int | `0` | Exit after N frames (`0` = never) |

`retarget_node` parameters (`ros2 launch movin_retarget retarget.launch.py`; also read-only):
`robot` (`unitree_g1`), `human_height` (`auto` or meters), `source_preset` (`auto`), `forward_mode`
(`isaac_world`), `world_frame` (`world`), `robot_root_frame` (`pelvis`), `robot_offset`
(`[0, 0, 0]`), `offset_to_ground` (`false`), `max_rate_hz` (`60.0`), `actor` (`''` = first actor
seen), `frame_qos_reliable` (`true`, must equal `mocap_node`'s).

## Topics, services and TF

| Topic | Type | Publisher | Notes |
|---|---|---|---|
| `/movin/mocap_frame` | `movin_msgs/MocapFrame` | `mocap_node` | Every assembled frame; reliable QoS by default |
| `/movin/status` | `movin_msgs/MocapStatus` | `mocap_node` | 1 Hz: mode, receiving, bind address, rate (0 while not receiving), error, drops, actors, preset, recording state, replay end |
| `/movin/skeleton_markers` | `visualization_msgs/MarkerArray` | `mocap_node` | Only with `publish_markers` |
| `/tf` | `tf2_msgs/TFMessage` | `mocap_node` | One message per frame with all bone edges |
| `/tf_static` | `tf2_msgs/TFMessage` | `mocap_node` | `world -> movin/<actor_id>` anchors |
| `/movin/joint_states` | `sensor_msgs/JointState` | `retarget_node` | Optional retargeting |
| `/movin/robot_description` | `std_msgs/String` | `robot_state_publisher` | Optional retargeting |

| Service | Type | Notes |
|---|---|---|
| `/movin/start_recording` | `movin_msgs/srv/StartRecording` | `path` (`''` = auto), `stream_type` (`''` = parameter) → `success`, `message`, `path` |
| `/movin/stop_recording` | `movin_msgs/srv/StopRecording` | `save` → `success`, `message`, `path`, `num_messages`, `duration_sec`; `success: false` with the OS error if the save failed (recording stays active) |
| `/movin/split_recording` | `std_srvs/srv/Trigger` | Close the current segment and start the next one |

```text
movin_msgs/MocapFrame            movin_msgs/Bone
  std_msgs/Header header           int32 bone_index
  string actor                     int32 parent_index          # -1 for the root
  string actor_frame               string name
  int32 frame_idx                  geometry_msgs/Vector3 position
  string source_timestamp          geometry_msgs/Quaternion rest_rotation
  string preset                    geometry_msgs/Quaternion rotation
  uint8 convention                 geometry_msgs/Vector3 scale
  Bone[] bones
```

TF frames: `world` → `movin/<actor_id>` (static) → `movin/<actor_id>/<bone_name>` for every
bone, plus `world` → `pelvis` and the robot links when retargeting.

## Testing

Without ROS (this repository only needs Python 3.10+; `movin_ros/test/conftest.py` puts the
submodule on `sys.path`):

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider movin_ros/test -k "not launch"
python3 -m pytest movin_ros/test movin_retarget/test -k "not launch"     # both packages in one session also works
```

The forward-kinematics parity tests compare against the SDK and are skipped when NumPy is not
installed; run them from an environment that has it. `test_no_numpy_import.py` guarantees the
core modules never import NumPy, SciPy or MuJoCo. (`PYTHONDONTWRITEBYTECODE` / `no:cacheprovider`
keep `__pycache__` and `.pytest_cache` out of the source tree; both are ignored by git anyway.)

With ROS 2 (natively or through Docker; includes `ament_flake8`, `ament_pep257` and the
`launch_testing` suites: replay of the fixture, live UDP on port 21235, recording services and
save-on-shutdown):

```bash
colcon test && colcon test-result --verbose
./docker/dev.sh jazzy colcon test && ./docker/dev.sh jazzy colcon test-result --verbose
./docker/ci.sh humble --symlink-install          # full CI run in a throw-away container
```

The SDK's own dependency-free test suite guards submodule bumps:

```bash
cd movin_sdk_python_vendor/MOVIN-SDK-Python && python3 -m unittest discover -s tests -v
```

Manual acceptance with MOVIN Studio: set the Studio destination to this computer, check
`ros2 topic hz /movin/mocap_frame` against the Studio frame rate and `frames_dropped == 0` over a
minute, record one minute with `record:=auto`, confirm `recording_info` reports about 60 s, replay
that file with `rviz:=true`, and check `view_frames` shows `world -> movin/<actor> -> ...`.
