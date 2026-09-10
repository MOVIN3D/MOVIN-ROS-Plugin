# MOVIN ROS Plugin

Receive and record real-time motion capture from MOVIN Studio in ROS 2.

`mocap_node` connects to [MOVIN Studio](https://www.movin3d.com) over OSC/UDP with the
[MOVIN SDK Python](https://github.com/MOVIN3D/MOVIN-SDK-Python) (git submodule, installed by colcon,
no `pip`), publishes every assembled frame as a lossless `movin_msgs/MocapFrame` plus a TF tree per
actor, and records the raw OSC stream to the SDK's `.pkl` format, which replays in ROS 2, in the SDK
and in the [MOVIN Isaac Plugin](https://github.com/MOVIN3D/MOVIN-Isaac-Plugin). The core has no
third-party Python dependencies and runs on ROS 2 Humble and Jazzy. Unitree G1 retargeting and RViz
are optional.

```text
MOVIN Studio ──OSC/UDP──> mocap_node ──> /movin/mocap_frame   (movin_msgs/MocapFrame, every frame)
                              │    └───> /tf  world -> movin/<actor> -> movin/<actor>/<bone>
                              └──> raw OSC recording (.pkl) ──> mode:=replay

optional:  /movin/mocap_frame ──> retarget_node ──> /movin/joint_states, world -> pelvis   (robot:=unitree_g1)
           /tf, markers, robot model ──> RViz                                            (rviz:=true)
```

| Package | Purpose | Default |
|---|---|:---:|
| `movin_msgs` | `MocapFrame`, `Bone`, `MocapStatus`; `StartRecording`, `StopRecording` services | yes |
| `movin_sdk_python_vendor` | installs the SDK submodule into the workspace | yes |
| `movin_ros` | `mocap_node` (receive, record, replay), `recording_info`, launch and RViz files | yes |
| `movin_retarget` | `retarget_node`: MOVIN skeleton to Unitree G1 joint states | optional |
| `movin_g1_description` | Unitree G1 URDF, meshes, `robot_state_publisher` launch, RViz config | optional |

Detailed behaviour and the full parameter list: [docs/REFERENCE.md](docs/REFERENCE.md).

## Setup

ROS 2 Jazzy (Ubuntu 24.04, primary) or Humble (22.04); both are built and tested in CI.

```bash
mkdir -p ~/movin_ws/src && cd ~/movin_ws/src
git clone --recurse-submodules https://github.com/MOVIN3D/MOVIN-ROS-Plugin.git
cd ~/movin_ws
source /opt/ros/jazzy/setup.bash                       # or humble
rosdep install --from-paths src --ignore-src -y        # headless: MOVIN_HEADLESS=1 rosdep install ... (skips rviz2)
colcon build --symlink-install
source install/setup.bash
```

Docker (no ROS on the host): `docker/dev.sh` builds `movin-ros-dev:<distro>` on first use and runs
commands in it with the host network, as your user, keeping build products out of the repository.

```bash
./docker/dev.sh jazzy colcon build
./docker/dev.sh jazzy ros2 launch movin_ros mocap.launch.py record:=auto
./docker/ci.sh humble --symlink-install                # the CI run, locally
```

## Live receive

```bash
ros2 launch movin_ros mocap.launch.py                                   # listen on 0.0.0.0:11235
ros2 launch movin_ros mocap.launch.py host:=192.168.0.25 port:=12000    # one interface, another port
ros2 launch movin_ros mocap.launch.py rviz:=true                        # with RViz
```

In MOVIN Studio enable OSC output and set the destination:

- Studio on the **same computer**: `127.0.0.1:11235` (on Linux Studio's default `0.0.0.0:11235`
  also works; on Windows use `127.0.0.1`).
- Studio on **another computer**: the receiver's real IP and port, e.g. `192.168.0.25:11235`
  (`hostname -I`). `0.0.0.0` never reaches another machine.

`host` is the receiver's local bind address, not the Studio address. Check the stream:

```bash
ros2 topic hz /movin/mocap_frame       # the Studio frame rate
ros2 topic echo /movin/status          # receiving, receive_rate_hz, frames_dropped, actors, preset, recording
ros2 run tf2_tools view_frames         # world -> movin/<actor> -> movin/<actor>/Hips -> ...
```

The skeleton preset (`movinman`, `movinman_v3`) is detected from the first frame of each actor.
All assembled frames are published (`publish_every_frame`), and `frame_idx` gaps are counted in
`status.frames_dropped`.

## Record

The raw OSC stream is recorded on the receive thread, before frame assembly, so a recording is
complete regardless of ROS load. The file is the SDK's `.pkl` recording format.

```bash
ros2 launch movin_ros mocap.launch.py record:=auto                          # ~/movin_recordings/movin_<timestamp>_000.pkl
ros2 launch movin_ros mocap.launch.py record:=/data/take1.pkl record_segment_sec:=300

ros2 service call /movin/start_recording movin_msgs/srv/StartRecording "{path: '', stream_type: ''}"
ros2 service call /movin/split_recording std_srvs/srv/Trigger
ros2 service call /movin/stop_recording  movin_msgs/srv/StopRecording  "{save: true}"

ros2 run movin_ros recording_info ~/movin_recordings/movin_20260909_161500_000.pkl
```

- Recordings are held in RAM until saved (about 1 GB per 10 minutes at 60 Hz), so they rotate into
  numbered segments every `record_segment_sec` (default 600 s) without losing a packet.
- `Ctrl-C` saves the active recording before the node exits; an empty recording is never written.
- The `.pkl` files of the former `motion_player` package and BVH files are not SDK recordings and
  are rejected by `mode:=replay`.

## Replay

```bash
ros2 launch movin_ros mocap.launch.py mode:=replay recording:=take1.pkl rviz:=true
ros2 launch movin_ros mocap.launch.py mode:=replay recording:=take1.pkl replay_loop:=false replay_exit_on_end:=true
# no Studio at hand: the one-second sample shipped with the package
ros2 launch movin_ros mocap.launch.py mode:=replay rviz:=true \
    recording:=$(ros2 pkg prefix movin_ros)/share/movin_ros/test/data/live_movinman_v3_1s.pkl
```

Replay goes through the same assembly and publishing path as live reception with the original
timing; `-p replay_realtime:=false` replays as fast as possible without dropping frames.

## ros2 bag

```bash
ros2 launch movin_ros mocap.launch.py bag:=true bag_dir:=./bags/take1    # records mocap_frame, status, /tf, /tf_static
ros2 bag play ./bags/take1
```

A bag holds the unmodified Studio values and can be retargeted later; use the `.pkl` recording when
the data must be read by the SDK or the Isaac plugin (both can run at once).

## Optional: robot retargeting

`retarget_node` subscribes to `/movin/mocap_frame` (live, replay or bag), runs the SDK `Retargeter`
and publishes `/movin/joint_states` and `world -> pelvis`; `movin_g1_description` adds the G1 URDF
and `robot_state_publisher`. It needs the SDK `[retargeting]` extra (NumPy, SciPy, MuJoCo, Mink);
`mujoco` and `mink` come from `pip`, which on Ubuntu 24.04 means a virtualenv activated before
`colcon build`:

```bash
python3 -m venv --system-site-packages ~/movin_venv && source ~/movin_venv/bin/activate
pip install mujoco mink numpy scipy
cd ~/movin_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch movin_ros mocap.launch.py robot:=unitree_g1 rviz:=true
ros2 launch movin_ros mocap.launch.py robot:=unitree_g1 human_height:=1.80 robot_offset:=1.0
```

`robot` is `unitree_g1` (29 joints) or `unitree_g1_with_hands` (43). In Docker use the
`WITH_RETARGETING=1` image. Without the extra `retarget_node` exits with a clear message and the
core node is unaffected.

## Optional: RViz

```bash
ros2 launch movin_ros mocap.launch.py rviz:=true                         # TF skeleton
ros2 launch movin_ros mocap.launch.py rviz:=true publish_markers:=true   # + joint spheres and bone cylinders
ros2 launch movin_ros mocap.launch.py rviz:=true robot:=unitree_g1      # + robot model
```

## Coordinate conventions

`MocapFrame` carries the Studio values unchanged (Unity left-handed, Y-up; local offsets and
rotations per bone). TF does the conversion: each bone edge is the X-mirrored local transform and one
static anchor rotation per actor, `world -> movin/<actor>`, makes the tree Z-up. Composing the
edges reproduces the SDK forward kinematics exactly.

| `forward_mode` | Anchor (xyzw) | Actor forward | Matches |
|---|---|---|---|
| `isaac_world` (default) | `(0.5, 0.5, 0.5, 0.5)` | ROS +X (REP-103) | SDK `Q_ISAAC_WORLD_FORWARD` |
| `coord_equivalent` | `(0.7071, 0, 0, 0.7071)` | ROS -Y | SDK `compute_forward_kinematics`, Isaac default |

The robot root gets the same rotation, so skeleton and robot always face the same way. Bone frames
follow the SDK orientation convention, so their axes appear rotated by the anchor in the T-pose;
this is expected. Frames: `world` -> `movin/<actor>` -> `movin/<actor>/<bone>` (and `pelvis` plus
the robot links when retargeting).

## Main launch arguments

`ros2 launch movin_ros mocap.launch.py` starts `mocap_node` in namespace `/movin`. All arguments
are node parameters; the full list with types is in [docs/REFERENCE.md](docs/REFERENCE.md).

| Argument | Default | Description |
|---|---|---|
| `mode` | `live` | `live` or `replay` |
| `host` / `port` | `0.0.0.0` / `11235` | local bind address and UDP port |
| `recording` | `''` | input `.pkl` for `mode:=replay` |
| `replay_loop` / `replay_exit_on_end` | `true` / `false` | loop, or exit 0 when a single pass ends |
| `record` | `''` | `''` off, `auto`, or an explicit path |
| `record_dir` / `record_segment_sec` | `~/movin_recordings` / `600.0` | auto-name directory, segment length (`0` = one file) |
| `preset` | `auto` | `auto`, `movinman`, `movinman_v3` |
| `forward_mode` | `isaac_world` | `isaac_world` or `coord_equivalent` |
| `world_frame` / `tf_prefix` | `world` / `movin` | TF names |
| `publish_markers` | `false` | skeleton markers for RViz |
| `robot` / `human_height` / `robot_offset` | `none` / `1.75` / `0.0` | retargeting |
| `rviz` | `false` | start RViz |
| `bag` / `bag_dir` | `false` / `''` | run `ros2 bag record` |

## Topics and services

| Name | Type | Notes |
|---|---|---|
| `/movin/mocap_frame` | `movin_msgs/MocapFrame` | every assembled frame |
| `/movin/status` | `movin_msgs/MocapStatus` | 1 Hz health, rates, drops, recording state |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | bone edges per frame; actor anchors |
| `/movin/skeleton_markers` | `visualization_msgs/MarkerArray` | with `publish_markers` |
| `/movin/joint_states` | `sensor_msgs/JointState` | with `robot:=...` |
| `/movin/start_recording` | `movin_msgs/srv/StartRecording` | `path` (`''` = auto) |
| `/movin/stop_recording` | `movin_msgs/srv/StopRecording` | `save`; returns path, messages, duration |
| `/movin/split_recording` | `std_srvs/srv/Trigger` | close the segment, start the next |

## Testing

```bash
python3 -m pytest movin_ros/test -k "not launch"     # ROS-free unit tests (no ROS needed)
colcon test && colcon test-result --verbose          # + lint and launch tests (ROS 2)
./docker/ci.sh jazzy                                 # the CI run in a container
```

## Project structure

```text
movin_msgs/               messages and services
movin_sdk_python_vendor/  MOVIN-SDK-Python submodule + colcon install
movin_ros/                mocap_node, recording_info, launch/, config/, rviz/, test/
movin_retarget/           retarget_node, launch/retarget.launch.py
movin_g1_description/     Unitree G1 urdf/, meshes/, launch/display.launch.py, rviz/
docker/                   Dockerfile, compose.yaml, dev.sh, ci.sh
docs/                     REFERENCE.md
```

## License

Apache License 2.0, see [LICENSE](LICENSE) and [NOTICE](NOTICE). Third-party components (MOVIN SDK
Python, Unitree G1 assets, GMR) are listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
