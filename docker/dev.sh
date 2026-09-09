#!/usr/bin/env bash
# Run a command in the movin-ros-dev image with this repository bind-mounted as the workspace source.
#
#   docker/dev.sh <jazzy|humble> <command...>
#
#   docker/dev.sh jazzy colcon build
#   docker/dev.sh jazzy colcon test && docker/dev.sh jazzy colcon test-result --verbose
#   docker/dev.sh jazzy ros2 launch movin_ros mocap.launch.py record:=auto
#   docker/dev.sh humble                      # interactive shell
#
# Builds the image movin-ros-dev:<distro> from docker/Dockerfile when it does not exist yet, then runs
#   docker run --rm -i --network host --user $(id -u):$(id -g) -e HOME=/tmp/home
#       -v <repo>:/ws/src/MOVIN-ROS-Plugin -v ${MOVIN_WS_CACHE:-$HOME/.cache/movin_ros_ws}/<distro>:/ws/.cache ...
# The container runs as the invoking user, so nothing in the repository becomes root-owned, and the
# repository stays free of build products: colcon build/, install/ and log/ go to /ws/.cache (a
# per-distro host directory) through COLCON_DEFAULTS_FILE (/etc/movin_ros/colcon_defaults.yaml,
# equivalent to `colcon build --build-base /ws/.cache/build --install-base /ws/.cache/install`) and
# COLCON_LOG_PATH. The entrypoint sources /opt/ros/<distro>/setup.bash and, when present,
# /ws/.cache/install/setup.bash, so a `colcon build` is picked up by the next invocation.
#
# Environment:
#   MOVIN_WS_CACHE    host directory for build products (default: $HOME/.cache/movin_ros_ws)
#   MOVIN_RECORD_DIR  host directory mounted as ~/movin_recordings in the container, the default
#                     record_dir of mocap_node (default: $HOME/movin_recordings)
#   MOVIN_IMAGE       image to run instead of movin-ros-dev:<distro>, e.g. movin-ros-dev:jazzy-retarget
#   MOVIN_BUILD_ARGS  extra `docker build` arguments when the image is built, e.g. "--build-arg WITH_RVIZ=1"
#   MOVIN_DOCKER_ARGS extra `docker run` arguments, e.g. "--gpus all"
#   MOVIN_HEADLESS, ROS_DOMAIN_ID, RMW_IMPLEMENTATION, ROS_LOCALHOST_ONLY, DISPLAY are passed through.
set -euo pipefail

usage() {
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

distro="${1:-}"
case "${distro}" in
    jazzy|humble) shift ;;
    -h|--help|'') usage; exit 2 ;;
    *) echo "dev.sh: unknown distro '${distro}' (expected jazzy or humble)" >&2; usage; exit 2 ;;
esac
if [ "$#" -eq 0 ]; then
    set -- bash
fi

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${MOVIN_IMAGE:-movin-ros-dev:${distro}}"
cache="${MOVIN_WS_CACHE:-${HOME}/.cache/movin_ros_ws}/${distro}"
record_dir="${MOVIN_RECORD_DIR:-${HOME}/movin_recordings}"

if ! docker image inspect "${image}" >/dev/null 2>&1; then
    echo "dev.sh: building ${image} from docker/Dockerfile (ROS_DISTRO=${distro})" >&2
    # shellcheck disable=SC2086
    docker build -f "${repo}/docker/Dockerfile" --build-arg "ROS_DISTRO=${distro}" \
        ${MOVIN_BUILD_ARGS:-} -t "${image}" "${repo}"
fi

# Create the host directories first so they are owned by the user, not by the docker daemon.
mkdir -p "${cache}" "${record_dir}"

run_args=(--rm -i --network host --ipc host --user "$(id -u):$(id -g)")
if [ -t 0 ] && [ -t 1 ]; then
    run_args+=(-t)
fi
run_args+=(
    -e HOME=/tmp/home
    -e COLCON_DEFAULTS_FILE=/etc/movin_ros/colcon_defaults.yaml
    -e COLCON_LOG_PATH=/ws/.cache/log
    -e MOVIN_HEADLESS -e ROS_DOMAIN_ID -e RMW_IMPLEMENTATION -e ROS_LOCALHOST_ONLY
    -v "${repo}:/ws/src/MOVIN-ROS-Plugin"
    -v "${cache}:/ws/.cache"
    -v "${record_dir}:/tmp/home/movin_recordings"
    -w /ws
)
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    run_args+=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix)
fi
# shellcheck disable=SC2086
exec docker run "${run_args[@]}" ${MOVIN_DOCKER_ARGS:-} "${image}" "$@"
