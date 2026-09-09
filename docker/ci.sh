#!/usr/bin/env bash
# Continuous-integration run of this repository for one ROS 2 distribution, inside Docker.
#
#   docker/ci.sh <jazzy|humble> [colcon build arguments...]
#
#   docker/ci.sh jazzy
#   docker/ci.sh humble --symlink-install
#
# Builds movin-ros-dev:<distro> from docker/Dockerfile, mounts the repository read-only and copies it
# into a fresh /ws/src inside the container (nothing is written back to the host), then runs
#   MOVIN_HEADLESS=1 rosdep install --from-paths src --ignore-src --skip-keys rviz2
#   colcon build [arguments]        (the GitHub workflow passes "" and "--symlink-install")
#   colcon test
#   colcon test-result --verbose
#   python3 -m unittest discover -s tests   (MOVIN SDK unit tests, in the submodule)
#
# Environment:
#   MOVIN_IMAGE        image tag to build and run (default: movin-ros-dev:<distro>)
#   MOVIN_CI_PACKAGES  space-separated package names for `--packages-select` (default: all packages)
#   MOVIN_CI_NO_BUILD  set to 1 to skip `docker build` and use the existing image
set -euo pipefail

# ---------------------------------------------------------------------------------------------------
# Inside the container (re-entered by the docker run below with MOVIN_CI_INSIDE=1).
# ---------------------------------------------------------------------------------------------------
if [ "${MOVIN_CI_INSIDE:-0}" = "1" ]; then
    set +u   # ROS setup.bash references unset variables
    set -x
    export MOVIN_HEADLESS=1
    export DEBIAN_FRONTEND=noninteractive
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"

    rm -rf /ws/src/MOVIN-ROS-Plugin /ws/build /ws/install /ws/log
    mkdir -p /ws/src
    cp -a /src_ro/MOVIN-ROS-Plugin /ws/src/MOVIN-ROS-Plugin
    rm -rf /ws/src/MOVIN-ROS-Plugin/.git
    cd /ws

    apt-get update
    rosdep update --rosdistro "${ROS_DISTRO}"
    rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" --skip-keys rviz2

    select_args=()
    if [ -n "${MOVIN_CI_PACKAGES:-}" ]; then
        # shellcheck disable=SC2206
        select_args=(--packages-select ${MOVIN_CI_PACKAGES})
    fi
    # shellcheck disable=SC2086
    colcon build --base-paths src --event-handlers console_cohesion+ "${select_args[@]}" ${MOVIN_CI_BUILD_ARGS:-}
    # shellcheck disable=SC1091
    source /ws/install/setup.bash
    colcon test --base-paths src --event-handlers console_cohesion+ "${select_args[@]}"
    colcon test-result --verbose

    cd /ws/src/MOVIN-ROS-Plugin/movin_sdk_python_vendor/MOVIN-SDK-Python
    python3 -m unittest discover -s tests -v
    exit 0
fi

# ---------------------------------------------------------------------------------------------------
# On the host.
# ---------------------------------------------------------------------------------------------------
usage() {
    sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

distro="${1:-}"
case "${distro}" in
    jazzy|humble) shift ;;
    -h|--help|'') usage; exit 2 ;;
    *) echo "ci.sh: unknown distro '${distro}' (expected jazzy or humble)" >&2; usage; exit 2 ;;
esac

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${MOVIN_IMAGE:-movin-ros-dev:${distro}}"

if [ "${MOVIN_CI_NO_BUILD:-0}" != "1" ]; then
    docker build -f "${repo}/docker/Dockerfile" --build-arg "ROS_DISTRO=${distro}" -t "${image}" "${repo}"
fi

exec docker run --rm --network host \
    -e MOVIN_CI_INSIDE=1 \
    -e "MOVIN_CI_BUILD_ARGS=$*" \
    -e MOVIN_CI_PACKAGES \
    -v "${repo}:/src_ro/MOVIN-ROS-Plugin:ro" \
    "${image}" bash /src_ro/MOVIN-ROS-Plugin/docker/ci.sh
