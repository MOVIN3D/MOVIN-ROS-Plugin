#!/bin/bash
# Entrypoint of the movin-ros-dev image.
#
# Sources /opt/ros/<distro>/setup.bash and, if present, the workspace install space
# (/ws/.cache/install for dev.sh/compose, /ws/install for ci.sh), makes sure $HOME exists when the
# container runs as an arbitrary --user, then execs the command.
set -e

if [ -n "${HOME:-}" ] && [ ! -d "${HOME}" ]; then
    mkdir -p "${HOME}" 2>/dev/null || true
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

for setup in /ws/.cache/install/setup.bash /ws/install/setup.bash; do
    if [ -f "${setup}" ]; then
        # shellcheck disable=SC1090
        source "${setup}"
        break
    fi
done

exec "$@"
