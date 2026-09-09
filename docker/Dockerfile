# Development and CI image for MOVIN-ROS-Plugin.
#
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy  -t movin-ros-dev:jazzy  .
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble -t movin-ros-dev:humble .
#   docker build -f docker/Dockerfile --build-arg WITH_RETARGETING=1 -t movin-ros-dev:jazzy-retarget .
#   docker build -f docker/Dockerfile --build-arg WITH_RVIZ=1        -t movin-ros-dev:jazzy-rviz .
#
# The repository is not copied into the image (see Dockerfile.dockerignore). docker/dev.sh and
# docker/compose.yaml bind-mount it at /ws/src/MOVIN-ROS-Plugin; docker/ci.sh copies it in at run time.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

# ARGs declared before FROM are not visible in the build stage unless redeclared.
ARG ROS_DISTRO
ARG WITH_RVIZ=0
ARG WITH_RETARGETING=0

LABEL org.opencontainers.image.source="https://github.com/MOVIN3D/MOVIN-ROS-Plugin" \
      org.opencontainers.image.description="MOVIN Studio motion capture for ROS 2 (${ROS_DISTRO})"

# Tooling on top of ros-base: pip/venv (optional [retargeting] extra), pytest, launch_testing_ros
# (launch tests), tf2_tools (view_frames). WITH_RVIZ=1 adds rviz2 plus a virtual X server for
# headless RViz smoke tests.
RUN set -eux; \
    export DEBIAN_FRONTEND=noninteractive; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-pytest \
        python3-venv \
        "ros-${ROS_DISTRO}-launch-testing-ros" \
        "ros-${ROS_DISTRO}-tf2-tools"; \
    if [ "${WITH_RVIZ}" = "1" ]; then \
        apt-get install -y --no-install-recommends \
            "ros-${ROS_DISTRO}-rviz2" \
            xvfb \
            x11-utils; \
    fi; \
    rm -rf /var/lib/apt/lists/*

# WITH_RETARGETING=1: dependencies of movin_sdk_python[retargeting] for movin_retarget. mujoco and
# mink have no rosdep key, so they come from pip. Ubuntu 24.04 (Jazzy) marks the system interpreter
# as externally managed (PEP 668) and needs --break-system-packages; Humble's pip 22 has no such
# flag and installs directly (plain pip3). Current mujoco/scipy wheels require a newer numpy than
# the Debian-packaged one that ros-base pulls in, and pip cannot uninstall a Debian package
# ("RECORD file not found"), so numpy is first installed with --ignore-installed into
# /usr/local, which shadows /usr/lib/python3/dist-packages.
RUN set -eux; \
    if [ "${WITH_RETARGETING}" = "1" ]; then \
        PIP_FLAGS=""; \
        if python3 -m pip install --help | grep -q -- --break-system-packages; then \
            PIP_FLAGS="--break-system-packages"; \
        fi; \
        python3 -m pip install --no-cache-dir ${PIP_FLAGS} --ignore-installed numpy; \
        python3 -m pip install --no-cache-dir ${PIP_FLAGS} mujoco mink numpy scipy; \
        python3 -c "import mujoco, mink, numpy, scipy; print('mujoco', mujoco.__version__, 'numpy', numpy.__version__)"; \
    fi

# colcon defaults used by dev.sh / compose (opt-in through COLCON_DEFAULTS_FILE) and an entrypoint
# that sources ROS 2 and the workspace install space if one exists.
COPY docker/colcon_defaults.yaml /etc/movin_ros/colcon_defaults.yaml
COPY docker/entrypoint.sh /movin_entrypoint.sh
# /ws (and the bind-mount parents dev.sh/compose use) are world-writable so that a non-root
# --user can still build there; /tmp/home is the HOME of such a user (ros2 needs ~/.ros).
RUN chmod 0755 /movin_entrypoint.sh \
    && mkdir -p /ws/src /ws/.cache /tmp/home \
    && chmod 1777 /ws /ws/src /ws/.cache /tmp/home

WORKDIR /ws
ENTRYPOINT ["/movin_entrypoint.sh"]
CMD ["bash"]
