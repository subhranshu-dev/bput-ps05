# BPUT Hackathon 2026 PS05 — version-locked simulation/autonomy environment.
#
# Host OS (Ubuntu 26.04 / Python 3.14) is not a supported PX4/ROS 2 platform,
# so the whole stack runs in an Ubuntu 24.04 container:
#   ROS 2 Jazzy  +  Gazebo Harmonic  +  PX4-Autopilot v1.17.0
#   Micro-XRCE-DDS-Agent v2.4.3  +  px4_msgs release/1.17 (pinned commit)
# See docs/ENVIRONMENT.md for the rationale and exact versions.

FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG PX4_TAG=v1.17.0
ARG XRCE_AGENT_TAG=v2.4.3
ARG PX4_MSGS_COMMIT=86d8239e962f6939e05c3737784f60c02fa884db

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---- base tooling ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo curl wget gnupg lsb-release git ca-certificates \
        build-essential cmake ninja-build python3-pip python3-venv \
        mesa-utils libgl1-mesa-dri libegl1 libegl-mesa0 vainfo \
        xvfb psmisc procps iproute2 less nano \
    && rm -rf /var/lib/apt/lists/*

# ---- Gazebo Harmonic (OSRF stable repo) -----------------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 Jazzy packages -------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
        ros-jazzy-rviz2 ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
        ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-sensor-msgs \
        ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-sensor-msgs-py \
        ros-jazzy-image-transport ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-launch-testing ros-jazzy-launch-testing-ament-cmake \
        python3-colcon-common-extensions python3-opencv python3-numpy \
        python3-scipy python3-yaml python3-pytest python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# ---- PX4-Autopilot (SITL only, no NuttX toolchain) ------------------------
RUN git clone --branch ${PX4_TAG} --depth 1 --recursive --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot
RUN apt-get update \
    && bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
    && rm -rf /var/lib/apt/lists/*
RUN cd /opt/PX4-Autopilot && make px4_sitl -j"$(nproc)" \
    && test -x build/px4_sitl_default/bin/px4

# ---- Micro XRCE-DDS Agent (PX4 <-> ROS 2 middleware) -----------------------
RUN git clone --branch ${XRCE_AGENT_TAG} --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/Micro-XRCE-DDS-Agent \
    && cmake -S /opt/Micro-XRCE-DDS-Agent -B /opt/Micro-XRCE-DDS-Agent/build \
    && cmake --build /opt/Micro-XRCE-DDS-Agent/build -j"$(nproc)" \
    && cmake --install /opt/Micro-XRCE-DDS-Agent/build && ldconfig /usr/local/lib/

# ---- px4_msgs overlay workspace -------------------------------------------
RUN mkdir -p /opt/px4_ws/src && cd /opt/px4_ws/src \
    && git clone https://github.com/PX4/px4_msgs.git && cd px4_msgs \
    && git checkout ${PX4_MSGS_COMMIT} \
    && cd /opt/px4_ws && source /opt/ros/jazzy/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion- \
    && rm -rf build log

# ---- backend Python deps (pinned) -----------------------------------------
COPY docker/requirements-container.txt /tmp/requirements-container.txt
RUN python3 -m pip install --break-system-packages --no-cache-dir -r /tmp/requirements-container.txt

ENV PX4_DIR=/opt/PX4-Autopilot \
    PX4_WS=/opt/px4_ws \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PYTHONUNBUFFERED=1

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
