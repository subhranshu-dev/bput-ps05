#!/usr/bin/env bash
# Shared helpers. Source from other scripts.
PS05_IMAGE=${PS05_IMAGE:-bput-ps05:jazzy-harmonic-px4v1.17.0}
PS05_CONTAINER=${PS05_CONTAINER:-ps05}
HOST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

in_container() { [ -f /.dockerenv ] && [ -d /opt/PX4-Autopilot ]; }

container_up() {
    if docker ps --format '{{.Names}}' | grep -qx "$PS05_CONTAINER"; then return 0; fi
    docker rm -f "$PS05_CONTAINER" >/dev/null 2>&1 || true
    local gid; gid=$(getent group render | cut -d: -f3)
    local xargs=()
    if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
        xargs=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
        command -v xhost >/dev/null && xhost +local:docker >/dev/null 2>&1 || true
    fi
    # Pass only Mesa-capable GPUs (Intel/AMD). NVIDIA nodes have no userspace driver in the container and
    # make headless EGL pick an unusable device (OGRE2 "OpenGL 3.3 is not supported" -> sensor thread crash).
    local devs=()
    for n in /dev/dri/card* /dev/dri/renderD*; do
        local v; v=$(cat /sys/class/drm/"$(basename "$n")"/device/vendor 2>/dev/null)
        [ -n "$v" ] && [ "$v" != "0x10de" ] && devs+=(--device "$n")
    done
    [ ${#devs[@]} -eq 0 ] && echo "[ps05] WARNING: no Mesa GPU found; Gazebo will use software rendering"
    docker run -d --name "$PS05_CONTAINER" --network host --ipc host \
        "${devs[@]}" ${gid:+--group-add "$gid"} --group-add video \
        -e PS05_ROOT=/workspace -e QT_X11_NO_MITSHM=1 -e ROS_LOCALHOST_ONLY=0 "${xargs[@]}" \
        -v "$HOST_ROOT":/workspace -w /workspace "$PS05_IMAGE" sleep infinity >/dev/null
    echo "[ps05] container $PS05_CONTAINER started"
}

# Re-run the calling script inside the container if we are on the host.
ensure_container() {
    if ! in_container; then
        container_up
        exec docker exec -i $( [ -t 0 ] && echo -t ) -e PS05_ROOT=/workspace "$PS05_CONTAINER" \
            bash -lc "/workspace/scripts/$(basename "$0") $(printf '%q ' "$@")"
    fi
    export PS05_ROOT=/workspace
    set +u
    source /opt/ros/jazzy/setup.bash
    source /opt/px4_ws/install/setup.bash
    [ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash
}

kill_sim() {
    pkill -INT -f "ros2 launch" 2>/dev/null || true
    sleep 2
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "bin/px4" 2>/dev/null || true
    pkill -9 -f MicroXRCEAgent 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "/workspace/ros2_ws/install/.*/lib/" 2>/dev/null || true
    pkill -9 -f rviz2 2>/dev/null || true
    ros2 daemon stop >/dev/null 2>&1 || true
}
