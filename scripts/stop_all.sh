#!/usr/bin/env bash
# Stop experiments, simulation, autonomy and backend; optionally remove the container (--rm).
source "$(dirname "$0")/lib.sh"
# kill by PID list (pkill -f inside `bash -c '<pattern>'` would match and kill its own shell first)
PATTERN='run_full_campaign|run_experiments|run_stress|run_mission|run_evaluation|ros2 launch|bin/px4|gz sim|MicroXRCEAgent|parameter_bridge|/workspace/ros2_ws/install/|uvicorn app.main|rviz2'
KILL="ps -eo pid,args | grep -E '$PATTERN' | grep -v -E 'grep|stop_all' | awk '{print \$1}' | xargs -r kill -9"
if in_container; then
    bash -c "$KILL"; fix_owner; echo "[ps05] stopped"; exit 0
fi
if docker ps --format '{{.Names}}' | grep -qx "$PS05_CONTAINER"; then
    docker exec "$PS05_CONTAINER" bash -c "$KILL; chown -R $(id -u):$(id -g) /workspace/results /workspace/simulation /workspace/ros2_ws 2>/dev/null"
    [ "${1:-}" = "--rm" ] && docker rm -f "$PS05_CONTAINER" >/dev/null && echo "[ps05] container removed"
fi
echo "[ps05] all stopped"
