#!/usr/bin/env bash
# Stop simulation, autonomy, backend; optionally remove the container (--rm).
source "$(dirname "$0")/lib.sh"
if in_container; then
    kill_sim; pkill -f "uvicorn app.main:app" 2>/dev/null || true; echo "[ps05] stopped"; exit 0
fi
if docker ps --format '{{.Names}}' | grep -qx "$PS05_CONTAINER"; then
    docker exec "$PS05_CONTAINER" bash -lc "source /workspace/scripts/lib.sh; source /opt/ros/jazzy/setup.bash; kill_sim; pkill -f 'uvicorn app.main:app' || true"
    [ "${1:-}" = "--rm" ] && docker rm -f "$PS05_CONTAINER" >/dev/null && echo "[ps05] container removed"
fi
echo "[ps05] all stopped"
