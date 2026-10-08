#!/usr/bin/env bash
# One-command BPUT PS05 demonstration (run on the host).
#   ./scripts/run_demo.sh [--gui] [--rviz] [--scenario demo]
# Starts the container, builds, launches the backend + dashboard, then the full autonomous mission.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
source "$HERE/lib.sh"
SCEN=demo; EXTRA=()
while [ $# -gt 0 ]; do case "$1" in --scenario) SCEN=$2; shift 2;; *) EXTRA+=("$1"); shift;; esac; done
if ! in_container; then
    docker image inspect "$PS05_IMAGE" >/dev/null 2>&1 || { echo "image $PS05_IMAGE missing: run docker build -f docker/Dockerfile -t $PS05_IMAGE ."; exit 1; }
    container_up
    [ -f "$HOST_ROOT/frontend/dist/index.html" ] || (cd "$HOST_ROOT/frontend" && npm ci && npm run build)
fi
"$HERE/build.sh"
"$HERE/start_backend.sh"
echo "=================================================================="
echo " BPUT PS05 ground control  : http://localhost:8000"
echo " API health                : http://localhost:8000/api/health"
echo " GPS                       : OFF (no GNSS sensor, EKF2_GPS_CTRL=0)"
echo " scenario                  : $SCEN"
echo "=================================================================="
"$HERE/run_mission.sh" --scenario "$SCEN" --run-id "demo_$(date +%Y%m%d_%H%M%S)" "${EXTRA[@]}"
