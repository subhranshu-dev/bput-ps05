#!/usr/bin/env bash
# Generate simulation assets and build the ROS 2 workspace (inside the container).
set -eo pipefail
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
cd /workspace
python3 scripts/generate_vehicle_model.py
python3 scripts/generate_world.py
cd ros2_ws
colcon build --symlink-install --event-handlers console_cohesion- 2>&1 | tail -25
echo "[ps05] build complete"
