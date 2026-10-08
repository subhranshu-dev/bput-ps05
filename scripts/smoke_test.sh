#!/usr/bin/env bash
# Phase 0/1 automated smoke test: PX4 SITL + Gazebo + ROS 2 + bridge + sensors + TF, then
# arm -> takeoff -> hover -> move -> land through px4_interface using GPS-free localisation.
set -o pipefail
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
OUT=/workspace/results/reports/smoke; mkdir -p "$OUT"
export PS05_SCENARIO_FILE=/workspace/simulation/scenarios/baseline.yaml
kill_sim
ros2 launch /workspace/launch/simulation.launch.py > "$OUT/simulation.log" 2>&1 &
sleep 8
ros2 launch /workspace/launch/sensors.launch.py > "$OUT/sensors.log" 2>&1 &
ros2 run px4_interface px4_interface --ros-args -p use_sim_time:=true > "$OUT/px4_interface.log" 2>&1 &
sleep 20
python3 /workspace/scripts/check_system.py --out "$OUT/system_check.json"; CHECK=$?
ros2 run px4_interface smoke_flight --out "$OUT/smoke_flight.json" --ros-args -p use_sim_time:=true; FLIGHT=$?
python3 /workspace/scripts/gps_audit.py --runtime --out "$OUT/gps_audit_runtime.json" > /dev/null; AUDIT=$?
kill_sim
echo "[ps05] smoke: system_check=$CHECK flight=$FLIGHT gps_audit=$AUDIT"
[ $CHECK -eq 0 ] && [ $FLIGHT -eq 0 ] && [ $AUDIT -eq 0 ]
