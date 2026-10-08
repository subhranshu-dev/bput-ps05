#!/usr/bin/env bash
# Developer helper: start simulation + sensors + px4_interface in the background (logs in results/logs/dev_*.log).
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
kill_sim
export PS05_SCENARIO_FILE=${PS05_SCENARIO_FILE:-/workspace/simulation/scenarios/baseline.yaml}
L=/workspace/results/logs
nohup ros2 launch /workspace/launch/simulation.launch.py > $L/dev_sim.log 2>&1 &
sleep 8
nohup ros2 launch /workspace/launch/sensors.launch.py > $L/dev_sensors.log 2>&1 &
nohup ros2 run px4_interface px4_interface --ros-args -p use_sim_time:=true > $L/dev_px4if.log 2>&1 &
echo "[ps05] dev stack starting"
