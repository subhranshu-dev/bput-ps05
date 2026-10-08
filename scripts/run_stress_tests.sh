#!/usr/bin/env bash
# Robustness + failure-injection matrix (Phases 21, 28). K trials per scenario (default 2).
#   ./scripts/run_stress_tests.sh [K] [scenario ...]
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
K=${1:-2}; shift || true
SCENS=("$@")
[ ${#SCENS[@]} -eq 0 ] && SCENS=(static_wall narrow_pole overhead dynamic_obstacle crossing_obstacle lidar_noise lidar_dropout
    camera_blur camera_exposure camera_noise extrinsic_perturbation timestamp_offset timestamp_offset_small landing_offset
    tag_loss pad_obstruction pad_near_obstacle start_position start_yaw altitude no_path lidar_failure camera_failure tracker_noise)
for s in "${SCENS[@]}"; do
    /workspace/scripts/run_experiments.sh --scenario "$s" --trials "$K" --seed 300 --tag "stress_$s"
done
python3 /workspace/scripts/aggregate_results.py
