#!/usr/bin/env bash
# Repeated trials: run_experiments.sh --scenario baseline --trials 25 [--seed 0] [--tag name]
set -o pipefail
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
SCEN=baseline; N=1; SEED=0; TAG=""
while [ $# -gt 0 ]; do case "$1" in
    --scenario) SCEN=$2; shift 2;; --trials) N=$2; shift 2;; --seed) SEED=$2; shift 2;; --tag) TAG=$2; shift 2;;
    *) echo "unknown arg $1"; exit 2;; esac; done
TAG=${TAG:-${SCEN}_$(date +%Y%m%d_%H%M%S)}
trap 'echo "[ps05] interrupted"; kill_sim; exit 130' INT TERM
for i in $(seq 1 "$N"); do
    s=$((SEED + i - 1))
    /workspace/scripts/run_mission.sh --scenario "$SCEN" --seed "$s" --run-id "${TAG}_t$(printf %02d "$i")"
done
python3 /workspace/scripts/aggregate_results.py
