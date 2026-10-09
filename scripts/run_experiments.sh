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
    RID="${TAG}_t$(printf %02d "$i")"
    /workspace/scripts/run_mission.sh --scenario "$SCEN" --seed "$s" --run-id "$RID"
    if [ $? -eq 75 ]; then   # simulator stalled (infrastructure): archive and retry once with the same seed
        mkdir -p /workspace/results/infra_failures && mv "/workspace/results/raw/$RID" "/workspace/results/infra_failures/$RID"
        echo "[ps05] $RID: simulator stall -> archived to results/infra_failures, retrying once"
        /workspace/scripts/run_mission.sh --scenario "$SCEN" --seed "$s" --run-id "$RID"
    fi
done
python3 /workspace/scripts/aggregate_results.py
