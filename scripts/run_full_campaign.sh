#!/usr/bin/env bash
# Full evaluation campaign: repeated trials (Phase 20) + robustness/failure matrix (Phases 21, 28).
#   run_full_campaign.sh [N_BASELINE=20] [N_DYNAMIC=10] [K_STRESS=1]
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
NB=${1:-20}; ND=${2:-10}; K=${3:-1}
/workspace/scripts/run_experiments.sh --scenario baseline --trials "$NB" --seed 100 --tag eval_baseline
/workspace/scripts/run_experiments.sh --scenario demo --trials "$ND" --seed 200 --tag eval_dynamic
/workspace/scripts/run_stress_tests.sh "$K"
python3 /workspace/scripts/aggregate_results.py
fix_owner
echo "[ps05] CAMPAIGN COMPLETE"
