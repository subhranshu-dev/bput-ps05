#!/usr/bin/env bash
# Repeated-trial evaluation (Phase 20): N baseline + N demo (dynamic obstacle) trials, then aggregate.
#   ./scripts/run_evaluation.sh [N=25]
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
N=${1:-25}
/workspace/scripts/run_experiments.sh --scenario baseline --trials "$N" --seed 100 --tag eval_baseline
/workspace/scripts/run_experiments.sh --scenario demo --trials "$N" --seed 200 --tag eval_dynamic
python3 /workspace/scripts/aggregate_results.py
