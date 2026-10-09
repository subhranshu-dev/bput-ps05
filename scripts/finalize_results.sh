#!/usr/bin/env bash
# Final validation + reporting on the current code:
#   smoke test (writes results/reports/smoke/*), full test suite, aggregation, report, traceability.
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
cd /workspace
/workspace/scripts/smoke_test.sh; SMOKE=$?
/workspace/scripts/run_tests.sh; TESTS=$?
python3 scripts/gps_audit.py --out results/reports/gps_audit_static.json > /dev/null
python3 scripts/bench_planner.py --unknown-beyond 25 > results/reports/planner_bench.json 2>/dev/null || true
python3 scripts/reevaluate_clearance.py > results/reports/reevaluate_clearance.txt
python3 scripts/aggregate_results.py > results/reports/aggregate.txt
python3 scripts/generate_report.py
python3 scripts/generate_traceability.py
fix_owner
echo "[ps05] finalize: smoke=$SMOKE tests=$TESTS"
