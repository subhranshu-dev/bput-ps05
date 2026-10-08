#!/usr/bin/env bash
# Unit + regression + backend tests (inside the container). Writes results/reports/test_report.txt
# usage: run_tests.sh [--with-sim]   (--with-sim also runs the simulation smoke/integration test)
set -o pipefail
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
cd /workspace
mkdir -p results/reports
RC=0
python3 -m pytest -q tests/unit tests/regression -p no:cacheprovider 2>&1 | tee results/reports/test_report.txt || RC=1
(cd backend && python3 -m pytest -q tests -p no:cacheprovider) 2>&1 | tee results/reports/backend_test_report.txt || RC=1
python3 scripts/gps_audit.py --out results/reports/gps_audit_static.json > /dev/null && echo "GPS audit (static): PASS" || { echo "GPS audit (static): FAIL"; RC=1; }
if [ "${1:-}" = "--with-sim" ]; then
    /workspace/scripts/smoke_test.sh || RC=1
fi
echo "[ps05] tests finished rc=$RC"
exit $RC
