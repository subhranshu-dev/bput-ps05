#!/usr/bin/env bash
# Run ONE complete mission trial headless and record results/raw/<run_id>/.
# usage: run_mission.sh --scenario demo [--run-id ID] [--seed N] [--timeout 600] [--gui] [--rviz] [--no-dashboard]
set -o pipefail
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
SCEN=baseline; RUN_ID=""; SEED=""; TIMEOUT=600; GUI=false; RVIZ=false; DASH=true
while [ $# -gt 0 ]; do case "$1" in
    --scenario) SCEN=$2; shift 2;; --run-id) RUN_ID=$2; shift 2;; --seed) SEED=$2; shift 2;;
    --timeout) TIMEOUT=$2; shift 2;; --gui) GUI=true; shift;; --rviz) RVIZ=true; shift;;
    --no-dashboard) DASH=false; shift;; *) echo "unknown arg $1"; exit 2;; esac; done
SFILE=/workspace/simulation/scenarios/$SCEN.yaml
[ -f "$SFILE" ] || { echo "no scenario $SFILE"; exit 2; }
RUN_ID=${RUN_ID:-${SCEN}_$(date +%Y%m%d_%H%M%S)}
OUT=/workspace/results/raw/$RUN_ID; mkdir -p "$OUT"
# per-run scenario copy with the seed applied
python3 - "$SFILE" "$OUT/scenario.yaml" "$SEED" <<'PY'
import sys, yaml
s = yaml.safe_load(open(sys.argv[1]))
if sys.argv[3]:
    s['seed'] = int(sys.argv[3])
yaml.safe_dump(s, open(sys.argv[2], 'w'), sort_keys=False)
PY
POSE=$(python3 -c "import yaml;print(yaml.safe_load(open('$OUT/scenario.yaml')).get('start_pose','5,5,0.25,0,0,0'))")
export PS05_SCENARIO_FILE=$OUT/scenario.yaml PS05_RUN_ID=$RUN_ID
kill_sim
echo "[ps05] run $RUN_ID scenario=$SCEN pose=$POSE timeout=${TIMEOUT}s"
ros2 launch /workspace/launch/full_system.launch.py gui:=$GUI rviz:=$RVIZ dashboard:=$DASH \
    start_pose:="$POSE" run_id:=$RUN_ID > "$OUT/launch.log" 2>&1 &
LPID=$!
T0=$(date +%s); RESULT=TIMEOUT
while kill -0 $LPID 2>/dev/null; do
    sleep 3
    if [ -f "$OUT/metrics.json" ]; then
        st=$(python3 -c "import json;print(json.load(open('$OUT/metrics.json')).get('state',''))" 2>/dev/null)
        if [ "$st" = "LANDED" ] || [ "$st" = "ABORTED" ]; then RESULT=$st; sleep 3; break; fi
    fi
    if [ $(( $(date +%s) - T0 )) -gt "$TIMEOUT" ]; then RESULT=TIMEOUT; break; fi
done
kill -INT $LPID 2>/dev/null; sleep 4; kill_sim; wait $LPID 2>/dev/null
cp -r /tmp/px4_rootfs/log "$OUT/px4_ulog" 2>/dev/null || true
echo "[ps05] run $RUN_ID finished: $RESULT (wall $(( $(date +%s) - T0 )) s)"
python3 -c "
import json; m=json.load(open('$OUT/metrics.json'))
print(json.dumps({k:m.get(k) for k in ('outcome','success','landing_error_m','collision_count','min_clearance_m','replans','mission_time_s')}))" 2>/dev/null
