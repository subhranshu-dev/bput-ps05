#!/usr/bin/env bash
# Start the FastAPI backend (serves the built frontend) in the background inside the container.
source "$(dirname "$0")/lib.sh"
ensure_container "$@"
if curl -sf http://127.0.0.1:8000/api/health >/dev/null; then echo "[ps05] backend already running"; exit 0; fi
mkdir -p /workspace/results/logs
cd /workspace/backend && nohup python3 -m uvicorn app.main:app --host 0.0.0.0 --port 8000 \
    > /workspace/results/logs/backend.log 2>&1 &
for i in $(seq 1 30); do curl -sf http://127.0.0.1:8000/api/health >/dev/null && { echo "[ps05] backend up on :8000"; exit 0; }; sleep 0.5; done
echo "[ps05] backend failed to start; see results/logs/backend.log"; exit 1
