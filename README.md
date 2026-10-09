# BPUT Hackathon 2026 — PS05: GPS-Denied 3D Autonomous UAV

**GPS-independent autonomous quadrotor (PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy) with LiDAR–camera fusion,
3D occupancy mapping, 3D A* inside an approved mission corridor, local replanning around unexpected obstacles,
AprilTag precision landing with LiDAR landing-surface validation, quantitative evaluation and a live
ground-control dashboard.**

```
GPS: OFF  ──  tracker (r,θ,φ) → EKF2 ──► camera + 3D LiDAR → TF2/sync → fusion → 3D map → inflation
            → approved corridor → 3D A* → trajectory → local check → PX4 → AprilTag + LiDAR → precision landing
```

All numbers in this repository come from logged simulation runs (`results/`). See
[`docs/FINAL_REPORT.md`](docs/FINAL_REPORT.md) for measured results and
[`docs/REQUIREMENTS_TRACEABILITY.md`](docs/REQUIREMENTS_TRACEABILITY.md) for requirement-by-requirement evidence.

## Quick start

```bash
# prerequisites: Docker, Node 18+ (frontend build), a Mesa-capable GPU (Intel/AMD) for sensor rendering
docker build -f docker/Dockerfile -t bput-ps05:jazzy-harmonic-px4v1.17.0 .   # ~40 min, version-locked
(cd frontend && npm ci && npm run build)
./scripts/run_demo.sh                 # full mission with an unexpected obstacle + dashboard
# open http://localhost:8000
```

## Installation
Everything (ROS 2 Jazzy, Gazebo Harmonic 8.15, PX4 v1.17.0, Micro XRCE-DDS Agent v2.4.3, px4_msgs release/1.17)
is installed in the Docker image defined in `docker/Dockerfile`. The host only needs Docker and Node.
`scripts/lib.sh` starts the container (`ps05`) with host networking and only Mesa DRI devices, mounting the repo at
`/workspace`; every script re-executes itself inside it. Exact versions: [`docs/ENVIRONMENT.md`](docs/ENVIRONMENT.md).

## Build
```bash
./scripts/build.sh      # generates the vehicle model + world from configs, then colcon build (17 packages)
```

## Simulation
```bash
./scripts/smoke_test.sh                          # Phase 0/1: PX4+Gazebo+ROS 2, sensors, TF, takeoff/hover/land
./scripts/run_mission.sh --scenario baseline     # one complete autonomous mission (results/raw/<run>/)
./scripts/run_mission.sh --scenario demo --gui --rviz   # with Gazebo GUI and RViz (needs X display)
./scripts/stop_all.sh                            # stop everything
```
Scenarios live in `simulation/scenarios/*.yaml` (baseline, demo, 24 robustness / failure-injection scenarios).

## Dashboard
`./scripts/start_backend.sh` serves the built frontend and API on **http://localhost:8000**
(`/api/health`, `/api/system/status`, `/api/mission/status`, … and WebSocket `/ws/telemetry`).
`cd frontend && npm run dev` gives a hot-reload dev server on :5173 proxied to the backend.
The dashboard is read-only with respect to the autonomy stack; it can start/stop experiment batches.

## Testing
```bash
./scripts/run_tests.sh               # unit + regression + backend + static GPS/ground-truth audit
./scripts/run_tests.sh --with-sim    # + simulation smoke test and runtime GPS audit
```

## Demo
[`docs/DEMO.md`](docs/DEMO.md) — the 26-step BPUT demonstration and what each panel shows.

## Evaluation
```bash
./scripts/run_evaluation.sh 25       # repeated baseline + dynamic-obstacle trials
./scripts/run_stress_tests.sh 2      # robustness + failure-injection matrix
./scripts/run_full_campaign.sh 20 10 1
python3 scripts/aggregate_results.py # -> results/summary.{csv,json}, results/plots/*.png
```

## Troubleshooting
[`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) — every failure found during bring-up, with root cause and fix.

## Architecture
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md), [`docs/ROS_GRAPH.md`](docs/ROS_GRAPH.md),
[`docs/SENSOR_FUSION.md`](docs/SENSOR_FUSION.md), [`docs/PLANNING.md`](docs/PLANNING.md), [`docs/LANDING.md`](docs/LANDING.md).

| Layer | Package(s) |
|---|---|
| Simulation | `simulation/` (world + model generated from YAML), `aeronav_simulation` (tracking-station sensor model, scenario harness) |
| Frames | `aeronav_description` (TF from `configs/sensors.yaml`) |
| Perception | `camera_processor`, `lidar_processor`, `sensor_fusion` |
| Localisation | `localization` (PS05 spherical→Cartesian → PX4 EKF2 external vision; **no GPS**) |
| Mapping / planning | `mapping`, `planner_3d`, `obstacle_avoidance` |
| Landing / mission | `landing_detector`, `landing_validator`, `landing_controller` |
| Flight control link | `px4_interface` |
| Evaluation / UI | `evaluation`, `dashboard_bridge`, `backend/`, `frontend/` |

## GPS-denied guarantee
* the simulated airframe has **no GNSS sensor**; PX4 `EKF2_GPS_CTRL=0`;
* position aiding = PS05 ground tracking station (noisy r, θ, φ at 60 Hz, 10 ms latency) → EKF2 external vision;
* simulator ground truth is consumed only by evaluation, the tracking-station *sensor model* and the scenario harness;
* `scripts/gps_audit.py` verifies this statically (source, model, airframe, truth routing) and at runtime
  (ROS graph subscriptions, GPS topics silent, no truth-injection publisher into PX4).

## Known limitations
See the "Limitations" section of [`docs/FINAL_REPORT.md`](docs/FINAL_REPORT.md). In short: the localisation source
is a simulated external tracker (as the PS05 specification prescribes), not on-board VIO/LiDAR odometry; all
results are simulation-only; Python nodes bound the update rates on a 12-core laptop.

## License
MIT — see `LICENSE`.
