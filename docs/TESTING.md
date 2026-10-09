# Testing

## Levels

| Level | Command | What it covers |
|---|---|---|
| Unit (pure Python, no ROS runtime) | `./scripts/run_tests.sh` → `tests/unit` | frames (ENU/NED, quaternions), PS05 spherical↔Cartesian + covariance, pinhole projection, colourisation, extrinsic-perturbation shift, edge chamfer, timestamp sync (perfect / small / large offset / out-of-order), voxel ray tracing (free/occupied/unknown, moving-obstacle clearing, max range), inflation radius, A* (empty, wall, blocked→over, overhead→under, impossible goal, enclosed goal, corridor violation, narrow passage, unsafe-clearance slot, unknown policy, no corner cutting, validity consistency), local/global grid equivalence, trajectory (continuity, v/a limits, corner speed, smoothing validity, collision + corridor safety), landing detector (synthetic AprilTag pose, occlusion, partial view, wrong id), LiDAR landing validation (safe, obstruction, tilt, no ground, misaligned), PID (saturation, anti-windup, convergence), mission state machine with a kinematic vehicle (nominal landing, obstacle→replan, no-path safe state + recovery, target loss + reacquire, unsafe pad abort/climb/search, permanently unsafe pad, sensor failure recovery, corridor breach containment, INIT waits for converged localisation), metrics, config overrides / corrupted config, TF tree (parents, acyclic, valid rotations, optical convention, down-pointing sensors, **model ↔ TF consistency**) |
| Regression | `tests/regression` | static GPS / ground-truth audit (and that the audit detects planted violations), autonomy packages never import evaluation / simulation / dashboard code, planner clearance > live-check envelope |
| Backend | `backend/tests` | health, GPS status, ingest + query, input validation, WebSocket hello + live update, scenario validation, experiment validation, path-traversal rejection |
| Integration (sim) | `./scripts/smoke_test.sh` | Gazebo→ROS topics and rates, CameraInfo, LiDAR frame, TF tree completeness + stability, PX4 connection, arm/takeoff/hover/move/land through `px4_interface`, runtime GPS audit |
| End-to-end (sim) | `./scripts/run_mission.sh --scenario <id>` | complete mission; outcome + metrics in `results/raw/<run>/metrics.json` |
| Robustness / failure injection | `./scripts/run_stress_tests.sh` | 24 scenarios (see `simulation/scenarios`) |

Reports: `results/reports/test_report.txt`, `backend_test_report.txt`, `gps_audit_static.json`, `smoke/`.
Results of the latest runs are summarised in `docs/FINAL_REPORT.md`.

## Rules followed
No test was disabled or weakened to obtain a green run. Where a test's *expectation* was wrong (e.g. boundary
semantics of `clearance < radius`) the test was corrected with an explanatory comment; where the *code* was wrong
(e.g. corner cutting, trajectory resampling, validation sampling) the code was fixed and a regression test added.
