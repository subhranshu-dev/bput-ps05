# BPUT PS05 Demonstration

## One command
```bash
./scripts/run_demo.sh            # headless sim + dashboard at http://localhost:8000
./scripts/run_demo.sh --gui --rviz   # also the Gazebo GUI and RViz2 (needs an X display)
```
`run_demo.sh` checks the image, starts the container, builds the frontend if needed, generates the model/world,
builds the ROS workspace, starts the backend, prints the URLs and runs the `demo` scenario
(`simulation/scenarios/demo.yaml`). Results land in `results/raw/demo_<timestamp>/`.

## What happens (and where to see it)

| # | Step | Evidence on the dashboard / logs |
|---|---|---|
| 1 | Full system launches (Gazebo, PX4 SITL, XRCE agent, bridge, 17 ROS nodes) | top bar *SIMULATION: RUNNING*, *PX4 CONNECTED* |
| 2 | **GPS OFF** | red *GPS: OFF / DISABLED* chip; `gps_audit.py --runtime` PASS |
| 3 | UAV starts on the yellow start pad (5, 5) | Vehicle status X/Y/Z |
| 4 | INIT waits for tracker/EKF2 convergence, then ARMING → TAKEOFF | state machine bar, event timeline |
| 5–6 | LiDAR and forward camera observe the world | camera panel; sensor health (rates) |
| 7 | LiDAR-camera fusion (time-synchronised, TF2, pinhole projection) | 3D view *Fused cloud* (RGB points); *Sync* pairs; *Fusion projection* px |
| 8 | 3D occupancy map grows | 3D view *Occupancy map* voxels (fused colours); *Map … occ* |
| 9 | 3D A* computes a route inside the approved corridor | yellow *Planned path*, green corridor tube; *Planner latency*, *Path length* |
| 10 | UAV follows the smoothed trajectory | blue *Local trajectory*, white *Actual path*; PS05 E vs Emax gauge |
| 11 | **Unexpected obstacle**: a 4 m × 7 m panel is placed across the path ahead | magenta panel in the 3D view; `scenario … unexpected_obstacle` event |
| 12 | LiDAR / map detect it | *OBSTACLE DETECTED* chip, event "obstacle x m ahead" |
| 13–14 | Local layer triggers STOP/HOVER | `OBSTACLE_STOP` |
| 15 | New path generated **inside the approved mission space** | `REPLANNING → NAVIGATE`, new yellow path, *Replan latency* |
| 16–17 | UAV continues to the landing region (approximate centre 41, 41) | `LANDING_ZONE_SEARCH` |
| 18 | Downward camera detects the AprilTag 36h11 | downward feed with green ring, *TAG 36H11 · x m* |
| 19 | LiDAR validates ground plane, tilt, roughness, clearance | *Precision landing* checklist PASS |
| 20 | PID alignment over the tag | *Align error*, PID u_x/u_y |
| 21 | Staged descent with re-validation at 2.0 / 1.0 / 0.5 m | checkpoint events |
| 22 | Final safety check | `FINAL_CHECK → LAND` |
| 23 | Touchdown (PX4 land detector), disarm | `LANDED` |
| 24 | Landing error computed vs. simulator ground truth (evaluation only) | *Touchdown result* card |
| 25 | Complete record saved | `results/raw/<run>/{metrics.json, telemetry.csv, events.jsonl, config.json, launch.log, px4_ulog/}` |
| 26 | Statistics | *Experiments* table; `results/summary.json`, `results/plots/` |

## Other demonstrations
* `--scenario pad_obstruction`: crate on the pad → `UNSAFE_PAD → ABORT_CLIMB → LANDING_ZONE_SEARCH` → lands after removal.
* `--scenario tag_loss`: tag occluded during descent → `TARGET_LOST` (hover) → reacquire → land.
* `--scenario no_path`: corridor too narrow → `NO_SAFE_PATH` safe state → `ABORTED` (controlled landing in place).
* `--scenario lidar_failure`: LiDAR blackout during navigation → `SENSOR_FAILURE` hover → recovery.
* `--scenario extrinsic_perturbation`: camera extrinsic error → larger fusion projection error (px).
