# Troubleshooting

Every entry below was hit during bring-up of this repository; the fix is in the code.

| Symptom | Root cause | Fix / where |
|---|---|---|
| `gz sim` exits −11 when the vehicle spawns; log: `OGRE EXCEPTION … OpenGL 3.3 is not supported`, `libEGL … pci id 10de` | Headless EGL enumerated the NVIDIA DRI node, which has no userspace driver in the container | Only Mesa (non-0x10de) DRI nodes are passed to the container — `scripts/lib.sh::container_up` |
| PX4 `Spawning Gazebo model` but nothing appears | Standalone mode spawns `file://$PX4_GZ_MODELS/<model>/model.sdf`; `PX4_GZ_MODELS` only set by `gz_env.sh` | Set in `launch/simulation.launch.py` |
| EKF2 position looks perfect / noise-free; `vehicle_visual_odometry` has a valid quaternion and zero variance | PX4 `gz_bridge` republishes `/model/<name>/odometry_with_covariance` as external vision — the evaluation OdometryPublisher's default output = **ground truth leaking into EKF2** | `<odom_covariance_topic>/sim/truth/odom_with_covariance</odom_covariance_topic>`; `scripts/gps_audit.py` checks it statically and at runtime |
| `estimator_status_flags.cs_ev_pos = False`; listener shows EV sample "1.1 s ago" | uXRCE-DDS timesync to the agent's wall clock while ROS stamps are sim time | `UXRCE_DDS_SYNCT 0` in the PS05 airframe (lockstep: PX4 time = Gazebo time) |
| Vehicle takes off toward the world origin | Home latched from `/vehicle/odom` before EKF2 switched to tracker aiding (local position still at 0,0) | INIT requires EKF/tracker agreement < 0.3 m for 2 s; home latched at arming |
| `Arming denied: Resolve system health failures first` for a few seconds | EKF2 converging after the first EV samples | Expected; ARMING retries every second (30 s timeout) |
| `mission_manager` crash: `Logger severity cannot be changed between calls` | rclpy forbids one call site logging at different severities | Explicit per-severity calls |
| First plan takes > 5 s / `search limit reached` | Plain A* in largely UNKNOWN space under CPU load (~10 k expansions/s) | Weighted A* w = 1.3 + unknown cost 1.2 (≈ 20× fewer expansions, +0.08 % length) — `scripts/bench_planner.py` |
| Repeated `OBSTACLE_STOP` at places with no geometry; points at exactly ±45°/±135° azimuth, some below ground | Gazebo `gpu_lidar` cube-map seams return spurious depths for beams exactly on the seam | Scan −179.5°…+179.5° (no beam on a seam). Phantom voxels 3.6 % → 0.8 % |
| `OBSTRUCTION` on a clear landing pad, `max_obstruction_height` ≈ vehicle height − 0.2 m | Downward flash LiDAR sees the landing-gear feet in its FOV corners | 0.35 m self-filter on the downward LiDAR |
| Tag detected in ~6 % of frames at 5 m | 0.40 m tag only ~27 px with an 86° lens | 60° downward camera, search at 4 m (~58 px) |
| `TARGET_LOST` just before touchdown | Below ~0.6 m the tag is larger than the image | Blind-descent zone below 1.0 m AGL (hold last estimate; LiDAR checks continue) |
| `trajectory failed final validation (collision)` although A* succeeded | (a) 26-connected diagonal moves clipped blocked corners; (b) different sampling densities in pruning / smoothing / final check | No-corner-cutting rule; one `polyline_clear()` at 2 cm everywhere, final check on the exact smoothed polyline |
| `pkill -f <name>` inside `docker exec bash -c '…<name>…'` does nothing | pkill matches and kills its own shell first | Kill by PID list (`scripts/stop_all.sh`) |
| Result files owned by root on the host | Container runs as root | `fix_owner` in `scripts/lib.sh` (HOST_UID/GID) |
| `pip` fails: `Cannot uninstall pluggy … installed by debian` | `pytest-asyncio` forced a pytest upgrade | Dependency removed |

## Quick checks

```bash
./scripts/run_tests.sh                    # unit + regression + backend + static GPS audit
./scripts/smoke_test.sh                   # sim + PX4 + sensors + TF + takeoff/hover/land + runtime GPS audit
python3 scripts/check_system.py           # (inside container, sim running) topic rates, CameraInfo, TF, PX4 link
python3 scripts/gps_audit.py --runtime    # (inside container, sim running)
```
Logs: `results/raw/<run>/launch.log` (all nodes), `results/logs/*.log`, rejected plans and LiDAR-triggered stops are
dumped to `results/debug/*.npz` for offline reproduction.
