# PROJECT AUDIT — BPUT Hackathon 2026 PS05

Audit date: 2026-10-09. Auditor: lead engineer (autonomous implementation run).

## 1. Repository state at start

| Item | Finding |
|---|---|
| Existing project repository | **None.** No PS05/aeronav/PX4 code anywhere on the machine (`find / -iname '*ps05*' -o -iname '*px4*' -o -iname '*aeronav*'`). |
| Branches | N/A — new repository `~/bput-ps05`, branch `main`. |
| ROS packages / launch files / worlds | None. |
| Frontend / backend | None. |
| Tests / configuration | None. |
| Source requirement documents | `~/Downloads/BPUT_Hackathon_2026_PS5_GPS_Denied_UAV_Technical_Report.pdf` (10 pp), `~/Downloads/BPUT_PS05_Complete_System_Architecture_and_Implementation_Guide.pdf` (16 pp), `~/Downloads/BPUT_PS05_LiDAR_Camera_Fusion_Virtual_Implementation_Report.pdf` (12 pp). |

Conclusion: greenfield implementation. Nothing to preserve, nothing broken to repair.

## 2. Host environment

| Item | Value |
|---|---|
| OS | Ubuntu 26.04.1 LTS (Resolute Raccoon), kernel 7.0.0-34 |
| CPU / RAM / disk | 12 logical cores, 14.7 GiB RAM + 4 GiB swap, ~44 GB free on `/` |
| GPU | Intel Alder Lake-P UHD (iGPU, `/dev/dri/renderD128`) + NVIDIA RTX 2050 4 GB (driver 595.91) |
| NVIDIA container runtime | **Not installed** (`docker info` runtimes: runc only) |
| Docker | 29.1.3, user is in `docker` group |
| Python (host) | 3.14.4 |
| Node / npm (host) | 22.22.1 / 9.2.0 |
| ROS 2 / Gazebo / PX4 (host) | **None installed** |
| Display | X11 `:0` + Wayland available |

## 3. Key requirements extracted from the three source documents

1. **GPS independence** — GPS excluded from the guidance chain; GPS hardware may exist only if logically isolated. RTK also prohibited.
2. **Permitted localisation** — external spatial tracking (radar / optical / acoustic) producing spherical measurements (r, θ, φ), converted with
   `X = r cosφ cosθ, Y = r cosφ sinθ, Z = r sinφ`, fused with onboard INS to bound drift. Tracking latency < 20 ms, telemetry > 50 Hz.
3. **Path enforcement** — pre-approved 3D trajectory inside a permissible corridor; spatial error `E = √(ΔX²+ΔY²+ΔZ²)`; `E > Emax` → critical alarm → containment. Phase-1 target: mean simulated trajectory error **E < 0.5 m** under simulated telemetry noise.
4. **Perception stack** (guides 2 & 3) — PX4 SITL + Gazebo + ROS 2, RGB camera + 3D LiDAR, TF2, explicit time sync, LiDAR→camera projection (`u = fx X/Z + cx`, `v = fy Y/Z + cy`), coloured point cloud.
5. **Mapping & planning** — 3D voxel occupancy (FREE/OCCUPIED/UNKNOWN, ray-traced), obstacle inflation, 3D A*, trajectory smoothing, local replanning around a dynamic obstacle.
6. **Precision landing** — AprilTag/ArUco + LiDAR surface/clearance validation; explicit state machine with TARGET_LOST / OBSTACLE / UNSAFE_PAD / SENSOR_FAILURE branches; landing error mean, RMSE, P95 over 20–30 trials.
7. **Evidence** — never invent metrics; ground truth only for evaluation.

## 4. Reconciliation decisions (documented engineering choices)

| Tension | Decision |
|---|---|
| PS05 spec says "no unrestricted dynamic planning" vs guides want A* + replanning | A* and the local replanner may only search inside the **Approved Mission Space** (box + corridor tube around the approved centreline). Every candidate path must pass map-collision, inflation and corridor checks. No valid path → HOVER safe state, then ABORT. |
| PS05 localisation is external tracking; guides say "permitted localisation/state estimation" | A simulated **ground tracking station** (sensor model in `aeronav_simulation`) emits noisy, delayed (r, θ, φ) measurements at 50+ Hz. `localization` converts them to Cartesian and feeds PX4 EKF2 as *external vision position*; EKF2 fuses with IMU/baro/mag. GPS sensor is **physically removed** from the simulated airframe and `EKF2_GPS_CTRL=0`. Like every simulated sensor, the tracker model reads simulator state internally; its *output* is only the noisy measurement. This is explicitly documented and covered by the GPS/ground-truth audit test. |
| Host OS 26.04 unsupported by PX4/ROS 2 | Run the full stack in a pinned Ubuntu 24.04 Docker image (ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17.0). Intel iGPU passed via `/dev/dri` for sensor rendering. Frontend is built on the host with Node 22. |
| LiDAR cannot see directly below the vehicle | Add a small downward **solid-state (flash) LiDAR** dedicated to landing-surface validation, plus a downward RGB camera for the tag. Forward RGB camera + 360° 3D LiDAR are used for fusion/mapping. |
| Ground-truth collision measurement | Evaluation computes clearance from ground-truth pose to the world's obstacle geometry parsed from the SDF (evaluation-only). |

## 5. Implementation plan (phase order)

Phase 0 environment image → Phase 1 PX4 offboard control node → Phase 2 world → Phases 3–5 sensors + TF → Phase 6–7 sync + fusion → Phase 8–10 map, inflation, corridor → Phase 11–13 A*, trajectory, local replanning → Phase 14 GPS audit → Phases 15–18 landing → Phases 19–21 evaluation + robustness → Phases 22–24 backend, frontend, RViz → Phases 25–35 testing, demo scripts, report, traceability.

Status of each phase is tracked in `docs/REQUIREMENTS_TRACEABILITY.md` with evidence links.
