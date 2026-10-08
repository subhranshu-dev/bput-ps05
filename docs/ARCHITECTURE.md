# System Architecture — BPUT PS05

## 1. Mental model

| Component | Responsibility | Does NOT |
|---|---|---|
| **Gazebo Harmonic** | 3D world, physics, X500 airframe, cameras, LiDARs, moving scenario objects | decide the path |
| **PX4 v1.17.0 SITL** | attitude/thrust/position control, flight modes, EKF2 state estimation (IMU + baro + mag + tracker, **no GPS**) | high-level planning |
| **ROS 2 Jazzy** | transport, TF2, all perception / mapping / planning / landing nodes | low-level flight control |
| **RViz2** | ROS debugging view (TF, clouds, map, paths, corridor, target) | physics |
| **Web dashboard** | judge / operator view, experiment control | feed the autonomy (read-only bridge) |

## 2. Data flow

```
                         GAZEBO (world + physics + sensors)
                ┌──────────────┬─────────────┬──────────────┬───────────────┐
        forward RGB      3D LiDAR (360x16)  down RGB     down flash LiDAR    vehicle state
                │              │             │              │                (truth, EVAL ONLY)
          ros_gz_bridge   ros_gz_bridge   ros_gz_bridge  ros_gz_bridge          │
                ▼              ▼             ▼              ▼                    ├──► ground_tracker (sensor model:
        camera_processor  lidar_processor  camera_proc.  lidar_processor         │      noisy r,θ,φ @60 Hz, 10 ms)
                │              │             │              │                    │            │
                └──► TF2 + TIME SYNC ◄─┘     │              │                    │     localization_node
                        sensor_fusion        │              │                    │   (PS05 polar→Cartesian,
                         /fused/points       │              │                    │    EKF2 external vision)
                              │              │              │                    │            │
                         mapping (ray-traced log-odds voxels, 0.25 m)            │     PX4 EKF2 ──► /vehicle/odom + TF
                              │ /occupancy_map/grid           │                  │
                ┌─────────────┴─────────────┐               │                  └──► evaluation_node (metrics)
           planner_3d                  obstacle_avoidance    │
   inflation + corridor + 3D A*      live LiDAR + map check   │
   + smoothing + validation               │                  │
                └───────► mission_manager (explicit state machine) ◄── landing_detector (AprilTag PnP)
                                    │                         ◄── landing_validator (plane fit, clearance)
                              px4_interface (offboard setpoints, ENU→NED)
                                    │ uXRCE-DDS
                                   PX4 ──► Gazebo
```

## 3. Approved Mission Space (reconciling path enforcement and replanning)

The PS05 specification requires a pre-approved trajectory/corridor; the implementation guides require A* and
replanning. Both are satisfied:

1. `configs/mission.yaml` defines the **Approved Mission Space**: a box (x, y, z limits) **and** a tube of radius
   `max_deviation` around the approved centreline, shrunk by `safety_margin`.
2. The planner's traversable set is `free ∧ ¬inflated ∧ inside approved space` — A* cannot produce a route outside.
3. Every trajectory is validated (map collision, inflated clearance, corridor at 0.125 m sampling) before it may be flown.
4. The local layer (obstacle_avoidance) only *detects*; the response is STOP/HOVER → replan **through the same
   constrained planner**, so replanning can never leave the approved space.
5. During flight, the PS05 spatial error `E = |p_est − p_ref|` is monitored against `Emax`. `E > Emax` for
   0.5 s → **CRITICAL ALARM → containment** (return to the last approved reference point and hold; if not
   recovered within 5 s → ABORT, land in place).
6. No route → `NO_SAFE_PATH` safe state (hover, bounded retries) → `ABORTED` (controlled landing in place).

## 4. GPS-denied design

* The Gazebo airframe `x500_ps05` is generated **without the NavSat (GNSS) sensor**.
* PX4 airframe `4050_gz_x500_ps05`: `EKF2_GPS_CTRL=0`, `EKF2_EV_CTRL=3`, `EKF2_HGT_REF=3` (external vision).
* Position aiding = PS05 ground tracking station: spherical (r, θ, φ) → `X = r cosφ cosθ, Y = r cosφ sinθ, Z = r sinφ`.
* `scripts/gps_audit.py` statically and at runtime proves that no autonomy node subscribes to GPS or truth topics.

| Data | Class | Consumers |
|---|---|---|
| camera / LiDAR / down camera / down LiDAR | AUTONOMY | processors, fusion, mapping, avoidance, landing |
| `/tracking/measurement` (noisy r,θ,φ) | AUTONOMY (permitted localisation) | localization_node → EKF2 |
| `/vehicle/odom`, TF `odom→base_link` (EKF2 output) | AUTONOMY | all autonomy nodes |
| `/sim/truth/odom` (Gazebo OdometryPublisher) | **EVALUATION ONLY** | evaluation_node, ground_tracker *sensor model*, scenario harness, dashboard (labelled) |
| `simulation/configs/world_ps05.yaml` | **EVALUATION ONLY** | world generator, evaluation (clearance/collisions) |

## 5. Package map

| Package | Node(s) | Purpose |
|---|---|---|
| `ps05_common` | — | config + scenario overrides, ENU/NED boundary conversions, topic classification, PointCloud2 numpy |
| `ps05_interfaces` | — | `TrackingMeasurement`, `VoxelGrid` msgs, `PlanPath` srv |
| `aeronav_description` | `static_frames` | TF tree from `configs/sensors.yaml` (+ optional extrinsic perturbation) |
| `aeronav_simulation` | `ground_tracker`, `scenario_events` | tracking-station sensor model; dynamic-object test harness |
| `camera_processor` | `camera_processor` | image validation, health, fault injection (blur/exposure/noise/offset/occlusion/blackout) |
| `lidar_processor` | `lidar_processor` | range/self filter, health, fault injection (noise/dropout/blackout) |
| `sensor_fusion` | `fusion_node` | explicit sync + TF2 + pinhole projection + RGB → `/fused/points` |
| `localization` | `localization_node` | spherical→Cartesian → EKF2 EV; EKF2 odometry → `/vehicle/odom` + TF |
| `mapping` | `mapping_node` | ray-traced log-odds 3D occupancy (FREE/UNKNOWN/OCCUPIED) |
| `planner_3d` | `planner_node` | inflation, corridor, 3D A*, pruning, smoothing, time parameterisation, validation |
| `obstacle_avoidance` | `avoidance_node` | look-ahead check vs live LiDAR and map |
| `landing_detector` | `detector_node` | AprilTag 36h11 detection + PnP + TF2 → target in map |
| `landing_validator` | `validator_node` | LiDAR plane fit, tilt/roughness, clearance, alignment |
| `landing_controller` | `mission_manager` | explicit mission/landing state machine, PID alignment |
| `px4_interface` | `px4_interface`, `smoke_flight` | offboard heartbeat/setpoints/commands; Phase-1 smoke flight |
| `evaluation` | `evaluation_node` | metrics vs ground truth, per-run records |
| `dashboard_bridge` | `dashboard_bridge` | read-only ROS → backend forwarder |
