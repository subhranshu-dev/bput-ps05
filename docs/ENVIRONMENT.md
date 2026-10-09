# Environment (version-locked)

## Why a container

The development host runs **Ubuntu 26.04.1 / Python 3.14**, which is not a supported platform for PX4 or ROS 2.
PX4 v1.17 documents Ubuntu 22.04/24.04 with Gazebo Harmonic; Gazebo recommends ROS 2 Jazzy for Harmonic on 24.04.
The whole stack therefore runs in one pinned Ubuntu 24.04 image built from `docker/Dockerfile`.

## Host (recorded 2026-10-09)

| Item | Value |
|---|---|
| OS / kernel | Ubuntu 26.04.1 LTS, Linux 7.0.0-34-generic |
| CPU / RAM | 12 logical cores, 14.7 GiB |
| GPU used for sensor rendering | Intel Alder Lake-P UHD (`/dev/dri/card1`, `renderD128`), Mesa |
| GPU **not** passed to the container | NVIDIA RTX 2050 (no NVIDIA container runtime; its DRI node breaks headless EGL) |
| Docker | 29.1.3 |
| Node.js / npm (frontend build) | 22.22.1 / 9.2.0 |

## Container image `bput-ps05:jazzy-harmonic-px4v1.17.0`

| Component | Version | Pin |
|---|---|---|
| Base | `ros:jazzy-ros-base-noble` → Ubuntu 24.04.5 LTS | image digest `sha256:066420e0…` (build log) |
| ROS 2 | Jazzy (rclpy 7.1.12, ros_gz_bridge 1.0.24) | apt, packages.ros.org |
| Gazebo | Harmonic, gz-sim **8.15.0** | apt `gz-harmonic`, packages.osrfoundation.org |
| PX4-Autopilot | **v1.17.0** (`make px4_sitl`) | git tag |
| PX4 gz models | PX4-gazebo-models `b6127f4e` | PX4 v1.17.0 submodule |
| Micro XRCE-DDS Agent | **v2.4.3** | git tag |
| px4_msgs | branch release/1.17 @ **86d8239e** | git commit |
| Python | 3.12.3 | Ubuntu |
| OpenCV | 4.6.0 (with `aruco`, AprilTag 36h11 dictionary) | Ubuntu `python3-opencv` |
| NumPy / SciPy / Matplotlib | 1.26.4 / 1.11.4 / 3.6.3 | Ubuntu |
| GCC / CMake | 13.3.0 / 3.28.3 | Ubuntu |
| FastAPI / Uvicorn / Pydantic / httpx | 0.115.6 / 0.32.1 / 2.10.3 / 0.28.1 | `docker/requirements-container.txt` |

Frontend (host build, `frontend/package.json`, exact pins): React 18.3.1, Vite 5.4.11, TypeScript 5.6.3,
Tailwind 3.4.17, three 0.170.0, @react-three/fiber 8.17.10, @react-three/drei 9.117.3, Recharts 2.15.0.

## Install / build / launch

```bash
# 1. image (≈ 40 min the first time: PX4 + XRCE agent + px4_msgs are compiled)
docker build -f docker/Dockerfile -t bput-ps05:jazzy-harmonic-px4v1.17.0 .
# 2. frontend (host)
cd frontend && npm ci && npm run build && cd ..
# 3. generate model/world + colcon build (runs inside the container automatically)
./scripts/build.sh
# 4. tests / smoke / demo
./scripts/run_tests.sh            # unit + regression + backend + static GPS audit
./scripts/smoke_test.sh           # PX4 + Gazebo + ROS 2 + sensors + TF + takeoff/hover/land
./scripts/run_demo.sh             # full mission + dashboard on http://localhost:8000
```
`scripts/lib.sh` starts the container (`--network host --ipc host`, Mesa DRI nodes only, repo mounted at `/workspace`)
and every script re-executes itself inside it.

## Compatibility notes found during bring-up (all fixed in the repo)

1. **Headless EGL picked the NVIDIA node** → OGRE2 "OpenGL 3.3 is not supported" → sensor thread segfault when the
   vehicle spawned. Fix: only pass Mesa-capable DRI nodes into the container (`scripts/lib.sh`).
2. **PX4 standalone gz spawn path**: `px4-rc.gzsim` spawns `file://$PX4_GZ_MODELS/<model>/model.sdf`, and
   `PX4_GZ_MODELS` is only set by `gz_env.sh` (not sourced in standalone mode) → set explicitly in the launch file.
3. **Ground truth leaking into EKF2**: PX4's `gz_bridge` subscribes to `/model/<name>/odometry_with_covariance` and
   republishes it as `vehicle_visual_odometry`. The evaluation-only OdometryPublisher published there by default.
   Fix: `<odom_covariance_topic>/sim/truth/odom_with_covariance</odom_covariance_topic>`; guarded by static and runtime
   checks in `scripts/gps_audit.py`.
4. **uXRCE-DDS timesync vs. simulation time**: the agent syncs to wall clock while ROS nodes stamp in sim time, so
   external-vision samples looked ~1 s stale and EKF2 rejected them. In lockstep SITL PX4 time = Gazebo time = `/clock`,
   so `UXRCE_DDS_SYNCT=0` in the PS05 airframe.
5. **PX4 under `ros2 launch`** needs `-d` (daemon, no interactive pxh shell).
6. **x500 spawn height**: base_link is 0.24 m above the feet → spawn at z = 0.25 m.
7. `pytest-asyncio` pulled a pytest/pluggy upgrade that pip cannot apply over Debian's packages → removed (not needed).
8. Gazebo Classic is not used anywhere.
