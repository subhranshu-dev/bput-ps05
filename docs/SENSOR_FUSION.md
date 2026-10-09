# Sensor Fusion (Phases 3–7)

## Sensors (generated into `simulation/models/x500_ps05/model.sdf` from `configs/sensors.yaml`)

| Sensor | Mount (base_link, FLU) | Spec | Topic / frame |
|---|---|---|---|
| Forward RGB camera | (0.12, 0, 0.05), level | 640×480, HFOV 80°, 10 Hz, σ=0.004 | `/camera/image_raw`, `camera_optical_frame` |
| 3D LiDAR | (0, 0, 0.20) | 360 × 16 beams (−179.5°…+179.5°, avoids gpu_lidar cube-map seams), ±25° vertical, 0.3–30 m, 10 Hz, σ=1 cm | `/lidar/points`, `lidar_frame` |
| Downward RGB camera | (0.10, 0, −0.06), pitched 90° down | 640×480, HFOV 60°, 15 Hz | `/down_camera/image_raw`, `down_camera_optical_frame` |
| Downward flash LiDAR | (−0.10, 0, −0.06), pitched 90° down | 32 × 32, 70°×70°, 0.1–15 m, 10 Hz, 0.35 m self-filter (landing-gear feet) | `/down_lidar/points`, `down_lidar_frame` |

The downward sensors are mounted fore/aft of the body centre so the landing-gear struts (y = ±0.1 m) stay out of view.

## TF2

All geometry goes through TF2. Mount poses are defined once (`configs/sensors.yaml`), used both to generate
the Gazebo model and to publish static TF, and `tests/unit/test_tf_model_consistency.py` checks they agree.
Optical frames use REP-103 (`rpy = (−π/2, 0, −π/2)` from the link). `scripts/check_system.py` validates at runtime:
all frames present, finite, static mounts not moving, and no vehicle pose jumps > 0.5 m between samples.

## Time synchronisation

`sensor_fusion/sync.py::TimestampSynchronizer` pairs each LiDAR scan with the nearest camera frame (queue 10)
and **rejects** pairs with `|t_cam − t_lidar| > max_delta_s` (0.05 s). Accepted/rejected counts, last/mean/max
|Δt| are published on `/fusion/status`. Tests cover perfect sync, a 17 ms offset (accepted), a 200 ms offset (rejected)
and out-of-order frames. The `timestamp_offset` scenarios inject +30 ms and +80 ms camera offsets in simulation.

## Projection

```
P_camera = T_camera_lidar · P_lidar        (T from TF2 at the LiDAR stamp)
u = fx · X/Z + cx,  v = fy · Y/Z + cy       (K from the live CameraInfo)
```
A point is coloured only if `Z > 0.3 m` and `(u, v)` is inside the image; others keep geometry with
`rgb = grey, in_fov = 0`. Output `/fused/points` fields: `x y z rgb in_fov`.

## Projection-error metrics

1. **Extrinsic shift (px)**: mean pixel displacement between projecting with the design extrinsic and with the
   TF actually in use — isolates the effect of a calibration perturbation (`extrinsic_perturbation` scenario).
2. **Edge-alignment chamfer (px)**: LiDAR depth discontinuities (object silhouettes) are projected into the image
   and their distance to the nearest Canny edge is averaged — an image-based check that needs no ground truth.

## Degraded mode

If the fused stream is stale for > 0.6 s (camera failure or sync rejection), `mapping_node` falls back to the raw
filtered LiDAR cloud and reports `degraded: true`; obstacle avoidance always uses raw LiDAR, so camera loss
never removes obstacle detection.
