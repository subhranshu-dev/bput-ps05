# Mapping, Inflation, Corridor, 3D A*, Trajectory and Local Replanning (Phases 8–13)

## 3D occupancy map (`mapping/voxel_map.py`)
* Bounded grid 50 × 50 × 12.5 m at 0.25 m (200 × 200 × 50 cells), log-odds per cell.
* Each return ray-traces free space (`miss = −0.4`) up to half a cell before the hit and marks the hit (`+0.85`);
  clamped to [−2, 3.5]. States: OCCUPIED ≥ 0.7, FREE ≤ −0.5, otherwise UNKNOWN.
* Ray tracing lets a moved obstacle be cleared when its former space is observed empty.
* Published at 2 Hz as `/occupancy_map/grid` (zlib-compressed uint8 FREE/UNKNOWN/OCCUPIED) and `/occupancy_map`
  (occupied voxel centres, coloured by fused RGB).

## Obstacle inflation (`planner_3d/grid.py`, `configs/safety.yaml`)
```
radius = vehicle_radius 0.40 + localization_uncertainty 0.15 + controller_error 0.15 + desired_clearance 0.25
       (+ 0.5 × planning cell = 0.25 for grid discretisation)            = 1.20 m on the 0.5 m planning grid
```
Map → planning grid by max-pooling (any occupied fine cell ⇒ occupied); inflation via Euclidean distance transform.
The live-LiDAR check uses 0.95 m (exact surface points, no discretisation term).

## Approved corridor (`planner_3d/corridor.py`, `configs/mission.yaml`)
Box x∈[1,49], y∈[1,49], z∈[1,10] m **and** within 12 m of the centreline
(5,5,3)→(20,8,4)→(33,22,4)→(41,41,5), shrunk by a 0.5 m safety margin. Encoded as a boolean mask on the
planning grid, so A* cannot expand outside it; every trajectory is re-validated against it.

## 3D A* (`planner_3d/astar.py`)
* 26-connected, `f = g + w·h`, Euclidean `h` (admissible, `w = 1`), vertical-move penalty 1.2.
* Unknown-cell policy: `optimistic` (traversable at 1.5× cost; default, because the map is built during flight and
  the replanner reacts to newly observed obstacles) or `pessimistic` (blocked).
* Start/goal snapping (≤ 4 cells) when they fall inside inflation; goal outside the corridor is rejected.
* Limits: 400 k expansions / 5 s. Failure reasons are explicit (`no path exists inside the approved mission space`,
  `goal outside approved mission corridor`, `search limit reached`).
* Line-of-sight pruning on the traversable mask (inflated + corridor).

## Trajectory (`planner_3d/trajectory.py`, `configs/planner.yaml`)
Chaikin corner cutting (3 passes, each accepted only if still traversable) → 0.5 m resampling → speed caps
(v ≤ 2 m/s, |vz| ≤ 1 m/s, curvature limit `v ≤ √(0.6 a_max / κ)`) → forward/backward acceleration passes
(|a| ≤ 1 m/s², zero speed at both ends) → time stamps; yaw follows the tangent with a 0.8 rad/s rate limit.
The mission manager follows it with a time-indexed carrot that **pauses when E > e_warn** (progress gating),
sending position + velocity-feed-forward setpoints to PX4.

## Final validation (`planner_3d/node.py::validate_trajectory`)
Densely sampled (0.125 m): not blocked on the inflated grid, inside the corridor, minimum clearance ≥ radius.
Failure ⇒ plan rejected (never flown).

## Local avoidance and replanning (`obstacle_avoidance`)
For the 8 m look-ahead of the active trajectory:
* **LiDAR check** on every scan: ≥ 3 returns within 0.95 m of the path ⇒ blocked.
* **Map check** on every grid: path sample inside an inflated cell ⇒ blocked.
Response (mission state machine): `NAVIGATE → OBSTACLE_STOP` (hold position, wait until speed < 0.3 m/s and ≥ 1 s
so the map integrates the obstacle) `→ REPLANNING` (A* from the current pose, same corridor) `→ NAVIGATE`.
No route ⇒ `NO_SAFE_PATH` (hover, 3 retries) ⇒ `ABORTED` (land in place).
Measured: detection-to-stop response, replanning latency, replans, validity of the new path.
