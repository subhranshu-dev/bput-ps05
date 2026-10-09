"""Planning grid: occupancy map down-sampling + obstacle inflation (Phase 9).

inflation radius = vehicle_radius + localization_uncertainty + controller_error
                   + desired_clearance (+ half a planning cell for discretisation)

All terms come from configs/safety.yaml; nothing is hard-coded in the planner.
"""
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

FREE, UNKNOWN, OCCUPIED = 0, 1, 2


def inflation_radius(safety_cfg: dict, planning_resolution: float) -> float:
    inf = safety_cfg['inflation']
    r = (float(inf['vehicle_radius']) + float(inf['localization_uncertainty'])
         + float(inf['controller_error']) + float(inf['desired_clearance']))
    if inf.get('grid_quantization', True):
        r += 0.5 * planning_resolution
    return r


@dataclass
class PlanningGrid:
    resolution: float
    origin: np.ndarray            # world coords of min corner of cell (0,0,0)
    occupied: np.ndarray          # bool (nx, ny, nz)
    unknown: np.ndarray           # bool
    clearance: np.ndarray         # float metres from cell centre to nearest occupied cell centre
    blocked: np.ndarray           # bool: occupied or inside inflation radius
    radius: float

    @property
    def shape(self):
        return self.occupied.shape

    def world_to_index(self, p):
        return np.floor((np.asarray(p, dtype=float) - self.origin) / self.resolution).astype(np.int64)

    def index_to_world(self, idx):
        return self.origin + (np.asarray(idx, dtype=float) + 0.5) * self.resolution

    def in_bounds(self, idx):
        idx = np.asarray(idx)
        return np.all((idx >= 0) & (idx < np.array(self.shape)), axis=-1)

    def is_blocked_world(self, pts) -> np.ndarray:
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        idx = self.world_to_index(pts)
        ok = self.in_bounds(idx)
        out = np.ones(len(pts), dtype=bool)
        if ok.any():
            i = idx[ok]
            out[ok] = self.blocked[i[:, 0], i[:, 1], i[:, 2]]
        return out

    def clearance_world(self, pts) -> np.ndarray:
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        idx = self.world_to_index(pts)
        ok = self.in_bounds(idx)
        out = np.zeros(len(pts))
        if ok.any():
            i = idx[ok]
            out[ok] = self.clearance[i[:, 0], i[:, 1], i[:, 2]]
        return out


def build_planning_grid(states: np.ndarray, map_resolution: float, map_origin, planning_resolution: float,
                        radius: float) -> PlanningGrid:
    """Max-pool the occupancy map to the planning resolution and inflate obstacles."""
    factor = int(round(planning_resolution / map_resolution))
    if factor < 1 or abs(factor * map_resolution - planning_resolution) > 1e-6:
        raise ValueError('planning resolution must be an integer multiple of the map resolution')
    nx, ny, nz = (s // factor for s in states.shape)
    s = states[: nx * factor, : ny * factor, : nz * factor]
    blocks = s.reshape(nx, factor, ny, factor, nz, factor)
    occupied = (blocks == OCCUPIED).any(axis=(1, 3, 5))
    unknown_frac = (blocks == UNKNOWN).mean(axis=(1, 3, 5))
    unknown = (~occupied) & (unknown_frac > 0.5)
    return inflate(occupied, unknown, planning_resolution, np.asarray(map_origin, dtype=float), radius)


def inflate(occupied: np.ndarray, unknown: np.ndarray, resolution: float, origin, radius: float) -> PlanningGrid:
    if occupied.any():
        clearance = ndimage.distance_transform_edt(~occupied) * resolution
    else:
        clearance = np.full(occupied.shape, np.inf)
    blocked = occupied | (clearance < radius)
    return PlanningGrid(resolution, np.asarray(origin, dtype=float), occupied, unknown, clearance, blocked, radius)


def build_local_grid(states: np.ndarray, map_resolution: float, map_origin, planning_resolution: float,
                     radius: float, lo_world, hi_world) -> PlanningGrid:
    """Planning grid restricted to the box [lo, hi] (+ inflation margin), aligned to the global coarse grid.

    Same semantics as build_planning_grid inside the box: blocks are max-pooled identically and the margin
    (>= radius + 1 cell) makes the distance transform exact for every cell within [lo, hi].
    """
    factor = int(round(planning_resolution / map_resolution))
    origin = np.asarray(map_origin, dtype=float)
    margin = radius + 2 * planning_resolution
    lo_c = np.floor((np.asarray(lo_world) - margin - origin) / planning_resolution).astype(int)
    hi_c = np.ceil((np.asarray(hi_world) + margin - origin) / planning_resolution).astype(int)
    n_c = np.array(states.shape) // factor
    lo_c = np.clip(lo_c, 0, n_c)
    hi_c = np.clip(hi_c, 0, n_c)
    if np.any(hi_c <= lo_c):
        lo_c, hi_c = np.zeros(3, int), n_c
    sub = states[lo_c[0] * factor:hi_c[0] * factor, lo_c[1] * factor:hi_c[1] * factor, lo_c[2] * factor:hi_c[2] * factor]
    return build_planning_grid(sub, map_resolution, origin + lo_c * planning_resolution, planning_resolution, radius)
