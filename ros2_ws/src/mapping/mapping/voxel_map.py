"""Bounded 3D log-odds voxel occupancy map (Phase 8).

States: FREE = 0, UNKNOWN = 1, OCCUPIED = 2.

Each LiDAR return updates the map along its ray: cells traversed by the ray
receive a 'miss' (evidence of free space), the end cell receives a 'hit'. This
lets moving obstacles be cleared from the map when the space they occupied is
later observed empty. The map is independent of the planner: it only exposes
cell states and a compressed snapshot.
"""
import zlib

import numpy as np

FREE, UNKNOWN, OCCUPIED = 0, 1, 2


class VoxelMap:
    def __init__(self, resolution, bounds_x, bounds_y, bounds_z, hit=0.85, miss=-0.4,
                 l_min=-2.0, l_max=3.5, occupied_threshold=0.7, free_threshold=-0.5):
        self.res = float(resolution)
        self.origin = np.array([bounds_x[0], bounds_y[0], bounds_z[0]], dtype=float)
        self.shape = (
            int(round((bounds_x[1] - bounds_x[0]) / self.res)),
            int(round((bounds_y[1] - bounds_y[0]) / self.res)),
            int(round((bounds_z[1] - bounds_z[0]) / self.res)),
        )
        # Fortran order so the x-fastest flat index used for transport is a zero-copy view
        self.logodds = np.zeros(self.shape, dtype=np.float32, order='F')
        self.hit, self.miss = float(hit), float(miss)
        self.l_min, self.l_max = float(l_min), float(l_max)
        self.occ_t, self.free_t = float(occupied_threshold), float(free_threshold)
        self.updates = 0

    @classmethod
    def from_config(cls, cfg: dict):
        lo = cfg.get('log_odds', {})
        b = cfg['bounds']
        return cls(cfg['resolution'], b['x'], b['y'], b['z'],
                   lo.get('hit', 0.85), lo.get('miss', -0.4), lo.get('min', -2.0), lo.get('max', 3.5),
                   lo.get('occupied_threshold', 0.7), lo.get('free_threshold', -0.5))

    # ---------------------------------------------------------------- indexing
    def world_to_index(self, pts) -> np.ndarray:
        return np.floor((np.asarray(pts, dtype=float) - self.origin) / self.res).astype(np.int64)

    def index_to_world(self, idx) -> np.ndarray:
        return self.origin + (np.asarray(idx, dtype=float) + 0.5) * self.res

    def in_bounds(self, idx) -> np.ndarray:
        idx = np.asarray(idx)
        return np.all((idx >= 0) & (idx < np.array(self.shape)), axis=-1)

    def _flat(self, idx):
        nx, ny, _ = self.shape
        return idx[:, 0] + nx * (idx[:, 1] + ny * idx[:, 2])

    # ---------------------------------------------------------------- update
    def integrate(self, points_world, sensor_origin, max_range: float = 30.0):
        """Ray-trace a scan (points already in the map frame) from sensor_origin."""
        pts = np.asarray(points_world, dtype=float).reshape(-1, 3)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) == 0:
            return 0
        o = np.asarray(sensor_origin, dtype=float)
        d = pts - o
        dist = np.linalg.norm(d, axis=1)
        keep = dist > 1e-3
        pts, d, dist = pts[keep], d[keep], dist[keep]
        clipped = dist > max_range
        dirs = d / dist[:, None]
        end_dist = np.minimum(dist, max_range)

        # --- free-space samples along every ray (stop half a cell before the hit)
        step = self.res * 0.5
        n_steps = np.floor(np.maximum(end_dist - self.res * 0.5, 0.0) / step).astype(np.int64)
        total = int(n_steps.sum())
        logical = self.logodds.reshape(-1, order='F')  # zero-copy flat view matching _flat()
        free_flat = np.empty(0, dtype=np.int64)
        if total > 0:
            ray_id = np.repeat(np.arange(len(pts)), n_steps)
            starts = np.cumsum(n_steps) - n_steps
            k = np.arange(total) - np.repeat(starts, n_steps)
            samples = o + dirs[ray_id] * ((k + 0.5) * step)[:, None]
            idx = self.world_to_index(samples)
            ok = self.in_bounds(idx)
            free_flat = np.unique(self._flat(idx[ok]))

        # --- hits (only for returns within max_range)
        hit_idx = self.world_to_index(pts[~clipped])
        ok = self.in_bounds(hit_idx)
        hit_flat = np.unique(self._flat(hit_idx[ok]))
        if len(free_flat) and len(hit_flat):
            free_flat = np.setdiff1d(free_flat, hit_flat, assume_unique=True)

        if len(free_flat):
            logical[free_flat] = np.clip(logical[free_flat] + self.miss, self.l_min, self.l_max)
        if len(hit_flat):
            logical[hit_flat] = np.clip(logical[hit_flat] + self.hit, self.l_min, self.l_max)
        self.updates += 1
        return len(hit_flat)

    # ---------------------------------------------------------------- queries
    def states(self) -> np.ndarray:
        s = np.full(self.shape, UNKNOWN, dtype=np.uint8)
        s[self.logodds >= self.occ_t] = OCCUPIED
        s[self.logodds <= self.free_t] = FREE
        return s

    def state_at(self, p) -> int:
        idx = self.world_to_index(np.asarray(p).reshape(1, 3))
        if not self.in_bounds(idx)[0]:
            return UNKNOWN
        i, j, k = idx[0]
        lo = self.logodds[i, j, k]
        return OCCUPIED if lo >= self.occ_t else (FREE if lo <= self.free_t else UNKNOWN)

    def occupied_centers(self) -> np.ndarray:
        idx = np.argwhere(self.logodds >= self.occ_t)
        return self.index_to_world(idx)

    def counts(self) -> dict:
        s = self.states()
        return {'free': int((s == FREE).sum()), 'unknown': int((s == UNKNOWN).sum()),
                'occupied': int((s == OCCUPIED).sum())}


# -------------------------------------------------------------------- transport
def encode_states(states: np.ndarray) -> bytes:
    """Flatten x-fastest (index = ix + nx*(iy + ny*iz)) and zlib-compress."""
    return zlib.compress(np.ascontiguousarray(states.reshape(-1, order='F')).tobytes(), 1)


def decode_states(data: bytes, nx: int, ny: int, nz: int) -> np.ndarray:
    flat = np.frombuffer(zlib.decompress(bytes(data)), dtype=np.uint8)
    if flat.size != nx * ny * nz:
        raise ValueError(f'voxel grid size mismatch: {flat.size} != {nx * ny * nz}')
    return flat.reshape((nx, ny, nz), order='F')
