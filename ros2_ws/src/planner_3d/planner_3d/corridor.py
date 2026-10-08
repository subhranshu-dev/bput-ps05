"""Approved Mission Space / permissible corridor (Phase 10).

A point is inside the approved space if it lies inside the mission box (shrunk
by the safety margin) AND (if enabled) within ``max_deviation - safety_margin``
of the approved centreline polyline. Every candidate path is validated against
this before it may be flown; the local replanner cannot escape it.
"""
import numpy as np


def point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance from points p (N,3) to segment ab."""
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return np.linalg.norm(p - a, axis=1)
    t = np.clip(((p - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(p - proj, axis=1)


class Corridor:
    def __init__(self, x, y, z, centerline=None, max_deviation=None, safety_margin=0.0, enabled=True):
        self.lo = np.array([x[0], y[0], z[0]], dtype=float)
        self.hi = np.array([x[1], y[1], z[1]], dtype=float)
        self.centerline = np.asarray(centerline, dtype=float) if centerline is not None else None
        self.max_dev = float(max_deviation) if max_deviation is not None else None
        self.margin = float(safety_margin)
        self.tube_enabled = bool(enabled and self.centerline is not None and len(self.centerline) >= 2)

    @classmethod
    def from_config(cls, mission_cfg: dict):
        ms = mission_cfg['mission_space']
        c = mission_cfg.get('corridor', {}) or {}
        return cls(ms['x'], ms['y'], ms['z'], c.get('centerline'), c.get('max_deviation'),
                   c.get('safety_margin', 0.0), c.get('enabled', True))

    def centerline_distance(self, pts) -> np.ndarray:
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        if not self.tube_enabled:
            return np.zeros(len(pts))
        d = np.full(len(pts), np.inf)
        for a, b in zip(self.centerline[:-1], self.centerline[1:]):
            d = np.minimum(d, point_segment_distance(pts, a, b))
        return d

    def contains(self, pts) -> np.ndarray:
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        inside = np.all((pts >= self.lo + self.margin) & (pts <= self.hi - self.margin), axis=1)
        if self.tube_enabled:
            inside &= self.centerline_distance(pts) <= (self.max_dev - self.margin)
        return inside

    def validate_path(self, pts, sample_step: float = 0.25) -> dict:
        """Densely sample a polyline and check every sample is inside the approved space."""
        dense = densify(np.asarray(pts, dtype=float).reshape(-1, 3), sample_step)
        inside = self.contains(dense)
        bad = np.flatnonzero(~inside)
        return {
            'ok': bool(len(bad) == 0),
            'violations': int(len(bad)),
            'first_violation': dense[bad[0]].tolist() if len(bad) else None,
            'max_centerline_deviation': float(self.centerline_distance(dense).max()) if len(dense) else 0.0,
        }

    def grid_mask(self, shape, origin, resolution) -> np.ndarray:
        """Boolean mask over a planning grid: True where the cell centre is approved."""
        nx, ny, nz = shape
        ix, iy, iz = np.meshgrid(np.arange(nx), np.arange(ny), np.arange(nz), indexing='ij')
        centers = np.stack([ix, iy, iz], axis=-1).reshape(-1, 3) * resolution + np.asarray(origin) + resolution / 2
        return self.contains(centers).reshape(shape)

    def to_dict(self) -> dict:
        return {'lo': self.lo.tolist(), 'hi': self.hi.tolist(),
                'centerline': self.centerline.tolist() if self.centerline is not None else None,
                'max_deviation': self.max_dev, 'safety_margin': self.margin, 'tube_enabled': self.tube_enabled}


def densify(pts: np.ndarray, step: float) -> np.ndarray:
    if len(pts) < 2:
        return pts
    out = [pts[:1]]
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(int(np.ceil(np.linalg.norm(b - a) / step)), 1)
        t = np.linspace(0.0, 1.0, n + 1)[1:, None]
        out.append(a + t * (b - a))
    return np.vstack(out)
