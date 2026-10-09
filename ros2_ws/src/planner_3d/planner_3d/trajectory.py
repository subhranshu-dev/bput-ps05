"""Waypoints -> smooth, time-parameterised trajectory (Phase 12).

1. Chaikin corner cutting (each pass is accepted only if the smoothed polyline
   remains traversable on the inflated grid and inside the corridor).
2. Uniform resampling at ``waypoint_spacing``.
3. Speed profile: v <= v_max (horizontal), |vz| <= v_max_z, curvature limit
   v <= sqrt(a_lat / kappa), then forward/backward passes enforcing
   |dv/dt| <= a_max with zero speed at both ends.
4. Yaw follows the path tangent with a yaw-rate limit (or stays fixed).
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Trajectory:
    t: np.ndarray        # (N,) seconds from trajectory start
    pos: np.ndarray      # (N,3)
    vel: np.ndarray      # (N,3)
    yaw: np.ndarray      # (N,)  ENU yaw
    speed: np.ndarray    # (N,)
    polyline: np.ndarray = None   # validated geometric polyline; every pos sample lies on its segments

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0

    def sample(self, t: float):
        """Interpolated (pos, vel, yaw) at time t (clamped)."""
        if len(self.t) == 1 or t <= 0:
            return self.pos[0].copy(), np.zeros(3), float(self.yaw[0])
        if t >= self.t[-1]:
            return self.pos[-1].copy(), np.zeros(3), float(self.yaw[-1])
        i = int(np.searchsorted(self.t, t) - 1)
        i = max(0, min(i, len(self.t) - 2))
        a = (t - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9)
        pos = self.pos[i] + a * (self.pos[i + 1] - self.pos[i])
        vel = self.vel[i] + a * (self.vel[i + 1] - self.vel[i])
        dy = math.atan2(math.sin(self.yaw[i + 1] - self.yaw[i]), math.cos(self.yaw[i + 1] - self.yaw[i]))
        return pos, vel, float(self.yaw[i] + a * dy)

    def closest_index(self, p) -> int:
        return int(np.argmin(np.linalg.norm(self.pos - np.asarray(p), axis=1)))

    def length(self) -> float:
        return float(np.linalg.norm(np.diff(self.pos, axis=0), axis=1).sum()) if len(self.pos) > 1 else 0.0


def chaikin(pts: np.ndarray) -> np.ndarray:
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        out.append(0.75 * a + 0.25 * b)
        out.append(0.25 * a + 0.75 * b)
    out.append(pts[-1])
    return np.array(out)


def resample(pts: np.ndarray, spacing: float) -> np.ndarray:
    """Resample each segment at <= spacing, keeping every input vertex.

    Keeping the vertices means the flown polyline is exactly the one that was
    validated against the inflated grid (uniform whole-path resampling would
    cut validated corners).
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return pts.copy()
    out = [pts[:1]]
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(int(math.ceil(np.linalg.norm(b - a) / spacing)), 1)
        t = np.linspace(0.0, 1.0, n + 1)[1:, None]
        out.append(a + t * (b - a))
    res = np.vstack(out)
    keep = np.concatenate([[True], np.linalg.norm(np.diff(res, axis=0), axis=1) > 1e-9])
    return res[keep]


def curvature(pts: np.ndarray) -> np.ndarray:
    """Discrete curvature (1/m) via the circumscribed circle of consecutive triples."""
    k = np.zeros(len(pts))
    if len(pts) < 3:
        return k
    a, b, c = pts[:-2], pts[1:-1], pts[2:]
    ab = np.linalg.norm(b - a, axis=1)
    bc = np.linalg.norm(c - b, axis=1)
    ca = np.linalg.norm(a - c, axis=1)
    cross = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    denom = ab * bc * ca
    k[1:-1] = np.where(denom > 1e-9, 2.0 * cross / np.maximum(denom, 1e-12), 0.0)
    return k


def smooth_waypoints(wps: np.ndarray, iterations: int, is_valid) -> np.ndarray:
    """Chaikin smoothing, accepting a pass only if ``is_valid(polyline)`` holds."""
    cur = np.asarray(wps, dtype=float)
    for _ in range(int(iterations)):
        cand = chaikin(cur)
        if is_valid(cand):
            cur = cand
        else:
            break
    return cur


def time_parameterize(pts: np.ndarray, cfg: dict, start_yaw: float | None = None) -> Trajectory:
    v_max = float(cfg.get('v_max', 2.0))
    a_max = float(cfg.get('a_max', 1.0))
    vz_max = float(cfg.get('v_max_z', 1.0))
    corner = float(cfg.get('corner_speed_factor', 0.6))
    yaw_mode = cfg.get('yaw_mode', 'path_tangent')
    yaw_rate = float(cfg.get('yaw_rate_max', 0.8))

    n = len(pts)
    if n == 1:
        y = start_yaw or 0.0
        return Trajectory(np.zeros(1), pts.copy(), np.zeros((1, 3)), np.array([y]), np.zeros(1))
    seg = np.diff(pts, axis=0)
    ds = np.linalg.norm(seg, axis=1)
    dirs = seg / np.maximum(ds[:, None], 1e-9)

    # per-point speed caps
    vcap = np.full(n, v_max)
    # vertical speed limit on the segment leaving each point
    vz_frac = np.abs(dirs[:, 2])
    seg_cap = np.where(vz_frac > 1e-6, np.minimum(v_max, vz_max / np.maximum(vz_frac, 1e-6)), v_max)
    vcap[:-1] = np.minimum(vcap[:-1], seg_cap)
    vcap[1:] = np.minimum(vcap[1:], seg_cap)
    kap = curvature(pts)
    a_lat = a_max * corner
    vcap = np.minimum(vcap, np.where(kap > 1e-6, np.sqrt(a_lat / np.maximum(kap, 1e-6)), v_max))
    vcap[0] = 0.0
    vcap[-1] = 0.0

    v = vcap.copy()
    for i in range(1, n):            # forward: acceleration limit
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_max * ds[i - 1]))
    for i in range(n - 2, -1, -1):   # backward: deceleration limit
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_max * ds[i]))

    t = np.zeros(n)
    for i in range(1, n):
        vavg = 0.5 * (v[i] + v[i - 1])
        t[i] = t[i - 1] + (ds[i - 1] / vavg if vavg > 1e-6 else 2.0 * ds[i - 1] / max(math.sqrt(2 * a_max * ds[i - 1]), 1e-3))

    vel = np.zeros((n, 3))
    vel[:-1] = dirs * v[:-1, None]
    vel[1:-1] = 0.5 * (dirs[:-1] + dirs[1:]) * v[1:-1, None]
    vel[-1] = 0.0

    yaw = np.zeros(n)
    y0 = start_yaw if start_yaw is not None else math.atan2(dirs[0, 1], dirs[0, 0])
    if yaw_mode == 'fixed':
        yaw[:] = y0
    else:
        yaw[0] = y0
        for i in range(1, n):
            d = dirs[min(i, n - 2)]
            desired = math.atan2(d[1], d[0]) if np.hypot(d[0], d[1]) > 0.1 else yaw[i - 1]
            err = math.atan2(math.sin(desired - yaw[i - 1]), math.cos(desired - yaw[i - 1]))
            dt = max(t[i] - t[i - 1], 1e-3)
            err = max(-yaw_rate * dt, min(yaw_rate * dt, err))
            yaw[i] = yaw[i - 1] + err
    return Trajectory(t, pts.copy(), vel, yaw, v)


def build_trajectory(waypoints: np.ndarray, cfg: dict, is_valid=lambda p: True,
                     start_yaw: float | None = None) -> Trajectory:
    smoothed = smooth_waypoints(waypoints, cfg.get('smoothing_iterations', 3), is_valid)
    dense = resample(smoothed, float(cfg.get('waypoint_spacing', 0.5)))
    traj = time_parameterize(dense, cfg, start_yaw)
    traj.polyline = smoothed
    return traj
