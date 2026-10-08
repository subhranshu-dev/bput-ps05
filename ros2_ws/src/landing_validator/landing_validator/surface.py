"""LiDAR landing-surface validation (Phase 16).

A detected marker alone never triggers landing. Before (and during) descent:
  1. ground plane exists under the target (enough returns inside pad_radius)
  2. plane is level (tilt <= max_tilt_deg) and flat (rms <= max_plane_rms)
  3. vehicle height above the plane is plausible
  4. no obstruction: no returns > obstacle_height above the plane within clearance_radius
  5. horizontal alignment target-vs-vehicle within max_alignment_error
All points are in the map frame (z up).
"""
import math

import numpy as np


def fit_plane(pts: np.ndarray, iters: int = 3, inlier_thresh: float = 0.05):
    """Least-squares plane with iterative inlier re-selection. Returns (normal, d, rms, inliers)."""
    use = np.ones(len(pts), dtype=bool)
    normal, d, rms = np.array([0.0, 0.0, 1.0]), 0.0, float('inf')
    for _ in range(iters):
        p = pts[use]
        if len(p) < 3:
            break
        c = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p - c, full_matrices=False)
        normal = vt[-1]
        if normal[2] < 0:
            normal = -normal
        d = -float(normal @ c)
        resid = pts @ normal + d
        rms = float(np.sqrt(np.mean((p @ normal + d) ** 2)))
        use = np.abs(resid) < max(inlier_thresh, 2.5 * rms)
    return normal, d, rms, use


def plane_z(normal, d, x, y):
    return -(normal[0] * x + normal[1] * y + d) / max(normal[2], 1e-6)


def validate(points_map: np.ndarray, target_xy, vehicle_pos, cfg: dict) -> dict:
    pad_r = float(cfg.get('pad_radius', 1.2))
    clr_r = float(cfg.get('clearance_radius', 2.0))
    max_tilt = float(cfg.get('max_tilt_deg', 8.0))
    max_rms = float(cfg.get('max_plane_rms', 0.05))
    obs_h = float(cfg.get('obstacle_height', 0.15))
    min_pts = int(cfg.get('min_points', 30))
    max_align = float(cfg.get('max_alignment_error', 0.35))

    out = {'safe': False, 'reasons': [], 'ground_points': 0, 'tilt_deg': None, 'plane_rms': None,
           'height_above_ground': None, 'obstruction_points': 0, 'max_obstruction_height': 0.0,
           'alignment_error': None, 'ground_z': None}
    pts = np.asarray(points_map, dtype=float).reshape(-1, 3)
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    tx, ty = float(target_xy[0]), float(target_xy[1])
    vx, vy, vz = (float(v) for v in vehicle_pos)
    out['alignment_error'] = math.hypot(vx - tx, vy - ty)

    rxy = np.hypot(pts[:, 0] - tx, pts[:, 1] - ty) if len(pts) else np.zeros(0)
    # candidate ground: inside pad disc and below the vehicle
    ground = pts[(rxy <= pad_r) & (pts[:, 2] < vz - 0.2)] if len(pts) else pts
    out['ground_points'] = int(len(ground))
    if len(ground) < min_pts:
        out['reasons'].append('NO_GROUND_PLANE')
        return out
    # fit to the lowest dominant layer so a box on the pad does not bias the plane
    zlow = np.percentile(ground[:, 2], 20)
    base = ground[ground[:, 2] < zlow + 0.3]
    if len(base) < 3:
        base = ground
    normal, d, rms, _ = fit_plane(base)
    tilt = math.degrees(math.acos(min(1.0, abs(normal[2]))))
    out['tilt_deg'] = tilt
    out['plane_rms'] = rms
    gz = plane_z(normal, d, tx, ty)
    out['ground_z'] = float(gz)
    out['height_above_ground'] = float(vz - plane_z(normal, d, vx, vy))
    if tilt > max_tilt:
        out['reasons'].append('SURFACE_TILTED')
    if rms > max_rms:
        out['reasons'].append('SURFACE_ROUGH')

    near = pts[(rxy <= clr_r) & (pts[:, 2] < vz - 0.15)]
    if len(near):
        h = near[:, 2] - plane_z(normal, d, near[:, 0], near[:, 1])
        obstr = h > obs_h
        out['obstruction_points'] = int(obstr.sum())
        out['max_obstruction_height'] = float(h.max())
        if obstr.sum() >= 3:
            out['reasons'].append('OBSTRUCTION')
    if out['height_above_ground'] is not None and out['height_above_ground'] < -0.1:
        out['reasons'].append('HEIGHT_INVALID')
    if out['alignment_error'] > max_align:
        out['reasons'].append('MISALIGNED')
    # MISALIGNED is not a surface problem; report separately
    surface_ok = not any(r in out['reasons'] for r in ('NO_GROUND_PLANE', 'SURFACE_TILTED', 'SURFACE_ROUGH',
                                                        'OBSTRUCTION', 'HEIGHT_INVALID'))
    out['surface_ok'] = surface_ok
    out['aligned'] = out['alignment_error'] <= max_align
    out['safe'] = surface_ok and out['aligned']
    return out
