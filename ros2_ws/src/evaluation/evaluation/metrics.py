"""Evaluation metrics (Phase 19). EVALUATION ONLY — uses simulator ground truth and the
world geometry spec; never imported by autonomy packages.
"""
import math

import numpy as np


# ------------------------------------------------------------------ world geometry
def load_world(spec: dict) -> list:
    prims = []
    for o in list(spec.get('obstacles', [])) + list(spec.get('dynamic', [])):
        prims.append(dict(o, pos=np.array(o['pos'], dtype=float)))
    return prims


def distance_to_primitive(p: np.ndarray, o: dict) -> float:
    """Euclidean distance from point p to the primitive surface (0 if inside)."""
    c = o['pos']
    if o['type'] == 'box':
        h = np.array(o['size'], dtype=float) / 2.0
        d = np.abs(p - c) - h
        return float(np.linalg.norm(np.maximum(d, 0.0)))
    if o['type'] == 'cylinder':
        r, hl = float(o['radius']), float(o['length']) / 2.0
        dr = max(math.hypot(p[0] - c[0], p[1] - c[1]) - r, 0.0)
        dz = max(abs(p[2] - c[2]) - hl, 0.0)
        return math.hypot(dr, dz)
    if o['type'] == 'sphere':
        return max(float(np.linalg.norm(p - c)) - float(o['radius']), 0.0)
    raise ValueError(o['type'])


def min_clearance(p, prims, ignore=()) -> tuple:
    best, name = float('inf'), None
    for o in prims:
        if o['name'] in ignore:
            continue
        d = distance_to_primitive(np.asarray(p, dtype=float), o)
        if d < best:
            best, name = d, o['name']
    return best, name


# ------------------------------------------------------------------ statistics
def error_stats(errors) -> dict:
    e = np.asarray([x for x in errors if x is not None and np.isfinite(x)], dtype=float)
    if len(e) == 0:
        return {'n': 0, 'mean': None, 'median': None, 'rmse': None, 'p95': None, 'max': None, 'std': None}
    return {'n': int(len(e)), 'mean': float(e.mean()), 'median': float(np.median(e)),
            'rmse': float(np.sqrt(np.mean(e ** 2))), 'p95': float(np.percentile(e, 95)),
            'max': float(e.max()), 'std': float(e.std())}


def landing_error(touchdown_xy, target_xy) -> float:
    """E = sqrt((Px - Tx)^2 + (Py - Ty)^2)."""
    return float(math.hypot(touchdown_xy[0] - target_xy[0], touchdown_xy[1] - target_xy[1]))


def path_length(pts) -> float:
    pts = np.asarray(pts, dtype=float)
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()) if len(pts) > 1 else 0.0


def interpolate_track(times, values, t):
    """Linear interpolation of an (N,3) track at time t (None outside range)."""
    if len(times) < 2 or t < times[0] or t > times[-1]:
        return None
    i = int(np.searchsorted(times, t))
    i = min(max(i, 1), len(times) - 1)
    a = (t - times[i - 1]) / max(times[i] - times[i - 1], 1e-9)
    return values[i - 1] + a * (values[i] - values[i - 1])
