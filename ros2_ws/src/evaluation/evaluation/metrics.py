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
        d = np.abs(_to_box_frame(p.reshape(1, 3), o)[0]) - h
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


def _to_box_frame(p: np.ndarray, o: dict) -> np.ndarray:
    """Points relative to the box centre, rotated into the box frame (yaw about z; scenario objects rotate)."""
    d = np.asarray(p, dtype=float) - np.asarray(o['pos'], dtype=float)
    yaw = float(o.get('yaw', 0.0))
    if yaw:
        c, s = np.cos(yaw), np.sin(yaw)
        d = np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], d[:, 2]])
    return d


def distances_to_primitive(p: np.ndarray, o: dict) -> np.ndarray:
    """Vectorised surface distance for points p (N,3) to one primitive (0 inside)."""
    c = np.asarray(o['pos'], dtype=float)
    if o['type'] == 'box':
        h = np.array(o['size'], dtype=float) / 2.0
        return np.linalg.norm(np.maximum(np.abs(_to_box_frame(p, o)) - h, 0.0), axis=1)
    if o['type'] == 'cylinder':
        dr = np.maximum(np.hypot(p[:, 0] - c[0], p[:, 1] - c[1]) - float(o['radius']), 0.0)
        dz = np.maximum(np.abs(p[:, 2] - c[2]) - float(o['length']) / 2.0, 0.0)
        return np.hypot(dr, dz)
    if o['type'] == 'sphere':
        return np.maximum(np.linalg.norm(p - c, axis=1) - float(o['radius']), 0.0)
    raise ValueError(o['type'])


def world_surfaces(spec: dict) -> list:
    """All LiDAR-visible geometry incl. ground (z=0 plane), landing pad and tracking-station mast."""
    prims = load_world(spec)
    pad = spec['landing_pad']
    prims.append({'name': 'landing_pad', 'type': 'box', 'pos': np.array([*pad['center'], pad['height'] / 2]),
                  'size': [*pad['size'], pad['height']]})
    st = spec['tracking_station']
    prims.append({'name': 'station_mast', 'type': 'cylinder', 'pos': np.array([st[0], st[1], st[2] / 2]),
                  'radius': 0.08, 'length': st[2]})
    prims.append({'name': 'station_head', 'type': 'sphere', 'pos': np.array(st, dtype=float), 'radius': 0.25})
    return prims


def map_accuracy(voxels: np.ndarray, prims: list, threshold: float = 0.5) -> dict:
    """Occupied voxel centres vs. true geometry: phantom = farther than threshold from every surface."""
    v = np.asarray(voxels, dtype=float).reshape(-1, 3)
    if len(v) == 0:
        return {'occupied': 0, 'phantom': 0, 'phantom_fraction': None, 'median_error_m': None, 'phantom_examples': []}
    d = np.abs(v[:, 2])                                  # ground plane
    for o in prims:
        d = np.minimum(d, distances_to_primitive(v, o))
    ph = d > threshold
    return {'occupied': int(len(v)), 'phantom': int(ph.sum()), 'phantom_fraction': float(ph.mean()),
            'median_error_m': float(np.median(d)), 'p95_error_m': float(np.percentile(d, 95)),
            'phantom_examples': np.round(v[ph][np.argsort(-d[ph])][:25], 2).tolist()}   # worst first
