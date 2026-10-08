"""Local obstacle check of the active trajectory (Phase 13).

Two independent checks, both restricted to the look-ahead window of the
trajectory in front of the vehicle:

1. live LiDAR returns (map frame) inside the inflated tube around the path
   -> catches newly appearing / moving obstacles within one scan;
2. the latest inflated planning grid -> catches obstacles that the map has
   accumulated (e.g. previously unknown space now observed occupied).
"""
import numpy as np
from scipy.spatial import cKDTree


def lookahead_segment(traj_pos: np.ndarray, current_index: int, lookahead_m: float) -> tuple:
    """Return (points, along_track_distance) of the trajectory ahead of current_index."""
    pts = traj_pos[current_index:]
    if len(pts) == 0:
        return pts, np.zeros(0)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    keep = s <= lookahead_m
    return pts[keep], s[keep]


def check_points(path_pts: np.ndarray, along: np.ndarray, obstacle_pts: np.ndarray, radius: float,
                 min_points: int = 3) -> dict:
    """Is any group of >= min_points LiDAR returns within ``radius`` of the upcoming path?"""
    res = {'blocked': False, 'hits': 0, 'distance_along_path': None, 'min_clearance': None,
           'obstacle_point': None}
    if len(path_pts) == 0 or obstacle_pts is None or len(obstacle_pts) == 0:
        return res
    tree = cKDTree(obstacle_pts)
    dist, idx = tree.query(path_pts, k=1)
    res['min_clearance'] = float(dist.min())
    near = tree.query_ball_point(path_pts, r=radius)
    hit_ids = set()
    first = None
    for i, ids in enumerate(near):
        if ids:
            hit_ids.update(ids)
            if first is None and len(hit_ids) >= min_points:
                first = i
    res['hits'] = len(hit_ids)
    if first is not None:
        res['blocked'] = True
        res['distance_along_path'] = float(along[first])
        res['obstacle_point'] = obstacle_pts[idx[first]].tolist()
    return res


def check_grid(path_pts: np.ndarray, along: np.ndarray, grid) -> dict:
    """Is the upcoming path inside a blocked (inflated) planning-grid cell?"""
    if len(path_pts) == 0 or grid is None:
        return {'blocked': False, 'distance_along_path': None}
    blocked = grid.is_blocked_world(path_pts)
    # ignore the first sample: the vehicle may itself be near an inflated edge after a replan
    blocked[0] = False
    if blocked.any():
        i = int(np.argmax(blocked))
        return {'blocked': True, 'distance_along_path': float(along[i]), 'point': path_pts[i].tolist()}
    return {'blocked': False, 'distance_along_path': None}
