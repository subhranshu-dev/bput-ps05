#!/usr/bin/env python3
"""Offline planner benchmark/profiler (test tooling, not autonomy).

Rasterises the world spec into a fully-known occupancy grid (this is a *test
fixture*, the live planner only ever sees the LiDAR-built map), then times the
pipeline stages for the mission start -> landing-zone query.
"""
import argparse
import cProfile
import json
import pstats
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT / 'ros2_ws/src').iterdir():
    sys.path.insert(0, str(p))

from evaluation.metrics import distance_to_primitive, load_world  # noqa: E402
from planner_3d.astar import AStarPlanner  # noqa: E402
from planner_3d.corridor import Corridor  # noqa: E402
from planner_3d.grid import build_planning_grid, inflation_radius  # noqa: E402
from planner_3d.trajectory import build_trajectory  # noqa: E402


def rasterize(spec, res=0.25, bounds=((0, 50), (0, 50), (-0.5, 12.0))):
    shape = tuple(int(round((b[1] - b[0]) / res)) for b in bounds)
    origin = np.array([b[0] for b in bounds])
    states = np.zeros(shape, np.uint8)            # FREE
    ix, iy, iz = np.meshgrid(*[np.arange(n) for n in shape], indexing='ij')
    centers = np.stack([ix, iy, iz], -1).reshape(-1, 3) * res + origin + res / 2
    occ = centers[:, 2] < 0.0                      # ground
    for o in load_world(spec):
        if o['name'] in ('dyn_panel', 'unsafe_crate', 'pad_pole'):
            continue
        c = o['pos']
        r = max(np.max(o.get('size', [0, 0, 0])) / 2, o.get('radius', 0), o.get('length', 0) / 2) + res
        near = np.all(np.abs(centers - c) <= r, axis=1)
        idx = np.flatnonzero(near)
        d = np.array([distance_to_primitive(centers[i], o) for i in idx])
        occ[idx[d <= res / 2]] = True
    states.reshape(-1)[occ] = 2
    return states, origin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', action='store_true')
    ap.add_argument('--w', type=float, default=None, help='override heuristic weight')
    ap.add_argument('--unknown-cost', type=float, default=None)
    ap.add_argument('--unknown-beyond', type=float, default=0.0,
                    help='mark free cells farther than this (m) from the start UNKNOWN (in-flight condition)')
    a = ap.parse_args()
    spec = yaml.safe_load(open(ROOT / 'simulation/configs/world_ps05.yaml'))
    pcfg = yaml.safe_load(open(ROOT / 'configs/planner.yaml'))
    scfg = yaml.safe_load(open(ROOT / 'configs/safety.yaml'))
    mcfg = yaml.safe_load(open(ROOT / 'configs/mission.yaml'))
    t = time.perf_counter()
    states, origin = rasterize(spec)
    t_r = time.perf_counter() - t
    if a.unknown_beyond > 0:
        ix, iy, iz = np.meshgrid(*[np.arange(n) for n in states.shape], indexing='ij')
        c = np.stack([ix, iy, iz], -1) * 0.25 + origin + 0.125
        far = np.linalg.norm(c[..., :2] - np.array([5.0, 5.0]), axis=-1) > a.unknown_beyond
        states[far] = 1          # far space (incl. its obstacles) not yet observed
    res = pcfg['astar']['resolution']
    radius = inflation_radius(scfg, res)
    t = time.perf_counter()
    grid = build_planning_grid(states, 0.25, origin, res, radius)
    t_g = time.perf_counter() - t
    cor = Corridor.from_config(mcfg)
    t = time.perf_counter()
    mask = cor.grid_mask(grid.shape, grid.origin, grid.resolution)
    t_m = time.perf_counter() - t
    if a.w is not None:
        pcfg['astar']['heuristic_weight'] = a.w
    if a.unknown_cost is not None:
        pcfg['astar']['unknown_cost'] = a.unknown_cost
    pcfg['astar']['timeout_s'] = 120
    pcfg['astar']['max_expansions'] = 5_000_000
    planner = AStarPlanner(pcfg)
    start = np.array([5.0, 5.0, 2.5])
    goal = np.array([*mcfg['landing_zone']['center'], mcfg['landing_zone']['approach_altitude']])
    prof = cProfile.Profile() if a.profile else None
    t = time.perf_counter()
    if prof:
        prof.enable()
    r = planner.plan(grid, start, goal, cor, mask)
    if prof:
        prof.disable()
    t_a = time.perf_counter() - t
    t = time.perf_counter()
    traj = build_trajectory(r.waypoints, pcfg['trajectory']) if r.success else None
    t_t = time.perf_counter() - t
    out = {'rasterize_s': t_r, 'grid_build_ms': t_g * 1e3, 'corridor_mask_ms': t_m * 1e3, 'astar_ms': t_a * 1e3,
           'trajectory_ms': t_t * 1e3, 'success': r.success, 'expansions': r.expansions, 'length_m': r.length_m,
           'waypoints': len(r.waypoints) if r.success else 0, 'traj_duration_s': traj.duration if traj else None,
           'grid_shape': list(grid.shape)}
    print(json.dumps(out, indent=1))
    if prof:
        pstats.Stats(prof).sort_stats('cumulative').print_stats(12)


if __name__ == '__main__':
    main()
