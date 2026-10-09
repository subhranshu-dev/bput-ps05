"""3D A* on the inflated planning grid (Phase 11).

    f(n) = g(n) + w * h(n)
    g(n): accumulated cost (Euclidean step length x vertical/unknown penalties)
    h(n): Euclidean distance to the goal (admissible for w = 1)

Traversable cell = not blocked (occupied/inflated) AND inside the approved
corridor AND (unknown_policy == optimistic OR not unknown).
"""
import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .corridor import Corridor, densify
from .grid import PlanningGrid


@dataclass
class PlanResult:
    success: bool
    message: str
    cells: list = field(default_factory=list)
    waypoints: np.ndarray = None       # world coords, after pruning
    raw_waypoints: np.ndarray = None   # world coords, all A* cells
    expansions: int = 0
    latency_ms: float = 0.0
    length_m: float = 0.0
    start_snapped: bool = False
    goal_snapped: bool = False


def _offsets(connectivity: int):
    offs = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                n = abs(dx) + abs(dy) + abs(dz)
                if n == 0:
                    continue
                if connectivity == 6 and n > 1:
                    continue
                if connectivity == 18 and n > 2:
                    continue
                offs.append((dx, dy, dz))
    return offs


def path_length(pts) -> float:
    pts = np.asarray(pts, dtype=float)
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()) if len(pts) > 1 else 0.0


class AStarPlanner:
    def __init__(self, cfg: dict):
        a = cfg['astar']
        self.connectivity = int(a.get('connectivity', 26))
        if self.connectivity not in (6, 18, 26):
            raise ValueError('connectivity must be 6, 18 or 26')
        self.unknown_policy = a.get('unknown_policy', 'optimistic')
        if self.unknown_policy not in ('optimistic', 'pessimistic'):
            raise ValueError('unknown_policy must be optimistic or pessimistic')
        self.unknown_cost = float(a.get('unknown_cost', 1.5))
        self.w = float(a.get('heuristic_weight', 1.0))
        self.vertical_cost = float(a.get('vertical_cost', 1.0))
        self.max_expansions = int(a.get('max_expansions', 400000))
        self.timeout_s = float(a.get('timeout_s', 5.0))
        self.goal_tol = int(a.get('goal_tolerance_cells', 1))
        self.snap_radius = int(a.get('snap_radius_cells', 4))
        self.prune = bool((cfg.get('pruning') or {}).get('enabled', True))

    # ------------------------------------------------------------------ helpers
    def traversable_mask(self, grid: PlanningGrid, corridor_mask: np.ndarray | None) -> np.ndarray:
        trav = ~grid.blocked
        if self.unknown_policy == 'pessimistic':
            trav &= ~grid.unknown
        if corridor_mask is not None:
            trav &= corridor_mask
        return trav

    def _snap(self, trav, idx):
        idx = np.asarray(idx)
        shape = np.array(trav.shape)
        if np.all((idx >= 0) & (idx < shape)) and trav[tuple(idx)]:
            return tuple(int(v) for v in idx), False
        best, best_d = None, None
        r = self.snap_radius
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    c = idx + (dx, dy, dz)
                    if np.any(c < 0) or np.any(c >= shape) or not trav[tuple(c)]:
                        continue
                    d = dx * dx + dy * dy + dz * dz
                    if best_d is None or d < best_d:
                        best, best_d = tuple(int(v) for v in c), d
        return best, True

    # ------------------------------------------------------------------ search
    def plan(self, grid: PlanningGrid, start_w, goal_w, corridor: Corridor | None = None,
             corridor_mask: np.ndarray | None = None) -> PlanResult:
        t0 = time.perf_counter()
        if corridor is not None and corridor_mask is None:
            corridor_mask = corridor.grid_mask(grid.shape, grid.origin, grid.resolution)
        trav = self.traversable_mask(grid, corridor_mask)

        if corridor is not None and not corridor.contains(np.asarray(goal_w).reshape(1, 3))[0]:
            return PlanResult(False, 'goal outside approved mission corridor',
                              latency_ms=(time.perf_counter() - t0) * 1e3)
        s_idx, s_snapped = self._snap(trav, grid.world_to_index(start_w))
        g_idx, g_snapped = self._snap(trav, grid.world_to_index(goal_w))
        if s_idx is None:
            return PlanResult(False, 'start not traversable (inside inflated obstacle or outside corridor)',
                              latency_ms=(time.perf_counter() - t0) * 1e3)
        if g_idx is None:
            return PlanResult(False, 'goal not traversable (inside inflated obstacle or outside corridor)',
                              latency_ms=(time.perf_counter() - t0) * 1e3)

        nx, ny, nz = trav.shape
        # padded flat arrays -> no bounds checks in the inner loop
        px, py, pz = nx + 2, ny + 2, nz + 2
        passable = np.zeros((px, py, pz), dtype=np.uint8)
        passable[1:-1, 1:-1, 1:-1] = trav
        unk = np.zeros((px, py, pz), dtype=np.uint8)
        unk[1:-1, 1:-1, 1:-1] = grid.unknown
        passable_f = passable.reshape(-1, order='F').tobytes()
        unknown_f = unk.reshape(-1, order='F').tobytes()
        sx, sxy = 1, px

        def flat(i, j, k):
            return (i + 1) + px * ((j + 1) + py * (k + 1))

        def unflat(f):
            k, rem = divmod(f, sxy * py)
            j, i = divmod(rem, sxy)
            return i - 1, j - 1, k - 1

        def off(dx, dy, dz):
            return dx * sx + dy * sxy + dz * sxy * py

        neigh = []
        for dx, dy, dz in _offsets(self.connectivity):
            step = math.sqrt(dx * dx + dy * dy + dz * dz)
            cost = step * (self.vertical_cost if dz != 0 else 1.0)
            # no corner cutting: every axis-aligned / planar sub-move of a diagonal must also be traversable,
            # otherwise the straight segment between the two cell centres clips a blocked cell
            comps = []
            for cx in ((0, dx) if dx else (0,)):
                for cy in ((0, dy) if dy else (0,)):
                    for cz in ((0, dz) if dz else (0,)):
                        if (cx, cy, cz) not in ((0, 0, 0), (dx, dy, dz)):
                            comps.append(off(cx, cy, cz))
            neigh.append((off(dx, dy, dz), cost, tuple(comps)))

        start = flat(*s_idx)
        goal = flat(*g_idx)
        w = self.w
        unknown_extra = self.unknown_cost if self.unknown_policy == 'optimistic' else 1.0
        # precomputed tables (flat padded index) -> no per-neighbour coordinate decoding in the inner loop
        ii, jj, kk = np.meshgrid(np.arange(px) - 1, np.arange(py) - 1, np.arange(pz) - 1, indexing='ij')
        h_tab = (w * np.sqrt((ii - g_idx[0]) ** 2 + (jj - g_idx[1]) ** 2 + (kk - g_idx[2]) ** 2)).reshape(-1, order='F').tolist()
        goal_tab = ((ii - g_idx[0]) ** 2 + (jj - g_idx[1]) ** 2 + (kk - g_idx[2]) ** 2
                    <= self.goal_tol ** 2).astype(np.uint8).reshape(-1, order='F').tobytes()
        n_cells = px * py * pz
        inf = math.inf
        g_cost = [inf] * n_cells
        g_cost[start] = 0.0
        parent = {start: -1}
        closed = bytearray(n_cells)
        open_heap = [(h_tab[start], 0.0, start)]
        heappop, heappush = heapq.heappop, heapq.heappush
        expansions = 0
        found = None
        deadline = t0 + self.timeout_s
        max_exp = self.max_expansions
        while open_heap:
            f, g, cur = heappop(open_heap)
            if closed[cur]:
                continue
            closed[cur] = 1
            expansions += 1
            if goal_tab[cur]:
                found = cur
                break
            if expansions > max_exp or (expansions & 1023) == 0 and time.perf_counter() > deadline:
                return PlanResult(False, f'search limit reached after {expansions} expansions',
                                  expansions=expansions, latency_ms=(time.perf_counter() - t0) * 1e3)
            for o, c, comps in neigh:
                nb = cur + o
                if not passable_f[nb] or closed[nb]:
                    continue
                if comps and not all(passable_f[cur + q] for q in comps):
                    continue
                ng = g + (c * unknown_extra if unknown_f[nb] else c)
                if ng < g_cost[nb]:
                    g_cost[nb] = ng
                    parent[nb] = cur
                    heappush(open_heap, (ng + h_tab[nb], ng, nb))
        if found is None:
            return PlanResult(False, 'no path exists inside the approved mission space',
                              expansions=expansions, latency_ms=(time.perf_counter() - t0) * 1e3)

        cells = []
        n = found
        while n != -1:
            cells.append(unflat(n))
            n = parent[n]
        cells.reverse()
        raw = grid.index_to_world(np.array(cells))
        # exact start/goal endpoints (if they were not snapped) instead of cell centres
        if not s_snapped:
            raw[0] = np.asarray(start_w, dtype=float)
        if not g_snapped:
            raw[-1] = np.asarray(goal_w, dtype=float)
        wps = prune_line_of_sight(raw, trav, grid) if self.prune else raw
        latency = (time.perf_counter() - t0) * 1e3
        return PlanResult(True, 'ok', cells, wps, raw, expansions, latency, path_length(wps),
                          s_snapped, g_snapped)


def segment_clear(a, b, trav: np.ndarray, grid: PlanningGrid) -> bool:
    pts = densify(np.vstack([a, b]), grid.resolution * 0.25)
    idx = grid.world_to_index(pts)
    if not np.all(grid.in_bounds(idx)):
        return False
    return bool(trav[idx[:, 0], idx[:, 1], idx[:, 2]].all())


def prune_line_of_sight(pts: np.ndarray, trav: np.ndarray, grid: PlanningGrid) -> np.ndarray:
    """Greedy shortcutting: from each anchor advance while the straight segment stays traversable.

    O(n) segment checks; every kept segment is verified against the inflated
    grid AND the corridor (trav already includes both).
    """
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    n = len(pts)
    while i < n - 1:
        j = i + 1
        while j + 1 < n and segment_clear(pts[i], pts[j + 1], trav, grid):
            j += 1
        out.append(pts[j])
        i = j
    return np.array(out)
