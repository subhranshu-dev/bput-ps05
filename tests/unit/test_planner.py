"""3D A* + inflation + corridor tests (Phase 9-11 test list)."""
import numpy as np
import pytest

from planner_3d.astar import AStarPlanner, path_length
from planner_3d.corridor import Corridor, densify
from planner_3d.grid import FREE, OCCUPIED, build_planning_grid, inflate, inflation_radius

CFG = {'astar': {'resolution': 0.5, 'connectivity': 26, 'unknown_policy': 'optimistic', 'unknown_cost': 1.5,
                 'heuristic_weight': 1.0, 'vertical_cost': 1.0, 'max_expansions': 500000, 'timeout_s': 20,
                 'goal_tolerance_cells': 0, 'snap_radius_cells': 2}, 'pruning': {'enabled': True}}
SAFETY = {'inflation': {'vehicle_radius': 0.4, 'localization_uncertainty': 0.15, 'controller_error': 0.15,
                        'desired_clearance': 0.25, 'grid_quantization': True}}


def world(shape=(40, 40, 16)):
    return np.zeros(shape, bool), np.zeros(shape, bool)


def plan(occ, unk, start, goal, corridor=None, radius=1.0, cfg=CFG):
    g = inflate(occ, unk, 0.5, np.zeros(3), radius)
    return AStarPlanner(cfg).plan(g, np.array(start, float), np.array(goal, float), corridor), g


def assert_clear(res, g):
    dense = densify(res.waypoints, 0.1)
    assert not g.is_blocked_world(dense).any()


def test_inflation_radius_from_config():
    assert inflation_radius(SAFETY, 0.5) == pytest.approx(0.4 + 0.15 + 0.15 + 0.25 + 0.25)
    s2 = {'inflation': dict(SAFETY['inflation'], grid_quantization=False)}
    assert inflation_radius(s2, 0.5) == pytest.approx(0.95)


def test_empty_world_straight_line():
    occ, unk = world()
    res, g = plan(occ, unk, [2, 2, 3], [17, 17, 5])
    assert res.success
    assert len(res.waypoints) == 2                         # pruned to a straight segment
    assert res.length_m == pytest.approx(np.linalg.norm([15, 15, 2]), rel=1e-6)


def test_single_wall_detour():
    occ, unk = world()
    occ[20, 0:30, :] = True                                # wall x=10m, y 0..15 m, full height
    res, g = plan(occ, unk, [3, 5, 3], [17, 5, 3])
    assert res.success
    assert_clear(res, g)
    assert res.waypoints[:, 1].max() > 15.0                # went around the end of the wall
    assert res.length_m > 14.0


def test_blocked_direct_path_goes_over():
    occ, unk = world()
    occ[20, :, 0:8] = True                                 # 4 m tall full-width wall
    res, g = plan(occ, unk, [3, 10, 2], [17, 10, 2])
    assert res.success
    assert_clear(res, g)
    # true 3D: climbs over. Clearance is cell-centre to cell-centre: top occupied cell centre 3.75 m
    # + radius 1.0 -> first free centre 4.75 m (the half-cell grid_quantization term covers this in configs).
    assert res.waypoints[:, 2].max() >= 3.75 + 1.0 - 1e-9


def test_overhead_obstacle_goes_under():
    occ, unk = world()
    occ[:, :, 8:10] = True                                 # ceiling slab at 4-5 m everywhere
    res, g = plan(occ, unk, [3, 3, 2], [17, 17, 2])
    assert res.success
    assert_clear(res, g)
    assert res.waypoints[:, 2].max() < 4.0 - 0.9


def test_impossible_goal_inside_obstacle():
    occ, unk = world()
    occ[30:36, 30:36, 0:16] = True
    res, _ = plan(occ, unk, [2, 2, 3], [16.5, 16.5, 3])
    assert not res.success
    assert 'goal' in res.message


def test_no_path_enclosed_goal():
    occ, unk = world()
    occ[25:39, 25, :] = occ[25:39, 38, :] = occ[25, 25:39, :] = occ[38, 25:39, :] = True
    occ[25:39, 25:39, 0] = occ[25:39, 25:39, 15] = True
    res, _ = plan(occ, unk, [2, 2, 3], [16, 16, 3])
    assert not res.success and 'no path' in res.message


def test_corridor_violation_rejected_and_respected():
    occ, unk = world()
    occ[20, 0:34, :] = True    # wall leaves a gap only at y > 17 m
    narrow = Corridor([0, 20], [0, 12], [0, 8], centerline=[[2, 5, 3], [18, 5, 3]], max_deviation=6)
    res, _ = plan(occ, unk, [3, 5, 3], [17, 5, 3], corridor=narrow)
    assert not res.success                                  # the only route leaves the approved space
    goal_out = Corridor([0, 20], [0, 20], [0, 8])
    res2, _ = plan(occ, unk, [3, 5, 3], [17, 5, 9], corridor=goal_out)
    assert not res2.success and 'corridor' in res2.message
    wide = Corridor([0, 20], [0, 20], [0, 8])
    res3, g = plan(occ, unk, [3, 5, 3], [17, 5, 3], corridor=wide)
    assert res3.success and wide.validate_path(res3.waypoints)['ok']


def test_narrow_passage():
    occ, unk = world()
    occ[20, :, :] = True
    occ[20, 18:23, 4:9] = False       # 2.5 m x 2.5 m window
    res, g = plan(occ, unk, [3, 10, 3], [17, 10, 3], radius=1.0)
    assert res.success
    assert_clear(res, g)
    # window half-width (cell-centre to occupied cell-centre) is 1.5 m; blocking is clearance < radius
    res2, _ = plan(occ, unk, [3, 10, 3], [17, 10, 3], radius=1.6)   # bigger vehicle -> window too small
    assert not res2.success


def test_inflation_rejects_unsafe_clearance_path():
    """A path that is collision-free on raw occupancy but violates clearance must be rejected."""
    occ, unk = world()
    occ[20, :, 0:12] = True                                  # 6 m wall; safe route exists over the top
    occ[20, 19:21, 4:8] = False                              # 1 m slot: raw-free but too tight
    g_raw = inflate(occ, unk, 0.5, np.zeros(3), 0.0)
    assert not g_raw.is_blocked_world([[10.25, 10.0, 3.0]])[0]   # raw: centre of slot is free
    res, g = plan(occ, unk, [3, 10, 3], [17, 10, 3], radius=1.0)
    assert res.success
    dense = densify(res.waypoints, 0.1)
    assert g.clearance_world(dense).min() >= 1.0            # route keeps the required clearance
    assert res.waypoints[:, 2].max() > 6.0                 # went over the wall, did not use the slot


def test_pessimistic_unknown_blocks():
    occ, unk = world()
    unk[15:25, :, :] = True
    cfg = {**CFG, 'astar': dict(CFG['astar'], unknown_policy='pessimistic')}
    res, _ = plan(occ, unk, [3, 10, 3], [17, 10, 3], cfg=cfg)
    assert not res.success
    res2, _ = plan(occ, unk, [3, 10, 3], [17, 10, 3])
    assert res2.success


def test_downsample_maxpool():
    states = np.full((8, 8, 8), FREE, np.uint8)
    states[3, 3, 3] = OCCUPIED
    g = build_planning_grid(states, 0.25, [0, 0, 0], 0.5, 0.0)
    assert g.shape == (4, 4, 4) and g.occupied[1, 1, 1] and g.occupied.sum() == 1


def test_latency_reported():
    occ, unk = world()
    res, _ = plan(occ, unk, [2, 2, 3], [17, 17, 5])
    assert res.latency_ms > 0 and res.expansions > 0
    assert path_length(res.waypoints) == pytest.approx(res.length_m)


def test_no_corner_cutting_every_segment_valid():
    """Diagonal moves must not clip blocked cells: the dense polyline of the *raw* A* cells stays traversable."""
    occ, unk = world()
    rng = np.random.default_rng(3)
    occ[rng.random(occ.shape) < 0.08] = True           # cluttered 3D field
    occ[:4, :4, :8] = False
    occ[-4:, -4:, :8] = False
    g = inflate(occ, unk, 0.5, np.zeros(3), 0.0)
    res = AStarPlanner(CFG).plan(g, np.array([0.75, 0.75, 1.25]), np.array([18.75, 18.75, 2.25]))
    assert res.success
    dense = densify(res.raw_waypoints, 0.05)
    assert not g.is_blocked_world(dense).any()
    assert not g.is_blocked_world(densify(res.waypoints, 0.05)).any()


def test_planner_clearance_exceeds_live_check_envelope():
    """Consistency: planned paths keep surface clearance above the live LiDAR hard envelope."""
    import yaml
    from ps05_common import config
    saf = yaml.safe_load(open(config.config_dir() / 'safety.yaml'))
    res = yaml.safe_load(open(config.config_dir() / 'planner.yaml'))['astar']['resolution']
    inf = saf['inflation']
    hard = inf['vehicle_radius'] + inf['localization_uncertainty'] + inf['controller_error']
    guaranteed_surface = inflation_radius(saf, res) - (3 ** 0.5) / 2 * res
    assert guaranteed_surface > hard


def test_local_grid_matches_global_inside_box():
    from planner_3d.grid import build_local_grid, build_planning_grid
    rng = np.random.default_rng(5)
    states = np.zeros((80, 80, 40), np.uint8)
    states[rng.random(states.shape) < 0.01] = OCCUPIED
    g = build_planning_grid(states, 0.25, [0, 0, -0.5], 0.5, 1.2)
    lo, hi = np.array([6.0, 7.0, 1.0]), np.array([12.0, 11.0, 4.0])
    loc = build_local_grid(states, 0.25, [0, 0, -0.5], 0.5, 1.2, lo, hi)
    pts = np.column_stack([rng.uniform(lo[0], hi[0], 500), rng.uniform(lo[1], hi[1], 500), rng.uniform(lo[2], hi[2], 500)])
    assert np.array_equal(g.is_blocked_world(pts), loc.is_blocked_world(pts))


def test_final_validation_consistent_with_planning():
    """Every successful plan's smoothed trajectory passes the same validity check used for final validation."""
    from planner_3d.astar import polyline_clear
    from planner_3d.trajectory import build_trajectory
    rng = np.random.default_rng(11)
    tcfg = {'waypoint_spacing': 0.5, 'smoothing_iterations': 3, 'v_max': 2.0, 'a_max': 1.0, 'v_max_z': 1.0,
            'corner_speed_factor': 0.6, 'yaw_mode': 'path_tangent', 'yaw_rate_max': 0.8}
    for trial in range(15):
        occ, unk = world()
        occ[rng.random(occ.shape) < 0.04] = True
        occ[:4, :4, :8] = occ[-4:, -4:, :8] = False
        g = inflate(occ, unk, 0.5, np.zeros(3), 0.6)
        pl = AStarPlanner(CFG)
        res = pl.plan(g, np.array([0.75, 0.75, 1.25]), np.array([18.75, 18.75, 2.25]))
        if not res.success:
            continue
        trav = pl.traversable_mask(g, None)
        tr = build_trajectory(res.waypoints, tcfg, lambda p: polyline_clear(p, trav, g))
        assert polyline_clear(res.waypoints, trav, g), trial
        assert polyline_clear(tr.polyline, trav, g), trial
        # every flown sample lies on the validated polyline (vertex-preserving resampling)
        from planner_3d.corridor import point_segment_distance
        dmin = np.min([point_segment_distance(tr.pos, a, b) for a, b in zip(tr.polyline[:-1], tr.polyline[1:])], axis=0)
        assert dmin.max() < 1e-9
