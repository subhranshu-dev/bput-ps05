import numpy as np
import pytest

from planner_3d.corridor import Corridor, densify
from planner_3d.grid import inflate
from planner_3d.trajectory import build_trajectory, chaikin, curvature, resample, time_parameterize

CFG = {'waypoint_spacing': 0.5, 'smoothing_iterations': 3, 'v_max': 2.0, 'a_max': 1.0, 'v_max_z': 1.0,
       'corner_speed_factor': 0.6, 'yaw_mode': 'path_tangent', 'yaw_rate_max': 0.8}
WPS = np.array([[0, 0, 2], [10, 0, 2], [10, 10, 4], [20, 10, 4]], float)


def test_continuity_and_endpoints():
    tr = build_trajectory(WPS, CFG)
    assert np.allclose(tr.pos[0], WPS[0]) and np.allclose(tr.pos[-1], WPS[-1])
    steps = np.linalg.norm(np.diff(tr.pos, axis=0), axis=1)
    assert steps.max() <= 0.5 + 1e-6
    assert np.all(np.diff(tr.t) > 0)
    assert tr.speed[0] == 0 and tr.speed[-1] == 0


def test_velocity_and_acceleration_limits():
    tr = build_trajectory(WPS, CFG)
    assert tr.speed.max() <= 2.0 + 1e-9
    vz = np.abs(tr.vel[:, 2])
    assert vz.max() <= 1.0 + 1e-6
    dv = np.diff(tr.speed)
    dt = np.diff(tr.t)
    assert np.all(np.abs(dv / dt) <= 1.0 + 1e-6)


def test_corner_speed_reduced():
    tr = time_parameterize(resample(WPS, 0.5), CFG)
    k = curvature(tr.pos)
    corner = np.argmax(k)
    assert tr.speed[corner] < 1.5


def test_smoothing_rejected_when_invalid():
    tr = build_trajectory(WPS, CFG, is_valid=lambda p: False)
    # unsmoothed: sharp corner point retained exactly
    assert np.min(np.linalg.norm(tr.pos - WPS[1], axis=1)) < 1e-9


def test_smoothing_cuts_corners_when_valid():
    tr = build_trajectory(WPS, CFG)
    assert np.min(np.linalg.norm(tr.pos - WPS[1], axis=1)) > 0.3


def test_collision_safety_and_corridor():
    occ = np.zeros((50, 40, 16), bool)
    occ[22:26, 0:10, :] = True   # obstacle near the first corner
    g = inflate(occ, np.zeros_like(occ), 0.5, np.zeros(3), 1.0)
    cor = Corridor([-1, 25], [-1, 15], [1, 6])
    trav = ~g.blocked

    def valid(p):
        d = densify(p, 0.1)
        idx = g.world_to_index(d)
        return bool(np.all(g.in_bounds(idx)) and trav[idx[:, 0], idx[:, 1], idx[:, 2]].all() and cor.contains(d).all())

    wps = np.array([[1, 7, 2], [8, 7, 2], [8, 13, 3], [20, 13, 3]], float)
    assert valid(wps)
    tr = build_trajectory(wps, CFG, valid)
    assert valid(tr.pos)


def test_sample_interpolates():
    tr = build_trajectory(WPS, CFG)
    p, v, y = tr.sample(tr.duration / 2)
    assert np.isfinite(p).all() and np.linalg.norm(v) > 0
    p_end, v_end, _ = tr.sample(tr.duration + 5)
    assert np.allclose(p_end, WPS[-1]) and np.allclose(v_end, 0)


def test_chaikin_preserves_endpoints():
    c = chaikin(WPS)
    assert np.allclose(c[0], WPS[0]) and np.allclose(c[-1], WPS[-1]) and len(c) == 2 * (len(WPS) - 1) + 2
