import math

import numpy as np

from ps05_common.frames import rpy_to_matrix
from sensor_fusion.projection import (colorize, edge_chamfer_px, pack_rgb, project, reprojection_shift,
                                      transform_points, unpack_rgb)

K = np.array([[381.36, 0, 320.0], [0, 381.36, 240.0], [0, 0, 1]])
OPT = rpy_to_matrix(-math.pi / 2, 0, -math.pi / 2)   # camera_link -> optical


def cam_from_lidar(dx=0.0, dyaw=0.0):
    # camera at (0.12,0,0.05), lidar at (0,0,0.2) in base_link
    base_cam = np.eye(4)
    base_cam[:3, :3] = rpy_to_matrix(0, 0, dyaw) @ OPT
    base_cam[:3, 3] = [0.12 + dx, 0, 0.05]
    base_lidar = np.eye(4)
    base_lidar[:3, 3] = [0, 0, 0.2]
    return np.linalg.inv(base_cam) @ base_lidar


def test_pinhole_equation():
    p = np.array([[0.5, -0.2, 4.0]])
    u, v, ok = project(p, K, 640, 480)
    assert ok[0]
    assert math.isclose(u[0], 381.36 * 0.5 / 4 + 320)
    assert math.isclose(v[0], 381.36 * -0.2 / 4 + 240)


def test_point_ahead_projects_near_centre_and_behind_rejected():
    t = cam_from_lidar()
    pts = np.array([[10.0, 0.0, 0.2 - 0.05 - 0.15 + 0.15], [-5.0, 0, 0], [10.0, 30.0, 0.0]])
    pc = transform_points(t, pts)
    u, v, ok = project(pc, K, 640, 480)
    assert ok[0] and abs(u[0] - 320) < 1.0          # straight ahead -> image centre column
    assert not ok[1]                                 # behind the camera
    assert not ok[2]                                 # outside the FOV
    # LiDAR above camera -> point at LiDAR height appears above image centre (smaller v)
    assert v[0] < 240


def test_left_point_projects_left():
    pc = transform_points(cam_from_lidar(), np.array([[8.0, 2.0, 0.0]]))
    u, _, ok = project(pc, K, 640, 480)
    assert ok[0] and u[0] < 320


def test_colorize_uses_image_pixel_and_flags_fov():
    img = np.zeros((480, 640, 3), np.uint8)
    img[:, :320] = [255, 0, 0]
    img[:, 320:] = [0, 0, 255]
    pts = np.array([[8.0, 1.0, 0.15], [8.0, -1.0, 0.15], [-3, 0, 0]])
    rgb, fov, _, _ = colorize(pts, cam_from_lidar(), K, img)
    assert fov.tolist() == [True, True, False]
    assert rgb[0].tolist() == [255, 0, 0] and rgb[1].tolist() == [0, 0, 255]
    assert rgb[2].tolist() == [128, 128, 128]      # not coloured


def test_rgb_pack_roundtrip():
    rgb = np.array([[1, 2, 3], [255, 128, 0]], np.uint8)
    assert np.array_equal(unpack_rgb(pack_rgb(rgb)), rgb)


def test_extrinsic_perturbation_increases_shift():
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(3, 20, 500), rng.uniform(-5, 5, 500), rng.uniform(-1, 2, 500)])
    s0, _, n0 = reprojection_shift(pts, cam_from_lidar(), cam_from_lidar(), K, 640, 480)
    s1, _, _ = reprojection_shift(pts, cam_from_lidar(), cam_from_lidar(dyaw=math.radians(1)), K, 640, 480)
    s2, _, _ = reprojection_shift(pts, cam_from_lidar(), cam_from_lidar(dyaw=math.radians(3)), K, 640, 480)
    assert n0 > 100 and s0 < 1e-9
    # 1 deg yaw ~ fx * tan(1 deg) ~ 6.7 px
    assert 5.0 < s1 < 8.0 and s2 > 2.5 * s1


def test_edge_chamfer():
    edges = np.zeros((480, 640), np.uint8)
    edges[:, 300] = 255
    d, n = edge_chamfer_px(edges, np.array([300.0, 305.0]), np.array([100.0, 200.0]))
    assert n == 2 and math.isclose(d, 2.5, abs_tol=0.6)
