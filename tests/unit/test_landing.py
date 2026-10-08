import math

import cv2
import numpy as np
import pytest

from landing_controller.pid import PID
from landing_detector.marker import MarkerDetector, make_tag_image, pixel_error
from landing_validator.surface import fit_plane, validate

K = np.array([[300.0, 0, 320], [0, 300.0, 240], [0, 0, 1]])
VCFG = {'pad_radius': 1.2, 'clearance_radius': 2.0, 'max_tilt_deg': 8, 'max_plane_rms': 0.05, 'obstacle_height': 0.15,
        'min_points': 30, 'max_alignment_error': 0.35}


def render_tag(dist=3.0, offset=(0.0, 0.0), occlude=False):
    """Synthetic downward view of a 0.4 m tag (0.6 m plate) at distance `dist`, via homography."""
    tag = make_tag_image('DICT_APRILTAG_36h11', 0, 400, 0.25)
    s = tag.shape[0]
    h = 0.3
    corners3d = np.array([[-h, -h, dist], [h, -h, dist], [h, h, dist], [-h, h, dist]]) + [offset[0], offset[1], 0]
    proj = (K @ corners3d.T).T
    dst = (proj[:, :2] / proj[:, 2:]).astype(np.float32)
    src = np.array([[0, 0], [s, 0], [s, s], [0, s]], np.float32)
    img = np.full((480, 640), 120, np.uint8)
    warped = cv2.warpPerspective(tag, cv2.getPerspectiveTransform(src, dst), (640, 480), borderValue=120)
    mask = cv2.warpPerspective(np.full_like(tag, 255), cv2.getPerspectiveTransform(src, dst), (640, 480))
    img[mask > 0] = warped[mask > 0]
    if occlude:
        img[150:330, 220:420] = 90
    return img


def test_marker_detect_and_pose():
    d = MarkerDetector('DICT_APRILTAG_36h11', 0, 0.4)
    r = d.detect(render_tag(3.0, (0.2, -0.1)), K)
    assert r is not None and r['id'] == 0
    assert np.allclose(r['tvec'], [0.2, -0.1, 3.0], atol=0.05)
    assert r['reprojection_px'] < 1.0
    ex, ey = pixel_error(r['center_px'], K)
    assert ex > 0 and ey < 0


def test_marker_partially_visible_and_hidden():
    d = MarkerDetector('DICT_APRILTAG_36h11', 0, 0.4)
    assert d.detect(render_tag(3.0, occlude=True), K) is None          # hidden -> no false detection
    assert d.detect(np.full((480, 640), 120, np.uint8), K) is None
    # partially outside the image: tag cut by the border -> rejected rather than a wrong pose
    r = d.detect(render_tag(2.0, (1.3, 0)), K)
    assert r is None or abs(r['tvec'][0] - 1.3) < 0.1


def test_wrong_marker_id_ignored():
    d = MarkerDetector('DICT_APRILTAG_36h11', 5, 0.4)
    assert d.detect(render_tag(3.0), K) is None


def ground(n=400, z=0.1, tilt=0.0, noise=0.005, seed=0, c=(42, 42)):
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-2.2, 2.2, (n, 2)) + c
    zz = z + math.tan(math.radians(tilt)) * (xy[:, 0] - c[0]) + rng.normal(0, noise, n)
    return np.column_stack([xy, zz])


def test_plane_fit():
    n, d, rms, _ = fit_plane(ground(tilt=5.0))
    assert math.degrees(math.acos(n[2])) == pytest.approx(5.0, abs=0.3)
    assert rms < 0.01


def test_validator_safe_pad():
    r = validate(ground(), (42, 42), (42.1, 42.0, 3.0), VCFG)
    assert r['safe'] and r['surface_ok'] and r['height_above_ground'] == pytest.approx(2.9, abs=0.05)


def test_validator_obstruction():
    pts = ground()
    box = np.column_stack([np.random.default_rng(1).uniform(42.3, 42.9, (60, 2)), np.full(60, 0.6)])
    r = validate(np.vstack([pts, box]), (42, 42), (42, 42, 3.0), VCFG)
    assert not r['safe'] and 'OBSTRUCTION' in r['reasons']


def test_validator_tilted_and_missing_ground():
    r = validate(ground(tilt=15), (42, 42), (42, 42, 3.0), VCFG)
    assert 'SURFACE_TILTED' in r['reasons'] and not r['safe']
    r2 = validate(np.zeros((0, 3)), (42, 42), (42, 42, 3.0), VCFG)
    assert 'NO_GROUND_PLANE' in r2['reasons'] and not r2['safe']


def test_validator_misaligned_but_surface_ok():
    r = validate(ground(), (42, 42), (43.0, 42.0, 3.0), VCFG)
    assert r['surface_ok'] and not r['aligned'] and not r['safe']


def test_pid_terms_saturation_antiwindup():
    p = PID(1.0, 0.5, 0.1, out_limit=0.6, i_limit=0.5)
    u = p.update(2.0, 0.05)
    assert u == pytest.approx(0.6)                      # saturated
    for _ in range(200):
        p.update(2.0, 0.05)
    assert abs(p.integral) <= 0.5 + 1e-9                # anti-windup clamp
    # error reverses: output must respond immediately (no wind-up lag)
    assert p.update(-0.5, 0.05) < 0.2


def test_pid_converges_first_order_plant():
    p = PID(0.8, 0.05, 0.15, out_limit=0.6, i_limit=0.5)
    x, target, dt = 0.0, 1.0, 0.05
    for _ in range(400):
        x += p.update(target - x, dt) * dt
    assert abs(x - target) < 0.02
