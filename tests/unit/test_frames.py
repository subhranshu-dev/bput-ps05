import math

import numpy as np
import pytest

from ps05_common import frames as F


def test_enu_ned_roundtrip():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(F.enu_to_ned(v), [2.0, 1.0, -3.0])
    assert np.allclose(F.ned_to_enu(F.enu_to_ned(v)), v)


@pytest.mark.parametrize('yaw', [0.0, 0.3, math.pi / 2, -2.0, 3.0])
def test_yaw_conversion_roundtrip(yaw):
    assert math.isclose(F.wrap_pi(F.yaw_ned_to_enu(F.yaw_enu_to_ned(yaw)) - yaw), 0.0, abs_tol=1e-9)


def test_yaw_east_is_ned_90():
    # facing East: ENU yaw 0, NED yaw +90 deg
    assert math.isclose(F.yaw_enu_to_ned(0.0), math.pi / 2)


@pytest.mark.parametrize('yaw_ned', [0.0, 0.7, -1.2, 3.0])
def test_attitude_conversion_level(yaw_ned):
    q_ned = np.array([math.cos(yaw_ned / 2), 0, 0, math.sin(yaw_ned / 2)])
    q_enu = F.quat_ned_frd_to_enu_flu(q_ned)
    assert math.isclose(F.wrap_pi(F.yaw_from_quat(q_enu) - F.yaw_ned_to_enu(yaw_ned)), 0.0, abs_tol=1e-9)
    # body z axis stays up in ENU
    r = F.quat_to_matrix(q_enu)
    assert np.allclose(r[:, 2], [0, 0, 1], atol=1e-9)
    assert np.allclose(F.quat_enu_flu_to_ned_frd(q_enu) * np.sign(F.quat_enu_flu_to_ned_frd(q_enu)[0] or 1),
                       q_ned * np.sign(q_ned[0] or 1), atol=1e-9)


def test_rotation_matrix_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(20):
        r = F.rpy_to_matrix(*rng.uniform(-3, 3, 3))
        q = F.matrix_to_quat(r)
        assert np.allclose(F.quat_to_matrix(q), r, atol=1e-9)
