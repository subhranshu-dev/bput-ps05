import math

import numpy as np
import pytest

from localization.spherical import (cartesian_to_spherical, spherical_covariance, spherical_to_cartesian,
                                    spatial_error, station_to_map)


def test_ps05_formula():
    r, th, ph = 10.0, math.radians(30), math.radians(20)
    x, y, z = spherical_to_cartesian(r, th, ph)
    assert math.isclose(x, r * math.cos(ph) * math.cos(th))
    assert math.isclose(y, r * math.cos(ph) * math.sin(th))
    assert math.isclose(z, r * math.sin(ph))


@pytest.mark.parametrize('p', [[3, 4, 5], [-10, 2, 1], [0.1, -7, 12], [40, 41, 5]])
def test_roundtrip(p):
    assert np.allclose(spherical_to_cartesian(*cartesian_to_spherical(p)), p)


def test_covariance_monte_carlo():
    rng = np.random.default_rng(1)
    r, th, ph = 30.0, 0.8, 0.15
    s = (0.05, 0.0015, 0.0015)
    samples = np.array([spherical_to_cartesian(r + rng.normal(0, s[0]), th + rng.normal(0, s[1]), ph + rng.normal(0, s[2]))
                        for _ in range(20000)])
    emp = np.cov(samples.T)
    pred = spherical_covariance(r, th, ph, *s)
    assert np.allclose(np.sqrt(np.diag(emp)), np.sqrt(np.diag(pred)), rtol=0.05)


def test_station_offset_and_error():
    p = station_to_map([1, 2, 3], [10, 0, 1.5])
    assert np.allclose(p, [11, 2, 4.5])
    d, e = spatial_error([1, 2, 2], [1, 0, 0])
    assert math.isclose(e, math.sqrt(8))
