"""PS05 ground-tracking geometry (PS05 report Sec. 8.1 and Sec. 11).

Polar/spherical -> Cartesian (station frame):
    X = r cos(phi) cos(theta)
    Y = r cos(phi) sin(theta)
    Z = r sin(phi)
Spatial error:
    E = sqrt(dX^2 + dY^2 + dZ^2)
"""
import math

import numpy as np


def spherical_to_cartesian(r: float, theta: float, phi: float) -> np.ndarray:
    return np.array([
        r * math.cos(phi) * math.cos(theta),
        r * math.cos(phi) * math.sin(theta),
        r * math.sin(phi),
    ])


def cartesian_to_spherical(p) -> tuple:
    x, y, z = (float(v) for v in p)
    r = math.sqrt(x * x + y * y + z * z)
    theta = math.atan2(y, x)
    phi = math.asin(z / r) if r > 1e-9 else 0.0
    return r, theta, phi


def spherical_covariance(r, theta, phi, sr, st, sp) -> np.ndarray:
    """First-order propagation of (r, theta, phi) noise to Cartesian covariance."""
    ct, stt = math.cos(theta), math.sin(theta)
    cp, spp = math.cos(phi), math.sin(phi)
    j = np.array([
        [cp * ct, -r * cp * stt, -r * spp * ct],
        [cp * stt, r * cp * ct, -r * spp * stt],
        [spp, 0.0, r * cp],
    ])
    s = np.diag([sr * sr, st * st, sp * sp])
    return j @ s @ j.T


def station_to_map(p_station, station_position) -> np.ndarray:
    """Station frame is axis-aligned with the map frame (surveyed); only translated."""
    return np.asarray(p_station, dtype=float) + np.asarray(station_position, dtype=float)


def map_to_station(p_map, station_position) -> np.ndarray:
    return np.asarray(p_map, dtype=float) - np.asarray(station_position, dtype=float)


def spatial_error(p_actual, p_target) -> tuple:
    d = np.asarray(p_actual, dtype=float) - np.asarray(p_target, dtype=float)
    return d, float(np.linalg.norm(d))
