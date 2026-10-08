"""TF validation (Phase 5): static TF tree vs. Gazebo model sensor poses, frame tree shape, optical convention."""
import math
import re
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from aeronav_description.static_frames import static_transforms
from ps05_common import config
from ps05_common.frames import rpy_to_matrix

SENSORS = yaml.safe_load(open(config.config_dir() / 'sensors.yaml'))
MODEL = config.repo_root() / 'simulation/models/x500_ps05/model.sdf'
EXPECTED_PARENT = {'map': 'world', 'odom': 'map', 'camera_link': 'base_link', 'camera_optical_frame': 'camera_link',
                   'lidar_link': 'base_link', 'lidar_frame': 'lidar_link', 'down_camera_link': 'base_link',
                   'down_camera_optical_frame': 'down_camera_link', 'down_lidar_link': 'base_link',
                   'down_lidar_frame': 'down_lidar_link'}


def tf_map(cfg=SENSORS):
    return {c: (p, xyz, rpy) for p, c, xyz, rpy in static_transforms(cfg)}


def test_tree_parents_and_no_missing_frames():
    m = tf_map()
    assert set(m) == set(EXPECTED_PARENT)
    for child, parent in EXPECTED_PARENT.items():
        assert m[child][0] == parent, child


def test_tree_is_acyclic_and_rooted_at_world():
    m = tf_map()
    for child in m:
        seen, cur = set(), child
        while cur in m:
            assert cur not in seen
            seen.add(cur)
            cur = m[cur][0]
        assert cur in ('world', 'base_link')     # base_link attaches via odom->base_link (dynamic)


def test_valid_transforms():
    for child, (_, xyz, rpy) in tf_map().items():
        assert all(math.isfinite(v) for v in list(xyz) + list(rpy)), child
        r = rpy_to_matrix(*rpy)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(r), 1.0)
        assert np.linalg.norm(xyz) < 1.0, f'{child}: implausible sensor offset'


def test_optical_frame_convention():
    """optical z = forward (link x), optical x = right (link -y), optical y = down (link -z)."""
    r = rpy_to_matrix(*SENSORS['optical_rpy'])
    assert np.allclose(r[:, 2], [1, 0, 0], atol=1e-9)
    assert np.allclose(r[:, 0], [0, -1, 0], atol=1e-9)
    assert np.allclose(r[:, 1], [0, 0, -1], atol=1e-9)


def test_down_sensors_point_down():
    for k in ('down_camera_link', 'down_lidar_link'):
        r = rpy_to_matrix(*SENSORS[k]['rpy'])
        assert np.allclose(r[:, 0], [0, 0, -1], atol=1e-9), k


@pytest.mark.skipif(not MODEL.exists(), reason='vehicle model not generated yet')
def test_model_sensor_poses_match_tf():
    root = ET.parse(MODEL).getroot()
    sensors = {s.get('name'): s for s in root.iter('sensor')}
    pairs = {'front_camera': 'camera_link', 'lidar_3d': 'lidar_link', 'down_camera': 'down_camera_link',
             'down_lidar': 'down_lidar_link'}
    frames = {'front_camera': 'camera_optical_frame', 'lidar_3d': 'lidar_frame', 'down_camera': 'down_camera_optical_frame',
              'down_lidar': 'down_lidar_frame'}
    for s, link in pairs.items():
        pose = [float(v) for v in sensors[s].find('pose').text.split()]
        assert np.allclose(pose[:3], SENSORS[link]['xyz']) and np.allclose(pose[3:], SENSORS[link]['rpy']), s
        assert sensors[s].find('gz_frame_id').text == frames[s]
    assert 'navsat_sensor' not in sensors


def test_perturbation_detected_as_transform_change():
    cfg = dict(SENSORS, camera_extrinsic_perturbation={'xyz': [0.05, 0, 0], 'rpy': [0, 0, 0.035]})
    a, b = tf_map()['camera_link'], tf_map(cfg)['camera_link']
    assert not np.allclose(a[1], b[1]) and not np.allclose(a[2], b[2])
