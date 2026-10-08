import math
import os

import numpy as np
import pytest
import yaml

from evaluation.metrics import distance_to_primitive, error_stats, landing_error, load_world, min_clearance
from ps05_common import config


def test_landing_error_formula():
    assert landing_error((42.3, 41.6), (42, 42)) == pytest.approx(0.5)


def test_error_stats():
    s = error_stats([0.1, 0.2, 0.3, 0.4, None, float('nan')])
    assert s['n'] == 4 and s['mean'] == pytest.approx(0.25)
    assert s['rmse'] == pytest.approx(math.sqrt((0.01 + 0.04 + 0.09 + 0.16) / 4))
    assert s['median'] == pytest.approx(0.25)
    assert error_stats([])['mean'] is None


def test_primitive_distances():
    box = {'type': 'box', 'pos': np.array([0.0, 0, 0]), 'size': [2, 2, 2], 'name': 'b'}
    assert distance_to_primitive(np.array([3.0, 0, 0]), box) == pytest.approx(2.0)
    assert distance_to_primitive(np.array([0.5, 0, 0]), box) == 0.0
    cyl = {'type': 'cylinder', 'pos': np.array([0.0, 0, 2]), 'radius': 0.5, 'length': 4, 'name': 'c'}
    assert distance_to_primitive(np.array([2.0, 0, 2]), cyl) == pytest.approx(1.5)
    sph = {'type': 'sphere', 'pos': np.array([0.0, 0, 0]), 'radius': 1, 'name': 's'}
    assert distance_to_primitive(np.array([0, 3.0, 0]), sph) == pytest.approx(2.0)


def test_world_spec_clearance():
    spec = yaml.safe_load(open(config.repo_root() / 'simulation/configs/world_ps05.yaml'))
    prims = load_world(spec)
    d, name = min_clearance([14.0, 18.0, 20.0], prims)     # above building_a (14 m tall)
    assert name == 'building_a' and d == pytest.approx(6.0)
    # start and pad are clear of obstacles
    assert min_clearance([5, 5, 1], prims)[0] > 3.0
    assert min_clearance([42, 42, 1], prims)[0] > 2.0


def test_config_scenario_override(tmp_path, monkeypatch):
    scen = tmp_path / 's.yaml'
    scen.write_text('id: t\nseed: 3\noverrides:\n  lidar:\n    fault_injection:\n      dropout_prob: 0.3\n')
    monkeypatch.setenv('PS05_SCENARIO_FILE', str(scen))
    c = config.load('lidar')
    assert c['fault_injection']['dropout_prob'] == 0.3
    assert c['main']['max_range'] == 30.0                   # untouched keys preserved
    assert config.load_scenario()['seed'] == 3


def test_corrupted_config_detected(tmp_path, monkeypatch):
    (tmp_path / 'planner.yaml').write_text('- not\n- a mapping\n')
    monkeypatch.setenv('PS05_CONFIG_DIR', str(tmp_path))
    with pytest.raises(ValueError):
        config.load('planner')
    with pytest.raises(KeyError):
        config.require({'a': {}}, 'a', 'b')


def test_all_configs_load():
    cfgs = config.load_all({})
    for name in ('safety', 'planner', 'mission', 'landing', 'fusion', 'mapping', 'camera', 'lidar', 'sensors'):
        assert name in cfgs
    assert len(config.config_hash(cfgs)) == 16
