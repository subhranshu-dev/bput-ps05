"""Regression: GPS / simulator ground truth must never be an autonomy input."""
import importlib.util
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('gps_audit', ROOT / 'scripts/gps_audit.py')
gps_audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gps_audit)


def test_static_gps_audit_passes():
    rep = gps_audit.static_audit()
    assert rep['source_findings'] == [], rep['source_findings']
    assert rep['model_has_no_gnss']
    assert rep['airframe_gps_disabled']


def test_audit_detects_violation(tmp_path, monkeypatch):
    """The audit must actually catch a planted violation (guards against a vacuous audit)."""
    fake = tmp_path / 'ros2_ws/src/planner_3d/planner_3d'
    fake.mkdir(parents=True)
    (fake / 'bad.py').write_text("sub = '/sim/truth/odom'\n")
    for d in ('simulation/models/x500_ps05', 'simulation/px4/airframes'):
        (tmp_path / d).mkdir(parents=True)
    (tmp_path / 'simulation/models/x500_ps05/model.sdf').write_text('<sensor type="navsat"/>')
    (tmp_path / 'simulation/px4/airframes/4050_gz_x500_ps05').write_text('param set-default EKF2_GPS_CTRL 7\n')
    monkeypatch.setattr(gps_audit, 'ROOT', tmp_path)
    rep = gps_audit.static_audit()
    assert rep['source_findings'] and not rep['model_has_no_gnss'] and not rep['airframe_gps_disabled']
    assert not rep['pass']


def test_autonomy_nodes_do_not_import_evaluation():
    for pkg in gps_audit.AUTONOMY_PACKAGES:
        for f in (ROOT / 'ros2_ws/src' / pkg).rglob('*.py'):
            t = f.read_text()
            assert not re.search(r'^\s*(from|import)\s+evaluation', t, re.M), f
            assert not re.search(r'^\s*(from|import)\s+aeronav_simulation', t, re.M), f
            assert not re.search(r'^\s*(from|import)\s+dashboard_bridge', t, re.M), f
