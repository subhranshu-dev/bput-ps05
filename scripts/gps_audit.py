#!/usr/bin/env python3
"""GPS / ground-truth dependency audit (Phase 14).

STATIC (always):
  1. No autonomy package source references GPS topics / truth topics / world spec.
  2. The vehicle model contains no navsat (GNSS) sensor.
  3. The PX4 airframe disables GPS fusion (EKF2_GPS_CTRL 0) and uses external vision.
RUNTIME (--runtime, needs the running system):
  4. For every autonomy node, its subscriptions contain no forbidden topic.
  5. PX4 publishes no GPS position (vehicle_gps_position / sensor_gps silent or absent).
Exit 0 = PASS. Writes a JSON report with --out.
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/ps05_common'))
from ps05_common import topics  # noqa: E402

AUTONOMY_PACKAGES = ['camera_processor', 'lidar_processor', 'sensor_fusion', 'localization', 'mapping', 'planner_3d',
                     'obstacle_avoidance', 'landing_detector', 'landing_validator', 'landing_controller', 'px4_interface',
                     'aeronav_description', 'ps05_common']
FORBIDDEN_PATTERNS = [r'/sim/truth', r'TRUTH_ODOM', r'world_ps05\.yaml', r'vehicle_gps_position', r'sensor_gps',
                      r'vehicle_global_position', r'SensorGps', r'VehicleGlobalPosition', r'NavSat', r'navsat',
                      r'/world/[a-z_]+/pose', r'/model/']
ALLOWED_FILES = {'ps05_common/topics.py'}   # defines the forbidden list itself


def static_audit():
    findings = []
    for pkg in AUTONOMY_PACKAGES:
        for f in (ROOT / 'ros2_ws/src' / pkg).rglob('*.py'):
            rel = f.relative_to(ROOT / 'ros2_ws/src').as_posix()
            if any(rel.endswith(a) for a in ALLOWED_FILES) or '/test' in rel:
                continue
            text = f.read_text()
            for pat in FORBIDDEN_PATTERNS:
                for m in re.finditer(pat, text):
                    line = text[: m.start()].count('\n') + 1
                    findings.append({'file': rel, 'line': line, 'pattern': pat})
    model = (ROOT / 'simulation/models/x500_ps05/model.sdf')
    model_ok = model.exists() and 'type="navsat"' not in model.read_text()
    af = (ROOT / 'simulation/px4/airframes/4050_gz_x500_ps05').read_text()
    airframe_ok = bool(re.search(r'EKF2_GPS_CTRL\s+0', af)) and bool(re.search(r'EKF2_EV_CTRL\s+[1-9]', af))
    return {'source_findings': findings, 'model_has_no_gnss': model_ok, 'airframe_gps_disabled': airframe_ok,
            'pass': not findings and model_ok and airframe_ok}


def runtime_audit():
    import rclpy
    rclpy.init()
    node = rclpy.create_node('gps_audit')
    import time
    time.sleep(2.0)
    report = {'nodes': {}, 'violations': []}
    for name, ns in node.get_node_names_and_namespaces():
        if name not in topics.AUTONOMY_NODES:
            continue
        subs = [t for t, _ in node.get_subscriber_names_and_types_by_node(name, ns)]
        bad = [t for t in subs if t in topics.FORBIDDEN_FOR_AUTONOMY or t.startswith(topics.FORBIDDEN_PREFIXES_FOR_AUTONOMY)]
        report['nodes'][name] = {'subscriptions': sorted(subs), 'forbidden': bad}
        report['violations'] += [{'node': name, 'topic': t} for t in bad]
    truth_subs = [n for n, _ in [(i.node_name, i) for i in node.get_subscriptions_info_by_topic(topics.TRUTH_ODOM)]]
    report['truth_subscribers'] = sorted(set(truth_subs))
    allowed = set(topics.EVALUATION_NODES) | set(topics.SENSOR_MODEL_NODES) | {'gz_bridge', '_ros2cli', 'gps_audit'}
    report['truth_subscriber_violations'] = [n for n in report['truth_subscribers'] if n not in allowed and not n.startswith('_')]
    all_topics = dict(node.get_topic_names_and_types())
    report['gps_topics_present'] = [t for t in topics.GPS_TOPICS if t in all_topics]
    report['expected_autonomy_nodes_seen'] = sorted(report['nodes'])
    node.destroy_node()
    rclpy.shutdown()
    report['pass'] = not report['violations'] and not report['truth_subscriber_violations']
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runtime', action='store_true')
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rep = {'static': static_audit()}
    if a.runtime:
        rep['runtime'] = runtime_audit()
    rep['pass'] = all(v['pass'] for v in rep.values() if isinstance(v, dict))
    txt = json.dumps(rep, indent=2)
    print(txt)
    if a.out:
        Path(a.out).write_text(txt)
    sys.exit(0 if rep['pass'] else 1)


if __name__ == '__main__':
    main()
