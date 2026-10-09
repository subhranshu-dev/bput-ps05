#!/usr/bin/env python3
"""Re-evaluate clearance / collisions / near misses for logged runs with the yaw-aware world geometry.

Evaluator v1 (before the fix in evaluation/metrics.py) ignored the yaw of rotated scenario objects
(dyn_panel), which produced a false collision in eval_dynamic_t10. This script recomputes the three
metrics from each run's own logged data — telemetry.csv (10 Hz ground truth, identical samples the
evaluator used) and events.jsonl (scenario object poses incl. yaw) — and updates metrics.json,
keeping the original values under evaluator_v1_* keys.  Usage: reevaluate_clearance.py [results/raw]
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/evaluation'))
from evaluation.metrics import load_world, min_clearance  # noqa: E402

AIRBORNE_EXCLUDE = ('INIT', 'ARMING', 'LANDED')


def reevaluate(run: Path, spec: dict, vehicle_r: float, near_r: float) -> dict | None:
    tel, evf, mf = run / 'telemetry.csv', run / 'events.jsonl', run / 'metrics.json'
    if not (tel.exists() and mf.exists()):
        return None
    moves = []
    if evf.exists():
        for line in open(evf):
            e = json.loads(line)
            if e.get('type') == 'object_pose':
                moves.append((float(e['stamp']), e['object'], e['pos'], float(e.get('yaw', 0.0))))
    moves.sort()
    prims = load_world(spec)
    by_name = {o['name']: o for o in prims}
    k = 0
    best, best_obj, coll, near = float('inf'), None, 0, 0
    in_coll = in_near = False
    for r in csv.DictReader(open(tel)):
        t = float(r['t'])
        while k < len(moves) and moves[k][0] <= t:
            _, name, pos, yaw = moves[k]
            if name in by_name:
                by_name[name]['pos'] = np.array(pos, dtype=float)
                by_name[name]['yaw'] = yaw
            k += 1
        if r['state'] in AIRBORNE_EXCLUDE:
            continue
        p = np.array([float(r['truth_x']), float(r['truth_y']), float(r['truth_z'])])
        c, name = min_clearance(p, prims)
        if c < best:
            best, best_obj = c, name
        if c < vehicle_r:
            coll += 0 if in_coll else 1
            in_coll = True
        else:
            in_coll = False
        if c < near_r:
            near += 0 if in_near else 1
            in_near = True
        else:
            in_near = False
    m = json.loads(mf.read_text())
    if 'evaluator_v1_min_clearance_m' not in m:
        for key in ('min_clearance_m', 'min_clearance_object', 'collision_count', 'near_misses'):
            m['evaluator_v1_' + key] = m.get(key)
    m.update({'min_clearance_m': None if best == float('inf') else best, 'min_clearance_object': best_obj,
              'collision_count': coll, 'near_misses': near, 'clearance_reevaluated': 'yaw-aware geometry (v2)'})
    m['success'] = m.get('outcome') == 'LANDED' and coll == 0
    exp = m.get('expected_outcome', 'LANDED')
    if 'expectation_met' in m:
        m['expectation_met'] = coll == 0 and ((exp == 'LANDED' and m.get('outcome') == 'LANDED')
                                              or (exp == 'ABORTED' and m.get('state') == 'ABORTED'))
    mf.write_text(json.dumps(m, indent=2, default=str))
    return {'run': run.name, 'v1': (m['evaluator_v1_min_clearance_m'], m['evaluator_v1_collision_count']),
            'v2': (m['min_clearance_m'], coll)}


def main():
    raw = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'results/raw'
    spec = yaml.safe_load(open(ROOT / 'simulation/configs/world_ps05.yaml'))
    saf = yaml.safe_load(open(ROOT / 'configs/safety.yaml'))['inflation']
    vr, nr = float(saf['vehicle_radius']), float(saf['vehicle_radius'] + saf['desired_clearance'])
    changed = 0
    for run in sorted(raw.iterdir()):
        r = reevaluate(run, spec, vr, nr) if run.is_dir() else None
        if r and (r['v1'][1] != r['v2'][1] or abs((r['v1'][0] or 0) - (r['v2'][0] or 0)) > 0.005):
            changed += 1
            print(f"{r['run']}: min clearance {r['v1'][0]:.2f} -> {r['v2'][0]:.2f} m, collisions {r['v1'][1]} -> {r['v2'][1]}")
    print(f'[reevaluate] done; {changed} run(s) changed')


if __name__ == '__main__':
    main()
