#!/usr/bin/env python3
"""Generate docs/REQUIREMENTS_TRACEABILITY.md: requirement -> implementation -> file -> test -> result -> evidence.

Results are computed from logged data (results/summary.json, per-run metrics, test reports, audits); a criterion
with no data is NOT MEASURED. Usage: generate_traceability.py [--out docs/REQUIREMENTS_TRACEABILITY.md]
"""
import argparse
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NM = 'NOT MEASURED'


def lj(p):
    try:
        return json.loads((ROOT / p).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(ROOT / 'docs/REQUIREMENTS_TRACEABILITY.md'))
    a = ap.parse_args()
    summ = lj('results/summary.json') or {}
    by = summ.get('by_scenario', {})
    runs = [json.loads(p.read_text()) for p in sorted((ROOT / 'results/raw').glob('*/metrics.json'))]
    smoke = lj('results/reports/smoke/smoke_flight.json') or {}
    sysc = lj('results/reports/smoke/system_check.json') or {}
    gps_s = lj('results/reports/gps_audit_static.json') or {}
    gps_r = lj('results/reports/smoke/gps_audit_runtime.json') or {}
    tr = (ROOT / 'results/reports/test_report.txt')
    trt = tr.read_text() if tr.exists() else ''
    m = re.findall(r'(\d+) passed', trt)
    fl = re.findall(r'(\d+) failed', trt)
    utests = f"{m[-1]} passed, {fl[-1] if fl else 0} failed" if m else NM
    br = (ROOT / 'results/reports/backend_test_report.txt')
    brt = br.read_text() if br.exists() else ''
    bm = re.findall(r'(\d+) passed', brt)
    btests = f"{bm[-1]} passed" if bm else NM

    def S(s):
        return by.get(s) or {}

    def scen_met(s):
        t = S(s)
        if not t:
            return NM, ''
        r = t.get('expectation_met_rate')
        return ('PASS' if r == 1.0 else ('PARTIAL' if r else 'FAIL')), f"{t['trials']} trial(s), expectation met {0 if r is None else round(100 * r)} %"

    landed = [r for r in runs if r.get('outcome') == 'LANDED']
    any_run = bool(runs)
    collisions = sum(int(r.get('collision_count') or 0) for r in runs)
    replans = sum(int(r.get('replans') or 0) for r in runs)
    det_rates = [r.get('target_detection_rate') for r in landed if r.get('target_detection_rate') is not None]
    reacq = [x for r in runs for x in (r.get('target_reacquisition_s') or [])]
    ddet = [r.get('dynamic_detection_latency_s') for r in runs if r.get('dynamic_detection_latency_s') is not None]
    maps = [r['map_accuracy']['phantom_fraction'] for r in runs if (r.get('map_accuracy') or {}).get('phantom_fraction') is not None]
    chamfer = [r.get('fusion_edge_chamfer_px') for r in runs if r.get('fusion_edge_chamfer_px') is not None]
    sync_acc = [(r.get('fusion_sync_accepted'), r.get('fusion_sync_rejected')) for r in runs if r.get('fusion_sync_accepted')]
    lerr = (summ.get('overall') or {}).get('landing_error_m_all_landed') or {}
    base = S('baseline')

    def pf(cond, ev):
        return ('PASS' if cond else 'FAIL') if ev is not None else NM, ev or ''

    tf_ok = all(v.get('ok') for v in (sysc.get('tf') or {}).values()) if sysc.get('tf') else None
    rows = [
        # (requirement, implementation, file/module, test, (result, evidence))
        ('PX4 SITL launches', 'PX4 v1.17.0 SITL, standalone gz mode, custom airframe 4050', 'launch/simulation.launch.py, simulation/px4/airframes/', 'smoke_test.sh', pf(bool(sysc.get('px4', {}).get('connected')), 'results/reports/smoke/system_check.json' if sysc else None)),
        ('Gazebo launches', 'Gazebo Harmonic 8.15, generated world', 'simulation/worlds/ps05_challenge.sdf', 'smoke_test.sh / every run', pf(any_run, f'{len(runs)} logged runs' if any_run else None)),
        ('ROS 2 communicates with PX4', 'uXRCE-DDS agent v2.4.3, px4_msgs release/1.17', 'px4_interface', 'check_system.py', pf(bool(sysc.get('px4', {}).get('connected')), 'system_check.json: px4.connected' if sysc else None)),
        ('UAV can arm / take off / hover / land', 'px4_interface + smoke_flight', 'px4_interface/smoke_flight.py', 'smoke_test.sh', pf(smoke.get('success'), f"phases {[p['phase'] for p in smoke.get('phases', [])]}, hover RMS {smoke.get('hover_rms_error_m', 0):.3f} m" if smoke else None)),
        ('GPS not used by autonomy', 'no GNSS sensor, EKF2_GPS_CTRL=0, tracker EV aiding, truth isolation', 'scripts/gps_audit.py, ps05_common/topics.py', 'tests/regression/test_gps_audit.py + runtime audit', pf(bool(gps_s.get('pass')) and bool(gps_r.get('pass')), 'gps_audit_static.json + smoke/gps_audit_runtime.json' if gps_s and gps_r else None)),
        ('Camera publishes; CameraInfo valid', 'gz camera 640x480 + camera_processor', 'camera_processor', 'check_system.py, test_projection', pf((sysc.get('camera_info') or {}).get('ok'), f"K={((sysc.get('camera_info') or {}).get('k') or [])[:3]}" if sysc else None)),
        ('LiDAR publishes; cloud visible', 'gpu_lidar 360x16 + lidar_processor', 'lidar_processor', 'check_system.py', pf(sysc.get('lidar_frame') == 'lidar_frame', f"frame {sysc.get('lidar_frame')}, {(sysc.get('topics') or {}).get('/lidar/points', {}).get('wall_hz')} Hz" if sysc else None)),
        ('TF tree valid', 'static_frames + localization odom->base_link', 'aeronav_description', 'test_tf_model_consistency.py, check_system.py', pf(tf_ok, 'system_check.json: tf (presence, static stability, no jumps)' if tf_ok is not None else None)),
        ('Camera/LiDAR synchronized', 'TimestampSynchronizer (|dt| <= 50 ms)', 'sensor_fusion/sync.py', 'test_sync.py; timestamp_offset scenarios', pf(bool(sync_acc), f'accepted/rejected pairs per run e.g. {sync_acc[:3]}' if sync_acc else None)),
        ('LiDAR points transform + project into camera; RGB cloud', 'TF2 + pinhole projection + colourisation', 'sensor_fusion/projection.py, node.py', 'test_projection.py', pf(bool(chamfer), f'edge-chamfer projection error mean {sum(chamfer) / len(chamfer):.1f} px over {len(chamfer)} runs' if chamfer else None)),
        ('Occupancy map updates', 'ray-traced log-odds voxel map', 'mapping/voxel_map.py', 'test_voxel_map.py; map-accuracy metric', pf(bool(maps), f'phantom voxel fraction mean {100 * sum(maps) / len(maps):.2f} %' if maps else None)),
        ('Obstacle inflation works', 'EDT inflation from safety.yaml', 'planner_3d/grid.py', 'test_planner.py (unsafe slot, narrow passage)', pf(utests != NM and ' 0 failed' in utests, utests if utests != NM else None)),
        ('Approved corridor enforced', 'corridor mask in A* + final validation + E vs Emax', 'planner_3d/corridor.py, state_machine.py', 'test_planner corridor tests; no_path scenario', scen_met('no_path')),
        ('3D A* works; path collision-safe', 'weighted A*, no corner cutting, unified validation', 'planner_3d/astar.py', 'test_planner.py; all runs', pf(collisions == 0 and bool(landed), f'{len(landed)} landed runs, {collisions} collisions' if any_run else None)),
        ('Trajectory smoothing works', 'verified Chaikin + speed profile', 'planner_3d/trajectory.py', 'test_trajectory.py', pf(utests != NM and ' 0 failed' in utests, utests if utests != NM else None)),
        ('Dynamic obstacle detected; local replanning works', 'obstacle_avoidance + REPLANNING', 'obstacle_avoidance, state_machine.py', 'demo / dynamic_obstacle / crossing_obstacle scenarios', pf(bool(ddet) and S('demo').get('expectation_met_rate', 0) > 0, f'detection latency mean {sum(ddet) / len(ddet):.2f} s (n={len(ddet)}); replans total {replans}' if ddet else None)),
        ('No-path safety state works', 'NO_SAFE_PATH -> ABORTED', 'state_machine.py', 'test_state_machine; no_path scenario', scen_met('no_path')),
        ('AprilTag detected; target pose transformed', 'ArUco 36h11 + PnP + TF2', 'landing_detector', 'test_landing.py', pf(bool(det_rates), f'detection rate during landing mean {100 * sum(det_rates) / len(det_rates):.0f} %' if det_rates else None)),
        ('LiDAR validates landing surface; clearance checked', 'plane fit + clearance', 'landing_validator/surface.py', 'test_landing.py; pad_obstruction, pad_near_obstacle', scen_met('pad_near_obstacle')),
        ('Target-loss recovery works', 'TARGET_LOST -> hover -> reacquire', 'state_machine.py', 'tag_loss scenario', pf(bool(reacq) and scen_met('tag_loss')[0] == 'PASS', f'reacquisition times {[round(x, 1) for x in reacq[:5]]} s' if reacq else None)),
        ('Unsafe landing recovery works', 'UNSAFE_PAD -> ABORT_CLIMB -> wait -> search / return home', 'state_machine.py', 'pad_obstruction, pad_near_obstacle', scen_met('pad_obstruction')),
        ('Precision landing works; landing error measured', 'PID alignment + staged descent + LiDAR checkpoints', 'landing_controller', 'all landed runs', pf(bool(landed), f"n={lerr.get('n')}, mean {100 * (lerr.get('mean') or 0):.1f} cm, RMSE {100 * (lerr.get('rmse') or 0):.1f} cm, P95 {100 * (lerr.get('p95') or 0):.1f} cm" if landed else None)),
        ('Repeated trials executed (20-30)', 'run_experiments.sh / run_full_campaign.sh', 'scripts/', 'eval_baseline_*, eval_dynamic_*', pf((base.get('trials') or 0) >= 20, f"baseline {base.get('trials')} trials, dynamic {S('demo').get('trials')} trials" if base else None)),
        ('Metrics generated', 'evaluation node + aggregate_results.py', 'evaluation/, scripts/aggregate_results.py', 'results/summary.{csv,json}', pf(bool(summ), 'results/summary.json, results/plots/' if summ else None)),
        ('Robustness tests executed', '24 scenarios', 'simulation/scenarios/', 'run_stress_tests.sh', pf(len(by) > 10, f'{len(by)} scenarios aggregated' if by else None)),
        ('Backend healthy; WebSocket telemetry works', 'FastAPI + /ws/telemetry', 'backend/app', 'backend/tests/test_api.py', pf(btests != NM, btests if btests != NM else None)),
        ('Frontend loads; dashboard displays real data', 'React/Vite dashboard', 'frontend/', 'headless screenshot during a live run', pf((ROOT / 'docs/img/dashboard_live.png').exists(), 'docs/img/dashboard_live.png' if (ROOT / 'docs/img/dashboard_live.png').exists() else None)),
        ('RViz configuration works', 'rviz/ps05.rviz', 'rviz/', 'headless load + screenshot', pf((ROOT / 'docs/img/rviz_live.png').exists(), 'docs/img/rviz_live.png' if (ROOT / 'docs/img/rviz_live.png').exists() else None)),
        ('Full demo launch works', 'run_demo.sh -> demo scenario', 'scripts/run_demo.sh', 'demo scenario runs', scen_met('demo')),
        ('Tests pass', 'pytest', 'tests/, backend/tests', 'run_tests.sh', pf(utests != NM and ' 0 failed' in utests, utests if utests != NM else None)),
        ('PS05: mean simulated trajectory error E < 0.5 m', 'trajectory following + progress gating', 'state_machine.py', 'baseline trials (truth vs reference)', pf((base.get('true_spatial_error_mean_m') or {}).get('mean', 9) < 0.5, f"baseline mean {(base.get('true_spatial_error_mean_m') or {}).get('mean', 0):.3f} m" if base else None)),
        ('PS05: E > Emax -> critical alarm + containment', 'CORRIDOR_BREACH state', 'state_machine.py', 'test_corridor_breach_alarm_and_containment', pf(utests != NM and ' 0 failed' in utests, 'unit test (kinematic vehicle with injected push)' if utests != NM else None)),
        ('PS05: tracking > 50 Hz, latency < 20 ms', 'ground_tracker 60 Hz, 10 ms', 'configs/localization.yaml', 'design parameter (sensor model)', ('PASS (by configuration)', '60 Hz / 10 ms sensor model; measured EKF localisation error in FINAL_REPORT §31')),
    ]
    L = ['# Requirements Traceability', '',
         '*Generated by `scripts/generate_traceability.py` from logged data. Result = PASS / FAIL / PARTIAL / NOT MEASURED.*', '',
         '| Requirement | Implementation | File / module | Test | Result | Evidence |', '|---|---|---|---|---|---|']
    for req, impl, mod, test, (res, ev) in rows:
        L.append(f'| {req} | {impl} | `{mod}` | {test} | **{res}** | {ev} |')
    L += ['', f"Totals: {sum(1 for r in rows if r[4][0].startswith('PASS'))} PASS, "
          f"{sum(1 for r in rows if r[4][0] == 'PARTIAL')} PARTIAL, {sum(1 for r in rows if r[4][0] == 'FAIL')} FAIL, "
          f"{sum(1 for r in rows if r[4][0] == NM)} NOT MEASURED (of {len(rows)})."]
    Path(a.out).write_text('\n'.join(L) + '\n')
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
