#!/usr/bin/env python3
"""Aggregate results/raw/*/metrics.json -> results/summary.{csv,json}, results/processed/*.csv, results/plots/*.png.

Only measured values are used. Runs without metrics.json are counted as missing
and never imputed. Usage: aggregate_results.py [--prefix TAG] [--root results]
"""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def stats(vals):
    v = np.array([x for x in vals if x is not None and isinstance(x, (int, float)) and math.isfinite(x)], float)
    if len(v) == 0:
        return {'n': 0, 'mean': None, 'median': None, 'rmse': None, 'p95': None, 'min': None, 'max': None, 'std': None}
    return {'n': int(len(v)), 'mean': float(v.mean()), 'median': float(np.median(v)), 'rmse': float(np.sqrt((v ** 2).mean())),
            'p95': float(np.percentile(v, 95)), 'min': float(v.min()), 'max': float(v.max()), 'std': float(v.std())}


def flat(lst):
    out = []
    for x in lst:
        if isinstance(x, list):
            out.extend(y for y in x if y is not None)
        elif x is not None:
            out.append(x)
    return out


def load_runs(raw: Path, prefix: str | None):
    runs = []
    for d in sorted(raw.iterdir()):
        if not d.is_dir() or (prefix and not d.name.startswith(prefix)):
            continue
        f = d / 'metrics.json'
        if f.exists():
            try:
                m = json.loads(f.read_text())
                m['_dir'] = d.name
                runs.append(m)
            except json.JSONDecodeError:
                pass
    return runs


def summarize(runs):
    by = defaultdict(list)
    for r in runs:
        by[r.get('scenario_id') or 'unknown'].append(r)
    table = {}
    for scen, rs in sorted(by.items()):
        n = len(rs)
        landed = [r for r in rs if r.get('outcome') == 'LANDED']
        table[scen] = {
            'trials': n,
            'landed': len(landed),
            'success_rate': sum(1 for r in rs if r.get('success')) / n if n else None,
            'outcomes': {o: sum(1 for r in rs if r.get('outcome') == o) for o in sorted({str(r.get('outcome')) for r in rs})},
            'collisions_total': sum(int(r.get('collision_count') or 0) for r in rs),
            'collision_rate': sum(1 for r in rs if (r.get('collision_count') or 0) > 0) / n if n else None,
            'near_misses_total': sum(int(r.get('near_misses') or 0) for r in rs),
            'landing_error_m': stats([r.get('landing_error_m') for r in landed]),
            'min_clearance_m': stats([r.get('min_clearance_m') for r in rs]),
            'mission_time_s': stats([r.get('mission_time_s') for r in rs]),
            'executed_path_length_m': stats([r.get('executed_path_length_m') for r in rs]),
            'planned_path_length_m': stats(flat([(r.get('planned_path_lengths_m') or [None])[:1] for r in rs])),
            'planning_latency_ms': stats(flat([r.get('planning_latency_ms') or [] for r in rs])),
            'replanning_latency_ms': stats(flat([r.get('replanning_latency_ms') or [] for r in rs])),
            'replans': stats([r.get('replans') for r in rs]),
            'stop_response_s': stats(flat([r.get('stop_response_s') or [] for r in rs])),
            'target_detection_rate': stats([r.get('target_detection_rate') for r in rs]),
            'target_reacquisition_s': stats(flat([r.get('target_reacquisition_s') or [] for r in rs])),
            'localization_error_mean_m': stats([(r.get('localization_error') or {}).get('mean') for r in rs]),
            'true_spatial_error_mean_m': stats([(r.get('true_spatial_error') or {}).get('mean') for r in rs]),
            'true_spatial_error_max_m': stats([(r.get('true_spatial_error') or {}).get('max') for r in rs]),
            'fusion_edge_chamfer_px': stats([r.get('fusion_edge_chamfer_px') for r in rs]),
            'fusion_extrinsic_shift_px': stats([r.get('fusion_extrinsic_shift_px') for r in rs]),
            'fusion_sync_mean_abs_delta_s': stats([r.get('fusion_sync_mean_abs_delta_s') for r in rs]),
            'aborts': stats([r.get('aborts') for r in rs]),
            'runs': [r['_dir'] for r in rs],
        }
    return table


def write_csv(table, path):
    cols = ['scenario', 'trials', 'landed', 'success_rate', 'collisions_total', 'near_misses_total',
            'landing_err_mean', 'landing_err_median', 'landing_err_rmse', 'landing_err_p95', 'min_clearance_min',
            'mission_time_mean', 'path_len_mean', 'plan_latency_mean_ms', 'replan_latency_mean_ms', 'replans_mean',
            'detection_rate_mean', 'reacq_mean_s', 'loc_err_mean', 'true_E_mean', 'fusion_chamfer_px', 'extrinsic_shift_px']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        for s, t in table.items():
            g = lambda k, f='mean': t[k][f]  # noqa: E731
            w.writerow([s, t['trials'], t['landed'], t['success_rate'], t['collisions_total'], t['near_misses_total'],
                        g('landing_error_m'), g('landing_error_m', 'median'), g('landing_error_m', 'rmse'),
                        g('landing_error_m', 'p95'), g('min_clearance_m', 'min'), g('mission_time_s'),
                        g('executed_path_length_m'), g('planning_latency_ms'), g('replanning_latency_ms'), g('replans'),
                        g('target_detection_rate'), g('target_reacquisition_s'), g('localization_error_mean_m'),
                        g('true_spatial_error_mean_m'), g('fusion_edge_chamfer_px'), g('fusion_extrinsic_shift_px')])


def plots(runs, table, out: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'figure.dpi': 120, 'axes.grid': True, 'grid.alpha': 0.3, 'axes.spines.top': False,
                         'axes.spines.right': False})
    blue, orange, green, red = '#2a78d4', '#e08a1e', '#2e9e5b', '#d03b3b'
    scen = list(table.keys())

    le = [r['landing_error_m'] for r in runs if r.get('landing_error_m') is not None]
    if le:
        fig, ax = plt.subplots(figsize=(6, 3.6))
        ax.hist(np.array(le) * 100, bins=min(20, max(5, len(le) // 2)), color=blue, edgecolor='white')
        st = stats(le)
        for k, c in (('mean', orange), ('p95', red)):
            ax.axvline(st[k] * 100, color=c, ls='--', label=f"{k} {st[k]*100:.1f} cm")
        ax.set_xlabel('horizontal landing error (cm)')
        ax.set_ylabel('trials')
        ax.set_title(f'Landing error distribution (n={len(le)}, RMSE {st["rmse"]*100:.1f} cm)')
        ax.legend()
        fig.tight_layout()
        fig.savefig(out / 'landing_error_distribution.png')
        plt.close(fig)
        # touchdown scatter
        xy = np.array([r['touchdown_xy'] for r in runs if r.get('touchdown_xy')])
        pad = np.array(runs[0].get('pad_center_xy') or [42, 42])
        fig, ax = plt.subplots(figsize=(4.4, 4.4))
        ax.add_patch(plt.Rectangle((-150, -150), 300, 300, fill=False, ec='grey', lw=1, label='3 m pad'))
        ax.add_patch(plt.Rectangle((-20, -20), 40, 40, fill=False, ec='k', lw=1, label='0.4 m tag'))
        ax.scatter((xy[:, 0] - pad[0]) * 100, (xy[:, 1] - pad[1]) * 100, s=22, color=blue, zorder=3, label='touchdown')
        ax.set_xlim(-60, 60)
        ax.set_ylim(-60, 60)
        ax.set_aspect('equal')
        ax.set_xlabel('x error (cm)')
        ax.set_ylabel('y error (cm)')
        ax.set_title('Touchdown points vs pad centre')
        ax.legend(fontsize=7, loc='upper right')
        fig.tight_layout()
        fig.savefig(out / 'touchdown_scatter.png')
        plt.close(fig)

    def bar(metric, field, label, fname, scale=1.0, color=blue):
        vals = [(s, table[s][metric][field]) for s in scen if table[s][metric][field] is not None]
        if not vals:
            return
        fig, ax = plt.subplots(figsize=(max(6, 0.45 * len(vals) + 2), 3.6))
        ax.bar([v[0] for v in vals], [v[1] * scale for v in vals], color=color)
        ax.set_ylabel(label)
        ax.tick_params(axis='x', rotation=60, labelsize=7)
        fig.tight_layout()
        fig.savefig(out / fname)
        plt.close(fig)

    bar('landing_error_m', 'rmse', 'landing RMSE (cm)', 'landing_rmse_by_scenario.png', 100)
    bar('min_clearance_m', 'min', 'minimum clearance (m)', 'min_clearance_by_scenario.png', 1, green)
    bar('planning_latency_ms', 'mean', 'planning latency (ms)', 'planning_latency_by_scenario.png', 1, orange)
    bar('replanning_latency_ms', 'mean', 'replanning latency (ms)', 'replanning_latency_by_scenario.png', 1, orange)
    bar('executed_path_length_m', 'mean', 'executed path length (m)', 'path_length_by_scenario.png')
    bar('target_detection_rate', 'mean', 'target detection rate', 'target_detection_rate.png', 1, green)
    bar('fusion_edge_chamfer_px', 'mean', 'edge chamfer (px)', 'fusion_projection_error.png', 1, red)
    bar('localization_error_mean_m', 'mean', 'mean localisation error (m)', 'localization_error.png', 1, blue)
    sr = [(s, table[s]['success_rate']) for s in scen if table[s]['success_rate'] is not None]
    if sr:
        fig, ax = plt.subplots(figsize=(max(6, 0.45 * len(sr) + 2), 3.6))
        ax.bar([s for s, _ in sr], [v * 100 for _, v in sr], color=green)
        ax.set_ylabel('success rate (%)')
        ax.set_ylim(0, 105)
        ax.tick_params(axis='x', rotation=60, labelsize=7)
        fig.tight_layout()
        fig.savefig(out / 'success_rate_by_scenario.png')
        plt.close(fig)
    col = [(s, table[s]['collisions_total']) for s in scen]
    fig, ax = plt.subplots(figsize=(max(6, 0.45 * len(col) + 2), 3.2))
    ax.bar([s for s, _ in col], [v for _, v in col], color=red)
    ax.set_ylabel('collisions (count)')
    ax.tick_params(axis='x', rotation=60, labelsize=7)
    fig.tight_layout()
    fig.savefig(out / 'collision_count.png')
    plt.close(fig)
    pl = flat([r.get('planning_latency_ms') or [] for r in runs])
    rl = flat([r.get('replanning_latency_ms') or [] for r in runs])
    if pl:
        fig, ax = plt.subplots(figsize=(6, 3.4))
        ax.hist(pl, bins=25, color=orange, alpha=0.8, label=f'all plans (n={len(pl)})')
        if rl:
            ax.hist(rl, bins=25, color=red, alpha=0.7, label=f'replans (n={len(rl)})')
        ax.set_xlabel('latency (ms)')
        ax.set_ylabel('count')
        ax.legend()
        fig.tight_layout()
        fig.savefig(out / 'planning_latency_hist.png')
        plt.close(fig)


def trajectory_plot(raw: Path, run_dir: str, out: Path):
    """Planned vs actual (truth) top view + spatial error over time for one run."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import yaml
    tel = raw / run_dir / 'telemetry.csv'
    if not tel.exists():
        return
    rows = list(csv.DictReader(open(tel)))
    if len(rows) < 10:
        return
    f = lambda r, k: float(r[k]) if r[k] not in ('', 'None') else np.nan  # noqa: E731
    t = np.array([f(r, 't') for r in rows])
    tx, ty = np.array([f(r, 'truth_x') for r in rows]), np.array([f(r, 'truth_y') for r in rows])
    rx, ry = np.array([f(r, 'ref_x') for r in rows]), np.array([f(r, 'ref_y') for r in rows])
    te = np.array([f(r, 'true_E') for r in rows])
    le = np.array([f(r, 'loc_err') for r in rows])
    w = yaml.safe_load(open(ROOT / 'simulation/configs/world_ps05.yaml'))
    fig, ax = plt.subplots(figsize=(6, 6))
    for o in w['obstacles']:
        x, y, _ = o['pos']
        if o['type'] == 'box':
            sx, sy, _ = o['size']
            ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='#888', alpha=0.6))
        else:
            ax.add_patch(plt.Circle((x, y), o['radius'], color='#888', alpha=0.6))
    px, py = w['landing_pad']['center']
    ax.add_patch(plt.Rectangle((px - 1.5, py - 1.5), 3, 3, fill=False, ec='k'))
    ax.plot(rx, ry, '--', color='#e08a1e', lw=1.5, label='reference (trajectory)')
    ax.plot(tx, ty, color='#2a78d4', lw=1.5, label='actual (ground truth)')
    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_aspect('equal')
    ax.legend(loc='lower right')
    ax.set_title(f'Planned vs actual path: {run_dir}')
    fig.tight_layout()
    fig.savefig(out / f'trajectory_{run_dir}.png')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 3.2))
    ax.plot(t - t[0], te, color='#2a78d4', label='true spatial error E (truth vs reference)')
    ax.plot(t - t[0], le, color='#2e9e5b', alpha=0.7, label='localisation error (estimate vs truth)')
    ax.axhline(0.5, color='#e08a1e', ls=':', label='PS05 target mean E < 0.5 m')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('m')
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / f'spatial_error_{run_dir}.png')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--prefix', default=None)
    ap.add_argument('--root', default=str(ROOT / 'results'))
    ap.add_argument('--trajectory-run', default=None)
    a = ap.parse_args()
    root = Path(a.root)
    raw = root / 'raw'
    runs = load_runs(raw, a.prefix)
    table = summarize(runs)
    overall = {
        'total_trials': len(runs),
        'landing_error_m_all_landed': stats([r.get('landing_error_m') for r in runs if r.get('outcome') == 'LANDED']),
        'collisions_total': sum(int(r.get('collision_count') or 0) for r in runs),
    }
    (root / 'processed').mkdir(parents=True, exist_ok=True)
    json.dump({'overall': overall, 'by_scenario': table}, open(root / 'summary.json', 'w'), indent=2)
    write_csv(table, root / 'summary.csv')
    with open(root / 'processed' / 'all_runs.csv', 'w', newline='') as f:
        keys = ['_dir', 'scenario_id', 'seed', 'outcome', 'success', 'landing_error_m', 'collision_count', 'near_misses',
                'min_clearance_m', 'mission_time_s', 'executed_path_length_m', 'replans', 'target_detection_rate', 'aborts']
        w = csv.writer(f)
        w.writerow(keys)
        for r in runs:
            w.writerow([r.get(k) for k in keys])
    plots(runs, table, root / 'plots')
    tr = a.trajectory_run or next((r['_dir'] for r in reversed(runs) if r.get('outcome') == 'LANDED'), None)
    if tr:
        trajectory_plot(raw, tr, root / 'plots')
    print(json.dumps({'trials': len(runs), 'scenarios': {s: {'trials': t['trials'], 'success_rate': t['success_rate'],
                                                             'landing_rmse_m': t['landing_error_m']['rmse']}
                                                         for s, t in table.items()}}, indent=1))


if __name__ == '__main__':
    main()
