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
            'expected_outcome': rs[0].get('expected_outcome'),
            'expectation_met_rate': sum(1 for r in rs if r.get('expectation_met')) / n if n else None,
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


# Reference data-viz palette (fixed categorical order; text never wears series colour)
S1, S2, S3 = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK2, GRID, SURFACE = '#0b0b0b', '#52514e', '#e4e3de', '#fcfcfb'


def _style(plt):
    plt.rcParams.update({
        'figure.dpi': 130, 'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': GRID, 'axes.labelcolor': INK2, 'axes.titlecolor': INK, 'axes.titlesize': 11,
        'axes.titleweight': 'semibold', 'axes.titlelocation': 'left', 'axes.labelsize': 9,
        'xtick.color': INK2, 'ytick.color': INK2, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
        'axes.grid': True, 'axes.grid.axis': 'y', 'grid.color': GRID, 'grid.linewidth': 0.8,
        'axes.spines.top': False, 'axes.spines.right': False, 'axes.spines.left': False,
        'legend.frameon': False, 'legend.fontsize': 8, 'lines.linewidth': 2.0, 'font.size': 9,
    })


def _bars(plt, out, fname, title, ylabel, items, color=S1, ylim=None, fmt='{:.2f}'):
    """One measure per chart, bars sorted as given; value label on each bar (few bars), n/a when unmeasured."""
    if not items:
        return
    fig, ax = plt.subplots(figsize=(max(6.0, 0.42 * len(items) + 2.2), 3.6))
    xs = range(len(items))
    vals = [v if v is not None else 0.0 for _, v in items]
    ax.bar(xs, vals, width=0.62, color=color, edgecolor=SURFACE, linewidth=2)
    for x, (_, v) in zip(xs, items):
        ax.text(x, (v or 0.0), 'n/a' if v is None else fmt.format(v), ha='center', va='bottom', fontsize=7, color=INK2)
    ax.set_xticks(list(xs), [k for k, _ in items], rotation=60, ha='right')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_ylim(*(ylim or (0, max(max(vals) * 1.18, 1.0))))
    fig.tight_layout()
    fig.savefig(out / fname)
    plt.close(fig)


def plots(runs, table, out: Path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _style(plt)
    out.mkdir(parents=True, exist_ok=True)
    scen = list(table.keys())

    le = [r['landing_error_m'] for r in runs if r.get('landing_error_m') is not None]
    if le:
        st = stats(le)
        fig, ax = plt.subplots(figsize=(6.4, 3.6))
        ax.hist(np.array(le) * 100, bins=min(20, max(6, len(le) // 2)), color=S1, edgecolor=SURFACE, linewidth=2)
        for k, ls in (('mean', '--'), ('p95', ':')):
            ax.axvline(st[k] * 100, color=INK2, ls=ls, lw=1.2)
            ax.text(st[k] * 100, ax.get_ylim()[1] * 0.95, f" {k} {st[k] * 100:.1f} cm", color=INK2, fontsize=8, va='top')
        ax.set_xlabel('horizontal landing error (cm)')
        ax.set_ylabel('trials')
        ax.set_title(f'Landing error, all landed trials (n={st["n"]}, RMSE {st["rmse"] * 100:.1f} cm)')
        fig.tight_layout()
        fig.savefig(out / 'landing_error_distribution.png')
        plt.close(fig)
        xy = np.array([r['touchdown_xy'] for r in runs if r.get('touchdown_xy')])
        pad = np.array(runs[0].get('pad_center_xy') or [42, 42])
        fig, ax = plt.subplots(figsize=(4.6, 4.6))
        ax.grid(True, axis='both')
        ax.add_patch(plt.Rectangle((-20, -20), 40, 40, fill=False, ec=INK2, lw=1))
        ax.text(-19, 21, '0.40 m tag', color=INK2, fontsize=8)
        ax.scatter((xy[:, 0] - pad[0]) * 100, (xy[:, 1] - pad[1]) * 100, s=36, color=S1, edgecolor=SURFACE,
                   linewidth=2, zorder=3)
        lim = max(30, float(np.abs((xy - pad) * 100).max()) + 5)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect('equal')
        ax.set_xlabel('x offset from pad centre (cm)')
        ax.set_ylabel('y offset (cm)')
        ax.set_title(f'Touchdown points (n={len(xy)})')
        fig.tight_layout()
        fig.savefig(out / 'touchdown_scatter.png')
        plt.close(fig)

    g = lambda m, f: [(s, table[s][m][f]) for s in scen]  # noqa: E731
    _bars(plt, out, 'landing_rmse_by_scenario.png', 'Landing RMSE by scenario', 'cm',
          [(s, None if v is None else v * 100) for s, v in g('landing_error_m', 'rmse')], fmt='{:.1f}')
    _bars(plt, out, 'min_clearance_by_scenario.png', 'Minimum obstacle clearance (worst trial)', 'm',
          g('min_clearance_m', 'min'), color=S3)
    _bars(plt, out, 'planning_latency_by_scenario.png', 'Planning latency (mean)', 'ms',
          g('planning_latency_ms', 'mean'), color=S2, fmt='{:.0f}')
    _bars(plt, out, 'replanning_latency_by_scenario.png', 'Replanning latency (mean)', 'ms',
          g('replanning_latency_ms', 'mean'), color=S2, fmt='{:.0f}')
    _bars(plt, out, 'path_length_by_scenario.png', 'Executed path length (mean)', 'm',
          g('executed_path_length_m', 'mean'), fmt='{:.0f}')
    _bars(plt, out, 'target_detection_rate.png', 'Tag detection rate during landing (mean)', '%',
          [(s, None if v is None else v * 100) for s, v in g('target_detection_rate', 'mean')], color=S3, fmt='{:.0f}')
    _bars(plt, out, 'fusion_projection_error.png', 'Fusion projection error (LiDAR silhouette to image edge)', 'px',
          g('fusion_edge_chamfer_px', 'mean'), fmt='{:.1f}')
    _bars(plt, out, 'localization_error.png', 'Localisation error vs ground truth (mean)', 'm',
          g('localization_error_mean_m', 'mean'), fmt='{:.3f}')
    _bars(plt, out, 'success_rate_by_scenario.png', 'Expected outcome met (landed, or safely refused when expected)',
          '%', [(s, None if table[s].get('expectation_met_rate') is None else table[s]['expectation_met_rate'] * 100)
                for s in scen], color=S3, ylim=(0, 110), fmt='{:.0f}')
    _bars(plt, out, 'collision_count.png', 'Collisions (total per scenario)', 'count',
          [(s, table[s]['collisions_total']) for s in scen], fmt='{:.0f}')
    pl = flat([r.get('planning_latency_ms') or [] for r in runs])
    rl = flat([r.get('replanning_latency_ms') or [] for r in runs])
    if pl:
        fig, ax = plt.subplots(figsize=(6.4, 3.6))
        bins = np.linspace(0, max(pl + rl) * 1.05, 26)
        ax.hist(pl, bins=bins, color=S1, edgecolor=SURFACE, linewidth=2, label=f'all plans (n={len(pl)})')
        if rl:
            ax.hist(rl, bins=bins, color=S2, edgecolor=SURFACE, linewidth=2, alpha=0.9, label=f'replans (n={len(rl)})')
        ax.set_xlabel('planner service latency (ms)')
        ax.set_ylabel('count')
        ax.set_title('Planning latency distribution')
        ax.legend()
        fig.tight_layout()
        fig.savefig(out / 'planning_latency_hist.png')
        plt.close(fig)


def trajectory_plot(raw: Path, run_dir: str, out: Path):
    """Planned vs actual (truth) top view + spatial / localisation error over time for one run."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import yaml
    _style(plt)
    tel = raw / run_dir / 'telemetry.csv'
    if not tel.exists():
        return
    rows = [r for r in csv.DictReader(open(tel)) if r['state'] not in ('INIT', 'ARMING', 'LANDED')]
    if len(rows) < 10:
        return
    f = lambda r, k: float(r[k]) if r[k] not in ('', 'None') else np.nan  # noqa: E731
    t = np.array([f(r, 't') for r in rows])
    tx, ty = np.array([f(r, 'truth_x') for r in rows]), np.array([f(r, 'truth_y') for r in rows])
    rx, ry = np.array([f(r, 'ref_x') for r in rows]), np.array([f(r, 'ref_y') for r in rows])
    te = np.array([f(r, 'true_E') for r in rows])
    le = np.array([f(r, 'loc_err') for r in rows])
    w = yaml.safe_load(open(ROOT / 'simulation/configs/world_ps05.yaml'))
    fig, ax = plt.subplots(figsize=(6.2, 6.2))
    ax.grid(True, axis='both')
    for o in w['obstacles']:
        x, y, _ = o['pos']
        if o['type'] == 'box':
            sx, sy, _ = o['size']
            ax.add_patch(plt.Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='#b9b8b2', lw=0))
        else:
            ax.add_patch(plt.Circle((x, y), o['radius'], color='#b9b8b2', lw=0))
    px, py = w['landing_pad']['center']
    ax.add_patch(plt.Rectangle((px - 1.5, py - 1.5), 3, 3, fill=False, ec=INK2, lw=1))
    ax.text(px - 1.5, py + 1.8, 'landing pad', color=INK2, fontsize=8)
    ax.plot(rx, ry, color=S2, lw=2, ls='--', label='reference trajectory')
    ax.plot(tx, ty, color=S1, lw=2, label='actual path (ground truth)')
    ax.set_xlim(0, 50)
    ax.set_ylim(0, 50)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='lower right')
    ax.set_title(f'Planned vs actual path (top view), {run_dir}')
    fig.tight_layout()
    fig.savefig(out / f'trajectory_{run_dir}.png')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(7.2, 3.4))
    t0 = t[0]
    ax.plot(t - t0, te, color=S1, lw=2, label='spatial error E (truth vs reference, NAVIGATE)')
    ok = np.isfinite(le)
    ax.plot((t - t0)[ok], le[ok], 'o', ms=3, color=S3, label='localisation error (estimate vs truth)')
    ax.axhline(0.5, color=INK2, ls=':', lw=1.2)
    ax.text((t - t0)[-1], 0.52, 'PS05 target: mean E < 0.5 m ', color=INK2, fontsize=8, ha='right', va='bottom')
    ax.set_xlabel('time since take-off (s)')
    ax.set_ylabel('m')
    ax.set_title(f'Path-tracking and localisation error, {run_dir}')
    ax.legend(loc='upper left')
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
