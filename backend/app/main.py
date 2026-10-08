"""BPUT PS05 dashboard backend (FastAPI).

* /api/ingest      <- dashboard_bridge (read-only ROS forwarder) posts snapshots/images/clouds
* REST endpoints   -> live state, sensors, planner, landing, metrics, experiments, logs
* /ws/telemetry    -> live stream for the frontend (5 Hz snapshots, images, clouds)
* experiment runs  -> spawns scripts/run_experiments.sh (one at a time)

The backend observes and controls *experiments*; it is never an input to the
autonomy stack (the stack runs identically without it).
"""
import asyncio
import json
import logging
import os
import signal
import subprocess
import time
from collections import deque
from pathlib import Path

import yaml
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .schemas import ExperimentStart, Ingest, ScenarioLoad

ROOT = Path(os.environ.get('PS05_ROOT', Path(__file__).resolve().parents[2]))
RESULTS = ROOT / 'results'
SCENARIOS = ROOT / 'simulation' / 'scenarios'
CFG = yaml.safe_load(open(ROOT / 'configs' / 'dashboard.yaml'))['backend']

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s %(message)s')
log = logging.getLogger('ps05.backend')

app = FastAPI(title='BPUT PS05 Ground Control API', version='1.0.0')
app.add_middleware(CORSMiddleware, allow_origins=CFG['cors_origins'], allow_methods=['*'], allow_headers=['*'])


class Store:
    def __init__(self):
        self.snapshot: dict = {}
        self.images: dict = {}
        self.clouds: dict = {}
        self.last_ingest = 0.0
        self.history: deque = deque(maxlen=3000)
        self.events: deque = deque(maxlen=500)
        self.seen_events: set = set()
        self.ingest_count = 0
        self.ws_clients: set = set()
        self.experiment: dict | None = None
        self.proc: subprocess.Popen | None = None
        self.scenario = 'baseline'


S = Store()


def _history_point(snap: dict) -> dict:
    m = snap.get('mission') or {}
    met = snap.get('metrics') or {}
    planner = snap.get('planner') or {}
    loc = (met.get('localization_error') or {})
    return {'t': snap.get('ros_time'), 'state': m.get('state'), 'spatial_error': m.get('spatial_error'),
            'pose': m.get('pose'), 'truth': snap.get('truth_position'),
            'min_clearance': met.get('min_clearance_m'), 'loc_err_mean': loc.get('mean'),
            'planner_latency_ms': planner.get('latency_ms'),
            'fusion_ms': (snap.get('fusion') or {}).get('process_ms'),
            'sync_delta': (snap.get('fusion') or {}).get('sync_delta_s')}


# ------------------------------------------------------------------ ingest
@app.post('/api/ingest')
async def ingest(body: Ingest):
    S.last_ingest = time.time()
    S.ingest_count += 1
    if body.kind == 'snapshot':
        S.snapshot = body.data
        S.history.append(_history_point(body.data))
        for ev in body.data.get('events', []):
            key = (ev.get('source'), ev.get('t') or ev.get('stamp'), ev.get('text') or ev.get('event') or ev.get('note'))
            if key not in S.seen_events:
                S.seen_events.add(key)
                S.events.append(ev)
        await broadcast({'type': 'snapshot', 'data': body.data})
    elif body.kind == 'images':
        S.images.update(body.data)
        await broadcast({'type': 'images', 'data': body.data})
    elif body.kind == 'clouds':
        S.clouds.update(body.data)
        await broadcast({'type': 'clouds', 'data': body.data})
    return {'ok': True}


async def broadcast(msg: dict):
    dead = []
    text = json.dumps(msg)
    for ws in list(S.ws_clients):
        try:
            await asyncio.wait_for(ws.send_text(text), timeout=0.5)
        except Exception:  # noqa: BLE001
            dead.append(ws)
    for ws in dead:
        S.ws_clients.discard(ws)


@app.websocket('/ws/telemetry')
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    S.ws_clients.add(ws)
    try:
        await ws.send_text(json.dumps({'type': 'hello', 'data': {'snapshot': S.snapshot, 'images': S.images,
                                                                    'clouds': S.clouds, 'scene': scene()}}))
        while True:
            await ws.receive_text()      # keepalive pings from the client
    except WebSocketDisconnect:
        pass
    finally:
        S.ws_clients.discard(ws)


# ------------------------------------------------------------------ status
def live() -> bool:
    return time.time() - S.last_ingest < 3.0


@app.get('/api/health')
def health():
    return {'status': 'ok', 'time': time.time(), 'telemetry_live': live(), 'ingest_count': S.ingest_count,
            'ws_clients': len(S.ws_clients)}


@app.get('/api/system/status')
def system_status():
    px4 = S.snapshot.get('px4') or {}
    return {'gps': 'DISABLED', 'simulation': 'RUNNING' if live() else 'OFFLINE', 'autonomous': True,
            'px4_connected': px4.get('connected'), 'experiment': experiment_state(), 'scenario': S.scenario,
            'telemetry_age_s': (time.time() - S.last_ingest) if S.last_ingest else None}


@app.get('/api/mission/status')
def mission_status():
    return S.snapshot.get('mission') or {}


@app.get('/api/vehicle/state')
def vehicle_state():
    m = S.snapshot.get('mission') or {}
    return {'position': m.get('pose'), 'velocity': m.get('velocity'), 'yaw': m.get('yaw'),
            'px4': S.snapshot.get('px4'), 'source': 'GPS-free localisation (tracker + EKF2)'}


@app.get('/api/sensors/status')
def sensors_status():
    return {'sensors': S.snapshot.get('sensors') or {}, 'fusion': S.snapshot.get('fusion')}


@app.get('/api/map/summary')
def map_summary():
    return S.snapshot.get('mapping') or {}


@app.get('/api/planner/status')
def planner_status():
    return {'planner': S.snapshot.get('planner'), 'avoidance': S.snapshot.get('avoidance'),
            'paths': S.snapshot.get('paths')}


@app.get('/api/landing/status')
def landing_status():
    return {'detection': S.snapshot.get('detection'), 'safety': S.snapshot.get('landing_safety'),
            'state': (S.snapshot.get('mission') or {}).get('state')}


@app.get('/api/metrics/current')
def metrics_current():
    return S.snapshot.get('metrics') or {}


@app.get('/api/metrics/history')
def metrics_history(limit: int = 600):
    limit = max(1, min(limit, 3000))
    return list(S.history)[-limit:]


@app.get('/api/scene')
def scene():
    """Static presentation scene: world spec (simulation ground truth, labelled), corridor, landing zone."""
    w = yaml.safe_load(open(ROOT / 'simulation/configs/world_ps05.yaml'))
    m = yaml.safe_load(open(ROOT / 'configs/mission.yaml'))
    return {'world': w, 'mission': m, 'note': 'world geometry = simulation ground truth (display only)'}


# ------------------------------------------------------------------ experiments
def experiment_state():
    if S.proc is None:
        return S.experiment
    rc = S.proc.poll()
    if S.experiment is not None:
        S.experiment['running'] = rc is None
        S.experiment['returncode'] = rc
    return S.experiment


def list_scenarios():
    return sorted(p.stem for p in SCENARIOS.glob('*.yaml'))


@app.get('/api/scenarios')
def scenarios():
    return {'scenarios': list_scenarios(), 'selected': S.scenario}


@app.post('/api/scenario/load')
def scenario_load(body: ScenarioLoad):
    if body.scenario not in list_scenarios():
        raise HTTPException(404, f'unknown scenario {body.scenario}')
    S.scenario = body.scenario
    return {'selected': S.scenario, 'definition': yaml.safe_load(open(SCENARIOS / f'{body.scenario}.yaml'))}


@app.post('/api/experiments/start')
def experiments_start(body: ExperimentStart):
    st = experiment_state()
    if st and st.get('running'):
        raise HTTPException(409, 'an experiment is already running')
    scen = body.scenario or S.scenario
    if scen not in list_scenarios():
        raise HTTPException(404, f'unknown scenario {scen}')
    tag = body.tag or time.strftime('exp_%Y%m%d_%H%M%S')
    logf = RESULTS / 'logs' / f'{tag}.log'
    logf.parent.mkdir(parents=True, exist_ok=True)
    cmd = [str(ROOT / 'scripts/run_experiments.sh'), '--scenario', scen, '--trials', str(body.trials),
           '--seed', str(body.seed), '--tag', tag]
    S.proc = subprocess.Popen(cmd, stdout=open(logf, 'w'), stderr=subprocess.STDOUT, cwd=ROOT,
                              start_new_session=True)
    S.experiment = {'tag': tag, 'scenario': scen, 'trials': body.trials, 'seed': body.seed, 'started': time.time(),
                    'running': True, 'log': str(logf), 'pid': S.proc.pid}
    log.info('started experiment %s', S.experiment)
    return S.experiment


@app.post('/api/experiments/stop')
def experiments_stop():
    if S.proc is None or S.proc.poll() is not None:
        raise HTTPException(409, 'no experiment running')
    os.killpg(os.getpgid(S.proc.pid), signal.SIGINT)
    return {'stopping': True, 'tag': (S.experiment or {}).get('tag')}


def _run_summary(d: Path):
    f = d / 'metrics.json'
    if not f.exists():
        return None
    try:
        m = json.loads(f.read_text())
    except json.JSONDecodeError:
        return None
    keep = ('run_id', 'scenario_id', 'seed', 'outcome', 'success', 'landing_error_m', 'collision_count',
            'near_misses', 'min_clearance_m', 'mission_time_s', 'replans', 'executed_path_length_m', 'note')
    return {k: m.get(k) for k in keep} | {'mtime': f.stat().st_mtime}


@app.get('/api/experiments')
def experiments():
    raw = RESULTS / 'raw'
    runs = [r for r in (_run_summary(d) for d in raw.iterdir() if d.is_dir()) if r] if raw.exists() else []
    runs.sort(key=lambda r: r['mtime'], reverse=True)
    summary = None
    if (RESULTS / 'summary.json').exists():
        summary = json.loads((RESULTS / 'summary.json').read_text())
    return {'runs': runs, 'summary': summary, 'current': experiment_state()}


@app.get('/api/experiments/{run_id}')
def experiment(run_id: str):
    if '/' in run_id or '..' in run_id:
        raise HTTPException(400, 'invalid run id')
    d = RESULTS / 'raw' / run_id
    if not (d / 'metrics.json').exists():
        raise HTTPException(404, 'run not found')
    return json.loads((d / 'metrics.json').read_text())


@app.get('/api/logs')
def logs(lines: int = 200):
    lines = max(1, min(lines, 2000))
    exp = experiment_state()
    f = Path(exp['log']) if exp and exp.get('log') else None
    if f is None:
        cands = sorted((RESULTS / 'logs').glob('*.log'), key=lambda p: p.stat().st_mtime) if (RESULTS / 'logs').exists() else []
        f = cands[-1] if cands else None
    text = f.read_text(errors='replace').splitlines()[-lines:] if f and f.exists() else []
    return {'file': str(f) if f else None, 'lines': text, 'events': list(S.events)[-100:]}


@app.get('/api/plots/{name}')
def plot(name: str):
    if '/' in name or '..' in name or not name.endswith('.png'):
        raise HTTPException(400, 'invalid plot name')
    f = RESULTS / 'plots' / name
    if not f.exists():
        raise HTTPException(404)
    return FileResponse(f)


@app.exception_handler(Exception)
async def unhandled(request, exc):
    log.exception('unhandled error on %s', request.url.path)
    return JSONResponse(status_code=500, content={'error': type(exc).__name__, 'detail': str(exc)})


DIST = ROOT / 'frontend' / 'dist'
if DIST.exists():
    app.mount('/', StaticFiles(directory=DIST, html=True), name='frontend')
