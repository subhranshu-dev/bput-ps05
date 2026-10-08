"""Backend API tests (FastAPI TestClient)."""
import pytest
from fastapi.testclient import TestClient

from app.main import S, app

c = TestClient(app)


def test_health():
    r = c.get('/api/health')
    assert r.status_code == 200 and r.json()['status'] == 'ok'


def test_system_status_reports_gps_disabled():
    assert c.get('/api/system/status').json()['gps'] == 'DISABLED'


def test_ingest_snapshot_and_query():
    snap = {'mission': {'state': 'NAVIGATE', 'pose': [1, 2, 3], 'spatial_error': 0.2}, 'ros_time': 12.5,
            'events': [{'source': 'mission', 't': 1.0, 'text': 'INIT -> ARMING'}], 'metrics': {'min_clearance_m': 2.0}}
    assert c.post('/api/ingest', json={'kind': 'snapshot', 'data': snap}).status_code == 200
    assert c.get('/api/mission/status').json()['state'] == 'NAVIGATE'
    assert c.get('/api/vehicle/state').json()['position'] == [1, 2, 3]
    assert c.get('/api/metrics/history').json()[-1]['spatial_error'] == 0.2
    assert c.get('/api/health').json()['telemetry_live'] is True


def test_ingest_validation():
    assert c.post('/api/ingest', json={'kind': 'bogus', 'data': {}}).status_code == 422
    assert c.post('/api/ingest', json={'kind': 'snapshot'}).status_code == 422


def test_websocket_receives_hello_and_updates():
    with c.websocket_connect('/ws/telemetry') as ws:
        hello = ws.receive_json()
        assert hello['type'] == 'hello' and 'scene' in hello['data']
        c.post('/api/ingest', json={'kind': 'snapshot', 'data': {'mission': {'state': 'ALIGN'}, 'events': []}})
        msg = ws.receive_json()
        assert msg['type'] == 'snapshot' and msg['data']['mission']['state'] == 'ALIGN'


def test_scenario_validation():
    assert c.post('/api/scenario/load', json={'scenario': '../etc/passwd'}).status_code == 422
    assert c.post('/api/scenario/load', json={'scenario': 'does_not_exist'}).status_code == 404
    assert c.post('/api/scenario/load', json={'scenario': 'baseline'}).status_code == 200


def test_experiment_input_validation():
    assert c.post('/api/experiments/start', json={'trials': 0}).status_code == 422
    assert c.post('/api/experiments/start', json={'trials': 500}).status_code == 422
    assert c.post('/api/experiments/stop').status_code in (409, 200)


def test_experiments_list_and_bad_ids():
    assert 'runs' in c.get('/api/experiments').json()
    assert c.get('/api/experiments/nonexistent_run').status_code == 404
    assert c.get('/api/plots/..%2Fsecret.png').status_code in (400, 404)


def test_scene():
    s = c.get('/api/scene').json()
    assert s['world']['name'] == 'ps05_challenge' and 'mission_space' in s['mission']
