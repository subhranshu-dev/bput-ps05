"""Mission/landing state machine against a kinematic fake vehicle (Phase 17 + safety branches)."""
import numpy as np
import pytest
import yaml

from landing_controller import state_machine as S
from planner_3d.trajectory import build_trajectory
from ps05_common import config

CFG_DIR = config.config_dir()
MISSION = yaml.safe_load(open(CFG_DIR / 'mission.yaml'))
LANDING = yaml.safe_load(open(CFG_DIR / 'landing.yaml'))
SAFETY = yaml.safe_load(open(CFG_DIR / 'safety.yaml'))
TRAJ = yaml.safe_load(open(CFG_DIR / 'planner.yaml'))['trajectory']
PAD = np.array([42.0, 42.0, 0.1])


class FakeWorld:
    """Kinematic vehicle + PX4 + perception stubs. Pose is what localisation would publish."""

    def __init__(self, **kw):
        self.sm = S.MissionStateMachine(MISSION, LANDING, SAFETY)
        self.t = 0.0
        self.pos = np.array([5.0, 5.0, 0.05])
        self.vel = np.zeros(3)
        self.armed = self.offboard = False
        self.landed = True
        self.landing = False
        self.plan_ok = kw.get('plan_ok', lambda n: True)
        self.block_at = kw.get('block_at')          # sim time at which avoidance reports blocked
        self.hide = kw.get('hide', (None, None))    # target hidden interval
        self.unsafe = kw.get('unsafe', (None, None))
        self.lidar_fail = kw.get('lidar_fail', (None, None))
        self.push = kw.get('push')                  # disturbance (t0, t1, velocity)
        self.pending = None
        self.blocked_sent = False
        self.n_plans = 0
        self.states = []

    def obs(self):
        o = S.Observation(t=self.t, pose=self.pos.copy(), yaw=0.0, vel=self.vel.copy(), px4_connected=True,
                          armed=self.armed, offboard=self.offboard, landed=self.landed,
                          sensors={'odom': True, 'lidar': not (self.lidar_fail[0] is not None and self.lidar_fail[0] <= self.t < self.lidar_fail[1]),
                                   'camera': True, 'down_camera': True, 'down_lidar': True})
        if self.pending is not None:
            req = self.pending
            self.pending = None
            self.n_plans += 1
            if self.plan_ok(self.n_plans):
                wps = np.array([req['start'], req['goal']])
                if req['reason'] != 'initial':
                    mid = (wps[0] + wps[1]) / 2 + [3, -3, 0]
                    wps = np.array([req['start'], mid, req['goal']])
                tr = build_trajectory(wps, TRAJ)
                o.plan_result = {'id': req['id'], 'success': True, 'trajectory': tr, 'latency_ms': 12.0, 'length_m': tr.length()}
            else:
                o.plan_result = {'id': req['id'], 'success': False, 'message': 'no path exists inside the approved mission space'}
        if self.block_at is not None and self.t >= self.block_at and not self.blocked_sent and self.sm.state == S.NAVIGATE:
            o.avoidance = {'trajectory_id': self.sm.traj_id, 'blocked': True, 'distance_along_path': 6.0, 'source': 'lidar'}
            self.blocked_sent = True
        hidden = self.hide[0] is not None and self.hide[0] <= self.t < self.hide[1]
        rel = PAD - self.pos
        if not hidden and np.hypot(*rel[:2]) < 0.9 * (self.pos[2] - PAD[2]) and self.pos[2] > 0.4:
            self.last_target = {'pos': PAD.tolist(), 'stamp': self.t}
        o.target = getattr(self, 'last_target', None)   # like mission_manager: last detection + its stamp
        unsafe = self.unsafe[0] is not None and self.unsafe[0] <= self.t < self.unsafe[1]
        agl = self.pos[2] - PAD[2]
        align = float(np.hypot(*rel[:2]))
        reasons = (['OBSTRUCTION'] if unsafe else []) + (['MISALIGNED'] if align > 0.35 else [])
        o.safety = {'stamp': self.t, 'surface_ok': not unsafe, 'safe': not unsafe and align <= 0.35, 'reasons': reasons,
                    'height_above_ground': agl}
        return o

    def apply(self, out, dt):
        if out.plan_request:
            self.pending = out.plan_request
        if out.command == 'offboard':
            self.offboard = True
        elif out.command == 'arm' and self.offboard:
            self.armed = True
        elif out.command == 'land':
            self.landing = True
        elif out.command == 'disarm':
            self.armed = False
        sp = out.setpoint
        if self.landing:
            v = np.array([0, 0, -0.4])
        elif sp is None or not self.armed:
            v = np.zeros(3)
        elif sp['mode'] == 'velocity':
            v = np.array(sp['vel'])
        else:
            v = np.clip(1.5 * (np.array(sp['pos']) - self.pos) + np.array(sp.get('vel') or [0, 0, 0]), -3, 3)
        if self.push and self.push[0] <= self.t < self.push[1]:
            v = v + self.push[2]
        self.vel = v
        self.pos = self.pos + v * dt
        ground = PAD[2] if np.hypot(*(self.pos - PAD)[:2]) < 1.5 else 0.0
        if self.pos[2] <= ground + 0.05 and (self.landing or not self.armed):
            self.pos[2] = ground + 0.05
            self.landed = True
        else:
            self.landed = self.pos[2] <= ground + 0.06

    def run(self, t_max=400.0, dt=0.05, until=None):
        while self.t < t_max:
            out = self.sm.step(self.obs())
            if not self.states or self.states[-1] != out.state:
                self.states.append(out.state)
            if until and out.state in until:
                break
            self.apply(out, dt)
            self.t += dt
        return self


def test_nominal_mission_lands_on_target():
    w = FakeWorld().run(until={S.LANDED, S.ABORTED})
    assert w.sm.state == S.LANDED, w.states
    for s in (S.ARMING, S.TAKEOFF, S.PLANNING, S.NAVIGATE, S.LANDING_ZONE_SEARCH, S.TARGET_LOCK, S.ALIGN,
              S.DESCEND, S.FINAL_CHECK, S.LAND, S.LANDED):
        assert s in w.states
    assert np.hypot(*(w.pos - PAD)[:2]) < 0.3
    assert w.sm.outcome == 'LANDED'


def test_obstacle_stop_then_replan_and_continue():
    w = FakeWorld(block_at=25.0).run(until={S.LANDED, S.ABORTED})
    i = w.states.index(S.OBSTACLE_STOP)
    assert w.states[i + 1] == S.REPLANNING and w.states[i + 2] == S.NAVIGATE
    assert w.sm.state == S.LANDED and w.sm.stats['replans'] == 1


def test_no_safe_path_enters_safe_state_then_aborts():
    w = FakeWorld(plan_ok=lambda n: False).run(until={S.ABORTED}, t_max=200)
    assert S.NO_SAFE_PATH in w.states and w.sm.state == S.ABORTED
    assert w.sm.outcome == 'NO_SAFE_PATH'
    assert S.NAVIGATE not in w.states


def test_no_safe_path_recovers_when_retry_succeeds():
    w = FakeWorld(plan_ok=lambda n: n >= 3).run(until={S.LANDED, S.ABORTED})
    assert S.NO_SAFE_PATH in w.states and w.sm.state == S.LANDED


def test_target_loss_hover_and_reacquire():
    w = FakeWorld()
    w.run(until={S.DESCEND})
    t = w.t
    w.hide = (t + 0.5, t + 4.0)
    w.run(until={S.LANDED, S.ABORTED})
    assert S.TARGET_LOST in w.states and w.sm.state == S.LANDED
    assert w.sm.stats['reacquire_times_s'] and w.sm.stats['reacquire_times_s'][0] == pytest.approx(3.5 - 1.5, abs=0.3)


def test_unsafe_pad_abort_climb_search_then_land():
    w = FakeWorld()
    w.run(until={S.ALIGN})
    w.unsafe = (w.t, w.t + 12.0)
    w.run(until={S.LANDED, S.ABORTED})
    i = w.states.index(S.UNSAFE_PAD)
    assert w.states[i + 1] == S.ABORT_CLIMB and w.states[i + 2] == S.LANDING_ZONE_SEARCH
    assert w.sm.state == S.LANDED


def test_permanently_unsafe_pad_aborts_without_landing_on_it():
    w = FakeWorld(unsafe=(0.0, 1e9)).run(until={S.ABORTED, S.LANDED})
    assert w.sm.state == S.ABORTED and w.sm.outcome == 'UNSAFE_LANDING_ZONE'
    assert S.DESCEND not in w.states


def test_sensor_failure_safe_state_and_recovery():
    w = FakeWorld(lidar_fail=(22.0, 25.0)).run(until={S.LANDED, S.ABORTED})
    assert S.SENSOR_FAILURE in w.states and w.sm.state == S.LANDED


def test_corridor_breach_alarm_and_containment():
    w = FakeWorld(push=(24.0, 27.0, np.array([0.0, 2.5, 0.0]))).run(until={S.LANDED, S.ABORTED})
    assert S.CORRIDOR_BREACH in w.states
    assert w.sm.stats['corridor_breaches'] >= 1
    assert w.sm.state in (S.LANDED, S.ABORTED)


def test_init_waits_for_converged_localisation():
    """Home must not be latched before EKF2 agrees with the tracker (bring-up bug: home taken at EKF default origin)."""
    w = FakeWorld()
    orig = w.obs

    def obs():
        o = orig()
        o.loc_converged = w.t >= 10.0
        return o
    w.obs = obs
    w.run(until={S.TAKEOFF}, t_max=60)
    arm_t = next(h['t'] for h in w.sm.stats['state_history'] if h['to'] == S.ARMING)
    assert arm_t >= 12.0 - 1e-6
    assert np.allclose(w.sm.home[:2], [5.0, 5.0])


def test_unsafe_pad_waits_before_retry_then_lands_when_cleared():
    w = FakeWorld()
    w.run(until={S.ALIGN})
    w.unsafe = (w.t, w.t + 15.0)
    w.run(until={S.LANDED, S.ABORTED})
    assert w.sm.state == S.LANDED, w.states
    assert 1 <= w.sm.abort_attempts <= 3
