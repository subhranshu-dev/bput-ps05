"""Mission + precision-landing state machine (Phases 13, 17, 18 and PS05 path enforcement).

Nominal chain:
    INIT -> ARMING -> TAKEOFF -> PLANNING -> NAVIGATE -> LANDING_ZONE_SEARCH
         -> TARGET_LOCK -> ALIGN -> DESCEND -> FINAL_CHECK -> LAND -> LANDED
Failure branches:
    OBSTACLE_STOP -> REPLANNING -> NAVIGATE           (obstacle on trajectory)
    NO_SAFE_PATH  -> (retry) -> ABORTED               (no route inside approved space)
    TARGET_LOST   -> hover -> reacquire | search      (marker lost)
    UNSAFE_PAD    -> ABORT_CLIMB -> LANDING_ZONE_SEARCH (LiDAR says pad unsafe)
    SENSOR_FAILURE -> safe state (hover / land)        (sensor watchdog)
    CORRIDOR_BREACH -> containment (return to path) -> replan | ABORTED (E > Emax)

The class is pure Python: the ROS node feeds an Observation each tick and
executes the returned Output (setpoint / PX4 command / plan request).
"""
import math
from dataclasses import dataclass, field

import numpy as np

# ---- state names (shown on the dashboard) -----------------------------------
INIT, ARMING, TAKEOFF, PLANNING, NAVIGATE = 'INIT', 'ARMING', 'TAKEOFF', 'PLANNING', 'NAVIGATE'
OBSTACLE_STOP, REPLANNING, NO_SAFE_PATH = 'OBSTACLE_STOP', 'REPLANNING', 'NO_SAFE_PATH'
LANDING_ZONE_SEARCH, TARGET_LOCK, ALIGN = 'LANDING_ZONE_SEARCH', 'TARGET_LOCK', 'ALIGN'
DESCEND, FINAL_CHECK, LAND, LANDED = 'DESCEND', 'FINAL_CHECK', 'LAND', 'LANDED'
TARGET_LOST, UNSAFE_PAD, ABORT_CLIMB = 'TARGET_LOST', 'UNSAFE_PAD', 'ABORT_CLIMB'
SENSOR_FAILURE, CORRIDOR_BREACH, ABORTED = 'SENSOR_FAILURE', 'CORRIDOR_BREACH', 'ABORTED'

FAILURE_STATES = {OBSTACLE_STOP, REPLANNING, NO_SAFE_PATH, TARGET_LOST, UNSAFE_PAD, ABORT_CLIMB,
                  SENSOR_FAILURE, CORRIDOR_BREACH, ABORTED}
AIRBORNE_STATES = {TAKEOFF, PLANNING, NAVIGATE, OBSTACLE_STOP, REPLANNING, NO_SAFE_PATH,
                   LANDING_ZONE_SEARCH, TARGET_LOCK, ALIGN, DESCEND, FINAL_CHECK, TARGET_LOST,
                   UNSAFE_PAD, ABORT_CLIMB, CORRIDOR_BREACH}
LANDING_STATES = {LANDING_ZONE_SEARCH, TARGET_LOCK, ALIGN, DESCEND, FINAL_CHECK, TARGET_LOST,
                  UNSAFE_PAD, ABORT_CLIMB}


@dataclass
class Observation:
    t: float
    pose: np.ndarray | None = None          # estimated position (map, ENU) - NEVER ground truth
    yaw: float = 0.0
    vel: np.ndarray | None = None
    px4_connected: bool = False
    armed: bool = False
    offboard: bool = False
    landed: bool = True
    sensors: dict = field(default_factory=dict)   # name -> healthy bool
    loc_converged: bool = True               # EKF2 odometry agrees with the latest tracker fix
    plan_result: dict | None = None         # {'id', 'success', 'trajectory', 'waypoints', 'message', 'latency_ms'}
    avoidance: dict | None = None           # {'trajectory_id', 'blocked', 'distance_along_path', 'source'}
    target: dict | None = None              # {'pos': (x,y,z) map, 'stamp': t_detect}
    safety: dict | None = None              # validator output + 'stamp'


@dataclass
class Output:
    state: str
    setpoint: dict | None = None     # {'mode': 'position'|'velocity', 'pos', 'vel', 'yaw'}
    command: str | None = None       # 'offboard' | 'arm' | 'land' | 'disarm'
    plan_request: dict | None = None  # {'id', 'start', 'goal', 'reason'}
    trajectory: object = None        # new active trajectory (when changed)
    events: list = field(default_factory=list)


class MissionStateMachine:
    def __init__(self, mission_cfg: dict, landing_cfg: dict, safety_cfg: dict, pid_factory=None):
        from .pid import PID
        self.m = mission_cfg
        self.lc = landing_cfg['controller']
        self.lv = landing_cfg['validator']
        self.s = safety_cfg
        pe = safety_cfg['path_enforcement']
        self.e_warn, self.e_max = float(pe['e_warn']), float(pe['e_max'])
        self.breach_persist = float(pe.get('breach_persist_s', 0.5))
        self.containment_hold = float(pe.get('containment_hold_s', 5.0))
        ss = safety_cfg['safe_state']
        self.no_path_hover = float(ss['no_path_hover_s'])
        self.no_path_retries = int(ss['no_path_retries'])
        self.abort_action = ss.get('abort_action', 'land_in_place')
        mk = pid_factory or (lambda: PID(self.lc['kp'], self.lc['ki'], self.lc['kd'],
                                         out_limit=self.lc['v_max_xy'], i_limit=self.lc['i_limit']))
        self.pid_x, self.pid_y = mk(), mk()

        lz = mission_cfg['landing_zone']
        self.lz_center = np.array(lz['center'], dtype=float)
        self.approach_alt = float(lz['approach_altitude'])
        self.search_alt = float(lz.get('search_altitude', self.approach_alt))
        self.search_radius = float(lz.get('search_radius', 3.0))
        self.takeoff_alt = float(mission_cfg['start']['takeoff_altitude'])

        self.state = INIT
        self.prev_state = None
        self.t_state = 0.0
        self.t_last = None
        self.home = None
        self.hold_pos = None
        self.hold_yaw = 0.0
        self.traj = None
        self.traj_id = 0
        self.traj_time = 0.0
        self.goal = None
        self.plan_id = 0
        self.pending_plan = None
        self.plan_failures = 0
        self.t_breach = None
        self.ref_pos = None
        self.spatial_error = 0.0
        self.search_wps = []
        self.search_idx = 0
        self.search_loops = 0
        self.target_filt = None
        self.lock_count = 0
        self.t_aligned = None
        self.checkpoints = sorted([float(c) for c in self.lc.get('checkpoints', [])], reverse=True)
        self.next_cp = 0
        self.cp_wait_since = None
        self.abort_attempts = 0
        self.resume_state = None
        self.t_target_lost = None
        self.last_cmd_t = -1e9
        self.outcome = None
        self.stats = {
            'replans': 0, 'obstacle_detections': 0, 'detection_events': [], 'replan_latencies_ms': [],
            'stop_response_s': [], 'target_losses': 0, 'reacquire_times_s': [], 'aborts': 0,
            'corridor_breaches': 0, 'sensor_failures': 0, 'plan_latencies_ms': [], 'max_spatial_error': 0.0,
            'spatial_error_sum': 0.0, 'spatial_error_n': 0, 'path_lengths_m': [], 'state_history': [],
            'touchdown_estimate': None, 'mission_start_t': None, 'mission_end_t': None,
        }

    # ================================================================ helpers
    def _goto(self, new_state, t, out, reason=''):
        if new_state == self.state:
            return
        out.events.append(('ERROR' if new_state in (ABORTED, CORRIDOR_BREACH, SENSOR_FAILURE, NO_SAFE_PATH)
                           else 'WARN' if new_state in FAILURE_STATES else 'INFO',
                           f'{self.state} -> {new_state}' + (f': {reason}' if reason else '')))
        self.stats['state_history'].append({'t': round(t, 3), 'from': self.state, 'to': new_state, 'reason': reason})
        self.prev_state = self.state
        self.state = new_state
        self.t_state = t
        out.state = new_state

    def _hold(self, pos, yaw):
        return {'mode': 'position', 'pos': np.asarray(pos, dtype=float).tolist(), 'vel': [0.0, 0.0, 0.0], 'yaw': yaw}

    def _vel(self, v, yaw):
        return {'mode': 'velocity', 'vel': np.asarray(v, dtype=float).tolist(), 'yaw': yaw}

    def _request_plan(self, start, goal, reason, out):
        self.plan_id += 1
        self.pending_plan = self.plan_id
        out.plan_request = {'id': self.plan_id, 'start': np.asarray(start, dtype=float).tolist(),
                            'goal': np.asarray(goal, dtype=float).tolist(), 'reason': reason}

    def _set_hold_here(self, obs):
        self.hold_pos = np.array(obs.pose, dtype=float)
        self.hold_yaw = obs.yaw

    def _target_fresh(self, obs, max_age):
        return obs.target is not None and (obs.t - obs.target['stamp']) <= max_age

    def _safety_fresh(self, obs, max_age=1.0):
        return obs.safety is not None and (obs.t - obs.safety.get('stamp', -1e9)) <= max_age

    def _height_agl(self, obs):
        if self._safety_fresh(obs, 1.0) and obs.safety.get('height_above_ground') is not None:
            return float(obs.safety['height_above_ground'])
        ground = self.home[2] if self.home is not None else 0.0
        return float(obs.pose[2] - ground)

    def _update_target(self, obs):
        if obs.target is None:
            return
        p = np.asarray(obs.target['pos'], dtype=float)
        a = float(self.lc.get('target_filter_alpha', 1.0))  # detector already low-pass filters
        self.target_filt = p if self.target_filt is None else (1 - a) * self.target_filt + a * p

    def _build_search(self, pos_z):
        c, r = self.lz_center, self.search_radius
        pts = [(0, 0), (r, 0), (r, r), (-r, r), (-r, -r), (r, -r), (r, 0), (0, 0)]
        self.search_wps = [np.array([c[0] + dx, c[1] + dy, pos_z]) for dx, dy in pts]
        self.search_idx = 0

    def _critical_sensor_fault(self, obs):
        s = obs.sensors or {}
        if not s.get('odom', True):
            return 'odom'
        if self.state in (NAVIGATE, PLANNING, REPLANNING, OBSTACLE_STOP) and not s.get('lidar', True):
            return 'lidar'
        if self.state in (DESCEND, FINAL_CHECK) and not s.get('down_lidar', True):
            return 'down_lidar'
        return None

    # ================================================================ main step
    def step(self, obs: Observation) -> Output:
        out = Output(state=self.state)
        dt = 0.05 if self.t_last is None else max(obs.t - self.t_last, 1e-3)
        self.t_last = obs.t
        if obs.pose is None and self.state != INIT:
            # localisation lost: PX4 must take over (its own failsafe will land)
            if self.state in AIRBORNE_STATES:
                self.stats['sensor_failures'] += 1
                self._goto(SENSOR_FAILURE, obs.t, out, 'localisation unavailable -> PX4 land')
                out.command = 'land'
            return out
        if obs.plan_result is not None and obs.plan_result.get('id') == self.pending_plan:
            self.pending_plan = None
            plan = obs.plan_result
            if plan.get('latency_ms') is not None:
                self.stats['plan_latencies_ms'].append(plan['latency_ms'])
        else:
            plan = None

        # ---- global watchdogs while airborne
        if self.state in AIRBORNE_STATES and self.state != SENSOR_FAILURE:
            fault = self._critical_sensor_fault(obs)
            if fault:
                self.stats['sensor_failures'] += 1
                self.resume_state = self.state
                self._set_hold_here(obs)
                self._goto(SENSOR_FAILURE, obs.t, out, f'{fault} watchdog timeout')

        handler = getattr(self, '_st_' + self.state.lower())
        handler(obs, out, plan, dt)
        if self.state in AIRBORNE_STATES and obs.pose is not None:
            self.stats['max_spatial_error'] = max(self.stats['max_spatial_error'], self.spatial_error)
        out.state = self.state
        return out

    # ================================================================ states
    def _st_init(self, obs, out, plan, dt):
        ready = obs.px4_connected and obs.pose is not None and obs.loc_converged and all(
            obs.sensors.get(k, False) for k in ('odom', 'lidar', 'camera'))
        if obs.pose is not None:
            out.setpoint = self._hold(obs.pose, obs.yaw)
        # require 2 s of continuous readiness (EKF2 must have switched to tracker aiding before home is taken)
        if not ready:
            self.t_ready = None
        elif getattr(self, 't_ready', None) is None:
            self.t_ready = obs.t
        if ready and obs.t - self.t_ready >= 2.0:
            self.home = np.array(obs.pose, dtype=float)
            self.hold_yaw = obs.yaw
            self.stats['mission_start_t'] = obs.t
            self._goto(ARMING, obs.t, out, 'sensors + localisation converged')

    def _st_arming(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.home, self.hold_yaw)
        if obs.armed and obs.offboard:
            # latch home at arming (vehicle on the ground, localisation converged)
            self.home = np.array(obs.pose, dtype=float)
            self.hold_pos = self.home.copy()
            self._goto(TAKEOFF, obs.t, out, f'armed in offboard; home ({self.home[0]:.2f}, {self.home[1]:.2f})')
            return
        if not obs.loc_converged:
            self.home = np.array(obs.pose, dtype=float)   # keep following the estimate until converged
        if obs.t - self.last_cmd_t > 1.0:
            out.command = 'offboard' if not obs.offboard else 'arm'
            self.last_cmd_t = obs.t
        if obs.t - self.t_state > 30.0:
            self._goto(ABORTED, obs.t, out, 'arming timeout')
            self.outcome = 'ARMING_FAILED'

    def _st_takeoff(self, obs, out, plan, dt):
        target_z = self.home[2] + self.takeoff_alt
        z_sp = min(target_z, self.home[2] + 0.8 * (obs.t - self.t_state) + 0.3)
        sp = np.array([self.home[0], self.home[1], z_sp])
        out.setpoint = self._hold(sp, self.hold_yaw)
        self.ref_pos = sp
        if abs(obs.pose[2] - target_z) < 0.3:
            self._set_hold_here(obs)
            self.goal = np.array([self.lz_center[0], self.lz_center[1], self.approach_alt])
            self._request_plan(obs.pose, self.goal, 'initial', out)
            self._goto(PLANNING, obs.t, out, 'takeoff altitude reached')
        elif obs.t - self.t_state > 40.0:
            self._goto(ABORTED, obs.t, out, 'takeoff timeout')
            self.outcome = 'TAKEOFF_FAILED'
            out.command = 'land'

    def _accept_plan(self, obs, out, plan, reason):
        traj = plan['trajectory']
        self.traj = traj
        self.traj_id += 1
        self.traj_time = 0.0
        out.trajectory = traj
        self.plan_failures = 0
        if plan.get('length_m') is not None:
            self.stats['path_lengths_m'].append(plan['length_m'])
        self._goto(NAVIGATE, obs.t, out, reason)

    def _st_planning(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        if plan is not None:
            if plan['success']:
                self._accept_plan(obs, out, plan, f"global path {plan.get('length_m', 0):.1f} m")
            else:
                self.plan_failures += 1
                self._goto(NO_SAFE_PATH, obs.t, out, plan.get('message', 'planning failed'))
        elif self.pending_plan is None:
            self._request_plan(obs.pose, self.goal, 'initial', out)

    def _st_navigate(self, obs, out, plan, dt):
        traj = self.traj
        ref, vel, yaw = traj.sample(self.traj_time)
        e = float(np.linalg.norm(obs.pose - ref))
        self.spatial_error = e
        self.ref_pos = ref
        self.stats['spatial_error_sum'] += e
        self.stats['spatial_error_n'] += 1
        # PS05 path enforcement: E vs Emax
        if e > self.e_max:
            if self.t_breach is None:
                self.t_breach = obs.t
            elif obs.t - self.t_breach >= self.breach_persist:
                self.stats['corridor_breaches'] += 1
                self.hold_pos = ref.copy()
                self.hold_yaw = yaw
                self._goto(CORRIDOR_BREACH, obs.t, out, f'E={e:.2f} m > Emax={self.e_max:.2f} m: CRITICAL ALARM')
                out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
                return
        else:
            self.t_breach = None
        # local obstacle check result for the *current* trajectory
        av = obs.avoidance
        if av is not None and av.get('trajectory_id') == self.traj_id and av.get('blocked'):
            self.stats['obstacle_detections'] += 1
            self.stats['detection_events'].append({'t': obs.t, 'source': av.get('source'), 'point': av.get('obstacle_point'),
                                                   'distance_along_path': av.get('distance_along_path'),
                                                   'detected_at': av.get('stamp', obs.t)})
            self._set_hold_here(obs)
            pt = av.get('obstacle_point')
            self._goto(OBSTACLE_STOP, obs.t, out,
                       f"obstacle {av.get('distance_along_path', 0) or 0:.1f} m ahead ({av.get('source')})"
                       + (f" at ({pt[0]:.1f}, {pt[1]:.1f}, {pt[2]:.1f})" if pt else ''))
            out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
            return
        # progress gating: do not let the carrot run away from the vehicle
        if e < self.e_warn:
            self.traj_time = min(self.traj_time + dt, traj.duration)
        out.setpoint = {'mode': 'position', 'pos': ref.tolist(), 'vel': vel.tolist(), 'yaw': yaw}
        if self.traj_time >= traj.duration and float(np.linalg.norm(obs.pose - traj.pos[-1])) < 0.5:
            self._set_hold_here(obs)
            self._build_search(self.search_alt)
            self.search_loops = 0
            self.ref_pos = None
            self.spatial_error = 0.0
            self._goto(LANDING_ZONE_SEARCH, obs.t, out, 'landing region reached')

    def _st_obstacle_stop(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        speed = float(np.linalg.norm(obs.vel)) if obs.vel is not None else 0.0
        # wait until stopped (and at least 1 s so the map integrates the new obstacle)
        if (speed < 0.3 and obs.t - self.t_state > 1.0) or obs.t - self.t_state > 4.0:
            self.stats['stop_response_s'].append(obs.t - self.t_state)
            self._set_hold_here(obs)
            self.t_replan_start = obs.t
            self._request_plan(obs.pose, self.goal, 'replan', out)
            self._goto(REPLANNING, obs.t, out, 'vehicle stopped; replanning inside approved space')

    def _st_replanning(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        if plan is not None:
            if plan['success']:
                self.stats['replans'] += 1
                self.stats['replan_latencies_ms'].append(plan.get('latency_ms'))
                self._accept_plan(obs, out, plan, f"replanned path {plan.get('length_m', 0):.1f} m")
            else:
                self.plan_failures += 1
                self._goto(NO_SAFE_PATH, obs.t, out, plan.get('message', 'replanning failed'))
        elif self.pending_plan is None:
            self._request_plan(obs.pose, self.goal, 'replan', out)
        elif obs.t - self.t_state > 15.0:
            self.plan_failures += 1
            self._goto(NO_SAFE_PATH, obs.t, out, 'planner timeout')

    def _st_no_safe_path(self, obs, out, plan, dt):
        # SAFE STATE: hover in place inside the approved space and retry periodically
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        if plan is not None:
            if plan['success']:
                self.stats['replans'] += 1
                self._accept_plan(obs, out, plan, 'safe path found after retry')
                return
            self.plan_failures += 1
        if self.plan_failures > self.no_path_retries:
            self.outcome = 'NO_SAFE_PATH'
            self.stats['aborts'] += 1
            self._goto(ABORTED, obs.t, out, f'no safe path after {self.plan_failures} attempts')
            return
        if self.pending_plan is None and obs.t - self.t_state > self.no_path_hover / max(self.no_path_retries, 1) * self.plan_failures:
            self._request_plan(obs.pose, self.goal, 'retry', out)

    def _st_corridor_breach(self, obs, out, plan, dt):
        # containment: return to the last approved reference point and hold
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        e = float(np.linalg.norm(obs.pose - self.hold_pos))
        self.spatial_error = e
        if e < self.e_warn and obs.t - self.t_state > 1.0:
            self._set_hold_here(obs)
            self._request_plan(obs.pose, self.goal, 'replan', out)
            self._goto(REPLANNING, obs.t, out, 'contained back inside corridor')
        elif obs.t - self.t_state > self.containment_hold:
            self.outcome = 'CORRIDOR_BREACH'
            self.stats['aborts'] += 1
            self._goto(ABORTED, obs.t, out, 'containment failed')

    def _st_sensor_failure(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        s = obs.sensors or {}
        healthy = all(s.get(k, True) for k in ('odom', 'lidar')) and (
            self.resume_state not in (DESCEND, FINAL_CHECK) or s.get('down_lidar', True))
        if healthy and obs.t - self.t_state > 1.0:
            resume = self.resume_state or PLANNING
            if resume in (NAVIGATE, OBSTACLE_STOP, REPLANNING, PLANNING):
                self._request_plan(obs.pose, self.goal, 'replan', out)
                self._goto(REPLANNING, obs.t, out, 'sensors recovered')
            elif resume in (DESCEND, FINAL_CHECK, ALIGN):
                self._goto(ALIGN, obs.t, out, 'sensors recovered')
            else:
                self._goto(resume, obs.t, out, 'sensors recovered')
        elif obs.t - self.t_state > 10.0:
            self.outcome = 'SENSOR_FAILURE'
            self.stats['aborts'] += 1
            self._goto(ABORTED, obs.t, out, 'sensor did not recover')

    def _st_aborted(self, obs, out, plan, dt):
        if self.outcome is None:
            self.outcome = 'ABORTED'
        if obs.landed or not obs.armed:
            self.stats['mission_end_t'] = self.stats['mission_end_t'] or obs.t
            return
        if self.abort_action == 'land_in_place' and obs.t - self.t_state > 2.0:
            if obs.t - self.last_cmd_t > 1.0:
                out.command = 'land'
                self.last_cmd_t = obs.t
        else:
            out.setpoint = self._hold(self.hold_pos if self.hold_pos is not None else obs.pose, self.hold_yaw)

    # ---------------------------------------------------------------- landing
    def _st_landing_zone_search(self, obs, out, plan, dt):
        self._update_target(obs)
        if self._target_fresh(obs, 0.3):
            self._set_hold_here(obs)
            self.lock_count = 0
            self._goto(TARGET_LOCK, obs.t, out, 'marker detected')
            out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
            return
        wp = self.search_wps[self.search_idx]
        d = wp - obs.pose
        dist = float(np.linalg.norm(d))
        if dist < 0.4:
            self.search_idx += 1
            if self.search_idx >= len(self.search_wps):
                self.search_idx = 0
                self.search_loops += 1
                if self.search_loops >= 2:
                    self.outcome = 'TARGET_NOT_FOUND'
                    self.stats['aborts'] += 1
                    self._set_hold_here(obs)
                    self._goto(ABORTED, obs.t, out, 'landing target not found in search area')
                    return
            wp = self.search_wps[self.search_idx]
            d = wp - obs.pose
            dist = float(np.linalg.norm(d))
        v = d / max(dist, 1e-6) * min(0.8, dist)
        out.setpoint = {'mode': 'position', 'pos': (obs.pose + v * 1.0).tolist(), 'vel': v.tolist(),
                        'yaw': self.hold_yaw}

    def _st_target_lock(self, obs, out, plan, dt):
        self._update_target(obs)
        # move over the current tag estimate (at constant altitude) so it stays centred in the image
        if self.target_filt is not None:
            self.hold_pos = np.array([self.target_filt[0], self.target_filt[1], self.hold_pos[2]])
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        if self._target_fresh(obs, 0.3):
            self.lock_count += 1
        if not self._target_fresh(obs, self.lc['target_lost_timeout_s']):
            self._enter_target_lost(obs, out)
            return
        if self.lock_count >= 10:   # ~0.5 s of consistent detections at 20 Hz
            self.pid_x.reset()
            self.pid_y.reset()
            self.t_aligned = None
            self._goto(ALIGN, obs.t, out, 'target locked')

    def _xy_control(self, obs, dt):
        ex = self.target_filt[0] - obs.pose[0]
        ey = self.target_filt[1] - obs.pose[1]
        vx = self.pid_x.update(ex, dt)
        vy = self.pid_y.update(ey, dt)
        return ex, ey, vx, vy

    def _enter_target_lost(self, obs, out):
        self.stats['target_losses'] += 1
        self.t_target_lost = obs.t
        self.resume_state = self.state if self.state in (ALIGN, DESCEND, FINAL_CHECK, TARGET_LOCK) else ALIGN
        self._set_hold_here(obs)
        self._goto(TARGET_LOST, obs.t, out, 'marker not seen for '
                   f"{self.lc['target_lost_timeout_s']} s -> hover")
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)

    def _surface_unsafe(self, obs):
        if not self._safety_fresh(obs, 1.0):
            return None
        sf = obs.safety
        bad = [r for r in sf.get('reasons', []) if r in ('OBSTRUCTION', 'SURFACE_TILTED', 'SURFACE_ROUGH',
                                                           'HEIGHT_INVALID')]
        return bad or None

    def _enter_unsafe(self, obs, out, reasons):
        self.stats['aborts'] += 1
        self.abort_attempts += 1
        self._set_hold_here(obs)
        self._goto(UNSAFE_PAD, obs.t, out, 'LiDAR validation failed: ' + ','.join(reasons))
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)

    def _st_align(self, obs, out, plan, dt):
        self._update_target(obs)
        if not self._target_fresh(obs, self.lc['target_lost_timeout_s']):
            self._enter_target_lost(obs, out)
            return
        bad = self._surface_unsafe(obs)
        if bad:
            self._enter_unsafe(obs, out, bad)
            return
        ex, ey, vx, vy = self._xy_control(obs, dt)
        err = math.hypot(ex, ey)
        # hold altitude while aligning
        z_ref = self.hold_pos[2]
        vz = max(-0.5, min(0.5, 1.0 * (z_ref - obs.pose[2])))
        out.setpoint = self._vel([vx, vy, vz], self.hold_yaw)
        if err < self.lc['align_tolerance']:
            self.t_aligned = self.t_aligned or obs.t
            surface_ok = self._safety_fresh(obs, 1.0) and obs.safety.get('surface_ok', False)
            if obs.t - self.t_aligned >= self.lc['align_hold_s'] and surface_ok:
                self.next_cp = 0
                agl = self._height_agl(obs)
                while self.next_cp < len(self.checkpoints) and agl < self.checkpoints[self.next_cp] + 0.1:
                    self.next_cp += 1
                self.cp_wait_since = None
                self._goto(DESCEND, obs.t, out, f'aligned (err {err:.2f} m), LiDAR surface OK')
        else:
            self.t_aligned = None

    def _blind(self, obs):
        return self._height_agl(obs) < float(self.lc.get('blind_height', 0.0))

    def _st_descend(self, obs, out, plan, dt):
        self._update_target(obs)
        if not self._target_fresh(obs, self.lc['target_lost_timeout_s']) and not self._blind(obs):
            self._enter_target_lost(obs, out)
            return
        bad = self._surface_unsafe(obs)
        if bad:
            self._enter_unsafe(obs, out, bad)
            return
        ex, ey, vx, vy = self._xy_control(obs, dt)
        agl = self._height_agl(obs)
        err = math.hypot(ex, ey)
        vz = -(self.lc['slow_descent_rate'] if agl < 1.2 else self.lc['descent_rate'])
        if err > 2.0 * self.lc['align_tolerance']:
            vz = 0.0     # pause descent until re-aligned
        # safety re-check at checkpoint heights
        if self.next_cp < len(self.checkpoints) and agl <= self.checkpoints[self.next_cp]:
            vz = 0.0
            if self.cp_wait_since is None:
                self.cp_wait_since = obs.t
                out.events.append(('INFO', f'checkpoint {self.checkpoints[self.next_cp]:.1f} m: re-validating'))
            fresh_ok = (self._safety_fresh(obs, 0.5) and obs.safety.get('surface_ok', False)
                        and obs.safety.get('stamp', 0) >= self.cp_wait_since)
            if fresh_ok and err < 1.5 * self.lc['align_tolerance']:
                out.events.append(('INFO', f'checkpoint {self.checkpoints[self.next_cp]:.1f} m passed'))
                self.next_cp += 1
                self.cp_wait_since = None
            elif obs.t - self.cp_wait_since > 5.0:
                self._enter_unsafe(obs, out, ['CHECKPOINT_TIMEOUT'])
                return
        out.setpoint = self._vel([vx, vy, vz], self.hold_yaw)
        if agl <= self.lc['touchdown_height']:
            self._goto(FINAL_CHECK, obs.t, out, f'touchdown zone ({agl:.2f} m AGL)')

    def _st_final_check(self, obs, out, plan, dt):
        self._update_target(obs)
        ex, ey, vx, vy = (0.0, 0.0, 0.0, 0.0)
        if self.target_filt is not None:
            ex, ey, vx, vy = self._xy_control(obs, dt)
        out.setpoint = self._vel([vx, vy, 0.0], self.hold_yaw)
        bad = self._surface_unsafe(obs)
        if bad:
            self._enter_unsafe(obs, out, bad)
            return
        err = math.hypot(ex, ey)
        if obs.t - self.t_state >= 0.5:
            if err <= 1.5 * self.lc['align_tolerance']:
                self.stats['touchdown_estimate'] = [float(obs.pose[0]), float(obs.pose[1])]
                self.stats['landing_target_estimate'] = self.target_filt[:2].tolist()
                self._goto(LAND, obs.t, out, f'final check passed (err {err:.2f} m)')
                out.command = 'land'
                self.last_cmd_t = obs.t
            elif obs.t - self.t_state > 4.0:
                self._goto(ALIGN, obs.t, out, 'final alignment out of tolerance')
                self.hold_pos = np.array(obs.pose, dtype=float)

    def _st_land(self, obs, out, plan, dt):
        # touchdown = PX4 land detector, or PX4 auto-disarm after landing (COM_DISARM_LAND)
        if obs.landed or (not obs.armed and obs.t - self.t_state > 1.0):
            self.outcome = 'LANDED'
            self.stats['mission_end_t'] = obs.t
            self._goto(LANDED, obs.t, out, 'touchdown confirmed by PX4 land detector')
            out.command = 'disarm'
            return
        if obs.t - self.t_state < 3.0 and obs.t - self.last_cmd_t > 1.0:
            out.command = 'land'
            self.last_cmd_t = obs.t

    def _st_landed(self, obs, out, plan, dt):
        if obs.armed and obs.t - self.last_cmd_t > 2.0:
            out.command = 'disarm'
            self.last_cmd_t = obs.t

    def _st_target_lost(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        self._update_target(obs)
        if self._target_fresh(obs, 0.3):
            self.stats['reacquire_times_s'].append(obs.t - self.t_target_lost)
            self.pid_x.reset()
            self.pid_y.reset()
            self.t_aligned = None
            self._goto(ALIGN, obs.t, out, f'target reacquired after {obs.t - self.t_target_lost:.1f} s')
            return
        if obs.t - self.t_state > self.lc['reacquire_timeout_s']:
            self.hold_pos = np.array([obs.pose[0], obs.pose[1], self.home[2] + self.search_alt])
            self._goto(ABORT_CLIMB, obs.t, out, 'reacquire timeout -> climb and search')

    def _st_unsafe_pad(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        if obs.t - self.t_state > 0.5:
            if self.abort_attempts > int(self.lc['max_abort_attempts']):
                self.outcome = 'UNSAFE_LANDING_ZONE'
                self._goto(ABORTED, obs.t, out, 'landing pad remained unsafe')
                return
            self.hold_pos = np.array([obs.pose[0], obs.pose[1], self.home[2] + float(self.lc['abort_climb_altitude'])])
            self._goto(ABORT_CLIMB, obs.t, out, f'abort attempt {self.abort_attempts}: climbing')

    def _st_abort_climb(self, obs, out, plan, dt):
        out.setpoint = self._hold(self.hold_pos, self.hold_yaw)
        if abs(obs.pose[2] - self.hold_pos[2]) < 0.3 and obs.t - self.t_state > 3.0:
            self.target_filt = None
            self._build_search(self.hold_pos[2])
            self.search_loops = 0
            self._goto(LANDING_ZONE_SEARCH, obs.t, out, 'climb complete, searching again')

    # ---------------------------------------------------------------- report
    def summary(self) -> dict:
        s = dict(self.stats)
        n = s.pop('spatial_error_n')
        tot = s.pop('spatial_error_sum')
        s['mean_spatial_error'] = tot / n if n else None
        s['outcome'] = self.outcome
        s['state'] = self.state
        return s
