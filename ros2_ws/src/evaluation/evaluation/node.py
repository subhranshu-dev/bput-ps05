"""Evaluation node (Phase 19) — the ONLY consumer of simulator ground truth (besides the
tracking-station sensor model). Records every run under results/raw/<run_id>/.

Measures: localisation error (estimate vs truth), true spatial error E vs the
reference, minimum clearance / collisions / near misses against the world
geometry, executed path length, landing error at touchdown, plus timings that
the autonomy reports (planning/replanning latency, detection, reacquisition).
"""
import csv
import json
import os
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from ps05_common import config, topics
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, json_msg, now_sec, stamp_to_sec

from .metrics import (error_stats, interpolate_track, landing_error, load_world, map_accuracy, min_clearance, path_length,
                      world_surfaces)


class EvaluationNode(Node):
    def __init__(self):
        super().__init__('evaluation_node')
        self.declare_parameter('run_id', os.environ.get('PS05_RUN_ID', time.strftime('run_%Y%m%d_%H%M%S')))
        self.declare_parameter('results_dir', str(config.repo_root() / 'results' / 'raw'))
        self.run_id = self.get_parameter('run_id').value
        self.dir = Path(self.get_parameter('results_dir').value) / self.run_id
        self.dir.mkdir(parents=True, exist_ok=True)
        self.scenario = config.load_scenario()
        spec_path = config.repo_root() / 'simulation/configs/world_ps05.yaml'
        self.world = yaml.safe_load(open(spec_path))
        self.prims = load_world(self.world)
        self.pad = np.array(self.world['landing_pad']['center'], dtype=float)
        safety = config.load('safety')['inflation']
        self.collision_radius = float(safety['vehicle_radius'])
        self.near_miss_radius = float(safety['vehicle_radius'] + safety['desired_clearance'])
        all_cfg = config.load_all()
        (self.dir / 'config.json').write_text(json.dumps({'scenario': self.scenario, 'configs': all_cfg,
                                                          'config_hash': config.config_hash(all_cfg)}, indent=1))
        self.cfg_hash = config.config_hash(all_cfg)
        self.truth_t, self.truth_p = [], []
        self.est_t, self.est_p = [], []
        self.loc_err = []
        self.true_e = []
        self.clearance_min = float('inf')
        self.clearance_obj = None
        self.collisions = 0
        self.in_collision = False
        self.near_misses = 0
        self.in_near = False
        self.mission = {}
        self.state = 'INIT'
        self.landed_xy = None
        self.landing_error = None
        self.det_frames = 0
        self.det_hits = 0
        self.fusion = {}
        self.planner = {}
        self.events_f = open(self.dir / 'events.jsonl', 'a')
        self.tel_f = open(self.dir / 'telemetry.csv', 'w', newline='')
        self.tel = csv.writer(self.tel_f)
        self.tel.writerow(['t', 'truth_x', 'truth_y', 'truth_z', 'est_x', 'est_y', 'est_z', 'ref_x', 'ref_y', 'ref_z',
                           'loc_err', 'true_E', 'clearance', 'state'])
        self.t0 = None
        self.finalized = False
        self.moved = set()
        self.create_subscription(Odometry, topics.TRUTH_ODOM, self.on_truth, SENSOR_QOS)
        self.create_subscription(Odometry, topics.VEHICLE_ODOM, self.on_est, 10)
        self.create_subscription(String, topics.MISSION_STATUS, self.on_mission, RELIABLE_QOS)
        self.create_subscription(String, topics.MISSION_EVENTS, lambda m: self.log_event('mission', m), RELIABLE_QOS)
        self.create_subscription(String, topics.SCENARIO_EVENTS, self.on_scenario, RELIABLE_QOS)
        self.create_subscription(String, topics.PLANNER_STATUS, self.on_planner, RELIABLE_QOS)
        self.create_subscription(String, topics.FUSION_STATUS, self.on_fusion, RELIABLE_QOS)
        self.create_subscription(String, topics.LANDING_DETECTION, self.on_detection, RELIABLE_QOS)
        self.surfaces = world_surfaces(self.world)
        self.map_acc = None
        self.create_subscription(PointCloud2, topics.OCCUPANCY_MAP, self.on_map, RELIABLE_QOS)
        self.pub = self.create_publisher(String, topics.EVAL_METRICS, RELIABLE_QOS)
        self.create_timer(0.1, self.sample)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'evaluation run {self.run_id} -> {self.dir}')

    # ------------------------------------------------------------------ inputs
    def on_truth(self, m):
        p = m.pose.pose.position
        self.truth_t.append(stamp_to_sec(m.header.stamp))
        self.truth_p.append(np.array([p.x, p.y, p.z]))
        if len(self.truth_t) > 200000:
            self.truth_t, self.truth_p = self.truth_t[-100000:], self.truth_p[-100000:]

    def on_est(self, m):
        p = m.pose.pose.position
        self.est_t.append(stamp_to_sec(m.header.stamp))
        self.est_p.append(np.array([p.x, p.y, p.z]))
        if len(self.est_t) > 200000:
            self.est_t, self.est_p = self.est_t[-100000:], self.est_p[-100000:]

    def on_mission(self, m):
        self.mission = json.loads(m.data)
        self.state = self.mission.get('state', self.state)
        if self.state in ('LANDED', 'ABORTED') and not self.finalized:
            self._land_check()

    def on_scenario(self, m):
        ev = json.loads(m.data)
        if ev.get('type') == 'object_pose':
            self.moved.add(ev['object'])
            for o in self.surfaces:
                if o['name'] == ev['object']:
                    o['pos'] = np.array(ev['pos'], dtype=float)
            for o in self.prims:
                if o['name'] == ev['object']:
                    o['pos'] = np.array(ev['pos'], dtype=float)
        self.log_event('scenario', m)

    def on_map(self, m):
        # evaluation of the autonomy's map against the true world (rate-limited)
        t = now_sec(self)
        if t - getattr(self, '_map_t', -1e9) < 5.0:
            return
        self._map_t = t
        from ps05_common.pc2 import xyz_from_cloud
        # dynamic objects are included at their current (parked or moved) pose: they are LiDAR-visible
        self.map_acc = map_accuracy(xyz_from_cloud(m), self.surfaces)

    def on_planner(self, m):
        self.planner = json.loads(m.data)

    def on_fusion(self, m):
        f = json.loads(m.data)
        if f.get('published'):
            self.fusion = f

    def on_detection(self, m):
        if self.state in ('TARGET_LOCK', 'ALIGN', 'DESCEND', 'FINAL_CHECK', 'TARGET_LOST'):
            d = json.loads(m.data)
            self.det_frames += 1
            self.det_hits += int(bool(d.get('detected')))

    def log_event(self, src, m):
        ev = json.loads(m.data)
        ev['source'] = src
        ev['wall_time'] = time.time()
        self.events_f.write(json.dumps(ev) + '\n')
        self.events_f.flush()

    # ------------------------------------------------------------------ sampling
    def sample(self):
        if not self.truth_t:
            return
        t = self.truth_t[-1]
        self.t0 = self.t0 or t
        p = self.truth_p[-1]
        clr, name = min_clearance(p, self.prims)
        airborne = self.state not in ('INIT', 'ARMING', 'LANDED')
        est = interpolate_track(np.array(self.est_t[-50:]), np.array(self.est_p[-50:]), t) if len(self.est_t) > 2 else None
        loc = float(np.linalg.norm(est - p)) if est is not None else None
        if loc is not None and airborne:
            self.loc_err.append(loc)
        ref = self.mission.get('reference')
        te = float(np.linalg.norm(p - np.array(ref))) if ref and self.state == 'NAVIGATE' else None
        if te is not None:
            self.true_e.append(te)
        if airborne:
            if clr < self.clearance_min:
                self.clearance_min, self.clearance_obj = clr, name
            if clr < self.collision_radius:
                if not self.in_collision:
                    self.collisions += 1
                    self.get_logger().error(f'COLLISION with {name} (clearance {clr:.2f} m)')
                self.in_collision = True
            else:
                self.in_collision = False
            if clr < self.near_miss_radius:
                if not self.in_near:
                    self.near_misses += 1
                self.in_near = True
            else:
                self.in_near = False
        e = est if est is not None else [None] * 3
        r = ref if ref else [None] * 3
        self.tel.writerow([round(t, 3), *np.round(p, 3), *(np.round(e, 3) if est is not None else e), *r, loc, te,
                           round(clr, 3), self.state])

    def _land_check(self):
        if self.state == 'LANDED' and self.truth_p:
            p = self.truth_p[-1]
            self.landed_xy = p[:2].tolist()
            self.landing_error = landing_error(p[:2], self.pad)
        self.finalize()

    # ------------------------------------------------------------------ output
    def metrics(self) -> dict:
        st = self.mission.get('stats', {}) or {}
        truth = np.array(self.truth_p) if self.truth_p else np.zeros((0, 3))
        start_t = st.get('mission_start_t')
        end_t = st.get('mission_end_t')
        flown = truth
        if start_t is not None and len(self.truth_t):
            tt = np.array(self.truth_t)
            sel = tt >= start_t
            if end_t:
                sel &= tt <= end_t
            flown = truth[sel]
        rep_lat = [x for x in st.get('replan_latencies_ms', []) if x is not None]
        return {
            'run_id': self.run_id,
            'scenario_id': self.scenario.get('id', 'baseline'),
            'seed': self.scenario.get('seed', 0),
            'config_hash': self.cfg_hash,
            'state': self.state,
            'outcome': self.mission.get('outcome'),
            'success': self.mission.get('outcome') == 'LANDED' and self.collisions == 0,
            'landing_error_m': self.landing_error,
            'touchdown_xy': self.landed_xy,
            'pad_center_xy': self.pad.tolist(),
            'collision_count': self.collisions,
            'near_misses': self.near_misses,
            'min_clearance_m': None if self.clearance_min == float('inf') else self.clearance_min,
            'min_clearance_object': self.clearance_obj,
            'executed_path_length_m': path_length(flown[::5]) if len(flown) > 5 else None,
            'planned_path_lengths_m': st.get('path_lengths_m'),
            'planning_latency_ms': st.get('plan_latencies_ms'),
            'replanning_latency_ms': rep_lat,
            'replans': st.get('replans'),
            'obstacle_detections': st.get('obstacle_detections'),
            'detection_events': st.get('detection_events'),
            'stop_response_s': st.get('stop_response_s'),
            'target_detection_rate': (self.det_hits / self.det_frames) if self.det_frames else None,
            'target_frames': self.det_frames,
            'target_losses': st.get('target_losses'),
            'target_reacquisition_s': st.get('reacquire_times_s'),
            'aborts': st.get('aborts'),
            'corridor_breaches': st.get('corridor_breaches'),
            'sensor_failures': st.get('sensor_failures'),
            'mission_time_s': (end_t - start_t) if (start_t is not None and end_t is not None) else None,
            'localization_error': error_stats(self.loc_err),
            'true_spatial_error': error_stats(self.true_e),
            'estimated_spatial_error_mean': st.get('mean_spatial_error'),
            'estimated_spatial_error_max': st.get('max_spatial_error'),
            'fusion_sync_mean_abs_delta_s': self.fusion.get('mean_abs_delta_s'),
            'fusion_sync_rejected': self.fusion.get('rejected'),
            'fusion_sync_accepted': self.fusion.get('accepted'),
            'fusion_edge_chamfer_px': self.fusion.get('edge_chamfer_px_mean'),
            'fusion_extrinsic_shift_px': self.fusion.get('extrinsic_shift_px_mean'),
            'fusion_process_ms': self.fusion.get('process_ms'),
            'map_accuracy': self.map_acc,
            'state_history': st.get('state_history'),
            'elapsed_sim_s': (self.truth_t[-1] - self.t0) if self.truth_t and self.t0 else None,
        }

    def publish(self):
        self.pub.publish(json_msg(self.metrics()))
        self.tel_f.flush()

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        m = self.metrics()
        (self.dir / 'metrics.json').write_text(json.dumps(m, indent=2, default=str))
        self.tel_f.flush()
        self.get_logger().info(f"run {self.run_id} finalized: outcome={m['outcome']} landing_error={m['landing_error_m']}")

    def destroy_node(self):
        if not self.finalized:
            m = self.metrics()
            m['note'] = 'run ended before LANDED/ABORTED (timeout or shutdown)'
            (self.dir / 'metrics.json').write_text(json.dumps(m, indent=2, default=str))
        self.tel_f.close()
        self.events_f.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = EvaluationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
