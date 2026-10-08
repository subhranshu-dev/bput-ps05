"""LiDAR processing (Phase 4): range/self filtering, optional voxel downsampling, health,
and test-harness fault injection (Gaussian range noise, dropped points, blackout).
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from ps05_common import config, topics
from ps05_common.pc2 import make_cloud, voxel_downsample, xyz_from_cloud
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, RateMeter, Stopwatch, health_msg, now_sec


def filter_points(pts: np.ndarray, c: dict) -> np.ndarray:
    """Finite + range window + self filter + voxel downsample (order preserving)."""
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    r = np.linalg.norm(pts, axis=1)
    keep = (r >= float(c['min_range'])) & (r <= float(c['max_range']))
    sf = float(c.get('self_filter_radius', 0.0))
    if sf > 0:
        keep &= r > sf
    pts = pts[keep]
    return pts[voxel_downsample(pts, float(c.get('voxel_downsample', 0.0)))]


def inject(pts: np.ndarray, f: dict, rng) -> np.ndarray:
    sigma = float(f.get('range_noise_sigma', 0.0))
    if sigma > 0 and len(pts):
        r = np.linalg.norm(pts, axis=1, keepdims=True)
        pts = pts * (1.0 + rng.normal(0.0, sigma, (len(pts), 1)) / np.maximum(r, 1e-6))
    p = float(f.get('dropout_prob', 0.0))
    if p > 0 and len(pts):
        pts = pts[rng.random(len(pts)) >= p]
    return pts


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.cfg = config.load('lidar')
        self.faults = self.cfg.get('fault_injection', {}) or {}
        seed = int(config.load_scenario().get('seed', 0))
        self.rng = np.random.default_rng(seed + 3)
        self.pub_health = self.create_publisher(String, topics.SENSOR_HEALTH, RELIABLE_QOS)
        self.trigger_state = self.faults.get('trigger_state')
        self.trigger_t = None
        self.t_start = None
        self.create_subscription(String, topics.MISSION_STATUS, self.on_mission, RELIABLE_QOS)
        self.streams = {}
        for which in ('main', 'down'):
            c = self.cfg[which]
            st = {'which': which, 'cfg': c, 'rate': RateMeter(), 'n_in': 0, 'n_out': 0, 'ms': 0.0,
                  'frame_ok': True, 'blackout': False,
                  'pub': self.create_publisher(PointCloud2, c['output_topic'], SENSOR_QOS)}
            self.streams[which] = st
            self.create_subscription(PointCloud2, c['input_topic'], lambda m, s=st: self.on_cloud(m, s), SENSOR_QOS)
        self.create_timer(0.2, self.health)

    def on_mission(self, msg):
        if self.trigger_state and self.trigger_t is None and json.loads(msg.data).get('state') == self.trigger_state:
            self.trigger_t = now_sec(self)

    def on_cloud(self, m: PointCloud2, st):
        sw = Stopwatch()
        t = now_sec(self)
        self.t_start = self.t_start or t
        c = st['cfg']
        st['frame_ok'] = (m.header.frame_id == c['frame'])
        bo = self.faults.get('blackout', {}) or {}
        clock0 = self.trigger_t if self.trigger_state else self.t_start
        if bo.get('enabled') and st['which'] == bo.get('lidar', 'main') and clock0 is not None:
            tr = t - clock0
            st['blackout'] = bo['start_s'] <= tr < bo['start_s'] + bo['duration_s']
            if st['blackout']:
                return
        pts = xyz_from_cloud(m)
        st['n_in'] = len(pts)
        pts = filter_points(pts, c)
        if st['which'] == 'main' or self.faults.get('apply_to_down', False):
            pts = inject(pts, self.faults, self.rng)
        st['n_out'] = len(pts)
        out = make_cloud(m.header, pts)
        out.header.frame_id = c['frame']
        st['pub'].publish(out)
        st['rate'].tick(t)
        st['ms'] = sw.ms()

    def health(self):
        t = now_sec(self)
        for which, st in self.streams.items():
            age = st['rate'].age(t)
            name = 'lidar' if which == 'main' else 'down_lidar'
            self.pub_health.publish(health_msg(
                name, age < 1.0 and st['frame_ok'], t, rate_hz=round(st['rate'].rate, 2),
                expected_hz=st['cfg']['expected_rate_hz'], age_s=None if age == float('inf') else round(age, 3),
                points_in=st['n_in'], points_out=st['n_out'], process_ms=round(st['ms'], 2),
                frame_ok=st['frame_ok'], blackout=st['blackout']))


def main():
    rclpy.init()
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
