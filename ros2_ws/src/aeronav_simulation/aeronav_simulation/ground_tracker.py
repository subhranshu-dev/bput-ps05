"""Simulated ground tracking station — a SENSOR MODEL, part of the simulated world.

PS05 (Sec. 5-8) specifies an external radar / optical / acoustic tracker that
measures the UAV's spherical state (r, theta, phi) relative to the station and
transmits it over RF telemetry at > 50 Hz with < 20 ms tracking latency.

Like every Gazebo sensor plugin, this model reads the simulator's vehicle state
internally (/sim/truth/odom). Its OUTPUT is only the noisy, delayed spherical
measurement /tracking/measurement with configured sigmas. It never republishes
the truth. Noise/latency/dropout come from configs/localization.yaml and are
overridable per scenario (localisation-degradation robustness tests).
"""
import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time

from ps05_common import config, topics
from ps05_common.ros_utils import SENSOR_QOS, now_sec, stamp_to_sec
from ps05_interfaces.msg import TrackingMeasurement


class GroundTracker(Node):
    def __init__(self):
        super().__init__('ground_tracker')
        cfg = config.load('localization')['tracker']
        mission = config.load('mission')
        self.station = np.array(cfg['station_position'], dtype=float)
        self.latency = float(cfg['latency_s'])
        self.sr, self.sa, self.se = float(cfg['sigma_range']), float(cfg['sigma_azimuth']), float(cfg['sigma_elevation'])
        self.dropout = float(cfg.get('dropout_prob', 0.0))
        self.centerline = np.array(mission['corridor']['centerline'], dtype=float)
        seed = int(config.load_scenario().get('seed', 0))
        self.rng = np.random.default_rng(seed + 1)
        self.buf = deque(maxlen=400)
        self.pub = self.create_publisher(TrackingMeasurement, topics.TRACKING, SENSOR_QOS)
        self.create_subscription(Odometry, topics.TRUTH_ODOM, self.on_truth, SENSOR_QOS)
        self.create_timer(1.0 / float(cfg['rate_hz']), self.tick)

    def on_truth(self, m):
        p = m.pose.pose.position
        self.buf.append((stamp_to_sec(m.header.stamp), np.array([p.x, p.y, p.z])))

    def _sample(self, t):
        if not self.buf:
            return None
        ts = [b[0] for b in self.buf]
        if t <= ts[0]:
            return None
        i = int(np.searchsorted(ts, t))
        if i >= len(ts):
            return self.buf[-1][1] if t - ts[-1] < 0.1 else None
        t0, p0 = self.buf[i - 1]
        t1, p1 = self.buf[i]
        a = (t - t0) / max(t1 - t0, 1e-9)
        return p0 + a * (p1 - p0)

    def tick(self):
        t_now = now_sec(self)
        t_meas = t_now - self.latency
        p = self._sample(t_meas)
        if p is None or (self.dropout > 0 and self.rng.random() < self.dropout):
            return
        d = p - self.station
        r = float(np.linalg.norm(d))
        theta = math.atan2(d[1], d[0])
        phi = math.asin(d[2] / r) if r > 1e-6 else 0.0
        m = TrackingMeasurement()
        m.header.stamp = Time(seconds=t_meas).to_msg()
        m.header.frame_id = 'tracking_station'
        m.range = r + self.rng.normal(0.0, self.sr)
        m.azimuth = theta + self.rng.normal(0.0, self.sa)
        m.elevation = phi + self.rng.normal(0.0, self.se)
        m.sigma_range, m.sigma_azimuth, m.sigma_elevation = self.sr, self.sa, self.se
        # Vh: heading toward the nearest corridor-centreline point, computed from the *measured* position
        meas = self.station + np.array([m.range * math.cos(m.elevation) * math.cos(m.azimuth),
                                        m.range * math.cos(m.elevation) * math.sin(m.azimuth),
                                        m.range * math.sin(m.elevation)])
        vh = self._heading(meas)
        m.heading_vector.x, m.heading_vector.y, m.heading_vector.z = map(float, vh)
        self.pub.publish(m)

    def _heading(self, p):
        best, bd = None, 1e18
        for a, b in zip(self.centerline[:-1], self.centerline[1:]):
            ab = b - a
            t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
            q = a + t * ab
            d = np.linalg.norm(p - q)
            if d < bd:
                best, bd = q, d
        v = best - p
        n = np.linalg.norm(v)
        return v / n if n > 1e-6 else np.zeros(3)


def main():
    rclpy.init()
    node = GroundTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
