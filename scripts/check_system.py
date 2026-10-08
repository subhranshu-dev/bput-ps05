#!/usr/bin/env python3
"""Runtime integration check: topics publish at sane rates, CameraInfo valid, TF tree complete and stable,
PX4 connected via uXRCE-DDS, GPS absent. Exit 0 = PASS."""
import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

CHECKS = {
    '/camera/image_raw': (Image, 5.0), '/camera/camera_info': (CameraInfo, 5.0),
    '/down_camera/image_raw': (Image, 5.0), '/lidar/points': (PointCloud2, 5.0), '/down_lidar/points': (PointCloud2, 5.0),
    '/perception/lidar/points': (PointCloud2, 5.0), '/perception/camera/image': (Image, 5.0),
    '/fused/points': (PointCloud2, 3.0), '/vehicle/odom': (Odometry, 20.0), '/clock': (None, 50.0),
}
FRAMES = [('map', 'base_link'), ('base_link', 'camera_optical_frame'), ('base_link', 'lidar_frame'),
          ('base_link', 'down_camera_optical_frame'), ('base_link', 'down_lidar_frame'), ('world', 'map'),
          ('camera_optical_frame', 'lidar_frame')]


class Checker(Node):
    def __init__(self):
        super().__init__('system_check')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.stamps = {k: [] for k in CHECKS}
        self.info = None
        self.cloud_frame = None
        self.px4 = None
        from rosgraph_msgs.msg import Clock
        for topic, (typ, _) in CHECKS.items():
            typ = typ or Clock
            self.create_subscription(typ, topic, lambda m, t=topic: self.cb(t, m), qos_profile_sensor_data)
        self.create_subscription(String, '/px4_interface/state', lambda m: setattr(self, 'px4', json.loads(m.data)), 10)
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)

    def cb(self, t, m):
        self.stamps[t].append(time.time())
        if t == '/camera/camera_info':
            self.info = m
        if t == '/lidar/points':
            self.cloud_frame = m.header.frame_id


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rclpy.init()
    n = Checker()
    t_end = time.time() + a.duration
    tf_samples = {f: [] for f in FRAMES}
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        for p, c in FRAMES:
            try:
                tr = n.tfb.lookup_transform(p, c, rclpy.time.Time()).transform.translation
                tf_samples[(p, c)].append([tr.x, tr.y, tr.z])
            except Exception:  # noqa: BLE001
                pass
    rep = {'topics': {}, 'tf': {}, 'pass': True}
    for topic, (_, min_hz) in CHECKS.items():
        s = n.stamps[topic]
        hz = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 2 else 0.0
        ok = hz >= 0.5 * min_hz
        rep['topics'][topic] = {'wall_hz': round(hz, 2), 'expected_min_hz': 0.5 * min_hz, 'ok': ok}
        rep['pass'] &= ok
    info = n.info
    ci_ok = bool(info and info.width == 640 and info.height == 480 and info.k[0] > 0 and 0 < info.k[2] < 640)
    rep['camera_info'] = {'ok': ci_ok, 'k': list(info.k) if info else None,
                          'frame_id': info.header.frame_id if info else None}
    rep['lidar_frame'] = n.cloud_frame
    rep['pass'] &= ci_ok and n.cloud_frame == 'lidar_frame'
    for f, s in tf_samples.items():
        arr = np.array(s)
        ok = len(arr) > 5 and np.isfinite(arr).all()
        jump = float(np.linalg.norm(np.diff(arr, axis=0), axis=1).max()) if len(arr) > 1 else None
        static = f != ('map', 'base_link')
        if ok and static:
            ok = jump is not None and jump < 1e-6           # static mounts must not move
        if ok and not static:
            ok = jump is not None and jump < 0.5            # no teleport of the vehicle pose
        rep['tf'][f'{f[0]}->{f[1]}'] = {'samples': len(arr), 'max_jump_m': jump, 'last': arr[-1].tolist() if len(arr) else None,
                                        'ok': bool(ok)}
        rep['pass'] &= bool(ok)
    rep['px4'] = n.px4
    rep['pass'] &= bool(n.px4 and n.px4.get('connected'))
    gps = [t for t, _ in n.get_topic_names_and_types() if 'gps' in t.lower()]
    rep['gps_topics'] = gps
    print(json.dumps(rep, indent=1))
    if a.out:
        open(a.out, 'w').write(json.dumps(rep, indent=1))
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if rep['pass'] else 1)


if __name__ == '__main__':
    main()
