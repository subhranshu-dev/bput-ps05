#!/usr/bin/env python3
"""Developer probe: fly to a point via px4_interface and record landing detection + LiDAR validation.

Requires the dev stack (scripts/dev_up.sh) plus detector_node and validator_node. Saves
results/debug/probe_<t>.npz with the downward cloud (map frame) and prints the verdicts.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from ps05_common.pc2 import xyz_from_cloud
from ps05_common.ros_utils import json_msg, lookup_matrix


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=42.0)
    ap.add_argument('--y', type=float, default=42.0)
    ap.add_argument('--z', type=float, default=5.0)
    ap.add_argument('--record', type=float, default=12.0)
    a, ros = ap.parse_known_args()
    rclpy.init(args=ros)
    n = rclpy.create_node('probe_landing')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    sp = n.create_publisher(String, '/px4_interface/setpoint', 10)
    cmd = n.create_publisher(String, '/px4_interface/command', 10)
    st = {'pos': None, 'px4': {}, 'safety': [], 'det': [], 'cloud': None}
    n.create_subscription(Odometry, '/vehicle/odom', lambda m: st.__setitem__('pos', np.array(
        [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])), 10)
    n.create_subscription(String, '/px4_interface/state', lambda m: st.__setitem__('px4', json.loads(m.data)), 10)
    n.create_subscription(String, '/landing/safety_status', lambda m: st['safety'].append(json.loads(m.data)), 10)
    n.create_subscription(String, '/landing/detection', lambda m: st['det'].append(json.loads(m.data)), 10)
    n.create_subscription(PointCloud2, '/perception/down_lidar/points', lambda m: st.__setitem__('cloud', m),
                          qos_profile_sensor_data)
    tfb = Buffer()
    TransformListener(tfb, n)

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(n, timeout_sec=0.02)

    def go(p):
        sp.publish(json_msg({'mode': 'position', 'pos': list(map(float, p)), 'vel': None, 'yaw': 0.0}))

    def c(s):
        m = String()
        m.data = s
        cmd.publish(m)

    spin(3)
    home = st['pos'].copy()
    for i in range(400):                                   # arm + takeoff
        go(home + [0, 0, 2.5] if st['px4'].get('armed') else home)
        if i % 20 == 0:
            c('offboard' if not st['px4'].get('offboard') else 'arm')
        spin(0.05)
        if st['px4'].get('armed') and abs(st['pos'][2] - home[2] - 2.5) < 0.3:
            break
    tgt = np.array([a.x, a.y, a.z])
    cur = st['pos'].copy()
    hi = np.array([cur[0], cur[1], 11.0])
    for wp in (hi, np.array([a.x, a.y, 11.0]), tgt):      # climb over everything, cross, descend
        for _ in range(2000):
            d = wp - st['pos']
            step = st['pos'] + d / max(np.linalg.norm(d), 1e-6) * min(2.0, np.linalg.norm(d))
            go(step)
            spin(0.05)
            if np.linalg.norm(d) < 0.3:
                break
    st['safety'].clear()
    st['det'].clear()
    for _ in range(int(a.record / 0.05)):
        go(tgt)
        spin(0.05)
    m = st['cloud']
    tm = lookup_matrix(tfb, 'map', m.header.frame_id, None)
    pts = xyz_from_cloud(m)
    pts = pts @ tm[:3, :3].T + tm[:3, 3]
    out = Path('/workspace/results/debug')
    out.mkdir(parents=True, exist_ok=True)
    f = out / f'probe_{int(time.time())}.npz'
    np.savez(f, pts=pts, pos=st['pos'], safety=json.dumps(st['safety'][-5:]), det=json.dumps(st['det'][-5:]))
    det = [d for d in st['det'] if d.get('detected')]
    print(json.dumps({'pos': st['pos'].round(2).tolist(), 'detections': f'{len(det)}/{len(st["det"])}',
                      'last_det': det[-1] if det else None,
                      'last_safety': st['safety'][-1] if st['safety'] else None, 'saved': str(f)}, indent=1))
    for _ in range(100):
        c('land')
        spin(0.1)
        if not st['px4'].get('armed'):
            break


if __name__ == '__main__':
    main()
