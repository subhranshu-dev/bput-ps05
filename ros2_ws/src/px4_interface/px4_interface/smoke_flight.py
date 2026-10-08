"""Phase 0/1 smoke flight: arm -> takeoff -> hover -> move -> hover -> land, via px4_interface.

Uses only autonomy data (/vehicle/odom from GPS-free localisation). Writes a
JSON verdict to --out and exits 0 on success, 1 on failure.
"""
import argparse
import json
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ps05_common import topics
from ps05_common.ros_utils import RELIABLE_QOS, json_msg, now_sec


class SmokeFlight(Node):
    def __init__(self, alt, move, hover_s):
        super().__init__('smoke_flight')
        self.alt, self.move, self.hover_s = alt, move, hover_s
        self.pub_sp = self.create_publisher(String, topics.PX4_SETPOINT, RELIABLE_QOS)
        self.pub_cmd = self.create_publisher(String, topics.PX4_COMMAND, RELIABLE_QOS)
        self.create_subscription(Odometry, topics.VEHICLE_ODOM, self.on_odom, 10)
        self.create_subscription(String, topics.PX4_STATE, self.on_state, RELIABLE_QOS)
        self.pos = None
        self.px4 = {}
        self.phase = 'WAIT'
        self.t_phase = None
        self.home = None
        self.result = {'phases': [], 'success': False}
        self.hover_err = []
        self.last_cmd = -1e9
        self.create_timer(0.05, self.tick)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pos = np.array([p.x, p.y, p.z])

    def on_state(self, m):
        self.px4 = json.loads(m.data)

    def goto(self, phase, t):
        self.result['phases'].append({'phase': phase, 't': round(t, 2),
                                      'pos': None if self.pos is None else self.pos.round(3).tolist()})
        self.get_logger().info(f'phase -> {phase}')
        self.phase = phase
        self.t_phase = t

    def sp(self, p):
        self.pub_sp.publish(json_msg({'mode': 'position', 'pos': list(map(float, p)), 'vel': None, 'yaw': 0.0}))

    def cmd(self, c, t):
        if t - self.last_cmd > 1.0:
            m = String()
            m.data = c
            self.pub_cmd.publish(m)
            self.last_cmd = t

    def tick(self):
        t = now_sec(self)
        if self.t_phase is None:
            self.t_phase = t
        el = t - self.t_phase
        if self.phase == 'WAIT':
            if self.pos is not None and self.px4.get('connected'):
                self.home = self.pos.copy()
                self.goto('ARM', t)
            elif el > 60:
                self.finish(False, 'PX4/localisation not available')
            return
        if self.phase == 'ARM':
            self.sp(self.home)
            self.cmd('offboard' if not self.px4.get('offboard') else 'arm', t)
            if self.px4.get('armed') and self.px4.get('offboard'):
                self.goto('TAKEOFF', t)
            elif el > 30:
                self.finish(False, 'arming failed')
        elif self.phase == 'TAKEOFF':
            tgt = self.home + [0, 0, self.alt]
            self.sp(tgt)
            if abs(self.pos[2] - tgt[2]) < 0.2:
                self.goto('HOVER', t)
            elif el > 30:
                self.finish(False, 'takeoff timeout')
        elif self.phase == 'HOVER':
            tgt = self.home + [0, 0, self.alt]
            self.sp(tgt)
            self.hover_err.append(float(np.linalg.norm(self.pos - tgt)))
            if el > self.hover_s:
                self.goto('MOVE', t)
        elif self.phase == 'MOVE':
            tgt = self.home + [self.move, 0, self.alt]
            self.sp(tgt)
            if np.linalg.norm(self.pos - tgt) < 0.25:
                self.goto('HOLD', t)
            elif el > 30:
                self.finish(False, 'move timeout')
        elif self.phase == 'HOLD':
            self.sp(self.home + [self.move, 0, self.alt])
            if el > 3:
                self.goto('LAND', t)
        elif self.phase == 'LAND':
            # no offboard setpoints in LAND: PX4 Land mode owns the descent; touchdown = landed or auto-disarmed
            if el < 3.0:
                self.cmd('land', t)
            if el > 2 and (self.px4.get('landed') or not self.px4.get('armed')):
                self.goto('LANDED', t)
                self.finish(True, 'ok')
            elif el > 60:
                self.finish(False, 'land timeout')

    def finish(self, ok, msg):
        self.result.update({'success': ok, 'message': msg,
                            'hover_rms_error_m': float(np.sqrt(np.mean(np.square(self.hover_err)))) if self.hover_err else None,
                            'hover_max_error_m': float(max(self.hover_err)) if self.hover_err else None})
        raise SystemExit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alt', type=float, default=2.5)
    ap.add_argument('--move', type=float, default=3.0)
    ap.add_argument('--hover', type=float, default=5.0)
    ap.add_argument('--out', default='')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = SmokeFlight(a.alt, a.move, a.hover)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    res = node.result
    print(json.dumps(res, indent=2))
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(res, f, indent=2)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if res.get('success') else 1)
