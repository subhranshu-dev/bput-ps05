"""Scenario event harness (TEST HARNESS — part of the simulated world, not autonomy).

Executes the ``events`` list of the active scenario file, moving Gazebo models
with the ``/world/<world>/set_pose`` service:

  - {type: spawn_ahead, object: dyn_panel, trigger: {state: NAVIGATE, delay_s: 6},
     distance_ahead: 9.0}                 # unexpected obstacle across the planned path
  - {type: crossing, object: dyn_panel, trigger: {...}, from: [x,y,z], to: [x,y,z], speed: 1.0}
  - {type: place, object: unsafe_crate, pose: [x,y,z], trigger: {state: ALIGN},
     remove_after_s: 12}                  # temporary landing-pad obstruction
Triggers: state (mission state reached), delay_s (after trigger), at_time (sim s).
Uses the ground-truth pose + published planned path only to *place* the obstacle.
"""
import json
import subprocess
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String

from ps05_common import config, topics
from ps05_common.ros_utils import LATCHED_QOS, RELIABLE_QOS, SENSOR_QOS, json_msg, now_sec


def set_pose(world: str, name: str, pos, yaw: float = 0.0) -> bool:
    qz, qw = np.sin(yaw / 2), np.cos(yaw / 2)
    req = (f'name: "{name}", position: {{x: {pos[0]:.3f}, y: {pos[1]:.3f}, z: {pos[2]:.3f}}}, '
           f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}')
    r = subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', req],
                       capture_output=True, text=True, timeout=5)
    return 'data: true' in r.stdout


class ScenarioEvents(Node):
    def __init__(self):
        super().__init__('scenario_events')
        self.declare_parameter('world', 'ps05_challenge')
        self.world = self.get_parameter('world').value
        self.scen = config.load_scenario()
        self.events = [dict(e, _state='armed') for e in (self.scen.get('events') or [])]
        self.pub = self.create_publisher(String, topics.SCENARIO_EVENTS, RELIABLE_QOS)
        self.create_subscription(String, topics.MISSION_STATUS, self.on_mission, RELIABLE_QOS)
        self.create_subscription(Odometry, topics.TRUTH_ODOM, self.on_truth, SENSOR_QOS)
        self.create_subscription(Path, topics.LOCAL_TRAJECTORY, self.on_traj, LATCHED_QOS)
        self.state = None
        self.state_t = {}
        self.truth = None
        self.traj = None
        self.t0 = None
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"scenario {self.scen.get('id', 'baseline')}: {len(self.events)} events")

    def on_mission(self, m):
        s = json.loads(m.data).get('state')
        if s != self.state:
            self.state = s
            self.state_t.setdefault(s, now_sec(self))

    def on_truth(self, m):
        p = m.pose.pose.position
        self.truth = np.array([p.x, p.y, p.z])

    def on_traj(self, m):
        if m.poses:
            self.traj = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses])

    def emit(self, payload):
        self.pub.publish(json_msg({'stamp': now_sec(self), **payload}))

    def move(self, name, pos, yaw=0.0, note=''):
        def work():
            ok = set_pose(self.world, name, pos, yaw)
            self.emit({'type': 'object_pose', 'object': name, 'pos': list(map(float, pos)), 'yaw': yaw, 'ok': ok,
                       'note': note})
            self.get_logger().info(f'moved {name} -> {np.round(pos, 2).tolist()} ok={ok} {note}')
        threading.Thread(target=work, daemon=True).start()

    def triggered(self, ev, t):
        tr = ev.get('trigger', {}) or {}
        if 'at_time' in tr:
            return self.t0 is not None and t - self.t0 >= float(tr['at_time'])
        st = tr.get('state')
        if st and st in self.state_t:
            return t - self.state_t[st] >= float(tr.get('delay_s', 0.0))
        return False

    def tick(self):
        t = now_sec(self)
        self.t0 = self.t0 or t
        for ev in self.events:
            if ev['_state'] == 'armed' and self.triggered(ev, t):
                self.fire(ev, t)
            elif ev['_state'] == 'placed' and ev.get('remove_after_s') is not None and t - ev['_t'] >= float(ev['remove_after_s']):
                self.move(ev['object'], ev.get('park', [1.5, 48.5, 0.25]), note='removed')
                ev['_state'] = 'done'
            elif ev['_state'] == 'crossing':
                self.step_crossing(ev, t)

    def fire(self, ev, t):
        typ = ev['type']
        ev['_t'] = t
        if typ == 'spawn_ahead':
            if self.truth is None or self.traj is None:
                return
            i = int(np.argmin(np.linalg.norm(self.traj - self.truth, axis=1)))
            s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(self.traj[i:], axis=0), axis=1))])
            j = i + int(np.searchsorted(s, float(ev.get('distance_ahead', 9.0))))
            j = min(j, len(self.traj) - 2)
            p = self.traj[j]
            d = self.traj[min(j + 1, len(self.traj) - 1)] - self.traj[max(j - 1, 0)]
            yaw = float(np.arctan2(d[1], d[0]))   # panel's thin x-axis along the path -> its 4 m width blocks it
            height = float(ev.get('height', 7.0))
            self.move(ev['object'], [p[0], p[1], height / 2.0], yaw, note='spawn_ahead')
            self.emit({'type': 'event', 'event': 'unexpected_obstacle', 'object': ev['object'],
                       'path_point': p.tolist(), 'vehicle_truth': self.truth.tolist()})
            ev['_state'] = 'placed'
        elif typ == 'place':
            self.move(ev['object'], ev['pose'], float(ev.get('yaw', 0.0)), note='place')
            self.emit({'type': 'event', 'event': 'place', 'object': ev['object'], 'pos': ev['pose']})
            ev['_state'] = 'placed'
        elif typ == 'crossing':
            ev['_state'] = 'crossing'
            ev['_last'] = -1e9
        else:
            self.get_logger().error(f'unknown event type {typ}')
            ev['_state'] = 'done'

    def step_crossing(self, ev, t):
        if t - ev['_last'] < 0.2:
            return
        ev['_last'] = t
        a, b = np.array(ev['from'], dtype=float), np.array(ev['to'], dtype=float)
        dur = np.linalg.norm(b - a) / float(ev.get('speed', 1.0))
        f = min((t - ev['_t']) / max(dur, 1e-6), 1.0)
        self.move(ev['object'], a + f * (b - a), float(ev.get('yaw', 0.0)), note='crossing')
        if f >= 1.0:
            ev['_state'] = 'done'


def main():
    rclpy.init()
    node = ScenarioEvents()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
