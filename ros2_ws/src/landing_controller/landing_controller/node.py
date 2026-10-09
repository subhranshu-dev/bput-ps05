"""Mission manager node: runs the explicit MissionStateMachine at 20 Hz (Phases 13, 17, 18).

Inputs (AUTONOMY DATA ONLY): /vehicle/odom, /px4_interface/state, /sensors/health,
/avoidance/status, /landing/target_pose, /landing/safety_status, planner service.
Outputs: /px4_interface/setpoint, /px4_interface/command, /local_trajectory,
/mission/status, /mission/events.
"""
import json

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String

from planner_3d.trajectory import Trajectory
from ps05_common import config, topics
from ps05_common.frames import quat_from_yaw, yaw_from_quat
from ps05_common.ros_utils import LATCHED_QOS, RELIABLE_QOS, json_msg, now_sec, stamp_to_sec
from ps05_interfaces.srv import PlanPath

from .state_machine import FAILURE_STATES, MissionStateMachine, Observation


def trajectory_from_path(path: Path) -> Trajectory:
    pos = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses])
    t = np.array([p.header.stamp.sec + p.header.stamp.nanosec * 1e-9 for p in path.poses])
    yaw = np.array([yaw_from_quat((p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y,
                                   p.pose.orientation.z)) for p in path.poses])
    vel = np.zeros_like(pos)
    if len(pos) > 1:
        dt = np.maximum(np.diff(t), 1e-6)[:, None]
        v = np.diff(pos, axis=0) / dt
        vel[:-1] = v
        vel[1:-1] = 0.5 * (v[:-1] + v[1:])
    speed = np.linalg.norm(vel, axis=1)
    return Trajectory(t, pos, vel, yaw, speed)


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.mcfg = config.load('mission')
        self.lcfg = config.load('landing')
        self.scfg = config.load('safety')
        self.sm = MissionStateMachine(self.mcfg, self.lcfg, self.scfg)
        self.wd = self.scfg['watchdog']
        self.pub_sp = self.create_publisher(String, topics.PX4_SETPOINT, RELIABLE_QOS)
        self.pub_cmd = self.create_publisher(String, topics.PX4_COMMAND, RELIABLE_QOS)
        self.pub_traj = self.create_publisher(Path, topics.LOCAL_TRAJECTORY, LATCHED_QOS)
        self.pub_status = self.create_publisher(String, topics.MISSION_STATUS, RELIABLE_QOS)
        self.pub_events = self.create_publisher(String, topics.MISSION_EVENTS, RELIABLE_QOS)
        self.cli = self.create_client(PlanPath, '/planner/plan_path')
        self.create_subscription(Odometry, topics.VEHICLE_ODOM, self.on_odom, 10)
        self.create_subscription(String, topics.PX4_STATE, self.on_px4, RELIABLE_QOS)
        self.create_subscription(String, topics.SENSOR_HEALTH, self.on_health, RELIABLE_QOS)
        self.create_subscription(String, topics.AVOIDANCE_STATUS, self.on_avoid, RELIABLE_QOS)
        self.create_subscription(PoseStamped, topics.LANDING_TARGET, self.on_target, RELIABLE_QOS)
        self.create_subscription(String, topics.LANDING_SAFETY, self.on_safety, RELIABLE_QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/tracker_pose', self.on_tracker, 10)
        self.tracker = None
        self.tracker_t = -1e9
        self.odom = None
        self.odom_t = -1e9
        self.px4 = {}
        self.health = {}
        self.avoid = None
        self.target = None
        self.safety = None
        self.plan_result = None
        self.traj_stamp = None
        self.traj_pub_id = 0
        self.events = []
        self.last_setpoint = None
        self.create_timer(0.05, self.tick)

    # ------------------------------------------------------------------ inputs
    def on_odom(self, m):
        self.odom = m
        self.odom_t = now_sec(self)

    def on_px4(self, m):
        self.px4 = json.loads(m.data)

    def on_health(self, m):
        h = json.loads(m.data)
        h['rx'] = now_sec(self)
        self.health[h['sensor']] = h

    def on_avoid(self, m):
        self.avoid = json.loads(m.data)

    def on_target(self, m):
        self.target = {'pos': [m.pose.position.x, m.pose.position.y, m.pose.position.z],
                       'stamp': stamp_to_sec(m.header.stamp)}

    def on_tracker(self, m):
        p = m.pose.pose.position
        self.tracker = np.array([p.x, p.y, p.z])
        self.tracker_t = now_sec(self)

    def on_safety(self, m):
        self.safety = json.loads(m.data)

    def _sensor_ok(self, name, timeout, t):
        h = self.health.get(name)
        return bool(h and h.get('ok') and t - h['rx'] < timeout)

    # ------------------------------------------------------------------ planner
    def request_plan(self, req):
        if not self.cli.service_is_ready():
            self.plan_result = {'id': req['id'], 'success': False, 'message': 'planner service unavailable'}
            return
        r = PlanPath.Request()
        r.start = Point(x=req['start'][0], y=req['start'][1], z=req['start'][2])
        r.goal = Point(x=req['goal'][0], y=req['goal'][1], z=req['goal'][2])
        r.reason = req['reason']
        fut = self.cli.call_async(r)
        fut.add_done_callback(lambda f, rid=req['id']: self.on_plan_done(f, rid))

    def on_plan_done(self, fut, rid):
        try:
            res = fut.result()
        except Exception as e:  # noqa: BLE001
            self.plan_result = {'id': rid, 'success': False, 'message': f'planner call failed: {e}'}
            return
        out = {'id': rid, 'success': res.success, 'message': res.message, 'latency_ms': res.latency_ms,
               'length_m': res.length_m}
        if res.success and res.trajectory.poses:
            out['trajectory'] = trajectory_from_path(res.trajectory)
        elif res.success:
            out.update(success=False, message='empty trajectory')
        self.plan_result = out

    # ------------------------------------------------------------------ main loop
    def tick(self):
        t = now_sec(self)
        pose = yaw = vel = None
        odom_ok = self.odom is not None and t - self.odom_t < float(self.wd['odom_timeout_s'])
        if odom_ok:
            p = self.odom.pose.pose
            pose = np.array([p.position.x, p.position.y, p.position.z])
            yaw = yaw_from_quat((p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z))
            tw = self.odom.twist.twist.linear
            vel = np.array([tw.x, tw.y, tw.z])
        sensors = {
            'odom': odom_ok,
            'lidar': self._sensor_ok('lidar', float(self.wd['lidar_timeout_s']), t),
            'camera': self._sensor_ok('camera', float(self.wd['camera_timeout_s']), t),
            'down_camera': self._sensor_ok('down_camera', float(self.wd['down_camera_timeout_s']), t),
            'down_lidar': self._sensor_ok('down_lidar', float(self.wd['down_lidar_timeout_s']), t),
        }
        av = None
        if self.avoid is not None and self.traj_stamp is not None and \
                abs((self.avoid.get('trajectory_stamp') or -1) - self.traj_stamp) < 1e-6:
            av = dict(self.avoid)
            av['trajectory_id'] = self.traj_pub_id
        plan, self.plan_result = self.plan_result, None
        safety = None
        if self.safety is not None and self.safety.get('reference') == 'target':
            safety = self.safety
        loc_conv = bool(pose is not None and self.tracker is not None and t - self.tracker_t < 0.5
                        and np.linalg.norm(pose - self.tracker) < 0.3)
        obs = Observation(t=t, pose=pose, loc_converged=loc_conv, yaw=yaw or 0.0, vel=vel, px4_connected=bool(self.px4.get('connected')),
                          armed=bool(self.px4.get('armed')), offboard=bool(self.px4.get('offboard')),
                          landed=bool(self.px4.get('landed', True)), sensors=sensors, plan_result=plan,
                          avoidance=av, target=self.target, safety=safety)
        out = self.sm.step(obs)
        if out.trajectory is not None:
            self.publish_trajectory(out.trajectory)
        if out.plan_request is not None:
            self.request_plan(out.plan_request)
        if out.setpoint is not None:
            self.last_setpoint = out.setpoint
            self.pub_sp.publish(json_msg(out.setpoint))
        if out.command:
            m = String()
            m.data = out.command
            self.pub_cmd.publish(m)
        for level, text in out.events:
            ev = {'t': t, 'level': level, 'state': out.state, 'text': text}
            self.events.append(ev)
            self.pub_events.publish(json_msg(ev))
            if level == 'ERROR':
                self.get_logger().error(text)
            elif level == 'WARN':
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)
        s = self.sm.summary()
        self.pub_status.publish(json_msg({
            'stamp': t, 'state': out.state, 'failure': out.state in FAILURE_STATES, 'outcome': s['outcome'],
            'spatial_error': self.sm.spatial_error, 'e_max': self.sm.e_max, 'e_warn': self.sm.e_warn,
            'reference': None if self.sm.ref_pos is None else np.asarray(self.sm.ref_pos).tolist(),
            'pose': None if pose is None else pose.tolist(), 'localisation_converged': loc_conv, 'yaw': yaw, 'velocity': None if vel is None else vel.tolist(),
            'goal': None if self.sm.goal is None else self.sm.goal.tolist(),
            'trajectory_stamp': self.traj_stamp, 'trajectory_progress': (self.sm.traj_time / self.sm.traj.duration)
            if self.sm.traj is not None and self.sm.traj.duration > 0 else None,
            'setpoint': self.last_setpoint, 'sensors': sensors, 'target': self.target,
            'landing_safety': {k: self.safety.get(k) for k in ('safe', 'surface_ok', 'reasons', 'tilt_deg', 'plane_rms',
                                                               'height_above_ground', 'alignment_error',
                                                               'obstruction_points')} if self.safety else None,
            'pid': {'x': self.sm.pid_x.last, 'y': self.sm.pid_y.last},
            'stats': s, 'events': self.events[-30:],
        }))

    def publish_trajectory(self, traj: Trajectory):
        self.traj_pub_id = self.sm.traj_id
        p = Path()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        self.traj_stamp = stamp_to_sec(p.header.stamp)
        for i, x in enumerate(traj.pos):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, x)
            q = quat_from_yaw(float(traj.yaw[i]))
            ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = map(float, q)
            p.poses.append(ps)
        self.pub_traj.publish(p)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
