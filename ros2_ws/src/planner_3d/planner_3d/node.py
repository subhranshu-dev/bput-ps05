"""3D global planner node (Phases 9-12).

Service /planner/plan_path (ps05_interfaces/PlanPath):
    occupancy grid -> planning grid (max-pool) -> obstacle inflation (configs/safety.yaml)
    -> approved-corridor mask (configs/mission.yaml) -> 3D A* -> line-of-sight pruning
    -> Chaikin smoothing (verified) -> time-parameterised trajectory
    -> FINAL VALIDATION: map collision + inflation clearance + corridor (every sample)
Publishes /planned_path, /planner/status, /mission/corridor (markers), /planner/inflated.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from mapping.voxel_map import decode_states
from ps05_common import config, topics
from ps05_common.frames import quat_from_yaw
from ps05_common.pc2 import make_cloud
from ps05_common.ros_utils import LATCHED_QOS, RELIABLE_QOS, json_msg, now_sec
from ps05_interfaces.msg import VoxelGrid
from ps05_interfaces.srv import PlanPath

from .astar import VALIDATION_STEP, AStarPlanner, polyline_clear
from .corridor import Corridor, densify
from .grid import build_planning_grid, inflation_radius
from .trajectory import build_trajectory


def validate_trajectory(pos: np.ndarray, grid, corridor: Corridor, radius: float) -> dict:
    dense = densify(pos, VALIDATION_STEP)
    blocked = grid.is_blocked_world(dense)
    cor = corridor.validate_path(dense)
    clear = grid.clearance_world(dense)
    return {'collision_free': bool(not blocked.any()), 'blocked_samples': int(blocked.sum()),
            'corridor_ok': cor['ok'], 'corridor_violations': cor['violations'],
            'max_centerline_deviation': cor['max_centerline_deviation'],
            'min_clearance_m': float(clear.min()) if len(clear) else None,
            'clearance_ok': bool(len(clear) and clear.min() >= radius - 1e-6)}


def path_msg(pts, frame, stamp, yaws=None, times=None):
    p = Path()
    p.header.frame_id = frame
    p.header.stamp = stamp
    for i, x in enumerate(pts):
        ps = PoseStamped()
        ps.header.frame_id = frame
        if times is not None:
            sec = float(times[i])
            ps.header.stamp.sec = int(sec)
            ps.header.stamp.nanosec = int((sec - int(sec)) * 1e9)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, x)
        q = quat_from_yaw(float(yaws[i])) if yaws is not None else (1.0, 0.0, 0.0, 0.0)
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = map(float, q)
        p.poses.append(ps)
    return p


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.pcfg = config.load('planner')
        self.scfg = config.load('safety')
        self.mcfg = config.load('mission')
        self.res = float(self.pcfg['astar']['resolution'])
        self.radius = inflation_radius(self.scfg, self.res)
        self.corridor = Corridor.from_config(self.mcfg)
        self.planner = AStarPlanner(self.pcfg)
        self.grid_msg = None
        self.grid = None
        self.grid_stamp = None
        self.corridor_mask = None
        cb_grid = MutuallyExclusiveCallbackGroup()
        cb_srv = ReentrantCallbackGroup()
        self.create_subscription(VoxelGrid, topics.OCCUPANCY_GRID, self.on_grid, RELIABLE_QOS, callback_group=cb_grid)
        self.srv = self.create_service(PlanPath, '/planner/plan_path', self.on_plan, callback_group=cb_srv)
        self.pub_path = self.create_publisher(Path, topics.PLANNED_PATH, LATCHED_QOS)
        self.pub_status = self.create_publisher(String, topics.PLANNER_STATUS, LATCHED_QOS)
        self.pub_corr = self.create_publisher(MarkerArray, topics.CORRIDOR_MARKERS, LATCHED_QOS)
        self.pub_infl = self.create_publisher(PointCloud2, '/planner/inflated', LATCHED_QOS)
        self.history = []
        self.publish_corridor()
        self.get_logger().info(f'inflation radius = {self.radius:.2f} m, resolution = {self.res} m, '
                               f"unknown policy = {self.pcfg['astar']['unknown_policy']}")

    def on_grid(self, m: VoxelGrid):
        self.grid_msg = m

    def _ensure_grid(self):
        m = self.grid_msg
        if m is None:
            return False
        if self.grid_stamp == (m.header.stamp.sec, m.header.stamp.nanosec):
            return True
        states = decode_states(m.data, m.nx, m.ny, m.nz)
        self.grid_states = (states, m.resolution, [m.origin.x, m.origin.y, m.origin.z])
        self.grid = build_planning_grid(states, m.resolution, [m.origin.x, m.origin.y, m.origin.z], self.res, self.radius)
        if self.corridor_mask is None or self.corridor_mask.shape != self.grid.shape:
            self.corridor_mask = self.corridor.grid_mask(self.grid.shape, self.grid.origin, self.grid.resolution)
        self.grid_stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
        return True

    def on_plan(self, req, resp):
        t0 = time.perf_counter()
        start = np.array([req.start.x, req.start.y, req.start.z])
        goal = np.array([req.goal.x, req.goal.y, req.goal.z])
        if not self._ensure_grid():
            resp.success = False
            resp.message = 'no occupancy map received yet'
            return resp
        grid = self.grid
        res = self.planner.plan(grid, start, goal, self.corridor, self.corridor_mask)
        status = {'stamp': now_sec(self), 'reason': req.reason, 'start': start.tolist(), 'goal': goal.tolist(),
                  'success': res.success, 'message': res.message, 'expansions': res.expansions,
                  'astar_ms': round(res.latency_ms, 2), 'inflation_radius': self.radius,
                  'unknown_policy': self.planner.unknown_policy}
        if res.success:
            trav = self.planner.traversable_mask(grid, self.corridor_mask)

            def valid(poly):
                return polyline_clear(poly, trav, grid)

            if res.start_snapped:
                # escape segment already verified collision-free against occupied cells; validate the rest normally
                esc, body = res.waypoints[:2], res.waypoints[1:]
                traj_body = build_trajectory(body, self.pcfg['trajectory'], valid)
                traj = build_trajectory(np.vstack([esc[:1], traj_body.polyline]),
                                        dict(self.pcfg['trajectory'], smoothing_iterations=0), lambda p: True)
                check = validate_trajectory(traj_body.polyline, grid, self.corridor, self.radius)
                check['escape_segment_m'] = float(np.linalg.norm(esc[1] - esc[0]))
            else:
                traj = build_trajectory(res.waypoints, self.pcfg['trajectory'], valid)
                check = validate_trajectory(traj.polyline, grid, self.corridor, self.radius)
            if not (check['collision_free'] and check['corridor_ok']) and not res.start_snapped:
                # fall back to the un-smoothed, already validated waypoint polyline
                traj = build_trajectory(res.waypoints, dict(self.pcfg['trajectory'], smoothing_iterations=0), valid)
                check = validate_trajectory(traj.polyline, grid, self.corridor, self.radius)
                check['smoothing_fallback'] = True
            status.update(check)
            if not (check['collision_free'] and check['corridor_ok']):
                res.success = False
                status['success'] = False
                why = ('collision' if not check['collision_free'] else '') + (' corridor' if not check['corridor_ok'] else '')
                status['message'] = res.message = f'trajectory failed final validation ({why.strip()})'
                self.dump_failure(req, res, traj, check)
        latency = (time.perf_counter() - t0) * 1e3
        status['latency_ms'] = round(latency, 2)
        resp.success = res.success
        resp.message = res.message
        resp.latency_ms = latency
        resp.expansions = res.expansions
        if res.success:
            stamp = self.get_clock().now().to_msg()
            resp.path = path_msg(res.waypoints, 'map', stamp)
            resp.trajectory = path_msg(traj.pos, 'map', stamp, traj.yaw, traj.t)
            resp.length_m = float(traj.length())
            status.update({'length_m': resp.length_m, 'waypoints': int(len(res.waypoints)),
                           'raw_cells': int(len(res.raw_waypoints)), 'duration_s': traj.duration,
                           'path': np.round(res.waypoints, 2).tolist()})
            self.pub_path.publish(resp.path)
            self.publish_inflated(grid)
        self.history.append({k: status.get(k) for k in ('stamp', 'reason', 'success', 'latency_ms', 'length_m')})
        status['history'] = self.history[-20:]
        self.pub_status.publish(json_msg(status))
        self.get_logger().info(f"plan[{req.reason}] success={res.success} {res.message} "
                               f"latency={latency:.0f} ms len={resp.length_m:.1f} m")
        return resp

    def dump_failure(self, req, res, traj, check):
        """Save the exact planning inputs of a rejected plan for offline reproduction."""
        try:
            d = config.repo_root() / 'results' / 'debug'
            d.mkdir(parents=True, exist_ok=True)
            states, resolution, origin = self.grid_states   # exactly the grid this plan used
            f = d / f'plan_fail_{int(time.time() * 1000)}.npz'
            np.savez_compressed(f, states=states, resolution=resolution, origin=origin,
                                start=[req.start.x, req.start.y, req.start.z], goal=[req.goal.x, req.goal.y, req.goal.z],
                                waypoints=res.waypoints, raw=res.raw_waypoints, traj=traj.pos,
                                check=json.dumps(check))
            self.get_logger().warn(f'final validation failed {check}; inputs saved to {f}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'could not dump failure: {e}')

    def publish_inflated(self, grid):
        idx = np.argwhere(grid.blocked & ~grid.occupied)
        if len(idx) > 30000:
            idx = idx[np.random.default_rng(0).choice(len(idx), 30000, replace=False)]
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'map'
        self.pub_infl.publish(make_cloud(h, grid.index_to_world(idx)))

    def publish_corridor(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        box = Marker()
        box.header.frame_id = 'map'
        box.header.stamp = stamp
        box.ns, box.id, box.type, box.action = 'mission_space', 0, Marker.CUBE, Marker.ADD
        c = (self.corridor.lo + self.corridor.hi) / 2
        s = self.corridor.hi - self.corridor.lo
        box.pose.position.x, box.pose.position.y, box.pose.position.z = map(float, c)
        box.pose.orientation.w = 1.0
        box.scale.x, box.scale.y, box.scale.z = map(float, s)
        box.color.r, box.color.g, box.color.b, box.color.a = 0.2, 0.8, 1.0, 0.05
        ma.markers.append(box)
        if self.corridor.tube_enabled:
            line = Marker()
            line.header = box.header
            line.ns, line.id, line.type, line.action = 'centerline', 1, Marker.LINE_STRIP, Marker.ADD
            line.scale.x = 0.15
            line.color.r, line.color.g, line.color.b, line.color.a = 0.2, 1.0, 0.6, 0.9
            line.pose.orientation.w = 1.0
            for p in self.corridor.centerline:
                line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
            ma.markers.append(line)
            for i, p in enumerate(self.corridor.centerline):
                sph = Marker()
                sph.header = box.header
                sph.ns, sph.id, sph.type, sph.action = 'tube', 10 + i, Marker.SPHERE, Marker.ADD
                sph.pose.position.x, sph.pose.position.y, sph.pose.position.z = map(float, p)
                sph.pose.orientation.w = 1.0
                d = 2 * (self.corridor.max_dev - self.corridor.margin)
                sph.scale.x = sph.scale.y = sph.scale.z = float(d)
                sph.color.r, sph.color.g, sph.color.b, sph.color.a = 0.2, 1.0, 0.6, 0.04
                ma.markers.append(sph)
        self.pub_corr.publish(ma)


def main():
    rclpy.init()
    node = PlannerNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
