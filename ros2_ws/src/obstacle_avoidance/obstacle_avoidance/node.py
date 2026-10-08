"""Local obstacle-avoidance monitor (Phase 13). Independent of the global planner (Rule 9).

For the active trajectory (/local_trajectory) it checks the look-ahead window
in front of the vehicle against
  (a) every new LiDAR scan (TF2 -> map), inflated tube radius from safety.yaml;
  (b) the latest occupancy grid, inflated with the same radius.
It publishes /avoidance/status; the mission state machine reacts with
STOP/HOVER -> REPLAN inside the approved space.
"""
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from mapping.voxel_map import decode_states
from planner_3d.grid import build_planning_grid, inflation_radius
from ps05_common import config, topics
from ps05_common.pc2 import xyz_from_cloud
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, json_msg, lookup_matrix, now_sec, stamp_to_sec
from ps05_interfaces.msg import VoxelGrid

from .path_monitor import check_grid, check_points, lookahead_segment


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        self.scfg = config.load('safety')
        self.pcfg = config.load('planner')
        lc = self.scfg['local_check']
        self.lookahead = float(lc['lookahead_m'])
        self.min_points = int(lc['min_points'])
        self.res = float(self.pcfg['astar']['resolution'])
        inf = dict(self.scfg['inflation'])
        # LiDAR points are exact surface samples: no grid-quantisation margin needed
        self.point_radius = inflation_radius({'inflation': {**inf, 'grid_quantization': False}}, self.res)
        self.grid_radius = inflation_radius(self.scfg, self.res)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.traj = None
        self.traj_stamp = None
        self.pose = None
        self.grid = None
        self.pub = self.create_publisher(String, topics.AVOIDANCE_STATUS, RELIABLE_QOS)
        self.create_subscription(Path, topics.LOCAL_TRAJECTORY, self.on_traj, RELIABLE_QOS)
        self.create_subscription(Odometry, topics.VEHICLE_ODOM, self.on_odom, 10)
        self.create_subscription(PointCloud2, '/perception/lidar/points', self.on_cloud, SENSOR_QOS)
        self.create_subscription(VoxelGrid, topics.OCCUPANCY_GRID, self.on_grid, RELIABLE_QOS)

    def on_traj(self, m: Path):
        if not m.poses:
            self.traj = None
            return
        self.traj = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses])
        self.traj_stamp = stamp_to_sec(m.header.stamp)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pose = np.array([p.x, p.y, p.z])

    def _window(self):
        i = int(np.argmin(np.linalg.norm(self.traj - self.pose, axis=1)))
        return lookahead_segment(self.traj, i, self.lookahead)

    def on_grid(self, m: VoxelGrid):
        t0 = time.perf_counter()
        states = decode_states(m.data, m.nx, m.ny, m.nz)
        self.grid = build_planning_grid(states, m.resolution, [m.origin.x, m.origin.y, m.origin.z], self.res,
                                        self.grid_radius)
        if self.traj is None or self.pose is None:
            return
        seg, along = self._window()
        r = check_grid(seg, along, self.grid)
        if r['blocked']:
            self.publish(r['blocked'], 'map', r['distance_along_path'], None, stamp_to_sec(m.header.stamp),
                         (time.perf_counter() - t0) * 1e3)

    def on_cloud(self, m: PointCloud2):
        if self.traj is None or self.pose is None:
            return
        t0 = time.perf_counter()
        tmat = lookup_matrix(self.tf_buffer, 'map', m.header.frame_id, m.header.stamp, 0.05)
        if tmat is None:
            return
        pts = xyz_from_cloud(m)
        pts = pts @ tmat[:3, :3].T + tmat[:3, 3]
        seg, along = self._window()
        r = check_points(seg, along, pts, self.point_radius, self.min_points)
        self.publish(r['blocked'], 'lidar', r['distance_along_path'], r['min_clearance'],
                     stamp_to_sec(m.header.stamp), (time.perf_counter() - t0) * 1e3, r.get('obstacle_point'))

    def publish(self, blocked, source, dist, min_clear, stamp, ms, point=None):
        self.pub.publish(json_msg({
            'stamp': now_sec(self), 'sensor_stamp': stamp, 'trajectory_stamp': self.traj_stamp,
            'blocked': bool(blocked), 'source': source, 'distance_along_path': dist,
            'min_clearance_lookahead': min_clear, 'check_ms': round(ms, 2), 'obstacle_point': point,
            'point_radius': self.point_radius, 'grid_radius': self.grid_radius}))


def main():
    rclpy.init()
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
