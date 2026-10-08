"""3D occupancy mapping node (Phase 8).

Input:  /fused/points (preferred; carries RGB) — falls back to the filtered raw
        LiDAR cloud if the fused stream is stale (camera failure = degraded mode).
TF2:    map <- cloud frame at the scan stamp (pose from GPS-free localisation).
Output: /occupancy_map (PointCloud2 of occupied voxel centres, RGB from fusion)
        /occupancy_map/grid (ps05_interfaces/VoxelGrid, FREE/UNKNOWN/OCCUPIED, zlib)
        /mapping/status (JSON)
Mapping knows nothing about planning (Rule 8).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

from ps05_common import config, topics
from ps05_common.pc2 import cloud_fields, cloud_to_array, make_cloud
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, Stopwatch, json_msg, lookup_matrix, now_sec
from ps05_interfaces.msg import VoxelGrid

from sensor_fusion.projection import pack_rgb, unpack_rgb

from .voxel_map import OCCUPIED, VoxelMap, encode_states


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.cfg = config.load('mapping')
        self.map = VoxelMap.from_config(self.cfg)
        self.color = np.full(self.map.shape + (3,), 160, dtype=np.uint8)
        self.frame = self.cfg['frame']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_cloud = self.create_publisher(PointCloud2, topics.OCCUPANCY_MAP, RELIABLE_QOS)
        self.pub_grid = self.create_publisher(VoxelGrid, topics.OCCUPANCY_GRID, RELIABLE_QOS)
        self.pub_status = self.create_publisher(String, topics.MAP_STATUS, RELIABLE_QOS)
        self.create_subscription(PointCloud2, self.cfg['input_topic'], lambda m: self.on_cloud(m, 'fused'), SENSOR_QOS)
        self.create_subscription(PointCloud2, self.cfg['fallback_topic'], lambda m: self.on_cloud(m, 'lidar'), SENSOR_QOS)
        self.last_fused = -1e9
        self.source = 'none'
        self.update_ms = 0.0
        self.scans = {'fused': 0, 'lidar': 0}
        self.tf_fail = 0
        self.create_timer(1.0 / float(self.cfg['publish_rate_hz']), self.publish)

    def on_cloud(self, m: PointCloud2, kind: str):
        t = now_sec(self)
        if kind == 'fused':
            self.last_fused = t
        elif t - self.last_fused < float(self.cfg['fallback_after_s']):
            return   # fused stream healthy; raw LiDAR not needed
        sw = Stopwatch()
        tmat = lookup_matrix(self.tf_buffer, self.frame, m.header.frame_id, m.header.stamp, 0.1)
        if tmat is None:
            self.tf_fail += 1
            return
        f = cloud_fields(m)
        has_rgb = 'rgb' in f and 'in_fov' in f
        arr = cloud_to_array(m, ('x', 'y', 'z', 'rgb', 'in_fov') if has_rgb else ('x', 'y', 'z'))
        pts = arr[:, :3] @ tmat[:3, :3].T + tmat[:3, 3]
        origin = tmat[:3, 3]
        self.map.integrate(pts, origin, float(self.cfg['max_ray_length']))
        if has_rgb:
            sel = arr[:, 4] > 0.5
            if sel.any():
                idx = self.map.world_to_index(pts[sel])
                ok = self.map.in_bounds(idx)
                rgb = unpack_rgb(arr[sel, 3].astype(np.float32))[ok]
                i = idx[ok]
                self.color[i[:, 0], i[:, 1], i[:, 2]] = rgb
        self.source = kind
        self.scans[kind] += 1
        self.update_ms = sw.ms()

    def publish(self):
        sw = Stopwatch()
        states = self.map.states()
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.frame
        occ_idx = np.argwhere(states == OCCUPIED)
        centers = self.map.index_to_world(occ_idx)
        rgb = self.color[occ_idx[:, 0], occ_idx[:, 1], occ_idx[:, 2]] if len(occ_idx) else np.zeros((0, 3), np.uint8)
        self.pub_cloud.publish(make_cloud(h, centers, pack_rgb(rgb)))
        g = VoxelGrid()
        g.header = h
        g.resolution = self.map.res
        g.origin.x, g.origin.y, g.origin.z = map(float, self.map.origin)
        g.nx, g.ny, g.nz = self.map.shape
        g.data = encode_states(states)
        g.occupied_count = int(len(occ_idx))
        g.free_count = int((states == 0).sum())
        self.pub_grid.publish(g)
        total = states.size
        self.pub_status.publish(json_msg({
            'stamp': h.stamp.sec + h.stamp.nanosec * 1e-9, 'source': self.source, 'scans': self.scans,
            'occupied': int(len(occ_idx)), 'free': int(g.free_count),
            'unknown': int(total - len(occ_idx) - g.free_count), 'resolution': self.map.res,
            'shape': list(self.map.shape), 'update_ms': round(self.update_ms, 2), 'publish_ms': round(sw.ms(), 2),
            'compressed_bytes': len(g.data), 'tf_failures': self.tf_fail,
            'degraded': self.source == 'lidar'}))


def main():
    rclpy.init()
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
