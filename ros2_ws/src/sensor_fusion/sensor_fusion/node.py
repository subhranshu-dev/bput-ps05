"""LiDAR + camera fusion node (Phases 6-7).

LiDAR PointCloud2 -> TF2 (lidar_frame -> camera_optical_frame at the scan stamp)
-> pinhole projection with the live CameraInfo -> RGB lookup -> /fused/points
(x, y, z, rgb, in_fov) in the LiDAR frame, stamped with the LiDAR stamp.

Synchronisation is explicit: each scan is paired with the nearest camera frame
and rejected if |dt| > max_delta (configs/fusion.yaml). Points outside the
image or behind the camera keep their geometry but are NOT coloured
(rgb = grey, in_fov = 0) — no fake colours.
"""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from ps05_common import config, topics
from ps05_common.frames import rpy_to_matrix
from ps05_common.pc2 import make_cloud, xyz_from_cloud
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, Stopwatch, json_msg, lookup_matrix, now_sec, stamp_to_sec

from .projection import (colorize, depth_discontinuity_mask, edge_chamfer_px, intrinsics_from_camera_info, pack_rgb,
                         reprojection_shift, transform_points)
from .sync import TimestampSynchronizer


def nominal_cam_from_lidar(sensors_cfg) -> np.ndarray:
    """Design (unperturbed) camera_optical <- lidar_frame transform from the mount config (metric only)."""
    def t(xyz, rpy):
        m = np.eye(4)
        m[:3, :3] = rpy_to_matrix(*rpy)
        m[:3, 3] = xyz
        return m
    base_cam = t(sensors_cfg['camera_link']['xyz'], sensors_cfg['camera_link']['rpy']) @ t([0, 0, 0], sensors_cfg['optical_rpy'])
    base_lidar = t(sensors_cfg['lidar_link']['xyz'], sensors_cfg['lidar_link']['rpy'])
    return np.linalg.inv(base_cam) @ base_lidar


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.cfg = config.load('fusion')
        self.sensors = config.load('sensors')
        self.t_nominal = nominal_cam_from_lidar(self.sensors)
        s = self.cfg['sync']
        self.sync = TimestampSynchronizer(float(s['max_delta_s']), int(s['queue_size']))
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.k = None
        self.pub = self.create_publisher(PointCloud2, self.cfg['output_topic'], SENSOR_QOS)
        self.pub_status = self.create_publisher(String, topics.FUSION_STATUS, RELIABLE_QOS)
        self.create_subscription(Image, self.cfg['image_topic'], self.on_image, SENSOR_QOS)
        self.create_subscription(CameraInfo, self.cfg['info_topic'], self.on_info, SENSOR_QOS)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self.on_cloud, SENSOR_QOS)
        self.n_scans = 0
        self.tf_failures = 0
        self.edge_hist = []

    def on_info(self, m):
        try:
            self.k = intrinsics_from_camera_info(m.k)
        except ValueError as e:
            self.get_logger().error(str(e))

    def on_image(self, m):
        self.sync.add_camera(stamp_to_sec(m.header.stamp), m)

    def on_cloud(self, cloud: PointCloud2):
        sw = Stopwatch()
        self.n_scans += 1
        t_l = stamp_to_sec(cloud.header.stamp)
        img_msg, delta = self.sync.match_lidar(t_l)
        status = {'stamp': now_sec(self), 'lidar_stamp': t_l, 'sync_delta_s': delta, **self.sync.stats(),
                  'tf_failures': self.tf_failures, 'published': False}
        if img_msg is None or self.k is None:
            status['reason'] = 'no camera frame within max_delta' if self.k is not None else 'no CameraInfo'
            self.pub_status.publish(json_msg(status))
            return
        cam_frame = img_msg.header.frame_id or self.cfg['camera_frame']
        t_cam_lidar = lookup_matrix(self.tf_buffer, cam_frame, cloud.header.frame_id, cloud.header.stamp, 0.05)
        if t_cam_lidar is None:
            self.tf_failures += 1
            status['reason'] = f'TF {cam_frame} <- {cloud.header.frame_id} unavailable'
            status['tf_failures'] = self.tf_failures
            self.pub_status.publish(json_msg(status))
            return
        pts = xyz_from_cloud(cloud)
        img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
        rgb, in_fov, u, v = colorize(pts, t_cam_lidar, self.k, img, float(self.cfg['min_depth']))
        out = make_cloud(cloud.header, pts, pack_rgb(rgb), {'in_fov': in_fov.astype(np.float32)})
        self.pub.publish(out)
        status.update({'published': True, 'points': int(len(pts)), 'points_in_fov': int(in_fov.sum()),
                       'fov_fraction': float(in_fov.mean()) if len(pts) else 0.0, 'process_ms': round(sw.ms(), 2),
                       'image_size': [int(img.shape[1]), int(img.shape[0])],
                       'fx': float(self.k[0, 0]), 'fy': float(self.k[1, 1]),
                       'cx': float(self.k[0, 2]), 'cy': float(self.k[1, 2])})
        # --- fusion projection-error metrics (calibration quality)
        mean_shift, p95_shift, _ = reprojection_shift(pts, self.t_nominal, t_cam_lidar, self.k, img.shape[1], img.shape[0])
        status['extrinsic_shift_px_mean'] = mean_shift
        status['extrinsic_shift_px_p95'] = p95_shift
        ec = self.cfg.get('edge_alignment', {})
        if ec.get('enabled', True) and self.n_scans % int(ec.get('every_n', 10)) == 0 and in_fov.sum() > 50:
            import cv2
            edges = cv2.Canny(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), 50, 150)
            pc = transform_points(t_cam_lidar, pts)
            sil = depth_discontinuity_mask(pc, 1, 0.75) & in_fov
            if sil.sum() > 10:
                chamfer, n_sil = edge_chamfer_px(edges, u[sil], v[sil])
                self.edge_hist = (self.edge_hist + [chamfer])[-50:]
                status['edge_chamfer_px'] = chamfer
                status['edge_points'] = n_sil
        if self.edge_hist:
            status['edge_chamfer_px_mean'] = float(np.mean(self.edge_hist))
        self.pub_status.publish(json_msg(status))


def main():
    rclpy.init()
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
