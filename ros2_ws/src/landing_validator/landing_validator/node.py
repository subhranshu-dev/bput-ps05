"""LiDAR landing-safety validation node (Phase 16).

Downward flash-LiDAR cloud -> TF2 (map) -> plane fit + clearance + alignment
around the detected target (or under the vehicle if no target yet)
-> /landing/safety_status (JSON). A marker alone never triggers descent.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from ps05_common import config, topics
from ps05_common.pc2 import xyz_from_cloud
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, Stopwatch, json_msg, lookup_matrix, stamp_to_sec

from .surface import validate


class ValidatorNode(Node):
    def __init__(self):
        super().__init__('validator_node')
        self.cfg = config.load('landing')['validator']
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target = None
        self.target_t = -1e9
        self.pose = None
        self.pub = self.create_publisher(String, topics.LANDING_SAFETY, RELIABLE_QOS)
        self.create_subscription(PoseStamped, topics.LANDING_TARGET, self.on_target, RELIABLE_QOS)
        self.create_subscription(Odometry, topics.VEHICLE_ODOM, self.on_odom, 10)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self.on_cloud, SENSOR_QOS)

    def on_target(self, m):
        self.target = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.target_t = stamp_to_sec(m.header.stamp)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pose = np.array([p.x, p.y, p.z])

    def on_cloud(self, m: PointCloud2):
        if self.pose is None:
            return
        sw = Stopwatch()
        tmat = lookup_matrix(self.tf_buffer, 'map', m.header.frame_id, m.header.stamp, 0.05)
        if tmat is None:
            return
        pts = xyz_from_cloud(m)
        pts = pts @ tmat[:3, :3].T + tmat[:3, 3]
        t = stamp_to_sec(m.header.stamp)
        use_target = self.target is not None and t - self.target_t < 2.0
        center = self.target[:2] if use_target else self.pose[:2]
        r = validate(pts, center, self.pose, self.cfg)
        r.update({'stamp': t, 'reference': 'target' if use_target else 'vehicle', 'center': center.tolist(),
                  'points': int(len(pts)), 'process_ms': round(sw.ms(), 2)})
        self.pub.publish(json_msg(r))


def main():
    rclpy.init()
    node = ValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
