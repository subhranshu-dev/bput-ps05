"""Landing-target detection node (Phase 15).

down camera -> AprilTag 36h11 detection -> PnP pose in camera optical frame
-> TF2 (map <- down_camera_optical_frame at the image stamp) -> /landing/target_pose (map)
The marker is a precision-landing aid only; navigation to the landing region
uses the GPS-free navigation stack.
"""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from ps05_common import config, topics
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, Stopwatch, json_msg, lookup_matrix, now_sec, stamp_to_sec

from .marker import MarkerDetector, pixel_error


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        c = config.load('landing')['detector']
        self.cfg = c
        self.det = MarkerDetector(c['dictionary'], c['marker_id'], c['marker_size'])
        self.alpha = float(c['filter_alpha'])
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.k = None
        self.dist = None
        self.filt = None
        self.frames = 0
        self.detections = 0
        self.pub_pose = self.create_publisher(PoseStamped, topics.LANDING_TARGET, RELIABLE_QOS)
        self.pub_det = self.create_publisher(String, topics.LANDING_DETECTION, RELIABLE_QOS)
        self.pub_dbg = self.create_publisher(Image, '/landing/debug_image', SENSOR_QOS)
        self.create_subscription(CameraInfo, c['info_topic'], self.on_info, SENSOR_QOS)
        self.create_subscription(Image, c['image_topic'], self.on_image, SENSOR_QOS)

    def on_info(self, m):
        self.k = np.array(m.k, dtype=float).reshape(3, 3)
        self.dist = np.array(m.d, dtype=float) if len(m.d) else None

    def on_image(self, m: Image):
        if self.k is None:
            return
        sw = Stopwatch()
        self.frames += 1
        img = self.bridge.imgmsg_to_cv2(m, desired_encoding='mono8')
        d = self.det.detect(img, self.k, self.dist)
        status = {'stamp': now_sec(self), 'image_stamp': stamp_to_sec(m.header.stamp), 'detected': False,
                  'frames': self.frames, 'detections': self.detections}
        if d is not None:
            tmat = lookup_matrix(self.tf_buffer, 'map', m.header.frame_id, m.header.stamp, 0.05)
            if tmat is None:
                status['reason'] = 'TF map <- camera unavailable'
            else:
                p_map = tmat[:3, :3] @ d['tvec'] + tmat[:3, 3]
                if self.filt is None or np.linalg.norm(p_map - self.filt) > 1.0:
                    self.filt = p_map
                else:
                    self.filt = (1 - self.alpha) * self.filt + self.alpha * p_map
                self.detections += 1
                ex, ey = pixel_error(d['center_px'], self.k)
                ps = PoseStamped()
                ps.header.stamp = m.header.stamp
                ps.header.frame_id = 'map'
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, self.filt)
                ps.pose.orientation.w = 1.0
                self.pub_pose.publish(ps)
                status.update({'detected': True, 'detections': self.detections, 'target_map': self.filt.tolist(),
                               'target_map_raw': p_map.tolist(), 'pixel_error': [ex, ey],
                               'center_px': d['center_px'].tolist(), 'range_m': float(np.linalg.norm(d['tvec'])),
                               'reprojection_px': d['reprojection_px'], 'area_px': d['area_px']})
        status['detection_rate'] = self.detections / max(self.frames, 1)
        status['process_ms'] = round(sw.ms(), 2)
        self.pub_det.publish(json_msg(status))


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
