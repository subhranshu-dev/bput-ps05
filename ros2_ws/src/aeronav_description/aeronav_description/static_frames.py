"""Static TF2 tree for the PS05 vehicle (Phase 5).

    world -> map -> odom -> base_link            (odom->base_link: localization_node)
    base_link -> camera_link -> camera_optical_frame
    base_link -> lidar_link -> lidar_frame
    base_link -> down_camera_link -> down_camera_optical_frame
    base_link -> down_lidar_link -> down_lidar_frame

Mount poses come from configs/sensors.yaml (the same values are used in the
Gazebo model; tests/unit/test_tf_model_consistency.py checks they agree).
An optional camera extrinsic perturbation (robustness test) is applied here.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from ps05_common import config
from ps05_common.frames import matrix_to_quat, rpy_to_matrix


def make_tf(parent, child, xyz, rpy, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    q = matrix_to_quat(rpy_to_matrix(*rpy))
    t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = map(float, q)
    return t


def static_transforms(cfg: dict):
    """Return list of (parent, child, xyz, rpy). Pure function for unit tests."""
    pert = cfg.get('camera_extrinsic_perturbation') or {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    cam = cfg['camera_link']
    cam_xyz = list(np.add(cam['xyz'], pert['xyz']))
    cam_rpy = list(np.add(cam['rpy'], pert['rpy']))
    opt = cfg['optical_rpy']
    z3 = [0.0, 0.0, 0.0]
    return [
        ('world', 'map', z3, z3),
        ('map', 'odom', z3, z3),
        ('base_link', 'camera_link', cam_xyz, cam_rpy),
        ('camera_link', 'camera_optical_frame', z3, opt),
        ('base_link', 'lidar_link', cfg['lidar_link']['xyz'], cfg['lidar_link']['rpy']),
        ('lidar_link', 'lidar_frame', z3, z3),
        ('base_link', 'down_camera_link', cfg['down_camera_link']['xyz'], cfg['down_camera_link']['rpy']),
        ('down_camera_link', 'down_camera_optical_frame', z3, opt),
        ('base_link', 'down_lidar_link', cfg['down_lidar_link']['xyz'], cfg['down_lidar_link']['rpy']),
        ('down_lidar_link', 'down_lidar_frame', z3, z3),
    ]


class StaticFrames(Node):
    def __init__(self):
        super().__init__('static_frames')
        cfg = config.load('sensors')
        self.b = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        tfs = [make_tf(p, c, xyz, rpy, stamp) for p, c, xyz, rpy in static_transforms(cfg)]
        self.b.sendTransform(tfs)
        pert = cfg.get('camera_extrinsic_perturbation')
        self.get_logger().info(f'published {len(tfs)} static transforms; camera perturbation={pert}')


def main():
    rclpy.init()
    node = StaticFrames()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
