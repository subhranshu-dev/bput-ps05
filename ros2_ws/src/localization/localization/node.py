"""GPS-free localisation (Phase 14 / PS05 Sec. 7-9).

Tracking -> Telemetry -> Coordinate conversion -> EKF2 -> odometry:
* /tracking/measurement (r, theta, phi from the ground tracking station)
    -> X,Y,Z (PS05 polar->Cartesian) -> map frame
    -> PX4 /fmu/in/vehicle_visual_odometry (external position aiding, NED)
* PX4 /fmu/out/vehicle_odometry (EKF2: IMU + baro + mag + tracker; NO GPS)
    -> /vehicle/odom (nav_msgs/Odometry, map ENU) + TF odom -> base_link

This is the only pose source used by the autonomy stack.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster

from ps05_common import config, topics
from ps05_common.frames import enu_to_ned, ned_to_enu, quat_ned_frd_to_enu_flu
from ps05_common.ros_utils import PX4_QOS, RELIABLE_QOS, SENSOR_QOS, health_msg, now_sec, stamp_to_sec
from ps05_interfaces.msg import TrackingMeasurement
from std_msgs.msg import String

from .spherical import spherical_covariance, spherical_to_cartesian, station_to_map


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        cfg = config.load('localization')
        self.station = np.array(cfg['tracker']['station_position'], dtype=float)
        self.var_floor = float(cfg['ekf_feed']['position_variance_floor'])
        fr = cfg['frames']
        self.map_frame, self.odom_frame, self.base_frame = fr['map'], fr['odom'], fr['base']
        self.pub_ev = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', PX4_QOS)
        self.pub_odom = self.create_publisher(Odometry, topics.VEHICLE_ODOM, 10)
        self.pub_track = self.create_publisher(PoseWithCovarianceStamped, '/localization/tracker_pose', 10)
        self.pub_path = self.create_publisher(Path, topics.ACTUAL_PATH, RELIABLE_QOS)
        self.pub_health = self.create_publisher(String, topics.SENSOR_HEALTH, RELIABLE_QOS)
        self.tfb = TransformBroadcaster(self)
        self.create_subscription(TrackingMeasurement, topics.TRACKING, self.on_tracking, SENSOR_QOS)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_px4_odom, PX4_QOS)
        self.px4_offset = None     # ros_time - px4_time (seconds), running minimum
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.last_path_t = -1e9
        self.n_track = 0
        self.n_odom = 0
        self.last_track_t = -1e9
        self.last_odom_t = -1e9
        self.create_timer(0.2, self.health)

    # ------------------------------------------------------------ tracking -> EKF2
    def on_tracking(self, m: TrackingMeasurement):
        p_st = spherical_to_cartesian(m.range, m.azimuth, m.elevation)
        p_map = station_to_map(p_st, self.station)
        cov = spherical_covariance(m.range, m.azimuth, m.elevation, m.sigma_range, m.sigma_azimuth, m.sigma_elevation)
        var = np.maximum(np.diag(cov), self.var_floor)
        self.n_track += 1
        self.last_track_t = now_sec(self)

        ev = VehicleOdometry()
        t_sample_us = self._ros_to_px4_us(m.header.stamp)
        ev.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ev.timestamp_sample = t_sample_us if t_sample_us is not None else ev.timestamp
        ev.pose_frame = VehicleOdometry.POSE_FRAME_NED
        ev.position = enu_to_ned(p_map).astype(np.float32).tolist()
        ev.q = [float('nan')] * 4
        ev.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        ev.velocity = [float('nan')] * 3
        ev.angular_velocity = [float('nan')] * 3
        v_ned = [var[1], var[0], var[2]]
        ev.position_variance = [float(v) for v in v_ned]
        ev.orientation_variance = [float('nan')] * 3
        ev.velocity_variance = [float('nan')] * 3
        ev.reset_counter = 0
        ev.quality = 100
        self.pub_ev.publish(ev)

        tp = PoseWithCovarianceStamped()
        tp.header.stamp = m.header.stamp
        tp.header.frame_id = self.map_frame
        tp.pose.pose.position.x, tp.pose.pose.position.y, tp.pose.pose.position.z = map(float, p_map)
        tp.pose.pose.orientation.w = 1.0
        c = np.zeros((6, 6))
        c[:3, :3] = cov
        tp.pose.covariance = c.ravel().tolist()
        self.pub_track.publish(tp)

    def _ros_to_px4_us(self, stamp):
        if self.px4_offset is None:
            return None
        return int((stamp_to_sec(stamp) - self.px4_offset) * 1e6)

    # ------------------------------------------------------------ EKF2 -> /vehicle/odom
    def on_px4_odom(self, m: VehicleOdometry):
        now = now_sec(self)
        px4_t = m.timestamp_sample * 1e-6
        off = now - px4_t
        self.px4_offset = off if self.px4_offset is None else min(self.px4_offset + 1e-4, off)
        stamp_s = px4_t + self.px4_offset
        stamp = Time(seconds=stamp_s).to_msg()
        if not all(math.isfinite(v) for v in m.position):
            return
        p = ned_to_enu(m.position)
        q = quat_ned_frd_to_enu_flu(m.q) if all(math.isfinite(v) for v in m.q) else np.array([1.0, 0, 0, 0])
        v = ned_to_enu(m.velocity) if all(math.isfinite(x) for x in m.velocity) else np.zeros(3)
        # body angular rates FRD -> FLU
        w = np.array([m.angular_velocity[0], -m.angular_velocity[1], -m.angular_velocity[2]])
        self.n_odom += 1
        self.last_odom_t = now

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, p)
        o.pose.pose.orientation.w, o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z = map(float, q)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, v)  # map-frame velocity
        if all(math.isfinite(x) for x in w):
            o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = map(float, w)
        pv = m.position_variance
        cov = np.zeros((6, 6))
        if all(math.isfinite(x) for x in pv):
            cov[0, 0], cov[1, 1], cov[2, 2] = pv[1], pv[0], pv[2]
        o.pose.covariance = cov.ravel().tolist()
        self.pub_odom.publish(o)

        tf = TransformStamped()
        tf.header = o.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, p)
        tf.transform.rotation = o.pose.pose.orientation
        self.tfb.sendTransform(tf)

        if now - self.last_path_t > 0.5:
            self.last_path_t = now
            ps = PoseStamped()
            ps.header = o.header
            ps.header.frame_id = self.map_frame
            ps.pose = o.pose.pose
            self.path.poses.append(ps)
            self.path.poses = self.path.poses[-2000:]
            self.path.header.stamp = stamp
            self.pub_path.publish(self.path)

    def health(self):
        t = now_sec(self)
        self.pub_health.publish(health_msg('odom', t - self.last_odom_t < 0.5, t, rate_count=self.n_odom))
        self.pub_health.publish(health_msg('tracker', t - self.last_track_t < 0.5, t, rate_count=self.n_track))


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
