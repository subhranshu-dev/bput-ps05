"""Read-only ROS -> web backend telemetry forwarder (Phase 22).

Subscribes to status topics, images, clouds and paths, and POSTs compact JSON
snapshots to the FastAPI backend (configs/dashboard.yaml). It never publishes
to ROS, so the autonomy stack runs identically with or without the dashboard
(Rule 12). Posting is fire-and-forget in a worker thread; if the backend is
down, snapshots are dropped.
"""
import base64
import json
import queue
import threading
import urllib.request

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

from ps05_common import config, topics
from ps05_common.pc2 import cloud_fields, cloud_to_array
from ps05_common.ros_utils import LATCHED_QOS, RELIABLE_QOS, SENSOR_QOS, lookup_matrix, now_sec

from tf2_ros import Buffer, TransformListener

JSON_TOPICS = {
    'mission': topics.MISSION_STATUS, 'px4': topics.PX4_STATE, 'planner': topics.PLANNER_STATUS,
    'fusion': topics.FUSION_STATUS, 'mapping': topics.MAP_STATUS, 'avoidance': topics.AVOIDANCE_STATUS,
    'detection': topics.LANDING_DETECTION, 'landing_safety': topics.LANDING_SAFETY, 'metrics': topics.EVAL_METRICS,
}


def path_to_list(m: Path, step=1):
    return [[round(p.pose.position.x, 2), round(p.pose.position.y, 2), round(p.pose.position.z, 2)]
            for p in m.poses[::step]]


class DashboardBridge(Node):
    def __init__(self):
        super().__init__('dashboard_bridge')
        c = config.load('dashboard')['bridge']
        self.url = c['url']
        self.img_w = int(c['image_width'])
        self.q = int(c['jpeg_quality'])
        self.max_pts = int(c['max_cloud_points'])
        self.max_vox = int(c['max_map_voxels'])
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state = {k: None for k in JSON_TOPICS}
        self.health = {}
        self.events = []
        self.paths = {}
        self.truth = None
        self.images = {}
        self.clouds = {}
        self.rng = np.random.default_rng(0)
        for key, topic in JSON_TOPICS.items():
            qos = LATCHED_QOS if key == 'planner' else RELIABLE_QOS
            self.create_subscription(String, topic, lambda m, k=key: self.on_json(k, m), qos)
        self.create_subscription(String, topics.SENSOR_HEALTH, self.on_health, RELIABLE_QOS)
        self.create_subscription(String, topics.MISSION_EVENTS, lambda m: self.on_event('mission', m), RELIABLE_QOS)
        self.create_subscription(String, topics.SCENARIO_EVENTS, lambda m: self.on_event('scenario', m), RELIABLE_QOS)
        for key, topic, qos in (('planned', topics.PLANNED_PATH, LATCHED_QOS), ('local', topics.LOCAL_TRAJECTORY, LATCHED_QOS),
                                ('actual', topics.ACTUAL_PATH, RELIABLE_QOS)):
            self.create_subscription(Path, topic, lambda m, k=key: self.on_path(k, m), qos)
        self.create_subscription(Odometry, topics.TRUTH_ODOM, self.on_truth, SENSOR_QOS)
        self.create_subscription(Image, '/perception/camera/image', lambda m: self.on_image('camera', m), SENSOR_QOS)
        self.create_subscription(Image, '/perception/down_camera/image', lambda m: self.on_image('down_camera', m), SENSOR_QOS)
        self.create_subscription(PointCloud2, topics.FUSED_POINTS, self.on_fused, SENSOR_QOS)
        self.create_subscription(PointCloud2, topics.OCCUPANCY_MAP, self.on_map, RELIABLE_QOS)
        self.outq = queue.Queue(maxsize=4)
        threading.Thread(target=self.sender, daemon=True).start()
        self.create_timer(1.0 / float(c['rate_hz']), self.send_snapshot)
        self.create_timer(0.4, self.send_images)
        self.create_timer(1.0, self.send_clouds)
        self.ok = 0
        self.fail = 0
        self.fail_streak = 0
        self.last_probe = 0.0

    # ------------------------------------------------------------------ inputs
    def on_json(self, k, m):
        try:
            self.state[k] = json.loads(m.data)
        except json.JSONDecodeError:
            pass

    def on_health(self, m):
        h = json.loads(m.data)
        self.health[h['sensor']] = h

    def on_event(self, src, m):
        ev = json.loads(m.data)
        ev['source'] = src
        self.events = (self.events + [ev])[-100:]

    def on_path(self, k, m):
        self.paths[k] = path_to_list(m, 1 if k != 'actual' else 2)

    def on_truth(self, m):
        p = m.pose.pose.position
        self.truth = [p.x, p.y, p.z]

    def on_image(self, k, m):
        self.images[k] = m

    def on_fused(self, m):
        self.clouds['fused_msg'] = m

    def on_map(self, m):
        self.clouds['map_msg'] = m

    # ------------------------------------------------------------------ output
    def backend_down(self) -> bool:
        """After 5 consecutive failures only probe every 5 s (saves CPU when no dashboard is running)."""
        import time as _t
        if self.fail_streak < 5:
            return False
        if _t.monotonic() - self.last_probe > 5.0:
            self.last_probe = _t.monotonic()
            return False
        return True

    def post(self, kind, data):
        try:
            self.outq.put_nowait({'kind': kind, 'data': data})
        except queue.Full:
            pass

    def sender(self):
        while True:
            item = self.outq.get()
            try:
                req = urllib.request.Request(self.url, data=json.dumps(item, default=float).encode(),
                                             headers={'Content-Type': 'application/json'})
                urllib.request.urlopen(req, timeout=1.0).read()
                self.ok += 1
                self.fail_streak = 0
            except Exception:  # noqa: BLE001  backend absent/down: drop
                self.fail += 1
                self.fail_streak += 1

    def send_snapshot(self):
        if self.backend_down() and 'send_snapshot' != 'send_snapshot':
            return
        mission = self.state['mission'] or {}
        snap = {k: v for k, v in self.state.items() if k != 'mission'}
        mission = {k: v for k, v in mission.items() if k != 'events'}
        snap.update({'mission': mission, 'sensors': self.health, 'events': self.events[-40:], 'paths': self.paths,
                     'truth_position': self.truth, 'ros_time': now_sec(self), 'gps': 'DISABLED'})
        self.post('snapshot', snap)

    def send_images(self):
        if self.backend_down() and 'send_images' != 'send_snapshot':
            return
        out = {}
        for k, m in list(self.images.items()):
            img = self.bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')
            h, w = img.shape[:2]
            img = cv2.resize(img, (self.img_w, int(h * self.img_w / w)))
            if k == 'down_camera':
                det = self.state.get('detection') or {}
                if det.get('detected') and det.get('center_px'):
                    s = self.img_w / w
                    cx, cy = det['center_px']
                    cv2.circle(img, (int(cx * s), int(cy * s)), 14, (0, 255, 120), 2)
                    cv2.line(img, (self.img_w // 2, img.shape[0] // 2), (int(cx * s), int(cy * s)), (0, 200, 255), 1)
            ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.q])
            if ok:
                out[k] = base64.b64encode(buf.tobytes()).decode()
        if out:
            self.post('images', out)

    def _sample(self, arr, n):
        if len(arr) > n:
            arr = arr[self.rng.choice(len(arr), n, replace=False)]
        return arr

    def send_clouds(self):
        if self.backend_down() and 'send_clouds' != 'send_snapshot':
            return
        data = {}
        m = self.clouds.get('fused_msg')
        if m is not None:
            tmat = lookup_matrix(self.tf_buffer, 'map', m.header.frame_id, m.header.stamp, 0.05)
            if tmat is not None and 'rgb' in cloud_fields(m):
                a = cloud_to_array(m, ('x', 'y', 'z', 'rgb', 'in_fov'))
                a = self._sample(a, self.max_pts)
                xyz = a[:, :3] @ tmat[:3, :3].T + tmat[:3, 3]
                rgb = a[:, 3].astype(np.float32).view(np.uint32)
                data['fused'] = {'xyz': np.round(xyz, 2).ravel().tolist(), 'rgb': rgb.tolist(),
                                 'in_fov': a[:, 4].astype(int).tolist()}
        m = self.clouds.get('map_msg')
        if m is not None and m.width > 0:
            a = cloud_to_array(m, ('x', 'y', 'z', 'rgb'))
            a = self._sample(a, self.max_vox)
            data['map'] = {'xyz': np.round(a[:, :3], 2).ravel().tolist(),
                           'rgb': a[:, 3].astype(np.float32).view(np.uint32).tolist(), 'total': int(m.width)}
        if data:
            self.post('clouds', data)


def main():
    rclpy.init()
    node = DashboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
