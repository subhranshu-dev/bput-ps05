"""Camera processing (Phase 3): validation, health heartbeat and test-harness fault injection.

Republishes forward and downward camera streams on /perception/... after
checking encoding, size, timestamps and CameraInfo. Fault injection (blur,
exposure, noise, timestamp offset, occlusion, blackout) is configured in
configs/camera.yaml (or a scenario override) and is OFF by default.
"""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from ps05_common import config, topics
from ps05_common.ros_utils import RELIABLE_QOS, SENSOR_QOS, RateMeter, health_msg, now_sec, stamp_to_sec


def apply_faults(img: np.ndarray, f: dict, rng, t_rel: float, which: str) -> tuple:
    """Return (image, blacked_out). Pure function (unit-tested)."""
    import cv2
    out = img
    bo = f.get('blackout', {}) or {}
    if bo.get('enabled') and bo.get('camera', 'forward') == which and bo['start_s'] <= t_rel < bo['start_s'] + bo['duration_s']:
        return None, True
    k = int(f.get('blur_kernel', 0) or 0)
    if k > 1:
        k = k + 1 if k % 2 == 0 else k
        out = cv2.GaussianBlur(out, (k, k), 0)
    g = float(f.get('exposure_gain', 1.0))
    if abs(g - 1.0) > 1e-6:
        out = np.clip(out.astype(np.float32) * g, 0, 255).astype(np.uint8)
    s = float(f.get('noise_sigma', 0.0))
    if s > 0:
        out = np.clip(out.astype(np.float32) + rng.normal(0, s, out.shape), 0, 255).astype(np.uint8)
    oc = f.get('occlusion', {}) or {}
    if oc.get('enabled') and oc.get('camera', 'down') == which and oc['start_s'] <= t_rel < oc['start_s'] + oc['duration_s']:
        out = out.copy()
        h, w = out.shape[:2]
        fr = float(oc.get('fraction', 0.6))
        y0, y1 = int(h * (1 - fr) / 2), int(h * (1 + fr) / 2)
        x0, x1 = int(w * (1 - fr) / 2), int(w * (1 + fr) / 2)
        out[y0:y1, x0:x1] = 90
    return out, False


class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.cfg = config.load('camera')
        self.faults = self.cfg.get('fault_injection', {}) or {}
        seed = int(config.load_scenario().get('seed', 0))
        self.rng = np.random.default_rng(seed + 2)
        self.bridge = CvBridge()
        self.pub_health = self.create_publisher(String, topics.SENSOR_HEALTH, RELIABLE_QOS)
        self.streams = {}
        self.t_start = None
        self.fault_trigger = self.faults.get('trigger_state')   # optional: start fault clock at mission state
        self.trigger_t = None
        self.create_subscription(String, topics.MISSION_STATUS, self.on_mission, RELIABLE_QOS)
        for which in ('forward', 'down'):
            c = self.cfg[which]
            st = {'which': which, 'rate': RateMeter(), 'info': None, 'info_ok': False, 'last_stamp': None,
                  'nonmono': 0, 'expected': float(c['expected_rate_hz']), 'blackout': False,
                  'pub_img': self.create_publisher(Image, c['output_image_topic'], SENSOR_QOS),
                  'pub_info': self.create_publisher(CameraInfo, c['output_info_topic'], SENSOR_QOS)}
            self.streams[which] = st
            self.create_subscription(Image, c['image_topic'], lambda m, s=st: self.on_image(m, s), SENSOR_QOS)
            self.create_subscription(CameraInfo, c['info_topic'], lambda m, s=st: self.on_info(m, s), SENSOR_QOS)
        self.create_timer(0.2, self.health)

    def on_mission(self, msg):
        import json
        if self.fault_trigger and self.trigger_t is None:
            if json.loads(msg.data).get('state') == self.fault_trigger:
                self.trigger_t = now_sec(self)
                self.get_logger().warn(f'fault-injection clock started at state {self.fault_trigger}')

    def on_info(self, m: CameraInfo, st):
        k = np.array(m.k).reshape(3, 3)
        st['info_ok'] = bool(m.width > 0 and m.height > 0 and k[0, 0] > 0 and k[1, 1] > 0
                             and 0 < k[0, 2] < m.width and 0 < k[1, 2] < m.height)
        st['info'] = m

    def on_image(self, m: Image, st):
        t = now_sec(self)
        if self.t_start is None:
            self.t_start = t
        ts = stamp_to_sec(m.header.stamp)
        if st['last_stamp'] is not None and ts <= st['last_stamp']:
            st['nonmono'] += 1
        st['last_stamp'] = ts
        clock0 = self.trigger_t if self.fault_trigger else self.t_start
        t_rel = (t - clock0) if clock0 is not None else -1e9
        img = self.bridge.imgmsg_to_cv2(m, desired_encoding='rgb8')
        img, blacked = apply_faults(img, self.faults, self.rng, t_rel, st['which'])
        st['blackout'] = blacked
        if blacked:
            return   # simulated camera failure: no output at all
        st['rate'].tick(t)
        out = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        out.header = m.header
        off = float(self.faults.get('stamp_offset_s', 0.0)) if st['which'] == 'forward' else 0.0
        if off:
            out.header.stamp = Time(seconds=ts + off).to_msg()
        st['pub_img'].publish(out)
        if st['info'] is not None:
            info = st['info']
            info.header.stamp = out.header.stamp
            info.header.frame_id = m.header.frame_id or info.header.frame_id
            st['pub_info'].publish(info)

    def health(self):
        t = now_sec(self)
        timeout = 1.0
        for which, st in self.streams.items():
            name = 'camera' if which == 'forward' else 'down_camera'
            age = st['rate'].age(t)
            ok = age < timeout and st['info_ok']
            info = st['info']
            self.pub_health.publish(health_msg(
                name, ok, t, rate_hz=round(st['rate'].rate, 2), expected_hz=st['expected'],
                age_s=None if age == float('inf') else round(age, 3), info_ok=st['info_ok'],
                width=info.width if info else None, height=info.height if info else None,
                fx=float(info.k[0]) if info else None, nonmonotonic=st['nonmono'], blackout=st['blackout']))


def main():
    rclpy.init()
    node = CameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
