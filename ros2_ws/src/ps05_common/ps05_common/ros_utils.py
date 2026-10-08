"""Small rclpy helpers shared by nodes (imported lazily so pure modules stay ROS-free)."""
import json
import time

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
# PX4 uXRCE-DDS topics use best-effort / transient-local / keep-last
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)


def json_msg(payload: dict):
    from std_msgs.msg import String
    m = String()
    m.data = json.dumps(payload, default=_default)
    return m


def _default(o):
    try:
        import numpy as np
        if isinstance(o, np.generic):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
    except ImportError:
        pass
    return str(o)


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def now_sec(node) -> float:
    return node.get_clock().now().nanoseconds * 1e-9


class RateMeter:
    """Exponentially-weighted message rate + last-seen time for watchdogs."""

    def __init__(self, alpha: float = 0.2):
        self.alpha = alpha
        self.rate = 0.0
        self.last = None
        self.count = 0

    def tick(self, t: float):
        if self.last is not None and t > self.last:
            inst = 1.0 / (t - self.last)
            self.rate = inst if self.count < 2 else (1 - self.alpha) * self.rate + self.alpha * inst
        self.last = t
        self.count += 1

    def age(self, t: float) -> float:
        return float('inf') if self.last is None else t - self.last


class Stopwatch:
    def __init__(self):
        self.t0 = time.perf_counter()

    def ms(self) -> float:
        return (time.perf_counter() - self.t0) * 1000.0


def lookup_matrix(tf_buffer, target: str, source: str, stamp=None, timeout_s: float = 0.05):
    """TF2 lookup -> 4x4 matrix (target <- source). Returns None if unavailable.

    Falls back to the latest transform if the exact stamp is not (yet) available
    and the latest one is within 0.2 s (static sensor mounts are time-invariant).
    """
    import rclpy.time
    from rclpy.duration import Duration

    from .frames import tf_to_matrix
    try:
        t = rclpy.time.Time() if stamp is None else rclpy.time.Time.from_msg(stamp)
        tf = tf_buffer.lookup_transform(target, source, t, timeout=Duration(seconds=timeout_s))
        return tf_to_matrix(tf.transform)
    except Exception:  # noqa: BLE001  (tf2 raises several exception types)
        if stamp is None:
            return None
        try:
            tf = tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            age = abs(stamp_to_sec(stamp) - stamp_to_sec(tf.header.stamp))
            return tf_to_matrix(tf.transform) if age < 0.2 else None
        except Exception:  # noqa: BLE001
            return None


def health_msg(sensor: str, ok: bool, t: float, **extra):
    return json_msg({'sensor': sensor, 'ok': bool(ok), 'stamp': t, **extra})
