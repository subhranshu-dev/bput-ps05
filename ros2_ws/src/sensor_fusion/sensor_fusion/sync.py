"""Explicit, measurable camera/LiDAR timestamp synchronisation (Phase 6).

Each LiDAR scan is paired with the camera frame closest in time. A pair is
accepted only if |t_cam - t_lidar| <= max_delta. Every decision is counted so
the synchronisation quality is observable (/fusion/status).
"""
from collections import deque


class TimestampSynchronizer:
    def __init__(self, max_delta: float, queue_size: int = 10):
        if max_delta <= 0:
            raise ValueError('max_delta must be > 0')
        self.max_delta = max_delta
        self.cam = deque(maxlen=queue_size)
        self.accepted = 0
        self.rejected = 0
        self.deltas = deque(maxlen=200)
        self.last_delta = None

    def add_camera(self, t: float, msg):
        if self.cam and t < self.cam[-1][0]:
            # out-of-order frame: keep queue sorted
            items = sorted(list(self.cam) + [(t, msg)], key=lambda x: x[0])
            self.cam.clear()
            self.cam.extend(items)
        else:
            self.cam.append((t, msg))

    def match_lidar(self, t_lidar: float):
        """Return (camera_msg, delta) or (None, delta_or_None)."""
        if not self.cam:
            self.rejected += 1
            return None, None
        best = min(self.cam, key=lambda c: abs(c[0] - t_lidar))
        delta = best[0] - t_lidar
        self.last_delta = delta
        if abs(delta) <= self.max_delta:
            self.accepted += 1
            self.deltas.append(abs(delta))
            return best[1], delta
        self.rejected += 1
        return None, delta

    def stats(self) -> dict:
        d = list(self.deltas)
        return {
            'accepted': self.accepted,
            'rejected': self.rejected,
            'last_delta_s': self.last_delta,
            'mean_abs_delta_s': (sum(d) / len(d)) if d else None,
            'max_abs_delta_s': max(d) if d else None,
            'max_delta_allowed_s': self.max_delta,
        }
