"""Frame-convention conversions at the PX4 boundary.

This is the ONLY module in the project that converts between the ROS map frame
(ENU, body FLU) and the PX4 local frame (NED, body FRD). All sensor-to-sensor
and sensor-to-map transforms go through TF2; ENU<->NED is a fixed convention
change between ROS (REP-103) and PX4, not a sensor extrinsic, so it lives here.
"""
import math

import numpy as np


def enu_to_ned(v):
    """Position/velocity vector ENU -> NED."""
    x, y, z = v
    return np.array([y, x, -z], dtype=float)


def ned_to_enu(v):
    x, y, z = v
    return np.array([y, x, -z], dtype=float)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """ENU yaw (from +X/East, CCW) -> NED yaw (from North, CW)."""
    return wrap_pi(math.pi / 2.0 - yaw_enu)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    return wrap_pi(math.pi / 2.0 - yaw_ned)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_mul(q1, q2):
    """Hamilton product, quaternions as (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


# q_ENU<-NED : rotation taking NED-frame vectors into ENU
_Q_NED_TO_ENU = np.array([0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0])
# q_FRD<-FLU (body): 180 deg about X
_Q_FLU_TO_FRD = np.array([0.0, 1.0, 0.0, 0.0])


def quat_ned_frd_to_enu_flu(q_ned_frd):
    """PX4 attitude (body FRD in NED, wxyz) -> ROS attitude (body FLU in ENU, wxyz)."""
    q = quat_mul(quat_mul(_Q_NED_TO_ENU, np.asarray(q_ned_frd, dtype=float)), _Q_FLU_TO_FRD)
    return q / np.linalg.norm(q)


def quat_enu_flu_to_ned_frd(q_enu_flu):
    q_enu_to_ned = _Q_NED_TO_ENU * np.array([1, -1, -1, -1])  # conjugate
    q_frd_to_flu = _Q_FLU_TO_FRD * np.array([1, -1, -1, -1])
    q = quat_mul(quat_mul(q_enu_to_ned, np.asarray(q_enu_flu, dtype=float)), q_frd_to_flu)
    return q / np.linalg.norm(q)


def yaw_from_quat(q_wxyz) -> float:
    w, x, y, z = q_wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def quat_to_matrix(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_quat(r):
    """3x3 rotation -> (w, x, y, z)."""
    r = np.asarray(r, dtype=float)
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def transform_matrix(translation, q_wxyz):
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(q_wxyz)
    t[:3, 3] = translation
    return t


def tf_to_matrix(tf_msg_transform):
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    tr = tf_msg_transform.translation
    ro = tf_msg_transform.rotation
    return transform_matrix([tr.x, tr.y, tr.z], [ro.w, ro.x, ro.y, ro.z])
