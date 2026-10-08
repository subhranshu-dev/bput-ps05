"""LiDAR -> camera projection (Phase 7).

    P_camera = T_camera_lidar * P_lidar
    u = fx * X / Z + cx
    v = fy * Y / Z + cy

Points are only coloured when Z > min_depth and the projected pixel lies inside
the image. All other points keep their geometry but are flagged in_fov = 0.
"""
import numpy as np


def intrinsics_from_camera_info(k_flat) -> np.ndarray:
    k = np.asarray(k_flat, dtype=float).reshape(3, 3)
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('invalid camera intrinsics (fx/fy <= 0)')
    return k


def transform_points(t44: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    return pts @ t44[:3, :3].T + t44[:3, 3]


def project(pts_cam: np.ndarray, k: np.ndarray, width: int, height: int, min_depth: float = 0.3):
    """Pinhole projection. Returns (u, v, valid) where valid = in front + inside image."""
    pts_cam = np.asarray(pts_cam, dtype=float).reshape(-1, 3)
    z = pts_cam[:, 2]
    in_front = z > min_depth
    zs = np.where(in_front, z, 1.0)
    u = k[0, 0] * pts_cam[:, 0] / zs + k[0, 2]
    v = k[1, 1] * pts_cam[:, 1] / zs + k[1, 2]
    valid = in_front & (u >= 0) & (u <= width - 1) & (v >= 0) & (v <= height - 1)
    return u, v, valid


def colorize(pts_lidar: np.ndarray, t_cam_lidar: np.ndarray, k: np.ndarray, image_rgb: np.ndarray,
             min_depth: float = 0.3):
    """Return (rgb uint8 Nx3, in_fov bool N, u, v). Points outside the FOV get grey (128)."""
    h, w = image_rgb.shape[:2]
    pts_cam = transform_points(t_cam_lidar, pts_lidar)
    u, v, valid = project(pts_cam, k, w, h, min_depth)
    rgb = np.full((len(pts_cam), 3), 128, dtype=np.uint8)
    if valid.any():
        ui = np.rint(u[valid]).astype(int)
        vi = np.rint(v[valid]).astype(int)
        rgb[valid] = image_rgb[vi, ui, :3]
    return rgb, valid, u, v


def pack_rgb(rgb: np.ndarray) -> np.ndarray:
    """Pack uint8 RGB into the float32 'rgb' field used by PCL/RViz."""
    rgb = np.asarray(rgb, dtype=np.uint32)
    packed = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    return packed.astype(np.uint32).view(np.float32)


def unpack_rgb(packed: np.ndarray) -> np.ndarray:
    p = np.asarray(packed, dtype=np.float32).view(np.uint32)
    return np.stack([(p >> 16) & 255, (p >> 8) & 255, p & 255], axis=1).astype(np.uint8)


def reprojection_shift(pts_lidar, t_true, t_used, k, width, height, min_depth=0.3):
    """Pixel displacement caused by using t_used instead of the true extrinsic t_true.

    Returns (mean_px, p95_px, n) over points valid under both transforms.
    """
    u0, v0, ok0 = project(transform_points(t_true, pts_lidar), k, width, height, min_depth)
    u1, v1, ok1 = project(transform_points(t_used, pts_lidar), k, width, height, min_depth)
    ok = ok0 & ok1
    if not ok.any():
        return float('nan'), float('nan'), 0
    d = np.hypot(u1[ok] - u0[ok], v1[ok] - v0[ok])
    return float(d.mean()), float(np.percentile(d, 95)), int(ok.sum())


def depth_discontinuity_mask(pts_cam: np.ndarray, ring_size: int, jump: float = 0.5) -> np.ndarray:
    """Mark LiDAR points at range discontinuities along each scan ring (object silhouettes)."""
    n = len(pts_cam)
    rng = np.linalg.norm(pts_cam, axis=1)
    mask = np.zeros(n, dtype=bool)
    if ring_size <= 1 or n < ring_size:
        d = np.abs(np.diff(rng))
        mask[1:] |= d > jump
        return mask
    rows = n // ring_size
    r = rng[: rows * ring_size].reshape(rows, ring_size)
    d = np.abs(np.diff(r, axis=1)) > jump
    m = np.zeros_like(r, dtype=bool)
    m[:, 1:] |= d
    m[:, :-1] |= d
    mask[: rows * ring_size] = m.ravel()
    return mask


def edge_chamfer_px(edge_image: np.ndarray, u: np.ndarray, v: np.ndarray, max_px: float = 20.0):
    """Mean distance (px) from projected LiDAR silhouette points to the nearest image edge.

    This is the image-based *fusion projection error* metric: with a correct
    extrinsic, LiDAR depth discontinuities land on visual edges.
    """
    import cv2  # local import keeps module importable without OpenCV
    if len(u) == 0:
        return float('nan'), 0
    inv = np.where(edge_image > 0, 0, 255).astype(np.uint8)
    dist = cv2.distanceTransform(inv, cv2.DIST_L2, 3)
    ui = np.clip(np.rint(u).astype(int), 0, dist.shape[1] - 1)
    vi = np.clip(np.rint(v).astype(int), 0, dist.shape[0] - 1)
    d = np.minimum(dist[vi, ui], max_px)
    return float(d.mean()), int(len(d))
