"""Fast PointCloud2 <-> numpy conversion (vectorised; no per-point Python loops)."""
import numpy as np

_DT = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def cloud_fields(msg) -> dict:
    return {f.name: (f.offset, _DT[f.datatype]) for f in msg.fields}


def cloud_to_array(msg, names=('x', 'y', 'z')) -> np.ndarray:
    """Return an (N, len(names)) float array; non-finite points are kept (caller filters)."""
    fields = cloud_fields(msg)
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, len(names)))
    dtype = np.dtype({'names': list(names), 'formats': [fields[k][1] for k in names],
                      'offsets': [fields[k][0] for k in names], 'itemsize': msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
    return np.stack([raw[k].astype(np.float64) for k in names], axis=1)


def xyz_from_cloud(msg) -> np.ndarray:
    return cloud_to_array(msg, ('x', 'y', 'z'))


def make_cloud(header, xyz: np.ndarray, rgb_packed: np.ndarray | None = None, extra: dict | None = None):
    """Build a PointCloud2 with float32 x,y,z [+ rgb float32] [+ extra float32 fields]."""
    from sensor_msgs.msg import PointCloud2, PointField
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    names = ['x', 'y', 'z']
    cols = [xyz[:, 0], xyz[:, 1], xyz[:, 2]]
    if rgb_packed is not None:
        names.append('rgb')
        cols.append(np.asarray(rgb_packed, dtype=np.float32))
    for k, v in (extra or {}).items():
        names.append(k)
        cols.append(np.asarray(v, dtype=np.float32))
    arr = np.empty(len(xyz), dtype=np.dtype({'names': names, 'formats': [np.float32] * len(names)}))
    for k, c in zip(names, cols):
        arr[k] = c
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, k in enumerate(names)]
    msg.is_bigendian = False
    msg.point_step = 4 * len(names)
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = arr.tobytes()
    return msg


def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    """Keep the first point in each voxel (order preserving). Returns indices."""
    if voxel <= 0 or len(pts) == 0:
        return np.arange(len(pts))
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return np.sort(idx)
