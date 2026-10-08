import numpy as np

from mapping.voxel_map import FREE, OCCUPIED, UNKNOWN, VoxelMap, decode_states, encode_states


def make():
    return VoxelMap(0.25, [0, 10], [0, 10], [0, 5])


def test_initially_unknown():
    m = make()
    assert m.state_at([5, 5, 2]) == UNKNOWN
    assert m.counts()['occupied'] == 0


def test_hit_marks_occupied_and_ray_marks_free():
    m = make()
    origin = np.array([1.0, 5.0, 2.0])
    wall = np.array([[8.0, 5.0, 2.0]])
    for _ in range(2):
        m.integrate(wall, origin)
    assert m.state_at([8.0, 5.0, 2.0]) == OCCUPIED
    for x in (2.0, 4.0, 6.0, 7.5):
        assert m.state_at([x, 5.0, 2.0]) == FREE, x
    assert m.state_at([9.5, 5.0, 2.0]) == UNKNOWN       # behind the wall: not observed
    assert m.state_at([5.0, 8.0, 2.0]) == UNKNOWN       # off-ray


def test_moving_obstacle_is_cleared():
    m = make()
    o = np.array([1.0, 5.0, 2.0])
    for _ in range(2):
        m.integrate(np.array([[5.0, 5.0, 2.0]]), o)
    assert m.state_at([5.0, 5.0, 2.0]) == OCCUPIED
    for _ in range(8):   # obstacle gone, now see the far wall through that space
        m.integrate(np.array([[9.0, 5.0, 2.0]]), o)
    assert m.state_at([5.0, 5.0, 2.0]) == FREE


def test_out_of_bounds_and_nan_ignored():
    m = make()
    m.integrate(np.array([[50.0, 5, 2], [np.nan, 1, 1]]), np.array([1.0, 5, 2]), max_range=30)
    assert m.counts()['occupied'] == 0


def test_max_range_no_hit():
    m = make()
    for _ in range(2):   # FREE needs two consistent misses (miss -0.4, free threshold -0.5)
        m.integrate(np.array([[9.5, 5.0, 2.0]]), np.array([0.5, 5.0, 2.0]), max_range=4.0)
    assert m.state_at([9.5, 5, 2]) == UNKNOWN
    assert m.state_at([3.0, 5, 2]) == FREE


def test_encode_decode_roundtrip():
    m = make()
    m.integrate(np.array([[8.0, 5.0, 2.0]]), np.array([1.0, 5.0, 2.0]))
    s = m.states()
    nx, ny, nz = s.shape
    assert np.array_equal(decode_states(encode_states(s), nx, ny, nz), s)
