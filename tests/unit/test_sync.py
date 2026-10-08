import pytest

from sensor_fusion.sync import TimestampSynchronizer


def test_perfect_sync():
    s = TimestampSynchronizer(0.05)
    for i in range(10):
        s.add_camera(i * 0.1, f'img{i}')
    m, d = s.match_lidar(0.5)
    assert m == 'img5' and d == pytest.approx(0.0)


def test_small_offset_accepted():
    s = TimestampSynchronizer(0.05)
    for i in range(10):
        s.add_camera(i * 0.1 + 0.017, f'img{i}')
    m, d = s.match_lidar(0.5)
    assert m == 'img5' and d == pytest.approx(0.017)
    assert s.stats()['accepted'] == 1


def test_large_offset_rejected():
    s = TimestampSynchronizer(0.05)
    for i in range(10):
        s.add_camera(i * 0.1 + 0.2, f'img{i}')
    m, d = s.match_lidar(0.05)
    assert m is None and abs(d) > 0.05
    assert s.stats()['rejected'] == 1


def test_out_of_order_frames():
    s = TimestampSynchronizer(0.05)
    s.add_camera(0.3, 'c')
    s.add_camera(0.1, 'a')
    s.add_camera(0.2, 'b')
    assert s.match_lidar(0.11)[0] == 'a'


def test_invalid_max_delta():
    with pytest.raises(ValueError):
        TimestampSynchronizer(0.0)
