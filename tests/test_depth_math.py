from perception.depth_math import depth_to_distance_bucket


def test_close_reading():
    bucket, metres = depth_to_distance_bucket([0.4, 0.42, 0.41])
    assert bucket == "close"
    assert metres == 0.41


def test_medium_reading():
    bucket, metres = depth_to_distance_bucket([1.0, 1.1, 0.9])
    assert bucket == "medium"
    assert metres == 1.0


def test_far_reading():
    bucket, metres = depth_to_distance_bucket([3.0, 3.0, 3.0])
    assert bucket == "far"
    assert metres == 3.0


def test_all_invalid_returns_none():
    bucket, metres = depth_to_distance_bucket([0.0, 0.0, 0.0])
    assert bucket is None
    assert metres is None


def test_median_ignores_invalid_zeros_and_outliers():
    # 0.0 values are invalid and dropped; median of [0.5, 0.5, 0.5] is 0.5.
    bucket, metres = depth_to_distance_bucket([0.0, 0.5, 0.0, 0.5, 0.5])
    assert bucket == "close"
    assert metres == 0.5
