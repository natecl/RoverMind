from perception.depth_math import depth_to_distance_bucket, sample_depth_patch


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


def _grid(value_mm, width=10, height=10):
    """A height x width depth image where every pixel is value_mm."""
    return [[value_mm for _ in range(width)] for _ in range(height)]


def test_sample_depth_patch_converts_mm_to_metres():
    depth_mm = _grid(500)  # every pixel 500 mm
    samples = sample_depth_patch(depth_mm, 0.5, 0.5, patch_radius=1)
    assert samples == [0.5] * 9  # 3x3 patch, all 0.5 m


def test_sample_depth_patch_clamps_to_image_bounds():
    depth_mm = _grid(800)
    # Point at the top-left corner; the patch is clipped to in-bounds pixels.
    samples = sample_depth_patch(depth_mm, 0.0, 0.0, patch_radius=2)
    assert all(s == 0.8 for s in samples)
    assert len(samples) == 9  # 3x3 of the 5x5 patch lies inside the image
