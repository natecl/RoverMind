"""Pure logic: depth-camera samples -> metric distance buckets.

Zero ROS and zero ML imports — fully unit-testable on the dev laptop.
"""

import math
from typing import Optional, Tuple

from perception.scene_parsing import Distance

# Bucket thresholds in metres. `close` extends a little past the controller's
# stop_distance (0.4 m) so "close" trips before the rover reaches the target.
CLOSE_MAX_M = 0.6
MEDIUM_MAX_M = 1.5


def depth_to_distance_bucket(
    depth_values_m,
    close_max_m: float = CLOSE_MAX_M,
    medium_max_m: float = MEDIUM_MAX_M,
) -> Tuple[Optional[Distance], Optional[float]]:
    """Reduce depth samples (in metres) to a (bucket, median_distance) pair.

    A sample is valid when finite and > 0 (a 0 reading is a depth-camera hole).
    Returns (None, None) when no sample is valid. Otherwise returns the bucket
    for the median valid sample and that median distance.
    """
    valid = sorted(
        d for d in depth_values_m
        if d is not None and math.isfinite(d) and d > 0.0
    )
    if not valid:
        return (None, None)
    n = len(valid)
    if n % 2 == 1:
        median = valid[n // 2]
    else:
        median = (valid[n // 2 - 1] + valid[n // 2]) / 2.0
    if median <= close_max_m:
        return ("close", median)
    if median <= medium_max_m:
        return ("medium", median)
    return ("far", median)


def sample_depth_patch(depth_mm, x_norm: float, y_norm: float,
                       patch_radius: int = 2):
    """Sample a square patch of a depth image around a normalised point.

    `depth_mm` is a height x width grid (list-of-lists or ndarray) of raw depth
    in millimetres. `x_norm`/`y_norm` are in [0, 1]. Returns a flat list of the
    in-bounds patch values converted to metres (0 mm holes are kept as 0.0 and
    filtered later by depth_to_distance_bucket).
    """
    height = len(depth_mm)
    width = len(depth_mm[0])
    cx = min(width - 1, max(0, round(x_norm * (width - 1))))
    cy = min(height - 1, max(0, round(y_norm * (height - 1))))
    samples = []
    for yy in range(cy - patch_radius, cy + patch_radius + 1):
        for xx in range(cx - patch_radius, cx + patch_radius + 1):
            if 0 <= yy < height and 0 <= xx < width:
                samples.append(depth_mm[yy][xx] / 1000.0)
    return samples
