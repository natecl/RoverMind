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
