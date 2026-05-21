"""Pure decision logic for the autonomous emergency braking (AEB) layer.

Zero ROS imports -- fully unit-testable on the dev laptop. The ROS wiring lives
in aeb_node.py.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from safety_controller_layer.control_math import wrap_angle


@dataclass(frozen=True)
class AebParams:
    """Tunable parameters for the emergency braking layer."""

    trigger_distance_m: float = 0.40
    release_distance_m: float = 0.60
    release_dwell_s: float = 0.5
    forward_arc_deg: float = 60.0
    output_rate_hz: float = 20.0
    command_timeout_s: float = 0.5
    scan_timeout_s: float = 1.0

    def __post_init__(self) -> None:
        if self.release_distance_m <= self.trigger_distance_m:
            raise ValueError(
                f"release_distance_m ({self.release_distance_m}) must be greater "
                f"than trigger_distance_m ({self.trigger_distance_m})"
            )
