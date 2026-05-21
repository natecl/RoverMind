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


def gate_twist(
    linear_x: float, angular_z: float, braking: bool
) -> Tuple[float, float]:
    """Zero forward linear velocity while braking; pass rotation and reverse."""
    if braking and linear_x > 0.0:
        return (0.0, angular_z)
    return (linear_x, angular_z)


class BrakeStateMachine:
    """Hysteresis state machine: maps forward obstacle distance to a brake flag.

    Braking trips when the nearest obstacle is closer than trigger_distance_m.
    It releases only after the path stays clear beyond release_distance_m for
    release_dwell_s continuously -- the gap between the two distances is a dead
    band that prevents on/off chatter.
    """

    def __init__(self, params: AebParams) -> None:
        self.params = params
        self.braking = False
        self._clear_since: Optional[float] = None

    def update(self, min_range: float, now: float) -> bool:
        params = self.params
        if min_range < params.trigger_distance_m:
            self.braking = True
            self._clear_since = None
        elif self.braking:
            if min_range > params.release_distance_m:
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since >= params.release_dwell_s - 1e-9:
                    self.braking = False
                    self._clear_since = None
            else:
                # dead band: trigger_distance_m <= min_range <= release_distance_m
                self._clear_since = None
        return self.braking
