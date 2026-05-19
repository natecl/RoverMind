import math
from dataclasses import dataclass
from typing import Callable, Tuple


@dataclass(frozen=True)
class ControllerParams:
    max_linear: float = 0.3
    max_angular: float = 0.5
    heading_tolerance_rad: float = 0.035
    heading_kp: float = 1.0
    loop_rate_hz: float = 20.0
    timeout_safety_margin_s: float = 2.0
    min_timeout_s: float = 1.0
    max_timeout_s: float = 30.0


def wrap_angle(rad: float) -> float:
    """Wrap an angle in radians to the range [-pi, pi)."""
    return ((rad + math.pi) % (2 * math.pi)) - math.pi


def proportional_turn(error_rad: float, params: ControllerParams) -> float:
    """P-controller output for heading, clamped to +/- max_angular."""
    raw = params.heading_kp * error_rad
    return max(-params.max_angular, min(params.max_angular, raw))
