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
