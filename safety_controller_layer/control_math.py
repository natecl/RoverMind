import math
from dataclasses import dataclass
from typing import Callable, Optional, Tuple


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


class ControllerTimeoutError(Exception):
    """Raised when a control phase exceeds its computed timeout budget."""


class ControllerCancelledError(Exception):
    """Raised when a control phase is aborted via the should_abort predicate."""


def rotate_timeout_seconds(heading_delta_rad: float, params: ControllerParams) -> float:
    nominal = abs(heading_delta_rad) / params.max_angular if params.max_angular > 0 else params.max_timeout_s
    budget = nominal + params.timeout_safety_margin_s
    return max(params.min_timeout_s, min(params.max_timeout_s, budget))


def drive_timeout_seconds(distance_m: float, params: ControllerParams) -> float:
    nominal = abs(distance_m) / params.max_linear if params.max_linear > 0 else params.max_timeout_s
    budget = nominal + params.timeout_safety_margin_s
    return max(params.min_timeout_s, min(params.max_timeout_s, budget))


class SafetyController:
    def __init__(
        self,
        params: ControllerParams,
        get_yaw: Callable[[], float],
        get_position: Callable[[], Tuple[float, float]],
        publish_twist: Callable[[float, float], None],
        sleep: Callable[[float], None],
        now: Callable[[], float],
        should_abort: Optional[Callable[[], bool]] = None,
    ):
        self.params = params
        self._get_yaw = get_yaw
        self._get_position = get_position
        self._publish = publish_twist
        self._sleep = sleep
        self._now = now
        self._should_abort = should_abort if should_abort is not None else (lambda: False)

    def rotate_to_heading(self, heading_delta_rad: float) -> None:
        params = self.params
        period = 1.0 / params.loop_rate_hz
        start_yaw = self._get_yaw()
        target_yaw = start_yaw + heading_delta_rad
        deadline = self._now() + rotate_timeout_seconds(heading_delta_rad, params)
        while True:
            error = wrap_angle(target_yaw - self._get_yaw())
            if abs(error) < params.heading_tolerance_rad:
                self._publish(0.0, 0.0)
                return
            if self._should_abort():
                self._publish(0.0, 0.0)
                raise ControllerCancelledError("rotate_to_heading cancelled")
            if self._now() >= deadline:
                self._publish(0.0, 0.0)
                raise ControllerTimeoutError(
                    f"rotate_to_heading did not converge within budget "
                    f"(target_delta={heading_delta_rad:.3f} rad)"
                )
            angular = proportional_turn(error, params)
            self._publish(0.0, angular)
            self._sleep(period)

    def drive_distance(self, distance_m: float) -> None:
        params = self.params
        period = 1.0 / params.loop_rate_hz
        start_xy = self._get_position()
        deadline = self._now() + drive_timeout_seconds(distance_m, params)
        while True:
            travelled = displacement(start_xy, self._get_position())
            if travelled >= distance_m:
                self._publish(0.0, 0.0)
                return
            if self._should_abort():
                self._publish(0.0, 0.0)
                raise ControllerCancelledError("drive_distance cancelled")
            if self._now() >= deadline:
                self._publish(0.0, 0.0)
                raise ControllerTimeoutError(
                    f"drive_distance did not converge within budget "
                    f"(target_distance={distance_m:.3f} m)"
                )
            self._publish(params.max_linear, 0.0)
            self._sleep(period)

    def execute_command(self, heading_degree: float, distance_m: float) -> None:
        self.rotate_to_heading(math.radians(heading_degree))
        self.drive_distance(distance_m)


def displacement(start_xy: Tuple[float, float], current_xy: Tuple[float, float]) -> float:
    """Euclidean distance between two 2D points."""
    dx = current_xy[0] - start_xy[0]
    dy = current_xy[1] - start_xy[1]
    return math.sqrt(dx * dx + dy * dy)
