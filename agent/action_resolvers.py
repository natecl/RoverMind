"""Pure bucket → numeric resolvers for the movement tools.

Left = CCW positive, matching the ExecuteCommand.action convention.
"""

from typing import Literal

from agent.params import ActionParams

Direction = Literal["left", "right"]
Magnitude = Literal["small", "large"]
Distance = Literal["short", "medium"]


def turn_degrees(direction: str, magnitude: str,
                 params: ActionParams) -> float:
    """Resolve a turn bucket pair to a signed heading change in degrees."""
    if magnitude == "small":
        mag = params.turn_small_deg
    elif magnitude == "large":
        mag = params.turn_large_deg
    else:
        raise ValueError(
            f"magnitude must be 'small' or 'large', got {magnitude!r}"
        )
    if direction == "left":
        return mag
    if direction == "right":
        return -mag
    raise ValueError(
        f"direction must be 'left' or 'right', got {direction!r}"
    )


def forward_meters(distance: str, params: ActionParams) -> float:
    """Resolve a forward bucket to a non-negative distance in metres."""
    if distance == "short":
        return params.forward_short_m
    if distance == "medium":
        return params.forward_medium_m
    raise ValueError(
        f"distance must be 'short' or 'medium', got {distance!r}"
    )


def search_degrees(params: ActionParams) -> float:
    """Resolve the search rotation to a signed heading change in degrees."""
    return params.search_deg
