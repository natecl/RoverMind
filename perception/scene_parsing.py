"""Pure logic for turning Moondream2's free-form answers into structured data.

Zero ROS and zero ML imports — fully unit-testable on the dev laptop.
"""

from dataclasses import dataclass
from typing import Literal, Optional

Direction = Literal["left", "center", "right"]
Distance = Literal["close", "medium", "far"]


@dataclass(frozen=True)
class SceneObservation:
    """Structured result of looking for one target object in one frame."""

    target: str
    found: bool
    direction: Optional[Direction]
    distance: Optional[Distance]
    should_stop: bool
    raw_answers: dict
