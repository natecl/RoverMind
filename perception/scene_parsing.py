"""Pure logic for turning Moondream2's free-form answers into structured data.

Zero ROS and zero ML imports — fully unit-testable on the dev laptop.
"""

import re
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


def parse_yes_no(answer: str) -> bool:
    """Map a visibility answer to a boolean.

    Finds a 'yes'/'no' token even inside a sentence. Ambiguous input (both
    tokens, neither, or empty) returns False — failing toward 'not found' is
    safe, because the agent then keeps searching instead of acting on a
    phantom target.
    """
    text = answer.lower()
    has_yes = re.search(r"\byes\b", text) is not None
    has_no = re.search(r"\bno\b", text) is not None
    return has_yes and not has_no
