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


def parse_direction(answer: str) -> Optional[Direction]:
    """Map a direction answer to left/center/right.

    Accepts 'middle' and 'centre' as synonyms for center. If the answer names
    more than one direction, or none, it is ambiguous and returns None.
    """
    text = answer.lower()
    present = {
        "left": "left" in text,
        "center": ("center" in text or "centre" in text or "middle" in text),
        "right": "right" in text,
    }
    matches = [name for name, found in present.items() if found]
    if len(matches) == 1:
        return matches[0]
    return None


def parse_distance(answer: str) -> Optional[Distance]:
    """Map a distance answer to close/medium/far.

    Accepts synonyms: 'near' -> close; 'moderate' -> medium; 'distant' -> far.
    If the answer names more than one bucket, or none, it returns None.
    """
    text = answer.lower()
    present = {
        "close": ("close" in text or "near" in text),
        "medium": ("medium" in text or "moderate" in text),
        "far": ("far" in text or "distant" in text),
    }
    matches = [name for name, found in present.items() if found]
    if len(matches) == 1:
        return matches[0]
    return None


def build_observation(
    target: str,
    visible_answer: str,
    direction_answer: str,
    distance_answer: str,
) -> SceneObservation:
    """Assemble a SceneObservation from three raw Moondream2 answers.

    `should_stop` is derived here (found and distance == "close"), never asked
    of the VLM. When the target is not found, direction/distance are None and
    should_stop is False.
    """
    raw_answers = {
        "visible": visible_answer,
        "direction": direction_answer,
        "distance": distance_answer,
    }
    found = parse_yes_no(visible_answer)
    if not found:
        return SceneObservation(
            target=target, found=False, direction=None, distance=None,
            should_stop=False, raw_answers=raw_answers,
        )
    direction = parse_direction(direction_answer)
    distance = parse_distance(distance_answer)
    should_stop = distance == "close"
    return SceneObservation(
        target=target, found=True, direction=direction, distance=distance,
        should_stop=should_stop, raw_answers=raw_answers,
    )
