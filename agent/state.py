"""RoverState TypedDict and the target-extraction helper.

The LangGraph state is intentionally narrow: a message list (the LLM's
chat history), the original task and extracted target, the most recent
structured observation, a step counter, and a terminal-status pair. The
`add_messages` reducer lets nodes return only the NEW messages they
produced — LangGraph appends them.
"""

import re
from typing import Annotated, Literal, Optional, TypedDict

from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages

from perception.scene_parsing import SceneObservation

Status = Literal["running", "arrived", "failed_max_steps", "aborted"]


class RoverState(TypedDict):
    messages: Annotated[list[BaseMessage], add_messages]
    task: str
    target: str
    last_observation: Optional[SceneObservation]
    step_count: int
    status: Status
    status_message: str


_TARGET_PATTERN = re.compile(
    r"(?:drive\s+to|go\s+to|find|locate)\s+(?:the\s+)?(.+?)\s*[.!?]*$",
    re.IGNORECASE,
)


def extract_target(task: str) -> str:
    """Pull the target object out of a natural-language task.

    Tries common driving phrasings; falls back to the whole trimmed task
    string when no pattern matches, so the LLM still has something useful
    to ground its `look` calls in.
    """
    stripped = task.strip()
    match = _TARGET_PATTERN.match(stripped)
    if match:
        return match.group(1).strip().rstrip(".!?")
    return stripped.rstrip(".!?")
