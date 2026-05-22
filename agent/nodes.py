"""LangGraph nodes for the RoverMind agent.

Nodes return a partial state dict; LangGraph merges it into the running
state (with `add_messages` appending message-list updates).
"""

from typing import Any, Dict

from langchain_core.messages import HumanMessage, SystemMessage

from agent.prompts import SYSTEM_PROMPT
from agent.state import extract_target


def init_node(state: Dict[str, Any]) -> Dict[str, Any]:
    """Extract target from task and seed the message list. Runs once."""
    task = state["task"]
    target = extract_target(task)
    return {
        "task": task,
        "target": target,
        "step_count": 0,
        "status": "running",
        "status_message": "",
        "last_observation": None,
        "messages": [
            SystemMessage(content=SYSTEM_PROMPT),
            HumanMessage(content=task),
        ],
    }
