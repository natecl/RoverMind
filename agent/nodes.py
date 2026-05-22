"""LangGraph nodes for the RoverMind agent.

Nodes return a partial state dict; LangGraph merges it into the running
state (with `add_messages` appending message-list updates).
"""

from typing import Any, Dict

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage

from agent.prompts import SYSTEM_PROMPT
from agent.state import extract_target
from agent.tools import ToolBundle


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


def act_node(state: Dict[str, Any], *,
             tool_bundle: ToolBundle) -> Dict[str, Any]:
    """Dispatch the tool call on the latest AIMessage.

    Looks up the tool by name, invokes it with the call's args, and
    appends a ToolMessage with the return value. Special cases:
    - `look` populates last_observation by reading from the bundle's
      observation holder (which the look tool writes to internally).
    - `stop` sets status="arrived" and status_message=<reason>.
    - If the latest AIMessage carries no tool call, abort.
    """
    by_name = {t.name: t for t in tool_bundle.tools}
    latest = state["messages"][-1]
    if not isinstance(latest, AIMessage) or not getattr(latest, "tool_calls", None):
        return {
            "status": "aborted",
            "status_message": "no tool call on latest AIMessage",
            "messages": [],
        }
    call = latest.tool_calls[0]
    name = call["name"]
    args = call.get("args", {})
    call_id = call.get("id", "tool_call")
    tool_obj = by_name.get(name)
    if tool_obj is None:
        return {
            "messages": [
                ToolMessage(content=f"unknown tool: {name}", tool_call_id=call_id)
            ],
        }

    result_str = tool_obj.invoke(args)
    out: Dict[str, Any] = {
        "messages": [ToolMessage(content=result_str, tool_call_id=call_id)],
        "status": state.get("status", "running"),
    }
    if name == "stop":
        out["status"] = "arrived"
        out["status_message"] = args.get("reason", "stop")
    if name == "look":
        obs = tool_bundle.look_observation_holder.get("obs")
        if obs is not None:
            out["last_observation"] = obs
    return out
