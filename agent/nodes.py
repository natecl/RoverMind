"""LangGraph nodes for the RoverMind agent.

Nodes return a partial state dict; LangGraph merges it into the running
state (with `add_messages` appending message-list updates).
"""

from typing import Any, Dict

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langgraph.graph import END

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
    - If status is already non-running (e.g., reason node set to "aborted"),
      pass through without processing.
    """
    # If status is already set to a terminal state (e.g., reason node caught
    # an exception), don't overwrite it.
    if state.get("status") != "running":
        return {"messages": []}

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


def check_node(state: Dict[str, Any], *, max_steps: int) -> Dict[str, Any]:
    """Increment step counter; enforce max-steps backstop."""
    new_count = state["step_count"] + 1
    if state["status"] == "running" and new_count >= max_steps:
        return {
            "step_count": new_count,
            "status": "failed_max_steps",
            "status_message": f"hit max steps ({max_steps})",
        }
    return {"step_count": new_count, "status": state["status"]}


def should_continue(state: Dict[str, Any]) -> str:
    """Conditional-edge function used after `check`."""
    if state["status"] == "running":
        return "reason"
    return END


def make_reason_node(llm):
    """Factory: build a `reason` node bound to a tool-bound LLM."""

    def reason(state: Dict[str, Any]) -> Dict[str, Any]:
        try:
            ai = llm.invoke(state["messages"])
        except Exception as exc:
            return {
                "status": "aborted",
                "status_message": f"llm error: {exc}",
                "messages": [],
            }
        return {"messages": [ai]}

    return reason
