"""Wire the four nodes into a LangGraph StateGraph.

Edges:
    START -> init -> reason -> act -> check
    check --(running)--> reason
    check --(terminal)--> END
"""

from typing import Callable

from langgraph.graph import END, START, StateGraph

from agent.command_executor import ExecuteResult
from agent.nodes import act_node, check_node, init_node, make_reason_node, should_continue
from agent.params import ActionParams, AgentParams
from agent.state import RoverState
from agent.tools import build_tools
from perception.scene_parsing import SceneObservation

ExecuteFn = Callable[[float, float], ExecuteResult]
CaptureFn = Callable[[str], SceneObservation]


def build_graph(*,
                llm,
                execute_command: ExecuteFn,
                capture_and_analyze: CaptureFn,
                agent_params: AgentParams,
                action_params: ActionParams):
    """Compose tools + nodes into a runnable StateGraph."""
    bundle = build_tools(execute_command, capture_and_analyze, action_params)
    bound_llm = llm.bind_tools(
        bundle.tools, tool_choice="required", parallel_tool_calls=False,
    )

    g = StateGraph(RoverState)
    g.add_node("init", init_node)
    g.add_node("reason", make_reason_node(bound_llm))
    g.add_node("act", lambda state: act_node(state, tool_bundle=bundle))
    g.add_node("check", lambda state: check_node(state, max_steps=agent_params.max_steps))

    g.add_edge(START, "init")
    g.add_edge("init", "reason")
    g.add_edge("reason", "act")
    g.add_edge("act", "check")
    g.add_conditional_edges("check", should_continue, {"reason": "reason", END: END})

    return g.compile()
