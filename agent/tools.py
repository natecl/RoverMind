"""Factory for the five LangGraph agent tools.

Tools are built via a factory that takes injected dependencies (the
command executor and capture_and_analyze) so unit tests can swap in
fakes. In production, the factory is called once at startup with the
real CommandExecutor and capture_and_analyze.

The factory returns a `ToolBundle` carrying both the tool list and a
small mutable dict the `look` tool writes the latest `SceneObservation`
into. The `act` node reads from that dict to populate `last_observation`
on the graph state — `@tool`-wrapped functions can't carry arbitrary
attributes, so a captured-by-closure dict is the simplest sink.

All tool failures become short strings the LLM can read; only the
LLM-side (`agent.nodes.reason`) raises exceptions.
"""

from dataclasses import dataclass
from typing import Callable, Dict, List, Literal, Optional

from langchain_core.tools import BaseTool, tool

from agent.action_resolvers import (
    forward_meters,
    search_degrees,
    turn_degrees,
)
from agent.command_executor import ExecuteResult
from agent.observation_formatter import format_observation
from agent.params import ActionParams
from perception.scene_parsing import SceneObservation

ExecuteFn = Callable[[float, float], ExecuteResult]
CaptureFn = Callable[[str], SceneObservation]


@dataclass
class ToolBundle:
    """Tools plus the shared sink the `look` tool writes observations into."""

    tools: List[BaseTool]
    look_observation_holder: Dict[str, Optional[SceneObservation]]


def build_tools(execute_command: ExecuteFn,
                capture_and_analyze: CaptureFn,
                params: ActionParams) -> ToolBundle:
    """Build the five LangGraph tools with injected dependencies."""

    look_obs_holder: Dict[str, Optional[SceneObservation]] = {"obs": None}

    @tool
    def look(target: str) -> str:
        """Look at the world and report where `target` is and how far away
        it is. Call this before every move so you know what to do next."""
        try:
            obs = capture_and_analyze(target)
        except Exception as exc:
            look_obs_holder["obs"] = None
            return f"look failed: {exc}"
        look_obs_holder["obs"] = obs
        return format_observation(obs)

    @tool
    def turn(direction: Literal["left", "right"],
             magnitude: Literal["small", "large"]) -> str:
        """Turn the rover in place. small ~ 30 deg, large ~ 60 deg."""
        deg = turn_degrees(direction, magnitude, params)
        result = execute_command(deg, 0.0)
        if not result.success:
            return f"move failed: {result.message}"
        return f"turn complete: rotated {deg:.0f} deg"

    @tool
    def forward(distance: Literal["short", "medium"]) -> str:
        """Drive forward. short ~ 0.3 m, medium ~ 0.6 m. Do not use if the
        target is far to the side — turn toward it first."""
        m = forward_meters(distance, params)
        result = execute_command(0.0, m)
        if not result.success:
            return f"move failed: {result.message}"
        return f"forward complete: drove {m:.2f} m"

    @tool
    def search() -> str:
        """Rotate ~45 deg in place to look for the target. Use when look()
        reports the target is not found."""
        deg = search_degrees(params)
        result = execute_command(deg, 0.0)
        if not result.success:
            return f"move failed: {result.message}"
        return f"search complete: rotated {deg:.0f} deg"

    @tool
    def stop(reason: str) -> str:
        """Stop the rover and end the task. Call this once you are close
        to and centered on the target. `reason` is logged for debugging."""
        return f"stop acknowledged: {reason}"

    return ToolBundle(
        tools=[look, turn, forward, search, stop],
        look_observation_holder=look_obs_holder,
    )
