# LangGraph Agent Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the RoverMind LangGraph agent that translates a natural-language driving command into a ReAct loop over five tools (`look`, `turn`, `forward`, `search`, `stop`), with OpenAI GPT as the reasoner and the existing `ExecuteCommand` ROS2 action server as the actuator.

**Architecture:** Pure-Python `agent/` package built in four vertical-slice phases. Each phase is independently runnable and tested before the next begins. The reasoner (OpenAI), the actuator (ROS2 action client), and the perception (Moondream2 via `capture_and_analyze`) are all injected behind narrow interfaces so the graph itself stays laptop-testable with fakes. The only modules that import `rclpy` or `langchain-openai` are thin wrappers that get verified on the rover and via a static-image rehearsal script.

**Tech Stack:** Python 3.10+, LangGraph, LangChain Core, `langchain-openai`, ROS2 Foxy (`rclpy`, action client), existing `perception/` module, `pytest`.

---

## File structure

**Created by this plan:**

- `agent/__init__.py` — package marker.
- `agent/params.py` — `AgentParams` + `ActionParams` dataclasses (pure config).
- `agent/command_executor.py` — sync wrapper around the `ExecuteCommand` ROS2 action client; **rclpy import is module-level → hardware-only**.
- `agent/action_resolvers.py` — pure bucket→numeric resolvers (`turn_degrees`, `forward_meters`, `search_degrees`).
- `agent/observation_formatter.py` — pure `SceneObservation → str` formatter for the `look` tool's return value.
- `agent/tools.py` — `build_tools(execute_command, capture_and_analyze, params)` factory returning a `ToolBundle` (the 5 `@tool`-decorated functions plus a `look_observation_holder` dict the `act` node reads to populate `last_observation`).
- `agent/prompts.py` — system prompt string.
- `agent/state.py` — `RoverState` TypedDict and the `extract_target` helper.
- `agent/nodes.py` — `init`, `act`, `check`, plus `make_reason_node(llm)` factory.
- `agent/graph.py` — `build_graph(...)` wiring nodes and edges.
- `agent/llm.py` — `build_llm(params)` returning a `ChatOpenAI` bound to the tools.
- `config/params.yaml` — single tunable-config file (agent + actions sections).
- `config/__init__.py` — package marker so the dir is importable in tests.
- `agent/config_loader.py` — pure `load_params(path)` reading YAML into `AgentParams`/`ActionParams`.
- `scripts/run_agent.py` — entry point: `python scripts/run_agent.py "drive to the water bottle"`.
- `scripts/test_agent_static.py` — static-image rehearsal: real Moondream + real GPT + fake `execute_command`.
- `tests/test_action_resolvers.py` — pure unit tests.
- `tests/test_observation_formatter.py` — pure unit tests.
- `tests/test_tools.py` — tool factory + each tool with injected fakes.
- `tests/test_state.py` — `extract_target` unit tests.
- `tests/test_nodes.py` — node-level unit tests with fakes.
- `tests/test_graph_happy_path.py` — graph end-to-end with scripted fake LLM.
- `tests/test_graph_termination.py` — graph termination paths (stop, max_steps, errors).
- `tests/test_config_loader.py` — YAML→dataclass unit tests.

**Modified by this plan:**

- `requirements.txt` — add `langgraph`, `langchain`, `langchain-openai`, `langchain-core`, `pyyaml`.
- `README.md` — mark Phase 3 done in the roadmap; mention `OPENAI_API_KEY` setup.

**rclpy handling:** Tests that touch `command_executor.py` use `rclpy = pytest.importorskip("rclpy")` and are skipped on the dev laptop. All other agent modules import zero ROS — they work fine with `pytest` locally.

**Import path note:** `tests/conftest.py` already inserts the repo root into `sys.path`. `scripts/run_agent.py` and `scripts/test_agent_static.py` insert the repo root themselves so they can be run as `python scripts/...` from the repo root.

---

# Phase 1 (Chunk A) — Command executor adapter

The first vertical slice: a thin sync Python wrapper around the `ExecuteCommand` ROS2 action. After Phase 1 you can move the rover from a Python REPL.

## Task 1: `ExecuteResult` dataclass and goal validator

**Files:**
- Create: `agent/__init__.py`
- Create: `agent/command_executor.py`
- Create: `tests/test_command_executor_pure.py`

- [ ] **Step 1: Create the package marker**

Create `agent/__init__.py`:

```python
"""RoverMind LangGraph agent package."""
```

- [ ] **Step 2: Write the failing test for `ExecuteResult`**

Create `tests/test_command_executor_pure.py`:

```python
import pytest

from agent.command_executor import ExecuteResult, validate_command


def test_execute_result_holds_outcome():
    r = ExecuteResult(success=True, message="ok")
    assert r.success is True
    assert r.message == "ok"


def test_validate_command_accepts_zero_distance():
    # turning in place is a valid command (used by search and turn tools)
    validate_command(heading_deg=45.0, distance_m=0.0)


def test_validate_command_accepts_typical_move():
    validate_command(heading_deg=-30.0, distance_m=0.6)


def test_validate_command_rejects_negative_distance():
    with pytest.raises(ValueError, match="distance_m must be non-negative"):
        validate_command(heading_deg=0.0, distance_m=-0.1)


def test_validate_command_rejects_nan_or_inf():
    with pytest.raises(ValueError):
        validate_command(heading_deg=float("nan"), distance_m=0.0)
    with pytest.raises(ValueError):
        validate_command(heading_deg=0.0, distance_m=float("inf"))
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `pytest tests/test_command_executor_pure.py -v`

Expected: All four tests FAIL with `ModuleNotFoundError` or `ImportError` for `agent.command_executor`.

- [ ] **Step 4: Write the minimal implementation**

Create `agent/command_executor.py`:

```python
"""Sync Python wrapper around the ExecuteCommand ROS2 action.

Pure-logic helpers (validate_command, ExecuteResult) are laptop-testable.
The real ActionClient call lives in CommandExecutor below and imports rclpy
at module level — that class is verified on the rover, not unit-tested
locally. Tests that exercise CommandExecutor must `pytest.importorskip("rclpy")`.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ExecuteResult:
    """Outcome of one execute_command call."""

    success: bool
    message: str


def validate_command(heading_deg: float, distance_m: float) -> None:
    """Raise ValueError if the command would be unsafe or malformed.

    The ROS2 action server itself rejects negative distances, but validating
    here gives the agent a fast, local failure rather than waiting for the
    action result. Non-finite inputs are rejected outright — they almost
    always indicate a bug in the upstream resolver.
    """
    if not math.isfinite(heading_deg):
        raise ValueError(f"heading_deg must be finite, got {heading_deg!r}")
    if not math.isfinite(distance_m):
        raise ValueError(f"distance_m must be finite, got {distance_m!r}")
    if distance_m < 0.0:
        raise ValueError(
            f"distance_m must be non-negative, got {distance_m!r}"
        )
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_command_executor_pure.py -v`

Expected: All four tests PASS.

- [ ] **Step 6: Commit**

```bash
git add agent/__init__.py agent/command_executor.py tests/test_command_executor_pure.py
git commit -m "feat(agent): add ExecuteResult and validate_command pure helpers"
```

---

## Task 2: `CommandExecutor` — hardware action-client wrapper

**Files:**
- Modify: `agent/command_executor.py`

This task adds the rclpy-side code. It is **not** unit-tested locally; rclpy is unimportable on the dev laptop and the `ExecuteCommand` interface only exists after `colcon build`. Verification is the on-rover script in Step 3.

- [ ] **Step 1: Add the `CommandExecutor` class**

Append to `agent/command_executor.py`:

```python
# --- Hardware-only ROS2 action client ------------------------------------
#
# Everything below imports rclpy and the generated ExecuteCommand interface
# and is therefore untested on the dev laptop. Mirrors the pattern in
# safety_controller_layer/safety_controller_node.py.

import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from safety_controller_layer_interfaces.action import ExecuteCommand


ACTION_NAME = "execute_command"
DEFAULT_SERVER_TIMEOUT_S = 5.0


class CommandExecutorError(RuntimeError):
    """Raised when the action server is unreachable or aborts a goal."""


class CommandExecutor:
    """Owns an rclpy Node + ActionClient; exposes a sync execute() call.

    Constructed once per process. `execute(heading_deg, distance_m)` blocks
    until the action server returns a result and yields an `ExecuteResult`.
    Cancellation, retries, and async fan-out are intentionally not exposed —
    the LangGraph agent runs one tool call at a time.
    """

    def __init__(self, node: Optional[Node] = None,
                 server_timeout_s: float = DEFAULT_SERVER_TIMEOUT_S):
        if not rclpy.ok():
            rclpy.init()
        self._owns_node = node is None
        self._node = node or Node("rovermind_command_executor")
        self._client = ActionClient(self._node, ExecuteCommand, ACTION_NAME)
        self._server_timeout_s = server_timeout_s

    def execute(self, heading_deg: float, distance_m: float) -> ExecuteResult:
        validate_command(heading_deg, distance_m)
        if not self._client.wait_for_server(timeout_sec=self._server_timeout_s):
            raise CommandExecutorError(
                f"action server '{ACTION_NAME}' not available within "
                f"{self._server_timeout_s:.1f}s"
            )
        goal = ExecuteCommand.Goal()
        goal.heading_degree = float(heading_deg)
        goal.distance_m = float(distance_m)

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return ExecuteResult(success=False, message="goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            return ExecuteResult(success=False, message="no result returned")
        action_result = wrapped.result
        return ExecuteResult(
            success=bool(action_result.success),
            message=str(action_result.message),
        )

    def close(self) -> None:
        if self._owns_node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
```

- [ ] **Step 2: Add the convenience module-level callable**

Append to `agent/command_executor.py`:

```python
_default_executor: Optional[CommandExecutor] = None


def execute_command(heading_deg: float, distance_m: float) -> ExecuteResult:
    """Process-singleton convenience for short scripts.

    The first call constructs a CommandExecutor (which initializes rclpy and
    creates a Node). Subsequent calls reuse it. For long-running processes,
    construct a CommandExecutor explicitly so its lifecycle is visible.
    """
    global _default_executor
    if _default_executor is None:
        _default_executor = CommandExecutor()
    return _default_executor.execute(heading_deg, distance_m)
```

- [ ] **Step 3: Verify on the rover (manual)**

On the rover, with `safety_controller_node` and the LIMO base running:

```bash
python3 -c "from agent.command_executor import execute_command; print(execute_command(30.0, 0.0))"
```

Expected: rover rotates 30 degrees CCW; stdout shows `ExecuteResult(success=True, message='completed: ...')`.

Then a forward move:

```bash
python3 -c "from agent.command_executor import execute_command; print(execute_command(0.0, 0.3))"
```

Expected: rover drives 0.3 m forward; stdout shows success.

- [ ] **Step 4: Commit**

```bash
git add agent/command_executor.py
git commit -m "feat(agent): add CommandExecutor action-client wrapper (hardware-only)"
```

---

# Phase 2 (Chunk B) — Tool layer

Five tool functions that resolve the LLM-visible bucket vocabulary into either `capture_and_analyze` (for `look`) or `execute_command` (for movement). All unit-testable with injected fakes; no LLM yet.

## Task 3: `AgentParams` / `ActionParams` dataclasses

**Files:**
- Create: `agent/params.py`
- Create: `tests/test_params.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_params.py`:

```python
from agent.params import ActionParams, AgentParams


def test_action_params_has_documented_defaults():
    p = ActionParams()
    assert p.turn_small_deg == 30.0
    assert p.turn_large_deg == 60.0
    assert p.search_deg == 45.0
    assert p.forward_short_m == 0.3
    assert p.forward_medium_m == 0.6


def test_agent_params_has_documented_defaults():
    p = AgentParams()
    assert p.llm_provider == "openai"
    assert p.llm_model == "gpt-4o-mini"
    assert p.llm_temperature == 0.0
    assert p.max_steps == 20


def test_action_params_is_frozen():
    p = ActionParams()
    try:
        p.turn_small_deg = 99.0  # type: ignore[misc]
    except Exception:
        return
    raise AssertionError("ActionParams should be frozen")
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_params.py -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'agent.params'`.

- [ ] **Step 3: Write the implementation**

Create `agent/params.py`:

```python
"""Tunable parameters for the LangGraph agent.

Frozen dataclasses so the values are safe to share across nodes and tools.
Defaults match the design spec; production code loads overrides from
config/params.yaml via agent.config_loader.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ActionParams:
    """Bucket → numeric resolution for the movement tools."""

    turn_small_deg: float = 30.0
    turn_large_deg: float = 60.0
    search_deg: float = 45.0
    forward_short_m: float = 0.3
    forward_medium_m: float = 0.6


@dataclass(frozen=True)
class AgentParams:
    """LLM and graph-level parameters."""

    llm_provider: str = "openai"
    llm_model: str = "gpt-4o-mini"
    llm_temperature: float = 0.0
    max_steps: int = 20
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_params.py -v`

Expected: All three tests PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/params.py tests/test_params.py
git commit -m "feat(agent): add AgentParams and ActionParams dataclasses"
```

---

## Task 4: Action-bucket resolvers

**Files:**
- Create: `agent/action_resolvers.py`
- Create: `tests/test_action_resolvers.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_action_resolvers.py`:

```python
import pytest

from agent.action_resolvers import (
    forward_meters,
    search_degrees,
    turn_degrees,
)
from agent.params import ActionParams


def test_turn_left_small_returns_positive_30():
    # CCW positive convention matches ExecuteCommand.action; left = CCW.
    assert turn_degrees("left", "small", ActionParams()) == 30.0


def test_turn_right_small_returns_negative_30():
    assert turn_degrees("right", "small", ActionParams()) == -30.0


def test_turn_left_large_returns_positive_60():
    assert turn_degrees("left", "large", ActionParams()) == 60.0


def test_turn_right_large_returns_negative_60():
    assert turn_degrees("right", "large", ActionParams()) == -60.0


def test_turn_rejects_unknown_direction():
    with pytest.raises(ValueError, match="direction"):
        turn_degrees("up", "small", ActionParams())


def test_turn_rejects_unknown_magnitude():
    with pytest.raises(ValueError, match="magnitude"):
        turn_degrees("left", "tiny", ActionParams())


def test_forward_short_returns_0_3_m():
    assert forward_meters("short", ActionParams()) == 0.3


def test_forward_medium_returns_0_6_m():
    assert forward_meters("medium", ActionParams()) == 0.6


def test_forward_rejects_unknown_distance():
    with pytest.raises(ValueError, match="distance"):
        forward_meters("huge", ActionParams())


def test_search_returns_configured_degrees():
    assert search_degrees(ActionParams()) == 45.0
    assert search_degrees(ActionParams(search_deg=60.0)) == 60.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_action_resolvers.py -v`

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

Create `agent/action_resolvers.py`:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_action_resolvers.py -v`

Expected: All ten tests PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/action_resolvers.py tests/test_action_resolvers.py
git commit -m "feat(agent): add bucket→numeric action resolvers"
```

---

## Task 5: `SceneObservation` formatter

**Files:**
- Create: `agent/observation_formatter.py`
- Create: `tests/test_observation_formatter.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_observation_formatter.py`:

```python
from agent.observation_formatter import format_observation
from perception.scene_parsing import SceneObservation


def _obs(**kw):
    base = dict(
        target="water bottle", found=False, direction=None, distance=None,
        should_stop=False, raw_answers={"visible": "", "direction": "", "distance": ""},
    )
    base.update(kw)
    return SceneObservation(**base)


def test_format_not_found():
    obs = _obs(found=False)
    assert format_observation(obs) == "target not found"


def test_format_found_with_direction_and_distance():
    obs = _obs(
        found=True, direction="left", distance="medium",
        raw_answers={"visible": "Yes.", "direction": "left", "distance": "medium"},
    )
    assert format_observation(obs) == (
        "target found at left, medium distance"
    )


def test_format_found_close_includes_arrival_hint():
    obs = _obs(
        found=True, direction="center", distance="close", should_stop=True,
        raw_answers={"visible": "Yes.", "direction": "center", "distance": "close"},
    )
    assert format_observation(obs) == (
        "target found at center, close distance (arrived: call stop)"
    )


def test_format_found_unclear_direction():
    obs = _obs(
        found=True, direction=None, distance="medium",
        raw_answers={"visible": "Yes.", "direction": "?", "distance": "medium"},
    )
    assert format_observation(obs) == (
        "target found at unknown direction, medium distance"
    )


def test_format_found_unclear_distance():
    obs = _obs(
        found=True, direction="right", distance=None,
        raw_answers={"visible": "Yes.", "direction": "right", "distance": "?"},
    )
    assert format_observation(obs) == (
        "target found at right, unknown distance"
    )
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_observation_formatter.py -v`

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

Create `agent/observation_formatter.py`:

```python
"""Format a SceneObservation into a short string for the LLM."""

from perception.scene_parsing import SceneObservation


def format_observation(obs: SceneObservation) -> str:
    """Turn a SceneObservation into the string the LLM reads.

    Strings use the bucket vocabulary verbatim so the LLM, the system
    prompt, and the perception layer share words.
    """
    if not obs.found:
        return "target not found"

    direction = obs.direction or "unknown direction"
    distance = (
        f"{obs.distance} distance" if obs.distance is not None
        else "unknown distance"
    )

    base = f"target found at {direction}, {distance}"
    if obs.should_stop:
        return f"{base} (arrived: call stop)"
    return base
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_observation_formatter.py -v`

Expected: All five tests PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/observation_formatter.py tests/test_observation_formatter.py
git commit -m "feat(agent): add SceneObservation formatter for the look tool"
```

---

## Task 6: Tool factory + the five tools

**Files:**
- Create: `agent/tools.py`
- Create: `tests/test_tools.py`

- [ ] **Step 1: Add LangChain dependencies**

Append to `requirements.txt`:

```
langgraph
langchain-core
langchain-openai
pyyaml
```

Install locally for testing:

```bash
pip install langgraph langchain-core langchain-openai pyyaml
```

- [ ] **Step 2: Write the failing test**

Create `tests/test_tools.py`:

```python
from typing import List, Tuple

import pytest

from agent.command_executor import ExecuteResult
from agent.params import ActionParams
from agent.tools import build_tools
from perception.scene_parsing import SceneObservation


class FakeExecutor:
    """Records every (heading_deg, distance_m) call. Returns canned results."""

    def __init__(self, result: ExecuteResult = ExecuteResult(True, "ok")):
        self.calls: List[Tuple[float, float]] = []
        self._result = result

    def __call__(self, heading_deg: float, distance_m: float) -> ExecuteResult:
        self.calls.append((heading_deg, distance_m))
        return self._result


def _found_obs(direction="center", distance="medium", should_stop=False):
    return SceneObservation(
        target="water bottle",
        found=True,
        direction=direction,
        distance=distance,
        should_stop=should_stop,
        raw_answers={"visible": "Yes.", "direction": direction, "distance": distance},
    )


def _not_found_obs():
    return SceneObservation(
        target="water bottle", found=False, direction=None, distance=None,
        should_stop=False,
        raw_answers={"visible": "No.", "direction": "", "distance": ""},
    )


def _by_name(tools, name):
    for t in tools:
        if t.name == name:
            return t
    raise AssertionError(f"no tool named {name!r}")


def test_build_tools_returns_bundle_with_five_named_tools():
    bundle = build_tools(
        execute_command=FakeExecutor(),
        capture_and_analyze=lambda target: _found_obs(),
        params=ActionParams(),
    )
    names = sorted(t.name for t in bundle.tools)
    assert names == ["forward", "look", "search", "stop", "turn"]
    assert bundle.look_observation_holder == {"obs": None}


def test_look_calls_capture_and_returns_formatted_string():
    calls = []

    def fake_capture(target):
        calls.append(target)
        return _found_obs(direction="left", distance="medium")

    bundle = build_tools(FakeExecutor(), fake_capture, ActionParams())
    look = _by_name(bundle.tools, "look")

    result = look.invoke({"target": "water bottle"})

    assert calls == ["water bottle"]
    assert result == "target found at left, medium distance"


def test_look_records_observation_in_holder():
    obs = _found_obs(direction="right", distance="far")
    bundle = build_tools(FakeExecutor(), lambda t: obs, ActionParams())
    look = _by_name(bundle.tools, "look")
    look.invoke({"target": "water bottle"})
    assert bundle.look_observation_holder["obs"] == obs


def test_look_when_not_found():
    bundle = build_tools(
        FakeExecutor(), lambda target: _not_found_obs(), ActionParams(),
    )
    look = _by_name(bundle.tools, "look")
    assert look.invoke({"target": "water bottle"}) == "target not found"


def test_look_surfaces_capture_errors_as_strings_and_clears_holder():
    def bad_capture(target):
        raise RuntimeError("no camera frame")

    bundle = build_tools(FakeExecutor(), bad_capture, ActionParams())
    look = _by_name(bundle.tools, "look")
    bundle.look_observation_holder["obs"] = "stale"  # simulate prior look
    assert look.invoke({"target": "water bottle"}) == (
        "look failed: no camera frame"
    )
    # A failed look must NOT leave stale data behind.
    assert bundle.look_observation_holder["obs"] is None


def test_turn_left_small_dispatches_positive_30_zero_distance():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    turn = _by_name(bundle.tools, "turn")

    msg = turn.invoke({"direction": "left", "magnitude": "small"})

    assert fx.calls == [(30.0, 0.0)]
    assert "turn complete" in msg


def test_turn_right_large_dispatches_negative_60_zero_distance():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    turn = _by_name(bundle.tools, "turn")
    turn.invoke({"direction": "right", "magnitude": "large"})
    assert fx.calls == [(-60.0, 0.0)]


def test_forward_short_dispatches_zero_30cm():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    forward = _by_name(bundle.tools, "forward")
    forward.invoke({"distance": "short"})
    assert fx.calls == [(0.0, 0.3)]


def test_forward_medium_dispatches_zero_60cm():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    forward = _by_name(bundle.tools, "forward")
    forward.invoke({"distance": "medium"})
    assert fx.calls == [(0.0, 0.6)]


def test_search_dispatches_45_degrees_zero_distance():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    search = _by_name(bundle.tools, "search")
    search.invoke({})
    assert fx.calls == [(45.0, 0.0)]


def test_movement_tool_surfaces_action_failure_as_string():
    fx = FakeExecutor(result=ExecuteResult(False, "aborted: AEB"))
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    forward = _by_name(bundle.tools, "forward")
    msg = forward.invoke({"distance": "short"})
    assert "move failed" in msg and "AEB" in msg


def test_stop_returns_string_acknowledgement():
    bundle = build_tools(FakeExecutor(), lambda t: _found_obs(), ActionParams())
    stop = _by_name(bundle.tools, "stop")
    msg = stop.invoke({"reason": "arrived at water bottle"})
    assert msg == "stop acknowledged: arrived at water bottle"
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `pytest tests/test_tools.py -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'agent.tools'`.

- [ ] **Step 4: Write the implementation**

Create `agent/tools.py`:

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_tools.py -v`

Expected: All 11 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add agent/tools.py tests/test_tools.py requirements.txt
git commit -m "feat(agent): add five LangGraph tools (look/turn/forward/search/stop) with factory"
```

---

# Phase 3 (Chunk C) — LangGraph state machine with a mocked LLM

The graph wired up and tested end-to-end with a scripted fake LLM. No real OpenAI calls yet; all of Phase 3 is deterministic.

## Task 7: `RoverState` and `extract_target`

**Files:**
- Create: `agent/state.py`
- Create: `tests/test_state.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_state.py`:

```python
from agent.state import extract_target


def test_extract_target_drive_to_the_x():
    assert extract_target("drive to the water bottle") == "water bottle"


def test_extract_target_go_to_the_x():
    assert extract_target("go to the red chair") == "red chair"


def test_extract_target_find_the_x():
    assert extract_target("find the laptop") == "laptop"


def test_extract_target_trailing_punctuation_stripped():
    assert extract_target("drive to the water bottle.") == "water bottle"
    assert extract_target("go to the chair!") == "chair"


def test_extract_target_case_insensitive_match():
    assert extract_target("Drive To The Backpack") == "Backpack"


def test_extract_target_falls_back_to_full_task_if_no_pattern():
    # Unstructured task → return the trimmed task string itself so the
    # downstream LLM sees something usable rather than an empty target.
    assert extract_target("the green book please") == "the green book please"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_state.py -v`

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the implementation**

Create `agent/state.py`:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_state.py -v`

Expected: All six tests PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/state.py tests/test_state.py
git commit -m "feat(agent): add RoverState TypedDict and extract_target helper"
```

---

## Task 8: System prompt + `init` node

**Files:**
- Create: `agent/prompts.py`
- Create: `agent/nodes.py`
- Create: `tests/test_nodes.py`

- [ ] **Step 1: Write the failing test for the init node**

Create `tests/test_nodes.py`:

```python
from langchain_core.messages import HumanMessage, SystemMessage

from agent.nodes import init_node


def test_init_extracts_target_and_seeds_state():
    initial = {"task": "drive to the water bottle"}
    out = init_node(initial)

    assert out["target"] == "water bottle"
    assert out["step_count"] == 0
    assert out["status"] == "running"
    assert out["status_message"] == ""
    assert out["last_observation"] is None
    # Two messages: system prompt + user task.
    msgs = out["messages"]
    assert len(msgs) == 2
    assert isinstance(msgs[0], SystemMessage)
    assert "five tools" in msgs[0].content.lower()
    assert isinstance(msgs[1], HumanMessage)
    assert msgs[1].content == "drive to the water bottle"


def test_init_preserves_original_task_string():
    out = init_node({"task": "find the laptop please"})
    assert out["task"] == "find the laptop please"
    assert out["target"] == "laptop please"
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `pytest tests/test_nodes.py -v`

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Write the system prompt**

Create `agent/prompts.py`:

```python
"""System prompt for the RoverMind LangGraph agent."""

SYSTEM_PROMPT = """You control a rover with five tools: look, turn, forward, search, stop.
Vision reports direction as {left, center, right} and distance as {close, medium, far}.

Strategy:
1. Call look(target) to find out where the target is.
2. If not found, call search() and look again.
3. If found but not centered, turn toward it (left -> turn left, right -> turn right).
4. If centered but not close, forward.
5. When the target is centered AND close, call stop("arrived").

Always call look between movements. Never act blind. One tool per turn."""
```

- [ ] **Step 4: Write the `init` node**

Create `agent/nodes.py`:

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_nodes.py -v`

Expected: Both tests PASS.

- [ ] **Step 6: Commit**

```bash
git add agent/prompts.py agent/nodes.py tests/test_nodes.py
git commit -m "feat(agent): add system prompt and init node"
```

---

## Task 9: `act` node — dispatch tool calls

**Files:**
- Modify: `agent/nodes.py`
- Modify: `tests/test_nodes.py`

- [ ] **Step 1: Add the failing tests for `act_node`**

Append to `tests/test_nodes.py`:

```python
from typing import List

from langchain_core.messages import AIMessage, ToolMessage

from agent.command_executor import ExecuteResult
from agent.nodes import act_node
from agent.params import ActionParams
from agent.tools import build_tools
from perception.scene_parsing import SceneObservation


def _fake_executor():
    calls: List[tuple] = []

    def fn(h, d):
        calls.append((h, d))
        return ExecuteResult(True, "ok")

    fn.calls = calls  # type: ignore[attr-defined]
    return fn


def _found_obs():
    return SceneObservation(
        target="water bottle", found=True, direction="center",
        distance="medium", should_stop=False,
        raw_answers={"visible": "Yes.", "direction": "center", "distance": "medium"},
    )


def _ai_with_tool_call(name, args):
    return AIMessage(
        content="",
        tool_calls=[{"id": "call_1", "name": name, "args": args}],
    )


def test_act_dispatches_turn_and_returns_tool_message():
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    state = {
        "messages": [_ai_with_tool_call("turn", {"direction": "left", "magnitude": "small"})],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert fx.calls == [(30.0, 0.0)]  # type: ignore[attr-defined]
    msgs = out["messages"]
    assert len(msgs) == 1
    assert isinstance(msgs[0], ToolMessage)
    assert msgs[0].tool_call_id == "call_1"
    assert "turn complete" in msgs[0].content
    assert out["status"] == "running"


def test_act_dispatches_look_and_records_last_observation():
    obs = _found_obs()
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: obs, ActionParams())
    state = {
        "messages": [_ai_with_tool_call("look", {"target": "water bottle"})],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert out["last_observation"] == obs
    assert isinstance(out["messages"][0], ToolMessage)
    assert "target found" in out["messages"][0].content


def test_act_on_stop_sets_terminal_status():
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    state = {
        "messages": [_ai_with_tool_call("stop", {"reason": "arrived"})],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert out["status"] == "arrived"
    assert out["status_message"] == "arrived"
    assert isinstance(out["messages"][0], ToolMessage)
    assert "stop acknowledged" in out["messages"][0].content


def test_act_aborts_when_latest_message_has_no_tool_call():
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    state = {
        "messages": [AIMessage(content="hello")],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert out["status"] == "aborted"
    assert "no tool call" in out["status_message"]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_nodes.py -v`

Expected: Four new tests FAIL with `ImportError: cannot import name 'act_node'`.

- [ ] **Step 3: Implement `act_node`**

Append to `agent/nodes.py`:

```python
from langchain_core.messages import AIMessage, ToolMessage

from agent.tools import ToolBundle


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
    }
    if name == "stop":
        out["status"] = "arrived"
        out["status_message"] = args.get("reason", "stop")
    if name == "look":
        obs = tool_bundle.look_observation_holder.get("obs")
        if obs is not None:
            out["last_observation"] = obs
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_nodes.py tests/test_tools.py -v`

Expected: All tests PASS (the existing `test_tools.py` tests still pass).

- [ ] **Step 5: Commit**

```bash
git add agent/nodes.py tests/test_nodes.py
git commit -m "feat(agent): add act node that dispatches tool calls through ToolBundle"
```

---

## Task 10: `check` node — termination logic

**Files:**
- Modify: `agent/nodes.py`
- Modify: `tests/test_nodes.py`

- [ ] **Step 1: Add the failing tests**

Append to `tests/test_nodes.py`:

```python
from agent.nodes import check_node, should_continue


def _running_state(step_count=0, status="running"):
    return {
        "step_count": step_count,
        "status": status,
        "status_message": "",
    }


def test_check_increments_step_count_when_running():
    out = check_node(_running_state(step_count=3), max_steps=20)
    assert out["step_count"] == 4
    assert out["status"] == "running"


def test_check_sets_failed_max_steps_when_cap_hit():
    out = check_node(_running_state(step_count=19), max_steps=20)
    assert out["step_count"] == 20
    assert out["status"] == "failed_max_steps"
    assert "max steps" in out["status_message"]


def test_check_passes_terminal_status_through():
    out = check_node(_running_state(step_count=5, status="arrived"), max_steps=20)
    assert out["status"] == "arrived"
    assert out["step_count"] == 6  # still counts the cycle


def test_should_continue_returns_reason_when_running():
    state = {"status": "running", "step_count": 4}
    assert should_continue(state) == "reason"


def test_should_continue_returns_end_when_terminal():
    for status in ("arrived", "failed_max_steps", "aborted"):
        state = {"status": status, "step_count": 4}
        assert should_continue(state) == "__end__"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_nodes.py::test_check_increments_step_count_when_running -v`

Expected: FAIL with `ImportError`.

- [ ] **Step 3: Implement `check_node` and `should_continue`**

Append to `agent/nodes.py`:

```python
from langgraph.graph import END


def check_node(state: Dict[str, Any], *, max_steps: int) -> Dict[str, Any]:
    """Increment step counter; enforce max-steps backstop."""
    new_count = state["step_count"] + 1
    if state["status"] == "running" and new_count >= max_steps:
        return {
            "step_count": new_count,
            "status": "failed_max_steps",
            "status_message": f"hit max steps ({max_steps})",
        }
    return {"step_count": new_count}


def should_continue(state: Dict[str, Any]) -> str:
    """Conditional-edge function used after `check`."""
    if state["status"] == "running":
        return "reason"
    return END
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_nodes.py -v`

Expected: All node tests PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/nodes.py tests/test_nodes.py
git commit -m "feat(agent): add check node and should_continue conditional edge"
```

---

## Task 11: `reason` node — LLM call wrapper

**Files:**
- Modify: `agent/nodes.py`
- Modify: `tests/test_nodes.py`

The `reason` node is a thin wrapper around the bound LLM. Tests use a fake LLM that returns scripted `AIMessage`s.

- [ ] **Step 1: Add the failing tests**

Append to `tests/test_nodes.py`:

```python
from agent.nodes import make_reason_node


class ScriptedLLM:
    """Fake LLM that returns scripted AIMessages, one per call.

    Records the messages it was given so tests can assert on the
    conversation context the real LLM would see.
    """

    def __init__(self, scripted_responses):
        self._responses = list(scripted_responses)
        self.received: list = []

    def invoke(self, messages):
        self.received.append(list(messages))
        if not self._responses:
            raise AssertionError("ScriptedLLM ran out of responses")
        return self._responses.pop(0)


def _ai(name, args):
    return AIMessage(
        content="",
        tool_calls=[{"id": f"call_{name}", "name": name, "args": args}],
    )


def test_reason_calls_llm_with_messages_and_appends_response():
    llm = ScriptedLLM([_ai("look", {"target": "water bottle"})])
    reason = make_reason_node(llm)
    state = {
        "messages": [HumanMessage(content="drive to the water bottle")],
    }

    out = reason(state)

    assert len(llm.received) == 1
    assert llm.received[0] == state["messages"]
    assert len(out["messages"]) == 1
    assert isinstance(out["messages"][0], AIMessage)
    assert out["messages"][0].tool_calls[0]["name"] == "look"


def test_reason_aborts_on_llm_exception():
    class ExplodingLLM:
        def invoke(self, messages):
            raise RuntimeError("openai api down")

    reason = make_reason_node(ExplodingLLM())
    out = reason({"messages": [HumanMessage(content="drive to the water bottle")]})

    assert out["status"] == "aborted"
    assert "openai api down" in out["status_message"]
    assert out["messages"] == []
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_nodes.py -v -k reason`

Expected: FAIL with `ImportError: cannot import name 'make_reason_node'`.

- [ ] **Step 3: Implement `make_reason_node`**

Append to `agent/nodes.py`:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_nodes.py -v`

Expected: All node tests PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/nodes.py tests/test_nodes.py
git commit -m "feat(agent): add reason node factory bound to an injected LLM"
```

---

## Task 12: `build_graph` — wire nodes into a LangGraph

**Files:**
- Create: `agent/graph.py`
- Create: `tests/test_graph_happy_path.py`

- [ ] **Step 1: Write a happy-path graph test using a scripted fake LLM**

Create `tests/test_graph_happy_path.py`:

```python
from langchain_core.messages import AIMessage, HumanMessage, ToolMessage

from agent.command_executor import ExecuteResult
from agent.graph import build_graph
from agent.params import ActionParams, AgentParams
from perception.scene_parsing import SceneObservation


def _found(direction="center", distance="medium", should_stop=False):
    return SceneObservation(
        target="water bottle", found=True, direction=direction,
        distance=distance, should_stop=should_stop,
        raw_answers={"visible": "Yes.", "direction": direction, "distance": distance},
    )


def _ai(name, args, call_id="x"):
    return AIMessage(
        content="",
        tool_calls=[{"id": call_id, "name": name, "args": args}],
    )


class ScriptedLLM:
    def __init__(self, responses):
        self._responses = list(responses)

    def invoke(self, messages):
        return self._responses.pop(0)

    def bind_tools(self, tools, **kw):
        # graph.py uses llm.bind_tools(...) for the real LLM; scripted
        # LLM ignores tool binding because it returns prebaked tool calls.
        return self


def test_happy_path_look_turn_look_forward_look_stop():
    fx_calls = []

    def fake_execute(h, d):
        fx_calls.append((h, d))
        return ExecuteResult(True, "ok")

    observations = [
        _found(direction="left", distance="medium"),   # 1st look
        _found(direction="center", distance="medium"), # 2nd look (after turn)
        _found(direction="center", distance="close",   # 3rd look (after forward)
               should_stop=True),
    ]

    def fake_capture(target):
        return observations.pop(0)

    llm = ScriptedLLM([
        _ai("look", {"target": "water bottle"}, "c1"),
        _ai("turn", {"direction": "left", "magnitude": "small"}, "c2"),
        _ai("look", {"target": "water bottle"}, "c3"),
        _ai("forward", {"distance": "medium"}, "c4"),
        _ai("look", {"target": "water bottle"}, "c5"),
        _ai("stop", {"reason": "arrived at the water bottle"}, "c6"),
    ])

    graph = build_graph(
        llm=llm,
        execute_command=fake_execute,
        capture_and_analyze=fake_capture,
        agent_params=AgentParams(max_steps=20),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "arrived"
    assert final["status_message"] == "arrived at the water bottle"
    assert final["target"] == "water bottle"
    assert fx_calls == [(30.0, 0.0), (0.0, 0.6)]
    # Three looks → three observations recorded; last one is should_stop.
    assert final["last_observation"].should_stop is True

    # Sanity-check the messages contain six AIMessages and six ToolMessages
    # plus the system + user (8 + system + human = 14 total).
    kinds = [type(m).__name__ for m in final["messages"]]
    assert kinds.count("AIMessage") == 6
    assert kinds.count("ToolMessage") == 6
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `pytest tests/test_graph_happy_path.py -v`

Expected: FAIL with `ModuleNotFoundError: No module named 'agent.graph'`.

- [ ] **Step 3: Implement `build_graph`**

Create `agent/graph.py`:

```python
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
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `pytest tests/test_graph_happy_path.py -v`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/graph.py tests/test_graph_happy_path.py
git commit -m "feat(agent): wire init/reason/act/check into a LangGraph StateGraph"
```

---

## Task 13: Termination-path graph tests

**Files:**
- Create: `tests/test_graph_termination.py`

- [ ] **Step 1: Write the termination tests**

Create `tests/test_graph_termination.py`:

```python
from langchain_core.messages import AIMessage

from agent.command_executor import ExecuteResult
from agent.graph import build_graph
from agent.params import ActionParams, AgentParams
from perception.scene_parsing import SceneObservation


def _found():
    return SceneObservation(
        target="water bottle", found=True, direction="left",
        distance="medium", should_stop=False,
        raw_answers={"visible": "Yes.", "direction": "left", "distance": "medium"},
    )


def _ai(name, args, call_id="x"):
    return AIMessage(
        content="",
        tool_calls=[{"id": call_id, "name": name, "args": args}],
    )


class ScriptedLLM:
    def __init__(self, responses):
        self._responses = list(responses)

    def invoke(self, messages):
        if not self._responses:
            # Default to "look" forever; lets the max_steps test loop.
            return _ai("look", {"target": "water bottle"}, "ck")
        return self._responses.pop(0)

    def bind_tools(self, tools, **kw):
        return self


def test_max_steps_backstop_fires_when_llm_never_stops():
    def fake_execute(h, d):
        return ExecuteResult(True, "ok")

    def fake_capture(target):
        return _found()

    graph = build_graph(
        llm=ScriptedLLM([]),  # always "look"
        execute_command=fake_execute,
        capture_and_analyze=fake_capture,
        agent_params=AgentParams(max_steps=5),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "failed_max_steps"
    assert final["step_count"] == 5
    assert "max steps" in final["status_message"]


def test_aborts_when_llm_raises():
    class ExplodingLLM:
        def invoke(self, messages):
            raise RuntimeError("openai api down")

        def bind_tools(self, tools, **kw):
            return self

    graph = build_graph(
        llm=ExplodingLLM(),
        execute_command=lambda h, d: ExecuteResult(True, "ok"),
        capture_and_analyze=lambda t: _found(),
        agent_params=AgentParams(max_steps=20),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "aborted"
    assert "openai api down" in final["status_message"]


def test_move_failure_surfaces_as_tool_message_and_loop_continues():
    fx_calls = []

    def fake_execute(h, d):
        fx_calls.append((h, d))
        # First move fails; subsequent moves succeed.
        if len(fx_calls) == 1:
            return ExecuteResult(False, "aborted: AEB triggered")
        return ExecuteResult(True, "ok")

    def fake_capture(target):
        return _found()

    llm = ScriptedLLM([
        _ai("forward", {"distance": "short"}, "c1"),
        _ai("look", {"target": "water bottle"}, "c2"),
        _ai("stop", {"reason": "good enough"}, "c3"),
    ])

    graph = build_graph(
        llm=llm,
        execute_command=fake_execute,
        capture_and_analyze=fake_capture,
        agent_params=AgentParams(max_steps=20),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "arrived"
    # The failed move's ToolMessage must include the AEB string so the
    # LLM (real or scripted) can see what happened.
    assert any(
        "AEB triggered" in getattr(m, "content", "")
        for m in final["messages"]
    )
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `pytest tests/test_graph_termination.py -v`

Expected: All three tests PASS. (Implementation already exists from Task 12; this is a pure verification task.)

- [ ] **Step 3: Commit**

```bash
git add tests/test_graph_termination.py
git commit -m "test(agent): cover max_steps, llm-error, and tool-failure paths"
```

---

# Phase 4 (Chunk D) — Real OpenAI integration, config, and rehearsal scripts

The final vertical slice: swap the scripted LLM for `ChatOpenAI`, load tunables from YAML, and provide two entry points — `scripts/run_agent.py` for the live rover and `scripts/test_agent_static.py` for a real-LLM rehearsal that does not touch hardware.

## Task 14: YAML config loader

**Files:**
- Create: `config/params.yaml`
- Create: `agent/config_loader.py`
- Create: `tests/test_config_loader.py`

- [ ] **Step 1: Write `config/params.yaml`**

Create `config/params.yaml`:

```yaml
agent:
  llm_provider: openai
  llm_model: gpt-4o-mini
  llm_temperature: 0.0
  max_steps: 20

actions:
  turn_small_deg: 30.0
  turn_large_deg: 60.0
  search_deg: 45.0
  forward_short_m: 0.3
  forward_medium_m: 0.6
```

- [ ] **Step 2: Write the failing tests**

Create `tests/test_config_loader.py`:

```python
from pathlib import Path

import pytest

from agent.config_loader import load_params
from agent.params import ActionParams, AgentParams


def _write(tmp_path: Path, content: str) -> Path:
    path = tmp_path / "params.yaml"
    path.write_text(content)
    return path


def test_loads_default_repo_yaml(tmp_path):
    yaml_text = (
        "agent:\n"
        "  llm_provider: openai\n"
        "  llm_model: gpt-4o-mini\n"
        "  llm_temperature: 0.0\n"
        "  max_steps: 20\n"
        "actions:\n"
        "  turn_small_deg: 30.0\n"
        "  turn_large_deg: 60.0\n"
        "  search_deg: 45.0\n"
        "  forward_short_m: 0.3\n"
        "  forward_medium_m: 0.6\n"
    )
    agent_p, action_p = load_params(_write(tmp_path, yaml_text))
    assert agent_p == AgentParams()
    assert action_p == ActionParams()


def test_overrides_propagate(tmp_path):
    yaml_text = (
        "agent:\n"
        "  llm_model: gpt-4o\n"
        "  max_steps: 5\n"
        "actions:\n"
        "  turn_small_deg: 20.0\n"
    )
    agent_p, action_p = load_params(_write(tmp_path, yaml_text))
    assert agent_p.llm_model == "gpt-4o"
    assert agent_p.max_steps == 5
    # Unspecified fields keep defaults.
    assert agent_p.llm_provider == "openai"
    assert action_p.turn_small_deg == 20.0
    assert action_p.turn_large_deg == 60.0


def test_missing_file_raises_filenotfound(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_params(tmp_path / "no_such.yaml")
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `pytest tests/test_config_loader.py -v`

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 4: Implement the loader**

Create `agent/config_loader.py`:

```python
"""Load AgentParams + ActionParams from a YAML file.

Unspecified keys fall back to the dataclass defaults, so the YAML can
contain only the values that differ from the defaults.
"""

from dataclasses import fields
from pathlib import Path
from typing import Tuple

import yaml

from agent.params import ActionParams, AgentParams


def _build(cls, data: dict):
    """Construct `cls` using only keys it knows about; others are silently dropped."""
    known = {f.name for f in fields(cls)}
    filtered = {k: v for k, v in data.items() if k in known}
    return cls(**filtered)


def load_params(path: Path) -> Tuple[AgentParams, ActionParams]:
    """Parse `path` and return (AgentParams, ActionParams)."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"params file not found: {path}")
    raw = yaml.safe_load(path.read_text()) or {}
    agent_data = raw.get("agent", {}) or {}
    actions_data = raw.get("actions", {}) or {}
    return _build(AgentParams, agent_data), _build(ActionParams, actions_data)
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_config_loader.py -v`

Expected: All three tests PASS.

- [ ] **Step 6: Commit**

```bash
git add config/params.yaml agent/config_loader.py tests/test_config_loader.py
git commit -m "feat(agent): add YAML config loader and default params.yaml"
```

---

## Task 15: `ChatOpenAI` wiring

**Files:**
- Create: `agent/llm.py`
- Create: `tests/test_llm.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_llm.py`:

```python
import os

import pytest

from agent.llm import build_llm
from agent.params import AgentParams


def test_build_llm_returns_a_chat_model_object(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test-not-real")
    llm = build_llm(AgentParams())
    # ChatOpenAI exposes model_name and temperature attributes; this is a
    # smoke test that the constructor wiring took effect.
    assert getattr(llm, "model_name", None) == "gpt-4o-mini"
    assert getattr(llm, "temperature", None) == 0.0


def test_build_llm_requires_openai_key(monkeypatch):
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    with pytest.raises(RuntimeError, match="OPENAI_API_KEY"):
        build_llm(AgentParams())


def test_build_llm_rejects_unknown_provider(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test-not-real")
    with pytest.raises(ValueError, match="provider"):
        build_llm(AgentParams(llm_provider="some-other-cloud"))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_llm.py -v`

Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement `build_llm`**

Create `agent/llm.py`:

```python
"""Construct the ChatOpenAI instance the reason node will call.

Kept tiny and provider-aware so swapping vendors later is a one-file
change.
"""

import os

from agent.params import AgentParams


def build_llm(params: AgentParams):
    """Return a tool-bindable chat model configured per AgentParams."""
    if params.llm_provider != "openai":
        raise ValueError(
            f"unsupported llm_provider {params.llm_provider!r}; only 'openai' is wired"
        )
    if not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError(
            "OPENAI_API_KEY environment variable is required for the openai provider"
        )
    # Imported here so test environments without langchain-openai installed
    # can still import agent.llm to inspect errors.
    from langchain_openai import ChatOpenAI

    return ChatOpenAI(
        model=params.llm_model,
        temperature=params.llm_temperature,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_llm.py -v`

Expected: All three tests PASS.

- [ ] **Step 5: Commit**

```bash
git add agent/llm.py tests/test_llm.py
git commit -m "feat(agent): add ChatOpenAI builder gated by OPENAI_API_KEY"
```

---

## Task 16: `scripts/run_agent.py` — live-rover entry point

**Files:**
- Create: `scripts/run_agent.py`

This script is hardware-only. No unit test; verification is the manual on-rover run in Step 2.

- [ ] **Step 1: Create the script**

Create `scripts/run_agent.py`:

```python
"""Run the LangGraph agent on the live rover.

Usage:
    python scripts/run_agent.py "drive to the water bottle"

Requires:
    - safety_controller_node running (provides /execute_command action).
    - LIMO base drivers running (so /cmd_vel actually moves the rover).
    - emergency-braking gate running.
    - OPENAI_API_KEY exported.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from agent.command_executor import execute_command  # noqa: E402
from agent.config_loader import load_params  # noqa: E402
from agent.graph import build_graph  # noqa: E402
from agent.llm import build_llm  # noqa: E402
from perception.moondream_client import Moondream  # noqa: E402
from perception.vision_tool import (  # noqa: E402
    capture_and_analyze as _capture_and_analyze,
    ros_capture_fn,
    ros_depth_capture_fn,
)


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: run_agent.py <natural-language task>", file=sys.stderr)
        return 2
    task = " ".join(sys.argv[1:])

    agent_params, action_params = load_params(REPO_ROOT / "config" / "params.yaml")
    llm = build_llm(agent_params)

    moondream = Moondream()

    def capture_and_analyze(target: str):
        # The vision tool's signature is capture_and_analyze(target, *, capture_fn,
        # moondream, depth_fn). Bind hardware-specific args here.
        return _capture_and_analyze(
            target,
            capture_fn=ros_capture_fn,
            moondream=moondream,
            depth_fn=ros_depth_capture_fn,
        )

    graph = build_graph(
        llm=llm,
        execute_command=execute_command,
        capture_and_analyze=capture_and_analyze,
        agent_params=agent_params,
        action_params=action_params,
    )

    final = graph.invoke({"task": task})

    print(f"\n=== Run complete ===")
    print(f"status:         {final['status']}")
    print(f"status_message: {final['status_message']}")
    print(f"steps:          {final['step_count']}")
    print(f"target:         {final['target']}")
    return 0 if final["status"] == "arrived" else 1


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Verify on the rover (manual)**

On the rover, with `safety_controller_node`, AEB gate, and the LIMO base drivers all up, and `OPENAI_API_KEY` exported:

```bash
python3 scripts/run_agent.py "drive to the water bottle"
```

Expected: rover finds and approaches a water bottle placed in view; ends with `status: arrived`.

- [ ] **Step 3: Commit**

```bash
git add scripts/run_agent.py
git commit -m "feat(agent): add run_agent.py live-rover entry point"
```

---

## Task 17: `scripts/test_agent_static.py` — static-image rehearsal

**Files:**
- Create: `scripts/test_agent_static.py`

Runs the full graph with real Moondream + real GPT but a fake `execute_command`. Useful for tuning prompts without touching the rover.

- [ ] **Step 1: Create the script**

Create `scripts/test_agent_static.py`:

```python
"""Static-image rehearsal for the LangGraph agent.

Replaces ros_capture_fn with a sequence of PIL images loaded from disk so
the agent can be exercised end-to-end on a laptop. Real Moondream2, real
OpenAI; fake execute_command that just logs the move it would have made.

Usage:
    python scripts/test_agent_static.py path/to/frame1.jpg path/to/frame2.jpg ...

Each `look` call consumes the next frame in order; once exhausted, the
last frame repeats.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from PIL import Image  # noqa: E402

from agent.command_executor import ExecuteResult  # noqa: E402
from agent.config_loader import load_params  # noqa: E402
from agent.graph import build_graph  # noqa: E402
from agent.llm import build_llm  # noqa: E402
from perception.moondream_client import Moondream  # noqa: E402
from perception.vision_tool import capture_and_analyze as _capture_and_analyze  # noqa: E402


def main() -> int:
    if len(sys.argv) < 2:
        print(
            "usage: test_agent_static.py <task> <frame1> [<frame2> ...]",
            file=sys.stderr,
        )
        return 2
    task = sys.argv[1]
    frame_paths = [Path(p) for p in sys.argv[2:]]
    frames = [Image.open(p).convert("RGB") for p in frame_paths]
    if not frames:
        print("at least one frame is required", file=sys.stderr)
        return 2

    cursor = {"i": 0}

    def stub_capture_fn():
        i = min(cursor["i"], len(frames) - 1)
        cursor["i"] += 1
        return frames[i]

    moondream = Moondream()

    def capture_and_analyze(target: str):
        return _capture_and_analyze(
            target, capture_fn=stub_capture_fn, moondream=moondream,
        )

    def fake_execute(heading_deg: float, distance_m: float) -> ExecuteResult:
        print(f"  [fake move] heading={heading_deg:+.1f}deg, distance={distance_m:.2f}m")
        return ExecuteResult(True, "ok")

    agent_params, action_params = load_params(REPO_ROOT / "config" / "params.yaml")
    llm = build_llm(agent_params)

    graph = build_graph(
        llm=llm,
        execute_command=fake_execute,
        capture_and_analyze=capture_and_analyze,
        agent_params=agent_params,
        action_params=action_params,
    )

    final = graph.invoke({"task": task})

    print(f"\n=== Rehearsal complete ===")
    print(f"status:         {final['status']}")
    print(f"status_message: {final['status_message']}")
    print(f"steps:          {final['step_count']}")
    return 0 if final["status"] == "arrived" else 1


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 2: Smoke-run with a single frame (manual)**

With `OPENAI_API_KEY` set and a `.jpg` of a water bottle in `~/frames/bottle.jpg`:

```bash
python scripts/test_agent_static.py "drive to the water bottle" ~/frames/bottle.jpg
```

Expected: terminal logs a sequence of fake moves and ends with `status: arrived` (assuming the frame shows the bottle close and centered). If the frame shows it further away or off-center, the agent should issue plausible turns/forwards.

- [ ] **Step 3: Commit**

```bash
git add scripts/test_agent_static.py
git commit -m "feat(agent): add static-image rehearsal script for prompt tuning"
```

---

## Task 18: README + roadmap update

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update the roadmap section**

In `README.md`, change the Phase 3 roadmap line from:

```
- [ ] Phase 3: LangGraph agent state machine
```

to:

```
- [x] Phase 3: LangGraph agent state machine (OpenAI-backed ReAct loop with look/turn/forward/search/stop tools)
```

- [ ] **Step 2: Add OPENAI_API_KEY to the running instructions**

In `README.md`, in the `Running` section, change the agent terminal example to:

```bash
# Terminal 4 — start the agent
export OPENAI_API_KEY=sk-...
python scripts/run_agent.py "drive to the water bottle"
```

- [ ] **Step 3: Run the full test suite for sanity**

Run: `pytest tests/ -v`

Expected: All tests PASS (skips for the hardware-only paths are acceptable).

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "docs: mark Phase 3 (LangGraph agent) complete in roadmap"
```

---

# Done

After Task 18, the agent is ready for end-to-end demos on the rover. The roadmap items still open after this plan (multi-step tasks, dashboard, voice input) are out of scope; each would be a separate spec.
