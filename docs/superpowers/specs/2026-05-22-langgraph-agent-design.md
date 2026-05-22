# LangGraph Agent — Design Spec

> **Status:** approved design, ready for implementation planning.
> **Date:** 2026-05-22

## Goal

Build the RoverMind LangGraph agent: a stateful tool-using agent that takes a
natural-language driving command ("drive to the water bottle") and
autonomously steers the LIMO Pro to the target by interleaving perception
calls and movement commands.

This is **Phase 3 (LangGraph Agent)** of the project roadmap. It depends on:

- Phase 1 — `safety_controller_layer` (ROS2 `ExecuteCommand` action server,
  already implemented).
- Phase 2 — `capture_and_analyze` vision tool returning a structured
  `SceneObservation` (already implemented).

The agent is the glue: it owns the reasoning loop and the conversation with
the LLM, and routes the LLM's decisions to the controller.

## Context

The repo already contains:

- `safety_controller_layer/` — `ExecuteCommand` action server, AEB velocity
  gate.
- `safety_controller_layer_interfaces/` — ROS2 action interface package.
- `perception/` — `capture_and_analyze`, `SceneObservation`, depth math.
- `tests/` — pure-logic unit tests; matches the existing repo pattern of
  pure-Python decision code with thin ROS wrappers.

There is no `agent/` directory yet. The README's `limo_vlm_agent/agent/`
layout is aspirational and is created by this spec.

## Brain choice — OpenAI GPT, not Claude or local

The LangGraph agent's "reasoner" is a cloud LLM (OpenAI GPT), not Moondream2
and not a deterministic policy. Reasons:

- A tool-calling LLM cleanly composes the bucket vocabulary the vision tool
  emits (left/center/right, close/medium/far) into multi-step plans
  ("turn left small, then forward medium, then look again").
- Moondream2 is a perception model, not a tool-calling reasoner — it stays in
  its lane as the "eyes."
- A local tool-calling LLM (e.g. Qwen2.5-3B) on the same Jetson Orin would
  share memory with Moondream2 and risks tool-call reliability issues on a
  small model.

OpenAI is preferred over Anthropic for this project; the integration is
abstracted behind LangChain's chat-model interface so swapping providers is a
config change.

## Action space — discrete buckets, two primitives

The vision tool returns *discrete* observations
(`direction ∈ {left, center, right}`, `distance ∈ {close, medium, far}`).
Continuous numeric arguments give the LLM no real expressive power over
discrete observations, and small LLMs are unreliable at numeric arguments.
Therefore the agent exposes a discrete action set built from two composable
primitives plus search/stop:

| Tool | Args | Maps to |
|---|---|---|
| `look(target)` | `target: str` | `capture_and_analyze(target)` → `SceneObservation`, returned as a short string |
| `turn(direction, magnitude)` | `direction ∈ {left, right}`, `magnitude ∈ {small, large}` | `execute_command(±30° or ±60°, 0)` |
| `forward(distance)` | `distance ∈ {short, medium}` | `execute_command(0, 0.3 m or 0.6 m)` |
| `search()` | — | `execute_command(45°, 0)` (single in-place rotation) |
| `stop(reason)` | `reason: str` | Sets terminal status `"arrived"`, logs reason |

Concrete (heading, distance) values come from `config/params.yaml` so the
mapping is tunable without code changes.

`look` and `turn` are intentionally separate (not collapsed into one
`move(turn, forward)` action) so the LLM can interleave looking and turning
at low risk — the rover never drives forward without a fresh observation.

## Termination — LLM owns `stop`, graph backstops

The LLM decides when to stop via the `stop` tool. The `SceneObservation` does
include a derived `should_stop` field (close + found), but the graph does
**not** short-circuit on it; the LLM is responsible for calling `stop` when
it sees that signal, which keeps the LLM in control of the demo's success
criterion.

The graph still enforces deterministic backstops:

- `step_count >= max_steps` → terminate with `status="failed_max_steps"`.
- LLM API error after one retry → terminate with `status="aborted"`.

## Observation — pure ReAct (`look` is a tool, no forced-observe node)

The agent uses the standard ReAct pattern: the LLM owns when to look. The
graph does not auto-call `capture_and_analyze` between turns. This matches
the project README's tool-based framing and is the most natural LangGraph
idiom. The cost is that the LLM could in principle act blind by skipping
`look`; the system prompt explicitly instructs "Always call look between
movements. Never act blind."

## Architecture

```
START
  │
  ▼
┌──────────┐
│   init   │   (extract target, seed messages, status="running")
└────┬─────┘
     ▼
┌──────────┐ ◄────────────────┐
│  reason  │  (GPT + 5 tools) │
└────┬─────┘                  │
     ▼                        │
┌──────────┐                  │
│   act    │                  │
└────┬─────┘                  │
     ▼                        │
┌──────────┐ status="running" │
│  check   │──────────────────┘
└────┬─────┘
      status terminal OR step_count >= max_steps
     ▼
    END
```

## State schema

```python
from typing import Annotated, Literal, Optional, TypedDict
from langchain_core.messages import BaseMessage
from langgraph.graph.message import add_messages
from perception.scene_parsing import SceneObservation

class RoverState(TypedDict):
    # Full chat history with the LLM: system prompt, user task,
    # each look-result summary, each assistant tool call, each tool result.
    # The add_messages reducer means nodes only return NEW messages.
    messages: Annotated[list[BaseMessage], add_messages]

    # Original task and target extracted from it. Set by `init`; never mutated.
    task: str
    target: str

    # Most recent structured observation. Written by the `act` node when the
    # LLM calls `look`. Optional because no `look` has been called between
    # `init` and the first `reason` call.
    last_observation: Optional[SceneObservation]

    # Backstop counter. Incremented in `check`.
    step_count: int

    # Terminal status. Stays "running" until graph ends.
    status: Literal["running", "arrived", "failed_max_steps", "aborted"]
    status_message: str
```

## Nodes

| Node | Type | Responsibility |
|---|---|---|
| `init` | pure Python | Extract `target` from `task` (regex on "drive to the ..."). Seed `messages` with system prompt + user task. `step_count=0`, `status="running"`. Runs once. |
| `reason` | LLM call | Invoke the chat model with current `messages` + the 5 bound tools. `tool_choice="required"`, `parallel_tool_calls=False`, `temperature=0`. Returns an `AIMessage` carrying exactly one tool call. |
| `act` | side-effecting | Dispatch the tool call on the latest `AIMessage`. `look` writes `last_observation`. `stop` sets `status="arrived"` and `status_message`. Append a `ToolMessage` with the tool's return string. |
| `check` | pure Python | `step_count += 1`. If `status != "running"` → END. If `step_count >= max_steps` → set `status="failed_max_steps"`, END. Else → back to `reason`. |

## Edges

- `START → init → reason → act → check` are all unconditional.
- One conditional edge from `check`: terminal status or step cap → `END`,
  otherwise → `reason`.
- `check` is the single termination point. Even when the LLM calls `stop`,
  the END transition happens in `check` for consistency — `act` only sets
  the status flag.

## Tool surface (LLM-visible)

```python
@tool
def look(target: str) -> str:
    """Look at the world and report where `target` is and how far away it is.
    Call this before every move so you know what to do next."""

@tool
def turn(direction: Literal["left", "right"],
         magnitude: Literal["small", "large"]) -> str:
    """Turn the rover in place. small ~ 30 deg, large ~ 60 deg."""

@tool
def forward(distance: Literal["short", "medium"]) -> str:
    """Drive forward. short ~ 0.3 m, medium ~ 0.6 m. Do not use if the
    target is far to the side — turn toward it first."""

@tool
def search() -> str:
    """Rotate ~45 deg in place to look for the target. Use when look()
    reports the target is not found."""

@tool
def stop(reason: str) -> str:
    """Stop the rover and end the task. Call this once you are close to and
    centered on the target. `reason` is logged for debugging."""
```

Tool returns are short human-readable strings appended as `ToolMessage`
content for the next LLM turn:

- `look` → `"target found at left, medium"` / `"target not found"` /
  `"look failed: no camera frame"`.
- `turn` / `forward` / `search` → `"turn complete: rotated 30 deg left"` etc.
- `stop` → `"stop acknowledged"`.

## System prompt

Lives in `agent/prompts.py`. Initial version:

```
You control a rover with five tools: look, turn, forward, search, stop.
Vision reports direction as {left, center, right} and distance as {close,
medium, far}.

Strategy:
1. Call look(target) to find out where the target is.
2. If not found, call search() and look again.
3. If found but not centered, turn toward it (left -> turn left,
   right -> turn right).
4. If centered but not close, forward.
5. When the target is centered AND close, call stop("arrived").

Always call look between movements. Never act blind. One tool per turn.
```

The prompt is tunable in Phase 4; the spec only fixes the contract (tools,
vocabulary, strategy outline).

## Error handling

All tool failures become strings the LLM reads. Only LLM API failure
terminates the graph.

| Failure | Where | What happens |
|---|---|---|
| `capture_and_analyze` raises `FrameCaptureError` | inside `look` tool | Tool returns `"look failed: no camera frame"`. LLM retries or searches. |
| Moondream2 returns unparseable answer | already handled by `scene_parsing.py` | `look` returns `"target not found"` or `"target found, position unclear"`. |
| ROS2 `execute_command` action fails or aborts | inside `turn`/`forward`/`search` | Tool returns `"move failed: <reason>"`. LLM can stop or re-look. |
| LLM API error | `reason` node | One retry with backoff; if still failing, `status="aborted"`, END. |
| LLM never calls `stop` | `check` node | `max_steps` backstop fires. |
| Rover physically blocked | handled upstream by AEB | `forward` returns success but scene unchanged on next `look`; LLM eventually turns, stops, or hits `max_steps`. |

## Configuration

New entries in `config/params.yaml`:

```yaml
agent:
  llm_provider: openai
  llm_model: gpt-4o-mini
  llm_temperature: 0
  max_steps: 20

actions:
  turn_small_deg: 30
  turn_large_deg: 60
  search_deg: 45
  forward_short_m: 0.3
  forward_medium_m: 0.6
```

OpenAI key comes from `OPENAI_API_KEY` env var; not committed.

## File layout

```
agent/
├── __init__.py
├── command_executor.py   # sync wrapper around ROS2 ExecuteCommand action client
├── tools.py              # @tool functions: look, turn, forward, search, stop
├── prompts.py            # system prompt string
├── state.py              # RoverState TypedDict
├── nodes.py              # init, reason, act, check
└── graph.py              # build_graph(): wires nodes + edges
scripts/
├── run_agent.py          # entry point: `python scripts/run_agent.py "drive to the water bottle"`
└── test_agent_static.py  # static-image rehearsal: real GPT + real Moondream + fake execute_command
tests/
├── test_command_executor.py   # fake action client → assert goal construction
├── test_tools.py              # fake execute_command, fake capture_and_analyze
├── test_graph_with_mocked_llm.py  # scripted tool calls → assert state transitions
└── test_graph_termination.py  # max_steps, stop, error paths
```

## Build chunks (vertical slices)

The implementation is split into four end-to-end runnable chunks. Each chunk
proves one capability before the next begins.

### Chunk A — `command_executor.py` (no LangGraph, no LLM)

A sync Python wrapper around the ROS2 `ExecuteCommand` action client:

```python
def execute_command(heading_deg: float, distance_m: float) -> ExecuteResult: ...
```

Unit-testable with a fake action client. Verified on the rover with hardcoded
calls.

### Chunk B — `tools.py` (pure functions, no LLM)

The five `@tool`-decorated functions, each resolving bucket vocabulary to a
single `execute_command` call (or `capture_and_analyze` for `look`).
Unit-testable with injected fakes; proves the entire vocabulary-to-motion
layer without any LLM in the loop.

### Chunk C — LangGraph with mocked LLM

Wire up `state.py`, `nodes.py`, `graph.py` and test with a scripted fake LLM
that returns a deterministic sequence of tool calls. Verifies graph wiring,
state updates, termination conditions, and the `max_steps` backstop.

### Chunk D — Real GPT + static-image rehearsal + on-rover demo

Swap in `ChatOpenAI`. `scripts/test_agent_static.py` runs the full graph
against a recorded sequence of camera frames (real Moondream, real GPT, fake
`execute_command`). Final end-to-end run on the live rover closes Phase 3.

## Testing strategy (TDD per chunk)

- **Chunk A** — unit tests against a fake action client; on-rover hardcoded
  command verification.
- **Chunk B** — unit tests with injected fakes for `execute_command` and
  `capture_and_analyze`. Assert each tool resolves to the correct call.
- **Chunk C** — unit tests with a scripted fake LLM. Cover the happy path
  (look → turn → look → forward → look → stop), the `max_steps` backstop,
  the LLM-never-stops path, and tool-failure-as-string paths.
- **Chunk D** — integration test via `scripts/test_agent_static.py` over
  recorded frames; final on-rover end-to-end demo.

## Out of scope

- Multi-step tasks ("go to X, then come back").
- Obstacle awareness beyond the existing AEB velocity gate.
- Real-time web dashboard.
- Voice command input.
- Replacing Moondream2 with a different VLM.
- Changing the `safety_controller_layer` action interface.

## Open items deferred to implementation planning

- Exact regex for `target` extraction in `init` (fallback to "the whole task
  string as target" if no match).
- Whether `run_agent.py` should spin up an rclpy node itself or assume one is
  running.
- Logging format for reasoning chain (structured JSON vs plain text).
