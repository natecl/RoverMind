# RoverMind — agent orientation

**What this is:** a LIMO Agilex Pro rover that executes natural-language driving commands
("drive to the water bottle"). A LangGraph VLM agent loop reasons on the **Mac**; a real-time
ROS2 controller drives the **rover**; the two talk over a TCP bridge.

This file loads every session — it's deliberately short. Drill into `context/` and the per-module
`CLAUDE.md` files (which load lazily when you touch that directory) for detail.

## Map

| Where | What | Read when |
|-------|------|-----------|
| `agent/` | LangGraph loop, tools, prompt (Mac, py3.10) | changing reasoning/tools/params |
| `perception/` | Moondream2 VLM → `SceneObservation` (rover) | changing vision/parsing |
| `bridge/` | TCP RPC across py3.8↔py3.10 (`client`/`server`/`wire`) | changing the rover↔Mac interface |
| `safety_controller_layer/` | motion controller + AEB (ROS2, rover) | changing motion/braking |
| `safety_controller_layer_interfaces/` | `ExecuteCommand.action` definition | changing the action contract |
| `config/params.yaml` | LLM + action tunables | tuning behavior |
| `launch/`, `scripts/` | ROS2 bring-up; `run_agent.py` entry point | running it |

## Where to find context

- **How it fits together** → `context/ARCHITECTURE.md` (data flow, node/topic table, boundaries)
- **Vocabulary** (`SceneObservation`, `RoverState`, buckets, AEB…) → `context/GLOSSARY.md`
- **What can fail & how to diagnose** → `context/ERRORS.md`
- **Connecting to the rover** (MAC, networks, find-the-IP) → `context/ENVIRONMENT.md`
- **Why the design is this way** → `context/decisions/`
- **Running on real hardware** → `AGENT_WORKFLOW.md` (procedure + field notes), `LIMO_WORKFLOW.md` (SSH/bring-up)

## Golden rules (the non-obvious invariants)

1. **AEB is the `/cmd_vel` relay — never run with `use_aeb:=false`.** It's the only thing
   republishing `/cmd_vel_raw → /cmd_vel`; off = no motion. (ADR 0003)
2. **Two Python versions.** Mac = 3.10 agent, **no ROS/ML imports**; rover = 3.8 with rclpy +
   Moondream. Cross only via `bridge/`. (ADR 0002)
3. **Pure/impure split.** Real logic lives in `*_math.py`, `scene_parsing.py`, `control_math.py`
   (pure, laptop-testable). `*_node.py` / `*_server.py` do I/O and need hardware. Add logic to the
   pure layer; inject it into wrappers.
4. **Deploy boundary.** Perception/parsing or controller changes need a **rover sync + bridge
   restart**; `agent/prompts.py` and other Mac-side changes don't.

## Conventions

- **TDD.** Test-first; build in vertical slices. Tests in `tests/`, `test_*.py`, run `pytest` from
  the repo root. Pattern: pure logic unit-tested directly; nodes/servers tested with injected fakes.
- `docs/` and `workflows/` are **gitignored** (local-only plans). `context/` **is** committed.

## Keep this context alive (self-maintaining)

Before you finish work that changed things, run **`/learn`** — it captures durable decisions,
gotchas, and conventions into the right layer (nearest `CLAUDE.md` / `context/decisions/` ADR /
memory), dedups, and prunes contradictions. A `Stop` hook reminds you when a turn ends with
uncommitted changes. **The code is the source of truth** — if a doc disagrees with the code, fix
the doc. See `context/README.md` for the full contract.
