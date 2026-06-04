# 0002 — TCP bridge to span the rover's Python 3.8 and the Mac's 3.10

- **Status:** Accepted
- **Date:** 2026-05-26

## Context

The rover (Jetson, ROS2 Foxy) is pinned to **Python 3.8** with rclpy + torch + Moondream. The
agent stack (LangGraph, langchain-openai) wants **Python 3.10+** and is developed on the Mac.
Running everything on the rover's 3.8 failed on the agent dependencies; running ROS on the Mac
isn't possible. The two halves must talk.

## Decision

Introduce a thin **TCP RPC bridge** (`bridge/`). The rover runs `bridge_server.py` (3.8) exposing
exactly two methods — `execute_command` and `capture_and_analyze` — plus `ping`. The Mac runs
`bridge/client.py` (3.10) which calls them identically to in-process functions. Transport is
length-prefixed JSON (`bridge/wire.py`) over an SSH-tunnelled socket on port 9000. The bridge
**reuses** `agent/command_executor.py` and `perception/vision_tool.py` unchanged.

## Consequences

- The Mac imports **no** ROS or ML; all hardware lives behind the bridge. Clean dependency split.
- The agent graph takes `execute_command` / `capture_and_analyze` as injected callables, so the
  same graph runs in-process (tests, static rehearsal) or bridged (live) with no changes.
- Cost: a deploy step — perception/parsing or controller changes need a **rover repo sync +
  bridge restart**; only `agent/`-side changes are Mac-local. The SSH link is also flaky (RF).
- `SceneObservation` / `ExecuteResult` must stay JSON-serializable (see `bridge/wire.py`).
