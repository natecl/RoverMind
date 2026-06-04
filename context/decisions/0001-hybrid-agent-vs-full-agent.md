# 0001 — Hybrid agent + real-time controller, not full agentic control

- **Status:** Accepted
- **Date:** 2026-05-26 (back-filled from README)

## Context

A VLM call (Moondream2 on the Jetson) takes ~1.5–3 s. If the LLM/agent drove the motors
directly, the rover would travel roughly half a metre **blind** between decisions, at the mercy
of non-deterministic LLM steering and possibly unsafe speeds.

## Decision

Split into two layers: a **VLM agent loop** (~1 Hz) for high-level perception and reasoning, and
a **real-time ROS2 controller** (~10–30 Hz) for low-level motion with speed clamping and timeouts.
The agent issues coarse `heading + distance` commands; the controller executes them safely.

## Consequences

- Safe and smooth during the agent's multi-second reasoning gaps; the controller clamps all output.
- Debuggable along a clean seam — PID/controller plots for motion issues, reasoning logs for agent
  issues — instead of one opaque loop.
- Cost: two layers and an interface (`ExecuteCommand` action) to keep in sync; the agent can't do
  fine continuous control, only discrete moves.
- Implemented in `agent/` (loop) ↔ `safety_controller_layer/` (controller). See `ARCHITECTURE.md`.
