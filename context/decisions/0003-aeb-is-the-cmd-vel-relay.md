# 0003 — The AEB node is the `/cmd_vel` relay (always run it on)

- **Status:** Accepted
- **Date:** 2026-05-29 (learned during first live bring-up)

## Context

The intuition is that emergency braking is *optional* safety you could disable for debugging
(`use_aeb:=false`). On this rover that intuition is wrong and silently breaks all motion.

## Decision

Treat `aeb_node` as a **mandatory part of the motion path**, not an optional gate. The topic
wiring is: `safety_controller` publishes **only** `/cmd_vel_raw`; `limo_base` subscribes **only**
to `/cmd_vel`; `aeb_node` is the only thing that republishes `/cmd_vel_raw → /cmd_vel`. So with
AEB off, `/cmd_vel` has **0 publishers**, commands never reach the motors, the rover sits still,
and `execute_command` aborts with `"drive_distance did not converge within budget"`.

## Consequences

- **Always launch with AEB on** (no `use_aeb` arg). The `use_aeb:=false` lines in older docs /
  the smoke runbook are wrong for this rover.
- Sanity check before driving: `ros2 topic info /cmd_vel` must show `Publisher count: 1`.
- Recorded in memory as well ([[project_aeb_is_cmd_vel_relay]]). If the topology is ever changed
  so the controller publishes `/cmd_vel` directly, supersede this ADR.
