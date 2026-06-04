# context/ — RoverMind's AI-native context system

This directory and the `CLAUDE.md` files throughout the repo exist so that an agent
(or human) can understand RoverMind **without reverse-engineering it every time**, and
so that understanding **stays fresh instead of rotting**.

## How the system is laid out (two layers)

1. **Lazy per-module context** — `CLAUDE.md` lives in each major directory
   (`agent/`, `bridge/`, `perception/`, `safety_controller_layer/`) and at the repo root.
   Claude Code loads the **root one every session** and a **nested one only when you touch
   files in that subtree**. That lazy loading is the point: the always-on context window
   stays small, and the deep detail loads only when it's relevant.

2. **This cross-cutting hub** — `context/` holds the facts that don't belong to any single
   module:
   - `ARCHITECTURE.md` — how the pieces fit: data flow, ROS node/topic table, the two boundaries.
   - `GLOSSARY.md` — the shared vocabulary (`SceneObservation`, `RoverState`, buckets, AEB…).
   - `ERRORS.md` — the error taxonomy: what can fail, where, and how to diagnose it.
   - `ENVIRONMENT.md` — rover identity + network profiles + how to find the rover on the current network.
   - `decisions/` — Architecture Decision Records (the *why* behind non-obvious, costly-to-reverse choices).

The root `CLAUDE.md` is the front door — start there.

## The maintenance contract (this is what keeps it from rotting)

Capturing context is not "write more docs." A stale or duplicated doc is **worse than none**.
The repo ships a `/learn` skill that does this correctly — dedup, route to one layer, prune.

- **When you finish a piece of work**, run **`/learn`**. A `Stop` hook reminds you whenever a
  turn ends with uncommitted changes.
- **Route each durable learning to exactly one place:**
  - a convention/navigation fact for a module → that module's `CLAUDE.md`
  - a non-obvious, costly-to-reverse decision → a new ADR in `context/decisions/`
  - a cross-cutting fact/preference/constraint → memory (`/memory`)
- **Dedup before writing** — if the fact exists, update it in place; never add a second copy.
- **Prune on contradiction** — if this session proved a doc wrong, fix it (or supersede the ADR) now.
- **The code is the source of truth.** If a doc here disagrees with the code, the code wins —
  fix the doc. Every doc names the files it describes so you can check it quickly.

## Freshness

Operational docs that drift (rover IP, bring-up steps) carry a `> Last verified: <date>` header.
Treat anything well past its verified date as suspect until re-checked against reality.
