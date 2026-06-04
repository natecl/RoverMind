# Architecture Decision Records (ADRs)

Each ADR captures **one** non-obvious, costly-to-reverse decision and the reasoning behind it,
so a future session inherits the *why* and doesn't relitigate it.

## Format

One file per decision, named `NNNN-short-kebab-title.md`, numbered sequentially. Use the
template below. Keep it short — context, decision, consequences.

```markdown
# NNNN — Title

- **Status:** Accepted | Superseded by [NNNN](NNNN-...md)
- **Date:** YYYY-MM-DD

## Context
What forced a decision? The constraints in play.

## Decision
What we chose.

## Consequences
What this makes easy, what it costs, what to watch out for.
```

## Rules

- **Supersede, don't rewrite.** When a decision changes, mark the old ADR `Superseded by …`
  and write a new one. Don't edit history out.
- **Decisions only.** Conventions/navigation facts go in a `CLAUDE.md`; cross-cutting
  facts go in memory. ADRs are for "we chose X over Y, and reversing it is expensive".
- The `/learn` skill routes here automatically. Update this index when you add an ADR.

## Index

| # | Title | Status |
|---|-------|--------|
| [0001](0001-hybrid-agent-vs-full-agent.md) | Hybrid agent + real-time controller, not full agentic control | Accepted |
| [0002](0002-tcp-bridge-py38-py310.md) | TCP bridge to span the rover's Python 3.8 and the Mac's 3.10 | Accepted |
| [0003](0003-aeb-is-the-cmd-vel-relay.md) | AEB node is the `/cmd_vel` relay (always run it on) | Accepted |
