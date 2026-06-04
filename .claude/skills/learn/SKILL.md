---
name: learn
description: Capture durable learnings from the current session into the project's context architecture (nearest CLAUDE.md, context/decisions ADRs, or memory) with explicit dedup and prune. Use at the end of a work session, or whenever a non-obvious decision, gotcha, convention, or course-reversal occurred that the next session should inherit.
---

# /learn — structured session capture

Turn what this session discovered into durable context for the next one — without
adding rot. Capturing is not "write more docs": the dedup (step 4) and prune (step 6)
steps are what actually keep context healthy. A stale or duplicated doc is worse than none.

Run this as a deliberate pass, not a brain-dump.

## Procedure

1. **Gather what changed.** Look at the session: `git status` and `git diff` for code,
   plus the decisions, gotchas, and dead ends from the conversation. Write yourself a
   raw list of candidate learnings.

2. **Filter durable vs. throwaway.** Keep only what a *future* session benefits from:
   non-obvious decisions, conventions, "we tried X, it failed" lessons, constraints,
   gotchas. Drop anything obvious from the code, one-off, or already in git history.
   When unsure, drop it — over-capture is the failure mode here.

3. **Route each survivor to exactly one layer:**
   - **nearest `CLAUDE.md`** → a convention or navigation fact for a module
     ("how this part works, where things are, the rule to follow").
   - **`context/decisions/` (new ADR)** → a decision + its rationale that's non-obvious and
     costly to reverse. Follow `context/decisions/README.md`; next sequential number.
   - **memory** (`/memory`) → a cross-cutting fact/preference/constraint not tied to one
     module. Follow the memory format and add the `MEMORY.md` pointer.

4. **Dedup before writing.** Search the target layer for the fact first. If it already
   exists, **update it in place** — do not add a second copy. Two docs saying overlapping
   things is the root of context rot.

5. **Write tersely.** Match the existing style of the file you're editing. Link across
   layers (`context/decisions/0001`, `[[memory-slug]]`) instead of copying. Update the ADR
   index (`context/decisions/README.md`) if you added an ADR.

6. **Prune.** Anything this session *contradicted* — a now-wrong line in a `CLAUDE.md`, a
   superseded decision — fix or supersede it now. An ADR is superseded (mark the old one,
   write a new one), not edited; `CLAUDE.md`/memory facts are corrected in place.

7. **Report.** List what you captured, where, and what you pruned. If nothing was durable,
   say so plainly and write nothing — that's a valid outcome.

## Guardrails

- Per the global guidelines, writing files is fine but **deleting/overwriting existing
  content needs the structure above to justify it** — supersede ADRs, correct facts in
  place; don't silently rewrite someone's doc.
- Never invent decisions that weren't made. Capture only what actually happened.
- The code is the source of truth. If a doc disagrees with the code, the code wins.
