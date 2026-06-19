---
name: review-diff
description: >-
  Fresh-eyes review of the current uncommitted working changes (git diff) before
  they are committed, with a physics/simulation-aware checklist on top of the
  usual correctness and cleanup review. Use when the user asks to "review my
  changes / my diff", "check this before I commit", "look over what I changed",
  or otherwise wants a second opinion on work-in-progress — especially when the
  diff touches the physics/simulation/destruction code, where a subtle change can
  destabilise the solver or per-frame cost in ways that only show under load.
  Primarily invoked explicitly as /review-diff.
---

# Review the working diff (fresh eyes)

The point of this skill is a review that is **not anchored to why the code was
written the way it was**. The agent that just wrote a change is biased toward its
own rationale and blind to its own assumptions, so a review done in the same head
mostly re-confirms the implementation. Get genuine distance, then report ranked
findings — don't silently rewrite.

## 1. Gather the changes

Collect the full picture of what's uncommitted:

```sh
git status --short
git diff            # unstaged
git diff --cached   # staged
```

If the user names a base (e.g. "vs master"), use `git diff master...` instead.
Note which files changed and roughly what the change is *trying* to do — but
treat that intent as a claim to test, not a fact.

## 2. Review with fresh eyes

Prefer to spawn a **subagent** (Explore/general-purpose) given *only* the diff
and the checklist below — not the conversation that produced it. That removes the
anchoring bias and is the whole reason this skill exists. Ask it to return ranked
findings, not prose. Fall back to an inline review only if subagents are
unavailable; if inline, deliberately re-derive whether each change is correct
from the code as written, ignoring your memory of why you wrote it.

For the general correctness + reuse/simplification/efficiency pass, the built-in
**`code-review`** skill is the engine — use it. This skill adds the physics
checklist and the fresh-eyes discipline on top; don't duplicate the generic pass.

## 3. Physics / simulation checklist

Apply these when the diff touches the engine, simulation, or destruction code.
They are the failure modes that have actually bitten this codebase (see
`CLAUDE.md` and the `physics-perf-debugging` skill):

- **Solver stability.** Does the change risk handing the solver an impossible
  configuration? Watch for: many solid, overlapping, *still-constrained* bodies
  retained instead of removed/freed; constraint stiffness pushed up; freed pieces
  left in a layer that lets them shove structure. These manifest as bodies flying
  / unable to settle, only under load.
- **Instantaneous vs sustained decisions.** Any new gate/threshold that fires an
  *irreversible* action (break, fracture, detach) off a single-frame value — is
  it robust to the one-frame transient at wake / re-engage / support-removal? A
  sustained-state timer is usually correct where an instantaneous test is not.
- **Per-frame cost & scaling.** Does it add work that runs every frame regardless
  of activity? Does it rebuild a whole-scene structure when an incremental update
  would do? Is a graph/region pass scoped to what changed, or O(scene)? Does
  settled state still sleep / stay out of the moving broadphase?
- **Index/handle stability.** New code indexing into arrays that use swap-and-pop
  removal — are the indices invalidated on removal? Prefer stable handles/refs.
- **Batching.** Bodies/constraints added or removed one at a time in a loop where
  a batch API exists?
- **Layers & groups.** Are new bodies in the correct object/broadphase layer and
  collision group, so they collide with what they should and nothing they
  shouldn't (and don't self-collide within a structure)?
- **Determinism / units.** If the project cares about determinism, does the change
  introduce non-deterministic ordering or FP-order dependence? Are thresholds in
  sensible units (and mass/size-invariant where they should be)?

## 4. Also the usual

- **Correctness**: off-by-one, lifetime/ownership, null/empty, error paths,
  concurrency (anything read on the physics thread vs written on main).
- **Conventions**: match `CLAUDE.md` (naming, `.find()!=.end()` over `.count()`,
  `inArg` types, no RTTI/exceptions, header include order).
- **Reuse/simplification**: duplicated logic, a helper that already exists, dead
  code, leftover debug/printf that wasn't meant to ship.

## 5. Report

Return findings **ranked by severity**, each with `file:line`, what's wrong, and
why it matters — most important first. Separate "bugs / will break" from
"should fix" from "optional/nits". Do **not** auto-apply fixes; this is a review.
If the user then says to fix something, do it as a normal edit. If the diff is
clean, say so plainly rather than inventing nitpicks.
