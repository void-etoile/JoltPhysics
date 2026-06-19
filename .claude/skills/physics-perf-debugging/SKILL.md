---
name: physics-perf-debugging
description: >-
  Profile-driven workflow for debugging high-performance real-time physics and
  simulation code (game physics engines, rigid-body/constraint solvers, cloth,
  particles, destruction, large agent sims — e.g. Jolt, PhysX, Box2D, Bullet, or
  bespoke solvers). Use this whenever the task is a per-frame spike or frame-time
  regression, a solver blow-up (bodies flying / jitter / NaNs / "can't settle"),
  or simulation behaviour that "looks wrong" (collapses, ragdolls, vehicles,
  fluids behaving unrealistically) and you need to find the cause and fix it
  without destabilising the sim. Trigger it even when the user just says "this is
  slow", "it spikes", "it explodes", "it looks wrong", or "why is it doing that"
  in a physics/simulation context, before reaching for a code change.
---

# Profile-driven physics & simulation debugging

Real-time physics is a tightly-coupled emergent system: one number changes how
the solver, contacts, sleeping, and your own per-frame passes interact, often
non-locally and only under load. That makes it the worst possible domain for
guessing. The single biggest predictor of progress is **measuring before
changing** and **changing one thing at a time**. This skill encodes that loop
and the domain rules of thumb that repeatedly matter.

Use it for both flavours of problem — they share the same loop:
- **Performance**: per-frame spikes, frame-time regressions, "it's slow".
- **Behaviour/stability**: solver explosions (bodies flying, jitter, NaNs,
  "can't lose velocity"), or simulation that "looks wrong".

## The loop

1. **Reproduce and characterise.** Get a reliable repro and the *simplest* one
   that still shows it. A minimal repro (one building, not the whole city; one
   joint, not the scene) collapses the search space and is the difference
   between "it looks wrong" and a one-line diagnosis.
2. **Measure — do not guess.** Profile, or add *attributed* instrumentation
   (see below), until the data names the responsible subsystem/pass. Resist the
   urge to fix what you *think* it is. In practice, every hypothesis formed
   without data has a high chance of being wrong; every fix applied to a
   confirmed cause tends to land.
3. **Form one hypothesis and confirm it with a targeted probe.** Don't fix on a
   plausible story — add the cheap measurement that would distinguish your
   hypothesis from the alternatives, and read it. (Is that body *moving* or
   *asleep*? Is that pass's cost the *work* or a *rebuild* folded into its timer?)
4. **Make the minimal change at the confirmed cause.** One variable. Prefer the
   smallest change that addresses the root, not a compensating tweak elsewhere.
5. **Verify in the running app, not just the build.** Behaviour bugs only show
   under simulation. Re-run the repro and read the same instrumentation: did the
   signature you targeted actually drop/disappear? Did anything else move?
6. **Revert fast.** If a change regresses (especially a stability regression),
   restore the last known-good state *immediately* and re-diagnose — do not
   stack a second fix on top of a suspect one. A worse regression than the
   original (e.g. trading "looks wrong" for "solver explodes") is a hard stop.
7. **Record the finding.** Note the cause, the fix, and any dead end so the next
   person (or session) doesn't repeat the search or the dead end.

The times this loop is violated — guessing a cause, or fixing two things at
once — are the times you go backwards. Treat steps 2 and 3 as non-negotiable.

## Attributed instrumentation

Generic counters ("N fractures this frame") tell you *that* something happened,
not *why* or *which*. The fast path to a diagnosis is instrumentation that
**attributes** cost/events to a specific pass, source, and category. Build it
*before* guessing — it is cheaper than one wrong fix.

Patterns that pay off:

- **Per-sub-pass timers in the hot function.** Bracket each phase with timestamps
  and print a one-line breakdown on frames over a threshold:
  `[SPIKE] total=6.5 deform=2.0 cascade=2.1 [scope=.. graphA=.. graphB=..] other=2.4`.
  This localises a spike to a sub-pass in one run instead of a profiling session.
- **Fold hidden costs into the breakdown.** If a shared helper (a rebuild, a
  cache miss) can fire inside several passes, give it its *own* accumulator
  (`rebuildMs xN`) so it doesn't masquerade as the pass that happened to trigger
  it. A cost that "floats" between passes across runs is the tell for this.
- **Per-event attribution.** When an event (a break, a spawn, a wake) can come
  from several sources, log per frame *which source* fired it and *what class*
  of object: `[EVENT] via=passB classA=3 classB=1 (max metric=..)`. Dominance of
  one source/class is usually the whole diagnosis.
- **Probe the discriminating state, not a proxy.** When two explanations predict
  the same outcome, log the field that separates them. ("Was the body *asleep*
  or *awake-but-slow* when we acted on it?" decides between "needs a wake" and
  "needs a sustained-state gate".)
- **Use the engine's profiler for the baseline.** Many engines can dump a
  per-frame flame chart (Jolt → `Build/.../profile_chart_*.html`); if yours
  can't, your per-sub-pass timers above are the substitute. Convert cycles→time once
  and read **self-time** (subtract children) to find where time truly goes;
  parent scopes mislead. Confirm a capture is the scenario you mean (mtime, and
  the active-object counts in it) before trusting it.

Keep this instrumentation in-tree behind a cheap threshold/printf while
iterating; it is the asset that makes each round fast. Retire it (or gate it
behind a debug flag) once the area is stable.

## High-performance physics rules of thumb

These recur across engines. Most "spikes" are really "too much was *active* this
frame"; most "explosions" are really "the solver was handed an impossible
configuration".

**Cost / scaling**
- **Let settled state sleep, and keep it out of the moving broadphase.** A
  standing/idle structure should cost ~0. Put would-be-static geometry in a
  non-moving broadphase layer; tune sleep thresholds so settled islands actually
  sleep. The cheapest work is work that doesn't run.
- **Scope per-frame work to the affected region, not the whole scene.** Graph
  passes, reachability, neighbour queries — seed them from what changed and walk
  the connected component, so idle frames cost 0 and busy frames cost
  O(affected) not O(scene). Gate the whole pass on "did anything change".
- **Never rebuild a whole-scene structure each frame; maintain it
  incrementally.** A graph/index/adjacency that's rebuilt whenever *anything*
  changes is a classic hidden spike (and can run several times per frame). Keep
  a live structure updated on add/remove instead. Beware index-into-array
  snapshots: swap-and-pop invalidates indices and forces a full rebuild — store
  stable handles/refs.
- **Batch insert/remove.** Adding/removing bodies or constraints one at a time
  often degrades the broadphase or re-sorts islands repeatedly; use the engine's
  batch API.
- **Read self-time, parallelism, and counts together.** Summed worker time isn't
  wall time; a number that looks huge may be parallel. An active frame is usually
  dominated by core constraint+contact solving, which you reduce by shrinking the
  active set, not by micro-optimising your own passes (which are often already
  negligible — confirm with the profiler before optimising them).

**Stability (avoid handing the solver the impossible)**
- **Never accumulate interpenetrating constrained rigid bodies.** Hundreds of
  solid, overlapping, still-constrained bodies make the solver explode (bodies
  flying, unable to settle). If you must keep many freed pieces, make them
  unconstrained and in a layer that collides only with static geometry, so they
  can't shove each other or the structure.
- **Stiff constraints live in a narrow stability band.** At a fixed timestep,
  pushing constraint stiffness too high oscillates/explodes; reading break/state
  decisions off the *transient* state of a stiff constraint reads noise. Prefer
  position-based / sustained criteria over instantaneous velocity for
  irreversible decisions.
- **Watch for one-frame transients at events.** When a body wakes, a constraint
  re-engages, or support is removed, there's a one-frame burst (velocity spike,
  or a body still at its pre-event state). An **instantaneous** gate evaluated on
  that frame misfires — it sees a body that's "asleep / at rest" the very frame
  its support vanished, or a velocity burst that isn't real motion. Use a
  **sustained-state gate** (a settle/yield timer: require the condition to hold
  for N frames, reset when violated) so transients are filtered and genuine
  states still trigger. This was the difference between "disintegrates straight
  down on impact" and "leans and topples".
- **Prefer breaking connections over deleting/fracturing in place** when you want
  things to move as coherent chunks rather than vanish or shatter unnaturally.

## Anti-patterns (hard-won)

- **Guessing the cause from a plausible story.** Repeatedly wrong. Add the probe.
- **Fixing two things at once**, or stacking a fix on an unverified change — you
  lose the ability to attribute the result.
- **Reaching for a global knob** (raise iterations, raise a threshold, raise a
  cap) before locating the cause. It usually trades one artifact for another.
- **An instantaneous test where the trigger fires the same frame as the event.**
  Use a sustained-state timer instead.
- **Keeping un-fractured/un-removed bodies solid and constrained to dodge a
  visual** — it detonates the solver under load. (A real dead end from this repo.)

## Verifying behaviour

A physics behaviour fix is only proven by watching the sim. Build, run the repro,
and read the same instrumentation you added — confirm the targeted signature
dropped and nothing new appeared. If the engine has a samples/test harness,
prefer it: a known build command + a specific test/scene, with the diagnostic
printfs visible. Treat "it builds" as necessary, never sufficient.

## Project specifics

For repo-specific build/run commands, layer setup, profiler paths, and the
hard-won destruction-behaviour lessons, see `CLAUDE.md` (Performance Notes and
Destruction Behaviour Notes sections) and any saved memory — read those before
diagnosing here so you start from the known state rather than rediscovering it.
