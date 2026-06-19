# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

All build scripts are in `Build/`. CMake 3.20+ required. The main `CMakeLists.txt` is at `Build/CMakeLists.txt`.

**Linux/macOS (Clang/GCC, make):**
```sh
cd Build
./cmake_linux_clang_gcc.sh        # generates Linux_Debug, Linux_Release, etc.
cd Linux_Debug
make -j$(nproc)
./UnitTests
```

**Linux (Ninja, multi-config):**
```sh
cd Build && ./cmake_ninja.sh
cd Ninja_MultiConfig
cmake --build . --config Debug
./UnitTests
```

**macOS (Xcode):**
```sh
cd Build && ./cmake_xcode_macos.sh   # opens Xcode
```

**Windows (MSVC):**
```
Build\cmake_vs2026_cl.bat            # generates VS2026_CL\JoltPhysics.sln
```

**Run a single test** (doctest filter):
```sh
./UnitTests --test-case="*BroadPhase*"
```

**Disable CPU instructions** if you hit `Illegal instruction`:
```sh
./cmake_linux_clang_gcc.sh Release clang++ -DUSE_AVX=OFF -DUSE_AVX2=OFF -DUSE_SSE4_1=OFF -DUSE_SSE4_2=OFF
```

## Key CMake Options

| Option | Default | Notes |
|---|---|---|
| `DOUBLE_PRECISION` | OFF | Enable for large worlds (positions as doubles) |
| `CROSS_PLATFORM_DETERMINISTIC` | OFF | Disables FMADD; slower but deterministic |
| `USE_ASSERTS` | OFF | Turns on `JPH_ENABLE_ASSERTS` |
| `OBJECT_LAYER_BITS` | 16 | 16 or 32 bits for ObjectLayer |
| `ENABLE_OBJECT_STREAM` | ON | Serialization support |
| `CPP_RTTI_ENABLED` | OFF | Jolt does not use RTTI |
| `CPP_EXCEPTIONS_ENABLED` | OFF | Jolt does not use exceptions |

Important: always include `Jolt/Jolt.h` **before** any other Jolt header.

## Architecture Overview

```
Jolt/
  Core/         Utilities: Array, HashTable, JobSystem, Memory, RTTI, Mutex, Profiler
  Math/         SIMD types: Vec3/Vec4/Mat44/Quat/DVec3 and scalar equivalents
  Geometry/     AABox, GJK, EPA, convex hull, ray-cast primitives
  AABBTree/     BVH used internally by broad phase
  Physics/      Main simulation engine (see below)
  ObjectStream/ Serialization (binary/text) for shapes and scenes
  Renderer/     Debug drawing abstraction (lines, triangles, text)
  Skeleton/     Skeletal pose/animation helpers for ragdolls
  Compute/      GPU compute abstraction (DX12, Metal, Vulkan, CPU fallback)
  Shaders/      HLSL compute shaders for GPU-side simulation
```

**Physics subsystems** (`Jolt/Physics/`):

| Directory | Responsibility |
|---|---|
| `Body/` | `Body`, `BodyManager`, `BodyInterface` (thread-safe API), `BodyCreationSettings` |
| `Collision/` | Shape hierarchy, broad phase (AABB tree), narrow phase (GJK/EPA/SAT), CCD |
| `Constraints/` | Fixed, Distance, Hinge, Slider, Cone, 6DOF, Pulley, Gear, Rack, SwingTwist |
| `Character/` | `CharacterVirtual` — capsule-based character controller |
| `Vehicle/` | Wheeled and tracked vehicle simulation |
| `Ragdoll/` | Ragdoll construction and animation |
| `SoftBody/` | Soft body (cloth/deformable) simulation |
| `Hair/` | GPU strand-based hair |

**Entry points:**
- `PhysicsSystem` — top-level system; owns bodies, constraints, broad phase, narrow phase
- `BodyInterface` — thread-safe body CRUD and property queries (locking variant preferred)
- `BodyLockRead` / `BodyLockWrite` — RAII body access; always check `lock.Succeeded()`

**Body lifecycle:** `CreateBody` → `AddBody` → (simulate) → `RemoveBody` → `DestroyBody`. Use batch functions (`AddBodiesPrepare` / `AddBodiesFinalize`) when inserting many bodies; adding one at a time degrades the broadphase.

## Code Conventions

**Container lookups:** Always use `.find() != .end()` for existence checks on `UnorderedMap` / `UnorderedSet`. Do **not** use `.count()` — it reads as a quantity, not a boolean, and is inconsistent with the rest of the codebase.

**Naming:**
- Macros: `JPH_` prefix (e.g. `JPH_NAMESPACE_BEGIN`, `JPH_EXPORT`)
- Enum types: `E` prefix (e.g. `EMotionType`, `EAllowedDOFs`)
- Member variables: `m` prefix; static members: `s` prefix; constants: `c` prefix
- Parameters: `in` for inputs, `out` for outputs, `io` for in-out
- Method suffixes: `NoLock` (skips mutex), `Unchecked` (no validation)

**API argument types:** Pass `Vec3Arg`, `Mat44Arg`, `RVec3Arg`, etc. for performance (avoids copies on x86).

**Namespacing:** All library code is inside `JPH_NAMESPACE_BEGIN` / `JPH_NAMESPACE_END` (expands to `namespace JPH {}`).

**No RTTI, no exceptions** — do not use `dynamic_cast` or `throw` inside Jolt code.

**Custom allocation** — override `Allocate`/`Free`/`AlignedAllocate`/`AlignedFree` in `Memory.h`. Use `JPH_OVERRIDE_NEW_DELETE` in classes that need per-class allocation.

## Testing

Tests use **doctest** (header at `UnitTests/doctest.h`). The test helper `PhysicsTestContext` (in `UnitTests/`) sets up a complete `PhysicsSystem` with layers and listeners for use in tests.

Test categories: `Core/` (containers, hash, sort), `Math/` (SIMD types), `Physics/` (bodies, shapes, constraints, characters, vehicles, soft bodies, determinism).

## Common Link Errors

- `file format not recognized` — your app must enable interprocedural optimizations to match Jolt's LTO.
- `unresolved external symbol` for `JPH::ConvexShape::GetSubmergedVolume` — `JPH_DEBUG_RENDERER` define mismatch between Jolt and your project.
- `typeinfo for JPH::DebugRenderer` — RTTI mismatch; set `CPP_RTTI_ENABLED` on Jolt or disable RTTI in your project.

## Performance Notes (DestructibleTest / destruction)

Hard-won lessons for the destructible-environment sample (`Samples/Tests/General/DestructibleTest*`). These generalise to any per-frame structural / graph work layered on top of Jolt.

- **Let intact structure sleep in the `NON_MOVING` broadphase.** Structural bodies use a dedicated `STRUCTURE` object layer that maps to the `NON_MOVING` broadphase (see `Samples/Layers.h`). Sleeping structural bodies in that layer avoids the O(N) per-frame moving-broadphase rebuild — a standing/settled building then costs ~0. Don't put would-be-static structure in `MOVING`.

- **Don't rebuild whole-scene graphs every frame; walk a live, incrementally-maintained adjacency.** The collapse spikes were `EnsureFrameSnapshot()` rebuilding a whole-scene frame-constraint adjacency (and grounded BFS) every time *any* pass broke a joint — up to 4×/frame. The fix: all graph passes walk the live `mFrameAdj` (`UnorderedMap<BodyID, Array<Ref<SixDOFConstraint>>>`, kept current by `TrackConstraint`/`UntrackConstraint`). Avoid index-into-array snapshots: swap-and-pop removal invalidates the indices and forces a full rebuild.

- **Scope per-frame passes to the affected region, not the whole scene.** Ground-reachability never leaves a connected component, so the grounded-set BFS and the cascade checks are restricted to the damaged buildings' components (`mDamagedBuildings` → `ExpandToComponents`). Cost becomes O(damaged scope) instead of O(scene); idle frames cost 0 (gated on the constraint count changing).

- **Profiling:** Jolt's profiler dumps one frame as a flame chart to `Build/XCode_MacOS/Release/profile_chart_*.html` (`cycles_per_second ≈ 24e6`, µs = cycles/24). The `[DESTRUCT SPIKE]` `printf` in `PrePhysicsUpdate` self-reports a per-sub-pass breakdown on frames over threshold — the fastest way to localise a spike. The win is almost always reducing how much is *active* (sleep, scoping), since core constraint+contact solving dominates an active frame.

## Destruction Behaviour Notes (DestructibleTest)

How a building comes apart is governed by several passes in `PrePhysicsUpdate`. Two failure *looks* and their causes:

- **`CheckSupportStability` fractures overhanging elements in place** (turns them to debris when their XZ leaves the ground-stub footprint). This is only correct for a *settled* overhang (a house roof left cantilevered after a wall is shot out). It is wrong for anything moving, where it makes the structure "disintegrate straight down" instead of leaning/toppling. Two guards keep it in its lane:
  - **Height-scaled footprint tolerance** (`sFrameSupportLeanTan`, tan of a lean angle ≈ 9.5°): tolerance = `max(0.5 m, tan · heightAboveStubs)`. A flat tolerance strips tall buildings floor-by-floor because high elements ride a tiny tilt far past the *ground* footprint.
  - **Settle timer, not an instantaneous speed test** (`sFrameSupportRestSpeed` 0.1 m/s + `sFrameSupportSettleTime` 0.4 s): an overhang is fractured only after it has stayed below the rest speed *continuously* for the settle time; moving faster resets its `mSupportSettleTime`. An instantaneous test fails for short buildings — right after the base is shot out the overhang is still asleep (speed 0) yet about to tip, so it gets shattered before it can fall, while tall buildings move fast enough to dodge the test (height-dependent artifact). The timer gives a tipping side time to start moving (it then resets and topples via its joints); only a genuinely settled cantilever stays slow the whole window and fractures. This gate only ever makes the pass fire *less*, so it cannot destabilise the solver.

- **To break a structure realistically, prefer breaking JOINTS (it detaches and topples as a connected chunk) over fracturing bodies in place (debris cloud).** The deform / gravitational-moment / structural-integrity passes break joints; support-stability and contact-damage fracture bodies. When a result looks like "vanishing/disintegrating straight down" rather than "leaning over," suspect a fracture pass firing too eagerly, not the joint passes.

- **Fracturing has a hard shard budget** (`cMaxShards`, 300). When exhausted, a fracture deletes the body and spawns little/no debris (sections can appear to vanish in a violent collapse). **Do NOT "fix" this by keeping the un-fractured body solid and constrained** — under mass collapse that piles up hundreds of interpenetrating constrained rigid bodies and the constraint solver explodes (bodies flying around, unable to settle). The only safe direction is converting an over-budget body to a *single, unconstrained* `DEBRIS`-layer body (DEBRIS collides only with `NON_MOVING`, so it can't shove anything).

- **Diagnostics** (currently in-tree, temporary): the `[BREAK]` line reports, per frame, which joints broke via which pass (V = vertical column↔column, H = horizontal, stub) and which bodies fractured via which source (`dmg`/`support`/`zeroconn`) classified col/beam/slab. This per-pass/per-source attribution is what turns "it looks wrong" into "pass X is doing Y" — build it before guessing at a fix.
