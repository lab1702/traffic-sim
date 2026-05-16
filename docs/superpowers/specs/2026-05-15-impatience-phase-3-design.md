# Driver-Gap Heterogeneity + Impatience — Phase 3 Design

**Date:** 2026-05-15
**Status:** Approved (brainstorming phase)

## Goal

Make critical-gap acceptance heterogeneous and time-varying:

1. **Per-driver heterogeneity** — each vehicle samples a `GapFactor` at spawn (Normal-then-clamp, same pattern as `SpeedFactor`). The factor multiplies both `gapThresholdSec` (straight crossings) and `leftTurnGapSec` (left turns), so an aggressive driver is consistently aggressive across all gap-acceptance decisions.

2. **Impatience** — each vehicle accumulates `WaitTime` while effectively stopped at a gap-acceptance yield. The accepted gap shrinks linearly with wait time, floored at `minAcceptedGap`. Closes the "infinite wait on a busy priority-road left turn" artifact called out in the Phase 4 spec.

Phase 3 does NOT apply to red lights — red is a hard stop, not a gap-acceptance yield.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Impatience curve | Linear decay with floor: `gap(t) = baseGap × GapFactor − decayRate × WaitTime`, clamped to `[minAcceptedGap, ∞)`. |
| Factor scope | One shared `GapFactor` for both straight and left-turn maneuvers. |
| Decay rate | `impatienceDecayRate = 0.1` (seconds-of-accepted-gap per second of wait). |
| Floor | `minAcceptedGap = 1.5` (aggressive end of normal human gap acceptance; prevents unsafe crossings). |
| Heterogeneity distribution | Normal(1.0, 0.1) clamped to [0.8, 1.2]. Same Normal-then-clamp shape as `SpeedFactor`. |
| WaitTime trigger | Accumulates when `V < stuckSpeedThresh AND (mustYield OR mustYieldLT)`. Resets the moment either condition stops being true. |
| Red-light exclusion | WaitTime does NOT increment when the vehicle is stopped at a red light. Red is a hard stop, not a yield. |
| Storage | Two new `Vehicle` fields: `GapFactor float64` (set at spawn), `WaitTime float64` (updated each tick). |

## Data model changes

### `internal/sim/vehicle.go`

Add to `Vehicle`:

```go
// GapFactor is a per-driver multiplier on critical-gap thresholds
// (gapThresholdSec for straight crossings, leftTurnGapSec for left
// turns). Sampled at spawn from Normal(1.0, gapFactorStdDev) and
// clamped to [gapFactorMin, gapFactorMax]. A zero value is treated
// as 1.0 to keep hand-constructed test vehicles working without
// modification.
GapFactor float64

// WaitTime accumulates sim-seconds during which the vehicle is
// effectively stopped (V < stuckSpeedThresh) AND yielding via
// gap-acceptance (mustYield or mustYieldLT). Resets to 0 the moment
// either condition stops being true (vehicle moves OR yield clears).
// Reduces the effective accepted gap via the impatience decay curve.
WaitTime float64
```

### `internal/sim/world.go` — new constants

```go
// Per-driver gap preference — same Normal-then-clamp shape as SpeedFactor.
gapFactorStdDev = 0.1
gapFactorMin    = 0.8
gapFactorMax    = 1.2

// impatienceDecayRate is the seconds of accepted-gap reduction per
// second of wait time. At 0.1, a 30-second wait reduces the accepted
// gap by 3 seconds. The reduction floors at minAcceptedGap.
impatienceDecayRate = 0.1

// minAcceptedGap is the lower bound on the accepted gap regardless
// of wait time. Prevents impatience from producing physically unsafe
// gaps. 1.5s is on the aggressive end of normal human gap acceptance.
minAcceptedGap = 1.5
```

## Effective-gap helper

New helper in `internal/sim/world.go`, placed near `yieldGapCheck` and `leftTurnYieldsToOpposing`:

```go
// effectiveGap returns the gap (in seconds of oncoming-ETA) that v
// will accept for a maneuver whose base critical gap is baseGap.
// Applies the per-driver GapFactor multiplier and shrinks linearly
// with WaitTime, floored at minAcceptedGap.
//
// A zero GapFactor on a hand-built test vehicle is treated as 1.0 so
// existing fixtures don't have to be updated.
func effectiveGap(v *Vehicle, baseGap float64) float64 {
    factor := v.GapFactor
    if factor == 0 {
        factor = 1.0
    }
    g := baseGap*factor - impatienceDecayRate*v.WaitTime
    if g < minAcceptedGap {
        g = minAcceptedGap
    }
    return g
}
```

## Call-site changes

Both existing gap-check inner loops change from a constant comparison to `effectiveGap(v, ...)`.

### `yieldGapCheck`

```go
if d/ovV < effectiveGap(v, gapThresholdSec) {
    return myDist, true
}
```

### `leftTurnYieldsToOpposing`

```go
if d/ovV < effectiveGap(v, leftTurnGapSec) {
    return myDist, true
}
```

Two single-line changes; identical pattern.

## Spawn-time sampling

In `trySpawn` (`internal/sim/world.go`), after the existing `SpeedFactor` sampling, add the `GapFactor` sample using the same `w.rng` source and Normal-then-clamp pattern:

```go
gapFactor := 1.0 + w.rng.NormFloat64()*gapFactorStdDev
if gapFactor < gapFactorMin {
    gapFactor = gapFactorMin
} else if gapFactor > gapFactorMax {
    gapFactor = gapFactorMax
}
```

Then add `GapFactor: gapFactor` to the `Vehicle{...}` literal.

## WaitTime updates

In `Step` (`internal/sim/world.go`), in the per-vehicle stepping pass, after the three virtual-leader checks have computed `isRed`, `mustYield`, `mustYieldLT`, add (alongside the existing stuck-vehicle guard):

```go
// Update WaitTime: accumulates while the vehicle is effectively
// stopped AND yielding via gap-acceptance. Resets the moment either
// condition stops being true. Drives the impatience curve in
// effectiveGap; does NOT apply to red lights (red is a hard stop,
// not a gap-acceptance yield).
if v.V < stuckSpeedThresh && (mustYield || mustYieldLT) {
    v.WaitTime += w.dt
} else {
    v.WaitTime = 0
}
```

This reuses the already-cached `mustYield` and `mustYieldLT` from Phase 4's refactor.

### Edge-transition reset

In `stepIDM` (`internal/sim/vehicle.go`), where `StoppedSinceSec` is already cleared on edge transition, add `v.WaitTime = 0`:

```go
v.Edge = v.Route[v.RouteIdx]
edge = &net.Edges[v.Edge]

// Clear any mandatory-stop arrival timestamp and accumulated
// impatience now that we've left the prior approach.
v.StoppedSinceSec = 0
v.WaitTime = 0
```

The in-Step reset already covers the common case (WaitTime resets when not yielding). The edge-transition reset is belt-and-suspenders against any tick-timing edge case where the vehicle could carry residual WaitTime into the next intersection.

## Behavior under representative scenarios

**Aggressive driver at a stop sign with light cross-traffic.**
GapFactor = 0.9. Base 3s × 0.9 = 2.7s. Cross-traffic ETA = 3s → 3 > 2.7, vehicle accepts the gap. A cautious driver (GapFactor = 1.1) at the same intersection would see 3s vs 3.3s, yield. Heterogeneity produces visible variance in how aggressively drivers cross.

**Patient driver, busy priority road, left turn.**
GapFactor = 1.0. Base 6s. Oncoming ETA = 2s (perpetual). At t=0, vehicle yields. WaitTime accumulates. At t=30s, effectiveGap = 6 − 0.1×30 = 3s → still yielding (2 < 3). At t=45s, effectiveGap = 1.5s (floor) → 2 > 1.5 → vehicle accepts. Real-world: priority-road left-turners eventually force a smaller gap.

**Floor protects against unsafe crossings.**
Oncoming ETA = 1s. effectiveGap floors at 1.5s. 1 < 1.5 always → vehicle waits forever. The floor prevents impatience from producing collision-imminent crossings.

**Reset on movement.**
Vehicle waits 30s, accepts the gap, accelerates past `stuckSpeedThresh`. Next tick: WaitTime resets to 0. If new cross-traffic appears, the vehicle re-engages the gap check with full base gap (no leftover impatience).

**Reset on edge transition.**
Vehicle crosses an intersection where it waited. `stepIDM` zeros WaitTime as it transitions to the outbound edge. Vehicle arrives at the next intersection with WaitTime = 0 (not pre-impatient).

## Cross-cutting

### Determinism

`GapFactor` sample uses `w.rng.NormFloat64()` — same rng as `SpeedFactor`. `WaitTime` updates deterministically from existing sim state. `TestWorld_TraceDeterminism` should pass unchanged. The trace doesn't change format because `GapFactor`/`WaitTime` are internal state not written to the trace.

### Trace format

Unchanged. `GapFactor` and `WaitTime` are derivable internal state. The observable position/velocity history (vehicles wait shorter or longer, accept different-sized gaps) is already recorded.

### Performance

`effectiveGap` adds one multiply, one subtract, one floor-compare per gap-check inner-loop iteration. At 10k vehicles the overhead is bounded; will be measured against post-Phase-4 baseline (1k=0.54ms, 5k=1.86ms, 10k=3.31ms).

### Stuck-vehicle guard interaction

`WaitTime` is independent of `StuckTime`. Both can accumulate, but they have different reset conditions:
- `StuckTime` accumulates when slow AND NOT yielding; resets when moving OR legitimately yielding.
- `WaitTime` accumulates when slow AND yielding; resets when moving OR not yielding.

The two timers count complementary states. A legitimately yielding vehicle has StuckTime=0 (won't despawn) and rising WaitTime (impatience builds). A bug-stuck vehicle has rising StuckTime (eventually despawns) and WaitTime=0.

### Renderer

No renderer change required. Phase 3 behavior is observable through vehicle motion (different wait durations).

## Testing

### New tests in `internal/sim/world_test.go`

- `TestWorld_Impatience_StraightCrossingShrinksGap` — Yield-controlled vehicle facing perpetual cross-traffic at ETA = 2.5s. With base gap 3s, vehicle yields at t=0. After ~5s wait, effectiveGap drops to 2.5s and vehicle accepts. Verify wait duration is in the predicted range.

- `TestWorld_Impatience_LeftTurnShrinksGap` — Priority-road left turner facing perpetual opposing traffic at ETA = 3.5s. Base gap 6s; with GapFactor=1.0 and decayRate=0.1, vehicle waits ~25s before accepting the 3.5s gap.

- `TestWorld_Impatience_FloorPreventsCollision` — Same fixture but oncoming ETA = 1.0s (below 1.5s floor). Vehicle waits indefinitely. Run for 200s of sim; vehicle still on approach edge.

- `TestWorld_Impatience_ResetsOnMovement` — Vehicle yields, accumulates WaitTime, gap clears, vehicle accelerates past `stuckSpeedThresh`. Verify WaitTime resets to 0 on the next tick. Prevents flip-flop where a shrunken accepted gap persists into a re-tightened cross-traffic situation.

- `TestWorld_Impatience_ResetsOnEdgeTransition` — Vehicle yields at intersection A, accumulates WaitTime, eventually crosses, transitions to outbound edge. Verify `v.WaitTime == 0` immediately after the edge transition.

- `TestWorld_Impatience_NotAppliedToRedLight` — Signaled intersection in `ModeNormal` with red. Vehicle stopped at red for 60s. Verify `v.WaitTime == 0` throughout (red is not a gap-acceptance yield).

- `TestWorld_GapFactor_Heterogeneous` — Spawn 200 vehicles via `trySpawn` with a fixed seed. Inspect their `GapFactor` distribution: mean within [0.95, 1.05] (±0.05 of 1.0), std within [0.07, 0.13] (±0.03 of 0.1), all values within [0.8, 1.2]. 200 samples is enough that these ranges catch real distribution bugs without flaking on small-N noise. Determinism: re-run with same seed produces the same 200 values bit-for-bit.

### Tests to update

- `TestWorld_StuckAtYieldNotDespawned` (Phase 1) — pins priority vehicle at S=99, V=0.5 → ETA=2s. Under Phase 3, the yielder accumulates impatience and eventually accepts that gap (~5s wait reduces gap from 3s to 2.5s, then less). To preserve the test's intent ("legitimately yielding vehicle not despawned"), change the priority vehicle's pinning so ETA stays below `minAcceptedGap=1.5s` (e.g., S=99, V=1.0 → d=1, V=1.0 → ETA=1.0s, below the floor). The yielder stays yielding indefinitely.

- `TestWorld_LeftTurn_StuckGuardBypassed` (Phase 4) — same fix. Push opposing-vehicle pinning so ETA < 1.5s perpetually.

- `TestWorld_StopSign_GapAcceptance` (Phase 1) — pins priority at S=99, V=0.5 → ETA=2s. After ~5s, impatience drops the gap below 2s and the stop vehicle proceeds. Fix the same way: push the pin to ETA < 1.5s, OR reduce the test's tick budget so the assertion runs before impatience takes effect. The pin fix is cleaner.

- `TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing` (Phase 4) — pins opposing at S=98, V=0.5 → ETA=4s. Base gap 6s. Within the 300-tick (15s) test window, impatience drops gap to ~4.5s, still > 4s ETA → vehicle yields. Likely passes unchanged but margins are thin; tighten the pin to V≥1.5 (ETA < 1.5s) for robustness.

- `TestWorld_LeftTurn_AllWayStop_YieldsToOpposing` (Phase 4) — pins opposing at S=98, V=0.5 → ETA=4s, 500-tick (25s) budget. At 25s wait, gap drops to 3.5s — below 4s, vehicle accepts. **Definitely needs pin fix:** push opposing to ETA < 1.5s (e.g., V=1.5 → ETA=1.33s).

- `TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing` (Phase 4) — pins opposing at S=98, V=0.5 → ETA=4s, 300-tick (15s) budget. Gap drops to 4.5s, still > 4s. Likely passes; tighten the pin for robustness.

- `TestWorld_LeftTurn_AllWayStop_BothLeftsPass` (Phase 4) — both vehicles turning left, mutual-left clause skips opposing. No perpetual yielding. No change expected.

## Files changed

- `internal/sim/vehicle.go` — add `GapFactor`, `WaitTime` fields; zero `WaitTime` in edge transition.
- `internal/sim/world.go` — add Phase 3 constants; add `effectiveGap` helper; update `yieldGapCheck` and `leftTurnYieldsToOpposing` call sites; sample `GapFactor` in `trySpawn`; update `WaitTime` in `Step`.
- `internal/sim/world_test.go` — 7 new tests; ~4 fixture updates.

## Out of scope (deferred)

- Per-maneuver gap factors (separate `StraightGapFactor` and `LeftGapFactor`).
- Non-linear impatience curves (exponential, piecewise step).
- Honking / peer pressure on stalled cross-traffic.
- Red-light running (impatience does not apply to red).
- Driver fatigue (impatience persisting across intersections).
- Configurable per-intersection patience overrides.

## Known follow-ups

After Phase 3 lands, the impatience floor (`minAcceptedGap`) becomes the new "infinite wait" boundary: a vehicle facing oncoming traffic with ETA < 1.5s perpetually still waits forever. In a real city this rarely happens — the 1.5s threshold is conservative. If real-OSM e2e runs surface persistent gridlock at busy 4-way signaled intersections with permissive lefts, the next lever is either lowering the floor or adding an "abandon maneuver" behavior where extreme-impatience vehicles re-route.
