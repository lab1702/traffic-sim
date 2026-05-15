# Unsignalized Intersection Control — Phase 1 Design

**Date:** 2026-05-15
**Status:** Approved (brainstorming phase)

## Goal

Replace the current "lower-indexed `Incoming` slot always wins" priority rule at unsignalized (and signal-off / signal-flash) intersections with an explicit per-approach control type — `None`, `Yield`, `Stop`, or `AllWayStop`. Ingest stop/yield-sign tags from OSM where available, infer sensible defaults from highway functional class where they aren't, and enforce a mandatory-stop dwell at `Stop` and `AllWayStop` approaches. Arbitrate `AllWayStop` intersections by FIFO of stop-line arrival time.

This is Phase 1 of a layered effort to improve unsignalized-intersection realism. Out of scope here: per-vehicle critical-gap distributions, impatience-shrinking gaps, and per-movement conflict logic for priority-road left turns. Those are Phases 2-4.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Scope | Data ingestion + replace the existing yield rule + add mandatory-stop dwell. All in one phase. |
| Fallback for untagged unsignalized junctions | Class-based: unequal classes → lower-class approaches get `Stop`; equal classes → all approaches get `AllWayStop`. |
| AllWayStop arbitration | FIFO by stop-line arrival timestamp. Tie-break by `Incoming` index. |
| Signal flash/off modes | Unify under `Control`. `ModeOff` ≡ `AllWayStop`; `ModeFlashB` blinking-red approach ≡ `Stop`, blinking-yellow ≡ `None`. |
| Control storage | Parallel `IncomingControl []Control` slice on `Intersection`, kept in sync with `Incoming`. |
| Mandatory-stop state | One new `float64` field on `Vehicle`. |
| Mandatory-stop dwell | `0.5 s` hardcoded constant. |
| Direction tag handling for OSM signs | Lenient — if `direction=*` missing, apply to all approaches passing through the node. |

## Data model changes

### `internal/network/types.go`

Add the `Control` enum:

```go
type Control uint8

const (
    ControlNone Control = iota // through-movement, no sign
    ControlYield               // yield — slow, no mandatory stop
    ControlStop                // stop sign — mandatory dwell, then gap-accept
    ControlAllWayStop          // all-way stop — Stop + FIFO arbitration
)
```

Extend `Intersection`:

```go
type Intersection struct {
    ID              IntersectionID
    NodeID          NodeID
    Incoming        []EdgeID
    IncomingControl []Control  // NEW — parallel to Incoming; len equal
    Outgoing        []EdgeID
    HasSignal       bool
    BannedTurns     []TurnRestriction
}
```

**Invariants:**
- `len(IncomingControl) == len(Incoming)` always.
- `HasSignal == true` does not preclude `IncomingControl` entries; they're consulted under `ModeOff` / `ModeFlashA` / `ModeFlashB`, not under `ModeNormal`.

### `internal/sim/vehicle.go`

Add one field to `Vehicle`:

```go
// StoppedSinceSec is the sim-time at which this vehicle came to a
// complete stop at its current approach's stop line. Zero means "not
// currently stopped at a stop line." Reset to zero when the vehicle
// transitions to a new edge (clears the intersection).
StoppedSinceSec float64
```

## OSM ingestion

### `internal/osmload/osmload.go`

Extend node-retention in `collect` to also keep nodes tagged with any of: `highway=stop`, `highway=give_way`, `stop=all`, `stop=minor`. (Today they are retained only if referenced by a kept way; in practice `highway=stop`/`give_way` always are, but `stop=all`/`stop=minor` on intersection nodes need explicit retention.)

### `internal/netbuild/netbuild.go`

After intersection construction and **after** `sortIncomingByPriority` runs, populate `IncomingControl` for each intersection X using the following resolution order. The first rule that matches a given approach wins:

1. **`stop=all` on X's node** → every approach gets `ControlAllWayStop`.
2. **`stop=minor` on X's node** → approaches with `highwayPriority(class) > min(class over all approaches at X)` get `ControlStop`; the others get `ControlNone`.
3. **`highway=stop` or `highway=give_way` on X's node**, with `direction=forward` or `direction=backward` → applies only to the matching approach. Without `direction=*` → applies to all approaches passing through X.
4. **`highway=stop` or `highway=give_way` on an interior approach-segment node** (the stop-line position) → applies to the specific approach whose chain contains that node.
5. **Fallback (no OSM signage on X or its approach chains):**
   - Unequal `highwayPriority` classes among approaches → lower-class approaches get `ControlStop`, highest-class approaches get `ControlNone`.
   - Equal classes among all approaches → every approach gets `ControlAllWayStop`.
6. **Degree-2 geometry-only nodes** (which we already skip for intersection emission unless `signalNodes[id]` is true) → not affected. If such a node *is* a signal, `IncomingControl` entries are filled with `ControlNone`.

Population happens after the priority sort, so values are emitted in the final `Incoming` order — no co-sort needed.

## Yield rule rewrite

Replace `World.stopDistanceForYield` in `internal/sim/world.go:222`. The new function dispatches on the effective `Control` for the current approach.

### Effective control

For an approach at position `myPos` on intersection `x`:

- If `x.HasSignal == false` → effective control = `x.IncomingControl[myPos]`.
- If `x.HasSignal == true`:
  - `ModeNormal` → return `(0, false)` immediately. Signal-phase logic owns this case via `stopDistanceForRed`.
  - `ModeOff` → effective control = `ControlAllWayStop` for every approach.
  - `ModeFlashA` / `ModeFlashB`:
    - If `st.GreenFor(myPos) == true` (blinking yellow) → effective control = `ControlNone`.
    - Otherwise (blinking red) → effective control = `ControlStop`.

These overrides are computed at decision time. They do **not** mutate stored `IncomingControl`.

### Dispatch by effective control

- **`ControlNone`** — no yield obligation. Return `(0, false)`.

- **`ControlYield`** — gap-acceptance against every approach at `x` whose effective control is `ControlNone`. Reuse existing `gapThresholdSec`. No mandatory-stop branch; do not set `StoppedSinceSec`.

- **`ControlStop`** — two-stage:
  1. Mandatory stop: if `v.StoppedSinceSec == 0`, command a hard stop at the line. Set `v.StoppedSinceSec = w.Time` once `v.V < stuckSpeedThresh` AND the distance from `v` to the stop line is less than `stopLineTolMeters` (i.e., the vehicle is effectively at the line, not just slow upstream). Once set, if `w.Time - v.StoppedSinceSec < stopDwellSec`, still command hard stop. Once dwell elapses, fall through to step 2.
  2. Gap-acceptance against every approach with effective control `ControlNone`. Same `gapThresholdSec`.

- **`ControlAllWayStop`** — same two-stage as Stop, except step 2 is the FIFO procedure below.

### AllWayStop FIFO

After dwell elapses, scan every other approach `j != myPos` at `x`:

1. Find the lead vehicle on approach `j` — vehicle in `byEdge[x.Incoming[j]]` with the smallest `edge.Length - ov.S` (i.e., closest to the stop line).
2. If that lead vehicle exists and has `StoppedSinceSec > 0` (currently stopped at its line) and `StoppedSinceSec < v.StoppedSinceSec` (it stopped before us), we yield. Return `(myDist, true)`.

If no other approach has an earlier-stopped lead, return `(0, false)` and the vehicle proceeds.

**Tie-break** when two leads share the same `StoppedSinceSec` (same tick): the lower `Incoming` index wins. The higher-indexed approach yields. Deterministic.

### Stop-state clearing

In the existing edge-transition path (where `v.Edge` advances to the next route edge), zero `v.StoppedSinceSec`. This is the single clearing point.

## Constants

In `internal/sim/world.go` alongside existing yield/stuck constants:

```go
// stopDwellSec is the minimum sim-seconds a vehicle must remain
// effectively stationary at a Stop or AllWayStop line before being
// allowed to begin gap-acceptance.
stopDwellSec = 0.5

// stopLineTolMeters is the maximum distance from the stop line at
// which a slow-moving vehicle (V < stuckSpeedThresh) is considered
// to have arrived at the line. Beyond this, the vehicle is "slow
// upstream" but not yet stopped.
stopLineTolMeters = 2.0
```

`stuckSpeedThresh`, `gapThresholdSec`, and `stuckTimeoutSec` stay as-is.

## Cross-cutting

### Determinism

No new randomness. `StoppedSinceSec` is derived from `w.Time` and existing `Vehicle` state. `TestWorld_TraceDeterminism` should pass unchanged.

### Trace format

`Vehicle.StoppedSinceSec` is internal sim state. It is **not** written to the trace. The trace already records the observable position/velocity history that reflects the stop. No format change; existing trace files remain replayable.

### Stuck-vehicle despawn

The 60-second stuck-despawn guard at `world.go:217-219` stays as a safety net. Legitimate yielders and dwell-waiters return `(myDist, true)` from `stopDistanceForYield`, which means `StuckTime` does not accumulate during legitimate waits — same as today. With more realistic unsignalized behavior, false-positive despawns should decrease; we don't change the timer here.

### Renderer

No renderer change required for Phase 1. Showing stop/yield signs visually is deferred.

### Performance

Mandatory-stop check is O(1) per vehicle per tick. AllWayStop FIFO is O(degree) per AllWayStop approach per tick, same big-O as today's priority scan. No expected impact on the 50 ms tick budget at 10,000 vehicles.

## Testing

### New tests in `internal/sim/world_test.go`

- `TestWorld_StopSign_MandatoryDwell` — vehicle approaches a Stop-controlled approach with no cross-traffic. Must come to `v ≈ 0` and dwell `stopDwellSec` before departing.
- `TestWorld_StopSign_GapAcceptance` — Stop-controlled vehicle + priority cross-traffic with short ETA. Vehicle stops, dwells, then waits for gap.
- `TestWorld_AllWayStop_FIFO` — three vehicles arriving on three approaches at staggered times. Departure order matches arrival order.
- `TestWorld_AllWayStop_TickTie` — two vehicles arriving in the same tick on different approaches. Verify deterministic tie-break by `Incoming` index.
- `TestWorld_AllWayStop_StoppedSinceClears` — vehicle clears an intersection, approaches another AllWayStop. Prior `StoppedSinceSec` does not bleed through.
- `TestWorld_SignalOff_TreatedAsAllWayStop` — signaled intersection in `ModeOff`. Behavior matches `ControlAllWayStop` everywhere.
- `TestWorld_SignalFlashB_TreatedAsStop` — blinking-red approach commands mandatory stop; blinking-yellow approach acts as `None`.

### New tests in `internal/netbuild/netbuild_test.go`

- `TestNetbuild_StopTag_Direction` — node tagged `highway=stop direction=forward` on an approach. Only the forward direction gets `ControlStop`.
- `TestNetbuild_StopAll` — intersection tagged `stop=all`. All approaches get `ControlAllWayStop`.
- `TestNetbuild_StopMinor` — intersection tagged `stop=minor`. Minor-class approaches get `ControlStop`; major-class approaches get `ControlNone`.
- `TestNetbuild_Fallback_UnequalClass` — residential meets primary, no signage. Residential gets `ControlStop`, primary gets `ControlNone`.
- `TestNetbuild_Fallback_EqualClass` — two residential roads meet, no signage. All approaches get `ControlAllWayStop`.
- `TestNetbuild_GiveWayTag` — node tagged `highway=give_way` on an approach. That approach gets `ControlYield`.

### Tests to update

- `TestWorld_StuckAtYieldNotDespawned` (`internal/sim/world_test.go:691`) — currently relies on the implicit `Incoming[0]=priority` rule. Update fixture to set `IncomingControl=[ControlNone, ControlYield]`. Behavior expectation unchanged: legitimate yielder is not despawned.
- All `world_test.go` fixtures that build an `Intersection` literal must add an `IncomingControl` slice of matching length (use `ControlNone` entries for signaled intersections; signal-mode yield tests need explicit values).
- Flash/off assertions in `signal_test.go` need updating to include `IncomingControl` setup and verify mandatory-stop dwell on blinking-red approaches.

### Test helper

Add to `world_test.go`:

```go
func controlsOf(controls ...network.Control) []network.Control {
    return controls
}
```

A trivial helper, used purely to make fixture literals readable.

## Files changed

- `internal/network/types.go` — add `Control` enum, add `IncomingControl` field.
- `internal/osmload/osmload.go` — extend node retention to include sign tags.
- `internal/netbuild/netbuild.go` — populate `IncomingControl` after priority sort; new helpers for sign-tag resolution and class-based fallback.
- `internal/sim/world.go` — rewrite `stopDistanceForYield`; add `stopDwellSec` and `stopLineTolMeters` constants; zero `StoppedSinceSec` on edge transition.
- `internal/sim/vehicle.go` — add `StoppedSinceSec` field.
- `internal/sim/world_test.go` — new tests + fixture updates.
- `internal/sim/signal_test.go` — update flash/off assertions.
- `internal/netbuild/netbuild_test.go` — new ingestion tests.

## Out of scope (deferred)

- Per-vehicle critical-gap distributions (Phase 3).
- Impatience: gap shrinks with wait time (Phase 3).
- Per-movement priority logic for priority-road left turns (Phase 4).
- Visual rendering of stop/yield signs.
- Promoting `stopDwellSec` and `gapThresholdSec` to runtime config.
- Removing or lengthening the 60-second stuck-vehicle despawn.

## Known follow-ups

After Phase 1 lands, watch real-OSM `e2e` runs for AllWayStop saturation: the fallback rule will produce many AllWayStops on dense residential grids, which may cause throughput to drop noticeably. If it does, the next lever is either tightening `stopDwellSec` or biasing the equal-class fallback toward `Stop`-on-minor when a clearer functional split exists.
