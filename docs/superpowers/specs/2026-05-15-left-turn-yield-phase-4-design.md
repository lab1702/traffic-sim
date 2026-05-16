# Left-Turn Yield (Per-Movement Priority) — Phase 4 Design

**Date:** 2026-05-15
**Status:** Approved (brainstorming phase)

## Goal

Make left turners yield to opposing through-and-right traffic in all three contexts where they currently don't: unsignalized priority-road intersections, signaled normal-green permissive lefts, and AllWayStop intersections where opposing FIFO winners collide. This closes the most visible artifact remaining after Phase 1: priority-road and green-light left turners currently plow through opposing through-traffic without yielding.

The mechanism is a per-intersection "opposing approach" relation, computed once at netbuild time, plus a new `leftTurnYieldsToOpposing` virtual-leader check layered on top of Phase 1's existing yield rules.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Scope | Apply at unsignalized priority-road, signaled normal-green permissive-left, and AllWayStop opposing FIFO ties. |
| Opposing definition | Same axis bucket (8-bucket, 22.5°) as `DefaultSignalConfig` AND `|Δheading| > π/2` to exclude same-direction misalignment. |
| Opposing storage | Per-intersection parallel slice `Intersection.Opposing []int8`, computed at netbuild. |
| Resolution timing | After `sortIncomingByPriority`. Co-sort with index remapping so values stay valid after permutation. |
| Mutual-left handling | Two opposing left-turners pass simultaneously (left-to-left). Encoded via "skip opposing left-turners" inside the gap loop. |
| Critical gap | `leftTurnGapSec = 6.0` (vs existing `gapThresholdSec = 3.0` for straight crossings). Left turns take longer; literature places critical gap at 6–8s. |
| Right-turn refinement | Out of scope; Phase 1's `Control` already approximates right-turn yielding via stop/yield signs. |
| AllWayStop simultaneous left+through | Left turner yields to through after both pass FIFO. No new code; the Section 3 helper covers this naturally. |

## Data model changes

### `internal/network/types.go`

Add field to `Intersection`:

```go
type Intersection struct {
    ID              IntersectionID
    NodeID          NodeID
    Incoming        []EdgeID
    IncomingControl []Control
    // Opposing is parallel to Incoming: Opposing[i] is the position of
    // approach i's opposing approach (the same road's other direction),
    // or -1 if none exists. Populated by netbuild after
    // sortIncomingByPriority. Symmetric: Opposing[Opposing[i]] == i
    // whenever Opposing[i] != -1.
    Opposing []int8
    Outgoing    []EdgeID
    HasSignal   bool
    BannedTurns []TurnRestriction
}
```

Invariants:
- `len(Opposing) == len(Incoming)` always.
- `Opposing[i] == -1` for approaches with no axis-mate (e.g., the stem at a T-intersection).
- `Opposing[i] == j` implies `Opposing[j] == i` (symmetric).
- Each approach has at most one opposing partner; at degenerate stars (>2 bucket-mates), the largest `|Δheading|` wins.

### No new Vehicle field

The left-turn yield decision uses only `Vehicle.Route[Vehicle.RouteIdx+1]` and `network.ClassifyTurn`. No vehicle state is added.

## Opposing-approach resolution

A new helper `resolveOpposing` in `internal/netbuild/control.go`, called from `Build` (`netbuild.go`) after `resolveControls`. Because `resolveOpposing` needs to call `network.ArrivalHeading`, it receives the edges slice (or a partially-assembled `*network.Network` containing at least `Edges`).

```go
// resolveOpposing populates x.Opposing for each intersection. Two
// approaches are opposing iff:
//   1. Their arrival headings fold to the same axis bucket
//      (same 8-bucket / 22.5° resolution as DefaultSignalConfig),
//   2. AND their arrival headings are > π/2 apart (excludes
//      same-direction misalignment at Y-junctions and skewed forks).
//
// If a bucket has more than two members (degenerate star geometry),
// each approach pairs with whichever bucket-mate has the largest
// |Δheading|, i.e. the one most nearly opposite.
func resolveOpposing(xs []network.Intersection, net *network.Network) {
    const numBuckets = 8
    for i := range xs {
        x := &xs[i]
        x.Opposing = make([]int8, len(x.Incoming))
        for k := range x.Opposing {
            x.Opposing[k] = -1
        }
        headings := make([]float64, len(x.Incoming))
        buckets := make([]int, len(x.Incoming))
        for j, eid := range x.Incoming {
            h := network.ArrivalHeading(net, eid)
            headings[j] = h
            ax := math.Mod(h, math.Pi)
            if ax < 0 {
                ax += math.Pi
            }
            buckets[j] = int(math.Round(ax*numBuckets/math.Pi)) % numBuckets
        }
        for j := range x.Incoming {
            best := -1
            bestDelta := math.Pi / 2
            for k := range x.Incoming {
                if k == j || buckets[k] != buckets[j] {
                    continue
                }
                d := math.Abs(angleDiff(headings[j], headings[k]))
                if d > bestDelta {
                    bestDelta = d
                    best = k
                }
            }
            if best >= 0 {
                x.Opposing[j] = int8(best)
            }
        }
    }
}

func angleDiff(a, b float64) float64 {
    d := b - a
    for d > math.Pi {
        d -= 2 * math.Pi
    }
    for d <= -math.Pi {
        d += 2 * math.Pi
    }
    return d
}
```

### Co-sort with priority sort

`sortIncomingByPriority` already co-sorts `Incoming` and `IncomingControl` via index permutation. It must now also:
1. Permute `Opposing` (move slot values).
2. **Remap** each `Opposing` value through the inverse permutation: if approach `oldI` ends up at `newI`, every `Opposing[k] == oldI` becomes `Opposing[k] == newI`.

The remap step is essential because `Opposing` stores **indices into `Incoming`**, not edge IDs. After re-sorting, those indices have shifted.

`resolveOpposing` runs *after* `sortIncomingByPriority`, so in practice the in-flight values inside `Opposing` are all `-1` (default) at sort time and the remap is a no-op. We still co-sort defensively for future-proofing.

## Yield rule extension

New helper in `internal/sim/world.go`:

```go
// leftTurnYieldsToOpposing returns (distance to stop line, true) when
// the vehicle is making a left turn and an opposing-approach vehicle
// has imminent ETA. Layered on top of Phase 1's yield rules — only
// engages when v would otherwise proceed (priority road, green signal,
// or AllWayStop FIFO winner).
func (w *World) leftTurnYieldsToOpposing(v *Vehicle, byEdge map[network.EdgeID][]int) (float64, bool) {
    if v.RouteIdx+1 >= len(v.Route) {
        return 0, false
    }
    edge := &w.Net.Edges[v.Edge]
    x, ok := w.xByNodeID[edge.To]
    if !ok {
        return 0, false
    }
    myPos := IncomingPos(x, v.Edge)
    if myPos < 0 || myPos >= len(x.Opposing) {
        return 0, false
    }
    oppPos := int(x.Opposing[myPos])
    if oppPos < 0 {
        return 0, false
    }
    nextEdge := v.Route[v.RouteIdx+1]
    if network.ClassifyTurn(w.Net, v.Edge, nextEdge) != network.TurnLeft {
        return 0, false
    }
    if !w.entitledToProceed(v, byEdge) {
        return 0, false
    }

    myDist := edge.Length - v.S
    if myDist < 0 {
        myDist = 0
    }

    oppEdgeID := x.Incoming[oppPos]
    oppVehicles := byEdge[oppEdgeID]
    if len(oppVehicles) == 0 {
        return 0, false
    }
    oppEdge := &w.Net.Edges[oppEdgeID]
    for _, oi := range oppVehicles {
        ov := &w.Vehicles[oi]
        if ov.RouteIdx+1 < len(ov.Route) &&
            network.ClassifyTurn(w.Net, ov.Edge, ov.Route[ov.RouteIdx+1]) == network.TurnLeft {
            continue
        }
        d := oppEdge.Length - ov.S
        ovV := ov.V
        if ovV < 0.5 {
            ovV = 0.5
        }
        if d/ovV < leftTurnGapSec {
            return myDist, true
        }
    }
    return 0, false
}

func (w *World) entitledToProceed(v *Vehicle, byEdge map[network.EdgeID][]int) bool {
    if _, isRed := w.stopDistanceForRed(v); isRed {
        return false
    }
    // Phase 1's yield rule covers ControlYield/Stop/AllWayStop plus
    // signal flash/off. If it says we must yield, we're not entitled
    // to proceed (and shouldn't double-yield via the left-turn check).
    if _, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
        return false
    }
    return true
}
```

`stopDistanceForYield` has an idempotent side effect (`maybeMarkStopped`); calling it from both `Step` and `entitledToProceed` in the same tick is safe — the second call is a no-op. The per-tick double-call is O(degree) work twice; if benchmarks reveal it as hot, cache the result in `Step` and pass through.

### New constant

```go
// leftTurnGapSec is the minimum oncoming-traffic ETA a left turner
// accepts before crossing. Larger than gapThresholdSec (straight) because
// the left-turn maneuver takes longer to execute. Literature: 6–8s.
leftTurnGapSec = 6.0
```

`gapThresholdSec`, `stopDwellSec`, `stopLineTolMeters` stay as-is.

### `Step` integration

In the per-vehicle stepping pass in `World.Step` (around `world.go:447`), add the new virtual-leader check after the existing two:

```go
if d, isRed := w.stopDistanceForRed(v); isRed {
    virtualS := v.S + d
    if !has || virtualS < lS {
        lS, lV, has = virtualS, 0, true
    }
}
if d, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
    virtualS := v.S + d
    if !has || virtualS < lS {
        lS, lV, has = virtualS, 0, true
    }
}
// NEW: left-turn opposing-traffic check.
if d, mustYield := w.leftTurnYieldsToOpposing(v, byEdge); mustYield {
    virtualS := v.S + d
    if !has || virtualS < lS {
        lS, lV, has = virtualS, 0, true
    }
}
```

### Stuck-vehicle guard extension

Update the existing stuck-vehicle check:

```go
if !v.Despawned && v.V < stuckSpeedThresh {
    _, isRed := w.stopDistanceForRed(v)
    _, mustYield := w.stopDistanceForYield(v, byEdge)
    _, mustLeftYield := w.leftTurnYieldsToOpposing(v, byEdge)
    if !isRed && !mustYield && !mustLeftYield {
        v.StuckTime += w.dt
        // ... existing despawn logic
    } else {
        v.StuckTime = 0
    }
}
```

## Behavior coverage

**Unsignalized priority road (`ControlNone`):**
- Left turner runs `leftTurnYieldsToOpposing`, finds opposing through/right with imminent ETA, yields. Otherwise proceeds.

**Signaled normal green (`ModeNormal` + `GreenFor(myPos)` + `!IsYellow`):**
- Same. The "permissive left" semantics. Yields to opposing-green through-and-right.

**AllWayStop:**
- Both vehicles complete dwell. FIFO clears both (or staggers — either way, eventually the left turner is FIFO-cleared and reaches `leftTurnYieldsToOpposing`).
- If the opposing FIFO winner is a through or right, left turner yields via the ETA check. Through/right proceeds.
- If both opposing are turning left, mutual-left clause lets both proceed.

**Signaled red:** `stopDistanceForRed` returns the hard stop; `entitledToProceed` returns false; `leftTurnYieldsToOpposing` short-circuits. No double-stop.

**Unsignalized stop sign on my approach:** `effectiveControl` returns non-`ControlNone`; `entitledToProceed` returns false; `leftTurnYieldsToOpposing` short-circuits. The existing stop-then-yield-to-priority logic handles this; the left-turn-specific check would only kick in if my approach is `ControlNone`, which by definition it isn't.

## Mutual-yield resolution

Two opposing left turners both call `leftTurnYieldsToOpposing`. Each scans the opposing approach for vehicles to yield to. The "skip opposing left-turners" clause inside the gap loop means each ignores the other → both return `(0, false)` → both proceed. This produces the correct real-world behavior: opposing lefts pass each other left-to-left.

## Cross-cutting

### Determinism

`leftTurnYieldsToOpposing` iterates `byEdge[oppEdgeID]` (a slice, deterministic order). `resolveOpposing` iterates `x.Incoming` in slice order with a deterministic inner loop. No new randomness. `TestWorld_TraceDeterminism` should pass unchanged.

### Trace format

`x.Opposing` is part of the network (built once, immutable thereafter), not vehicle state. Not written to the trace.

### Performance

`leftTurnYieldsToOpposing` adds one extra virtual-leader check per vehicle per tick. The hot path:
- Most vehicles aren't turning left → `ClassifyTurn != TurnLeft` short-circuit, O(1).
- Left turners: O(opposing-approach-queue-depth) scan, typically <10 vehicles, with an inner short-circuit on opposing-left-turners.

No significant regression expected; will be measured against the current 1k=0.45ms / 5k=2.09ms / 10k=3.24ms baseline.

### Renderer

No renderer change required for Phase 4. The behavior is visible in vehicle motion (left turners pause for oncoming traffic).

## Testing

### New tests in `internal/sim/world_test.go`

- `TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing` — unsignalized 4-way, two opposing `ControlNone` approaches. A turning left, B (opposing) going straight with imminent ETA. A yields until B clears.
- `TestWorld_LeftTurn_PriorityRoad_NoOpposingTraffic` — same fixture, no opposing vehicle. Left turner sails through, `StoppedSinceSec == 0`, `StuckTime == 0`.
- `TestWorld_LeftTurn_MutualLeftsPass` — both opposing vehicles turning left. Both proceed without yielding.
- `TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing` — signaled intersection, normal green for opposing axis. North turner yields to south through.
- `TestWorld_LeftTurn_SignaledRed_NotAffected` — same fixture but left turner's approach is red. The red hard-stop owns the decision; no double-stop.
- `TestWorld_LeftTurn_AllWayStop_YieldsToOpposing` — AllWayStop with two opposing approaches. Both FIFO-clear. Left turner yields to opposing through.
- `TestWorld_LeftTurn_AllWayStop_BothLeftsPass` — both FIFO winners turning left. Both proceed.
- `TestWorld_LeftTurn_StuckGuardBypassed` — perpetual opposing traffic. Left turner waits indefinitely without being despawned.

### New tests in `internal/netbuild/control_test.go`

- `TestNetbuild_Opposing_FourWay` — 4-way + intersection. N pairs with S; E pairs with W.
- `TestNetbuild_Opposing_TThrough` — T-intersection. Through-road approaches pair; stem gets `-1`.
- `TestNetbuild_Opposing_Symmetric` — for every intersection in a mixed-geometry network, `Opposing[Opposing[i]] == i` when `Opposing[i] != -1`.
- `TestNetbuild_Opposing_CoSortsWithIncoming` — force a non-trivial priority sort; assert `Opposing` indices remap correctly.

### Existing tests

Existing intersection-literal fixtures don't set `Opposing`, so it defaults to `nil`. The new `leftTurnYieldsToOpposing` short-circuits when `len(x.Opposing) == 0` (the `myPos >= len(x.Opposing)` guard). Existing tests pass unchanged.

`TestWorld_TraceDeterminism` should pass. The new code introduces no randomness.

## Files changed

- `internal/network/types.go` — add `Opposing []int8` to `Intersection`.
- `internal/netbuild/control.go` — add `resolveOpposing` + `angleDiff` helpers; call from `resolveControls`.
- `internal/netbuild/priority.go` — extend `sortIncomingByPriority` co-sort to include `Opposing` + remap indices.
- `internal/sim/world.go` — add `leftTurnGapSec` constant; add `leftTurnYieldsToOpposing` + `entitledToProceed` helpers; wire into `Step`'s virtual-leader pass and stuck-vehicle guard.
- `internal/sim/world_test.go` — eight new tests covering the three contexts + mutual-left + stuck-guard.
- `internal/netbuild/control_test.go` — four new tests covering opposing resolution.

## Out of scope (deferred)

- Right-turn refinements (right-on-red yielding only to crossing-from-the-right).
- Per-vehicle critical-gap distributions (Phase 3, planned next).
- Impatience: gap shrinks with wait time (Phase 3).
- Multi-lane left-turn pockets (dedicated left-turn lanes).
- "Carrier left" — a left turner waits in the intersection itself and clears on yellow.
- Pedestrian conflicts (pedestrians not modeled at all yet).

## Known follow-ups

After Phase 4 lands, the most visible artifact left in the model is the "infinite wait on a busy priority road's left turn" — Phase 3's impatience mechanism is the natural next fix. The phenomenon will be especially visible at signaled normal-green permissive lefts on busy arterials.
