# GPS Reroute-When-Blocked — Design

**Date:** 2026-05-22
**Status:** Approved (brainstorming phase)

## Goal

When a road is fully closed in front of a GPS-equipped vehicle that is already
committed to it, the vehicle should **reroute around the closure promptly** —
diverting at the upstream intersection without first stopping — instead of
queuing at the closure entrance until a stuck-vehicle timeout clears it.

This is a follow-up to the entry-block fix (`fix(sim): block entry into a
fully-closed edge`). That fix stopped vehicles from driving *through* a closed
edge by stopping them at the entrance. But GPS vehicles that entered the upstream
edge *before* the closure was placed (so they already ran their on-entry reroute
while the edge was open) have no opportunity to reroute again — rerouting only
fires on edge entry, and a vehicle stopped at the entrance never enters a new
edge. They sit at the closure until the 60s stuck-timeout. This design gives them
a prompt reroute.

## Background — why the existing reroute misses this

`maybeReroute` (in `internal/sim/world.go`) already routes correctly around a
closed edge: `edgeCost` returns a very large finite cost (1e9) for a FullClose
edge, so a route tail that begins with the closed edge looks ~1e9-expensive and
any real alternative beats it well past `switchMargin`. The gap is purely in
*when* `maybeReroute` is allowed to run:

1. **Trigger:** it is called only on edge entry (`v.Edge != prevEdge`). A
   vehicle approaching a closure on its current edge does not cross an edge
   boundary, so the trigger never fires.
2. **Cooldown:** it self-gates on a 20s cooldown (`rerouteCooldownSec`). A
   vehicle that just entered its current edge rerouted then (setting
   `LastRerouteSec`), so even if the trigger fired it would be suppressed for up
   to 20s.

So the fix is about the trigger and the cooldown, not the routing math.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Divert timing | While approaching — reroute as soon as the next edge is fully closed, bypassing the 20s cooldown, so the vehicle diverts at the upstream intersection without stopping. |
| Scope | FullClose only. LaneClose/Slowdown edges stay passable and are already handled by ordinary cost-based rerouting. |
| No-alternative case | The entry-block obstacle remains the safety net: a vehicle whose destination is genuinely cut off keeps its route and queues at the entrance (eventually stuck-timeout). |
| Performance trade-off | Bypassing the cooldown means a blocked vehicle with no alternative re-runs A* each tick it stays blocked, bounded by the existing `maxReroutesPerTick` (64). Accepted for v1; gateable later. |
| Determinism | Preserved. No rng/wall-clock; deterministic helper; stable vehicle-order iteration; no-incident runs are byte-identical. |

## Architecture & data flow

All changes are in `internal/sim/world.go`. No new fields, no schema/trace
changes, no new goroutines. The routing computation (`maybeReroute` body,
`edgeCost`, the A* router) is unchanged — only the call gate widens.

```
Step(), per vehicle:
  ... build leaders, apply virtual leaders (incl. incident entry block) ...
  prevEdge := v.Edge
  stepIDM(...)
  reroute trigger:                                                    [CHANGED]
    fire maybeReroute if (edge entry) OR (GPS && next edge FullClose)
       maybeReroute:                                                  [CHANGED]
         cooldown gate skipped when next edge is FullClose
         (unchanged) route from edge.To via edgeCost; switch if cheaper tail
```

## Component 1 — `nextEdgeFullClosed` helper

New small method on `*World` (placed near `maybeReroute` in `world.go`):

```go
// nextEdgeFullClosed reports whether the vehicle's next route edge exists and is
// fully closed. Used to (a) trigger an immediate reroute and (b) bypass the
// reroute cooldown, so a GPS vehicle diverts around a closure ahead of it
// instead of queuing at the entry block.
func (w *World) nextEdgeFullClosed(v *Vehicle) bool {
	if v.RouteIdx+1 >= len(v.Route) {
		return false
	}
	return w.Incidents[v.Route[v.RouteIdx+1]] == FullClose
}
```

O(1): a bounds check plus one map lookup. Returns false whenever there are no
active incidents, so it is inert on normal runs.

## Component 2 — Reroute trigger (`Step`)

The current trigger:

```go
		// GPS rerouting fires on edge entry (a decision point), bounded by the
		// per-tick budget. maybeReroute self-gates on HasGPS and cooldown.
		if !v.Despawned && v.Edge != prevEdge && rerouteBudget > 0 {
			if w.maybeReroute(v) {
				rerouteBudget--
			}
		}
```

becomes:

```go
		// GPS rerouting fires on edge entry (a decision point) and, additionally,
		// when the next edge ahead is fully closed (so a committed vehicle diverts
		// around a closure rather than queuing at it). Bounded by the per-tick
		// budget. maybeReroute self-gates on HasGPS and cooldown.
		if !v.Despawned && rerouteBudget > 0 &&
			(v.Edge != prevEdge || (v.HasGPS && w.nextEdgeFullClosed(v))) {
			if w.maybeReroute(v) {
				rerouteBudget--
			}
		}
```

`maybeReroute` still self-gates on `HasGPS` (so the `v.HasGPS` in the trigger is
a cheap pre-filter to avoid the helper call for non-GPS vehicles; correctness
does not depend on it). When a blocked vehicle reroutes onto a clear tail,
`nextEdgeFullClosed` becomes false and this extra trigger stops firing.

## Component 3 — Cooldown bypass (`maybeReroute`)

The current cooldown gate:

```go
	if w.SimTime-v.LastRerouteSec < rerouteCooldownSec {
		return false
	}
	v.LastRerouteSec = w.SimTime
```

becomes:

```go
	if !w.nextEdgeFullClosed(v) && w.SimTime-v.LastRerouteSec < rerouteCooldownSec {
		return false
	}
	v.LastRerouteSec = w.SimTime
```

When the next edge is fully closed, the cooldown is bypassed so the reroute
happens on the next tick. `LastRerouteSec` is still updated on every attempt, so
once the vehicle is no longer blocked it returns to the normal 20s cadence. The
rest of `maybeReroute` is unchanged.

The two `nextEdgeFullClosed(v)` calls per blocked vehicle per tick (trigger +
cooldown gate) are O(1) and only occur for vehicles actually approaching a
closure; the inert path (no incidents) does one map lookup in the trigger only.

## Behavior under representative scenarios

- **Closure ahead, alternative exists (the target case).** A GPS vehicle is
  mid-edge on E with next edge N; the user closes N. Next tick: the trigger fires
  (`nextEdgeFullClosed`), the cooldown is bypassed, `maybeReroute` routes from
  `E.To` avoiding N (cost 1e9), finds a cheaper tail, and switches — emitting a
  `VehicleReroute`. The vehicle continues to `E.To` and turns onto the new tail,
  never entering N and without stopping.
- **Closure ahead, no alternative (destination cut off).** `maybeReroute` finds
  no better tail (or `RouteCost` returns the same closed path), so the route is
  unchanged. The entry-block obstacle stops the vehicle at the entrance; it
  queues and is eventually cleared by the stuck-timeout. The vehicle re-attempts
  A* each tick while blocked, bounded by `maxReroutesPerTick`.
- **Non-GPS vehicle.** `maybeReroute` returns false on the `HasGPS` gate
  (and the trigger pre-filters on `v.HasGPS`); it queues at the entry block as
  before. Unchanged.
- **Closure cleared.** `nextEdgeFullClosed` becomes false; the vehicle (if it
  had queued) resumes via the now-removed entry block, on its original route.
- **No incidents at all.** `nextEdgeFullClosed` is always false; the trigger
  reduces to the original edge-entry condition and the cooldown is never
  bypassed. Behavior is byte-identical to today.

## Cross-cutting

### Determinism

Preserved. `nextEdgeFullClosed` reads only `v.Route`, `v.RouteIdx`, and
`w.Incidents` — no rng, no wall-clock. The trigger runs inside the existing
stable vehicle-index loop and consumes `rerouteBudget` in order. With no
incidents the helper is always false, so `TestWorld_TraceDeterminism` (which
injects none) is unaffected.

### Performance

The inert (no-incident) path adds at most one O(1) map lookup per GPS vehicle
per tick in the trigger. For vehicles approaching a closure, the reroute A* runs
as for any reroute, bounded by `maxReroutesPerTick`. The only new cost is the
rare no-alternative case re-running A* each tick while blocked, also bounded by
that budget.

### Interaction with the entry block

Complementary, not redundant. The entry block (in `incidentStopDistance`) is the
physical safety net that guarantees no vehicle drives through a closed edge;
reroute-when-blocked is the behavioral layer that lets GPS vehicles avoid
reaching that net. A vehicle that cannot reroute still relies on the entry block.

### Trace / replay

No new event. Diverts continue to emit the existing `VehicleReroute`, so
`tracereplay` already follows the path taken. No `tracereplay` change.

## Testing

### New tests — `internal/sim`

- `TestWorld_FullClose_GPSReroutesWhenBlocked` — a small graph where a GPS
  vehicle on the upstream edge E has next edge N and an alternative tail from
  `E.To` to the destination. Set `LastRerouteSec` to "just rerouted" (within
  cooldown) to prove the bypass, then close N. After stepping (or a direct
  `maybeReroute`), the vehicle's tail no longer contains N (it diverts), a
  `VehicleReroute` is emitted, and across the run it never has `Edge == N`.
- `TestWorld_FullClose_GPSNoAlternativeQueues` — a graph where N is the only path
  to the destination. Close N. The GPS vehicle's route is unchanged and it stops
  at the entry block (does not enter N); confirms the safety net still holds when
  rerouting can't help.
- `TestWorld_NextEdgeFullClosed` — unit test of the helper: false when no next
  edge, false when the next edge is open / Slowdown / LaneClose, true only when
  the next edge is FullClose.

### Existing tests

`TestWorld_Reroute_*` (edge-entry rerouting, cooldown, hysteresis, non-GPS),
`TestWorld_FullClose_BlocksEntryFromUpstream`, and `TestWorld_TraceDeterminism`
must all stay green. In particular `TestWorld_Reroute_CooldownRespected` must
still pass — it uses a vehicle whose next edge is *not* closed, so the cooldown
bypass does not apply.

### Suite

`go build ./...`, `go vet ./...`, `go test ./... -count=1` all green.

## Files changed

- `internal/sim/world.go` — `nextEdgeFullClosed` helper; widen the reroute
  trigger in `Step`; bypass the cooldown in `maybeReroute` when the next edge is
  fully closed.
- `internal/sim/incident_test.go` (or `world_test.go`) — the three new tests.
- `README.md` — one line in the incidents section noting GPS vehicles reroute
  around a closure promptly (and only queue at it when no alternative exists).

## Out of scope (deferred)

- Gating the per-tick A* retry for the rare no-alternative blocked case (e.g.,
  back off to the normal cooldown after a failed blocked attempt). Accepted as-is
  for v1; bounded by `maxReroutesPerTick`.
- Treating LaneClose/Slowdown as reroute-forcing — they remain passable and are
  handled by ordinary cost-based rerouting.
- Any change to the entry block, trace format, or non-GPS behavior.
