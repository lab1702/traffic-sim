# GPS Rerouting Around Traffic Jams — Design

**Date:** 2026-05-21
**Status:** Approved (brainstorming phase)

## Goal

Give vehicles GPS-like behavior: each vehicle observes live travel times on
the road network and re-routes around congestion to reduce its own trip time,
the way a real driver following a navigation app does.

Concretely:

1. **Live congestion signal** — each edge tracks an EWMA-smoothed average of
   the speeds of vehicles currently on it. Routing cost becomes
   `length / observed_speed` (free-flow travel time when the edge is empty),
   so a jammed edge looks expensive and a clear edge looks cheap.

2. **Per-vehicle rerouting** — when a vehicle crosses into a new edge (a
   decision point), past a cooldown, it recomputes the remaining path to its
   destination against current costs and switches only if the new route is
   meaningfully faster (hysteresis). Believable, individual GPS behavior — the
   primary goal chosen in brainstorming.

3. **On by default, with a penetration knob** — every vehicle has GPS by
   default (`--gps-share 1.0`); the share flag lets a fraction of the fleet be
   GPS-equipped for mixed-fleet studies.

The simulation's hard determinism contract ("same seed + OSM + spawn-rate →
byte-identical trace," verified by `TestWorld_TraceDeterminism`) is preserved.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Primary goal | Realistic avoidance — believable individual GPS behavior, lower trip times. |
| Congestion signal | Observed average speed per edge; routing cost = `length / observed_speed`. |
| Smoothing | EWMA over time (rejects transient dips); empty edges target free-flow speed. |
| Reroute trigger | On entering a new edge, gated by a per-vehicle cooldown. Naturally staggers A* work. |
| Switch criterion | Hysteresis — switch only if the candidate route beats the current remaining route by `switchMargin`. |
| Control & default | On by default for every vehicle; `--gps-share 0..1` knob (default 1.0). |
| Cost clamps | Floor at `minEdgeSpeed` (jam ≠ infinite cost) and ceiling at free-flow (keeps A* heuristic admissible). |
| Determinism | Preserved. No wall-clock, no map-iteration-order dependence, A* deterministic, reroutes in stable vehicle-index order. |
| Replay faithfulness | New `VehicleReroute` trace event keeps `tracereplay` on the path the vehicle actually took. |

## Architecture & data flow

One new component, `Congestion`, plus small changes threaded through the
existing `World.Step()` tick. No new goroutines, no locks (the sim goroutine
owns all of it), no wall-clock reads.

```
Step():
  build byEdge / byEdgeLane buckets         (exists)
    └─> Congestion.Update(net, byEdge, veh)  ← refresh smoothed per-edge speeds   [NEW]
  precompute leaders                         (exists)
  step each vehicle (IDM + virtual leaders)  (exists)
    └─> on edge transition: maybeReroute(v)  ← A* on live cost + hysteresis        [NEW]
         └─> emit VehicleReroute             ← keeps tracereplay faithful          [NEW]
  publish snapshot / compact / advance time  (exists)
```

`Congestion.Update` runs after the `byEdge` buckets are built (so it sees this
tick's positions/speeds) and before vehicles step, so any reroute later in the
same tick reads the freshly-updated costs.

## Component 1 — Congestion observatory

New file `internal/sim/congestion.go`.

```go
// Congestion tracks an EWMA-smoothed average speed per edge, refreshed once
// per tick from the vehicles currently on each edge. It converts those
// speeds into routing costs (free-flow travel time, inflated by congestion).
type Congestion struct {
    speed []float64 // smoothed observed speed (m/s), indexed by EdgeID
    alpha float64   // EWMA blend factor in (0,1], derived from half-life and dt
}
```

- `NewCongestion(net *network.Network, halfLifeSec, dt float64) *Congestion`:
  allocate `speed` with `len(net.Edges)`, initialize every entry to that
  edge's free-flow speed (`edge.SpeedLimit`). Derive `alpha` from the
  EWMA half-life:
  `alpha = 1 - math.Exp(-math.Ln2 * dt / halfLifeSec)`.

- `Update(net *network.Network, byEdge map[network.EdgeID][]int, vehicles []Vehicle)`:
  for each edge `eid` in `0..len(speed)-1`, compute the target speed:
  - if `byEdge[eid]` is non-empty: arithmetic mean of `vehicles[i].V` over the
    bucket (sum then divide — order-independent).
  - if empty: the edge's free-flow speed (`edge.SpeedLimit`).

  Then blend: `speed[eid] += alpha * (target - speed[eid])`.

- `Cost(net *network.Network, eid network.EdgeID) float64`:
  `edge.Length / clamp(speed[eid], minEdgeSpeed, freeFlowSpeed)`, where
  `freeFlowSpeed = max(edge.SpeedLimit, minEdgeSpeed)`.

Two deliberate clamps in `Cost`:

- **Floor (`minEdgeSpeed`, 0.5 m/s):** a fully-stopped edge is very expensive
  but never *infinite* cost, so the router can still choose it when it is the
  only way through (dead-end streets, single-bridge crossings). Prevents
  `ErrNoRoute` regressions.
- **Free-flow ceiling:** congestion can only make an edge *slower* than
  free-flow, never faster. This keeps the router's existing A* heuristic
  (which divides straight-line distance by a global max speed of 31.3 m/s)
  admissible — observed speed used in cost never exceeds free-flow ≤ that max,
  so the heuristic never over-estimates and A* stays optimal.

### `internal/sim/congestion.go` — new constants

```go
// minEdgeSpeed floors the speed used for routing cost. A jammed edge is
// expensive but never infinite-cost, so it stays selectable when it is the
// only path. 0.5 m/s ~ 1.8 km/h (crawling).
const minEdgeSpeed = 0.5

// ewmaHalfLifeSec is the half-life of the per-edge speed EWMA: the time for
// a step change in observed speed to be half-reflected in the smoothed value.
// 10s rejects single-vehicle transients (one car braking) while tracking real
// jams forming/clearing over tens of seconds.
const ewmaHalfLifeSec = 10.0
```

## Component 2 — Router dynamic cost

`internal/sim/router.go`. Today `Route` hard-codes
`tentative := gScore[cur.state] + e.Length/speed`. Extract the search body
into a cost-parameterized method; keep `Route` as a thin, behavior-preserving
wrapper.

```go
// RouteCost is Route with a caller-supplied per-edge cost function. cost(eid)
// must return a positive traversal cost (e.g. travel time). The heuristic
// stays distance/maxSpeed, which is admissible as long as cost never implies
// a speed above maxSpeed — Congestion.Cost guarantees this via its free-flow
// ceiling.
func (r *Router) RouteCost(src, dst network.NodeID, cost func(network.EdgeID) float64) ([]network.EdgeID, error)

// Route computes the free-flow shortest path (length / speed limit). Behavior
// is identical to before this change; it now delegates to RouteCost.
func (r *Router) Route(src, dst network.NodeID) ([]network.EdgeID, error) {
    return r.RouteCost(src, dst, func(eid network.EdgeID) float64 {
        e := &r.net.Edges[eid]
        speed := e.SpeedLimit
        if speed < 0.1 {
            speed = 0.1
        }
        return e.Length / speed
    })
}
```

The only change inside the search loop is `e.Length/speed` → `cost(eid)`. Turn
restrictions, U-turn handling, the priority queue, and `reconstruct` are
untouched. Determinism of A* is unchanged: edges are relaxed in `adjOut` slice
order (deterministic), and the priority queue ties break by stable insertion
order.

## Component 3 — Vehicle, spawn & flag

### `internal/sim/vehicle.go` — new fields

```go
// HasGPS marks a vehicle that re-routes around congestion. Set at spawn from
// World.GpsShare. Hand-constructed test vehicles default to false (zero value)
// and are never re-routed, so existing fixtures are unaffected.
HasGPS bool

// DestNode is the vehicle's destination node, cached at spawn so re-routing
// doesn't have to re-derive it from the route tail each time. Equal to the To
// node of the final route edge.
DestNode network.NodeID

// LastRerouteSec is the sim-time of this vehicle's most recent reroute (or
// spawn). Gates re-routing to at most once per rerouteCooldownSec, so a
// vehicle doesn't recompute on every short urban edge it crosses.
LastRerouteSec float64
```

### `internal/sim/world.go` — new field & constants

```go
// GpsShare is the fraction of spawned vehicles given GPS rerouting, in [0,1].
// Defaults to 1.0 (every vehicle) in NewWorld; overridden from --gps-share.
GpsShare float64

// Cong tracks live per-edge congestion and supplies routing costs.
Cong *Congestion
```

```go
// rerouteCooldownSec is the minimum sim-time between a vehicle's reroutes.
// 20s gives GPS-like periodic re-evaluation without thrashing A* on every
// short edge a vehicle crosses.
const rerouteCooldownSec = 20.0

// switchMargin is the hysteresis threshold: a vehicle adopts a candidate route
// only if its estimated cost is at least this fraction cheaper than the current
// remaining route. 0.15 (15%) stops vehicles from flapping between near-equal
// routes as smoothed speeds wobble.
const switchMargin = 0.15

// maxReroutesPerTick caps reroute attempts per tick as a defensive guard on
// the 50ms tick budget under pathological spawn rates. In normal operation the
// edge-transition trigger plus cooldown keep the count well below this.
const maxReroutesPerTick = 64
```

`NewWorld` sets `GpsShare: 1.0` and constructs
`Cong: NewCongestion(net, ewmaHalfLifeSec, DefaultDt)`. The signature is
unchanged — both are fields, so existing callers and tests keep compiling;
`cmd/trafficsim` overrides `w.GpsShare` after construction (the same pattern
already used for `w.Control` and `w.EmitTrace`).

### Spawn-time assignment — `trySpawn`

After the existing `SpeedFactor` and `GapFactor` draws (preserving their
order), add one more `w.rng` draw for GPS membership, then route accordingly:

```go
gpsRoll := w.rng.Float64()
hasGPS := gpsRoll < w.GpsShare

var route []network.EdgeID
var err error
if hasGPS {
    route, err = w.Router.RouteCost(r.OriginNode, r.DestNode, func(eid network.EdgeID) float64 {
        return w.Cong.Cost(w.Net, eid)
    })
} else {
    route, err = w.Router.Route(r.OriginNode, r.DestNode)
}
if err != nil || len(route) == 0 {
    return
}
```

(The current `trySpawn` routes once at the top; this moves that call below the
factor draws so the rng-draw order is fixed and the cost function can be
chosen. The free-flow path for non-GPS vehicles is byte-identical to today's
behavior.)

The `Vehicle` literal gains `HasGPS: hasGPS`, `DestNode: r.DestNode`, and
`LastRerouteSec: w.SimTime` (so the first reroute is allowed one cooldown after
spawn).

GPS vehicles spawn with a congestion-aware initial route; non-GPS vehicles
spawn with — and keep — a static free-flow route.

### CLI — `cmd/trafficsim/main.go`

Add to `runFlags` / `newRunFlagSet`:

```go
fs.Float64Var(&f.gpsShare, "gps-share", 1.0,
    "fraction of vehicles (0..1) with GPS rerouting around congestion")
```

In `runRun`, validate and wire it:

```go
if f.gpsShare < 0 || f.gpsShare > 1 {
    return fmt.Errorf("--gps-share must be in [0,1], got %v", f.gpsShare)
}
...
w := sim.NewWorld(net, spawner, overrides)
w.GpsShare = f.gpsShare
```

The cooldown, EWMA half-life, switch margin, and floor stay as named constants
in `sim` for v1 (documented above); they can be promoted to flags later if
tuning demands it.

## Component 4 — Reroute decision

### Trigger detection — `World.Step`

In the per-vehicle stepping pass, capture the edge before `stepIDM` and compare
after, so reroute logic lives in `World` (where Router/Congestion/EmitTrace are
reachable) rather than in `stepIDM`:

```go
prevEdge := v.Edge
v0 := w.computeDesiredSpeed(v)
stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

if !v.Despawned && v.Edge != prevEdge && rerouteBudget > 0 {
    if w.maybeReroute(v) {
        rerouteBudget--
    }
}
```

`rerouteBudget` is initialized to `maxReroutesPerTick` at the top of the pass.
A vehicle can cross more than one edge in a single tick on very short edges;
comparing `v.Edge != prevEdge` correctly fires once for any such tick (we
reroute from the vehicle's final edge of the tick, which is what matters).

### `World.maybeReroute(v *Vehicle) bool`

Returns true if it consumed a reroute attempt (whether or not the route
actually changed — the budget is about A* cost, which is paid on any attempt).

```go
func (w *World) maybeReroute(v *Vehicle) bool {
    if !v.HasGPS {
        return false
    }
    if v.RouteIdx+1 >= len(v.Route) {
        return false // on the last edge: nothing downstream to change
    }
    if w.SimTime-v.LastRerouteSec < rerouteCooldownSec {
        return false
    }
    v.LastRerouteSec = w.SimTime

    src := w.Net.Edges[v.Edge].To
    costFn := func(eid network.EdgeID) float64 { return w.Cong.Cost(w.Net, eid) }
    candidate, err := w.Router.RouteCost(src, v.DestNode, costFn)
    if err != nil || len(candidate) == 0 {
        return true // attempt made; keep current route
    }

    // Compare candidate vs the current remaining tail under the same cost.
    curCost := 0.0
    for _, eid := range v.Route[v.RouteIdx+1:] {
        curCost += costFn(eid)
    }
    newCost := 0.0
    for _, eid := range candidate {
        newCost += costFn(eid)
    }

    if newCost < curCost*(1-switchMargin) && !sameTail(v.Route[v.RouteIdx+1:], candidate) {
        // Splice: keep history + current edge, replace the tail.
        v.Route = append(v.Route[:v.RouteIdx+1:v.RouteIdx+1], candidate...)
        w.emitReroute(v, v.RouteIdx+1, candidate)
    }
    return true
}
```

`sameTail` is a small slice-equality helper; the explicit check avoids emitting
a no-op reroute event when A* returns the identical path. `RouteIdx` is
unchanged (still points at the current edge); the candidate's first edge starts
at `src = Edges[v.Edge].To`, i.e. exactly the position after the current edge,
so the splice index `RouteIdx+1` is correct.

Cooldown is consumed on every *attempt* (the `LastRerouteSec` assignment is
before the early returns that follow it), so a vehicle on a clear road doesn't
re-run A* every edge.

## Component 5 — Trace event & replay

### `internal/trace/events.go`

```go
const KindVehicleReroute Kind = 9

// VehicleReroute records that a vehicle replaced the tail of its route at
// runtime (GPS rerouting around congestion). AtIndex is the route index of
// the first replaced edge; NewTail is the replacement edge sequence. Replayers
// splice route[:AtIndex] + NewTail to follow the path actually taken.
type VehicleReroute struct {
    VehicleID uint32
    AtIndex   uint32
    NewTail   []uint32
}

func (*VehicleReroute) Kind() Kind { return KindVehicleReroute }
```

### `internal/trace/writer.go` / `reader.go`

Encode/decode mirroring `VehicleSpawn`'s route handling: `VehicleID` (u32),
`AtIndex` (u32), then a u32 count followed by `count` u32 edge IDs. The wire
format's per-event `length` field means older readers skip unknown kind 9
cleanly (forward-compatible), and `tracereplay` built before this change still
reads new traces (it just won't apply reroutes).

### `tracereplay` player — `cmd/tracereplay/player.go`

Handle the new event in `apply`:

```go
case *trace.VehicleReroute:
    rv := p.vehicles[e.VehicleID]
    if rv == nil || int(e.AtIndex) > len(rv.route) {
        return
    }
    tail := make([]uint32, len(e.NewTail))
    copy(tail, e.NewTail)
    rv.route = append(rv.route[:e.AtIndex:e.AtIndex], tail...)
```

The unchanged prefix (`route[:AtIndex]`) preserves the replay vehicle's current
`routeIdx`, so its kinematic position interpolation continues seamlessly onto
the new tail. Replay remains the existing speed-limit approximation — the event
only corrects which *edges* the vehicle traverses, not its precise speed.

## Behavior under representative scenarios

**Direct route jams; alternate is clear.** A grid where A→B has a short direct
arterial and a slightly longer parallel street. Stalled vehicles pin the
arterial's observed speed near `minEdgeSpeed`, so its cost balloons. A GPS
vehicle crossing into the node before the arterial recomputes, finds the
parallel street's cost is >15% cheaper, and switches — emitting a
`VehicleReroute`. A non-GPS vehicle behind it stays on the arterial.

**Transient dip, no flap.** One car brakes briefly on an otherwise clear edge.
The EWMA barely moves (10s half-life over a 1–2s dip), so the cost stays low
and no reroute fires. Without smoothing (the rejected Approach 2) this would
cause a spurious switch.

**Near-equal routes.** Two paths within 15% of each other under current costs.
Hysteresis (`switchMargin`) keeps the vehicle on its current route rather than
oscillating as smoothed speeds wobble around the tie.

**Jam clears.** Stalled vehicles on the arterial clear; its observed speed
recovers toward free-flow over ~10s (EWMA). The next GPS vehicle to reach the
decision point sees the arterial cheap again and routes back onto it.

**Cooldown.** A vehicle crossing a sequence of short 30 m edges transitions
every ~2s but only re-evaluates every 20s, so the per-tick A* load stays bounded
and behavior stays GPS-like (periodic re-check) rather than frantic.

**Penetration < 1.** With `--gps-share 0.5`, half the spawned vehicles (decided
by a deterministic per-vehicle rng draw) reroute; the rest run fixed free-flow
routes, letting you compare equipped vs unequipped trip times.

## Cross-cutting

### Determinism

Preserved. `Congestion.Update` computes per-edge means as sums (order-
independent) and blends in `EdgeID` order. `maybeReroute` runs inside the
existing stable vehicle-index loop, uses no rng, and calls deterministic A*.
The new spawn-time `w.rng.Float64()` draw is at a fixed position in the draw
order, so the rng stream is reproducible. `TestWorld_TraceDeterminism` compares
two runs of the *same* binary and stays green; traces differ from pre-feature
binaries, which the contract explicitly allows ("same seed + OSM + spawn-rate,"
not "same across versions").

### Performance

`Congestion.Update` is O(edges + vehicles) per tick, reusing the `byEdge`
buckets already built in `Step`. Reroute A* fires only for GPS vehicles that
changed edges this tick and are past cooldown — a small fraction per tick —
bounded above by `maxReroutesPerTick`. Will be measured against the current
40×40-grid benchmark (1k = 0.46 ms/tick, 5k = 1.97 ms/tick) to confirm the 50 ms
budget holds.

### Renderer

No renderer change. GPS behavior is observable directly: cars take visibly
different edges. A congestion heatmap is explicitly out of scope (below).

### Trace format

One new event kind (9), additive and forward-compatible. Existing kinds and
the `tracereplay` of old traces are unaffected.

## Testing

### New tests — `internal/sim/congestion_test.go`

- `TestCongestion_EmptyEdgeFreeFlow` — an edge with no vehicles reports its
  free-flow speed; `Cost` equals `length / speedLimit`.
- `TestCongestion_JamRaisesCost` — pin several stopped vehicles on an edge,
  run several `Update`s; observed speed converges toward `minEdgeSpeed` and
  `Cost` rises far above free-flow.
- `TestCongestion_EWMASmoothing` — a one-tick speed dip moves the smoothed
  value by roughly `alpha`, not all the way; verify the blend formula.
- `TestCongestion_CostClamps` — fully-stopped edge yields finite cost
  (floor); a fast edge (SpeedFactor>1 drivers) never costs *less* than
  free-flow (ceiling).

### New tests — `internal/sim/router_test.go`

- `TestRouter_RouteCostAvoidsExpensiveEdge` — a graph with a short direct edge
  and a longer detour; with a cost function that inflates the direct edge,
  `RouteCost` returns the detour. With uniform free-flow cost it returns the
  direct edge (and matches `Route`).
- `TestRouter_RouteUnchanged` — `Route` output is identical to the pre-refactor
  result on a fixed graph (guards the wrapper).

### New tests — `internal/sim/world_test.go`

- `TestWorld_Reroute_SwitchesAroundJam` — small grid, jam the direct path with
  stopped vehicles, spawn a GPS vehicle (`GpsShare=1`); assert it adopts the
  alternate tail and a `VehicleReroute` is emitted.
- `TestWorld_Reroute_NonGPSDoesNotSwitch` — same fixture, hand-built vehicle
  with `HasGPS=false`; route is unchanged, no event emitted.
- `TestWorld_Reroute_CooldownRespected` — a GPS vehicle crossing rapid short
  edges re-evaluates at most once per `rerouteCooldownSec`.
- `TestWorld_Reroute_HysteresisNoFlap` — a candidate within `switchMargin` of
  the current route does not trigger a switch.
- `TestWorld_GpsShare_DeterministicSplit` — spawn N vehicles with
  `GpsShare=0.5` and a fixed seed; the set of GPS vehicles is identical across
  two runs, and the share is approximately 0.5.
- `TestWorld_TraceDeterminism` (existing) — must still pass unchanged.

### New test — `internal/trace` round-trip

- `TestWriteRead_VehicleReroute` — write then read a `VehicleReroute` (incl.
  empty and multi-edge `NewTail`); fields survive the round-trip.

### Tests to review

Spawned-vehicle tests in `internal/sim/world_test.go` that assert specific
routes or counts may shift because spawned vehicles now have GPS by default.
Hand-built-vehicle tests (`HasGPS=false`) are unaffected. Each affected test
will either set `w.GpsShare = 0` (to pin pre-feature routing) or have its
assertion updated to the rerouting behavior, decided per test during
implementation.

## Files changed

- `internal/sim/congestion.go` — **new**: `Congestion` type, `Update`, `Cost`,
  constants.
- `internal/sim/router.go` — extract `RouteCost`; `Route` delegates to it.
- `internal/sim/vehicle.go` — add `HasGPS`, `DestNode`, `LastRerouteSec`.
- `internal/sim/world.go` — `GpsShare`/`Cong` fields; construct `Cong` in
  `NewWorld`; `Congestion.Update` call; reroute trigger + `maybeReroute` +
  `emitReroute` + `sameTail`; GPS assignment and cost-aware routing in
  `trySpawn`; reroute constants.
- `internal/trace/events.go` — `KindVehicleReroute`, `VehicleReroute`.
- `internal/trace/writer.go` / `reader.go` — encode/decode the new event.
- `cmd/trafficsim/main.go` — `--gps-share` flag, validation, wiring.
- `cmd/tracereplay/player.go` — apply `VehicleReroute`.
- Tests: `congestion_test.go` (new), `router_test.go`, `world_test.go`,
  `trace` round-trip.
- `README.md` — document `--gps-share` and the rerouting behavior.

## Out of scope (deferred)

- **Congestion heatmap / new rendering.** Diversions are already visible as
  cars taking different edges; a per-edge color overlay is a separate feature.
- **Emitting `MetricsTick.CongestionIdx`.** The dormant trace field could be
  populated cheaply from `Congestion`, but it is an independent follow-up.
- **Predictive / historical speeds.** Only current smoothed observation; no
  time-of-day profiles or look-ahead prediction.
- **Turn / intersection-delay penalties in cost.** Cost is edge travel time
  only; signal and turn delays are not modeled into the routing cost.
- **Per-edge capacity / fundamental-diagram modeling.** Rejected in
  brainstorming in favor of observed speed.
- **Exposing cooldown / half-life / margin as flags.** Constants for v1.

## Known follow-ups

- If real-OSM runs show GPS vehicles overloading a single alternate (herding),
  the next lever is randomized cost perturbation or stochastic route choice
  among near-equal options.
- Populating `MetricsTick.CongestionIdx` would let `tracereplay` and offline
  analysis quantify how rerouting changes network-wide congestion — a natural
  next step now that the per-edge signal exists.
