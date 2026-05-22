# Interactive Incidents — Design

**Date:** 2026-05-22
**Status:** Approved (brainstorming phase)

## Goal

Let a user inject **road incidents** into a running simulation by clicking an
edge in the live viewer, and watch traffic respond — slow, merge, queue, and
(for GPS-equipped vehicles) reroute around the disruption.

Concretely:

1. **Three severities, one mechanism.** An incident is per-edge state with a
   severity — *Slowdown*, *LaneClose*, or *FullClose*. All three reduce to a
   desired-speed cap and/or a virtual stopped obstacle in specific lanes at the
   edge's downstream end, plus a routing-cost penalty — reusing the existing
   IDM car-following, lane-change, congestion, and GPS-rerouting machinery
   rather than adding parallel systems.

2. **Interactive injection.** Shift + left-click on an edge in the viewer
   cycles its incident: `none → Slowdown → LaneClose → FullClose → none`.
   Incidents stay until cleared (manual clear — no auto-expiry).

3. **Faithful replay.** Each set/cycle/clear is recorded as a new
   `IncidentSet` trace event, so `tracereplay` reconstructs incidents at the
   same ticks they occurred live, even though injection itself is a manual
   (non-deterministic) input.

The simulation's determinism contract is preserved for runs without manual
injection: `TestWorld_TraceDeterminism` (which injects nothing) stays
byte-identical, and the incident *logic* is exercised by tests that call the
apply path programmatically rather than through the UI.

## Major decisions (from brainstorming)

| Axis | Decision |
|---|---|
| Trigger source | Interactive only — click an edge in the live viewer. No config file, no random spawning (deferred). |
| Severity | Configurable per incident: Slowdown / LaneClose / FullClose. |
| Lifecycle | Manual clear; incidents persist until the user clears them. No fixed duration. |
| Injection gesture | Shift + left-click cycles severity on the nearest edge (plain left-click is already intersection-select / pan). |
| Modeling approach | Explicit per-edge incident state (`map[EdgeID]Severity`) consulted in `Step`; each severity maps to its fitting mechanism. |
| Effect mechanism | Slowdown = desired-speed (`v0`) cap; LaneClose = virtual stopped obstacle in the closed lane + forced merge-out; FullClose = obstacle in all lanes + large routing cost. |
| Routing | A new `edgeCost` combines `Cong.Cost` with an incident penalty, used by both spawn-time routing and rerouting, so GPS vehicles avoid incidents. |
| Replay faithfulness | New `IncidentSet` trace event (kind 10); `tracereplay` tracks and renders incident state. |
| Rendering | `Snapshot` gains an `Incidents` overlay; the viewer colors affected edges by severity and the HUD shows the active count. |
| Determinism | Preserved for no-injection runs; manual injection is an external input, recorded in the trace. |

## Architecture & data flow

Incident state is owned by the sim goroutine (no locks). Injection mirrors the
existing UI→sim `Control` channel exactly. No new goroutines, no wall-clock
reads.

```
viewer: Shift+click edge
   └─> hitTestEdge → look up current severity in last Snapshot → next severity
        └─> Viewport.OnIncident(edgeID, severity)             [NEW]
             └─> incidentCh <- IncidentEvent                  [NEW]

Step():
  drain Control channel        (exists)
  drain IncidentControl channel → applyIncident               [NEW]
      └─> set/clear World.Incidents + emit IncidentSet         [NEW]
  advance signals              (exists)
  demand / spawn               (exists)  trySpawn routes via edgeCost (GPS) [CHANGED]
  build byEdge / byEdgeLane    (exists)
  Congestion.Update            (exists)
  precompute leaders           (exists)
  step each vehicle:
      v0 = computeDesiredSpeed  → Slowdown cap                 [CHANGED]
      incident virtual leader (closed lane / full close)       [NEW]
      stepIDM                  (exists)
      maybeReroute → edgeCost (congestion + incident penalty)  [CHANGED]
      tryLaneChange(..., closedLane) → vacate closed lane      [CHANGED]
  publishSnapshot → fill Snapshot.Incidents                    [CHANGED]
  compact / advance time       (exists)

tracereplay: apply(IncidentSet) → track map → render overlay   [NEW]
```

`applyIncident` runs at the top of the tick (before stepping) so an incident
takes effect on the same tick it is injected.

## Severity semantics — the unifying model

An incident is a stopped obstacle in specific lanes at the **downstream end** of
the edge, plus a desired-speed cap and/or routing-cost penalty. The three
severities are points on that spectrum:

| Severity | `v0` cap | Virtual stopped obstacle | Routing penalty |
|---|---|---|---|
| Slowdown | yes (`incidentSlowdownFactor` × limit) | none | modest multiplier |
| LaneClose | none | in the closed lane, at edge end | multiplier |
| FullClose | none | in **every** lane, at edge end | large finite cost |

- **Slowdown.** Cars crawl through at a fraction of the limit. Their reduced
  speed lowers the edge's observed (congestion) speed organically; the routing
  multiplier nudges GPS cars to reroute without waiting for the EWMA to catch
  up.
- **LaneClose.** One lane (the curb lane, index 0) is closed. Cars in it see a
  virtual stopped leader at the edge end and the lane-change logic forces them
  to merge into an open lane before reaching it. Throughput drops; congestion
  rises; GPS reroutes. On a single-lane edge, LaneClose has no open lane to
  merge into and therefore behaves identically to FullClose (documented).
- **FullClose.** Every lane is closed: all cars queue behind the obstacle and
  none traverse. The large finite routing cost makes GPS vehicles avoid the
  edge entirely; cars already on the edge queue at the closed end (a documented
  simplification — see Cross-cutting → Known limitations).

The routing penalty is a **large finite** cost, never infinite — consistent
with `congestion.go`'s existing `minEdgeSpeed` floor philosophy ("expensive but
never infinite-cost, so it stays selectable when it is the only path through").

## Component 1 — Incident state & cost (`internal/sim/incident.go`, new)

```go
// Severity is the kind/intensity of a road incident on an edge.
type Severity uint8

const (
    SeverityNone Severity = 0 // no incident / clear (used by IncidentSet to clear)
    Slowdown     Severity = 1
    LaneClose    Severity = 2
    FullClose    Severity = 3
)

// IncidentEvent is a UI→sim command to set (or clear, with SeverityNone) the
// incident on an edge. Delivered over World.IncidentControl, mirroring
// ControlEvent / World.Control.
type IncidentEvent struct {
    EdgeID   network.EdgeID
    Severity Severity
}
```

Constants (tunable; documented inline like the GPS constants):

```go
// incidentSlowdownFactor caps desired speed on a Slowdown edge to this
// fraction of its limit. 0.3 ~ a hazard crawl.
const incidentSlowdownFactor = 0.3

// Routing-cost penalties applied in edgeCost. Slowdown/LaneClose multiply the
// base (congestion) cost so GPS reroutes promptly without waiting for the EWMA;
// FullClose uses a large finite cost so the edge is avoided but still selectable
// as a last resort (mirrors the minEdgeSpeed floor philosophy).
const (
    incidentSlowdownCostMul  = 1.5
    incidentLaneCloseCostMul = 3.0
    incidentFullCloseCost    = 1e9
)
```

Helpers (methods on `*World`, defined here for cohesion):

```go
// edgeCost is the routing cost for an edge: congestion travel time, adjusted
// for any incident. Used by both spawn-time routing and rerouting.
func (w *World) edgeCost(eid network.EdgeID) float64 {
    base := w.Cong.Cost(w.Net, eid)
    switch w.Incidents[eid] {
    case Slowdown:
        return base * incidentSlowdownCostMul
    case LaneClose:
        return base * incidentLaneCloseCostMul
    case FullClose:
        return incidentFullCloseCost
    default:
        return base
    }
}

// closedLaneFor returns the lane index closed on an edge and whether any lane
// is closed. LaneClose closes the curb lane (0). FullClose is handled by the
// virtual-leader path (all lanes), not here. A 1-lane LaneClose returns
// (0, true) and is treated as a full block by the obstacle logic.
func (w *World) closedLaneFor(eid network.EdgeID) (uint8, bool)

// applyIncident sets or clears the incident on an edge and records it. Bounds-
// checks EdgeID and ignores out-of-range ids (defensive, like applyControl).
func (w *World) applyIncident(ev IncidentEvent) {
    if int(ev.EdgeID) < 0 || int(ev.EdgeID) >= len(w.Net.Edges) {
        return
    }
    if ev.Severity == SeverityNone {
        delete(w.Incidents, ev.EdgeID)
    } else {
        w.Incidents[ev.EdgeID] = ev.Severity
    }
    w.EmitTrace(w.Tick, w.SimTime, &trace.IncidentSet{
        EdgeID:   uint32(ev.EdgeID),
        Severity: uint8(ev.Severity),
    })
}
```

## Component 2 — World wiring (`internal/sim/world.go`)

New fields on `World`:

```go
// Incidents maps an edge to its active incident severity. Absent key == no
// incident. Owned by the sim goroutine; read by publishSnapshot.
Incidents map[network.EdgeID]Severity

// IncidentControl delivers runtime incident commands from the UI. Step drains
// it non-blocking at the top of each tick, like Control. Nil disables.
IncidentControl <-chan IncidentEvent
```

`NewWorld` initializes `Incidents: make(map[network.EdgeID]Severity)`. The
constructor signature is unchanged; `cmd/trafficsim` sets `w.IncidentControl`
after construction, the same pattern already used for `w.Control`.

In `Step`, after the existing `Control` drain, add a symmetric bounded drain:

```go
if w.IncidentControl != nil {
    for i := 0; i < 64; i++ {
        select {
        case ev := <-w.IncidentControl:
            w.applyIncident(ev)
        default:
            i = 64
        }
    }
}
```

In the per-vehicle stepping pass, fold an incident virtual stopped leader in
beside the existing red/yield/left-turn virtual leaders:

```go
// Incident obstacle: full closure blocks every lane; a lane closure blocks
// only the closed lane. A blocked car stops at the edge end (the obstacle),
// forcing it to merge out (LaneClose) or queue (FullClose).
edge := &w.Net.Edges[v.Edge]
sev := w.Incidents[v.Edge]
blocked := sev == FullClose
if sev == LaneClose {
    if cl, ok := w.closedLaneFor(v.Edge); ok && v.Lane == cl {
        blocked = true
    }
}
if blocked {
    virtualS := edge.Length // stop at the downstream end
    if !has || virtualS < lS {
        lS, lV, has = virtualS, 0, true
    }
}
```

In `maybeReroute`, replace the bare congestion cost function with `edgeCost`:

```go
costFn := func(eid network.EdgeID) float64 { return w.edgeCost(eid) }
```

In `trySpawn`, the GPS branch routes against `edgeCost` instead of `Cong.Cost`
(non-GPS vehicles keep static free-flow `Route`), so freshly spawned GPS
vehicles also avoid incident edges. The rng draw order is unchanged.

The lane-change call passes the closed lane so the mover can vacate it:

```go
closed, hasClosed := w.closedLaneFor(v.Edge)
cl := int8(-1)
if hasClosed && sev == LaneClose {
    cl = int8(closed)
}
tryLaneChange(v, i, lanes, w.Vehicles, w.Net, cl)
```

## Component 3 — Desired-speed cap (`internal/sim/cornering.go`)

`computeDesiredSpeed` gains a Slowdown cap, applied to the base `v0` before the
existing corner-cap logic so the lower of the two wins:

```go
v0 := edge.SpeedLimit * factor
if w.Incidents[v.Edge] == Slowdown {
    if cap := edge.SpeedLimit * incidentSlowdownFactor; cap < v0 {
        v0 = cap
    }
}
```

(The corner-cap block below is unchanged; it already returns the smaller cap
when applicable.)

## Component 4 — Forced merge-out (`internal/sim/lanechange.go`)

`tryLaneChange` gains a `closedLane int8` parameter (-1 = none). When the
vehicle's current lane equals `closedLane`, vacating takes priority over the
normal MOBIL incentive: the function attempts to move to an adjacent **open**
lane as soon as a safe gap exists (the existing gap-acceptance / safety checks
are reused unchanged — we never force an unsafe change). When `closedLane` is
-1, behavior is byte-identical to today.

This is the main genuinely-new behavioral logic. The virtual stopped obstacle
(Component 2) supplies the *pressure* to leave the closed lane; this supplies
the *mechanism* to actually change out of it.

## Component 5 — Snapshot & rendering

### `internal/snapshot/snapshot.go`

```go
// Incident severities, kept here (not in sim/) so the renderer and replayer can
// switch on them without importing sim. Values match sim.Severity exactly; the
// two constant blocks are intentionally redundant and guarded by a test, like
// the signal-mode constants.
const (
    SevNone      uint8 = 0
    SevSlowdown  uint8 = 1
    SevLaneClose uint8 = 2
    SevFullClose uint8 = 3
)

type IncidentView struct {
    EdgeID   uint32
    Severity uint8
}
```

`Snapshot` gains `Incidents []IncidentView`. Per the snapshot immutability
contract, `publishSnapshot` allocates a fresh slice each tick from
`World.Incidents` (map iteration order doesn't matter — rendering is
order-independent).

### `internal/render/viewport.go`

- `hitTestEdge(mx, my int) (network.EdgeID, bool)` — convert the cursor to world
  space (same transform as `hitTestIntersection`) and return the nearest edge by
  point-to-segment distance within a radius. Mirrors `hitTestIntersection`.
- In `Update`, the "click without drag" branch checks for the Shift modifier:

  ```go
  if v.dragging && !v.movedSinceDown {
      if ebiten.IsKeyPressed(ebiten.KeyShift) {
          if eid, ok := v.hitTestEdge(mx, my); ok && v.OnIncident != nil {
              v.OnIncident(uint32(eid), nextSeverity(v.severityOf(eid)))
          }
      } else if id, ok := v.hitTestIntersection(mx, my); ok {
          v.selectedID, v.hasSelection = id, true
      } else {
          v.hasSelection = false
      }
  }
  ```

  `severityOf(eid)` reads the current severity from the latest snapshot;
  `nextSeverity` advances `none → Slowdown → LaneClose → FullClose → none`. The
  cycle is computed in the viewer from observed state and pushed as an absolute
  severity; `applyIncident` just sets/clears whatever it is told. Shift+drag
  still pans (only Shift+click-without-drag injects).
- New `OnIncident func(edgeID uint32, severity uint8)` callback field.
- `Draw` overlays incident edges in `drawRoadBands`: amber (Slowdown), orange
  (LaneClose), red (FullClose), drawn from `Snapshot.Incidents`.

### `internal/render/hud.go`

Add a line showing the active incident count (`len(snapshot.Incidents)`), and a
one-line hint for the Shift+click control.

## Component 6 — Trace event & replay

### `internal/trace/events.go`

```go
const KindIncidentSet Kind = 10

// IncidentSet records that the incident on an edge was set or cleared at
// runtime. Severity 0 (SeverityNone) is a clear; 1/2/3 are Slowdown/LaneClose/
// FullClose. Replayers track the latest severity per edge.
type IncidentSet struct {
    EdgeID   uint32
    Severity uint8
}

func (*IncidentSet) Kind() Kind { return KindIncidentSet }
```

### `internal/trace/writer.go` / `reader.go`

Encode/decode mirroring the simple fixed-size events: `EdgeID` (u32) then
`Severity` (u8). The wire format's per-event length field means older readers
skip unknown kind 10 cleanly (forward-compatible), and a `tracereplay` built
before this change still reads new traces (it just won't show incidents).

### `tracereplay` player — `cmd/tracereplay/player.go`

Add `p.incidents map[uint32]uint8`, handle the event in `apply`:

```go
case *trace.IncidentSet:
    if e.Severity == 0 {
        delete(p.incidents, e.EdgeID)
    } else {
        p.incidents[e.EdgeID] = e.Severity
    }
```

`publish` copies the map into `Snapshot.Incidents`, so the replay viewer shows
incidents appearing and clearing at exactly the recorded ticks. The replay
viewer is read-only (no injection).

## Behavior under representative scenarios

**Full closure with GPS.** A user Shift+clicks an arterial up to FullClose. Its
`edgeCost` jumps to `incidentFullCloseCost`; the next GPS vehicle reaching the
upstream decision point reroutes onto a parallel street (emitting
`VehicleReroute`). Cars already on the closed edge drive up to the obstacle at
the edge end and queue; non-GPS cars committed to it queue behind them and, if
truly wedged, are eventually cleared by the existing stuck-timeout.

**Lane closure and merge.** A multi-lane edge set to LaneClose: cars in the
curb lane get a stopped obstacle at the edge end, the lane-change logic moves
them into an open lane as gaps appear, and the edge keeps flowing at reduced
throughput. Congestion rises and some GPS vehicles reroute.

**Slowdown.** A Slowdown edge caps `v0` to 30% of the limit; cars crawl through,
the observed speed drops, and `edgeCost` (× `incidentSlowdownCostMul`) plus the
organic congestion rise prompt GPS cars to weigh alternates.

**Clear.** Shift+clicking a FullClose edge once more cycles it back to `none`:
`applyIncident` deletes the map entry, the obstacle and routing penalty vanish,
and the queue drains over the next ticks.

**Single-lane LaneClose.** On a 1-lane edge, LaneClose has no open lane to merge
into; `closedLaneFor` returns the lone lane and the obstacle blocks it like a
FullClose — the documented degenerate case.

## Cross-cutting

### Determinism

Preserved for runs without manual injection. `applyIncident` uses no rng and no
wall-clock; given the same sequence of `IncidentEvent`s at the same ticks, the
sim is reproducible. `TestWorld_TraceDeterminism` injects nothing and stays
byte-identical. Manual UI injection is an external input (like the existing
signal-mode clicks) and is recorded in the trace for faithful replay; it is not
expected to be reproducible from seed alone, consistent with how `Control`
events are already treated. Incident-logic tests drive `applyIncident`
programmatically.

### Performance

Negligible. `edgeCost` and the per-vehicle incident check are O(1) map lookups
on a map that holds only *active* incidents (typically 0). `publishSnapshot`
copies that small map. No new per-edge or per-vehicle allocation in the common
(no-incident) case. The 50 ms tick budget is unaffected.

### Trace format

One new event kind (10), additive and forward-compatible. Existing kinds and
`tracereplay` of old traces are unaffected.

### Known limitations (documented, accepted)

- **FullClose traps on-edge vehicles.** Cars already on a fully-closed edge
  queue at the closed (downstream) end rather than draining; the stuck-timeout
  eventually despawns truly wedged non-GPS cars. Acceptable for an interactive
  demo tool.
- **Edge-level granularity.** An incident applies to a whole edge, not a point
  partway along it — matching the per-edge congestion/routing model and the
  click-an-edge UX.
- **Headless mode.** Incidents are injected only through the viewer, so a
  `--headless` run has none. Programmatic injection (tests) still works.
- **Curb-lane only for LaneClose.** v1 always closes lane 0; choosing which
  lane to close is a deferred enhancement.

## Testing

### New tests — `internal/sim/incident_test.go`

- `TestEdgeCost_FullCloseAvoided` — `edgeCost` returns the large finite cost for
  FullClose and a multiple of `Cong.Cost` for Slowdown/LaneClose; no incident
  returns the bare congestion cost.
- `TestApplyIncident_SetClear` — set then clear an edge; map reflects each
  transition; out-of-range EdgeID is ignored; an `IncidentSet` event is emitted
  per call.
- `TestComputeDesiredSpeed_SlowdownCap` — a Slowdown edge caps `v0` to
  `incidentSlowdownFactor × limit`; non-Slowdown edges are unchanged.
- `TestIncidentVirtualLeader_Brakes` — a vehicle in the closed lane (or any
  vehicle on a FullClose edge) gets a stopped virtual leader at the edge end and
  decelerates.
- `TestClosedLaneFor_SingleLane` — LaneClose on a 1-lane edge reports the lone
  lane closed (degenerate full block).

### New / changed tests — `internal/sim/world_test.go`

- `TestWorld_FullClose_GPSReroutes` — small grid, FullClose the direct edge,
  spawn a GPS vehicle; it adopts an alternate tail (and emits `VehicleReroute`).
- `TestWorld_LaneClose_CarsMergeOut` — multi-lane edge, LaneClose lane 0, a car
  starting in lane 0 changes to an open lane before the edge end.
- `TestWorld_Incident_DrainOnClear` — set FullClose, then clear; the obstacle
  and penalty are gone and queued cars resume.
- `TestWorld_TraceDeterminism` (existing) — must still pass unchanged.

### New tests — `internal/sim/lanechange_test.go`

- `TestLaneChange_VacatesClosedLane` — with `closedLane` set, a car in that lane
  moves out when a safe gap exists; with `closedLane = -1`, behavior is
  unchanged from the baseline fixture.

### New test — `internal/trace` round-trip

- `TestWriteRead_IncidentSet` — write then read `IncidentSet` for each severity
  including the clear (0); fields survive the round-trip.

### New test — `internal/snapshot`

- Extend the existing constant-match guard so `snapshot.Sev*` equals the
  corresponding `sim.Severity` values.

### New test — `internal/render`

- `TestHitTestEdge` — a cursor near a known segment returns that edge; a cursor
  far from any edge returns `false`.

## Files changed

- `internal/sim/incident.go` — **new**: `Severity`, `IncidentEvent`, constants,
  `edgeCost`, `closedLaneFor`, `applyIncident`.
- `internal/sim/world.go` — `Incidents` map + `IncidentControl` channel fields;
  init in `NewWorld`; drain in `Step`; incident virtual leader; `edgeCost` in
  `maybeReroute`; `edgeCost` in `trySpawn` (GPS branch); `closedLane` arg to
  `tryLaneChange`; fill `Snapshot.Incidents` in `publishSnapshot`.
- `internal/sim/cornering.go` — Slowdown `v0` cap in `computeDesiredSpeed`.
- `internal/sim/lanechange.go` — `closedLane` parameter + vacate logic.
- `internal/snapshot/snapshot.go` — `IncidentView`, `Snapshot.Incidents`,
  severity constants.
- `internal/render/viewport.go` — `hitTestEdge`, Shift+click handling,
  `OnIncident` callback, severity overlay in `drawRoadBands`.
- `internal/render/hud.go` — incident count + control hint.
- `internal/trace/events.go` — `KindIncidentSet`, `IncidentSet`.
- `internal/trace/writer.go` / `reader.go` — encode/decode the new event.
- `cmd/trafficsim/main.go` — create `incidentCh`, set `w.IncidentControl`, wire
  `vp.OnIncident` in `runLive`.
- `cmd/tracereplay/player.go` — track incidents, apply `IncidentSet`, include in
  `publish`.
- Tests: `incident_test.go` (new), `world_test.go`, `lanechange_test.go`,
  `snapshot` guard, `render` hit-test, `trace` round-trip.
- `README.md` — document interactive incidents and the Shift+click control.

## Out of scope (deferred)

- **Config-file and random incidents.** Only interactive injection in v1; a
  scripted YAML scenario and seeded random incidents are natural follow-ups that
  reuse the same `applyIncident` path (and would be fully deterministic).
- **Fixed-duration / auto-clearing incidents.** Manual clear only.
- **Sub-edge (point) incidents and incident position along the edge.**
- **Choosing which lane to close** (curb lane only in v1) and multi-lane
  closures.
- **Severity selection menu / number-key UX.** Click-cycling only.
- **Emergency-vehicle preemption, weather, and other incident-adjacent
  features.** Separate features.

## Known follow-ups

- A scripted-incidents YAML (like `--signals`) would make incident scenarios
  reproducible and headless-runnable — the apply path and trace event already
  support it.
- Populating `MetricsTick.CongestionIdx` (already dormant) would let offline
  analysis quantify an incident's network-wide impact.
