# Interactive Incidents Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a user inject road incidents (Slowdown / LaneClose / FullClose) by Shift+clicking an edge in the live viewer, so traffic slows, merges, queues, and reroutes around the disruption — recorded to the trace for faithful replay.

**Architecture:** A per-edge `World.Incidents` map drives three effects that reuse existing machinery: a desired-speed cap (Slowdown), a virtual stopped obstacle at the edge's downstream end (LaneClose for the closed lane / FullClose for all lanes), and a routing-cost penalty so GPS vehicles avoid the edge. Injection mirrors the existing UI→sim `Control` channel; a new `IncidentSet` trace event keeps `tracereplay` faithful.

**Tech Stack:** Go 1.x, Ebitengine v2 (viewer), the project's TSIM binary trace format. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-05-22-incidents-design.md`

**Conventions for every commit:** end the commit message body with:
```
Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
```
Work on the existing `feat/incidents` branch (already checked out; the spec commit is already there).

---

## File Structure

| File | Responsibility | Action |
|---|---|---|
| `internal/trace/events.go` | `KindIncidentSet`, `IncidentSet` event | Modify |
| `internal/trace/writer.go` | Encode `IncidentSet` | Modify |
| `internal/trace/reader.go` | Decode `IncidentSet` | Modify |
| `internal/trace/incident_test.go` | Round-trip test | Create |
| `internal/snapshot/snapshot.go` | `IncidentView`, `Snapshot.Incidents`, `Sev*` constants | Modify |
| `internal/sim/incident.go` | `Severity`, `IncidentEvent`, constants, `edgeCost`, `closedLaneFor`, `incidentStopDistance`, `applyIncident` | Create |
| `internal/sim/world.go` | `Incidents`/`IncidentControl` fields, `NewWorld` init, `Step` drain + virtual leader, `edgeCost` routing, lane-change wiring, snapshot fill | Modify |
| `internal/sim/cornering.go` | Slowdown `v0` cap | Modify |
| `internal/sim/lanechange.go` | `closedLane` parameter + vacate logic | Modify |
| `internal/sim/incident_test.go` | Sim incident unit + behavioral tests, constant guard | Create |
| `internal/sim/lanechange_test.go` | Vacate-closed-lane test | Modify |
| `internal/render/viewport.go` | `hitTestEdge`, `segDist2`, Shift+click, `OnIncident`, overlay | Modify |
| `internal/render/viewport_test.go` | `hitTestEdge` / `segDist2` / `nextSeverity` tests | Create |
| `internal/render/hud.go` | Incident count line | Modify |
| `cmd/trafficsim/main.go` | Create `incidentCh`, wire `OnIncident` | Modify |
| `cmd/tracereplay/player.go` | Apply `IncidentSet`, render overlay | Modify |
| `README.md` | Document the feature + control | Modify |

Tasks are ordered bottom-up so each leaves the tree compiling and green.

---

## Task 1: Trace event `IncidentSet`

**Files:**
- Modify: `internal/trace/events.go`
- Modify: `internal/trace/writer.go:142-143` (after the `*TraceDropped` case)
- Modify: `internal/trace/reader.go:160-165` (after the `KindTraceDropped` case)
- Test: `internal/trace/incident_test.go` (create)

- [ ] **Step 1: Write the failing round-trip test**

Create `internal/trace/incident_test.go`:

```go
package trace

import (
	"bytes"
	"testing"
)

func TestWriteRead_IncidentSet(t *testing.T) {
	cases := []IncidentSet{
		{EdgeID: 0, Severity: 0},   // clear
		{EdgeID: 7, Severity: 1},   // slowdown
		{EdgeID: 42, Severity: 2},  // lane close
		{EdgeID: 999, Severity: 3}, // full close
	}
	for _, want := range cases {
		var buf bytes.Buffer
		w := NewWriter(&buf)
		if err := w.Write(5, 1.25, &want); err != nil {
			t.Fatalf("write: %v", err)
		}
		r := NewReader(&buf)
		_, ev, err := r.Next()
		if err != nil {
			t.Fatalf("read: %v", err)
		}
		got, ok := ev.(*IncidentSet)
		if !ok {
			t.Fatalf("decoded type = %T, want *IncidentSet", ev)
		}
		if got.EdgeID != want.EdgeID || got.Severity != want.Severity {
			t.Fatalf("round-trip = %+v, want %+v", *got, want)
		}
	}
}
```

- [ ] **Step 2: Run it to confirm it fails to compile**

Run: `go test ./internal/trace/ -run TestWriteRead_IncidentSet`
Expected: FAIL — `undefined: IncidentSet`.

- [ ] **Step 3: Add the event type**

In `internal/trace/events.go`, add the kind constant after `KindVehicleReroute Kind = 9` (line 33):

```go
	// KindIncidentSet records that the incident on an edge was set or cleared
	// at runtime (interactive injection in the viewer).
	KindIncidentSet Kind = 10
```

And add the type after the `VehicleReroute` definition (after line 132):

```go
// IncidentSet records that the incident on an edge was set or cleared at
// runtime. Severity 0 is a clear; 1/2/3 are Slowdown/LaneClose/FullClose,
// matching sim.Severity. Replayers track the latest severity per edge.
type IncidentSet struct {
	EdgeID   uint32
	Severity uint8
}

func (*IncidentSet) Kind() Kind { return KindIncidentSet }
```

- [ ] **Step 4: Add the encoder**

In `internal/trace/writer.go`, add a case after the `*TraceDropped` case (after line 143):

```go
	case *IncidentSet:
		if err := binary.Write(b, le, ev.EdgeID); err != nil {
			return err
		}
		return binary.Write(b, le, ev.Severity)
```

- [ ] **Step 5: Add the decoder**

In `internal/trace/reader.go`, add a case after the `KindTraceDropped` case (after line 165):

```go
	case KindIncidentSet:
		e := &IncidentSet{}
		if err := binary.Read(rd, le, &e.EdgeID); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.Severity); err != nil {
			return nil, err
		}
		return e, nil
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `go test ./internal/trace/ -run TestWriteRead_IncidentSet -v`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add internal/trace/events.go internal/trace/writer.go internal/trace/reader.go internal/trace/incident_test.go
git commit -m "$(cat <<'EOF'
feat(trace): add IncidentSet event (kind 10)

Records interactive incident set/clear per edge so tracereplay can
reconstruct incidents at the ticks they occurred.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Snapshot incident view + constants

**Files:**
- Modify: `internal/snapshot/snapshot.go`

This task adds data types only (a struct, a slice field, and constants). The correctness guard that the constants match `sim.Severity` lives in Task 3, where `sim.Severity` exists (snapshot must not import sim).

- [ ] **Step 1: Add severity constants**

In `internal/snapshot/snapshot.go`, after the signal-mode constant block (after line 29):

```go
// Incident severities used in IncidentView.Severity. Kept here (not in sim/)
// so the renderer and replayer can switch on them without importing sim. The
// values match sim.Severity exactly; a sim-package test guards the match,
// mirroring the signal-mode constants above.
const (
	SevNone      uint8 = 0
	SevSlowdown  uint8 = 1
	SevLaneClose uint8 = 2
	SevFullClose uint8 = 3
)
```

- [ ] **Step 2: Add the IncidentView type**

After the `SignalView` struct (after line 64):

```go
// IncidentView is one active incident, for rendering an edge overlay.
type IncidentView struct {
	EdgeID   uint32
	Severity uint8 // Sev* constants
}
```

- [ ] **Step 3: Add the Snapshot field**

In the `Snapshot` struct (lines 31-37), add `Incidents` after `Signals`:

```go
type Snapshot struct {
	Tick      uint64
	SimTime   float64
	Vehicles  []VehicleView
	Signals   []SignalView
	Incidents []IncidentView
	Bounds    network.BoundingBox
}
```

- [ ] **Step 4: Verify the package still builds and tests pass**

Run: `go test ./internal/snapshot/`
Expected: PASS (existing `TestDoubleBuffer_SwapIsAtomic` unaffected).

- [ ] **Step 5: Commit**

```bash
git add internal/snapshot/snapshot.go
git commit -m "$(cat <<'EOF'
feat(snapshot): add IncidentView and Snapshot.Incidents

Severity constants mirror sim.Severity (guarded by a sim-package test)
so the renderer and replayer can draw incident overlays without
importing sim.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: Sim incident core — state, cost, apply

**Files:**
- Create: `internal/sim/incident.go`
- Modify: `internal/sim/world.go` (struct fields, `NewWorld`, `Step` drain)
- Test: `internal/sim/incident_test.go` (create)

- [ ] **Step 1: Write the failing tests**

Create `internal/sim/incident_test.go`:

```go
package sim

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// incidentTestNet: one 2-lane edge, 200m, 10 m/s.
func incidentTestNet() *network.Network {
	return &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
				Lanes: []network.Lane{{Index: 0}, {Index: 1}}},
		},
	}
}

func TestSeverityConstantsMatchSnapshot(t *testing.T) {
	if uint8(SeverityNone) != snapshot.SevNone ||
		uint8(Slowdown) != snapshot.SevSlowdown ||
		uint8(LaneClose) != snapshot.SevLaneClose ||
		uint8(FullClose) != snapshot.SevFullClose {
		t.Fatal("sim.Severity values must match snapshot.Sev* constants")
	}
}

func TestEdgeCost_BySeverity(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	base := w.Cong.Cost(net, 0)

	if got := w.edgeCost(0); math.Abs(got-base) > 1e-9 {
		t.Fatalf("no incident: edgeCost=%v want base=%v", got, base)
	}
	w.Incidents[0] = Slowdown
	if got := w.edgeCost(0); math.Abs(got-base*incidentSlowdownCostMul) > 1e-9 {
		t.Fatalf("slowdown: edgeCost=%v want %v", got, base*incidentSlowdownCostMul)
	}
	w.Incidents[0] = LaneClose
	if got := w.edgeCost(0); math.Abs(got-base*incidentLaneCloseCostMul) > 1e-9 {
		t.Fatalf("laneclose: edgeCost=%v want %v", got, base*incidentLaneCloseCostMul)
	}
	w.Incidents[0] = FullClose
	if got := w.edgeCost(0); got != incidentFullCloseCost {
		t.Fatalf("fullclose: edgeCost=%v want %v", got, incidentFullCloseCost)
	}
}

func TestApplyIncident_SetClear(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	var events []*trace.IncidentSet
	w.EmitTrace = func(_ uint64, _ float64, e trace.Event) {
		if is, ok := e.(*trace.IncidentSet); ok {
			events = append(events, is)
		}
	}

	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: FullClose})
	if w.Incidents[0] != FullClose {
		t.Fatalf("after set: severity=%d want FullClose", w.Incidents[0])
	}
	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: SeverityNone})
	if _, present := w.Incidents[0]; present {
		t.Fatal("after clear: edge should be absent from the map")
	}
	// Out-of-range edge id is ignored and emits nothing.
	w.applyIncident(IncidentEvent{EdgeID: 9999, Severity: FullClose})

	if len(events) != 2 {
		t.Fatalf("emitted %d IncidentSet events, want 2 (set + clear)", len(events))
	}
}

func TestIncidentStopDistance_Blocks(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	v := &Vehicle{Edge: 0, Lane: 0, S: 50}

	if _, ok := w.incidentStopDistance(v); ok {
		t.Fatal("no incident should not block")
	}
	w.Incidents[0] = FullClose
	if d, ok := w.incidentStopDistance(v); !ok || math.Abs(d-150) > 1e-9 {
		t.Fatalf("FullClose got (%.2f,%v) want (150,true)", d, ok)
	}
	w.Incidents[0] = LaneClose // closes curb lane 0
	if d, ok := w.incidentStopDistance(v); !ok || math.Abs(d-150) > 1e-9 {
		t.Fatalf("LaneClose lane0 got (%.2f,%v) want (150,true)", d, ok)
	}
	v.Lane = 1
	if _, ok := w.incidentStopDistance(v); ok {
		t.Fatal("LaneClose should not block a vehicle in the open lane")
	}
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `go test ./internal/sim/ -run 'TestSeverity|TestEdgeCost|TestApplyIncident|TestIncidentStopDistance'`
Expected: FAIL — `undefined: SeverityNone`, `w.edgeCost`, `w.Incidents`, etc.

- [ ] **Step 3: Create the incident core file**

Create `internal/sim/incident.go`:

```go
package sim

import (
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// Severity is the kind/intensity of a road incident on an edge. Values match
// the snapshot.Sev* constants (guarded by TestSeverityConstantsMatchSnapshot).
type Severity uint8

const (
	SeverityNone Severity = 0 // no incident / clear
	Slowdown     Severity = 1
	LaneClose    Severity = 2
	FullClose    Severity = 3
)

// IncidentEvent is a UI->sim command to set (or clear, with SeverityNone) the
// incident on an edge. Delivered over World.IncidentControl, mirroring
// ControlEvent / World.Control.
type IncidentEvent struct {
	EdgeID   network.EdgeID
	Severity Severity
}

const (
	// incidentSlowdownFactor caps desired speed on a Slowdown edge to this
	// fraction of its limit (a hazard crawl).
	incidentSlowdownFactor = 0.3

	// Routing-cost penalties used by edgeCost. Slowdown/LaneClose multiply the
	// base (congestion) cost so GPS reroutes promptly without waiting for the
	// EWMA; FullClose uses a large finite cost so the edge is avoided but still
	// selectable as a last resort (mirrors congestion.go's minEdgeSpeed floor).
	incidentSlowdownCostMul  = 1.5
	incidentLaneCloseCostMul = 3.0
	incidentFullCloseCost    = 1e9
)

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

// closedLaneFor returns the closed lane index and true when the edge has a
// LaneClose incident. v1 always closes the curb lane (index 0). FullClose is
// handled by incidentStopDistance (all lanes), not here.
func (w *World) closedLaneFor(eid network.EdgeID) (uint8, bool) {
	if w.Incidents[eid] != LaneClose {
		return 0, false
	}
	if len(w.Net.Edges[eid].Lanes) == 0 {
		return 0, false
	}
	return 0, true
}

// incidentStopDistance returns (distance from the vehicle's front bumper to
// the incident obstacle, true) when the vehicle is blocked by an incident on
// its current edge — a full closure (all lanes) or a lane closure of the
// vehicle's lane. The obstacle sits at the edge's downstream end. Mirrors
// stopDistanceForRed's shape so Step can fold it into the virtual-leader set.
func (w *World) incidentStopDistance(v *Vehicle) (float64, bool) {
	sev := w.Incidents[v.Edge]
	blocked := sev == FullClose
	if sev == LaneClose {
		if cl, ok := w.closedLaneFor(v.Edge); ok && v.Lane == cl {
			blocked = true
		}
	}
	if !blocked {
		return 0, false
	}
	d := w.Net.Edges[v.Edge].Length - v.S
	if d < 0 {
		d = 0
	}
	return d, true
}

// applyIncident sets or clears the incident on an edge and records it. Out-of-
// range edge ids are ignored (defensive, like applyControl).
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

- [ ] **Step 4: Add World fields**

In `internal/sim/world.go`, in the `World` struct, add after the `Cong *Congestion` field (after line 70):

```go
	// Incidents maps an edge to its active incident severity. Absent key means
	// no incident. Owned by the sim goroutine; read by publishSnapshot.
	Incidents map[network.EdgeID]Severity

	// IncidentControl delivers runtime incident commands from the UI. Step
	// drains it non-blocking at the top of each tick, like Control. Nil
	// disables.
	IncidentControl <-chan IncidentEvent
```

- [ ] **Step 5: Initialize the map in NewWorld**

In `NewWorld`, in the returned `&World{...}` literal, add after `Cong: NewCongestion(...)` (after line 158):

```go
		Incidents:    make(map[network.EdgeID]Severity),
```

- [ ] **Step 6: Drain the incident channel in Step**

In `internal/sim/world.go` `Step`, immediately after the existing `Control` drain block (after line 639), add:

```go
	// 0a-bis. Drain pending incident commands from the UI, like Control.
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

- [ ] **Step 7: Run the tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestSeverity|TestEdgeCost|TestApplyIncident|TestIncidentStopDistance' -v`
Expected: PASS (4 tests).

- [ ] **Step 8: Verify the whole sim package still builds and passes**

Run: `go test ./internal/sim/`
Expected: PASS (no regressions; `edgeCost`, `closedLaneFor`, `incidentStopDistance` are defined but not yet wired into stepping — that's fine, Go allows unused methods).

- [ ] **Step 9: Commit**

```bash
git add internal/sim/incident.go internal/sim/world.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): add per-edge incident state, cost, and apply path

World.Incidents + IncidentControl mirror the Control channel. edgeCost
folds an incident penalty into congestion routing; incidentStopDistance
mirrors stopDistanceForRed; applyIncident records each set/clear.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Slowdown desired-speed cap

**Files:**
- Modify: `internal/sim/cornering.go:73` (`computeDesiredSpeed`)
- Test: `internal/sim/incident_test.go` (append)

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/incident_test.go`:

```go
func TestComputeDesiredSpeed_SlowdownCap(t *testing.T) {
	net := incidentTestNet() // 10 m/s limit
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	v := &Vehicle{Edge: 0, Lane: 0, S: 0, Route: []network.EdgeID{0}}

	if got := w.computeDesiredSpeed(v); math.Abs(got-10) > 1e-9 {
		t.Fatalf("no incident: v0=%v want 10", got)
	}
	w.Incidents[0] = Slowdown
	want := 10.0 * incidentSlowdownFactor
	if got := w.computeDesiredSpeed(v); math.Abs(got-want) > 1e-9 {
		t.Fatalf("slowdown: v0=%v want %v", got, want)
	}
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `go test ./internal/sim/ -run TestComputeDesiredSpeed_SlowdownCap`
Expected: FAIL — slowdown case returns 10, not 3.

- [ ] **Step 3: Apply the cap**

In `internal/sim/cornering.go`, in `computeDesiredSpeed`, replace the line that computes `v0` (line 73) so the cap is applied before the last-edge early return:

```go
	v0 := edge.SpeedLimit * factor
	if w.Incidents[v.Edge] == Slowdown {
		if cap := edge.SpeedLimit * incidentSlowdownFactor; cap < v0 {
			v0 = cap
		}
	}
```

(The existing `if v.RouteIdx+1 >= len(v.Route) { return v0 }` and corner-cap logic below stay unchanged — the smaller of the slowdown cap and corner cap wins.)

- [ ] **Step 4: Run the test to verify it passes**

Run: `go test ./internal/sim/ -run TestComputeDesiredSpeed_SlowdownCap -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/cornering.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): cap desired speed on Slowdown incident edges

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: Incident virtual leader in Step

**Files:**
- Modify: `internal/sim/world.go` (per-vehicle stepping loop, near line 778-785)
- Test: `internal/sim/incident_test.go` (append)

- [ ] **Step 1: Write the failing behavioral test**

Append to `internal/sim/incident_test.go`:

```go
func TestWorld_FullClose_VehicleStopsBeforeEnd(t *testing.T) {
	// One 1-lane edge; a car starts 80m from the end at speed and must stop
	// at the FullClose obstacle (edge end) instead of running off the edge.
	net := &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
				Lanes: []network.Lane{{Index: 0}}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = FullClose
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 120, V: 15}}
	w.nextID = 2

	// 100 ticks = 5s: enough to brake to a stop, less than stuckTimeoutSec so
	// the stuck-guard hasn't despawned the (legitimately blocked) car yet.
	for i := 0; i < 100; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			t.Fatal("vehicle despawned too early; expected it blocked at the closure")
		}
	}
	v := &w.Vehicles[0]
	if v.S > 200.0 {
		t.Fatalf("vehicle ran past the closure: S=%.2f (edge len 200)", v.S)
	}
	if v.V > 1.0 {
		t.Fatalf("vehicle should be ~stopped at the closure: V=%.2f", v.V)
	}
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `go test ./internal/sim/ -run TestWorld_FullClose_VehicleStopsBeforeEnd`
Expected: FAIL — with no virtual leader the car crosses S=200 and despawns (the `len==0` fatal fires or `S>200`).

- [ ] **Step 3: Fold the incident virtual leader into Step**

In `internal/sim/world.go`, in the per-vehicle loop, after the left-turn opposing-traffic virtual-leader block (after line 785, before `prevEdge := v.Edge`), add:

```go
		// Apply incident virtual leader (stopped obstacle at the edge end) if
		// closer. Full closure blocks every lane; a lane closure blocks only
		// the vehicle's lane.
		dInc, incBlocked := w.incidentStopDistance(v)
		if incBlocked {
			virtualS := v.S + dInc
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `go test ./internal/sim/ -run TestWorld_FullClose_VehicleStopsBeforeEnd -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/world.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): brake vehicles at incident obstacle via virtual leader

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Incident-aware routing (reroute + spawn)

**Files:**
- Modify: `internal/sim/world.go:903` (`maybeReroute` cost function)
- Modify: `internal/sim/world.go` (`trySpawn` GPS routing — see Step 3)
- Test: `internal/sim/incident_test.go` (append)

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/incident_test.go`:

```go
func TestWorld_FullClose_GPSReroutes(t *testing.T) {
	net := buildRerouteGraph() // route [0,1] direct; detour [0,2,3]; dest node 3
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[1] = FullClose // close the direct edge

	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: true, DestNode: 3, LastRerouteSec: -1000,
	}
	if !w.maybeReroute(v) {
		t.Fatalf("maybeReroute returned false for an eligible GPS vehicle")
	}
	if len(v.Route) != 3 || v.Route[1] != 2 || v.Route[2] != 3 {
		t.Fatalf("route after reroute = %v, want [0 2 3] around the closure", v.Route)
	}
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `go test ./internal/sim/ -run TestWorld_FullClose_GPSReroutes`
Expected: FAIL — `maybeReroute` still uses bare congestion cost, so the closed edge looks cheap and no switch happens.

- [ ] **Step 3: Switch routing to edgeCost**

In `internal/sim/world.go` `maybeReroute`, replace the cost function (line 903):

```go
	costFn := func(eid network.EdgeID) float64 { return w.edgeCost(eid) }
```

Then in `trySpawn`, find the GPS routing branch that calls `w.Router.RouteCost(... w.Cong.Cost(w.Net, eid) ...)` (the closure passed for GPS vehicles) and replace its body so it uses `edgeCost`:

```go
		route, err = w.Router.RouteCost(r.OriginNode, r.DestNode, func(eid network.EdgeID) float64 {
			return w.edgeCost(eid)
		})
```

(Leave the non-GPS `w.Router.Route(...)` static branch unchanged.)

- [ ] **Step 4: Run the test to verify it passes**

Run: `go test ./internal/sim/ -run TestWorld_FullClose_GPSReroutes -v`
Expected: PASS.

- [ ] **Step 5: Run the existing reroute suite to confirm no regressions**

Run: `go test ./internal/sim/ -run TestWorld_Reroute`
Expected: PASS (all existing reroute tests still green — edgeCost equals congestion cost when no incident is set).

- [ ] **Step 6: Commit**

```bash
git add internal/sim/world.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): route around incidents via edgeCost in reroute and spawn

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: Forced merge-out of closed lane

**Files:**
- Modify: `internal/sim/lanechange.go:27` (`tryLaneChange` signature + vacate branch)
- Modify: `internal/sim/world.go:854` (call site)
- Test: `internal/sim/lanechange_test.go` (append)

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/lanechange_test.go`:

```go
func TestLaneChange_VacatesClosedLane(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 1000, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 1000, SpeedLimit: 20,
			Lanes: []network.Lane{{Index: 0}, {Index: 1}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	// Car in the closed curb lane (0); open lane (1) is empty -> must vacate.
	vs := []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 100, V: 10}}
	tryLaneChange(&vs[0], 0, map[uint8][]int{0: {0}}, vs, net, 0)
	if vs[0].Lane != 1 {
		t.Fatalf("car in closed lane should move to lane 1, got %d", vs[0].Lane)
	}

	// Baseline: no incident (closedLane = -1), no slow leader -> no change.
	vs2 := []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 100, V: 10}}
	tryLaneChange(&vs2[0], 0, map[uint8][]int{0: {0}}, vs2, net, -1)
	if vs2[0].Lane != 0 {
		t.Fatalf("no incident: lane should be unchanged, got %d", vs2[0].Lane)
	}
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `go test ./internal/sim/ -run TestLaneChange_VacatesClosedLane`
Expected: FAIL to compile — `tryLaneChange` takes 5 args, test passes 6.

- [ ] **Step 3: Add the parameter and vacate branch**

In `internal/sim/lanechange.go`, change the signature (line 27):

```go
func tryLaneChange(v *Vehicle, vi int, laneVehicles map[uint8][]int, vs []Vehicle, net *network.Network, closedLane int8) {
```

Then, immediately after the `numLanes < 2` early return (after line 35), insert the vacate branch:

```go
	// Incident: ego is in a closed lane — vacate to an adjacent open lane as
	// soon as a safe gap exists (overrides the normal speed/turn incentive,
	// but never overrides the safety gaps).
	if closedLane >= 0 && v.Lane == uint8(closedLane) {
		for _, dl := range []int8{1, -1} {
			nl := int(v.Lane) + int(dl)
			if nl < 0 || nl >= int(numLanes) || nl == int(closedLane) {
				continue
			}
			other := laneVehicles[uint8(nl)]
			frontS, hasFront := nextAheadS(other, vs, v.Edge, v.S)
			rearS, hasRear := nextBehindS(other, vs, v.Edge, v.S)
			if hasFront && frontS-v.S-VehicleLength < safetyGapFront {
				continue
			}
			if hasRear && v.S-rearS-VehicleLength < safetyGapRear {
				continue
			}
			v.Lane = uint8(nl)
			v.LaneChangeCooldown = laneChangeCooldown
			v.LastLCDir = dl
			return
		}
		return // blocked in the closed lane; don't fall through to normal LC
	}
```

- [ ] **Step 4: Update the call site**

In `internal/sim/world.go`, replace the lane-change call (line 854) with:

```go
		if lanes, ok := byEdgeLane[v.Edge]; ok {
			cl := int8(-1)
			if c, ok := w.closedLaneFor(v.Edge); ok {
				cl = int8(c)
			}
			tryLaneChange(v, i, lanes, w.Vehicles, w.Net, cl)
		}
```

- [ ] **Step 5: Run the new test and the existing lane-change suite**

Run: `go test ./internal/sim/ -run TestLaneChange -v`
Expected: PASS — `TestLaneChange_VacatesClosedLane` plus all existing lane-change tests (the new `closedLane int8` is -1 in their paths via the call site, so behavior is unchanged).

- [ ] **Step 6: Commit**

```bash
git add internal/sim/lanechange.go internal/sim/world.go internal/sim/lanechange_test.go
git commit -m "$(cat <<'EOF'
feat(sim): force vehicles to merge out of a closed lane

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Publish incidents into the snapshot

**Files:**
- Modify: `internal/sim/world.go:1108` (`publishSnapshot` Publish literal)
- Test: `internal/sim/incident_test.go` (append)

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/incident_test.go`:

```go
func TestPublishSnapshot_IncludesIncidents(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = Slowdown

	w.publishSnapshot()
	snap := w.SnapshotBuf.Read()
	if len(snap.Incidents) != 1 ||
		snap.Incidents[0].EdgeID != 0 ||
		snap.Incidents[0].Severity != snapshot.SevSlowdown {
		t.Fatalf("snapshot incidents = %+v, want one Slowdown on edge 0", snap.Incidents)
	}
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `go test ./internal/sim/ -run TestPublishSnapshot_IncludesIncidents`
Expected: FAIL — `snap.Incidents` is empty (not yet populated).

- [ ] **Step 3: Build and attach the incident views**

In `internal/sim/world.go` `publishSnapshot`, just before the `w.SnapshotBuf.Publish(...)` call (before line 1108), add:

```go
	incidents := make([]snapshot.IncidentView, 0, len(w.Incidents))
	for eid, sev := range w.Incidents {
		incidents = append(incidents, snapshot.IncidentView{
			EdgeID: uint32(eid), Severity: uint8(sev),
		})
	}
```

Then add `Incidents: incidents,` to the `snapshot.Snapshot{...}` literal:

```go
	w.SnapshotBuf.Publish(snapshot.Snapshot{
		Tick: w.Tick, SimTime: w.SimTime,
		Vehicles: views, Signals: sigs, Incidents: incidents, Bounds: w.Net.Bounds,
	})
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `go test ./internal/sim/ -run TestPublishSnapshot_IncludesIncidents -v`
Expected: PASS.

- [ ] **Step 5: Run the full sim package**

Run: `go test ./internal/sim/`
Expected: PASS (including `TestWorld_TraceDeterminism`).

- [ ] **Step 6: Commit**

```bash
git add internal/sim/world.go internal/sim/incident_test.go
git commit -m "$(cat <<'EOF'
feat(sim): publish active incidents in the render snapshot

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Viewer — edge hit-test, Shift+click injection, overlay

**Files:**
- Modify: `internal/render/viewport.go` (struct field, `Update`, helpers, `Draw`)
- Test: `internal/render/viewport_test.go` (create)

- [ ] **Step 1: Write the failing tests**

Create `internal/render/viewport_test.go`:

```go
package render

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

func TestSegDist2(t *testing.T) {
	// Segment (0,0)-(10,0). Point (5,3) is 3 away -> d2=9. Point (-5,0) is
	// off the end, nearest is (0,0) -> d2=25.
	if got := segDist2(5, 3, 0, 0, 10, 0); got != 9 {
		t.Fatalf("segDist2 perpendicular = %v, want 9", got)
	}
	if got := segDist2(-5, 0, 0, 0, 10, 0); got != 25 {
		t.Fatalf("segDist2 past-end = %v, want 25", got)
	}
}

func TestNextSeverity_Cycles(t *testing.T) {
	got := []uint8{
		nextSeverity(snapshot.SevNone),
		nextSeverity(snapshot.SevSlowdown),
		nextSeverity(snapshot.SevLaneClose),
		nextSeverity(snapshot.SevFullClose),
	}
	want := []uint8{
		snapshot.SevSlowdown, snapshot.SevLaneClose,
		snapshot.SevFullClose, snapshot.SevNone,
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("nextSeverity step %d = %d, want %d", i, got[i], want[i])
		}
	}
}

func TestHitTestEdge(t *testing.T) {
	net := &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10,
				Lanes:    []network.Lane{{Index: 0}},
				Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}}},
		},
		Bounds: network.BoundingBox{MinX: 0, MinY: -50, MaxX: 100, MaxY: 50},
	}
	vp := NewViewport(net, snapshot.New(), 800, 600)

	sx, sy := vp.toScreen(50, 0) // screen point over the edge midpoint
	if eid, ok := vp.hitTestEdge(int(sx), int(sy)); !ok || eid != 0 {
		t.Fatalf("hitTestEdge over edge = (%d,%v), want (0,true)", eid, ok)
	}
	fx, fy := vp.toScreen(50, 40) // 40m off the edge, beyond the 30m radius
	if _, ok := vp.hitTestEdge(int(fx), int(fy)); ok {
		t.Fatal("hitTestEdge far from any edge should miss")
	}
}
```

- [ ] **Step 2: Run to confirm failure**

Run: `go test ./internal/render/`
Expected: FAIL — `undefined: segDist2`, `nextSeverity`, `vp.hitTestEdge`.

- [ ] **Step 3: Add the OnIncident field**

In `internal/render/viewport.go`, in the `Viewport` struct, after the `OnSetMode` field (after line 146):

```go
	// OnIncident, if non-nil, is invoked when the user Shift+clicks an edge.
	// severity uses the snapshot.Sev* values. Same non-blocking, goroutine-
	// safe contract as OnSetMode (typically pushes onto a channel).
	OnIncident func(edgeID uint32, severity uint8)
```

- [ ] **Step 4: Add the geometry + severity helpers**

In `internal/render/viewport.go`, after `hitTestIntersection` (after line 324), add:

```go
// segDist2 returns the squared distance from point (px,py) to the segment
// (ax,ay)-(bx,by).
func segDist2(px, py, ax, ay, bx, by float64) float64 {
	dx, dy := bx-ax, by-ay
	if dx == 0 && dy == 0 {
		ex, ey := px-ax, py-ay
		return ex*ex + ey*ey
	}
	t := ((px-ax)*dx + (py-ay)*dy) / (dx*dx + dy*dy)
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	cx, cy := ax+t*dx, ay+t*dy
	ex, ey := px-cx, py-cy
	return ex*ex + ey*ey
}

// hitTestEdge returns the nearest edge to the screen-space cursor within a
// 30 m radius (point-to-polyline distance). Mirrors hitTestIntersection.
func (v *Viewport) hitTestEdge(mx, my int) (network.EdgeID, bool) {
	wx := v.camX + (float64(mx)-float64(v.Width)/2)/v.zoom
	wy := v.camY - (float64(my)-float64(v.Height)/2)/v.zoom
	const radius = 30.0
	bestD2 := radius * radius
	var bestID network.EdgeID
	found := false
	for i := range v.Net.Edges {
		e := &v.Net.Edges[i]
		pts := e.Geometry
		if len(pts) < 2 {
			pts = []network.Point{v.Net.Nodes[e.From].Pos, v.Net.Nodes[e.To].Pos}
		}
		for j := 0; j+1 < len(pts); j++ {
			d2 := segDist2(wx, wy, pts[j].X, pts[j].Y, pts[j+1].X, pts[j+1].Y)
			if d2 < bestD2 {
				bestD2 = d2
				bestID = network.EdgeID(i)
				found = true
			}
		}
	}
	return bestID, found
}

// shiftHeld reports whether either Shift key is down.
func shiftHeld() bool {
	return ebiten.IsKeyPressed(ebiten.KeyShiftLeft) || ebiten.IsKeyPressed(ebiten.KeyShiftRight)
}

// severityOf returns the current incident severity on an edge from the latest
// snapshot (snapshot.Sev* value), or SevNone.
func (v *Viewport) severityOf(eid network.EdgeID) uint8 {
	snap := v.Buf.Read()
	for _, inc := range snap.Incidents {
		if inc.EdgeID == uint32(eid) {
			return inc.Severity
		}
	}
	return snapshot.SevNone
}

// nextSeverity advances the click cycle none -> Slowdown -> LaneClose ->
// FullClose -> none.
func nextSeverity(cur uint8) uint8 {
	switch cur {
	case snapshot.SevNone:
		return snapshot.SevSlowdown
	case snapshot.SevSlowdown:
		return snapshot.SevLaneClose
	case snapshot.SevLaneClose:
		return snapshot.SevFullClose
	default:
		return snapshot.SevNone
	}
}
```

- [ ] **Step 5: Wire Shift+click into Update**

In `internal/render/viewport.go` `Update`, replace the "click without drag" block (lines 230-238) with:

```go
		if v.dragging && !v.movedSinceDown {
			// Shift+click cycles an incident on the nearest edge; plain click
			// selects an intersection.
			if shiftHeld() {
				if eid, ok := v.hitTestEdge(mx, my); ok && v.OnIncident != nil {
					v.OnIncident(uint32(eid), nextSeverity(v.severityOf(eid)))
				}
			} else if id, ok := v.hitTestIntersection(mx, my); ok {
				v.selectedID = id
				v.hasSelection = true
			} else {
				v.hasSelection = false
			}
		}
```

- [ ] **Step 6: Add the overlay renderer**

In `internal/render/viewport.go`, add after `drawRoadBands` (after line 516):

```go
// drawIncidents overlays each active-incident edge in a severity color,
// slightly thicker than the road band so it reads as a highlight.
func (v *Viewport) drawIncidents(screen *ebiten.Image, snap snapshot.Snapshot) {
	for _, inc := range snap.Incidents {
		if int(inc.EdgeID) >= len(v.Net.Edges) {
			continue
		}
		e := &v.Net.Edges[inc.EdgeID]
		g := e.Geometry
		if len(g) < 2 {
			continue
		}
		var clr color.RGBA
		switch inc.Severity {
		case snapshot.SevSlowdown:
			clr = color.RGBA{240, 180, 0, 220} // amber
		case snapshot.SevLaneClose:
			clr = color.RGBA{240, 120, 0, 230} // orange
		default:
			clr = color.RGBA{230, 40, 40, 240} // red (full close)
		}
		w := float32(e.Width*v.zoom) + 2
		if w < minRoadStrokePx+2 {
			w = minRoadStrokePx + 2
		}
		for j := 0; j+1 < len(g); j++ {
			x1, y1 := v.toScreen(g[j].X, g[j].Y)
			x2, y2 := v.toScreen(g[j+1].X, g[j+1].Y)
			vector.StrokeLine(screen, x1, y1, x2, y2, w, clr, true)
		}
	}
}
```

Then call it in `Draw`, right after `snap := v.Buf.Read()` (after line 336):

```go
	v.drawIncidents(screen, snap)
```

- [ ] **Step 7: Run the tests to verify they pass**

Run: `go test ./internal/render/ -v`
Expected: PASS — `TestSegDist2`, `TestNextSeverity_Cycles`, `TestHitTestEdge`.

- [ ] **Step 8: Commit**

```bash
git add internal/render/viewport.go internal/render/viewport_test.go
git commit -m "$(cat <<'EOF'
feat(render): inject incidents via Shift+click and draw edge overlay

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 10: HUD incident count

**Files:**
- Modify: `internal/render/hud.go` (`DrawHUD` signature, line count, new line)
- Modify: `internal/render/viewport.go:466` (call site)

- [ ] **Step 1: Update DrawHUD**

In `internal/render/hud.go`, bump the line count constant (line 20):

```go
const hudLineCount = 5
```

Change the `DrawHUD` signature (line 61) to add `incidentCount int`:

```go
func DrawHUD(screen *ebiten.Image, simTime float64, vehicleCount int, incidentCount int, viewWidthM, viewHeightM float64, stats speedStats) {
```

After the existing `line4` definition (line 65-66), add a fifth line and print it. Locate the block that builds `line1..line4` and prints them at `8+N*hudLineHeight`; add:

```go
	line5 := fmt.Sprintf("incidents=%d  (shift+click an edge to cycle)", incidentCount)
```

and, after the existing `ebitenutil.DebugPrintAt(screen, line4, 8, 8+3*hudLineHeight)` (line 70), add:

```go
	ebitenutil.DebugPrintAt(screen, line5, 8, 8+4*hudLineHeight)
```

- [ ] **Step 2: Update the call site**

In `internal/render/viewport.go` `Draw` (line 466), pass the incident count:

```go
	DrawHUD(screen, snap.SimTime, len(snap.Vehicles), len(snap.Incidents), viewWidthM, viewHeightM, stats)
```

- [ ] **Step 3: Build and run render tests**

Run: `go build ./... && go test ./internal/render/`
Expected: PASS (compiles with the new signature; `DrawSelectionPanel` offset uses `hudLineCount`, now 5, so it sits below the new line).

- [ ] **Step 4: Commit**

```bash
git add internal/render/hud.go internal/render/viewport.go
git commit -m "$(cat <<'EOF'
feat(render): show active incident count in the HUD

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 11: Wire the incident channel in trafficsim

**Files:**
- Modify: `cmd/trafficsim/main.go` (`runRun` channel, `runLive` signature + call, `OnIncident` wiring)

- [ ] **Step 1: Create the channel in runRun**

In `cmd/trafficsim/main.go`, after `w.Control = controlCh` (line 229), add:

```go
	// Incident channel from the UI to the sim, mirroring Control. Shift+click
	// on an edge in the viewer cycles its incident severity.
	incidentCh := make(chan sim.IncidentEvent, 32)
	w.IncidentControl = incidentCh
```

- [ ] **Step 2: Pass it to runLive**

Change the `runLive` call (line 270):

```go
	return runLive(ctx, w, controlCh, incidentCh, net)
```

- [ ] **Step 3: Update the runLive signature**

Change `runLive` (line 355):

```go
func runLive(parentCtx context.Context, w *sim.World, controlCh chan<- sim.ControlEvent, incidentCh chan<- sim.IncidentEvent, net *network.Network) error {
```

- [ ] **Step 4: Wire the OnIncident callback**

In `runLive`, after the `vp.OnSetMode = func(...) {...}` block (after line 386), add:

```go
	vp.OnIncident = func(edgeID uint32, severity uint8) {
		select {
		case incidentCh <- sim.IncidentEvent{
			EdgeID:   network.EdgeID(edgeID),
			Severity: sim.Severity(severity),
		}:
		default:
			slog.Warn("incident channel full; dropping incident change")
		}
	}
```

- [ ] **Step 5: Build the binary**

Run: `go build ./cmd/trafficsim/`
Expected: builds with no errors.

- [ ] **Step 6: Commit**

```bash
git add cmd/trafficsim/main.go
git commit -m "$(cat <<'EOF'
feat(trafficsim): wire viewer incident injection to the sim

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 12: Replay incidents in tracereplay

**Files:**
- Modify: `cmd/tracereplay/player.go` (struct field, `newPlayer` init, `apply` case, `publish` fill)

- [ ] **Step 1: Add the incidents map to the player struct**

In `cmd/tracereplay/player.go`, in the `player` struct (after line 37, `signalStates`):

```go
	incidents    map[uint32]uint8 // EdgeID -> snapshot.Sev*
```

- [ ] **Step 2: Initialize it in newPlayer**

In the returned `&player{...}` literal (after line 93, `vehicles: make(...)`):

```go
		incidents:    make(map[uint32]uint8),
```

- [ ] **Step 3: Handle the event in apply**

In `apply`, add a case after the `*trace.VehicleReroute` case (after line 163):

```go
	case *trace.IncidentSet:
		if e.Severity == 0 {
			delete(p.incidents, e.EdgeID)
		} else {
			p.incidents[e.EdgeID] = e.Severity
		}
```

- [ ] **Step 4: Fill the snapshot in publish**

In `publish`, before the `p.buf.Publish(...)` call (before line 249), build the incident views:

```go
	incViews := make([]snapshot.IncidentView, 0, len(p.incidents))
	for eid, sev := range p.incidents {
		incViews = append(incViews, snapshot.IncidentView{EdgeID: eid, Severity: sev})
	}
```

Then add `Incidents: incViews,` to the `snapshot.Snapshot{...}` literal:

```go
	p.buf.Publish(snapshot.Snapshot{
		SimTime:   simTime,
		Vehicles:  views,
		Signals:   sigViews,
		Incidents: incViews,
		Bounds:    p.net.Bounds,
	})
```

- [ ] **Step 5: Build the binary**

Run: `go build ./cmd/tracereplay/`
Expected: builds with no errors.

- [ ] **Step 6: Commit**

```bash
git add cmd/tracereplay/player.go
git commit -m "$(cat <<'EOF'
feat(tracereplay): reconstruct and render incidents on playback

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 13: Documentation

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add an Incidents section**

In `README.md`, after the "### GPS rerouting" section (after line 95, before "### Notes for Windows"), add:

```markdown
### Incidents (interactive)

In the live viewer you can inject road incidents on the fly. **Shift + left-click
an edge** to cycle its incident severity:

`none → slowdown → lane closed → fully closed → none`

- **Slowdown** — traffic crawls through the edge (desired speed capped).
- **Lane closed** — the curb lane is blocked; vehicles merge out of it.
- **Fully closed** — every lane is blocked; through-traffic queues and
  GPS-equipped vehicles reroute around it.

Incidents stay until you clear them (cycle back to `none`). The active count is
shown in the HUD. Each change is written to the trace as a `VehicleReroute`-style
`IncidentSet` event, so `tracereplay` shows incidents appearing and clearing at
the same moments they did live.

Incidents are a viewer-only (interactive) feature; `--headless` runs have none.
A fully-closed edge that a non-GPS vehicle is already committed to will queue it
until the existing stuck-vehicle timeout clears it.
```

- [ ] **Step 2: Verify it renders**

Run: `grep -n "Incidents (interactive)" README.md`
Expected: prints the new heading line.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "$(cat <<'EOF'
docs(readme): document interactive incidents and the shift+click control

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 14: Full verification

**Files:** none (verification only)

- [ ] **Step 1: Build everything**

Run: `go build ./...`
Expected: no output, exit 0.

- [ ] **Step 2: Vet**

Run: `go vet ./...`
Expected: no findings.

- [ ] **Step 3: Run the entire test suite**

Run: `go test ./...`
Expected: all packages PASS.

- [ ] **Step 4: Confirm determinism is intact**

Run: `go test ./internal/sim/ -run TestWorld_TraceDeterminism -v`
Expected: PASS (no-injection runs remain byte-identical).

- [ ] **Step 5: Run the sim benchmarks to confirm no budget regression**

Run: `go test ./internal/sim/ -bench=. -benchtime=1s -run=^$`
Expected: completes; ms/tick figures remain well under the 50 ms budget (incident lookups are O(1) on an empty map in the no-incident benchmark path).

- [ ] **Step 6: Manual smoke test (optional, requires a display)**

Run: `go build -o trafficsim ./cmd/trafficsim/ && ./trafficsim run --spawn-rate 20 --trace /tmp/inc.trace <path-to-osm>`
Then: Shift+click an edge a few times to watch the overlay cycle and traffic respond; quit; replay with `go build -o tracereplay ./cmd/tracereplay/ && ./tracereplay -osm <path-to-osm> -trace /tmp/inc.trace` and confirm the incidents reappear.

- [ ] **Step 7: Final confirmation**

The branch `feat/incidents` now contains the full feature across 13 implementation commits plus the spec. It is ready for `superpowers:finishing-a-development-branch` (merge / PR decision).

---

## Self-Review Notes

- **Spec coverage:** Severity model (Tasks 3-7), interactive injection (Tasks 9, 11), manual clear (Task 9 cycle to none + Task 3 delete), trace + replay (Tasks 1, 12), snapshot/render overlay + HUD (Tasks 2, 9, 10), determinism preserved (Task 14 Step 4), known limitations (documented, Task 13). All spec sections map to a task.
- **Type consistency:** `Severity`/`SeverityNone`/`Slowdown`/`LaneClose`/`FullClose`, `IncidentEvent{EdgeID, Severity}`, `World.Incidents`/`IncidentControl`, `edgeCost`, `closedLaneFor`, `incidentStopDistance`, `applyIncident`, `snapshot.IncidentView`/`Sev*`, `trace.IncidentSet`/`KindIncidentSet`, `tryLaneChange(..., closedLane int8)`, `Viewport.OnIncident`, `segDist2`, `hitTestEdge`, `nextSeverity`, `severityOf`, `player.incidents` — all used consistently across tasks.
- **No placeholders:** every code step shows complete code; every run step shows the command and expected result.
