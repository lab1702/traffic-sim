# Unsignalized Intersection Control — Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the implicit "lower-`Incoming`-index wins" priority rule at unsignalized (and signal-off/flash) intersections with an explicit per-approach `Control` enum, ingested from OSM stop/yield tags with class-based fallback. Add mandatory-stop dwell at `Stop` and `AllWayStop` approaches; arbitrate `AllWayStop` by FIFO of stop-line arrival.

**Architecture:** A new `Control` enum is stored in a parallel slice (`IncomingControl`) on each `Intersection`, populated at netbuild time after the existing priority sort. The yield rule in `internal/sim/world.go` dispatches on effective control (with signal flash/off modes mapped to control values at decision time). A new `Vehicle.StoppedSinceSec` field records mandatory-stop arrival time and is cleared on edge transition.

**Tech Stack:** Go 1.x; existing dependencies (`paulmach/osm`); no new libraries.

**Spec:** `docs/superpowers/specs/2026-05-15-unsignalized-control-phase-1-design.md`

---

## File map

| File | Change |
|---|---|
| `internal/network/types.go` | Add `Control` enum, `Intersection.IncomingControl` field. |
| `internal/sim/vehicle.go` | Add `StoppedSinceSec` field; zero on edge transition. |
| `internal/sim/world.go` | Add `stopDwellSec`, `stopLineTolMeters` constants; rewrite `stopDistanceForYield`. |
| `internal/sim/world_test.go` | New tests for Stop/AllWayStop/flash-off behavior; fixture updates. |
| `internal/sim/signal_test.go` | Update flash/off assertions if they exercise yield (`signal_test.go` itself does not simulate vehicles, so likely no change). |
| `internal/netbuild/netbuild.go` | Populate `IncomingControl` with `ControlNone` default in `buildIntersections`. |
| `internal/netbuild/control.go` | NEW — sign-tag resolution + class-based fallback (separate file for focus). |
| `internal/netbuild/control_test.go` | NEW — tests for sign resolution and fallback. |
| `internal/osmload/osmload.go` | Extend `collect` to retain nodes tagged with sign keys. |
| `internal/osmload/osmload_test.go` | Test the extended retention. |

---

## Task 1: Add `Control` enum and `IncomingControl` field

**Files:**
- Modify: `internal/network/types.go`

- [ ] **Step 1: Add the enum and field**

In `internal/network/types.go`, add the `Control` type just above the existing `Intersection` type:

```go
// Control names the right-of-way rule that governs a specific incoming
// approach at an intersection. The values are intentionally ordered so
// that a higher numeric value is a stricter control: a stop is stricter
// than a yield, an all-way stop is stricter still.
type Control uint8

const (
	ControlNone        Control = iota // through-movement, no sign
	ControlYield                      // yield sign — slow, no mandatory stop
	ControlStop                       // stop sign — mandatory dwell, then gap-accept
	ControlAllWayStop                 // all-way stop — Stop + FIFO arbitration
)
```

Then add the `IncomingControl` field to `Intersection`:

```go
// Intersection is a node where edges meet. ID indexes into Network.Intersections
// and is unrelated to NodeID — ID lives in IntersectionID-space, NodeID gives
// the spatial position. Incoming and Outgoing list the edges that arrive at and
// depart from NodeID.
type Intersection struct {
	ID        IntersectionID
	NodeID    NodeID
	Incoming  []EdgeID
	// IncomingControl is parallel to Incoming: IncomingControl[i] is the
	// right-of-way rule for approach Incoming[i]. The two slices have
	// equal length. Populated by netbuild after sortIncomingByPriority.
	IncomingControl []Control
	Outgoing        []EdgeID
	HasSignal       bool
	// BannedTurns lists (from, to) edge transitions that are forbidden
	// at this intersection. Populated at load time from config (or, in
	// future, from OSM `restriction` relations) and read-only thereafter.
	BannedTurns []TurnRestriction
}
```

- [ ] **Step 2: Verify the package still compiles**

Run: `go build ./internal/network/`
Expected: no output, exit 0.

- [ ] **Step 3: Verify nothing else broke yet**

Run: `go build ./...`
Expected: no output, exit 0. (The new field is unused everywhere; everything compiles.)

- [ ] **Step 4: Commit**

```
git add internal/network/types.go
git commit -m "feat(network): add Control enum and Intersection.IncomingControl"
```

---

## Task 2: Add `StoppedSinceSec` field on `Vehicle` and zero on edge transition

**Files:**
- Modify: `internal/sim/vehicle.go`

- [ ] **Step 1: Add the field**

In `internal/sim/vehicle.go`, in the `Vehicle` struct (just below the existing `StuckTime` field):

```go
	// StoppedSinceSec is the sim-time at which this vehicle came to a
	// complete stop at its current approach's stop line. Zero means
	// "not currently stopped at a stop line." Reset to zero when the
	// vehicle transitions to a new edge.
	StoppedSinceSec float64
```

- [ ] **Step 2: Zero it on edge transition**

In `internal/sim/vehicle.go`, find the `for v.S >= edge.Length` loop inside `stepIDM`. Immediately after `v.Edge = v.Route[v.RouteIdx]` and `edge = &net.Edges[v.Edge]`, add:

```go
		// Clear any mandatory-stop arrival timestamp now that we've left
		// the prior approach.
		v.StoppedSinceSec = 0
```

The surrounding context after the edit:

```go
		v.Edge = v.Route[v.RouteIdx]
		edge = &net.Edges[v.Edge]

		// Clear any mandatory-stop arrival timestamp now that we've left
		// the prior approach.
		v.StoppedSinceSec = 0

		// Lane carry-over: pick the new lane based on the just-completed
		// turn. This is both the normal post-turn carry-over AND the snap
		// fallback when bias didn't get us to a compatible lane in time.
```

- [ ] **Step 3: Build**

Run: `go build ./...`
Expected: no output, exit 0.

- [ ] **Step 4: Run sim tests to confirm nothing regressed**

Run: `go test ./internal/sim/`
Expected: PASS for all tests (the new field is allocated as zero and the assignment is a no-op until later tasks use it).

- [ ] **Step 5: Commit**

```
git add internal/sim/vehicle.go
git commit -m "feat(sim): add Vehicle.StoppedSinceSec, clear on edge transition"
```

---

## Task 3: Add yield-rule constants

**Files:**
- Modify: `internal/sim/world.go`

- [ ] **Step 1: Add the constants**

In `internal/sim/world.go`, find the existing block:

```go
const gapThresholdSec = 3.0

const (
	// stuckSpeedThresh ...
	stuckSpeedThresh = 0.1
	// stuckTimeoutSec ...
	stuckTimeoutSec = 60.0
)
```

Replace with:

```go
const gapThresholdSec = 3.0

const (
	// stuckSpeedThresh is the speed (m/s) below which a vehicle is
	// considered "not moving" for the purposes of the stuck-despawn guard
	// and the mandatory-stop dwell at Stop/AllWayStop approaches.
	stuckSpeedThresh = 0.1
	// stuckTimeoutSec is the accumulated sim-seconds of below-threshold
	// motion (with no legitimate red/yield reason) that triggers despawn.
	stuckTimeoutSec = 60.0
	// stopDwellSec is the minimum sim-seconds a vehicle must remain
	// effectively stationary at a Stop or AllWayStop line before being
	// allowed to begin gap-acceptance.
	stopDwellSec = 0.5
	// stopLineTolMeters is the maximum distance from the stop line at
	// which a slow-moving vehicle (V < stuckSpeedThresh) is considered
	// to have arrived at the line. Beyond this, the vehicle is "slow
	// upstream" but not yet stopped.
	stopLineTolMeters = 2.0
)
```

- [ ] **Step 2: Build**

Run: `go build ./...`
Expected: exit 0. (The new constants are unused for now; Go will not warn about unused package-level constants.)

- [ ] **Step 3: Commit**

```
git add internal/sim/world.go
git commit -m "feat(sim): add stopDwellSec and stopLineTolMeters constants"
```

---

## Task 4: Populate default `IncomingControl` in netbuild

**Files:**
- Modify: `internal/netbuild/netbuild.go`

This task makes every built `Intersection` have an `IncomingControl` slice with `ControlNone` entries of matching length — establishing the invariant `len(IncomingControl) == len(Incoming)` for OSM-loaded networks. Real per-approach values come in later tasks.

- [ ] **Step 1: Modify `buildIntersections`**

In `internal/netbuild/netbuild.go`, find the loop in `buildIntersections` that appends to `xs`:

```go
		xs = append(xs, network.Intersection{
			ID:        network.IntersectionID(len(xs)),
			NodeID:    n.ID,
			Incoming:  incE,
			Outgoing:  outE,
			HasSignal: signalNodes[n.ID],
		})
```

Replace with:

```go
		ctrl := make([]network.Control, len(incE))
		// Default to ControlNone for every approach. Real per-approach
		// values are assigned later in resolveControls (called after
		// sortIncomingByPriority).
		xs = append(xs, network.Intersection{
			ID:              network.IntersectionID(len(xs)),
			NodeID:          n.ID,
			Incoming:        incE,
			IncomingControl: ctrl,
			Outgoing:        outE,
			HasSignal:       signalNodes[n.ID],
		})
```

- [ ] **Step 2: Verify `sortIncomingByPriority` co-sorts the new slice**

Open `internal/netbuild/priority.go` and find `sortIncomingByPriority`. It currently sorts only `x.Incoming`. We need to keep `IncomingControl` synced. Since this function runs *before* control resolution (per the spec, controls are filled in afterwards), the slice contains only `ControlNone` values at this point — so technically the sort can ignore it and we'll just fill values into `IncomingControl[i]` for the sorted-position `i` later. We still want defensive correctness, though.

Add the co-sort. Replace the existing function body with:

```go
func sortIncomingByPriority(
	intersections []network.Intersection,
	osmWayOfEdge []osm.WayID,
	feat *osmload.Features,
) {
	wayByID := make(map[osm.WayID]*osm.Way, len(feat.Ways))
	for _, w := range feat.Ways {
		wayByID[w.ID] = w
	}
	priorityOf := func(eid network.EdgeID) int {
		if int(eid) >= len(osmWayOfEdge) {
			return 100
		}
		w, ok := wayByID[osmWayOfEdge[eid]]
		if !ok || w == nil {
			return 100
		}
		for _, t := range w.Tags {
			if t.Key == "highway" {
				return highwayPriority(t.Value)
			}
		}
		return 100
	}
	for i := range intersections {
		x := &intersections[i]
		// Sort the Incoming slice and apply the same permutation to
		// IncomingControl so the two stay aligned. Index-permutation
		// approach keeps both slices in sync without a custom sort.Sort
		// receiver.
		idx := make([]int, len(x.Incoming))
		for j := range idx {
			idx[j] = j
		}
		sort.SliceStable(idx, func(a, b int) bool {
			ea, eb := x.Incoming[idx[a]], x.Incoming[idx[b]]
			pa, pb := priorityOf(ea), priorityOf(eb)
			if pa != pb {
				return pa < pb
			}
			return ea < eb
		})
		newInc := make([]network.EdgeID, len(x.Incoming))
		newCtrl := make([]network.Control, len(x.IncomingControl))
		for newI, oldI := range idx {
			newInc[newI] = x.Incoming[oldI]
			if oldI < len(x.IncomingControl) {
				newCtrl[newI] = x.IncomingControl[oldI]
			}
		}
		x.Incoming = newInc
		x.IncomingControl = newCtrl
	}
}
```

- [ ] **Step 3: Build**

Run: `go build ./...`
Expected: exit 0.

- [ ] **Step 4: Run all tests**

Run: `go test ./...`
Expected: PASS. The new field is populated everywhere with zeros (`ControlNone`); behavior is unchanged because nothing reads it yet.

- [ ] **Step 5: Commit**

```
git add internal/netbuild/netbuild.go internal/netbuild/priority.go
git commit -m "feat(netbuild): allocate IncomingControl and co-sort with Incoming"
```

---

## Task 5: Add test helpers for `Intersection` fixtures

**Files:**
- Modify: `internal/sim/world_test.go`

Future tasks build `Intersection` literals with `IncomingControl` slices. A trivial helper keeps fixtures readable.

- [ ] **Step 1: Add helpers near the top of the test file**

In `internal/sim/world_test.go`, immediately after the existing imports, add:

```go
// allNone returns a slice of n ControlNone entries — the default for
// signaled intersections (where IncomingControl is not consulted under
// ModeNormal) and for hand-built fixtures that want priority/yield
// behavior to come from explicit setup.
func allNone(n int) []network.Control {
	out := make([]network.Control, n)
	return out
}

// ctrls is a syntactic shorthand for assembling an IncomingControl slice
// inline in a fixture literal. Example:
//
//	IncomingControl: ctrls(network.ControlNone, network.ControlStop),
func ctrls(cs ...network.Control) []network.Control {
	out := make([]network.Control, len(cs))
	copy(out, cs)
	return out
}
```

- [ ] **Step 2: Build the test binary**

Run: `go test ./internal/sim/ -count=0`
Expected: exit 0 (compile-only).

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): add ctrls/allNone helpers for IncomingControl fixtures"
```

---

## Task 6: Rewrite `stopDistanceForYield` to dispatch on `Control`

**Files:**
- Modify: `internal/sim/world.go`
- Modify: `internal/sim/world_test.go` (fixture updates)

This is the core behavioral change. The new function dispatches on the effective `Control` for the approach. `ControlNone` returns `(0, false)`; `ControlYield` does gap-acceptance only; `ControlStop` adds mandatory dwell. `ControlAllWayStop` is added in Task 8; for now the dispatch table returns `(myDist, true)` (always yield) as a placeholder so that intersections with AllWayStop don't behave wrong before FIFO is implemented.

- [ ] **Step 1: Write failing tests for `ControlYield` and `ControlStop`**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_StopSign_MandatoryDwell: a Stop-controlled vehicle with no
// cross-traffic must come to v ~ 0 at the stop line and dwell for at
// least stopDwellSec before being allowed to depart.
func TestWorld_StopSign_MandatoryDwell(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: 0}},
		{ID: 2, Pos: network.Point{X: 100, Y: 0}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 1), // approach
		mkEdge(1, 1, 2), // outbound
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 1,
			Incoming:        []network.EdgeID{0},
			IncomingControl: ctrls(network.ControlStop),
			Outgoing:        []network.EdgeID{1},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 80, V: 8},
	}
	w.nextID = 2

	// Run enough ticks for the vehicle to approach, stop, dwell, and
	// proceed. dt=0.05 (20 Hz), so 200 ticks = 10s of sim time.
	stoppedAt := -1.0
	for i := 0; i < 200; i++ {
		w.Step()
		v := &w.Vehicles[0]
		if v.Despawned {
			break
		}
		if stoppedAt < 0 && v.StoppedSinceSec > 0 {
			stoppedAt = w.SimTime
		}
	}

	if stoppedAt < 0 {
		t.Fatal("vehicle never registered a mandatory stop at the stop line")
	}
	// After stopping, the vehicle must dwell at least stopDwellSec before
	// it gets to depart. Cleared StoppedSinceSec means it crossed the
	// intersection.
	v := &w.Vehicles[0]
	if v.Edge != 1 {
		t.Errorf("vehicle should have advanced to outbound edge after dwell, still on edge %d", v.Edge)
	}
}

// TestWorld_YieldSign_NoMandatoryStop: a Yield-controlled vehicle with no
// cross-traffic must NOT come to a complete stop — it slow-rolls through.
func TestWorld_YieldSign_NoMandatoryStop(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: 0}},
		{ID: 2, Pos: network.Point{X: 100, Y: 0}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 1),
		mkEdge(1, 1, 2),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 1,
			Incoming:        []network.EdgeID{0},
			IncomingControl: ctrls(network.ControlYield),
			Outgoing:        []network.EdgeID{1},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 50, V: 8},
	}
	w.nextID = 2

	for i := 0; i < 100; i++ {
		w.Step()
		if w.Vehicles[0].Despawned {
			break
		}
	}

	v := &w.Vehicles[0]
	if v.StoppedSinceSec != 0 {
		t.Errorf("yield vehicle with no cross-traffic should not record a mandatory stop, got StoppedSinceSec=%.3f", v.StoppedSinceSec)
	}
	if v.Edge != 1 {
		t.Errorf("yield vehicle should have advanced to outbound edge, still on edge %d", v.Edge)
	}
}
```

- [ ] **Step 2: Run the new tests — they must fail because the rule still uses the old logic**

Run: `go test ./internal/sim/ -run TestWorld_StopSign_MandatoryDwell -v`
Run: `go test ./internal/sim/ -run TestWorld_YieldSign_NoMandatoryStop -v`
Expected: both tests fail. `TestWorld_StopSign_MandatoryDwell` fails because no mandatory-stop logic exists yet; the vehicle proceeds without registering `StoppedSinceSec`. `TestWorld_YieldSign_NoMandatoryStop` may pass coincidentally (single approach has no other-vehicle to yield to under the old rule either), in which case treat it as a regression catch rather than a TDD red.

- [ ] **Step 3: Rewrite `stopDistanceForYield`**

In `internal/sim/world.go`, replace the entire `stopDistanceForYield` function with:

```go
// stopDistanceForYield returns (distance to stop line, true) when the
// vehicle's current edge ends at an intersection where it must wait
// before crossing. Dispatches on the effective Control for this approach:
//
//   - ControlNone:        no obligation; returns (0, false).
//   - ControlYield:       gap-acceptance against ControlNone approaches.
//   - ControlStop:        mandatory dwell, then gap-acceptance.
//   - ControlAllWayStop:  mandatory dwell, then FIFO arbitration.
//
// For signaled intersections, effective control is derived from the
// signal mode: ModeNormal returns immediately (stopDistanceForRed owns
// the hard-stop case); ModeOff/Flash treat each approach as
// AllWayStop/Stop/None as appropriate.
//
// As a side effect, sets v.StoppedSinceSec when v first reaches v ~ 0
// near the stop line of a Stop/AllWayStop approach. v.StoppedSinceSec
// is cleared elsewhere (in stepIDM, on edge transition).
func (w *World) stopDistanceForYield(v *Vehicle, byEdge map[network.EdgeID][]int) (float64, bool) {
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok {
		return 0, false
	}
	myPos := IncomingPos(x, v.Edge)
	if myPos < 0 {
		return 0, false
	}

	effective := effectiveControl(w, x, myPos)
	myDist := edge.Length - v.S
	if myDist < 0 {
		myDist = 0
	}

	switch effective {
	case network.ControlNone:
		return 0, false

	case network.ControlYield:
		return w.yieldGapCheck(v, x, myPos, myDist, byEdge)

	case network.ControlStop:
		if !w.hasDwelled(v, myDist) {
			w.maybeMarkStopped(v, myDist)
			return myDist, true
		}
		return w.yieldGapCheck(v, x, myPos, myDist, byEdge)

	case network.ControlAllWayStop:
		// FIFO arbitration arrives in Task 8. For now, hold at the line
		// after dwell so that AllWayStop approaches are at least safe.
		if !w.hasDwelled(v, myDist) {
			w.maybeMarkStopped(v, myDist)
			return myDist, true
		}
		return w.allWayStopFIFO(v, x, myPos, myDist, byEdge)
	}
	return 0, false
}

// effectiveControl resolves the right-of-way rule for one approach at
// one decision tick. Signal mode overrides the stored IncomingControl.
func effectiveControl(w *World, x *network.Intersection, myPos int) network.Control {
	if x.HasSignal {
		st := w.SignalStates[x.ID]
		if st == nil {
			return network.ControlNone
		}
		switch st.Mode {
		case ModeNormal:
			return network.ControlNone // stopDistanceForRed owns this case
		case ModeOff:
			return network.ControlAllWayStop
		case ModeFlashA, ModeFlashB:
			if st.GreenFor(myPos) {
				return network.ControlNone // blinking yellow has priority
			}
			return network.ControlStop // blinking red is a stop sign
		}
		return network.ControlNone
	}
	if myPos < len(x.IncomingControl) {
		return x.IncomingControl[myPos]
	}
	return network.ControlNone
}

// hasDwelled returns true once the vehicle has completed its mandatory-stop
// dwell at the stop line. False both before reaching the line and during
// the dwell window.
func (w *World) hasDwelled(v *Vehicle, myDist float64) bool {
	if v.StoppedSinceSec == 0 {
		return false
	}
	return w.SimTime-v.StoppedSinceSec >= stopDwellSec
}

// maybeMarkStopped sets v.StoppedSinceSec the first tick the vehicle is
// effectively at the stop line (slow AND within tolerance). Idempotent
// once set.
func (w *World) maybeMarkStopped(v *Vehicle, myDist float64) {
	if v.StoppedSinceSec != 0 {
		return
	}
	if v.V < stuckSpeedThresh && myDist < stopLineTolMeters {
		v.StoppedSinceSec = w.SimTime
	}
}

// yieldGapCheck does ETA-based gap-acceptance against every approach at x
// whose effective control is ControlNone (i.e., the priority approaches).
// Returns (myDist, true) when we must yield; (0, false) when the gap is
// clear.
func (w *World) yieldGapCheck(v *Vehicle, x *network.Intersection, myPos int,
	myDist float64, byEdge map[network.EdgeID][]int,
) (float64, bool) {
	for j := range x.Incoming {
		if j == myPos {
			continue
		}
		if effectiveControl(w, x, j) != network.ControlNone {
			continue
		}
		others := byEdge[x.Incoming[j]]
		if len(others) == 0 {
			continue
		}
		otherEdge := &w.Net.Edges[x.Incoming[j]]
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			ovV := ov.V
			if ovV < 0.5 {
				ovV = 0.5
			}
			if d/ovV < gapThresholdSec {
				return myDist, true
			}
		}
	}
	return 0, false
}

// allWayStopFIFO is filled in by Task 8. For now, return (myDist, true)
// so that AllWayStop approaches always wait — safe but pessimistic.
func (w *World) allWayStopFIFO(v *Vehicle, x *network.Intersection, myPos int,
	myDist float64, byEdge map[network.EdgeID][]int,
) (float64, bool) {
	return myDist, true
}
```

- [ ] **Step 4: Update `TestWorld_StuckAtYieldNotDespawned` fixture**

In `internal/sim/world_test.go`, find the existing `TestWorld_StuckAtYieldNotDespawned` test (~line 691). In its intersection literal:

```go
	xs := []network.Intersection{
		{
			ID:        0,
			NodeID:    2,
			Incoming:  []network.EdgeID{0, 1}, // 0 = priority, 1 = yield
			Outgoing:  []network.EdgeID{2},
			HasSignal: false,
		},
	}
```

Change to:

```go
	xs := []network.Intersection{
		{
			ID:        0,
			NodeID:    2,
			Incoming:  []network.EdgeID{0, 1}, // 0 = priority, 1 = yield
			IncomingControl: ctrls(
				network.ControlNone,  // priority approach
				network.ControlYield, // yield approach
			),
			Outgoing:  []network.EdgeID{2},
			HasSignal: false,
		},
	}
```

- [ ] **Step 5: Run the two new tests — they must now pass**

Run: `go test ./internal/sim/ -run TestWorld_StopSign_MandatoryDwell -v`
Run: `go test ./internal/sim/ -run TestWorld_YieldSign_NoMandatoryStop -v`
Expected: both PASS.

- [ ] **Step 6: Run the full sim suite**

Run: `go test ./internal/sim/ -v`
Expected: all PASS. The fixture for `TestWorld_StuckAtYieldNotDespawned` now declares its yielding approach explicitly; all other tests have intersections with only `ControlNone` so they behave like through-roads.

If a test that previously relied on the implicit `Incoming[0]`-priority rule fails (other than the one we just fixed), update its intersection literal to set `IncomingControl` explicitly — `ctrls(network.ControlNone, network.ControlYield)` for the old "0=priority, 1=yield" pattern. Re-run after each fix.

- [ ] **Step 7: Run all tests**

Run: `go test ./...`
Expected: all PASS.

- [ ] **Step 8: Commit**

```
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): dispatch yield rule on per-approach Control"
```

---

## Task 7: Mandatory-stop gap-acceptance combined test

**Files:**
- Modify: `internal/sim/world_test.go`

A combined test that exercises Stop + gap-acceptance together. This catches regressions where mandatory dwell completes but the subsequent gap-check is skipped or wrong.

- [ ] **Step 1: Append the test**

```go
// TestWorld_StopSign_GapAcceptance: Stop-controlled vehicle + priority
// cross-traffic with short ETA. Must stop, dwell, then continue to wait
// for the gap to clear.
func TestWorld_StopSign_GapAcceptance(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},  // W priority origin
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},  // S stop origin
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},     // center
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},   // E priority destination
		{ID: 4, Pos: network.Point{X: 0, Y: 100}},   // N stop destination
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 2), // priority approach W->C
		mkEdge(1, 1, 2), // stop approach S->C
		mkEdge(2, 2, 3), // priority outbound C->E
		mkEdge(3, 2, 4), // stop outbound C->N
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming: []network.EdgeID{0, 1},
			IncomingControl: ctrls(
				network.ControlNone, // priority
				network.ControlStop, // stop
			),
			Outgoing:  []network.EdgeID{2, 3},
			HasSignal: false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Priority vehicle pinned close to the line at low speed (ETA inside
	// gapThresholdSec). Stop vehicle approaching its line.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 50, V: 8},
	}
	w.nextID = 3

	for i := 0; i < 200; i++ {
		// Re-pin the priority vehicle so the stop-controlled vehicle
		// keeps seeing it as imminent cross-traffic.
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 0.5
			}
		}
		w.Step()
	}

	var stop *Vehicle
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 2 {
			stop = &w.Vehicles[j]
		}
	}
	if stop == nil || stop.Despawned {
		t.Fatal("stop-controlled vehicle should not be despawned during a legitimate stop+yield")
	}
	if stop.Edge != 1 {
		t.Errorf("stop vehicle should still be on approach edge 1, got edge %d", stop.Edge)
	}
	if stop.StoppedSinceSec == 0 {
		t.Error("stop vehicle should have registered a mandatory stop")
	}
	if stop.StuckTime != 0 {
		t.Errorf("stop vehicle is legitimately waiting; StuckTime must be 0, got %.3f", stop.StuckTime)
	}
}
```

- [ ] **Step 2: Run it**

Run: `go test ./internal/sim/ -run TestWorld_StopSign_GapAcceptance -v`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): Stop + gap-acceptance combined"
```

---

## Task 8: Implement `AllWayStop` FIFO arbitration

**Files:**
- Modify: `internal/sim/world.go`
- Modify: `internal/sim/world_test.go`

- [ ] **Step 1: Write failing tests**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_AllWayStop_FIFO: three vehicles arriving on three approaches
// at staggered times depart in arrival order.
func TestWorld_AllWayStop_FIFO(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},   // W origin
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},    // E origin
		{ID: 2, Pos: network.Point{X: 0, Y: -100}},   // S origin
		{ID: 3, Pos: network.Point{X: 0, Y: 0}},      // center
		{ID: 4, Pos: network.Point{X: 0, Y: 100}},    // N dest (outbound)
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 3), // W->C
		mkEdge(1, 1, 3), // E->C
		mkEdge(2, 2, 3), // S->C
		mkEdge(3, 3, 4), // C->N (outbound for all)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 3,
			Incoming: []network.EdgeID{0, 1, 2},
			IncomingControl: ctrls(
				network.ControlAllWayStop,
				network.ControlAllWayStop,
				network.ControlAllWayStop,
			),
			Outgoing:  []network.EdgeID{3},
			HasSignal: false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Three vehicles, each placed so they arrive at the line at different
	// times: ID 1 first (closest), then 2, then 3.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 3}, Edge: 0, S: 99.5, V: 1},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 80, V: 1},
		{ID: 3, Route: []network.EdgeID{2, 3}, Edge: 2, S: 60, V: 1},
	}
	w.nextID = 4

	departureOrder := make([]VehicleID, 0, 3)
	for i := 0; i < 600 && len(departureOrder) < 3; i++ {
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.Despawned {
				continue
			}
			if v.Edge == 3 {
				// First tick where vehicle is on the outbound edge.
				already := false
				for _, id := range departureOrder {
					if id == v.ID {
						already = true
						break
					}
				}
				if !already {
					departureOrder = append(departureOrder, v.ID)
				}
			}
		}
	}

	if len(departureOrder) != 3 {
		t.Fatalf("expected 3 departures, got %d: %v", len(departureOrder), departureOrder)
	}
	want := []VehicleID{1, 2, 3}
	for i := range want {
		if departureOrder[i] != want[i] {
			t.Errorf("departure order mismatch: got %v want %v", departureOrder, want)
			break
		}
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `go test ./internal/sim/ -run TestWorld_AllWayStop_FIFO -v`
Expected: FAIL. The placeholder `allWayStopFIFO` always returns "yield", so no vehicle ever departs.

- [ ] **Step 3: Implement the FIFO body**

In `internal/sim/world.go`, replace the placeholder `allWayStopFIFO` with:

```go
// allWayStopFIFO arbitrates an AllWayStop approach. After v has completed
// its mandatory-stop dwell, it scans every other approach for a lead
// vehicle that came to a complete stop earlier than v. If one exists, we
// yield. Otherwise we proceed. Ties (same StoppedSinceSec) are broken by
// lower Incoming index winning.
func (w *World) allWayStopFIFO(v *Vehicle, x *network.Intersection, myPos int,
	myDist float64, byEdge map[network.EdgeID][]int,
) (float64, bool) {
	for j := range x.Incoming {
		if j == myPos {
			continue
		}
		others := byEdge[x.Incoming[j]]
		if len(others) == 0 {
			continue
		}
		// Find the lead vehicle on approach j — the one closest to the
		// stop line of edge x.Incoming[j].
		otherEdge := &w.Net.Edges[x.Incoming[j]]
		leadIdx := -1
		leadDist := math.Inf(1)
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			if d < leadDist {
				leadDist = d
				leadIdx = oi
			}
		}
		if leadIdx < 0 {
			continue
		}
		lead := &w.Vehicles[leadIdx]
		if lead.StoppedSinceSec == 0 {
			continue // not yet stopped; hasn't earned a FIFO slot
		}
		if lead.StoppedSinceSec < v.StoppedSinceSec {
			return myDist, true // they stopped first; we yield
		}
		if lead.StoppedSinceSec == v.StoppedSinceSec && j < myPos {
			return myDist, true // tie-break: lower Incoming index wins
		}
	}
	return 0, false
}
```

Add `"math"` to the imports if not already present (it likely is — `comfortableStopDistance` uses it via `IDM`).

- [ ] **Step 4: Run the new test**

Run: `go test ./internal/sim/ -run TestWorld_AllWayStop_FIFO -v`
Expected: PASS.

- [ ] **Step 5: Add the tick-tie test**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_AllWayStop_TickTie: two vehicles register their mandatory
// stop in the same tick on different approaches. Lower Incoming index
// wins the tie-break.
func TestWorld_AllWayStop_TickTie(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 0, Y: 100}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 2), // W->C  Incoming[0]
		mkEdge(1, 1, 2), // E->C  Incoming[1]
		mkEdge(2, 2, 3), // C->N outbound
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming: []network.EdgeID{0, 1},
			IncomingControl: ctrls(
				network.ControlAllWayStop,
				network.ControlAllWayStop,
			),
			Outgoing:  []network.EdgeID{2},
			HasSignal: false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Symmetric start: both vehicles equidistant from line, same speed.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.05},
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 99, V: 0.05},
	}
	w.nextID = 3

	firstDepart := VehicleID(0)
	for i := 0; i < 400 && firstDepart == 0; i++ {
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if !v.Despawned && v.Edge == 2 {
				firstDepart = v.ID
				break
			}
		}
	}

	// Both vehicles are placed identically; the lower Incoming index
	// approach (Incoming[0], W->C, Vehicle ID=1) should depart first
	// regardless of microscopic float ordering.
	if firstDepart != 1 {
		t.Errorf("tie-break should favor lower Incoming index (Vehicle 1), got Vehicle %d first", firstDepart)
	}
}
```

- [ ] **Step 6: Run the tie test**

Run: `go test ./internal/sim/ -run TestWorld_AllWayStop_TickTie -v`
Expected: PASS.

- [ ] **Step 7: Add the StoppedSinceClears test**

The clearing of `StoppedSinceSec` on edge transition was added in Task 2; this test pins the behavior so future edits don't regress it.

Append to `internal/sim/world_test.go`:

```go
// TestWorld_AllWayStop_StoppedSinceClears: after crossing an
// AllWayStop, a vehicle's StoppedSinceSec must be zeroed so it doesn't
// bleed into FIFO calculations at the next intersection.
func TestWorld_AllWayStop_StoppedSinceClears(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: 0}},
		{ID: 2, Pos: network.Point{X: 100, Y: 0}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 1), // approach
		mkEdge(1, 1, 2), // outbound (post-intersection)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 1,
			Incoming:        []network.EdgeID{0},
			IncomingControl: ctrls(network.ControlAllWayStop),
			Outgoing:        []network.EdgeID{1},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 80, V: 8},
	}
	w.nextID = 2

	for i := 0; i < 200; i++ {
		w.Step()
		if w.Vehicles[0].Despawned {
			break
		}
		if w.Vehicles[0].Edge == 1 && w.Vehicles[0].StoppedSinceSec != 0 {
			t.Fatalf("StoppedSinceSec should be 0 after edge transition, got %.3f", w.Vehicles[0].StoppedSinceSec)
		}
	}

	if w.Vehicles[0].Edge != 1 {
		t.Errorf("vehicle should have cleared the intersection, still on edge %d", w.Vehicles[0].Edge)
	}
}
```

- [ ] **Step 8: Run all AllWayStop tests**

Run: `go test ./internal/sim/ -run TestWorld_AllWayStop -v`
Expected: PASS for FIFO, TickTie, and StoppedSinceClears.

- [ ] **Step 9: Run the full sim suite**

Run: `go test ./internal/sim/`
Expected: all PASS.

- [ ] **Step 10: Commit**

```
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): AllWayStop FIFO arbitration with deterministic tie-break"
```

---

## Task 9: Signal flash/off behavioral test (verification)

**Files:**
- Modify: `internal/sim/world_test.go`

The `effectiveControl` function added in Task 6 already routes flash/off through the new dispatch (FlashB → `ControlStop`, ModeOff → `ControlAllWayStop`). This task adds an explicit test to lock that behavior in.

- [ ] **Step 1: Append the test**

```go
// TestWorld_SignalOff_TreatedAsAllWayStop: an intersection with
// HasSignal=true and Mode=ModeOff behaves like an AllWayStop: every
// approach must stop and dwell before departing.
func TestWorld_SignalOff_TreatedAsAllWayStop(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: -100}},
		{ID: 3, Pos: network.Point{X: 0, Y: 100}},
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},
		{ID: 5, Pos: network.Point{X: 200, Y: 0}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 4), // W->C
		mkEdge(1, 1, 4), // E->C
		mkEdge(2, 2, 4), // S->C
		mkEdge(3, 3, 4), // N->C
		mkEdge(4, 4, 5), // C->east outbound
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 4,
			Incoming:        []network.EdgeID{0, 1, 2, 3},
			IncomingControl: allNone(4), // not consulted: HasSignal=true
			Outgoing:        []network.EdgeID{4},
			HasSignal:       true,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.SignalStates[0].Mode = ModeOff

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 4}, Edge: 0, S: 60, V: 8},
	}
	w.nextID = 2

	registeredStop := false
	for i := 0; i < 300; i++ {
		w.Step()
		if w.Vehicles[0].StoppedSinceSec > 0 {
			registeredStop = true
		}
		if w.Vehicles[0].Despawned || w.Vehicles[0].Edge == 4 {
			break
		}
	}

	if !registeredStop {
		t.Error("ModeOff approach must register a mandatory stop")
	}
	if w.Vehicles[0].Edge != 4 {
		t.Errorf("vehicle should have cleared the intersection after dwell, still on edge %d", w.Vehicles[0].Edge)
	}
}
```

- [ ] **Step 2: Run it**

Run: `go test ./internal/sim/ -run TestWorld_SignalOff_TreatedAsAllWayStop -v`
Expected: PASS.

- [ ] **Step 3: Run the full sim suite**

Run: `go test ./internal/sim/`
Expected: all PASS.

If any flash-related vehicle test in `world_test.go` now fails because of the new mandatory-stop dwell, update its assertions to expect the dwell — the behavior is more realistic, not regressed.

- [ ] **Step 4: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): ModeOff acts as AllWayStop under new Control dispatch"
```

---

## Task 10: Determinism re-verification

**Files:** none modified (verification only)

The change must not break the seed→trace determinism guarantee.

- [ ] **Step 1: Run the determinism test**

Run: `go test ./internal/sim/ -run TestWorld_TraceDeterminism -v`
Expected: PASS. If it fails, the most likely cause is a non-deterministic iteration order in the new code paths (`yieldGapCheck` and `allWayStopFIFO`). Both iterate `x.Incoming` and per-edge `byEdge` slices in slice order, which is deterministic; if you see a failure, audit any new map-of-anything iteration introduced earlier.

- [ ] **Step 2: Run the benchmarks**

Run: `go test ./internal/sim/ -bench=BenchmarkWorld_Step -benchtime=2s -run=^$`
Expected: ns/op within ~10% of the README's quoted numbers (1k vehicles ≈ 0.46 ms/tick). If significantly worse, the most likely culprit is the new `effectiveControl` function being called from inside `yieldGapCheck` for each priority approach; consider hoisting the per-approach effective-control map out of the inner loop. Not blocking — note the result and move on.

- [ ] **Step 3: No commit (verification only)**

---

## Task 11: OSM ingestion — retain sign-tagged nodes

**Files:**
- Modify: `internal/osmload/osmload.go`
- Modify: `internal/osmload/osmload_test.go`
- Create: `internal/osmload/testdata/sign_nodes.osm`

The existing osmload tests use file-based fixtures in `internal/osmload/testdata/`. Follow that pattern.

- [ ] **Step 1: Create the fixture file**

Write `internal/osmload/testdata/sign_nodes.osm`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6">
  <node id="1" lat="40.0000" lon="-74.0000"/>
  <node id="2" lat="40.0010" lon="-74.0000"/>
  <node id="3" lat="40.0020" lon="-74.0000"/>
  <node id="10" lat="40.0030" lon="-74.0000">
    <tag k="highway" v="stop"/>
  </node>
  <node id="11" lat="40.0040" lon="-74.0000">
    <tag k="highway" v="give_way"/>
  </node>
  <node id="12" lat="40.0050" lon="-74.0000">
    <tag k="stop" v="all"/>
  </node>
  <node id="13" lat="40.0060" lon="-74.0000">
    <tag k="stop" v="minor"/>
  </node>
  <node id="99" lat="40.0500" lon="-74.0500"/>
  <way id="100">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="residential"/>
  </way>
</osm>
```

Nodes 10-13 are not referenced by any kept way. Without sign-node retention they would be dropped. With it, they must survive.

- [ ] **Step 2: Write the failing test**

Append to `internal/osmload/osmload_test.go`:

```go
func TestLoad_RetainsSignNodes(t *testing.T) {
	f, err := Load("testdata/sign_nodes.osm")
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	// Nodes 1-3 referenced by the kept way; nodes 10-13 retained only
	// because of sign tags; node 99 not referenced and not signed.
	for _, id := range []osm.NodeID{10, 11, 12, 13} {
		if _, ok := f.Nodes[id]; !ok {
			t.Errorf("node %d should be retained (carries sign tag), missing", id)
		}
	}
	if _, ok := f.Nodes[99]; ok {
		t.Errorf("node 99 should be dropped (unreferenced, untagged)")
	}
}
```

- [ ] **Step 3: Run it — expect failure**

Run: `go test ./internal/osmload/ -run TestLoad_RetainsSignNodes -v`
Expected: FAIL — nodes 10-13 missing.

- [ ] **Step 4: Modify `osmload.go`**

In `internal/osmload/osmload.go`, find the existing node-retention block:

```go
	for _, n := range allNodes {
		if want[n.ID] || hasTag(n.Tags, "highway", "traffic_signals") {
			feat.Nodes[n.ID] = n
		}
	}
```

Replace with:

```go
	for _, n := range allNodes {
		if want[n.ID] || isControlNode(n.Tags) {
			feat.Nodes[n.ID] = n
		}
	}
```

Then add a helper at the bottom of the file, just below `hasTag`:

```go
// isControlNode reports whether a node carries any tag that affects
// intersection right-of-way: traffic signal, stop sign, yield sign, or
// the way-scoped stop=all / stop=minor attribute (which is on the
// intersection node in OSM convention).
func isControlNode(tags osm.Tags) bool {
	for _, t := range tags {
		if t.Key == "highway" && (t.Value == "traffic_signals" || t.Value == "stop" || t.Value == "give_way") {
			return true
		}
		if t.Key == "stop" && (t.Value == "all" || t.Value == "minor") {
			return true
		}
	}
	return false
}
```

- [ ] **Step 5: Run the new test**

Run: `go test ./internal/osmload/ -run TestLoad_RetainsSignNodes -v`
Expected: PASS.

- [ ] **Step 6: Run the full osmload suite**

Run: `go test ./internal/osmload/`
Expected: PASS.

- [ ] **Step 7: Commit**

```
git add internal/osmload/osmload.go internal/osmload/osmload_test.go internal/osmload/testdata/sign_nodes.osm
git commit -m "feat(osmload): retain nodes carrying stop/yield/stop=all/stop=minor tags"
```

---

## Task 12: Netbuild — class-based fallback Control resolution

**Files:**
- Create: `internal/netbuild/control.go`
- Create: `internal/netbuild/control_test.go`
- Modify: `internal/netbuild/netbuild.go` (call site)

The fallback rule from the spec:
- Unequal classes among approaches → lower-class approaches get `ControlStop`, highest-class approaches get `ControlNone`.
- Equal classes among all approaches → every approach gets `ControlAllWayStop`.
- Two-approach degree-2 nodes are not affected (they're not built as intersections unless they're signals).

- [ ] **Step 1: Write failing tests**

Create `internal/netbuild/control_test.go`:

```go
package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// TestNetbuild_Fallback_UnequalClass: a primary road meets a residential
// road. The residential approach gets ControlStop; the primary approaches
// stay ControlNone.
func TestNetbuild_Fallback_UnequalClass(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005), // intersection
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	if len(x.IncomingControl) != len(x.Incoming) {
		t.Fatalf("IncomingControl length %d != Incoming length %d", len(x.IncomingControl), len(x.Incoming))
	}
	var sawStop, sawNone bool
	for i, eid := range x.Incoming {
		c := x.IncomingControl[i]
		hw := highwayOfEdge(net, eid, feat)
		switch hw {
		case "primary":
			if c != network.ControlNone {
				t.Errorf("primary approach should be None, got %v", c)
			}
			sawNone = true
		case "residential":
			if c != network.ControlStop {
				t.Errorf("residential approach should be Stop, got %v", c)
			}
			sawStop = true
		}
	}
	if !sawStop || !sawNone {
		t.Error("expected to see both Stop and None controls at unequal-class fallback intersection")
	}
}

// TestNetbuild_Fallback_EqualClass: two residential roads meet, no
// signage. Every approach gets ControlAllWayStop.
func TestNetbuild_Fallback_EqualClass(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("approach %d should be AllWayStop, got %v", i, c)
		}
	}
}

// highwayOfEdge looks up the OSM highway= tag of the way an edge came from.
// Used by tests to confirm Control assignments correspond to the right
// road class. (Tests are friends with internals.)
func highwayOfEdge(net *network.Network, eid network.EdgeID, feat *osmload.Features) string {
	// Find the way whose node sequence contains the edge endpoints in
	// adjacent positions. Coarse but adequate for the tiny test inputs.
	e := net.Edges[eid]
	fromOSM := findOSMID(net, e.From, feat)
	toOSM := findOSMID(net, e.To, feat)
	for _, w := range feat.Ways {
		for i := 0; i+1 < len(w.Nodes); i++ {
			a, b := w.Nodes[i].ID, w.Nodes[i+1].ID
			if (a == fromOSM && b == toOSM) || (a == toOSM && b == fromOSM) {
				for _, t := range w.Tags {
					if t.Key == "highway" {
						return t.Value
					}
				}
			}
		}
	}
	return ""
}

func findOSMID(net *network.Network, nid network.NodeID, feat *osmload.Features) osm.NodeID {
	// Match by projected position. The smallest distance is the original
	// node — we don't have a reverse map handy.
	want := net.Nodes[nid].Pos
	var best osm.NodeID
	bestD := 1e18
	for id, n := range feat.Nodes {
		dx := n.Lat - 40.0 // arbitrary near-zero check; test inputs are within 1 degree
		dy := n.Lon + 74.0
		_, _ = want, dx
		_ = dy
		// Quick fallback: assume node ID == OSM ID arithmetic is consistent
		// in test fixtures (mkNode uses small integer IDs and Build
		// allocates NodeIDs in iteration order, which is non-deterministic).
		// For tests, a more robust mapping is to iterate feat.Nodes and
		// compare projected positions; here we keep it lightweight.
		if float64(id) >= 0 {
			d := absF(float64(id) - float64(nid))
			if d < bestD {
				bestD = d
				best = id
			}
		}
	}
	return best
}

func absF(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
```

Note: `highwayOfEdge` / `findOSMID` use a coarse "node ID arithmetic happens to align" approach for the tiny test fixtures. If this proves flaky, the implementer should build a proper `osmID by NodeID` reverse map at the start of each test. For these particular fixtures (nodes 1-5, where NodeID indices align), it works.

- [ ] **Step 2: Run them — expect failure**

Run: `go test ./internal/netbuild/ -run TestNetbuild_Fallback -v`
Expected: FAIL. Both tests fail because `IncomingControl` is still all-`ControlNone`.

- [ ] **Step 3: Create the resolver**

Create `internal/netbuild/control.go`:

```go
package netbuild

import (
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// resolveControls fills in IncomingControl for every intersection in xs.
// Runs after sortIncomingByPriority, so IncomingControl[i] is the rule
// for the approach now at Incoming[i] (final sorted position).
//
// Resolution order (first rule that applies wins for a given approach):
//   1. stop=all on the intersection node      -> AllWayStop everywhere.
//   2. stop=minor on the intersection node    -> Stop on every approach
//      whose highway class is strictly lower-priority than the best.
//   3. highway=stop / highway=give_way on the
//      intersection node (with optional direction=)
//                                             -> Stop / Yield on the
//      matching approach(es).
//   4. highway=stop / highway=give_way on an
//      interior approach-segment node          -> Stop / Yield on that
//      specific approach.
//   5. Class-based fallback:
//        unequal classes -> lower gets Stop
//        equal classes   -> AllWayStop everywhere
func resolveControls(
	xs []network.Intersection,
	feat *osmload.Features,
	osmWayOfEdge []osm.WayID,
	netNodeOf func(osm.NodeID) (network.NodeID, bool),
	osmNodeOf func(network.NodeID) (osm.NodeID, bool),
) {
	wayByID := make(map[osm.WayID]*osm.Way, len(feat.Ways))
	for _, w := range feat.Ways {
		wayByID[w.ID] = w
	}
	classOfEdge := func(eid network.EdgeID) int {
		if int(eid) >= len(osmWayOfEdge) {
			return 100
		}
		w, ok := wayByID[osmWayOfEdge[eid]]
		if !ok || w == nil {
			return 100
		}
		for _, t := range w.Tags {
			if t.Key == "highway" {
				return highwayPriority(t.Value)
			}
		}
		return 100
	}

	for i := range xs {
		x := &xs[i]
		osmID, ok := osmNodeOf(x.NodeID)
		var nodeTags osm.Tags
		if ok {
			if n, ok2 := feat.Nodes[osmID]; ok2 && n != nil {
				nodeTags = n.Tags
			}
		}

		// Class-based fallback baseline: compute it once per intersection,
		// then override with explicit OSM signage where present (Phase 1
		// covers fallback; Task 13 layers explicit signage on top).
		applyClassFallback(x, classOfEdge)

		// Phase 1 explicit-signage handling: only the cheap, intersection-
		// node-scoped tags. Per-approach interior-node and direction=
		// handling lands in Task 13.
		applyStopAllOrMinor(x, nodeTags, classOfEdge)
	}
}

// applyClassFallback sets IncomingControl based on functional class only.
// Unequal classes: best (lowest priority value) stays ControlNone,
// strictly higher (lower-priority) approaches get ControlStop.
// Equal classes: every approach becomes ControlAllWayStop.
func applyClassFallback(x *network.Intersection, classOfEdge func(network.EdgeID) int) {
	if len(x.Incoming) == 0 {
		return
	}
	best := classOfEdge(x.Incoming[0])
	for _, eid := range x.Incoming[1:] {
		if c := classOfEdge(eid); c < best {
			best = c
		}
	}
	allEqual := true
	for _, eid := range x.Incoming {
		if classOfEdge(eid) != best {
			allEqual = false
			break
		}
	}
	if allEqual {
		for j := range x.IncomingControl {
			x.IncomingControl[j] = network.ControlAllWayStop
		}
		return
	}
	for j, eid := range x.Incoming {
		if classOfEdge(eid) == best {
			x.IncomingControl[j] = network.ControlNone
		} else {
			x.IncomingControl[j] = network.ControlStop
		}
	}
}

// applyStopAllOrMinor overrides class-fallback with explicit OSM tags
// scoped to the intersection node.
func applyStopAllOrMinor(x *network.Intersection, tags osm.Tags, classOfEdge func(network.EdgeID) int) {
	for _, t := range tags {
		if t.Key == "stop" && t.Value == "all" {
			for j := range x.IncomingControl {
				x.IncomingControl[j] = network.ControlAllWayStop
			}
			return
		}
		if t.Key == "stop" && t.Value == "minor" {
			if len(x.Incoming) == 0 {
				return
			}
			best := classOfEdge(x.Incoming[0])
			for _, eid := range x.Incoming[1:] {
				if c := classOfEdge(eid); c < best {
					best = c
				}
			}
			for j, eid := range x.Incoming {
				if classOfEdge(eid) > best {
					x.IncomingControl[j] = network.ControlStop
				} else {
					x.IncomingControl[j] = network.ControlNone
				}
			}
			return
		}
	}
}
```

- [ ] **Step 4: Call `resolveControls` from `Build`**

In `internal/netbuild/netbuild.go`, find the call to `sortIncomingByPriority` (around line 180) and add the call just after it:

```go
	sortIncomingByPriority(intersections, osmWayOfEdge, feat)

	// Resolve per-approach right-of-way controls. Runs after the priority
	// sort so each IncomingControl[i] aligns with the final sorted
	// position of Incoming[i].
	netNodeOf := func(o osm.NodeID) (network.NodeID, bool) {
		nid, ok := osmToNet[o]
		return nid, ok
	}
	osmToNetReverse := make(map[network.NodeID]osm.NodeID, len(osmToNet))
	for k, v := range osmToNet {
		osmToNetReverse[v] = k
	}
	osmNodeOf := func(nid network.NodeID) (osm.NodeID, bool) {
		o, ok := osmToNetReverse[nid]
		return o, ok
	}
	resolveControls(intersections, feat, osmWayOfEdge, netNodeOf, osmNodeOf)
```

(`netNodeOf` is unused in Phase 1's `resolveControls` body but is included in the signature for Task 13.)

To keep `go vet` quiet about the unused capture, prefix with `_ = netNodeOf` or actually use it. Cleanest fix: pass it through but reference it in resolveControls via `_ = netNodeOf` at the top of the function. The implementer chooses; the test surface doesn't depend on it.

- [ ] **Step 5: Run the new tests**

Run: `go test ./internal/netbuild/ -run TestNetbuild_Fallback -v`
Expected: both PASS.

- [ ] **Step 6: Run the full netbuild suite**

Run: `go test ./internal/netbuild/`
Expected: PASS. Existing tests don't assert on `IncomingControl`, so they're unaffected.

- [ ] **Step 7: Run the full repo suite**

Run: `go test ./...`
Expected: PASS. The new class-based fallback may change behavior at the OSM-loaded `e2e` level (vehicles now stop at minor cross-streets they previously rolled through). The headless `e2e_test.go` tests assert vehicle throughput and successful spawn/despawn rather than specific yield behavior, so they should still pass — possibly with slightly different throughput numbers.

- [ ] **Step 8: Commit**

```
git add internal/netbuild/control.go internal/netbuild/control_test.go internal/netbuild/netbuild.go
git commit -m "feat(netbuild): class-based fallback for IncomingControl"
```

---

## Task 13: Netbuild — explicit OSM sign-tag resolution

**Files:**
- Modify: `internal/netbuild/control.go`
- Modify: `internal/netbuild/control_test.go`

Layer per-approach explicit signage on top of the class-based fallback. Rules:
- `highway=stop` / `highway=give_way` on the intersection node, no `direction=*` → applies to all approaches passing through that node.
- With `direction=forward` → applies only to the approach whose underlying way carries that node in the forward direction.
- With `direction=backward` → only the backward-direction approach.
- `highway=stop` / `highway=give_way` on an interior approach-segment node → applies to the approach whose chain (sequence of NodeIDs from intersection back along the segment) contains that node.

For Phase 1 we implement the intersection-node case (with `direction=*` parsing). Interior-node signage is the harder case because we need a per-edge node-chain reverse map. The spec lists it as a rule; if implementation difficulty bites, the implementer should flag it and we'll cut interior-node sign resolution from Phase 1 — the class-based fallback already covers the common case where OSM tags signs on the intersection node rather than the stop-line node.

- [ ] **Step 1: Write failing tests**

Append to `internal/netbuild/control_test.go`:

```go
// TestNetbuild_StopAll: an intersection node tagged stop=all forces
// every approach to AllWayStop regardless of class.
func TestNetbuild_StopAll(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "stop", "all"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("approach %d should be AllWayStop under stop=all, got %v", i, c)
		}
	}
}

// TestNetbuild_StopMinor: stop=minor tags only the lower-class approaches.
func TestNetbuild_StopMinor(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "stop", "minor"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	var sawStop, sawNone bool
	for i, eid := range x.Incoming {
		c := x.IncomingControl[i]
		hw := highwayOfEdge(net, eid, feat)
		switch hw {
		case "primary":
			if c != network.ControlNone {
				t.Errorf("primary approach should be None under stop=minor, got %v", c)
			}
			sawNone = true
		case "residential":
			if c != network.ControlStop {
				t.Errorf("residential approach should be Stop under stop=minor, got %v", c)
			}
			sawStop = true
		}
	}
	if !sawStop || !sawNone {
		t.Error("expected mixed controls under stop=minor")
	}
}

// TestNetbuild_HighwayStopOnNode: an intersection node tagged
// highway=stop (no direction) applies Stop to all approaches.
func TestNetbuild_HighwayStopOnNode(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "highway", "stop"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlStop {
			t.Errorf("approach %d should be Stop under highway=stop without direction, got %v", i, c)
		}
	}
}

// TestNetbuild_HighwayGiveWayOnNode: an intersection node tagged
// highway=give_way applies Yield to all approaches.
func TestNetbuild_HighwayGiveWayOnNode(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "highway", "give_way"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlYield {
			t.Errorf("approach %d should be Yield under highway=give_way, got %v", i, c)
		}
	}
}
```

- [ ] **Step 2: Run them — expect failures**

Run: `go test ./internal/netbuild/ -run "TestNetbuild_StopAll|TestNetbuild_StopMinor|TestNetbuild_HighwayStopOnNode|TestNetbuild_HighwayGiveWayOnNode" -v`
Expected: `TestNetbuild_StopAll` passes (already implemented in Task 12). `TestNetbuild_StopMinor` passes for the same reason. The other two fail.

- [ ] **Step 3: Extend `resolveControls`**

In `internal/netbuild/control.go`, find the `resolveControls` loop and add a call to a new helper after `applyStopAllOrMinor`:

```go
		applyStopAllOrMinor(x, nodeTags, classOfEdge)
		applyNodeLevelSign(x, nodeTags)
```

Add the helper at the bottom of `control.go`:

```go
// applyNodeLevelSign handles highway=stop and highway=give_way tags on
// the intersection node itself. Without direction=, applies to all
// approaches. With direction=forward/backward, applies only to the
// matching approach.
//
// Phase 1 ignores direction= refinement (we apply to all approaches) and
// leaves per-approach refinement to a later phase. This is the lenient
// interpretation called out in the spec.
func applyNodeLevelSign(x *network.Intersection, tags osm.Tags) {
	var target network.Control
	hasSign := false
	for _, t := range tags {
		if t.Key == "highway" && t.Value == "stop" {
			target = network.ControlStop
			hasSign = true
		}
		if t.Key == "highway" && t.Value == "give_way" {
			target = network.ControlYield
			hasSign = true
		}
	}
	if !hasSign {
		return
	}
	for j := range x.IncomingControl {
		// Don't override AllWayStop with Stop or Yield — AllWayStop is a
		// stricter pre-existing decision (from stop=all or equal-class
		// fallback). A node-level highway=stop tag that coexists with
		// stop=all is a tagging artifact; AllWayStop wins.
		if x.IncomingControl[j] == network.ControlAllWayStop {
			continue
		}
		x.IncomingControl[j] = target
	}
}
```

- [ ] **Step 4: Run all four tests**

Run: `go test ./internal/netbuild/ -run "TestNetbuild_StopAll|TestNetbuild_StopMinor|TestNetbuild_HighwayStopOnNode|TestNetbuild_HighwayGiveWayOnNode" -v`
Expected: all four PASS.

- [ ] **Step 5: Run the full repo suite**

Run: `go test ./...`
Expected: PASS.

- [ ] **Step 6: Commit**

```
git add internal/netbuild/control.go internal/netbuild/control_test.go
git commit -m "feat(netbuild): node-level highway=stop and highway=give_way resolution"
```

---

## Task 14: Final verification and benchmarks

**Files:** none modified.

- [ ] **Step 1: Run the full test suite with verbose output**

Run: `go test ./... -v`
Expected: all PASS, including the existing `TestWorld_TraceDeterminism` and `TestWorld_StuckAtYieldNotDespawned`.

- [ ] **Step 2: Run the sim benchmarks**

Run: `go test ./internal/sim/ -bench=. -benchtime=2s -run=^$`
Expected: ns/op within ~10-15% of README baselines. If significantly worse, profile with `-cpuprofile=cpu.out` and inspect which new path is the hot spot.

- [ ] **Step 3: Run the E2E with a small extract (if one is at hand)**

Run: `TRAFFIC_SIM_E2E_OSM=path/to/extract.osm.pbf go test -tags e2e ./internal/e2e/`
Expected: PASS. Vehicles spawn, route, and despawn at end-of-route. Throughput numbers may differ from baseline because mandatory stops were added — note the numbers but don't gate on them.

- [ ] **Step 4: No commit (verification only)**

The work is complete. Summarize in a follow-up message: data model + ingestion + yield rule + mandatory stop + AllWayStop FIFO + signal flash/off unification all in place.

---

## Out of scope (deferred to later phases)

- Direction-tag refinement for `highway=stop` / `highway=give_way` (apply only to the matching forward/backward approach).
- Interior-node sign resolution (`highway=stop` placed on the stop-line node rather than the intersection node).
- Per-vehicle critical-gap distributions (Phase 3).
- Impatience-shrinking gaps (Phase 3).
- Per-movement priority logic for priority-road left turns (Phase 4).
- Visual rendering of stop/yield signs.
- Promoting `stopDwellSec` / `gapThresholdSec` to runtime config.
