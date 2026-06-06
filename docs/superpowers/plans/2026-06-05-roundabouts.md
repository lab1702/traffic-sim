# Roundabouts (Multi-Lane) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Model roundabouts as one-way circulating rings where entering traffic yields to circulating traffic (Phase A), and on multi-lane rings vehicles choose/weave lanes by exit position (Phase B).

**Architecture:** Reuse the existing per-approach `Control` + `yieldGapCheck` machinery. Netbuild marks circulating edges (`Edge.Roundabout`), forces the ring one-way, and assigns `ControlNone` to circulating approaches / `ControlYield` to entries; the sim then yields entries to circulating traffic with no new control logic. Phase B adds a route-forward-scan target-lane policy that feeds the existing `tryLaneChange` mechanics.

**Tech Stack:** Go. Packages: `internal/osmload`, `internal/netbuild`, `internal/network`, `internal/sim`. Tests are standard `go test` table/unit tests.

**Deviation from spec:** The spec proposed a `Network.Roundabouts` ring-grouping struct for exit ordering. Reading the code showed the chosen segment-count heuristic only needs a forward scan over the `Edge.Roundabout` flag along a vehicle's route — no persistent ring identity. This plan therefore drops `Network.Roundabouts` and keeps the data-model change to a single `Edge.Roundabout bool`. Everything else follows the spec.

**Handedness:** Right-hand traffic. Lane 0 = outer/right; higher index = inner/left. Rings circulate counterclockwise; entries yield to traffic from the left.

---

## File Structure

**Phase A**
- Modify `internal/netbuild/netbuild.go` — `isRoundabout` helper; `onewayDirection` recognizes `junction=roundabout`/`circular`; set `Roundabout` on constructed edges.
- Modify `internal/network/types.go` — add `Edge.Roundabout bool`.
- Modify `internal/network/hash.go` — fold `Roundabout` into the edge hash.
- Modify `internal/netbuild/control.go` — `applyRoundaboutControl`; call it in `resolveControls`.
- Modify `internal/sim/world.go` — `roundaboutGapSec` const; per-conflict-edge gap selection in `yieldGapCheck`.
- Tests: `internal/netbuild/netbuild_test.go`, `internal/netbuild/control_test.go`, `internal/network/hash_test.go`, `internal/sim/world_test.go`.

**Phase B**
- Modify `internal/sim/lanechange.go` — `roundaboutSegmentsToExit`, `roundaboutTargetLane`, and a roundabout branch in `tryLaneChange`.
- Modify `internal/sim/world.go` — `roundaboutWeaveLookahead` (K) const.
- Tests: `internal/sim/lanechange_test.go`, `internal/sim/world_test.go`.

---

# PHASE A — one-way ring, entry-yield, circulating priority

## Task A1: Recognize roundabouts as one-way

**Files:**
- Modify: `internal/netbuild/netbuild.go` (add `isRoundabout`; extend `onewayDirection`)
- Test: `internal/netbuild/netbuild_test.go`

- [ ] **Step 1: Write the failing test**

Add to `internal/netbuild/netbuild_test.go`:

```go
func TestOnewayDirection_Roundabout(t *testing.T) {
	rab := &osm.Way{Tags: osm.Tags{{Key: "highway", Value: "primary"}, {Key: "junction", Value: "roundabout"}}}
	if got := onewayDirection(rab); got != onewayForward {
		t.Fatalf("junction=roundabout: got %v, want onewayForward", got)
	}
	circ := &osm.Way{Tags: osm.Tags{{Key: "junction", Value: "circular"}}}
	if got := onewayDirection(circ); got != onewayForward {
		t.Fatalf("junction=circular: got %v, want onewayForward", got)
	}
	// Explicit oneway tag still wins over the junction implication.
	twoWay := &osm.Way{Tags: osm.Tags{{Key: "junction", Value: "roundabout"}, {Key: "oneway", Value: "no"}}}
	if got := onewayDirection(twoWay); got != onewayTwoWay {
		t.Fatalf("roundabout + oneway=no: got %v, want onewayTwoWay", got)
	}
}

func TestIsRoundabout(t *testing.T) {
	if !isRoundabout(&osm.Way{Tags: osm.Tags{{Key: "junction", Value: "roundabout"}}}) {
		t.Fatal("junction=roundabout should be a roundabout")
	}
	if !isRoundabout(&osm.Way{Tags: osm.Tags{{Key: "junction", Value: "circular"}}}) {
		t.Fatal("junction=circular should be a roundabout")
	}
	if isRoundabout(&osm.Way{Tags: osm.Tags{{Key: "highway", Value: "primary"}}}) {
		t.Fatal("plain primary should not be a roundabout")
	}
}
```

(If `osm` is not yet imported in this test file, add `"github.com/lab1702/traffic-sim/internal/osmload/osm"` — match the import path already used in `netbuild.go`.)

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/netbuild/ -run 'TestOnewayDirection_Roundabout|TestIsRoundabout' -v`
Expected: FAIL — `undefined: isRoundabout` (and the roundabout case of `onewayDirection` returns `onewayTwoWay`).

- [ ] **Step 3: Write minimal implementation**

In `internal/netbuild/netbuild.go`, add the helper near `onewayDirection`:

```go
// isRoundabout reports whether a way is a circulating roundabout ring.
// OSM tags these junction=roundabout (and the rarer junction=circular).
// Such ways are implicitly one-way even with no oneway tag.
func isRoundabout(w *osm.Way) bool {
	for _, t := range w.Tags {
		if t.Key == "junction" && (t.Value == "roundabout" || t.Value == "circular") {
			return true
		}
	}
	return false
}
```

Then, inside `onewayDirection`, after the explicit `oneway` tag switch and before the motorway rule, add the roundabout implication:

```go
	// junction=roundabout/circular is implicitly one-way (forward) unless an
	// explicit oneway tag above already decided. Checked before the motorway
	// rule; both are implicit-oneway sources.
	if isRoundabout(w) {
		return onewayForward
	}
```

(Placement matters: it must run only after the explicit-`oneway` switch has had its chance to `return`, so `oneway=no` still wins.)

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/netbuild/ -run 'TestOnewayDirection_Roundabout|TestIsRoundabout' -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/netbuild/netbuild.go internal/netbuild/netbuild_test.go
git commit -m "feat(netbuild): treat junction=roundabout as implicitly one-way"
```

---

## Task A2: Add and populate `Edge.Roundabout`

**Files:**
- Modify: `internal/network/types.go` (add field)
- Modify: `internal/netbuild/netbuild.go` (set field on constructed edges)
- Test: `internal/netbuild/netbuild_test.go`

- [ ] **Step 1: Write the failing test**

This test builds a tiny graph from an in-memory `Features` containing one roundabout ring way and asserts the ring edges are flagged and one-way. If the test file already has a `Features`-construction helper, reuse it; otherwise this uses the public `Build` entry point. Add to `internal/netbuild/netbuild_test.go`:

```go
func TestBuild_RoundaboutEdgesFlaggedOneWay(t *testing.T) {
	// A square ring of 4 nodes, tagged junction=roundabout, plus one
	// approach road meeting the ring at node 1.
	feat := squareRoundaboutFixture() // helper defined in Step 3
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	ringCount := 0
	for i := range net.Edges {
		e := &net.Edges[i]
		if e.Roundabout {
			ringCount++
		}
	}
	if ringCount != 4 {
		t.Fatalf("expected 4 one-way ring edges, got %d", ringCount)
	}
	// No reverse ring edge should exist: every ring edge's reverse
	// (To->From) must be absent.
	for i := range net.Edges {
		e := &net.Edges[i]
		if !e.Roundabout {
			continue
		}
		for j := range net.Edges {
			r := &net.Edges[j]
			if r.From == e.To && r.To == e.From && r.Roundabout {
				t.Fatalf("found a wrong-way ring edge %d->%d", r.From, r.To)
			}
		}
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/netbuild/ -run TestBuild_RoundaboutEdgesFlaggedOneWay -v`
Expected: FAIL — `e.Roundabout undefined` (field does not exist yet) and `squareRoundaboutFixture undefined`.

- [ ] **Step 3: Write minimal implementation**

(a) In `internal/network/types.go`, add the field to `Edge` (place it next to `Class` so related metadata stays together):

```go
	Class      RoadClass
	// Roundabout marks a circulating segment of a roundabout ring
	// (junction=roundabout/circular). Such edges carry priority at ring
	// nodes and never stop; entering approaches yield to them.
	Roundabout bool
	Geometry   []Point // polyline including endpoints
```

(b) In `internal/netbuild/netbuild.go`, inside the way loop (where `dir := onewayDirection(w)` is computed), add:

```go
		rab := isRoundabout(w)
```

Then add `Roundabout: rab,` to **each** of the four `network.Edge{...}` struct literals in the `switch dir` block (the `onewayForward`, `onewayReverse`, and both `onewayTwoWay` literals). Example for the forward case:

```go
			edges = append(edges, network.Edge{
				ID: network.EdgeID(len(edges)), From: fromID, To: toID,
				Lanes: makeLanes(lanesFwd), Length: length, SpeedLimit: speedFwd,
				Width: width, Class: class, Roundabout: rab, Geometry: geom,
			})
```

(c) Add the fixture helper in `internal/netbuild/netbuild_test.go`. Match the field/type names of the `osmload.Features` and `osm` structs already used elsewhere in this package's tests:

```go
// squareRoundaboutFixture builds a 4-node square ring tagged
// junction=roundabout, with one approach road meeting it at node 1.
// Coordinates are small lat/lon deltas so the projection yields a ring
// a few tens of meters across.
func squareRoundaboutFixture() *osmload.Features {
	f := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{}, Ways: nil}
	add := func(id osm.NodeID, lat, lon float64) {
		f.Nodes[id] = &osm.Node{ID: id, Lat: lat, Lon: lon}
	}
	add(1, 0.0000, 0.0000)
	add(2, 0.0003, 0.0000)
	add(3, 0.0003, 0.0003)
	add(4, 0.0000, 0.0003)
	add(5, -0.0005, 0.0000) // approach road endpoint, south of node 1
	ring := &osm.Way{
		ID:   100,
		Refs: []osm.NodeRef{{ID: 1}, {ID: 2}, {ID: 3}, {ID: 4}, {ID: 1}},
		Tags: osm.Tags{{Key: "highway", Value: "primary"}, {Key: "junction", Value: "roundabout"}},
	}
	approach := &osm.Way{
		ID:   101,
		Refs: []osm.NodeRef{{ID: 5}, {ID: 1}},
		Tags: osm.Tags{{Key: "highway", Value: "secondary"}},
	}
	f.Ways = []*osm.Way{ring, approach}
	return f
}
```

> Note: field names (`Refs`, `NodeRef`, `Lat`/`Lon`, `Tags`) must match this repo's `osm` package. If they differ, adapt the fixture to the real struct shapes — do not invent new ones. Confirm by reading an existing `netbuild` test that constructs `Features`.

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/netbuild/ -run TestBuild_RoundaboutEdgesFlaggedOneWay -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/network/types.go internal/netbuild/netbuild.go internal/netbuild/netbuild_test.go
git commit -m "feat(netbuild): flag circulating edges with Edge.Roundabout"
```

---

## Task A3: Fold `Roundabout` into the network hash

**Files:**
- Modify: `internal/network/hash.go`
- Test: `internal/network/hash_test.go`

- [ ] **Step 1: Write the failing test**

Add to `internal/network/hash_test.go`:

```go
func TestHash_RoundaboutFlagMatters(t *testing.T) {
	base := &Network{
		Edges: []Edge{{ID: 0, From: 0, To: 1, Length: 10, SpeedLimit: 10, Lanes: make([]Lane, 1)}},
	}
	withRab := &Network{
		Edges: []Edge{{ID: 0, From: 0, To: 1, Length: 10, SpeedLimit: 10, Lanes: make([]Lane, 1), Roundabout: true}},
	}
	if Hash(base) == Hash(withRab) {
		t.Fatal("Roundabout flag must change the network hash")
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/network/ -run TestHash_RoundaboutFlagMatters -v`
Expected: FAIL — hashes are equal (flag not yet hashed).

- [ ] **Step 3: Write minimal implementation**

In `internal/network/hash.go`, inside the `for i := range net.Edges` loop, after `putU32(uint32(len(e.Lanes)))`, add:

```go
		if e.Roundabout {
			putU8(1)
		} else {
			putU8(0)
		}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/network/ -run TestHash_RoundaboutFlagMatters -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/network/hash.go internal/network/hash_test.go
git commit -m "feat(network): include Edge.Roundabout in the network hash"
```

---

## Task A4: Assign roundabout right-of-way (circulating priority, entries yield)

**Files:**
- Modify: `internal/netbuild/control.go` (add `applyRoundaboutControl`; call it in `resolveControls`)
- Test: `internal/netbuild/control_test.go`

- [ ] **Step 1: Write the failing test**

Add to `internal/netbuild/control_test.go`:

```go
func TestApplyRoundaboutControl(t *testing.T) {
	// Node with two incoming edges: edge 0 is a circulating ring segment,
	// edge 1 is an entering approach road.
	edges := []network.Edge{
		{ID: 0, Roundabout: true},
		{ID: 1, Roundabout: false},
	}
	x := &network.Intersection{
		Incoming:        []network.EdgeID{0, 1},
		IncomingControl: make([]network.Control, 2),
	}
	if !applyRoundaboutControl(x, edges) {
		t.Fatal("expected applyRoundaboutControl to report the node as on-ring")
	}
	if x.IncomingControl[0] != network.ControlNone {
		t.Errorf("circulating approach: got %v, want ControlNone", x.IncomingControl[0])
	}
	if x.IncomingControl[1] != network.ControlYield {
		t.Errorf("entering approach: got %v, want ControlYield", x.IncomingControl[1])
	}

	// A node with no ring edge is left untouched and reports false.
	plain := []network.Edge{{ID: 0}, {ID: 1}}
	y := &network.Intersection{Incoming: []network.EdgeID{0, 1}, IncomingControl: make([]network.Control, 2)}
	if applyRoundaboutControl(y, plain) {
		t.Fatal("non-ring node should report false")
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/netbuild/ -run TestApplyRoundaboutControl -v`
Expected: FAIL — `undefined: applyRoundaboutControl`.

- [ ] **Step 3: Write minimal implementation**

(a) Add to `internal/netbuild/control.go`:

```go
// applyRoundaboutControl handles a node on a roundabout ring: circulating
// approaches (Roundabout edges) keep priority (ControlNone) and never stop;
// entering approaches yield (ControlYield). Returns true if the node is on a
// ring (has at least one incoming Roundabout edge), in which case the caller
// must skip the normal class/sign resolution chain. The sim's yieldGapCheck
// then makes entries yield to the circulating edge automatically, since the
// only ControlNone approach at the node is the circulating one.
func applyRoundaboutControl(x *network.Intersection, edges []network.Edge) bool {
	onRing := false
	for _, eid := range x.Incoming {
		if int(eid) < len(edges) && edges[eid].Roundabout {
			onRing = true
			break
		}
	}
	if !onRing {
		return false
	}
	for j, eid := range x.Incoming {
		if int(eid) < len(edges) && edges[eid].Roundabout {
			x.IncomingControl[j] = network.ControlNone
		} else {
			x.IncomingControl[j] = network.ControlYield
		}
	}
	return true
}
```

(b) Wire it into `resolveControls` in the same file. Inside the `for i := range xs` loop, **after** the `distinctNeighbors < 3` early-continue and **before** the `applyClassFallback` call, insert:

```go
		// Roundabout ring node: circulating priority, entries yield. Skip
		// the class/sign chain entirely. Signalled ring nodes (rare) fall
		// through to the normal + signal path instead.
		if !x.HasSignal && applyRoundaboutControl(x, edges) {
			continue
		}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/netbuild/ -run TestApplyRoundaboutControl -v`
Expected: PASS.

- [ ] **Step 5: Run the full netbuild suite (no regressions)**

Run: `go test ./internal/netbuild/`
Expected: PASS (the recent junction/AllWayStop fixes must still hold — the roundabout branch is gated on a Roundabout incoming edge, so non-ring nodes are untouched).

- [ ] **Step 6: Commit**

```bash
git add internal/netbuild/control.go internal/netbuild/control_test.go
git commit -m "feat(netbuild): roundabout entries yield, circulating keeps priority"
```

---

## Task A5: Roundabout-tuned entry critical gap in the sim

**Files:**
- Modify: `internal/sim/world.go` (`roundaboutGapSec` const; per-conflict-edge gap in `yieldGapCheck`)
- Test: `internal/sim/world_test.go`

- [ ] **Step 1: Write the failing test**

This is an integration test on a small ring world. Use the existing `world_test.go` helpers for constructing a `World` from a `*network.Network`; if a helper like `newTestWorld` exists, reuse it. Add to `internal/sim/world_test.go`:

```go
func TestRoundabout_EntryYieldsCirculatingProceeds(t *testing.T) {
	// Build a small ring with one approach. A circulating vehicle is close
	// to the entry node; an entering vehicle is at the stop line. The
	// entering vehicle must not cross (yields); the circulating vehicle
	// must keep moving (never stops).
	net := ringWorldFixture() // helper; reuse squareRoundaboutFixture via netbuild.Build
	w := newTestWorld(t, net)

	circ := w.spawnOnEdge(t, ringInEdge(net), /*s=*/ ringLen(net)-5, /*v=*/ 8)
	enter := w.spawnOnEdge(t, approachEdge(net), /*s=*/ approachLen(net)-1, /*v=*/ 2)

	for tick := 0; tick < 20; tick++ {
		w.Step()
	}

	if w.Vehicles[circ].V < 1.0 {
		t.Errorf("circulating vehicle stopped (V=%.2f); it has priority", w.Vehicles[circ].V)
	}
	// The entering vehicle should still be on the approach edge (yielded),
	// not yet on the ring, while the circulating vehicle is in the conflict
	// zone.
	if w.Vehicles[enter].Edge != approachEdge(net) {
		t.Errorf("entering vehicle crossed into the ring without yielding")
	}
}
```

> The exact helper names (`newTestWorld`, `spawnOnEdge`, `Step`, edge-lookup helpers) must match what `world_test.go` already provides. Read the existing tests first and adapt these calls; build the missing small helpers (`ringWorldFixture`, `ringInEdge`, `approachEdge`, `ringLen`, `approachLen`) on top of `netbuild.Build(squareRoundaboutFixture())`.

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/sim/ -run TestRoundabout_EntryYieldsCirculatingProceeds -v`
Expected: FAIL initially (helpers undefined, or — once helpers compile — the entering vehicle crosses because the entry gap is the looser 3.0 s straight-crossing value rather than the roundabout value). The yield itself already works via Task A4; this test also locks in the tuned gap added below.

- [ ] **Step 3: Write minimal implementation**

(a) In `internal/sim/world.go`, near `leftTurnGapSec`, add:

```go
// roundaboutGapSec is the minimum ETA of circulating traffic a vehicle
// accepts before entering a roundabout. Entry critical gaps run ~3-4 s;
// slightly above the straight-crossing gapThresholdSec because entering a
// moving ring is less forgiving. Shrinkable by effectiveGap/impatience like
// leftTurnGapSec.
const roundaboutGapSec = 3.5
```

(b) In `yieldGapCheck`, select the base gap per conflicting approach. Replace the single comparison line:

```go
			if d/ovV < effectiveGap(v, gapThresholdSec) {
				return myDist, true
			}
```

with:

```go
			baseGap := gapThresholdSec
			if otherEdge.Roundabout {
				baseGap = roundaboutGapSec
			}
			if d/ovV < effectiveGap(v, baseGap) {
				return myDist, true
			}
```

(`otherEdge` is already bound earlier in the loop to `&w.Net.Edges[x.Incoming[j]]`.)

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/sim/ -run TestRoundabout_EntryYieldsCirculatingProceeds -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): roundabout-tuned entry critical gap"
```

---

## Task A6: Confirm determinism still holds

**Files:**
- Test only: `internal/sim/world_test.go` (existing `TestWorld_TraceDeterminism`)

- [ ] **Step 1: Run the determinism test**

Run: `go test ./internal/sim/ -run TestWorld_TraceDeterminism -v`
Expected: PASS — roundabout control is derived deterministically at build time and flows into the hash via Task A3, so identical inputs still produce a byte-identical trace.

- [ ] **Step 2: Run the full suite**

Run: `go test ./...`
Expected: PASS across all packages. Phase A is complete and independently shippable here.

- [ ] **Step 3: Commit (only if anything changed)**

If the run revealed and required a fix, commit it. Otherwise no commit — Phase A ends at Task A5.

---

# PHASE B — multi-lane lane discipline

Phase B changes behavior only where ring edges have ≥2 lanes; single-lane rings are unaffected.

## Task B1: Exit distance from the route

**Files:**
- Modify: `internal/sim/lanechange.go` (`roundaboutSegmentsToExit`)
- Test: `internal/sim/lanechange_test.go`

- [ ] **Step 1: Write the failing test**

Add to `internal/sim/lanechange_test.go`:

```go
func TestRoundaboutSegmentsToExit(t *testing.T) {
	// Route: approach (non-ring) -> ring0 -> ring1 -> ring2 -> exit (non-ring).
	net := &network.Network{Edges: []network.Edge{
		{ID: 0, Roundabout: false}, // approach
		{ID: 1, Roundabout: true},  // ring seg
		{ID: 2, Roundabout: true},  // ring seg
		{ID: 3, Roundabout: true},  // ring seg
		{ID: 4, Roundabout: false}, // exit
	}}
	v := &Vehicle{Route: []network.EdgeID{0, 1, 2, 3, 4}}

	// Sitting on the first ring segment (RouteIdx 1): three ring segments
	// remain before the exit edge (4) -> distance 3.
	v.RouteIdx = 1
	if got := roundaboutSegmentsToExit(v, net); got != 3 {
		t.Errorf("on ring0: got %d, want 3", got)
	}
	// On the last ring segment (RouteIdx 3): the very next edge is the exit,
	// so distance 1.
	v.RouteIdx = 3
	if got := roundaboutSegmentsToExit(v, net); got != 1 {
		t.Errorf("on ring2: got %d, want 1", got)
	}
	// Not on a ring edge -> 0 (sentinel: "not applicable").
	v.RouteIdx = 0
	if got := roundaboutSegmentsToExit(v, net); got != 0 {
		t.Errorf("on approach: got %d, want 0", got)
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/sim/ -run TestRoundaboutSegmentsToExit -v`
Expected: FAIL — `undefined: roundaboutSegmentsToExit`.

- [ ] **Step 3: Write minimal implementation**

Add to `internal/sim/lanechange.go`:

```go
// roundaboutSegmentsToExit returns how many ring segments the vehicle will
// traverse, counting from its current edge, before leaving the roundabout.
// The exit is the first non-Roundabout edge in the remaining route. Returns
// the count including the current segment (1 == "exit immediately after this
// segment"). Returns 0 when the vehicle is not currently on a ring edge.
func roundaboutSegmentsToExit(v *Vehicle, net *network.Network) int {
	if int(v.Edge) >= len(net.Edges) || !net.Edges[v.Edge].Roundabout {
		return 0
	}
	count := 0
	for i := v.RouteIdx; i < len(v.Route); i++ {
		e := v.Route[i]
		if int(e) >= len(net.Edges) || !net.Edges[e].Roundabout {
			break
		}
		count++
	}
	return count
}
```

> Note: the test sets `v.RouteIdx` but the helper keys "am I on a ring edge" off `v.Edge`. In the test, set `v.Edge` to match the route position, or simplify the helper to read `v.Route[v.RouteIdx]`. Use whichever the surrounding `Vehicle` invariant guarantees (`v.Edge == v.Route[v.RouteIdx]` is the established invariant in this codebase — verify in `world.go` and rely on it). Adjust the test's `v.Edge` accordingly so the two agree.

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/sim/ -run TestRoundaboutSegmentsToExit -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/lanechange.go internal/sim/lanechange_test.go
git commit -m "feat(sim): compute roundabout exit distance from the route"
```

---

## Task B2: Target-lane policy (tags first, then heuristic)

**Files:**
- Modify: `internal/sim/world.go` (`roundaboutWeaveLookahead` const)
- Modify: `internal/sim/lanechange.go` (`roundaboutTargetLane`)
- Test: `internal/sim/lanechange_test.go`

- [ ] **Step 1: Write the failing test**

Add to `internal/sim/lanechange_test.go`:

```go
func TestRoundaboutTargetLane(t *testing.T) {
	// 2-lane ring (lanes 0=outer, 1=inner), no turn:lanes tags.
	twoLane := func(rab bool) network.Edge {
		return network.Edge{Roundabout: rab, Lanes: []network.Lane{{Index: 0}, {Index: 1}}}
	}
	net := &network.Network{Edges: []network.Edge{
		twoLane(false), // 0 approach
		twoLane(true),  // 1 ring
		twoLane(true),  // 2 ring
		twoLane(true),  // 3 ring
		twoLane(false), // 4 exit
	}}

	// Far from exit (3 segments, K=1) -> inner lane (highest index).
	v := &Vehicle{Edge: 1, RouteIdx: 1, Route: []network.EdgeID{0, 1, 2, 3, 4}}
	if lane, ok := roundaboutTargetLane(v, net); !ok || lane != 1 {
		t.Errorf("far from exit: got (lane=%d, ok=%v), want (1, true)", lane, ok)
	}

	// Within K of exit (1 segment) -> outer lane 0.
	v.Edge, v.RouteIdx = 3, 3
	if lane, ok := roundaboutTargetLane(v, net); !ok || lane != 0 {
		t.Errorf("at exit: got (lane=%d, ok=%v), want (0, true)", lane, ok)
	}

	// Not on a ring -> not applicable.
	v.Edge, v.RouteIdx = 0, 0
	if _, ok := roundaboutTargetLane(v, net); ok {
		t.Errorf("on approach: expected ok=false")
	}
}

func TestRoundaboutTargetLane_HonorsTags(t *testing.T) {
	// Ring segment whose lane 0 explicitly allows the continuation to the
	// next ring edge; the tag must win over the far-from-exit heuristic.
	ring := network.Edge{Roundabout: true, Lanes: []network.Lane{
		{Index: 0, AllowedTurns: []network.EdgeID{2}}, // outer feeds the continuation
		{Index: 1, AllowedTurns: []network.EdgeID{99}},
	}}
	net := &network.Network{Edges: []network.Edge{
		{Roundabout: false},                         // 0 approach
		ring,                                         // 1 ring (current)
		{Roundabout: true, Lanes: make([]network.Lane, 2)},  // 2 ring (next)
		{Roundabout: false},                         // 3 exit
	}}
	v := &Vehicle{Edge: 1, RouteIdx: 1, Route: []network.EdgeID{0, 1, 2, 3}}
	if lane, ok := roundaboutTargetLane(v, net); !ok || lane != 0 {
		t.Errorf("tagged lane: got (lane=%d, ok=%v), want (0, true)", lane, ok)
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/sim/ -run 'TestRoundaboutTargetLane' -v`
Expected: FAIL — `undefined: roundaboutTargetLane`.

- [ ] **Step 3: Write minimal implementation**

(a) In `internal/sim/world.go`, near the other roundabout/gap constants, add:

```go
// roundaboutWeaveLookahead (K) is how many ring segments before its exit a
// vehicle begins migrating to the outer lane (lane 0). 1 means "start the
// weave-out on the last ring segment before exiting." Tunable by viewer
// observation.
const roundaboutWeaveLookahead = 1
```

(b) Add to `internal/sim/lanechange.go`:

```go
// roundaboutTargetLane returns the lane a circulating vehicle should occupy,
// and ok=false when the vehicle is not on a multi-lane ring (so callers fall
// back to normal lane-change logic). Policy:
//   - tags first: if the current ring edge's lane AllowedTurns encode which
//     lane feeds the vehicle's next route edge, target the nearest such lane;
//   - otherwise heuristic by exit distance: within roundaboutWeaveLookahead
//     segments of the exit -> outer lane 0; farther -> inner lane.
func roundaboutTargetLane(v *Vehicle, net *network.Network) (uint8, bool) {
	if int(v.Edge) >= len(net.Edges) {
		return 0, false
	}
	edge := &net.Edges[v.Edge]
	if !edge.Roundabout || len(edge.Lanes) < 2 {
		return 0, false
	}
	nLanes := uint8(len(edge.Lanes))

	// Tags first: honor AllowedTurns toward the next route edge when present.
	if v.RouteIdx+1 < len(v.Route) {
		nextE := v.Route[v.RouteIdx+1]
		if lane, _, ok := nearestCompatibleLane(edge.Lanes, v.Lane, nextE); ok {
			// Only treat as a tag signal if some lane actually constrains the
			// turn (non-empty AllowedTurns); nearestCompatibleLane returns the
			// current lane for the "any" case, which we don't want to force.
			anyConstrained := false
			for i := range edge.Lanes {
				if len(edge.Lanes[i].AllowedTurns) > 0 {
					anyConstrained = true
					break
				}
			}
			if anyConstrained {
				return lane, true
			}
		}
	}

	// Heuristic by exit distance.
	if roundaboutSegmentsToExit(v, net) <= roundaboutWeaveLookahead {
		return 0, true // outer lane to exit
	}
	return nLanes - 1, true // inner lane while circulating
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/sim/ -run 'TestRoundaboutTargetLane' -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/world.go internal/sim/lanechange.go internal/sim/lanechange_test.go
git commit -m "feat(sim): roundabout target-lane policy (tags first, then heuristic)"
```

---

## Task B3: Drive weaving through the existing lane-change machinery

**Files:**
- Modify: `internal/sim/lanechange.go` (`tryLaneChange` roundabout branch)
- Test: `internal/sim/lanechange_test.go`, `internal/sim/world_test.go`

- [ ] **Step 1: Write the failing test**

Add a focused unit test that a circulating vehicle in the inner lane, within K of its exit, migrates one step toward lane 0 when the gap is safe. Add to `internal/sim/lanechange_test.go`:

```go
func TestTryLaneChange_RoundaboutWeavesToOuter(t *testing.T) {
	net := &network.Network{Edges: []network.Edge{
		{ID: 0, Roundabout: false, Length: 100, Lanes: make([]network.Lane, 2)},
		{ID: 1, Roundabout: true, Length: 30, Lanes: make([]network.Lane, 2)}, // current ring seg
		{ID: 2, Roundabout: false, Length: 100, Lanes: make([]network.Lane, 2)}, // exit
	}}
	v := Vehicle{Edge: 1, S: 5, V: 6, Lane: 1, RouteIdx: 1, Route: []network.EdgeID{0, 1, 2}}
	vs := []Vehicle{v}
	lanes := map[uint8][]int{1: {0}} // ego alone on inner lane; outer lane empty
	tryLaneChange(&vs[0], 0, lanes, vs, net, -1)
	if vs[0].Lane != 0 {
		t.Errorf("expected weave to outer lane 0, got lane %d", vs[0].Lane)
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `go test ./internal/sim/ -run TestTryLaneChange_RoundaboutWeavesToOuter -v`
Expected: FAIL — the vehicle stays in lane 1 (no roundabout branch yet; the speed-driven path finds no faster lane).

- [ ] **Step 3: Write minimal implementation**

In `internal/sim/lanechange.go`, inside `tryLaneChange`, add a roundabout branch **after** the incident-vacate block and **before** the turn-bias context block (it takes precedence over speed-driven LC, like turn bias, but never overrides safety gaps):

```go
	// Roundabout weave: on a multi-lane ring, migrate one step toward the
	// target lane (inner while circulating, outer within K of the exit).
	// Reuses the same safety-gap checks as the other modes.
	if target, ok := roundaboutTargetLane(v, net); ok && target != v.Lane {
		dl := int8(1)
		if target < v.Lane {
			dl = -1
		}
		nl := int(v.Lane) + int(dl)
		if nl >= 0 && nl < int(numLanes) {
			other := laneVehicles[uint8(nl)]
			frontS, hasFront := nextAheadS(other, vs, v.Edge, v.S)
			rearS, hasRear := nextBehindS(other, vs, v.Edge, v.S)
			frontOK := !hasFront || frontS-v.S-VehicleLength >= safetyGapFront
			rearOK := !hasRear || v.S-rearS-VehicleLength >= safetyGapRear
			if frontOK && rearOK {
				v.Lane = uint8(nl)
				v.LaneChangeCooldown = laneChangeCooldown
				v.LastLCDir = dl
			}
		}
		return // on a ring, the weave policy owns lane choice this tick
	}
```

(`numLanes` is already computed at the top of `tryLaneChange` as `uint8(len(edge.Lanes))`, and the `numLanes < 2` early-return above this block means a single-lane ring never reaches here.)

- [ ] **Step 4: Run test to verify it passes**

Run: `go test ./internal/sim/ -run TestTryLaneChange_RoundaboutWeavesToOuter -v`
Expected: PASS.

- [ ] **Step 5: Write the integration test (weave + exit; missed-exit loops)**

Add to `internal/sim/world_test.go` an integration test on a 2-lane ring built via `netbuild.Build` of a multi-lane variant of the square fixture (ring way tagged `lanes=2`). Assert: (a) a vehicle entering with a late exit uses the inner lane then reaches lane 0 and exits onto the correct edge within a bounded number of ticks; (b) a vehicle boxed out of lane 0 at its exit stays on the ring (its `Edge` is still a ring edge) rather than teleporting off. Use the existing world-stepping helpers.

```go
func TestRoundabout_MultiLaneWeaveAndExit(t *testing.T) {
	net := multiLaneRingFixture(t) // 2-lane ring + two approaches/exits
	w := newTestWorld(t, net)
	v := w.spawnRouted(t, /*from*/ approachA(net), /*to*/ exitFar(net)) // late exit
	exited := false
	for tick := 0; tick < 400; tick++ {
		w.Step()
		if w.Vehicles[v].Edge == exitFar(net) {
			exited = true
			break
		}
	}
	if !exited {
		t.Fatalf("vehicle never reached its far exit after weaving")
	}
}
```

> Build `multiLaneRingFixture`, `approachA`, `exitFar`, `spawnRouted` on top of the existing helpers. The ring way carries `{Key:"lanes", Value:"2"}`. If routing helpers don't exist, set `v.Route` explicitly to `[approach, ring…, exit]` using edge lookups.

- [ ] **Step 6: Run the integration test**

Run: `go test ./internal/sim/ -run TestRoundabout_MultiLaneWeaveAndExit -v`
Expected: PASS.

- [ ] **Step 7: Run the full suite**

Run: `go test ./...`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add internal/sim/lanechange.go internal/sim/lanechange_test.go internal/sim/world_test.go
git commit -m "feat(sim): weave to the outer lane before a roundabout exit"
```

---

## Task B4: Determinism + benchmarks regression check

**Files:**
- Test only.

- [ ] **Step 1: Determinism**

Run: `go test ./internal/sim/ -run TestWorld_TraceDeterminism -v`
Expected: PASS — lane choice is a pure function of positions/route, so trace determinism holds.

- [ ] **Step 2: Benchmarks (no budget blowout)**

Run: `go test ./internal/sim/ -bench=. -benchtime=1s -run=^$`
Expected: per-tick times remain well under the 50 ms budget (the roundabout branch is O(lanes) per vehicle only on ring edges). Record numbers; if a regression appears, investigate before proceeding.

- [ ] **Step 3: No commit unless a fix was needed.**

---

## Task B5: E2E + viewer acceptance (manual, gated)

**Files:**
- Possibly add: `internal/e2e/` assertion (behind `-tags e2e`).

- [ ] **Step 1: Pick a test extract**

Choose an OSM extract containing a multi-lane roundabout from `/home/lab/MEGA/OpenStreetMap/*.osm`. Record the chosen file in the PR description. (Open item #1 from the spec.)

- [ ] **Step 2: E2E assertion**

Add an `-tags e2e` test (mirroring the existing `internal/e2e` pattern) that builds the network from the chosen extract and asserts at least one roundabout exists whose ring edges are one-way (`Roundabout && no reverse twin`) and whose entry approaches are `ControlYield`.

Run: `TRAFFIC_SIM_E2E_OSM=/home/lab/MEGA/OpenStreetMap/<file>.osm go test -tags e2e ./internal/e2e/ -run Roundabout -v`
Expected: PASS.

- [ ] **Step 3: Viewer acceptance (final word per realism-priority rule)**

Run: `./trafficsim run --spawn-rate 20 /home/lab/MEGA/OpenStreetMap/<file>.osm`
Observe at a multi-lane roundabout: entries yield to circulating traffic; circulating flow never hard-stops; vehicles weave out to the outer lane and take the correct exit; no wrong-way circulation and no phantom stops on the ring. Note any artifacts as follow-up tasks.

- [ ] **Step 4: Commit any E2E test added**

```bash
git add internal/e2e/
git commit -m "test(e2e): assert roundabout ring is one-way and entries yield"
```

---

## Self-review notes (already reconciled)

- **Spec coverage:** geometry fix (A1), ring flag (A2), hash (A3), entry-yield + circulating priority (A4), tuned gap (A5), determinism (A6/B4), exit-distance (B1), tags-first+heuristic lane policy (B2), weaving + missed-exit-loops (B3), E2E + viewer (B5). The only spec element intentionally dropped is `Network.Roundabouts` grouping — see the deviation note at the top; its sole consumer (exit ordering) is served by the route scan in B1.
- **Type consistency:** `roundaboutSegmentsToExit`, `roundaboutTargetLane`, `applyRoundaboutControl`, and the `roundaboutGapSec` / `roundaboutWeaveLookahead` constants are referenced with the same names throughout.
- **Fixture caveat:** the `osm`/`osmload` struct field names in the test fixtures (`Refs`, `NodeRef`, `Lat`, `Lon`, `Tags`) must be reconciled against this repo's actual types before the tests compile — flagged inline at each fixture.
