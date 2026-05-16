# Left-Turn Yield (Per-Movement Priority) — Phase 4 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make left turners yield to opposing through-and-right traffic at unsignalized priority-road intersections, signaled normal-green permissive lefts, and AllWayStop opposing FIFO ties. Closes the most visible artifact remaining after Phase 1.

**Architecture:** A new per-intersection `Opposing []int8` slice (parallel to `Incoming`) is computed once at netbuild time using the same 8-bucket axis logic from `DefaultSignalConfig`, with a `|Δheading| > π/2` filter. A new `leftTurnYieldsToOpposing` virtual-leader check is layered on top of Phase 1's yield rules: it engages when a vehicle is turning left, has an opposing approach, and is otherwise entitled to proceed. Mutual-left resolution lets two opposing left turners pass simultaneously.

**Tech Stack:** Go 1.x; no new external dependencies.

**Spec:** `docs/superpowers/specs/2026-05-15-left-turn-yield-phase-4-design.md`

---

## File map

| File | Change |
|---|---|
| `internal/network/types.go` | Add `Intersection.Opposing []int8` field. |
| `internal/netbuild/netbuild.go` | Allocate `Opposing` (length `len(incE)`, default `-1`) in `buildIntersections`; call `resolveOpposing` after `resolveControls` in `Build`. |
| `internal/netbuild/priority.go` | Extend `sortIncomingByPriority` co-sort to permute `Opposing` slot values AND remap stored indices via the inverse permutation. |
| `internal/netbuild/control.go` | Add `resolveOpposing` + `angleDiff` helpers. |
| `internal/netbuild/control_test.go` | Four new tests (FourWay, TThrough, Symmetric, CoSortsWithIncoming). |
| `internal/sim/world.go` | Add `leftTurnGapSec` constant; add `leftTurnYieldsToOpposing` + `entitledToProceed` helpers; wire into `Step`'s virtual-leader pass; extend stuck-vehicle guard. |
| `internal/sim/world_test.go` | Eight new behavioral tests (PriorityRoad_Yields, PriorityRoad_NoOpposing, MutualLefts, SignaledGreen_Yields, SignaledRed_NotAffected, AllWayStop_Yields, AllWayStop_BothLefts, StuckGuardBypassed). |

---

## Task 1: Add `Opposing` field on `Intersection`, init to `-1` in netbuild

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/network/types.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/netbuild.go`

- [ ] **Step 1: Add the field**

In `internal/network/types.go`, in the `Intersection` struct, after `IncomingControl`:

```go
type Intersection struct {
	ID              IntersectionID
	NodeID          NodeID
	Incoming        []EdgeID
	IncomingControl []Control
	// Opposing is parallel to Incoming: Opposing[i] is the position of
	// approach i's opposing approach (the same road's other direction),
	// or -1 if none. Populated by netbuild after sortIncomingByPriority.
	// Symmetric: Opposing[Opposing[i]] == i whenever Opposing[i] != -1.
	Opposing []int8
	Outgoing []EdgeID
	HasSignal bool
	BannedTurns []TurnRestriction
}
```

- [ ] **Step 2: Initialize in `buildIntersections`**

In `internal/netbuild/netbuild.go`, find the loop in `buildIntersections` where `IncomingControl` is allocated and the Intersection literal is appended. Currently:

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

Replace with:

```go
		ctrl := make([]network.Control, len(incE))
		opp := make([]int8, len(incE))
		for k := range opp {
			opp[k] = -1
		}
		// Defaults: ControlNone for every approach; Opposing[i] = -1.
		// Real values are assigned later in resolveControls and
		// resolveOpposing (both called after sortIncomingByPriority).
		xs = append(xs, network.Intersection{
			ID:              network.IntersectionID(len(xs)),
			NodeID:          n.ID,
			Incoming:        incE,
			IncomingControl: ctrl,
			Opposing:        opp,
			Outgoing:        outE,
			HasSignal:       signalNodes[n.ID],
		})
```

- [ ] **Step 3: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0.

- [ ] **Step 4: Run all tests**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: PASS. The new field is unused everywhere; behavior unchanged.

- [ ] **Step 5: Commit**

```
git add internal/network/types.go internal/netbuild/netbuild.go
git commit -m "feat(network): add Intersection.Opposing field; init to -1 in netbuild"
```

---

## Task 2: Extend `sortIncomingByPriority` to co-sort and remap `Opposing`

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/priority.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

The existing co-sort handles `Incoming` and `IncomingControl`. We add `Opposing`, but with one extra wrinkle: `Opposing` values are **indices into `Incoming`**, so after permutation each value must be remapped via the inverse permutation.

- [ ] **Step 1: Write failing test**

Append to `internal/netbuild/control_test.go`:

```go
// TestNetbuild_Opposing_CoSortsWithIncoming: force a non-trivial
// priority sort and verify that Opposing indices are correctly
// remapped to point at the new positions.
func TestNetbuild_Opposing_CoSortsWithIncoming(t *testing.T) {
	// 4-way: a "service" road (lower priority) meets a "primary" road.
	// Pre-sort, Incoming order is whatever buildIntersections produces.
	// Post-sort, the primary approaches must be at positions 0 and 1.
	// Manually set Opposing on a fixture to verify remapping.
	xs := []network.Intersection{
		{
			ID:       0,
			NodeID:   0,
			Incoming: []network.EdgeID{0, 1, 2, 3},
			Opposing: []int8{1, 0, 3, 2}, // pre-sort opposition
		},
	}
	// osmWayOfEdge[i] = WayID, so we can fake highway priorities via the
	// way tags. Edges 0, 1 are on a service road; 2, 3 on a primary.
	feat := &osmload.Features{}
	feat.Ways = []*osm.Way{
		{ID: 100, Tags: []osm.Tag{{Key: "highway", Value: "service"}}},
		{ID: 200, Tags: []osm.Tag{{Key: "highway", Value: "primary"}}},
	}
	osmWayOfEdge := []osm.WayID{100, 100, 200, 200}

	sortIncomingByPriority(xs, osmWayOfEdge, feat)

	x := xs[0]
	// Verify Incoming is now [2, 3, 0, 1] (primary first, then service).
	wantIncoming := []network.EdgeID{2, 3, 0, 1}
	for i := range wantIncoming {
		if x.Incoming[i] != wantIncoming[i] {
			t.Errorf("Incoming[%d] = %d, want %d", i, x.Incoming[i], wantIncoming[i])
		}
	}
	// Verify Opposing was remapped:
	//   Pre-sort: Incoming=[0,1,2,3], Opposing=[1,0,3,2]
	//   Post-sort: Incoming=[2,3,0,1]  (old positions 2,3,0,1 -> new 0,1,2,3)
	//   So oldToNew = {0:2, 1:3, 2:0, 3:1}.
	//   New Opposing[new_i] = oldToNew[old Opposing[old_i]].
	//   - newI=0 was oldI=2; oldOpposing[2]=3; remapped to oldToNew[3]=1.
	//   - newI=1 was oldI=3; oldOpposing[3]=2; remapped to oldToNew[2]=0.
	//   - newI=2 was oldI=0; oldOpposing[0]=1; remapped to oldToNew[1]=3.
	//   - newI=3 was oldI=1; oldOpposing[1]=0; remapped to oldToNew[0]=2.
	wantOpposing := []int8{1, 0, 3, 2}
	for i := range wantOpposing {
		if x.Opposing[i] != wantOpposing[i] {
			t.Errorf("Opposing[%d] = %d, want %d", i, x.Opposing[i], wantOpposing[i])
		}
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_Opposing_CoSortsWithIncoming -v -count=1`
Expected: FAIL. The current co-sort handles `Incoming` and `IncomingControl` but not `Opposing`.

- [ ] **Step 3: Update `sortIncomingByPriority`**

In `internal/netbuild/priority.go`, find the `for i := range intersections` loop body. Currently (after Task 4 of Phase 1):

```go
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
```

Replace with:

```go
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
		// Build inverse permutation: oldToNew[oldI] = newI.
		oldToNew := make([]int, len(idx))
		for newI, oldI := range idx {
			oldToNew[oldI] = newI
		}
		newInc := make([]network.EdgeID, len(x.Incoming))
		newCtrl := make([]network.Control, len(x.IncomingControl))
		newOpp := make([]int8, len(x.Opposing))
		for newI, oldI := range idx {
			newInc[newI] = x.Incoming[oldI]
			if oldI < len(x.IncomingControl) {
				newCtrl[newI] = x.IncomingControl[oldI]
			}
			if oldI < len(x.Opposing) {
				oldVal := x.Opposing[oldI]
				if oldVal < 0 {
					newOpp[newI] = -1
				} else {
					newOpp[newI] = int8(oldToNew[int(oldVal)])
				}
			} else {
				newOpp[newI] = -1
			}
		}
		x.Incoming = newInc
		x.IncomingControl = newCtrl
		x.Opposing = newOpp
```

- [ ] **Step 4: Run the new test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_Opposing_CoSortsWithIncoming -v -count=1`
Expected: PASS.

- [ ] **Step 5: Run the full repo suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: PASS.

- [ ] **Step 6: Commit**

```
git add internal/netbuild/priority.go internal/netbuild/control_test.go
git commit -m "feat(netbuild): co-sort Opposing with Incoming and remap indices"
```

---

## Task 3: Add `resolveOpposing` and wire into `Build`

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/netbuild.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

- [ ] **Step 1: Write failing test**

Append to `internal/netbuild/control_test.go`:

```go
// TestNetbuild_Opposing_FourWay: a 4-way + intersection. The N and S
// approaches pair; the E and W approaches pair.
func TestNetbuild_Opposing_FourWay(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0010, -74.0005), // N origin
		2: mkNode(2, 40.0000, -74.0010), // W origin
		3: mkNode(3, 40.0000, -74.0005), // center
		4: mkNode(4, 40.0000, -74.0000), // E origin (approaches center from east)
		5: mkNode(5, 39.9990, -74.0005), // S origin
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 3, 5), // N-S road
		mkWay(20, "residential", false, 2, 3, 4), // W-E road
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	if len(x.Opposing) != len(x.Incoming) {
		t.Fatalf("Opposing length %d != Incoming length %d", len(x.Opposing), len(x.Incoming))
	}
	// Every approach must have an opposing approach in a 4-way.
	for i := range x.Incoming {
		if x.Opposing[i] < 0 {
			t.Errorf("approach %d (edge %d) has no opposing", i, x.Incoming[i])
		}
	}
	// Symmetry.
	for i := range x.Incoming {
		j := int(x.Opposing[i])
		if j < 0 {
			continue
		}
		if int(x.Opposing[j]) != i {
			t.Errorf("non-symmetric: Opposing[%d]=%d but Opposing[%d]=%d", i, j, j, x.Opposing[j])
		}
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_Opposing_FourWay -v -count=1`
Expected: FAIL. All Opposing entries are still `-1`.

- [ ] **Step 3: Add `resolveOpposing` and `angleDiff`**

In `internal/netbuild/control.go`, add these at the bottom of the file:

```go
// resolveOpposing populates x.Opposing for each intersection. Two
// approaches are opposing iff:
//
//  1. Their arrival headings fold to the same axis bucket (same
//     8-bucket / 22.5° resolution as DefaultSignalConfig in sim).
//  2. AND their arrival headings are > π/2 apart (excludes
//     same-direction misalignment at Y-junctions and skewed forks).
//
// If a bucket has more than two members (degenerate star geometry),
// each approach pairs with whichever bucket-mate has the largest
// |Δheading|, i.e. the one most nearly opposite.
//
// Receives the edges slice via the *network.Network argument so it can
// call network.ArrivalHeading without needing access to the full Network
// (called from Build before the Network struct is fully assembled, but
// after edges and intersections are in their final form).
func resolveOpposing(xs []network.Intersection, net *network.Network) {
	const numBuckets = 8
	for i := range xs {
		x := &xs[i]
		if len(x.Opposing) != len(x.Incoming) {
			x.Opposing = make([]int8, len(x.Incoming))
			for k := range x.Opposing {
				x.Opposing[k] = -1
			}
		} else {
			for k := range x.Opposing {
				x.Opposing[k] = -1
			}
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

// angleDiff returns the signed angle (radians, (-π, π]) from a to b.
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

You'll need to add `"math"` to the imports if not already present.

- [ ] **Step 4: Wire `resolveOpposing` into `Build`**

In `internal/netbuild/netbuild.go`, find the call to `resolveControls` (added in Phase 1 Task 12). Currently:

```go
	resolveControls(intersections, feat, osmWayOfEdge, osmNodeOf)
```

Add the `resolveOpposing` call just after, but before `keepLargestComponent` is called (if `resolveControls` is before it) or before the network struct is built:

```go
	resolveControls(intersections, feat, osmWayOfEdge, osmNodeOf)

	// Resolve opposing approaches for left-turn yield logic. Needs
	// edge geometry; build a partial *Network containing just edges.
	partialNet := &network.Network{Edges: edges}
	resolveOpposing(intersections, partialNet)
```

If `keepLargestComponent` runs between `resolveControls` and the final return, place `resolveOpposing` AFTER `keepLargestComponent` (which may reorder intersections and edges). To verify, search for the existing flow in netbuild.go and place the call so it runs once on the final intersection slice.

- [ ] **Step 5: Run the new test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run TestNetbuild_Opposing_FourWay -v -count=1`
Expected: PASS.

- [ ] **Step 6: Full repo suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: PASS.

- [ ] **Step 7: Commit**

```
git add internal/netbuild/control.go internal/netbuild/netbuild.go internal/netbuild/control_test.go
git commit -m "feat(netbuild): add resolveOpposing for left-turn yield logic"
```

---

## Task 4: T-intersection and symmetry tests for `Opposing`

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/netbuild/control_test.go`

- [ ] **Step 1: Append the two tests**

```go
// TestNetbuild_Opposing_TThrough: T-intersection. The two through-road
// approaches pair with each other; the stem approach gets -1.
func TestNetbuild_Opposing_TThrough(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W origin (through)
		2: mkNode(2, 40.0000, -74.0005), // center
		3: mkNode(3, 40.0000, -74.0000), // E origin (through)
		4: mkNode(4, 39.9990, -74.0005), // S origin (stem)
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 2, 3), // W-E through
		mkWay(20, "residential", false, 4, 2),    // S stem
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	// Three approaches: two through (W-arrival and E-arrival) and one
	// stem (N-arrival from south, i.e., heading north into the
	// intersection). The two through ones must pair; the stem must
	// have Opposing = -1.
	pairedCount := 0
	stemCount := 0
	for i := range x.Incoming {
		if x.Opposing[i] >= 0 {
			pairedCount++
		} else {
			stemCount++
		}
	}
	if pairedCount != 2 {
		t.Errorf("want 2 paired approaches, got %d", pairedCount)
	}
	if stemCount != 1 {
		t.Errorf("want 1 unpaired stem approach, got %d", stemCount)
	}
}

// TestNetbuild_Opposing_Symmetric: across a mixed-geometry network,
// Opposing[Opposing[i]] == i whenever Opposing[i] != -1.
func TestNetbuild_Opposing_Symmetric(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		// Two adjacent 4-ways sharing a node.
		1: mkNode(1, 40.0020, -74.0005),
		2: mkNode(2, 40.0010, -74.0010),
		3: mkNode(3, 40.0010, -74.0005), // first center
		4: mkNode(4, 40.0010, -74.0000),
		5: mkNode(5, 40.0000, -74.0005), // second center
		6: mkNode(6, 40.0000, -74.0010),
		7: mkNode(7, 40.0000, -74.0000),
		8: mkNode(8, 39.9990, -74.0005),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 3, 5, 8), // N-S spine
		mkWay(20, "residential", false, 2, 3, 4),    // first E-W
		mkWay(30, "residential", false, 6, 5, 7),    // second E-W
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	for xi, x := range net.Intersections {
		for i := range x.Incoming {
			j := int(x.Opposing[i])
			if j < 0 {
				continue
			}
			back := int(x.Opposing[j])
			if back != i {
				t.Errorf("intersection %d: Opposing[%d]=%d but Opposing[%d]=%d (not symmetric)", xi, i, j, j, back)
			}
		}
	}
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/netbuild/ -run "TestNetbuild_Opposing_TThrough|TestNetbuild_Opposing_Symmetric" -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/netbuild/control_test.go
git commit -m "test(netbuild): T-intersection and symmetry tests for Opposing"
```

---

## Task 5: Add `leftTurnGapSec` constant

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`

- [ ] **Step 1: Add the constant**

In `internal/sim/world.go`, in the existing const block (near `gapThresholdSec`, `stopDwellSec`, etc.), add:

```go
	// leftTurnGapSec is the minimum oncoming-traffic ETA a left turner
	// accepts before crossing. Larger than gapThresholdSec because the
	// left-turn maneuver takes longer to execute. Literature: 6–8s.
	leftTurnGapSec = 6.0
```

Place it immediately after `gapThresholdSec` if that's a standalone constant, or inside the existing const block. The existing block style (after Phase 1) is:

```go
const gapThresholdSec = 3.0

const (
	stuckSpeedThresh = 0.1
	stuckTimeoutSec = 60.0
	stopDwellSec = 0.5
	stopLineTolMeters = 8.0
)
```

After:

```go
const gapThresholdSec = 3.0

const leftTurnGapSec = 6.0

const (
	stuckSpeedThresh = 0.1
	stuckTimeoutSec = 60.0
	stopDwellSec = 0.5
	stopLineTolMeters = 8.0
)
```

- [ ] **Step 2: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0.

- [ ] **Step 3: Commit**

```
git add internal/sim/world.go
git commit -m "feat(sim): add leftTurnGapSec constant"
```

---

## Task 6: Add `entitledToProceed` helper

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`

- [ ] **Step 1: Add the helper**

In `internal/sim/world.go`, add this function just below `allWayStopFIFO` (or at the end of the existing helper cluster):

```go
// entitledToProceed reports whether v would otherwise proceed through
// the intersection at the end of its current edge — i.e., neither
// stopDistanceForRed nor stopDistanceForYield say to stop. Used by
// leftTurnYieldsToOpposing to layer the left-turn check on top of
// Phase 1's yield rules without double-stopping.
//
// stopDistanceForYield has an idempotent side effect (maybeMarkStopped);
// calling it twice per tick is safe — the second call is a no-op.
func (w *World) entitledToProceed(v *Vehicle, byEdge map[network.EdgeID][]int) bool {
	if _, isRed := w.stopDistanceForRed(v); isRed {
		return false
	}
	if _, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
		return false
	}
	return true
}
```

- [ ] **Step 2: Build**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go build ./...`
Expected: exit 0. (Unused function — that's fine; Task 7 wires it.)

- [ ] **Step 3: Run sim tests to confirm no regression**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: PASS.

- [ ] **Step 4: Commit**

```
git add internal/sim/world.go
git commit -m "feat(sim): add entitledToProceed helper"
```

---

## Task 7: Add `leftTurnYieldsToOpposing` + wire into `Step` + first behavioral test

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

This is the core behavioral task. We write a failing test, implement the helper, wire it into Step, and verify.

- [ ] **Step 1: Write the failing behavioral test**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing: a priority-road
// vehicle turning left across opposing through-traffic must yield until
// the gap clears.
func TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing(t *testing.T) {
	// 4-way: N-S priority road, with W-E side road (unsignalized).
	// Vehicle A: north approach, turning left (heading west out).
	// Vehicle B: south approach (opposing A), going straight (heading north out).
	// Both approaches are ControlNone (priority road).
	// Expect A to yield (mustYield via leftTurnYieldsToOpposing).
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},   // N origin (A starts here)
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},  // S origin (B starts here)
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},     // center
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},  // W destination
		{ID: 4, Pos: network.Point{X: 0, Y: -200}},  // S destination (A's exit not used; just need outbound edge)
		{ID: 5, Pos: network.Point{X: 0, Y: 200}},   // N destination (B's exit)
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
		mkEdge(0, 0, 2), // N->C   (A's approach)
		mkEdge(1, 1, 2), // S->C   (B's approach, opposing A)
		mkEdge(2, 2, 3), // C->W   (A turns left to here)
		mkEdge(3, 2, 5), // C->N   (B continues straight to here)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Opposing:        []int8{1, 0}, // N opposes S
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// A close to the line, slow. B close to the line, slow (perpetual
	// short-ETA cross-traffic).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 80, V: 5}, // A: N->C->W (left turn)
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 95, V: 0.5}, // B: S->C->N (straight, pinned close)
	}
	w.nextID = 3

	// Pin B at S=95, V=0.5 each tick so it perpetually shows imminent
	// ETA = 5/0.5 = 10s? Actually d = 100 - 95 = 5m, V = 0.5, ETA = 10s.
	// That's > leftTurnGapSec (6s), so A would proceed. Adjust: make
	// B closer or faster to keep ETA inside 6s.
	// d/ovV < 6  =>  d < 6 * ovV.  With ovV=0.5, d < 3. Use S=98, V=0.5
	// → d=2, ETA=4s. Inside threshold.
	w.Vehicles[1].S = 98
	w.Vehicles[1].V = 0.5

	for i := 0; i < 300; i++ {
		// Re-pin B.
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5
			}
		}
		w.Step()
	}

	// A should still be on edge 0 (its approach), not despawned, not stuck.
	var a *Vehicle
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 1 {
			a = &w.Vehicles[j]
		}
	}
	if a == nil || a.Despawned {
		t.Fatal("left-turning vehicle should not be despawned during legitimate yield")
	}
	if a.Edge != 0 {
		t.Errorf("left-turning vehicle should still be on approach edge 0 (yielding), got edge %d", a.Edge)
	}
	if a.StuckTime != 0 {
		t.Errorf("yielding vehicle's StuckTime must be 0, got %.3f", a.StuckTime)
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing -v -count=1`
Expected: FAIL. Without the left-turn check, vehicle A proceeds through the intersection (since it's on ControlNone) and either despawns or reaches edge 2. The test will fail at the `a.Edge != 0` assertion.

- [ ] **Step 3: Implement `leftTurnYieldsToOpposing`**

In `internal/sim/world.go`, add this function near the other yield helpers (e.g., after `allWayStopFIFO`):

```go
// leftTurnYieldsToOpposing returns (distance to stop line, true) when v
// is making a left turn and an opposing-approach vehicle has imminent
// ETA. Layered on top of Phase 1's yield rules — only engages when v
// would otherwise proceed (priority road, green signal, or AllWayStop
// FIFO winner). Two opposing left-turners pass simultaneously: the
// inner gap loop skips opposing vehicles that are also turning left.
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
		// Skip opposing left-turners — they're yielding to us, so we
		// don't yield to them (mutual-yield deadlock resolution).
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
```

- [ ] **Step 4: Wire into `Step`**

In `internal/sim/world.go`, find the `Step` function's per-vehicle stepping pass (around line 447, after Phase 1). Currently:

```go
		// Apply red-light virtual leader if closer.
		if d, isRed := w.stopDistanceForRed(v); isRed {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		// Apply unsignalized-yield virtual leader if closer.
		if d, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
```

Add the new check right after:

```go
		// Apply red-light virtual leader if closer.
		if d, isRed := w.stopDistanceForRed(v); isRed {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		// Apply unsignalized-yield virtual leader if closer.
		if d, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		// Apply left-turn opposing-traffic virtual leader if closer.
		if d, mustYield := w.leftTurnYieldsToOpposing(v, byEdge); mustYield {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
```

- [ ] **Step 5: Run the test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing -v -count=1`
Expected: PASS.

- [ ] **Step 6: Full sim suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: PASS.

- [ ] **Step 7: Commit**

```
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): left-turn yields to opposing through/right traffic"
```

---

## Task 8: Extend stuck-vehicle guard to consult left-turn yield

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world.go`
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

A vehicle legitimately waiting on a left-turn yield must not be despawned by the 60s stuck-vehicle guard.

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/world_test.go`:

```go
// TestWorld_LeftTurn_StuckGuardBypassed: a left turner waiting on
// perpetual opposing traffic must not be despawned by the 60s
// stuck-vehicle guard.
func TestWorld_LeftTurn_StuckGuardBypassed(t *testing.T) {
	// Same fixture as TestWorld_LeftTurn_PriorityRoad_YieldsToOpposing
	// but run for >stuckTimeoutSec sim-seconds.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: -200}},
		{ID: 5, Pos: network.Point{X: 0, Y: 200}},
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
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
		mkEdge(3, 2, 5),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5},
	}
	w.nextID = 3

	// Run for 130 sim-seconds (well past stuckTimeoutSec=60).
	for i := 0; i < 2600; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5
			}
		}
		w.Step()
	}

	var a *Vehicle
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 1 {
			a = &w.Vehicles[j]
		}
	}
	if a == nil || a.Despawned {
		t.Fatal("left turner waiting on perpetual opposing traffic must not be despawned")
	}
	if a.StuckTime != 0 {
		t.Errorf("StuckTime should stay 0 during legitimate left-turn yield, got %.3f", a.StuckTime)
	}
}
```

- [ ] **Step 2: Run it — expect failure**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_LeftTurn_StuckGuardBypassed -v -count=1`
Expected: FAIL. The current stuck guard doesn't know about left-turn yield; vehicle A's `StuckTime` accumulates past 60s → despawned.

- [ ] **Step 3: Extend the stuck-vehicle guard**

In `internal/sim/world.go`, find the stuck-vehicle check (around line 461 after Phase 1). Currently:

```go
		if !v.Despawned && v.V < stuckSpeedThresh {
			_, isRed := w.stopDistanceForRed(v)
			_, mustYield := w.stopDistanceForYield(v, byEdge)
			if !isRed && !mustYield {
				v.StuckTime += w.dt
				if v.StuckTime > stuckTimeoutSec {
					slog.Warn("stuck vehicle despawned",
```

Replace the condition logic with:

```go
		if !v.Despawned && v.V < stuckSpeedThresh {
			_, isRed := w.stopDistanceForRed(v)
			_, mustYield := w.stopDistanceForYield(v, byEdge)
			_, mustLeftYield := w.leftTurnYieldsToOpposing(v, byEdge)
			if !isRed && !mustYield && !mustLeftYield {
				v.StuckTime += w.dt
				if v.StuckTime > stuckTimeoutSec {
					slog.Warn("stuck vehicle despawned",
```

(Only the `mustLeftYield` line and the `!mustLeftYield` clause are added; the surrounding code is unchanged.)

- [ ] **Step 4: Run the new test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_LeftTurn_StuckGuardBypassed -v -count=1`
Expected: PASS.

- [ ] **Step 5: Full sim suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: PASS.

- [ ] **Step 6: Commit**

```
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): exempt left-turn yielders from stuck-vehicle guard"
```

---

## Task 9: No-opposing-traffic and mutual-lefts tests

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

- [ ] **Step 1: Append both tests**

```go
// TestWorld_LeftTurn_PriorityRoad_NoOpposingTraffic: a priority-road
// left turner with no opposing vehicle sails through without recording
// a stop and without StuckTime accumulation.
func TestWorld_LeftTurn_PriorityRoad_NoOpposingTraffic(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: -200}},
		{ID: 5, Pos: network.Point{X: 0, Y: 200}},
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
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
		mkEdge(3, 2, 5),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 50, V: 8}, // A: N->C->W
	}
	w.nextID = 2

	for i := 0; i < 200; i++ {
		w.Step()
		if len(w.Vehicles) == 0 || w.Vehicles[0].Despawned {
			break
		}
	}

	// A must have made it to edge 2 (the outbound left-turn edge) or
	// despawned legitimately.
	if len(w.Vehicles) > 0 && !w.Vehicles[0].Despawned && w.Vehicles[0].Edge != 2 {
		t.Errorf("left turner with no opposing traffic should reach edge 2, got edge %d", w.Vehicles[0].Edge)
	}
	if len(w.Vehicles) > 0 && w.Vehicles[0].StuckTime != 0 {
		t.Errorf("StuckTime must remain 0, got %.3f", w.Vehicles[0].StuckTime)
	}
}

// TestWorld_LeftTurn_MutualLeftsPass: two opposing left-turners do not
// yield to each other (left-to-left pass). Both proceed.
func TestWorld_LeftTurn_MutualLeftsPass(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},   // A's exit (left from N)
		{ID: 4, Pos: network.Point{X: 100, Y: 0}},    // B's exit (left from S)
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
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3), // C->W (A's left turn)
		mkEdge(3, 2, 4), // C->E (B's left turn)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 50, V: 8}, // A: N->W (left)
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 50, V: 8}, // B: S->E (left)
	}
	w.nextID = 3

	for i := 0; i < 300; i++ {
		w.Step()
	}

	// Both must have reached their respective outbound edges or despawned.
	aOK, bOK := false, false
	for j := range w.Vehicles {
		v := &w.Vehicles[j]
		if v.ID == 1 && (v.Despawned || v.Edge == 2) {
			aOK = true
		}
		if v.ID == 2 && (v.Despawned || v.Edge == 3) {
			bOK = true
		}
	}
	if !aOK {
		t.Error("Vehicle A (left turner) should have made it through; opposing left should not block")
	}
	if !bOK {
		t.Error("Vehicle B (left turner) should have made it through; opposing left should not block")
	}
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run "TestWorld_LeftTurn_PriorityRoad_NoOpposingTraffic|TestWorld_LeftTurn_MutualLeftsPass" -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): left-turn no-opposing and mutual-lefts cases"
```

---

## Task 10: Signaled (green + red) left-turn tests

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

- [ ] **Step 1: Append both tests**

```go
// TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing: at a signaled
// intersection in normal green, a left turner with opposing through
// traffic must yield (permissive-left semantics).
func TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},
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
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3), // C->W (A's left turn)
		mkEdge(3, 2, 4), // C->N (B's through)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: allNone(2), // signaled — not consulted in Normal
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       true,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Single-phase signal: both N and S approaches always green.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: []int{0, 1}, GreenDur: 1000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 80, V: 5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5},
	}
	w.nextID = 3

	for i := 0; i < 300; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5
			}
		}
		w.Step()
	}

	var a *Vehicle
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 1 {
			a = &w.Vehicles[j]
		}
	}
	if a == nil || a.Despawned {
		t.Fatal("left turner should not be despawned during legitimate yield")
	}
	if a.Edge != 0 {
		t.Errorf("permissive-left turner should still be on approach edge 0, got edge %d", a.Edge)
	}
	if a.StuckTime != 0 {
		t.Errorf("StuckTime must be 0 for legitimate yielder, got %.3f", a.StuckTime)
	}
}

// TestWorld_LeftTurn_SignaledRed_NotAffected: at a signaled intersection
// where the left turner's approach is red, the existing hard-stop owns
// the decision; the left-turn check must not double-stop (and the
// vehicle's stuck-guard must not accumulate StuckTime).
func TestWorld_LeftTurn_SignaledRed_NotAffected(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},
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
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
		mkEdge(3, 2, 4),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: allNone(2),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       true,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Force the signal to all-red.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: nil, GreenDur: 1000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 50, V: 10}, // left turner, red approach
	}
	w.nextID = 2

	for i := 0; i < 500; i++ {
		w.Step()
	}

	if len(w.Vehicles) != 1 {
		t.Fatalf("vehicle should be stopped at red, not despawned, got %d alive", len(w.Vehicles))
	}
	v := &w.Vehicles[0]
	if v.V > 0.1 {
		t.Errorf("vehicle should be stopped at red, V=%.2f", v.V)
	}
	if v.StuckTime != 0 {
		t.Errorf("vehicle legitimately stopped at red; StuckTime must be 0, got %.3f", v.StuckTime)
	}
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run "TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing|TestWorld_LeftTurn_SignaledRed_NotAffected" -v -count=1`
Expected: PASS.

- [ ] **Step 3: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): signaled green + red left-turn cases"
```

---

## Task 11: AllWayStop left-turn tests

**Files:**
- Modify: `C:/Users/lab17/tmp/traffic-sim/internal/sim/world_test.go`

- [ ] **Step 1: Append both tests**

```go
// TestWorld_LeftTurn_AllWayStop_YieldsToOpposing: at an AllWayStop with
// two opposing approaches, the left turner (after dwell + FIFO clears)
// must yield to the opposing through.
func TestWorld_LeftTurn_AllWayStop_YieldsToOpposing(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},
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
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3), // C->W (A's left turn)
		mkEdge(3, 2, 4), // C->N (B's through)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlAllWayStop, network.ControlAllWayStop),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 80, V: 5},   // A: left
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5}, // B: through, pinned imminent
	}
	w.nextID = 3

	for i := 0; i < 500; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5
			}
		}
		w.Step()
	}

	var a *Vehicle
	for j := range w.Vehicles {
		if w.Vehicles[j].ID == 1 {
			a = &w.Vehicles[j]
		}
	}
	if a == nil || a.Despawned {
		t.Fatal("AllWayStop left turner should not be despawned during legitimate yield")
	}
	if a.Edge != 0 {
		t.Errorf("AllWayStop left turner should still be on approach edge 0, got edge %d", a.Edge)
	}
	if a.StuckTime != 0 {
		t.Errorf("StuckTime must be 0 for legitimate yielder, got %.3f", a.StuckTime)
	}
}

// TestWorld_LeftTurn_AllWayStop_BothLeftsPass: at an AllWayStop, two
// opposing left turners both proceed simultaneously after dwell.
func TestWorld_LeftTurn_AllWayStop_BothLeftsPass(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}}, // A's exit (left from N)
		{ID: 4, Pos: network.Point{X: 100, Y: 0}},  // B's exit (left from S)
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
		mkEdge(0, 0, 2),
		mkEdge(1, 1, 2),
		mkEdge(2, 2, 3),
		mkEdge(3, 2, 4),
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlAllWayStop, network.ControlAllWayStop),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 80, V: 5}, // A: left
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 80, V: 5}, // B: left
	}
	w.nextID = 3

	for i := 0; i < 600; i++ {
		w.Step()
	}

	aOK, bOK := false, false
	for j := range w.Vehicles {
		v := &w.Vehicles[j]
		if v.ID == 1 && (v.Despawned || v.Edge == 2) {
			aOK = true
		}
		if v.ID == 2 && (v.Despawned || v.Edge == 3) {
			bOK = true
		}
	}
	if !aOK {
		t.Error("AllWayStop left turner A should have made it through (mutual lefts pass)")
	}
	if !bOK {
		t.Error("AllWayStop left turner B should have made it through (mutual lefts pass)")
	}
}
```

- [ ] **Step 2: Run them**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run "TestWorld_LeftTurn_AllWayStop_YieldsToOpposing|TestWorld_LeftTurn_AllWayStop_BothLeftsPass" -v -count=1`
Expected: PASS.

- [ ] **Step 3: Full sim suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/`
Expected: PASS.

- [ ] **Step 4: Commit**

```
git add internal/sim/world_test.go
git commit -m "test(sim): AllWayStop left-turn yield and mutual-lefts cases"
```

---

## Task 12: Determinism and benchmark verification

**Files:** none modified (verification only).

- [ ] **Step 1: Run the determinism test**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -run TestWorld_TraceDeterminism -v -count=1`
Expected: PASS. The new code introduces no randomness.

If it fails, the most likely cause is non-deterministic iteration in `leftTurnYieldsToOpposing` — but the function only iterates `byEdge[oppEdgeID]` (a slice, deterministic order). Investigate that loop if needed.

- [ ] **Step 2: Run sim benchmarks**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./internal/sim/ -bench=BenchmarkTick -benchtime=2s -run=^$`
Expected: ns/op within ~15% of the post-Phase-1 baseline (1k≈0.45ms, 5k≈2.09ms, 10k≈3.24ms). The new check adds an O(1) short-circuit for most vehicles (non-left-turners exit at `ClassifyTurn != TurnLeft`) and an O(opposing-queue) scan for left turners.

If significantly slower (>20%), profile with `-cpuprofile=cpu.out` and check whether `entitledToProceed`'s double-call to `stopDistanceForYield` is the hot spot. The spec notes this optimization deferral.

- [ ] **Step 3: Run full repo suite**

Run: `cd C:/Users/lab17/tmp/traffic-sim && go test ./...`
Expected: all PASS.

- [ ] **Step 4: No commit (verification only)**

Phase 4 is complete. Summarize in the user-facing message: opposing-approach map + left-turn yield rule + mutual-left pass + stuck-guard exemption all in place, covering the three call contexts (unsignalized priority road, signaled normal green, AllWayStop FIFO).

---

## Out of scope (deferred to later phases)

- Right-turn refinements (right-on-red yields to crossing-from-the-right).
- Per-vehicle critical-gap distributions (Phase 3, planned next).
- Impatience: gap shrinks with wait time (Phase 3).
- Multi-lane left-turn pockets (dedicated left-turn lanes).
- "Carrier left" — left turner waits in the intersection, clears on yellow.
- Pedestrian conflicts.
