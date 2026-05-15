# Phase 6 — Lane Changing

**Milestone:** Vehicles on multi-lane edges may switch lanes when the target lane offers a clearly better situation (faster leader, more headroom). A threshold-based test passes: a slow leader in lane 0 causes a fast follower to overtake into lane 1 within N ticks.

---

### Task 6.1: Lane-change state + decision rule

**Files:**
- Modify: `internal/sim/vehicle.go` (extend Vehicle)
- Create: `internal/sim/lanechange.go`
- Create: `internal/sim/lanechange_test.go`

**Rule (simple threshold MOBIL-lite):**
- Consider lane change when the same-lane leader is < `laneChangeCheckGap` ahead AND that leader is slower than ego desired speed by `vDiffThreshold`.
- Look at the immediate neighbor lane(s) for: a "rear gap" (distance back to nearest lane-mate vehicle behind ego's S) > `safetyGapRear` AND a "front gap" > `safetyGapFront`.
- If yes, switch lane instantly (no smooth animation in Phase 6 — interpolation is a Phase 7 renderer concern).

- [ ] **Step 1: Extend Vehicle**

Read `internal/sim/vehicle.go`, then modify the `Vehicle` struct to add a cooldown:
```go
type Vehicle struct {
	ID       VehicleID
	Route    []network.EdgeID
	RouteIdx int
	Edge     network.EdgeID
	Lane     uint8
	S        float64
	V        float64
	A        float64

	// LaneChangeCooldown counts down in seconds. Cannot change lanes
	// again until this reaches 0. Prevents oscillation.
	LaneChangeCooldown float64

	Despawned bool
}
```

- [ ] **Step 2: Write the failing test**

Write `internal/sim/lanechange_test.go`:
```go
package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestLaneChange_OvertakesSlowLeader(t *testing.T) {
	// One edge, 1000m, 2 lanes, speed limit 20 m/s.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 1000, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 1000, SpeedLimit: 20,
			Lanes: []network.Lane{{Index: 0}, {Index: 1}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	// Slow leader in lane 0; fast follower 30m behind, also lane 0.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 30, V: 20}, // follower
		{ID: 2, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 60, V: 5},  // slow leader
	}
	w.nextID = 3

	// Run 30 ticks (1.5s) — long enough to evaluate lane change.
	for i := 0; i < 30; i++ {
		w.Step()
	}

	var follower *Vehicle
	for i := range w.Vehicles {
		if w.Vehicles[i].ID == 1 {
			follower = &w.Vehicles[i]
		}
	}
	if follower == nil {
		t.Fatal("follower lost")
	}
	if follower.Lane != 1 {
		t.Errorf("follower should have changed to lane 1, still in lane %d", follower.Lane)
	}
}
```

- [ ] **Step 3: Run to verify failure**

Run: `go test ./internal/sim/ -run TestLaneChange -v`
Expected: FAIL.

- [ ] **Step 4: Implement lane-change decision**

Write `internal/sim/lanechange.go`:
```go
package sim

import "github.com/lab1702/traffic-sim/internal/network"

const (
	laneChangeCheckGap = 50.0 // m: only consider LC if leader within this
	vDiffThreshold     = 5.0  // m/s: leader must be this much slower
	safetyGapFront     = 20.0 // m
	safetyGapRear      = 10.0 // m
	laneChangeCooldown = 3.0  // s
)

// tryLaneChange mutates v.Lane if a beneficial change is available.
// laneVehicles[lane] is a sorted-by-S slice of vehicle indices on that
// lane of the current edge.
func tryLaneChange(v *Vehicle, vi int, laneVehicles map[uint8][]int, vs []Vehicle, net *network.Network) {
	if v.LaneChangeCooldown > 0 {
		return
	}
	edge := &net.Edges[v.Edge]
	numLanes := uint8(len(edge.Lanes))
	if numLanes < 2 {
		return
	}

	// Find same-lane leader.
	myLane := v.Lane
	same := laneVehicles[myLane]
	var myPos int = -1
	for i, idx := range same {
		if idx == vi {
			myPos = i
			break
		}
	}
	if myPos < 0 {
		return
	}
	var leaderV float64 = edge.SpeedLimit
	var leaderS float64 = edge.Length + 1e6 // effectively infinity
	if myPos+1 < len(same) {
		ld := &vs[same[myPos+1]]
		leaderV, leaderS = ld.V, ld.S
	}
	leaderGap := leaderS - v.S - VehicleLength
	if leaderGap > laneChangeCheckGap || edge.SpeedLimit-leaderV < vDiffThreshold {
		return // no reason to change
	}

	// Try each neighbor lane.
	for _, dl := range []int8{-1, 1} {
		nl := int(myLane) + int(dl)
		if nl < 0 || nl >= int(numLanes) {
			continue
		}
		other := laneVehicles[uint8(nl)]
		// Find the position where ego would sit and check front/rear gaps.
		frontS, hasFront := nextAheadS(other, vs, v.S)
		rearS, hasRear := nextBehindS(other, vs, v.S)
		if hasFront && frontS-v.S-VehicleLength < safetyGapFront {
			continue
		}
		if hasRear && v.S-rearS-VehicleLength < safetyGapRear {
			continue
		}
		// Commit the change.
		v.Lane = uint8(nl)
		v.LaneChangeCooldown = laneChangeCooldown
		return
	}
}

// nextAheadS returns the S of the closest vehicle on the lane with S > egoS.
func nextAheadS(idxs []int, vs []Vehicle, egoS float64) (float64, bool) {
	var best float64
	found := false
	for _, i := range idxs {
		s := vs[i].S
		if s <= egoS {
			continue
		}
		if !found || s < best {
			best, found = s, true
		}
	}
	return best, found
}

// nextBehindS returns the S of the closest vehicle on the lane with S < egoS.
func nextBehindS(idxs []int, vs []Vehicle, egoS float64) (float64, bool) {
	var best float64
	found := false
	for _, i := range idxs {
		s := vs[i].S
		if s >= egoS {
			continue
		}
		if !found || s > best {
			best, found = s, true
		}
	}
	return best, found
}
```

- [ ] **Step 5: Integrate into world.Step**

Read `internal/sim/world.go`, then modify the per-vehicle stepping loop:

After bucketing vehicles by edge, ALSO bucket by (edge, lane) so we have per-lane sorted lists. Replace the bucketing section with:
```go
byEdge := make(map[network.EdgeID][]int, 1024)
byEdgeLane := make(map[network.EdgeID]map[uint8][]int, 1024)
for i := range w.Vehicles {
	if w.Vehicles[i].Despawned {
		continue
	}
	eid := w.Vehicles[i].Edge
	byEdge[eid] = append(byEdge[eid], i)
	if _, ok := byEdgeLane[eid]; !ok {
		byEdgeLane[eid] = make(map[uint8][]int)
	}
	byEdgeLane[eid][w.Vehicles[i].Lane] = append(byEdgeLane[eid][w.Vehicles[i].Lane], i)
}
for _, idxs := range byEdge {
	sortVehicleIdxByS(w.Vehicles, idxs)
}
for _, lanes := range byEdgeLane {
	for _, idxs := range lanes {
		sortVehicleIdxByS(w.Vehicles, idxs)
	}
}
```

Then in the per-vehicle loop, also use the lane-specific bucket for leader-finding (replace the `same-edge leader` clause), AND consider lane change. The stepping loop should iterate vehicles in a stable order (e.g., by index) instead of by-edge clusters for simplicity. Replace the stepping loop with:
```go
// Pre-compute leaders per (edge, lane).
type leaderInfo struct {
	lS, lV float64
	has    bool
}
leaders := make(map[int]leaderInfo, len(w.Vehicles))
for eid, lanes := range byEdgeLane {
	edge := &w.Net.Edges[eid]
	for ln, idxs := range lanes {
		for pos, vi := range idxs {
			info := leaderInfo{}
			if pos+1 < len(idxs) {
				ld := &w.Vehicles[idxs[pos+1]]
				info.lS, info.lV, info.has = ld.S, ld.V, true
			} else {
				v := &w.Vehicles[vi]
				if v.RouteIdx+1 < len(v.Route) {
					nextE := v.Route[v.RouteIdx+1]
					if nlanes, ok := byEdgeLane[nextE]; ok {
						// Use first vehicle in the same lane index on the next edge if exists.
						if ne := &w.Net.Edges[nextE]; uint8(ln) < uint8(len(ne.Lanes)) {
							if nidxs, ok := nlanes[ln]; ok && len(nidxs) > 0 {
								nv := &w.Vehicles[nidxs[0]]
								info.lS = edge.Length + nv.S
								info.lV = nv.V
								info.has = true
							}
						}
					}
				}
			}
			leaders[vi] = info
		}
	}
}

// Apply signal/yield virtual leaders and step.
for i := range w.Vehicles {
	if w.Vehicles[i].Despawned {
		continue
	}
	v := &w.Vehicles[i]
	info := leaders[i]
	lS, lV, has := info.lS, info.lV, info.has

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

	stepIDM(v, lS, lV, has, w.Net, DefaultIDM(), w.dt)

	// Decrement cooldown.
	if v.LaneChangeCooldown > 0 {
		v.LaneChangeCooldown -= w.dt
		if v.LaneChangeCooldown < 0 {
			v.LaneChangeCooldown = 0
		}
	}
	// Try lane change AFTER stepping (so position is updated this tick;
	// the per-lane buckets we built are slightly stale, which is fine —
	// they're a tick-old snapshot used to decide change for next tick).
	if lanes, ok := byEdgeLane[v.Edge]; ok {
		tryLaneChange(v, i, lanes, w.Vehicles, w.Net)
	}
}
```

- [ ] **Step 6: Run tests**

Run: `go test ./internal/sim/ -v`
Expected: all PASS, including new lane-change test.

- [ ] **Step 7: Commit**

```bash
git add internal/sim/vehicle.go internal/sim/lanechange.go internal/sim/lanechange_test.go internal/sim/world.go
git commit -m "feat(sim): threshold-based lane change with cooldown"
```

---

**Phase 6 done when:**
- `go test ./...` is green.
- `TestLaneChange_OvertakesSlowLeader` passes.
