# Phase 4 — IDM Car-Following

**Milestone:** Vehicles use the Intelligent Driver Model (IDM) for longitudinal motion. Leader-finding uses per-edge sorted vehicle lists. A two-vehicle test on one edge produces visible following behavior.

---

### Task 4.1: IDM acceleration function

**Files:**
- Create: `internal/sim/idm.go`
- Create: `internal/sim/idm_test.go`

- [ ] **Step 1: Write the failing test**

Write `internal/sim/idm_test.go`:
```go
package sim

import (
	"math"
	"testing"
)

func TestIDM_FreeFlowAccelerates(t *testing.T) {
	p := DefaultIDM()
	// Alone on road, well below desired speed.
	a := IDMAcceleration(p, 5.0 /*v*/, 30.0 /*v0*/, math.Inf(1), 0)
	if a <= 0 {
		t.Errorf("free-flow well below v0 should accelerate, got a=%.2f", a)
	}
}

func TestIDM_AtDesiredSpeedNoAccel(t *testing.T) {
	p := DefaultIDM()
	a := IDMAcceleration(p, 30.0, 30.0, math.Inf(1), 0)
	if math.Abs(a) > 0.05 {
		t.Errorf("at desired speed with no leader: want a~=0, got %.3f", a)
	}
}

func TestIDM_BrakesForClosingLeader(t *testing.T) {
	p := DefaultIDM()
	// Approaching a slower (or stopped) leader at small gap.
	a := IDMAcceleration(p, 20.0, 25.0, 5.0 /*gap*/, 15.0 /*deltaV = ego - leader*/)
	if a >= 0 {
		t.Errorf("closing on slow leader at 5m gap should brake hard, got a=%.2f", a)
	}
}

func TestIDM_ZeroGapClampSafe(t *testing.T) {
	p := DefaultIDM()
	// Bumper-to-bumper, both stopped: should brake hard (not blow up).
	a := IDMAcceleration(p, 0.0, 25.0, 0.0, 0.0)
	if math.IsNaN(a) || math.IsInf(a, 0) {
		t.Errorf("zero gap must not produce NaN/Inf, got a=%v", a)
	}
}
```

- [ ] **Step 2: Run to verify failure**

Run: `go test ./internal/sim/ -run TestIDM -v`
Expected: FAIL — `DefaultIDM`, `IDMAcceleration` undefined.

- [ ] **Step 3: Implement IDM**

Write `internal/sim/idm.go`:
```go
package sim

import "math"

// IDMParams parameterize the Intelligent Driver Model (Treiber et al, 2000).
type IDMParams struct {
	A     float64 // max acceleration (m/s^2), typical 1.0
	B     float64 // comfortable deceleration (m/s^2), typical 1.5
	S0    float64 // minimum stopping gap (m), typical 2.0
	T     float64 // safe time headway (s), typical 1.5
	Delta float64 // free-flow acceleration exponent, typical 4
}

func DefaultIDM() IDMParams {
	return IDMParams{A: 1.0, B: 1.5, S0: 2.0, T: 1.5, Delta: 4}
}

// IDMAcceleration returns the acceleration in m/s^2.
//
//   v       current speed (m/s)
//   v0      desired speed (m/s) — typically the edge speed limit
//   gap     bumper-to-bumper distance to leader (m); pass math.Inf(1) if none
//   deltaV  v - vLeader (positive = closing)
//
// The result may be negative (braking) and is mathematically defined for
// all non-negative gaps; the caller is responsible for clamping the
// resulting speed to >= 0 after integration.
func IDMAcceleration(p IDMParams, v, v0, gap, deltaV float64) float64 {
	if v0 <= 0 {
		v0 = 0.1
	}
	freeTerm := 1.0 - math.Pow(v/v0, p.Delta)

	if math.IsInf(gap, 1) {
		return p.A * freeTerm
	}
	// Desired dynamic gap.
	sStar := p.S0 + math.Max(0, v*p.T+(v*deltaV)/(2*math.Sqrt(p.A*p.B)))
	if gap < 0.01 {
		gap = 0.01
	}
	intTerm := (sStar / gap) * (sStar / gap)
	return p.A * (freeTerm - intTerm)
}
```

- [ ] **Step 4: Run tests**

Run: `go test ./internal/sim/ -run TestIDM -v`
Expected: all four PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/idm.go internal/sim/idm_test.go
git commit -m "feat(sim): IDM acceleration model with unit tests"
```

---

### Task 4.2: Leader-finding via per-edge sorted index

**Files:**
- Modify: `internal/sim/world.go` (add EdgeIndex maintenance)
- Modify: `internal/sim/vehicle.go` (use IDM step)

- [ ] **Step 1: Read current files**

Read `internal/sim/world.go` and `internal/sim/vehicle.go` to see current state.

- [ ] **Step 2: Replace vehicle.go**

Write `internal/sim/vehicle.go`:
```go
package sim

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

type VehicleID uint32

// VehicleLength is the bumper-to-bumper length used for gap calculation.
const VehicleLength = 5.0 // meters

type Vehicle struct {
	ID       VehicleID
	Route    []network.EdgeID
	RouteIdx int
	Edge     network.EdgeID
	Lane     uint8
	S        float64 // meters along edge, measured at front bumper
	V        float64 // m/s
	A        float64 // m/s^2 (last computed accel; useful for tracing)

	Despawned bool
}

// stepIDM advances vehicle i by one tick using IDM with the supplied leader.
// leader may be nil; if non-nil, both vehicles are assumed to be on the same
// edge (cross-edge leaders are handled by world.go via lookahead).
func stepIDM(v *Vehicle, leaderS float64, leaderV float64, hasLeader bool,
	net *network.Network, params IDMParams, dt float64,
) {
	if v.Despawned {
		return
	}
	edge := &net.Edges[v.Edge]
	v0 := edge.SpeedLimit

	gap := math.Inf(1)
	deltaV := 0.0
	if hasLeader {
		gap = leaderS - v.S - VehicleLength
		if gap < 0 {
			gap = 0
		}
		deltaV = v.V - leaderV
	}
	v.A = IDMAcceleration(params, v.V, v0, gap, deltaV)
	v.V += v.A * dt
	if v.V < 0 {
		v.V = 0
	}
	v.S += v.V * dt

	for v.S >= edge.Length {
		v.S -= edge.Length
		v.RouteIdx++
		if v.RouteIdx >= len(v.Route) {
			v.Despawned = true
			v.S = 0
			return
		}
		v.Edge = v.Route[v.RouteIdx]
		edge = &net.Edges[v.Edge]
	}
}
```

- [ ] **Step 3: Replace world.go's Step function**

In `internal/sim/world.go`, replace `func (w *World) Step()` with the IDM version. Final `Step` should look like:

```go
func (w *World) Step() {
	// 1. Demand.
	reqs := w.Spawner.Tick(w.SimTime, w.dt)
	for _, r := range reqs {
		w.trySpawn(r)
	}

	// 2. Bucket vehicles by edge for leader lookup, sorted by S ascending.
	byEdge := make(map[network.EdgeID][]int, 1024)
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		eid := w.Vehicles[i].Edge
		byEdge[eid] = append(byEdge[eid], i)
	}
	for _, idxs := range byEdge {
		sortVehicleIdxByS(w.Vehicles, idxs)
	}

	// 3. Step each vehicle, finding its leader as the next vehicle ahead
	//    on the same edge (or the first vehicle on the next route edge
	//    if no same-edge leader exists and gap to end-of-edge is small).
	for _, idxs := range byEdge {
		for pos, vi := range idxs {
			v := &w.Vehicles[vi]
			var lS, lV float64
			has := false
			if pos+1 < len(idxs) {
				ld := &w.Vehicles[idxs[pos+1]]
				lS, lV, has = ld.S, ld.V, true
			} else if v.RouteIdx+1 < len(v.Route) {
				// Lookahead to next edge's first vehicle.
				nextE := v.Route[v.RouteIdx+1]
				if nidxs, ok := byEdge[nextE]; ok && len(nidxs) > 0 {
					nv := &w.Vehicles[nidxs[0]]
					edge := &w.Net.Edges[v.Edge]
					lS = edge.Length + nv.S
					lV = nv.V
					has = true
				}
			}
			stepIDM(v, lS, lV, has, w.Net, DefaultIDM(), w.dt)
		}
	}

	// 4. Compact and advance time.
	w.compact()
	w.Tick++
	w.SimTime += w.dt
}

// sortVehicleIdxByS sorts idxs ascending by Vehicles[i].S (insertion sort;
// fine for small per-edge counts).
func sortVehicleIdxByS(vs []Vehicle, idxs []int) {
	for i := 1; i < len(idxs); i++ {
		for j := i; j > 0 && vs[idxs[j-1]].S > vs[idxs[j]].S; j-- {
			idxs[j-1], idxs[j] = idxs[j], idxs[j-1]
		}
	}
}
```

(Delete the now-unused `stepConstantVelocity` function from `vehicle.go` if you didn't already in Step 2.)

- [ ] **Step 4: Verify build and tests still pass**

Run: `go test ./internal/sim/`
Expected: PASS. The integration tests still work because they don't depend on the specific stepping algorithm.

- [ ] **Step 5: Add an IDM-specific integration test**

Append to `internal/sim/world_test.go`:
```go
func TestWorld_IDMFollowingMaintainsGap(t *testing.T) {
	net := buildLineGraph() // 3 edges, 100m each, 10 m/s
	// No spawner — we'll inject vehicles directly.
	w := NewWorld(net, NewRandomOD(net, 0, 0))

	// Two vehicles on edge 0, leader 50m ahead, both starting at speed.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 10, V: 5},  // follower
		{ID: 2, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 60, V: 5},  // leader
	}
	w.nextID = 3

	// Run 200 ticks (10 sim-seconds).
	for i := 0; i < 200; i++ {
		w.Step()
	}

	// Both should be alive (course is 300m, won't complete in 10s at ~10 m/s).
	if len(w.Vehicles) != 2 {
		t.Fatalf("want 2 vehicles alive, got %d", len(w.Vehicles))
	}

	// Find them by ID (compact may have reordered).
	var f, l *Vehicle
	for i := range w.Vehicles {
		switch w.Vehicles[i].ID {
		case 1:
			f = &w.Vehicles[i]
		case 2:
			l = &w.Vehicles[i]
		}
	}
	if f == nil || l == nil {
		t.Fatal("lost a vehicle")
	}

	// Compute the linear position of each (S + edge_offset).
	pos := func(v *Vehicle) float64 {
		return float64(v.RouteIdx)*100 + v.S
	}
	gap := pos(l) - pos(f) - VehicleLength
	if gap < VehicleLength {
		t.Errorf("follower closed gap to %.2f m (collision-ish)", gap)
	}
}
```

- [ ] **Step 6: Run tests**

Run: `go test ./internal/sim/ -v`
Expected: all PASS, including the new one.

- [ ] **Step 7: Commit**

```bash
git add internal/sim/vehicle.go internal/sim/world.go internal/sim/world_test.go
git commit -m "feat(sim): wire IDM into tick loop with same-edge + lookahead leader"
```

---

**Phase 4 done when:**
- `go test ./...` green, including `TestWorld_IDMFollowingMaintainsGap`.
- IDM accel function passes free-flow, at-speed, and braking tests.
- `trafficsim run --headless` still runs without crashing.
