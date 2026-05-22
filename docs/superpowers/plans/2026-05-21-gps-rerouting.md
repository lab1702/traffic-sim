# GPS Rerouting Around Traffic Jams Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give every vehicle GPS-style behavior — observe live per-edge speeds and re-route around congestion to cut its own trip time.

**Architecture:** A new `Congestion` component tracks an EWMA-smoothed average speed per edge (refreshed each tick from the vehicles on it) and converts it to a routing cost (`length / observed_speed`). The router gains a cost-parameterized A* (`RouteCost`); the existing free-flow `Route` becomes a wrapper. On crossing into a new edge, past a cooldown, a GPS vehicle re-runs A* on live costs and switches only on a margin (hysteresis). A new `VehicleReroute` trace event keeps `tracereplay` faithful. Determinism and the 50 ms tick budget are preserved.

**Tech Stack:** Go 1.x, standard library (`container/heap`, `math`, `encoding/binary`); existing `internal/sim`, `internal/network`, `internal/trace` packages; Ebitengine viewer (unaffected).

**Spec:** `docs/superpowers/specs/2026-05-21-gps-rerouting-design.md`

---

## Task 1: Congestion observatory

**Files:**
- Create: `internal/sim/congestion.go`
- Test: `internal/sim/congestion_test.go`

- [ ] **Step 1: Write the failing tests**

Create `internal/sim/congestion_test.go`:

```go
package sim

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// congTestNet: two edges, 100m each, 10 and 20 m/s limits.
func congTestNet() *network.Network {
	return &network.Network{
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10},
			{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 20},
		},
	}
}

func TestCongestion_EmptyEdgeFreeFlow(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	// An edge nobody is on costs free-flow travel time.
	got := c.Cost(net, 0)
	want := 100.0 / 10.0
	if math.Abs(got-want) > 1e-9 {
		t.Fatalf("empty-edge cost = %v, want %v", got, want)
	}
	// And stays there after an update with no vehicles on it.
	c.Update(net, map[network.EdgeID][]int{}, nil)
	if got := c.Cost(net, 0); math.Abs(got-want) > 1e-9 {
		t.Fatalf("empty-edge cost after update = %v, want %v", got, want)
	}
}

func TestCongestion_JamRaisesCost(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	free := c.Cost(net, 0)
	vehicles := []Vehicle{{Edge: 0, V: 0}, {Edge: 0, V: 0}}
	byEdge := map[network.EdgeID][]int{0: {0, 1}}
	for i := 0; i < 2000; i++ {
		c.Update(net, byEdge, vehicles)
	}
	jammed := c.Cost(net, 0)
	if jammed <= free {
		t.Fatalf("jammed cost %v should exceed free-flow cost %v", jammed, free)
	}
	// Smoothed speed converges toward 0; Cost floors at length/minEdgeSpeed.
	want := 100.0 / minEdgeSpeed
	if math.Abs(jammed-want) > 1.0 {
		t.Fatalf("jammed cost %v, want ~%v", jammed, want)
	}
}

func TestCongestion_EWMASmoothing(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	vehicles := []Vehicle{{Edge: 0, V: 0}}
	byEdge := map[network.EdgeID][]int{0: {0}}
	before := c.speed[0]
	c.Update(net, byEdge, vehicles)
	after := c.speed[0]
	want := before + c.alpha*(0-before)
	if math.Abs(after-want) > 1e-9 {
		t.Fatalf("after one update speed = %v, want %v (alpha=%v)", after, want, c.alpha)
	}
	// A single-tick dip must not collapse the speed (smoothing, not snap).
	if after < before*0.5 {
		t.Fatalf("single-tick dip over-moved: %v from %v", after, before)
	}
}

func TestCongestion_CostClamps(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	// Floor: even at speed 0 the cost is finite, not +Inf.
	c.speed[0] = 0
	floored := c.Cost(net, 0)
	if math.IsInf(floored, 1) || math.Abs(floored-100.0/minEdgeSpeed) > 1e-9 {
		t.Fatalf("floor cost = %v, want %v", floored, 100.0/minEdgeSpeed)
	}
	// Ceiling: observed faster than free-flow is clamped to free-flow cost.
	c.speed[0] = 100 // way above the 10 m/s limit
	ceiled := c.Cost(net, 0)
	if math.Abs(ceiled-100.0/10.0) > 1e-9 {
		t.Fatalf("ceiling cost = %v, want %v (free-flow)", ceiled, 100.0/10.0)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./internal/sim/ -run TestCongestion -v`
Expected: FAIL — does not compile (`undefined: NewCongestion`, `undefined: ewmaHalfLifeSec`, `undefined: minEdgeSpeed`).

- [ ] **Step 3: Write the implementation**

Create `internal/sim/congestion.go`:

```go
package sim

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

const (
	// minEdgeSpeed floors the speed used for routing cost. A jammed edge is
	// expensive but never infinite-cost, so it stays selectable when it is the
	// only path through (dead ends, single bridges). 0.5 m/s ~ 1.8 km/h.
	minEdgeSpeed = 0.5

	// ewmaHalfLifeSec is the half-life of the per-edge speed EWMA: the time for
	// a step change in observed speed to be half-reflected in the smoothed
	// value. 10s rejects single-vehicle transients (one car braking) while
	// tracking real jams that form/clear over tens of seconds.
	ewmaHalfLifeSec = 10.0
)

// Congestion tracks an EWMA-smoothed average speed per edge, refreshed once per
// tick from the vehicles currently on each edge, and converts those speeds into
// routing costs (free-flow travel time, inflated by congestion).
type Congestion struct {
	speed []float64 // smoothed observed speed (m/s), indexed by EdgeID
	alpha float64   // EWMA blend factor in (0,1], derived from half-life and dt
}

// NewCongestion allocates a per-edge speed table seeded at each edge's
// free-flow speed. alpha is derived from the EWMA half-life and tick length:
// alpha = 1 - 2^(-dt/halfLife).
func NewCongestion(net *network.Network, halfLifeSec, dt float64) *Congestion {
	speed := make([]float64, len(net.Edges))
	for i := range net.Edges {
		speed[i] = net.Edges[i].SpeedLimit
	}
	alpha := 1.0
	if halfLifeSec > 0 {
		alpha = 1 - math.Exp(-math.Ln2*dt/halfLifeSec)
	}
	return &Congestion{speed: speed, alpha: alpha}
}

// Update blends each edge's smoothed speed toward this tick's observed mean
// speed (arithmetic mean of vehicle speeds on the edge — order-independent),
// or toward free-flow when the edge is empty. byEdge maps EdgeID to indices
// into vehicles.
func (c *Congestion) Update(net *network.Network, byEdge map[network.EdgeID][]int, vehicles []Vehicle) {
	for eid := range c.speed {
		var target float64
		idxs := byEdge[network.EdgeID(eid)]
		if len(idxs) > 0 {
			sum := 0.0
			for _, vi := range idxs {
				sum += vehicles[vi].V
			}
			target = sum / float64(len(idxs))
		} else {
			target = net.Edges[eid].SpeedLimit
		}
		c.speed[eid] += c.alpha * (target - c.speed[eid])
	}
}

// Cost returns the routing cost (travel time) for an edge: its length divided
// by the smoothed observed speed, clamped to [minEdgeSpeed, freeFlowSpeed].
// The free-flow ceiling keeps the router's A* heuristic admissible (congestion
// only makes edges slower than free-flow, never faster).
func (c *Congestion) Cost(net *network.Network, eid network.EdgeID) float64 {
	e := &net.Edges[eid]
	freeFlow := e.SpeedLimit
	if freeFlow < minEdgeSpeed {
		freeFlow = minEdgeSpeed
	}
	s := c.speed[eid]
	if s < minEdgeSpeed {
		s = minEdgeSpeed
	}
	if s > freeFlow {
		s = freeFlow
	}
	return e.Length / s
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `go test ./internal/sim/ -run TestCongestion -v`
Expected: PASS (all four `TestCongestion_*`).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/congestion.go internal/sim/congestion_test.go
git commit -m "$(cat <<'EOF'
feat(sim): add Congestion observatory for live per-edge speeds

EWMA-smoothed average speed per edge, refreshed from on-edge vehicles,
converted to routing cost = length / observed_speed with a floor (jams
stay finite-cost) and a free-flow ceiling (keeps A* heuristic admissible).

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: Router dynamic cost (`RouteCost`)

**Files:**
- Modify: `internal/sim/router.go` (split `Route` body into `RouteCost`)
- Test: `internal/sim/router_test.go` (append two tests)

- [ ] **Step 1: Write the failing tests**

Append to `internal/sim/router_test.go`:

```go
// buildDetourGraph: 0->2 direct (100m) plus a longer detour 0->1->2 (60+60m).
// Free-flow, the direct edge wins; under a cost that inflates it, the detour
// should win.
func buildDetourGraph() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 50, Y: 50}},
		{ID: 2, Pos: network.Point{X: 100, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 2, Length: 100, SpeedLimit: 10},
		{ID: 1, From: 0, To: 1, Length: 60, SpeedLimit: 10},
		{ID: 2, From: 1, To: 2, Length: 60, SpeedLimit: 10},
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestRouter_RouteCostAvoidsExpensiveEdge(t *testing.T) {
	net := buildDetourGraph()
	r := NewRouter(net)

	// Free-flow: the direct edge 0 wins.
	free, err := r.Route(0, 2)
	if err != nil {
		t.Fatalf("Route: %v", err)
	}
	if len(free) != 1 || free[0] != 0 {
		t.Fatalf("free-flow want [0], got %v", free)
	}

	// Inflate the direct edge; the detour must win.
	cost := func(eid network.EdgeID) float64 {
		if eid == 0 {
			return 1000
		}
		e := &net.Edges[eid]
		return e.Length / e.SpeedLimit
	}
	got, err := r.RouteCost(0, 2, cost)
	if err != nil {
		t.Fatalf("RouteCost: %v", err)
	}
	if len(got) != 2 || got[0] != 1 || got[1] != 2 {
		t.Fatalf("RouteCost want [1 2], got %v", got)
	}
}

func TestRouter_RouteUnchanged(t *testing.T) {
	net := buildLineGraph()
	r := NewRouter(net)
	got, err := r.Route(0, 3)
	if err != nil {
		t.Fatalf("Route: %v", err)
	}
	want := []network.EdgeID{0, 1, 2}
	if len(got) != len(want) {
		t.Fatalf("want %v, got %v", want, got)
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("step %d: want %d, got %d", i, want[i], got[i])
		}
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./internal/sim/ -run 'TestRouter_RouteCostAvoidsExpensiveEdge|TestRouter_RouteUnchanged' -v`
Expected: FAIL — does not compile (`r.RouteCost undefined`).

- [ ] **Step 3: Write the implementation**

In `internal/sim/router.go`, replace the `Route` method (currently lines 63–147, from the comment `// Route returns the edge IDs...` through its closing brace) with the wrapper plus the renamed search method below. The A* body is unchanged **except** the edge-relaxation cost line.

```go
// Route returns the edge IDs to traverse from src to dst using free-flow
// travel time (length / speed limit) as the cost, respecting any turn
// restrictions on the intermediate intersections. Behavior is identical to
// before RouteCost was introduced; it now delegates to RouteCost.
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

// RouteCost is Route with a caller-supplied per-edge cost function. cost(eid)
// must return a positive traversal cost (e.g. travel time). The A* heuristic
// is straight-line distance / max speed, which stays admissible as long as
// cost never implies a speed above that max — Congestion.Cost guarantees this
// via its free-flow ceiling.
func (r *Router) RouteCost(src, dst network.NodeID, cost func(network.EdgeID) float64) ([]network.EdgeID, error) {
	if src == dst {
		return nil, nil
	}

	dstPos := r.net.Nodes[dst].Pos
	heuristic := func(n network.NodeID) float64 {
		p := r.net.Nodes[n].Pos
		dx, dy := p.X-dstPos.X, p.Y-dstPos.Y
		return math.Sqrt(dx*dx+dy*dy) / 31.3 // admissible against max speed
	}

	gScore := make(map[searchState]float64)
	cameFrom := make(map[searchState]searchState)
	closed := make(map[searchState]bool)

	start := searchState{Node: src, ArrivedVia: noEdge}
	gScore[start] = 0

	open := &pq{}
	heap.Init(open)
	heap.Push(open, &pqItem{state: start, f: heuristic(src)})

	for open.Len() > 0 {
		cur := heap.Pop(open).(*pqItem)
		if cur.state.Node == dst {
			return reconstruct(cameFrom, start, cur.state), nil
		}
		if closed[cur.state] {
			continue
		}
		closed[cur.state] = true

		// Skip outgoing edges that are banned given our arrival edge.
		nodeBans := r.banned[cur.state.Node]
		var fromBans map[network.EdgeID]bool
		if nodeBans != nil && cur.state.ArrivedVia != noEdge {
			fromBans = nodeBans[cur.state.ArrivedVia]
		}

		// Prohibit U-turns at intermediate nodes UNLESS the U-turn is the only
		// non-banned option (e.g., dead-end streets).
		uTurnsAllowed := cur.state.ArrivedVia == noEdge
		if !uTurnsAllowed {
			uTurnsAllowed = true
			for _, eid := range r.adjOut[cur.state.Node] {
				if fromBans != nil && fromBans[eid] {
					continue
				}
				if network.ClassifyTurn(r.net, cur.state.ArrivedVia, eid) != network.TurnUTurn {
					uTurnsAllowed = false
					break
				}
			}
		}

		for _, eid := range r.adjOut[cur.state.Node] {
			if fromBans != nil && fromBans[eid] {
				continue
			}
			if !uTurnsAllowed &&
				network.ClassifyTurn(r.net, cur.state.ArrivedVia, eid) == network.TurnUTurn {
				continue
			}
			e := &r.net.Edges[eid]
			next := searchState{Node: e.To, ArrivedVia: eid}
			tentative := gScore[cur.state] + cost(eid)
			if existing, ok := gScore[next]; ok && tentative >= existing {
				continue
			}
			gScore[next] = tentative
			cameFrom[next] = cur.state
			heap.Push(open, &pqItem{state: next, f: tentative + heuristic(next.Node)})
		}
	}
	return nil, ErrNoRoute
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestRouter' -v`
Expected: PASS (existing `TestRouter_*` plus the two new ones).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/router.go internal/sim/router_test.go
git commit -m "$(cat <<'EOF'
feat(sim): add Router.RouteCost with caller-supplied edge costs

Route now delegates to a cost-parameterized A* (RouteCost) with its
free-flow cost as the default, so existing behavior is unchanged. Lets
GPS rerouting plug in live congestion costs.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: `VehicleReroute` trace event

**Files:**
- Modify: `internal/trace/events.go` (new kind + struct)
- Modify: `internal/trace/writer.go` (encode case)
- Modify: `internal/trace/reader.go` (decode case)
- Test: `internal/trace/trace_test.go` (append round-trip test)

- [ ] **Step 1: Write the failing test**

Append to `internal/trace/trace_test.go`:

```go
func TestRoundTrip_VehicleReroute(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)

	in := []Event{
		&VehicleReroute{VehicleID: 7, AtIndex: 2, NewTail: []uint32{5, 6, 7}},
		&VehicleReroute{VehicleID: 8, AtIndex: 0, NewTail: nil}, // empty tail
	}
	for i, e := range in {
		if err := w.Write(uint64(i), float64(i), e); err != nil {
			t.Fatalf("write %T: %v", e, err)
		}
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}

	r := NewReader(&buf)
	for i, want := range in {
		_, ev, err := r.Next()
		if err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
		gotRR, ok := ev.(*VehicleReroute)
		if !ok {
			t.Fatalf("event %d: got %T, want *VehicleReroute", i, ev)
		}
		wantRR := want.(*VehicleReroute)
		if gotRR.VehicleID != wantRR.VehicleID || gotRR.AtIndex != wantRR.AtIndex {
			t.Errorf("event %d: got %+v, want %+v", i, gotRR, wantRR)
		}
		if len(gotRR.NewTail) != len(wantRR.NewTail) {
			t.Errorf("event %d: tail len %d, want %d", i, len(gotRR.NewTail), len(wantRR.NewTail))
		}
		for j := range wantRR.NewTail {
			if gotRR.NewTail[j] != wantRR.NewTail[j] {
				t.Errorf("event %d tail[%d]: got %d, want %d", i, j, gotRR.NewTail[j], wantRR.NewTail[j])
			}
		}
	}
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `go test ./internal/trace/ -run TestRoundTrip_VehicleReroute -v`
Expected: FAIL — does not compile (`undefined: VehicleReroute`).

- [ ] **Step 3a: Add the event kind and struct**

In `internal/trace/events.go`, add the new kind to the `const` block (after `KindTraceDropped Kind = 8`):

```go
	// KindVehicleReroute records that a vehicle replaced the tail of its route
	// at runtime (GPS rerouting around congestion).
	KindVehicleReroute Kind = 9
```

And add the struct + `Kind()` method (place after the `TraceDropped` type, before `UnknownEvent`):

```go
// VehicleReroute records that a vehicle replaced the tail of its route at
// runtime (GPS rerouting around congestion). AtIndex is the route index of the
// first replaced edge; NewTail is the replacement edge sequence. Replayers
// splice route[:AtIndex] + NewTail to follow the path actually taken.
type VehicleReroute struct {
	VehicleID uint32
	AtIndex   uint32
	NewTail   []uint32
}

func (*VehicleReroute) Kind() Kind { return KindVehicleReroute }
```

- [ ] **Step 3b: Add the encode case**

In `internal/trace/writer.go`, inside `encodePayload`'s `switch`, add this case (e.g. after the `*VehicleDespawn` case):

```go
	case *VehicleReroute:
		if err := binary.Write(b, le, ev.VehicleID); err != nil {
			return err
		}
		if err := binary.Write(b, le, ev.AtIndex); err != nil {
			return err
		}
		if len(ev.NewTail) > 0xFFFF {
			return fmt.Errorf("trace VehicleReroute tail too long: %d edges (max %d)", len(ev.NewTail), 0xFFFF)
		}
		if err := binary.Write(b, le, uint16(len(ev.NewTail))); err != nil {
			return err
		}
		for _, eid := range ev.NewTail {
			if err := binary.Write(b, le, eid); err != nil {
				return err
			}
		}
		return nil
```

- [ ] **Step 3c: Add the decode case**

In `internal/trace/reader.go`, inside `decodePayload`'s `switch`, add this case (e.g. after the `KindVehicleDespawn` case):

```go
	case KindVehicleReroute:
		e := &VehicleReroute{}
		if err := binary.Read(rd, le, &e.VehicleID); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.AtIndex); err != nil {
			return nil, err
		}
		var n uint16
		if err := binary.Read(rd, le, &n); err != nil {
			return nil, err
		}
		e.NewTail = make([]uint32, n)
		for i := range e.NewTail {
			if err := binary.Read(rd, le, &e.NewTail[i]); err != nil {
				return nil, err
			}
		}
		return e, nil
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `go test ./internal/trace/ -v`
Expected: PASS (new `TestRoundTrip_VehicleReroute` plus all existing trace tests).

- [ ] **Step 5: Commit**

```bash
git add internal/trace/events.go internal/trace/writer.go internal/trace/reader.go internal/trace/trace_test.go
git commit -m "$(cat <<'EOF'
feat(trace): add VehicleReroute event (kind 9)

Records a runtime route-tail replacement so tracereplay follows the path
a GPS vehicle actually took. Additive and forward-compatible: older
readers skip it via the per-event length field.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Data model — Vehicle fields, World wiring, congestion update in Step

**Files:**
- Modify: `internal/sim/vehicle.go` (three new `Vehicle` fields)
- Modify: `internal/sim/world.go` (`GpsShare`/`Cong` fields; construct `Cong` in `NewWorld`; call `Cong.Update` in `Step`)
- Test: `internal/sim/world_test.go` (append one test)

- [ ] **Step 1: Write the failing test**

Append to `internal/sim/world_test.go`:

```go
func TestWorld_CongestionRisesUnderJam(t *testing.T) {
	net := buildLineGraph() // 3 edges, 100m, 10 m/s; edge 0 ends at a plain node
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	free := w.Cong.Cost(net, 0)

	// Two vehicles parked on edge 0, pinned stationary each tick so the
	// observed mean speed there stays ~0 and Congestion.Update drives the cost
	// up. (Hand-built vehicles have HasGPS=false, so no rerouting occurs.)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 20, V: 0},
		{ID: 2, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 40, V: 0},
	}
	w.nextID = 3

	for i := 0; i < 300; i++ { // 15 sim-seconds — under the 60s stuck-despawn
		for j := range w.Vehicles {
			w.Vehicles[j].V = 0
			w.Vehicles[j].S = 20 + float64(j)*20 // pin in place; never reach edge end
		}
		w.Step()
	}

	jammed := w.Cong.Cost(net, 0)
	if jammed <= free {
		t.Fatalf("jammed cost %v should exceed free-flow %v after a sustained stop", jammed, free)
	}
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `go test ./internal/sim/ -run TestWorld_CongestionRisesUnderJam -v`
Expected: FAIL — does not compile (`w.Cong undefined`).

- [ ] **Step 3a: Add the Vehicle fields**

In `internal/sim/vehicle.go`, add to the `Vehicle` struct (after the `LastLCDir` field, before `Despawned`):

```go
	// HasGPS marks a vehicle that re-routes around congestion. Set at spawn
	// from World.GpsShare. Hand-constructed test vehicles default to false
	// (zero value) and are never re-routed, so existing fixtures are
	// unaffected.
	HasGPS bool

	// DestNode is the vehicle's destination node, cached at spawn so
	// re-routing needn't re-derive it from the route tail. Equal to the To
	// node of the final route edge.
	DestNode network.NodeID

	// LastRerouteSec is the sim-time of this vehicle's most recent reroute
	// attempt (or spawn). Gates re-routing to at most once per
	// rerouteCooldownSec so a vehicle doesn't recompute on every short edge.
	LastRerouteSec float64
```

- [ ] **Step 3b: Add World fields**

In `internal/sim/world.go`, add to the `World` struct (after the `rng` field):

```go
	// Cong tracks live per-edge congestion and supplies routing costs.
	Cong *Congestion

	// GpsShare is the fraction of spawned vehicles given GPS rerouting, in
	// [0,1]. Defaults to 1.0 (every vehicle) in NewWorld; overridden from the
	// --gps-share flag.
	GpsShare float64
```

- [ ] **Step 3c: Construct Cong and default GpsShare in NewWorld**

In `internal/sim/world.go`, in the `return &World{...}` literal inside `NewWorld`, add these two fields (after `rng: ...`):

```go
		Cong:     NewCongestion(net, ewmaHalfLifeSec, DefaultDt),
		GpsShare: 1.0,
```

- [ ] **Step 3d: Call Congestion.Update in Step**

In `internal/sim/world.go`, in `Step`, immediately after the per-lane sort loop in section 2 (the loop `for _, lanes := range byEdgeLane { ... sortVehicleIdxByS(...) }`) and before section 3's leader pre-compute, insert:

```go
	// 2b. Refresh live per-edge congestion from this tick's positions/speeds.
	w.Cong.Update(w.Net, byEdge, w.Vehicles)
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `go test ./internal/sim/ -run TestWorld_CongestionRisesUnderJam -v`
Expected: PASS.

Then run the whole sim package to confirm nothing regressed:
Run: `go test ./internal/sim/`
Expected: PASS (ok).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/vehicle.go internal/sim/world.go internal/sim/world_test.go
git commit -m "$(cat <<'EOF'
feat(sim): wire Congestion into World and add GPS vehicle fields

NewWorld builds a Congestion table (default GpsShare=1.0); Step refreshes
it each tick from on-edge vehicles. Vehicle gains HasGPS, DestNode, and
LastRerouteSec for the rerouting logic that follows.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: GPS assignment and cost-aware spawn routing

**Files:**
- Modify: `internal/sim/world.go` (`trySpawn`)
- Test: `internal/sim/world_test.go` (append two tests)

- [ ] **Step 1: Write the failing tests**

Append to `internal/sim/world_test.go`:

```go
func TestWorld_GpsShare_BoundsAllOrNone(t *testing.T) {
	check := func(share float64, wantGPS bool) {
		net := build2x2Grid()
		w := NewWorld(net, NewRandomOD(net, 7, 30.0), nil)
		w.GpsShare = share
		w.Run(4.0)
		seen := false
		for i := range w.Vehicles {
			seen = true
			if w.Vehicles[i].HasGPS != wantGPS {
				t.Fatalf("share=%v: vehicle %d HasGPS=%v, want %v",
					share, w.Vehicles[i].ID, w.Vehicles[i].HasGPS, wantGPS)
			}
		}
		if !seen {
			t.Fatalf("share=%v: no vehicles alive to check", share)
		}
	}
	check(1.0, true)  // Float64() in [0,1) is always < 1.0
	check(0.0, false) // never < 0.0
}

func TestWorld_GpsShare_DeterministicSplit(t *testing.T) {
	run := func() (gps, total int) {
		net := build2x2Grid()
		w := NewWorld(net, NewRandomOD(net, 4242, 50.0), nil)
		w.GpsShare = 0.5
		w.Run(5.0)
		for i := range w.Vehicles {
			total++
			if w.Vehicles[i].HasGPS {
				gps++
			}
		}
		return
	}
	g1, t1 := run()
	g2, t2 := run()
	if g1 != g2 || t1 != t2 {
		t.Fatalf("non-deterministic GPS split: run1 (%d/%d) run2 (%d/%d)", g1, t1, g2, t2)
	}
	if t1 == 0 {
		t.Fatalf("no vehicles spawned")
	}
	frac := float64(g1) / float64(t1)
	if frac < 0.2 || frac > 0.8 {
		t.Fatalf("GPS fraction %v far from 0.5 (gps=%d total=%d)", frac, g1, t1)
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./internal/sim/ -run 'TestWorld_GpsShare' -v`
Expected: FAIL — assertions fail (spawned vehicles have `HasGPS=false` because `trySpawn` doesn't set it yet).

- [ ] **Step 3: Rewrite `trySpawn`**

In `internal/sim/world.go`, replace the entire `trySpawn` method with the version below. The change: draw the speed and gap factors first (unchanged formulas), then draw a GPS roll, then route — using cost-aware `RouteCost` for GPS vehicles and free-flow `Route` otherwise — and set the three new `Vehicle` fields.

```go
func (w *World) trySpawn(r SpawnRequest) {
	// Sample a per-driver speed preference: Normal(1.0, σ), clamped.
	factor := 1.0 + w.rng.NormFloat64()*speedFactorStdDev
	if factor < speedFactorMin {
		factor = speedFactorMin
	} else if factor > speedFactorMax {
		factor = speedFactorMax
	}

	gapFactor := 1.0 + w.rng.NormFloat64()*gapFactorStdDev
	if gapFactor < gapFactorMin {
		gapFactor = gapFactorMin
	} else if gapFactor > gapFactorMax {
		gapFactor = gapFactorMax
	}

	// Decide GPS membership deterministically against the configured share.
	hasGPS := w.rng.Float64() < w.GpsShare

	// GPS vehicles route on live congestion cost; others on free-flow time.
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

	// Spawn at this driver's cruising speed so they don't immediately brake.
	v := Vehicle{
		ID:             w.nextID,
		Route:          route,
		Edge:           route[0],
		Lane:           0,
		S:              0,
		V:              w.Net.Edges[route[0]].SpeedLimit * factor,
		SpeedFactor:    factor,
		GapFactor:      gapFactor,
		HasGPS:         hasGPS,
		DestNode:       r.DestNode,
		LastRerouteSec: w.SimTime,
	}
	w.nextID++
	w.Vehicles = append(w.Vehicles, v)

	// Emit spawn event.
	route32 := make([]uint32, len(route))
	for i, eid := range route {
		route32[i] = uint32(eid)
	}
	w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleSpawn{
		VehicleID:  uint32(v.ID),
		OriginNode: uint32(r.OriginNode),
		DestNode:   uint32(r.DestNode),
		Route:      route32,
	})
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestWorld_GpsShare' -v`
Expected: PASS (both).

Then confirm the determinism guarantee still holds:
Run: `go test ./internal/sim/ -run TestWorld_TraceDeterminism -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "$(cat <<'EOF'
feat(sim): assign GPS at spawn and route GPS cars on live cost

trySpawn draws a GPS membership roll against GpsShare; GPS vehicles get a
congestion-aware initial route via RouteCost, others keep free-flow Route.
Caches DestNode and seeds LastRerouteSec. Deterministic; trace still
byte-identical across runs with the same seed.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: Reroute decision and trigger (core)

**Files:**
- Modify: `internal/sim/world.go` (constants; `maybeReroute`, `emitReroute`, `sameTail`; trigger in `Step`)
- Test: `internal/sim/world_test.go` (append helper + five tests)

- [ ] **Step 1: Write the failing tests**

Append to `internal/sim/world_test.go`:

```go
// buildRerouteGraph: from node 1 a vehicle can reach dest node 3 directly via
// e1 (1->3, 150m) or via the detour e2,e3 (1->2->3, 110+110m). Edge e0 (0->1)
// is the entry edge. Free-flow, the direct e1 is cheaper.
func buildRerouteGraph() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 100, Y: -100}},
		{ID: 3, Pos: network.Point{X: 250, Y: 0}},
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10, Lanes: []network.Lane{{Index: 0}},
		}
	}
	edges := []network.Edge{
		mk(0, 0, 1, 100), // e0 entry
		mk(1, 1, 3, 150), // e1 direct
		mk(2, 1, 2, 110), // e2 detour leg 1
		mk(3, 2, 3, 110), // e3 detour leg 2
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestWorld_Reroute_SwitchesAroundJam(t *testing.T) {
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	var events []*trace.VehicleReroute
	w.EmitTrace = func(_ uint64, _ float64, e trace.Event) {
		if rr, ok := e.(*trace.VehicleReroute); ok {
			events = append(events, rr)
		}
	}
	w.Cong.speed[1] = minEdgeSpeed // jam the direct edge

	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: true, DestNode: 3, LastRerouteSec: -1000,
	}
	if !w.maybeReroute(v) {
		t.Fatalf("maybeReroute returned false for an eligible GPS vehicle")
	}
	if len(v.Route) != 3 || v.Route[0] != 0 || v.Route[1] != 2 || v.Route[2] != 3 {
		t.Fatalf("route after reroute = %v, want [0 2 3]", v.Route)
	}
	if len(events) != 1 {
		t.Fatalf("want 1 VehicleReroute event, got %d", len(events))
	}
	if events[0].AtIndex != 1 || len(events[0].NewTail) != 2 ||
		events[0].NewTail[0] != 2 || events[0].NewTail[1] != 3 {
		t.Fatalf("event = %+v, want AtIndex 1 NewTail [2 3]", events[0])
	}
}

func TestWorld_Reroute_NonGPSDoesNotSwitch(t *testing.T) {
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Cong.speed[1] = minEdgeSpeed
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: false, DestNode: 3, LastRerouteSec: -1000,
	}
	if w.maybeReroute(v) {
		t.Fatalf("non-GPS vehicle should not attempt a reroute")
	}
	if len(v.Route) != 2 || v.Route[1] != 1 {
		t.Fatalf("non-GPS route changed: %v", v.Route)
	}
}

func TestWorld_Reroute_CooldownRespected(t *testing.T) {
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Cong.speed[1] = minEdgeSpeed
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: true, DestNode: 3, LastRerouteSec: w.SimTime, // just rerouted
	}
	if w.maybeReroute(v) {
		t.Fatalf("within cooldown, maybeReroute should not attempt")
	}
	if len(v.Route) != 2 {
		t.Fatalf("route changed despite cooldown: %v", v.Route)
	}
}

func TestWorld_Reroute_HysteresisNoFlap(t *testing.T) {
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Mild slowdown on the direct edge: Cost(e1)=150/6.25=24; detour=22, which
	// is cheaper but within switchMargin (22 > 24*0.85=20.4) → no switch.
	w.Cong.speed[1] = 6.25
	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: true, DestNode: 3, LastRerouteSec: -1000,
	}
	if !w.maybeReroute(v) {
		t.Fatalf("eligible GPS vehicle should make an attempt (return true)")
	}
	if len(v.Route) != 2 || v.Route[1] != 1 {
		t.Fatalf("hysteresis failed: switched on a sub-margin improvement: %v", v.Route)
	}
}

func TestWorld_Reroute_TriggersOnEdgeEntry(t *testing.T) {
	// e_pre(4->0) feeds e0(0->1); from node 1, e1(1->3) direct or e2,e3 detour.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 100, Y: -100}},
		{ID: 3, Pos: network.Point{X: 250, Y: 0}},
		{ID: 4, Pos: network.Point{X: -100, Y: 0}},
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10, Lanes: []network.Lane{{Index: 0}}}
	}
	net := &network.Network{Nodes: nodes, Edges: []network.Edge{
		mk(0, 0, 1, 100), // e0
		mk(1, 1, 3, 150), // e1 direct (jammed)
		mk(2, 1, 2, 110), // e2 detour
		mk(3, 2, 3, 110), // e3 detour
		mk(4, 4, 0, 100), // e_pre
	}}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	var rerouted bool
	w.EmitTrace = func(_ uint64, _ float64, e trace.Event) {
		if _, ok := e.(*trace.VehicleReroute); ok {
			rerouted = true
		}
	}
	// Near the end of e_pre, about to cross into e0.
	w.Vehicles = []Vehicle{{
		ID: 1, Route: []network.EdgeID{4, 0, 1}, RouteIdx: 0, Edge: 4, S: 99, V: 10,
		HasGPS: true, DestNode: 3, LastRerouteSec: -1000,
	}}
	w.nextID = 2

	for i := 0; i < 5; i++ {
		w.Cong.speed[1] = minEdgeSpeed // keep the direct edge jammed each tick
		w.Step()
	}

	if len(w.Vehicles) == 0 {
		t.Fatalf("vehicle unexpectedly despawned")
	}
	got := w.Vehicles[0].Route
	if len(got) < 3 || got[0] != 4 || got[1] != 0 || got[2] != 2 {
		t.Fatalf("route after edge-entry reroute = %v, want prefix [4 0 2 ...]", got)
	}
	if !rerouted {
		t.Fatalf("no VehicleReroute event emitted on edge entry")
	}
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `go test ./internal/sim/ -run 'TestWorld_Reroute' -v`
Expected: FAIL — does not compile (`w.maybeReroute undefined`).

- [ ] **Step 3a: Add the reroute constants**

In `internal/sim/world.go`, add a `const` block near the other tuning constants (e.g. just after the `stuck*`/`stopDwellSec` block):

```go
const (
	// rerouteCooldownSec is the minimum sim-time between a vehicle's reroutes.
	// 20s gives GPS-like periodic re-evaluation without thrashing A* on every
	// short edge a vehicle crosses.
	rerouteCooldownSec = 20.0

	// switchMargin is the hysteresis threshold: a vehicle adopts a candidate
	// route only if its estimated cost is at least this fraction cheaper than
	// the current remaining route. 0.15 stops flapping between near-equal
	// routes as smoothed speeds wobble.
	switchMargin = 0.15

	// maxReroutesPerTick caps reroute attempts per tick as a defensive guard
	// on the 50ms tick budget under pathological spawn rates. The
	// edge-transition trigger plus cooldown keep the real count far lower.
	maxReroutesPerTick = 64
)
```

- [ ] **Step 3b: Add `maybeReroute`, `emitReroute`, and `sameTail`**

In `internal/sim/world.go`, add these (e.g. just before `trySpawn`):

```go
// maybeReroute re-evaluates a GPS vehicle's remaining path against live
// congestion costs and switches to a cheaper route if one beats the current
// remaining route by switchMargin. Called when the vehicle crosses into a new
// edge. Returns true if an attempt was made (A* was run or skipped only by the
// cheap pre-checks); true consumes one slot of the per-tick reroute budget.
//
// On switch it splices Route[:RouteIdx+1] + candidate (RouteIdx still points
// at the current edge, unchanged) and emits a VehicleReroute event so replay
// follows the new path.
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

	costFn := func(eid network.EdgeID) float64 { return w.Cong.Cost(w.Net, eid) }
	src := w.Net.Edges[v.Edge].To
	candidate, err := w.Router.RouteCost(src, v.DestNode, costFn)
	if err != nil || len(candidate) == 0 {
		return true // attempt made; keep current route
	}

	curCost := 0.0
	for _, eid := range v.Route[v.RouteIdx+1:] {
		curCost += costFn(eid)
	}
	newCost := 0.0
	for _, eid := range candidate {
		newCost += costFn(eid)
	}

	if newCost < curCost*(1-switchMargin) && !sameTail(v.Route[v.RouteIdx+1:], candidate) {
		idx := v.RouteIdx + 1
		// 3-index slice caps capacity so append allocates fresh, avoiding any
		// aliasing with the old tail.
		v.Route = append(v.Route[:idx:idx], candidate...)
		w.emitReroute(v, idx, candidate)
	}
	return true
}

// emitReroute writes a VehicleReroute trace event for a route-tail switch.
func (w *World) emitReroute(v *Vehicle, atIndex int, tail []network.EdgeID) {
	tail32 := make([]uint32, len(tail))
	for i, eid := range tail {
		tail32[i] = uint32(eid)
	}
	w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleReroute{
		VehicleID: uint32(v.ID),
		AtIndex:   uint32(atIndex),
		NewTail:   tail32,
	})
}

// sameTail reports whether two edge sequences are identical.
func sameTail(a, b []network.EdgeID) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
```

- [ ] **Step 3c: Add the trigger to `Step`**

In `internal/sim/world.go`, in `Step`'s section 4 (the per-vehicle stepping loop `for i := range w.Vehicles { ... }`):

First, declare a per-tick budget immediately before that loop. Replace these existing two comment lines + the `for`:

```go
	// 4. Step each vehicle, applying signal/yield virtual leaders.
	//    Iterate vehicles in stable index order to preserve determinism.
	for i := range w.Vehicles {
```

with:

```go
	// 4. Step each vehicle, applying signal/yield virtual leaders.
	//    Iterate vehicles in stable index order to preserve determinism.
	rerouteBudget := maxReroutesPerTick
	for i := range w.Vehicles {
```

Then capture the pre-step edge and add the reroute call around the `stepIDM`
invocation. Replace these two existing lines:

```go
		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)
```

with:

```go
		prevEdge := v.Edge
		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// GPS rerouting fires on edge entry (a decision point), bounded by the
		// per-tick budget. maybeReroute self-gates on HasGPS and cooldown.
		if !v.Despawned && v.Edge != prevEdge && rerouteBudget > 0 {
			if w.maybeReroute(v) {
				rerouteBudget--
			}
		}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `go test ./internal/sim/ -run 'TestWorld_Reroute' -v`
Expected: PASS (all five).

Then the full sim package, including determinism:
Run: `go test ./internal/sim/`
Expected: PASS (ok).

- [ ] **Step 5: Commit**

```bash
git add internal/sim/world.go internal/sim/world_test.go
git commit -m "$(cat <<'EOF'
feat(sim): reroute GPS vehicles around jams on edge entry

On crossing into a new edge, a GPS vehicle past its cooldown re-runs A*
on live congestion cost and switches only on a switchMargin improvement
(hysteresis), splicing the route tail and emitting VehicleReroute. A
per-tick budget guards the tick budget. Deterministic.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: CLI `--gps-share` flag

**Files:**
- Modify: `cmd/trafficsim/main.go` (`runFlags`, `newRunFlagSet`, `runRun`)

- [ ] **Step 1: Add the flag field**

In `cmd/trafficsim/main.go`, add to the `runFlags` struct (after `tracePath string`):

```go
	gpsShare float64
```

- [ ] **Step 2: Register the flag**

In `newRunFlagSet`, after the existing `fs.StringVar(&f.tracePath, ...)` line, add:

```go
	fs.Float64Var(&f.gpsShare, "gps-share", 1.0,
		"fraction of vehicles (0..1) with GPS rerouting around congestion")
```

- [ ] **Step 3: Validate and wire it**

In `runRun`, after the existing `if f.headless && f.duration == 0 { ... }` check, add validation:

```go
	if f.gpsShare < 0 || f.gpsShare > 1 {
		return fmt.Errorf("--gps-share must be in [0,1], got %v", f.gpsShare)
	}
```

Then, after the `w := sim.NewWorld(net, spawner, overrides)` line, add:

```go
	w.GpsShare = f.gpsShare
```

- [ ] **Step 4: Build and smoke-test**

Run: `go build ./cmd/trafficsim/`
Expected: builds with no errors.

Run: `./trafficsim run -h 2>&1 | grep gps-share`
Expected: a line documenting `-gps-share float ... (default 1)`.

Run: `./trafficsim run --gps-share 1.5 --headless --duration 1s configs/does-not-exist.osm 2>&1 | head -1`
Expected: an error mentioning `--gps-share must be in [0,1]` (the flag is validated before the OSM is loaded).

- [ ] **Step 5: Commit**

```bash
git add cmd/trafficsim/main.go
git commit -m "$(cat <<'EOF'
feat(cli): add --gps-share to tune GPS rerouting penetration

Fraction (0..1) of vehicles given GPS rerouting; defaults to 1.0 (every
vehicle). Validated to [0,1] and wired into World.GpsShare.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: `tracereplay` applies reroutes

**Files:**
- Modify: `cmd/tracereplay/player.go` (`apply`)
- Test: `cmd/tracereplay/player_test.go` (new)

- [ ] **Step 1: Write the failing test**

Create `cmd/tracereplay/player_test.go`:

```go
package main

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

func TestPlayer_AppliesReroute(t *testing.T) {
	net := &network.Network{
		Nodes: []network.Node{{ID: 0}, {ID: 1}, {ID: 2}, {ID: 3}},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10},
			{ID: 1, From: 1, To: 3, Length: 100, SpeedLimit: 10},
			{ID: 2, From: 1, To: 2, Length: 100, SpeedLimit: 10},
			{ID: 3, From: 2, To: 3, Length: 100, SpeedLimit: 10},
		},
	}
	p := newPlayer(net, nil, snapshot.New(), 1.0)
	hdr := trace.Header{}
	p.apply(hdr, &trace.VehicleSpawn{VehicleID: 7, Route: []uint32{0, 1}, OriginNode: 0, DestNode: 3})
	p.apply(hdr, &trace.VehicleReroute{VehicleID: 7, AtIndex: 1, NewTail: []uint32{2, 3}})

	rv := p.vehicles[7]
	if rv == nil {
		t.Fatalf("vehicle missing after spawn")
	}
	want := []uint32{0, 2, 3}
	if len(rv.route) != len(want) {
		t.Fatalf("route %v, want %v", rv.route, want)
	}
	for i := range want {
		if rv.route[i] != want[i] {
			t.Fatalf("route[%d]=%d, want %d", i, rv.route[i], want[i])
		}
	}
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `go test ./cmd/tracereplay/ -run TestPlayer_AppliesReroute -v`
Expected: FAIL — the `VehicleReroute` is ignored, so `rv.route` stays `[0 1]` (assertion fails on length).

- [ ] **Step 3: Handle the event in `apply`**

In `cmd/tracereplay/player.go`, in the `apply` method's `switch`, add this case (e.g. after the `*trace.VehicleDespawn` case):

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

- [ ] **Step 4: Run the test to verify it passes**

Run: `go test ./cmd/tracereplay/ -run TestPlayer_AppliesReroute -v`
Expected: PASS.

Build the binary too:
Run: `go build ./cmd/tracereplay/`
Expected: builds with no errors.

- [ ] **Step 5: Commit**

```bash
git add cmd/tracereplay/player.go cmd/tracereplay/player_test.go
git commit -m "$(cat <<'EOF'
feat(tracereplay): apply VehicleReroute to follow taken path

Splices route[:AtIndex] + NewTail, keeping the replay vehicle's routeIdx
valid against the unchanged prefix.

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Full verification and README

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Run the entire test suite**

Run: `go test ./...`
Expected: all packages `ok`. In particular `internal/sim` (incl. `TestWorld_TraceDeterminism`), `internal/trace`, and `cmd/tracereplay` pass.

If any pre-existing spawn-based test fails because spawned vehicles now reroute (it asserts a specific spawned route or exact post-spawn position), pin pre-feature routing in that test by adding `w.GpsShare = 0` right after its `NewWorld(...)` call — unless the test is specifically about rerouting. Re-run until green. (The determinism and `GapFactor`-distribution tests are robust to the new rng draw and should not need changes.)

- [ ] **Step 2: Vet and build everything**

Run: `go vet ./...`
Expected: no diagnostics.

Run: `go build ./cmd/trafficsim/ ./cmd/tracereplay/`
Expected: both build with no errors.

- [ ] **Step 3: Headless end-to-end smoke (optional, needs an OSM file)**

If an OSM extract is available (e.g. `extract.osm.pbf`):

Run: `./trafficsim run --headless --duration 30s --spawn-rate 20 --gps-share 1.0 --trace /tmp/gps.trace extract.osm.pbf`
Expected: completes, prints `done. final_vehicles=... ticks=... sim_time=30.00s`.

Run: `./tracereplay -osm extract.osm.pbf -trace /tmp/gps.trace -speed 8`
Expected: replay window opens and vehicles move; no fatal errors. (Skip this step if no OSM file or no display is available.)

- [ ] **Step 4: Update the README**

In `README.md`, under the `## Run` section (after the live-viewer example), add a short subsection documenting the flag. The exact markdown to add is shown below (outer four-backtick fence is only to display the inner triple-backtick block — add the inner content, starting at `### GPS rerouting`, to the README):

````markdown
### GPS rerouting

By default every vehicle has GPS and re-routes around congestion: each edge's
live average speed feeds a travel-time cost, and a vehicle re-evaluates its
remaining path when it enters a new edge, switching to a meaningfully faster
route when one exists. Tune the share of GPS-equipped vehicles with
`--gps-share` (0..1, default 1.0):

```
./trafficsim run --gps-share 0.5 --spawn-rate 20 extract.osm.pbf   # half the fleet
./trafficsim run --gps-share 0 --spawn-rate 20 extract.osm.pbf     # static routing
```

Reroutes are recorded in the trace (a `VehicleReroute` event), so `tracereplay`
follows the path each vehicle actually took.
````

- [ ] **Step 5: Commit**

```bash
git add README.md
git commit -m "$(cat <<'EOF'
docs(readme): document GPS rerouting and --gps-share

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

---

## Notes on determinism (read before implementing)

- The simulation's contract is "same seed + OSM + spawn-rate → byte-identical
  trace," verified by `TestWorld_TraceDeterminism` (two runs of the *same*
  binary). This feature keeps that: no wall-clock reads, `Congestion.Update`
  uses order-independent sums, reroutes run in stable vehicle-index order, and
  A* is deterministic. Traces differ from pre-feature binaries — which the
  contract allows.
- `trySpawn` now draws one extra `w.rng` value (the GPS roll) and routes after
  the factor draws. The `RandomOD` spawner uses a separate rng, so origin/dest
  selection is unaffected; only per-vehicle property values shift, identically
  across runs.
