package sim

import (
	"bytes"
	"log/slog"
	"math"
	"strings"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/trace"
)

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

// build2x2Grid returns a 2x2 grid: 4 nodes arranged in a square,
// with edges between adjacent nodes in both directions. 100m blocks.
//
// 2 --- 3
// |     |
// 0 --- 1
func build2x2Grid() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: 100}},
		{ID: 3, Pos: network.Point{X: 100, Y: 100}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID:         network.EdgeID(id),
			From:       network.NodeID(from),
			To:         network.NodeID(to),
			Length:     100,
			SpeedLimit: 10,
			Lanes:      []network.Lane{{Index: 0}},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 1), mkEdge(1, 1, 0),
		mkEdge(2, 0, 2), mkEdge(3, 2, 0),
		mkEdge(4, 1, 3), mkEdge(5, 3, 1),
		mkEdge(6, 2, 3), mkEdge(7, 3, 2),
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestWorld_VehiclesSpawnMoveDespawn(t *testing.T) {
	net := build2x2Grid()
	spawner := NewRandomOD(net, 7, 5.0) // 5 vehicles/sec
	w := NewWorld(net, spawner, nil)

	w.Run(11.0) // 11 simulated seconds (100m edges at 10 m/s = 10s/edge; 11s lets the first spawned vehicles finish)

	if w.Tick == 0 {
		t.Fatalf("no ticks ran")
	}
	// Some vehicles should have completed and despawned by now. With a
	// 100m block at 10 m/s, a single-edge trip is 10s — vehicles spawned in
	// the first second should have finished by 11s.
	if w.nextID == 0 {
		t.Errorf("expected some spawns over 11s @ 5/s, got 0")
	}

	// Some vehicles should have despawned (compact() actually fires).
	// nextID is total spawned ever; len(Vehicles) is alive count. Their
	// difference is the number that completed and were compacted out.
	spawned := uint32(w.nextID)
	alive := uint32(len(w.Vehicles))
	if spawned == 0 {
		t.Fatalf("no vehicles spawned")
	}
	if spawned <= alive {
		t.Errorf("expected some despawns over 11s, spawned=%d alive=%d", spawned, alive)
	}
}

func TestWorld_IDMFollowingMaintainsGap(t *testing.T) {
	net := buildLineGraph() // 3 edges, 100m each, 10 m/s
	// No spawner — we'll inject vehicles directly.
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

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

func TestWorld_StopsAtRedLight(t *testing.T) {
	// Build a single-edge graph ending at a signalized intersection.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{{Index: 0}}},
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Force the signal to all-red by giving it an empty-green phase.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: nil, GreenDur: 1000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 50, V: 10}}
	w.nextID = 2

	for i := 0; i < 500; i++ { // 25 sim-seconds
		w.Step()
	}

	if len(w.Vehicles) != 1 {
		t.Fatalf("vehicle should not have completed (red light), got %d alive", len(w.Vehicles))
	}
	v := &w.Vehicles[0]
	if v.V > 0.1 {
		t.Errorf("vehicle should be stopped at red, V=%.2f", v.V)
	}
	if v.S < 190 {
		t.Errorf("vehicle should be near the stop line, S=%.2f (edge length 200)", v.S)
	}
}

// TestWorld_CrossEdgeLeader_PostTurnLane: a right-turner in lane 1 of the
// inbound edge must find the leader in lane 0 of the outbound edge — the
// post-right-turn target — not lane 1. If the cross-edge leader lookup keys
// off the ego's pre-turn lane, the ego misses the stopped leader on edge 1,
// crosses the intersection at full speed, snaps to lane 0 (right-turn rule),
// and collides with the previously-invisible leader.
//
// Regression: see review #2 (2026-05-15).
func TestWorld_CrossEdgeLeader_PostTurnLane(t *testing.T) {
	// Edge 0: (0,0) → (200,0)  — heading east, 2 lanes, 200m long
	// Edge 1: (200,0) → (200,-100) — heading south (right turn from edge 0),
	//        2 lanes, 100m long. Turn angle = -90° → TurnRight.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: -100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}, {Index: 1}},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}, {Index: 1}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	// Sanity-check the turn classification — if this changes, the test setup
	// no longer exercises the bug.
	if cat := network.ClassifyTurn(net, 0, 1); cat != network.TurnRight {
		t.Fatalf("turn 0→1 should classify as TurnRight, got %v", cat)
	}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	// Ego: lane 1 of edge 0, 15m from end of edge 0, moving at v0=10.
	// Leader: stopped at S=5 on edge 1 in lane 0 (the post-right-turn target lane).
	// Cross-edge gap = (200 - 15) + 5 = 190... wait, ego S=185 → 200-185=15m to
	// end of edge 0, then 5m into edge 1 = 20m gap. With v=10 closing on v=0,
	// IDM safe-distance > 20m, so ego must decelerate strongly.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 185, V: 10, Lane: 1}, // ego
		{ID: 2, Route: []network.EdgeID{1}, Edge: 1, S: 5, V: 0, Lane: 0},       // leader (stopped)
	}
	w.nextID = 3

	// Linear distance helper: along-route arc-length, used for collision detection.
	pos := func(v *Vehicle) float64 {
		switch v.Edge {
		case 0:
			return v.S
		case 1:
			return edges[0].Length + v.S
		}
		return 0
	}

	// Run for 4 sim-seconds (80 ticks). The ego will cross the intersection
	// within ~1.5s. Any tick where gap < VehicleLength is a collision.
	minGap := math.Inf(1)
	for i := 0; i < 80; i++ {
		w.Step()
		// The leader vehicle is the stopped one (V=0 throughout); locate by ID.
		var ego, leader *Vehicle
		for j := range w.Vehicles {
			switch w.Vehicles[j].ID {
			case 1:
				ego = &w.Vehicles[j]
			case 2:
				leader = &w.Vehicles[j]
			}
		}
		if ego == nil || leader == nil {
			t.Fatalf("tick %d: lost a vehicle (ego=%v leader=%v)", i, ego, leader)
		}
		gap := pos(leader) - pos(ego) - VehicleLength
		if gap < minGap {
			minGap = gap
		}
		if gap < 0 {
			t.Fatalf("tick %d: collision — ego at edge=%d S=%.2f V=%.2f lane=%d, leader at edge=%d S=%.2f, gap=%.2f",
				i, ego.Edge, ego.S, ego.V, ego.Lane, leader.Edge, leader.S, gap)
		}
	}
	if minGap < 0 {
		t.Errorf("min gap over run was %.2f (collision)", minGap)
	}
}

// buildSignalApproach returns a single-edge graph ending at a signalized
// intersection: a 200m road from node 0 to node 1, with node 1 the
// signal. Used by the soft-red yellow tests.
func buildSignalApproach(speedLimit float64) *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: speedLimit,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
	}
	return &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}
}

// TestWorld_YellowCommitsWhenTooClose: a vehicle in the dilemma zone
// (closer to the stop line than its comfortable stopping distance) must
// commit through yellow rather than panic-brake. This is the soft-red
// yellow behavior — it prevents cars from being caught mid-intersection
// when the cross-stream gets green.
func TestWorld_YellowCommitsWhenTooClose(t *testing.T) {
	net := buildSignalApproach(13)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3}},
	})
	w.SignalStates[0].IsYellow = true

	// 10m from stop line at 13 m/s. Comfortable stop ≈ 13²/3 + 2 ≈ 58m,
	// so 10m << 58m → must commit.
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 190, V: 13}}
	w.nextID = 2

	if _, isRed := w.stopDistanceForRed(&w.Vehicles[0]); isRed {
		t.Errorf("yellow + 10m at 13 m/s: expected commit, got virtual stop leader")
	}
}

// TestWorld_YellowStopsWhenComfortable: a vehicle with comfortable
// stopping distance available should treat yellow as red and stop.
func TestWorld_YellowStopsWhenComfortable(t *testing.T) {
	net := buildSignalApproach(13)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3}},
	})
	w.SignalStates[0].IsYellow = true

	// 100m from line at 13 m/s. Comfortable stop ≈ 58m, so 100m > 58m → stop.
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 100, V: 13}}
	w.nextID = 2

	d, isRed := w.stopDistanceForRed(&w.Vehicles[0])
	if !isRed {
		t.Errorf("yellow + 100m at 13 m/s: expected stop, got commit")
	}
	if d != 100 {
		t.Errorf("yellow stop distance: want 100 (edge 200 - S 100), got %.2f", d)
	}
}

// TestWorld_PureGreenDoesNotStop: green (with IsYellow=false) must not
// apply a virtual leader. Regression guard for the soft-red split.
func TestWorld_PureGreenDoesNotStop(t *testing.T) {
	net := buildSignalApproach(13)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3}},
	})
	// IsYellow defaults to false → pure green for this approach.

	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 50, V: 13}}
	w.nextID = 2

	if _, isRed := w.stopDistanceForRed(&w.Vehicles[0]); isRed {
		t.Errorf("pure green: expected no virtual leader, got isRed=true")
	}
}

// TestWorld_YellowStopsWhenStandingStill: a vehicle already stopped at
// the line during yellow must remain stopped (comfortable stop distance
// from v=0 is just S0=2m, so any dist > 2m → stop; at dist≈0 the
// vehicle still doesn't move because V=0 and there's no leader pull).
// Confirms the soft-red check doesn't accidentally launch queued cars.
func TestWorld_YellowStopsWhenStandingStill(t *testing.T) {
	net := buildSignalApproach(13)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3}},
	})
	w.SignalStates[0].IsYellow = true

	// Stopped 20m from the line. Comfortable stop from V=0 is S0=2m, so
	// 20m > 2m → stop holds.
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 180, V: 0}}
	w.nextID = 2

	if _, isRed := w.stopDistanceForRed(&w.Vehicles[0]); !isRed {
		t.Errorf("stopped at yellow with 20m clearance: expected stop holds, got commit")
	}
}

func TestWorld_DeterminismSameSeed(t *testing.T) {
	run := func() (uint32, int) {
		net := build2x2Grid()
		w := NewWorld(net, NewRandomOD(net, 123, 3.0), nil)
		w.Run(5.0)
		return uint32(w.nextID), len(w.Vehicles)
	}
	a1, _ := run()
	a2, _ := run()
	if a1 != a2 {
		t.Errorf("determinism: same seed produced different nextID: %d vs %d", a1, a2)
	}
}

// TestWorld_SnapshotEmitsSignalPerApproach: a signalized intersection
// with N incoming approaches must produce N SignalViews in the snapshot,
// not one combined view. Guards against regressing back to per-intersection
// signal rendering.
func TestWorld_SnapshotEmitsSignalPerApproach(t *testing.T) {
	// Build a 4-way intersection: a center node with 4 incoming edges
	// from N, E, S, W (and 4 outgoing back to those nodes — needed so
	// the intersection has degree-8 in the graph).
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},   // N
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},   // E
		{ID: 2, Pos: network.Point{X: 0, Y: -100}},  // S
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},  // W
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},     // center
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID:         network.EdgeID(id),
			From:       network.NodeID(from),
			To:         network.NodeID(to),
			Length:     100,
			SpeedLimit: 10,
			Lanes:      []network.Lane{{Index: 0}},
			Geometry: []network.Point{
				nodes[from].Pos,
				nodes[to].Pos,
			},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 4), mkEdge(1, 4, 0), // N <-> center
		mkEdge(2, 1, 4), mkEdge(3, 4, 1), // E
		mkEdge(4, 2, 4), mkEdge(5, 4, 2), // S
		mkEdge(6, 3, 4), mkEdge(7, 4, 3), // W
	}
	xs := []network.Intersection{
		{
			ID:        0,
			NodeID:    4,
			Incoming:  []network.EdgeID{0, 2, 4, 6},
			Outgoing:  []network.EdgeID{1, 3, 5, 7},
			HasSignal: true,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Step() // one tick to populate the snapshot

	snap := w.SnapshotBuf.Read()
	if len(snap.Signals) != 4 {
		t.Fatalf("want 4 SignalViews (one per incoming approach), got %d", len(snap.Signals))
	}

	// Default plan groups approaches by arrival-heading axis. For this
	// orthogonal 4-way, expect exactly 2 green + 2 red.
	greens, reds := 0, 0
	for _, sv := range snap.Signals {
		if sv.IsRed {
			reds++
		} else {
			greens++
		}
	}
	if greens != 2 || reds != 2 {
		t.Errorf("default 2-phase plan: want 2 green + 2 red approaches in phase 0, got %d green / %d red", greens, reds)
	}
}

// TestDefaultSignalConfig_PairsOpposingApproaches: opposing legs of a
// 4-way intersection (N+S together, E+W together) must end up in the
// same phase. This is the user-visible "real signals work this way"
// property. Guards against accidentally regressing to index-parity grouping.
func TestDefaultSignalConfig_PairsOpposingApproaches(t *testing.T) {
	// Build the same 4-way geometry as the test above, then inspect the
	// generated default plan directly.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},  // N
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},  // E
		{ID: 2, Pos: network.Point{X: 0, Y: -100}}, // S
		{ID: 3, Pos: network.Point{X: -100, Y: 0}}, // W
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},    // center
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	// Use sequential IDs so Edges[i].ID == EdgeID(i) — this invariant
	// is what netbuild produces and what arrivalHeading/PositionOnEdge
	// rely on for O(1) lookup.
	net := &network.Network{
		Nodes: nodes,
		Edges: []network.Edge{
			mkEdge(0, 0, 4), // N -> C (southbound on arrival)
			mkEdge(1, 1, 4), // E -> C (westbound on arrival)
			mkEdge(2, 2, 4), // S -> C (northbound on arrival)
			mkEdge(3, 3, 4), // W -> C (eastbound on arrival)
		},
	}
	incoming := []network.EdgeID{0, 1, 2, 3} // [N, E, S, W] positions

	cfg := DefaultSignalConfig(incoming, net)
	if len(cfg.Phases) != 2 {
		t.Fatalf("want 2 phases for orthogonal 4-way, got %d", len(cfg.Phases))
	}

	// Find which phase contains position 0 (N) and assert position 2 (S)
	// is in the SAME phase. Then position 1 (E) and 3 (W) must be in the
	// OTHER phase.
	phaseOf := func(pos int) int {
		for i, p := range cfg.Phases {
			for _, gp := range p.GreenEdges {
				if gp == pos {
					return i
				}
			}
		}
		return -1
	}
	pN, pE, pS, pW := phaseOf(0), phaseOf(1), phaseOf(2), phaseOf(3)
	if pN < 0 || pE < 0 || pS < 0 || pW < 0 {
		t.Fatalf("every approach must belong to a phase; got N=%d E=%d S=%d W=%d", pN, pE, pS, pW)
	}
	if pN != pS {
		t.Errorf("N and S must share a phase, got N=phase%d S=phase%d", pN, pS)
	}
	if pE != pW {
		t.Errorf("E and W must share a phase, got E=phase%d W=phase%d", pE, pW)
	}
	if pN == pE {
		t.Errorf("NS pair must be in a different phase from EW pair; both ended up in phase %d", pN)
	}
}

// TestWorld_FlashRedYields: a vehicle approaching a flash-red signal
// with no crossing traffic should NOT hard-stop (it proceeds). A vehicle
// approaching the same intersection while another vehicle is bearing
// down the flash-yellow axis MUST stop near the line.
func TestWorld_FlashRedYields(t *testing.T) {
	// 4-way intersection identical to TestWorld_SnapshotEmitsSignalPerApproach.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: -100}},
		{ID: 3, Pos: network.Point{X: -100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},
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
		mkEdge(0, 0, 4), // N -> C
		mkEdge(1, 1, 4), // E -> C
		mkEdge(2, 2, 4), // S -> C
		mkEdge(3, 3, 4), // W -> C
		mkEdge(4, 4, 0), mkEdge(5, 4, 1), mkEdge(6, 4, 2), mkEdge(7, 4, 3), // outbound (unused but needed for compaction)
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 4,
			Incoming:  []network.EdgeID{0, 1, 2, 3},
			Outgoing:  []network.EdgeID{4, 5, 6, 7},
			HasSignal: true,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	// ModeFlashA: phase 0 = positions {0, 2} (N, S) = yellow priority;
	// positions {1, 3} (E, W) = red yield.
	cfg := DefaultSignalConfig(xs[0].Incoming, net)

	// --- Sub-test 1: vehicle on red approach proceeds when no crossing traffic.
	{
		w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
		w.SignalStates[0] = NewSignalState(cfg)
		// FlashB so phase-1 (NS by heading bucketing) is the yellow
		// priority axis and E/W must yield.
		w.SignalStates[0].Mode = ModeFlashB
		// One vehicle on E approach (flash-red), moving at speed limit.
		w.Vehicles = []Vehicle{
			{ID: 1, Route: []network.EdgeID{1, 7}, Edge: 1, S: 80, V: 10}, // E -> W straight-through; no corner cap
		}
		w.nextID = 2
		// With stuckSpeedThresh (0.1 m/s), the vehicle decelerates to ~0.1 m/s
		// at ~6s (tick ~120), dwells 0.5s, then gap-acceptance clears (no cross-
		// traffic) and the vehicle re-accelerates. It crosses to edge 7 by ~10s.
		// 250 ticks (12.5s) is ample.
		for i := 0; i < 250; i++ {
			w.Step()
			if len(w.Vehicles) == 0 {
				break
			}
			if w.Vehicles[0].Edge != 1 {
				break // advanced past the stop line
			}
		}
		// Should have advanced past the stop line (S would have rolled
		// over to the next edge in the route) or despawned.
		alive := 0
		for _, v := range w.Vehicles {
			if !v.Despawned {
				alive++
			}
		}
		// Vehicle either despawned or moved to outbound edge.
		if alive == 1 && w.Vehicles[0].Edge == 1 && w.Vehicles[0].V < 1.0 {
			t.Errorf("flash-red with no crossing traffic should proceed, but vehicle is stuck at S=%.1f V=%.1f", w.Vehicles[0].S, w.Vehicles[0].V)
		}
	}

	// --- Sub-test 2: vehicle on red approach waits when crossing traffic approaches.
	{
		w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
		w.SignalStates[0] = NewSignalState(cfg)
		// FlashB so phase-1 (NS by heading bucketing) is the yellow
		// priority axis and E/W must yield.
		w.SignalStates[0].Mode = ModeFlashB
		// E approach vehicle close to stop line + N approach vehicle
		// approaching with ETA ≈ 2s (well inside the 3s gap threshold).
		w.Vehicles = []Vehicle{
			{ID: 1, Route: []network.EdgeID{1, 7}, Edge: 1, S: 70, V: 10}, // E -> W straight (red, no corner cap)
			{ID: 2, Route: []network.EdgeID{0, 6}, Edge: 0, S: 80, V: 10}, // N -> S straight (yellow priority, ETA=2s)
		}
		w.nextID = 3
		// Tick twice — long enough for yield logic to kick in, short
		// enough that the priority vehicle hasn't cleared yet.
		for i := 0; i < 5; i++ {
			w.Step()
		}
		// Find the red-approach vehicle by ID.
		var red *Vehicle
		for i := range w.Vehicles {
			if w.Vehicles[i].ID == 1 {
				red = &w.Vehicles[i]
			}
		}
		if red == nil {
			t.Fatal("lost the flash-red vehicle")
		}
		if red.Edge != 1 {
			t.Errorf("red vehicle should still be on approach edge 1, got edge %d", red.Edge)
		}
		// Should be decelerating because of crossing traffic.
		if red.A >= 0 {
			t.Errorf("flash-red vehicle with priority traffic ETA<gap should be braking, A=%.2f", red.A)
		}
	}
}

// TestWorld_ControlChannelTogglesMode: a ControlEvent pushed through the
// control channel must change the signal's Mode by the next Step().
func TestWorld_ControlChannelTogglesMode(t *testing.T) {
	// Build a single-edge graph ending at a signalized intersection.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
	}
	edges := []network.Edge{
		{
			ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{{X: 0, Y: 0}, {X: 100, Y: 0}},
		},
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	ch := make(chan ControlEvent, 4)
	w.Control = ch

	// Initial mode: Normal.
	if w.SignalStates[0].Mode != ModeNormal {
		t.Fatalf("initial mode should be Normal, got %v", w.SignalStates[0].Mode)
	}

	// Push a Flash A event; one Step should apply it.
	ch <- ControlEvent{IntersectionID: 0, Mode: ModeFlashA}
	w.Step()
	if w.SignalStates[0].Mode != ModeFlashA {
		t.Errorf("after control event, mode should be FlashA, got %v", w.SignalStates[0].Mode)
	}

	// Push Off, then Normal — both should apply within Step's drain loop.
	ch <- ControlEvent{IntersectionID: 0, Mode: ModeOff}
	ch <- ControlEvent{IntersectionID: 0, Mode: ModeNormal}
	w.Step()
	if w.SignalStates[0].Mode != ModeNormal {
		t.Errorf("after two control events, final mode should be Normal, got %v", w.SignalStates[0].Mode)
	}
	// Returning to Normal must reset phase progression (Elapsed gets
	// one tick added by the subsequent Advance call, so check phase only).
	if w.SignalStates[0].PhaseIdx != 0 || w.SignalStates[0].IsYellow {
		t.Errorf("Normal reset incomplete: phase=%d yellow=%v",
			w.SignalStates[0].PhaseIdx, w.SignalStates[0].IsYellow)
	}
}

func TestWorld_TraceDeterminism(t *testing.T) {
	run := func() []byte {
		net := build2x2Grid()
		w := NewWorld(net, NewRandomOD(net, 9001, 5.0), nil)
		var buf bytes.Buffer
		tw := trace.NewWriter(&buf)
		w.EmitTrace = func(tick uint64, simTime float64, e trace.Event) {
			_ = tw.Write(tick, simTime, e)
		}
		w.Run(3.0)
		_ = tw.Close()
		return buf.Bytes()
	}
	a, b := run(), run()
	if !bytes.Equal(a, b) {
		t.Fatalf("trace bytes differ across runs with same seed (len %d vs %d)", len(a), len(b))
	}
}

// TestWorld_StuckVehicleDespawned: a vehicle below the stuck speed threshold
// for >60 sim-seconds on an edge with no red light and no yield must be
// logged at WARN level and despawned.
func TestWorld_StuckVehicleDespawned(t *testing.T) {
	// Single 200m edge, no intersection at the end. With no intersection,
	// stopDistanceForRed and stopDistanceForYield both return false, so the
	// stuck condition can trigger.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{{Index: 0}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 10, V: 0}}
	w.nextID = 2

	// Capture WARN logs via slog handler swap.
	var logBuf bytes.Buffer
	prev := slog.Default()
	slog.SetDefault(slog.New(slog.NewJSONHandler(&logBuf, &slog.HandlerOptions{Level: slog.LevelWarn})))
	defer slog.SetDefault(prev)

	// Pin V=0 before each tick. After stepIDM the vehicle's V will be
	// ~0.05 (one tick of free-acceleration from 0), still below the 0.1
	// threshold, so StuckTime accumulates dt per tick. >1200 ticks = >60
	// sim-seconds → despawn.
	for i := 0; i < 1500; i++ {
		if len(w.Vehicles) > 0 && !w.Vehicles[0].Despawned {
			w.Vehicles[0].V = 0
		}
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
	}

	if len(w.Vehicles) != 0 {
		t.Fatalf("stuck vehicle should have been despawned, %d still alive", len(w.Vehicles))
	}
	if !strings.Contains(logBuf.String(), "stuck vehicle despawned") {
		t.Errorf("expected WARN log containing 'stuck vehicle despawned', got: %q", logBuf.String())
	}
}

// TestWorld_StuckAtRedNotDespawned: a vehicle stopped at a red light for
// longer than the stuck timeout must NOT be despawned, because
// stopDistanceForRed returning true is the "legitimately stopped" branch.
func TestWorld_StuckAtRedNotDespawned(t *testing.T) {
	// Same setup as TestWorld_StopsAtRedLight: single edge ending in a
	// signalized intersection forced all-red.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{{Index: 0}}},
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Force the signal all-red by giving it an empty-green phase that
	// outlasts the run.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: nil, GreenDur: 1000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 50, V: 10}}
	w.nextID = 2

	// 1500 ticks = 75 sim-seconds, well past the 60-second stuck timeout.
	for i := 0; i < 1500; i++ {
		w.Step()
	}

	if len(w.Vehicles) != 1 {
		t.Fatalf("vehicle stopped at red should not be despawned, got %d alive", len(w.Vehicles))
	}
	v := &w.Vehicles[0]
	if v.StuckTime != 0 {
		t.Errorf("StuckTime should be 0 for a vehicle legitimately stopped at red, got %.3f", v.StuckTime)
	}
}

// TestWorld_StuckAtYieldNotDespawned: a vehicle correctly yielding at an
// unsignalized intersection must NOT be despawned, because
// stopDistanceForYield returning true is the "legitimately stopped" branch.
func TestWorld_StuckAtYieldNotDespawned(t *testing.T) {
	// Two incoming edges into an unsignalized intersection. Incoming[0]
	// (priority road) and Incoming[1] (yield road).
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},  // W (priority origin)
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},  // S (yield origin)
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},     // center
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},   // E (downstream of priority)
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
		mkEdge(0, 0, 2), // priority approach: W -> C
		mkEdge(1, 1, 2), // yield approach:   S -> C
		mkEdge(2, 2, 3), // outbound:        C -> E (route exit for both)
	}
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
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Priority vehicle pinned at S=99 (1 m from intersection) with V=2.0 m/s
	// → ETA = 0.5 s, which is below minAcceptedGap (1.5 s). Impatience can
	// never shrink effectiveGap below 1.5 s, so the yield vehicle must wait
	// indefinitely and must not be despawned by the stuck-vehicle guard.
	// Yield vehicle approaching its own stop line.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 2.0}, // priority, ~1m out @ 2.0 m/s = 0.5s ETA
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 50, V: 10},  // yield, approaching
	}
	w.nextID = 3

	// Pin the priority vehicle's S and V each tick to keep the yield
	// continuously active. Without pinning, the priority vehicle would
	// clear the intersection within a tick or two.
	for i := 0; i < 1500; i++ {
		// Find the priority vehicle by ID and re-pin its state.
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 99
				w.Vehicles[j].V = 2.0
			}
		}
		w.Step()
	}

	// Find the yield vehicle by ID.
	var yielder *Vehicle
	for i := range w.Vehicles {
		if w.Vehicles[i].ID == 2 {
			yielder = &w.Vehicles[i]
		}
	}
	if yielder == nil {
		t.Fatal("yield vehicle (ID=2) was unexpectedly despawned")
	}
	if yielder.Edge != 1 {
		t.Errorf("yield vehicle should still be on approach edge 1, got edge %d", yielder.Edge)
	}
	if yielder.StuckTime != 0 {
		t.Errorf("StuckTime should be 0 for a vehicle legitimately yielding, got %.3f", yielder.StuckTime)
	}
}

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
	// proceed. dt=0.05 (20 Hz), so 300 ticks = 15s of sim time.
	// With stuckSpeedThresh (0.1 m/s), V reaches ~0.1 at ~6s (tick ~120),
	// dwell completes at ~6.5s, and the vehicle crosses to edge 1 by ~10.5s.
	stoppedAt := -1.0
	departedAt := -1.0
	lastEdge := w.Vehicles[0].Edge
	for i := 0; i < 300; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
		v := &w.Vehicles[0]
		if v.Despawned {
			break
		}
		lastEdge = v.Edge
		if stoppedAt < 0 && v.StoppedSinceSec > 0 {
			stoppedAt = w.SimTime
		}
		if departedAt < 0 && v.Edge == 1 {
			departedAt = w.SimTime
		}
	}

	if stoppedAt < 0 {
		t.Fatal("vehicle never registered a mandatory stop at the stop line")
	}
	// After stopping, the vehicle must dwell at least stopDwellSec before
	// it gets to depart. lastEdge tracks the outbound edge the vehicle
	// advanced to (or the vehicle may have completed the route and been
	// compacted, which also means it successfully traversed the intersection).
	if lastEdge != 1 {
		t.Errorf("vehicle should have advanced to outbound edge (1) after dwell, last seen on edge %d", lastEdge)
	}
	if departedAt < 0 {
		t.Fatal("vehicle never crossed to outbound edge")
	}
	if departedAt-stoppedAt < stopDwellSec-w.dt-1e-9 {
		t.Errorf("vehicle departed only %.3fs after stopping; want >= %.3fs dwell",
			departedAt-stoppedAt, stopDwellSec)
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

	// 150 ticks = 7.5s: ample time for the vehicle (S=50, V=8, SpeedLimit=10)
	// to free-cruise ~50m to the edge end and transition to edge 1.
	for i := 0; i < 150; i++ {
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

	sawOutbound := false
	for i := 0; i < 500; i++ {
		w.Step()
		if len(w.Vehicles) == 0 || w.Vehicles[0].Despawned {
			break
		}
		if w.Vehicles[0].Edge == 1 {
			sawOutbound = true
			if w.Vehicles[0].StoppedSinceSec != 0 {
				t.Fatalf("StoppedSinceSec should be 0 after edge transition, got %.3f", w.Vehicles[0].StoppedSinceSec)
			}
		}
	}

	// Guard against the degenerate "vehicle deadlocked at the line" pass
	// mode: the in-loop invariant only fires once Edge advances. Require
	// either an observed outbound transition or a clean despawn.
	if !sawOutbound && len(w.Vehicles) > 0 && !w.Vehicles[0].Despawned {
		t.Errorf("vehicle never crossed the AllWayStop; in-loop invariant was not exercised")
	}
}

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
	// gapThresholdSec). Stop vehicle approaching its line (starts at S=50, V=8).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 50, V: 8},
	}
	w.nextID = 3

	for i := 0; i < 500; i++ {
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
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},  // N origin (A starts here)
		{ID: 1, Pos: network.Point{X: 0, Y: -100}}, // S origin (B starts here)
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},    // center
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A turns left here — southbound left = east)
		{ID: 4, Pos: network.Point{X: 0, Y: -200}}, // S destination (unused)
		{ID: 5, Pos: network.Point{X: 0, Y: 200}},  // N destination (B continues straight here)
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
		mkEdge(0, 0, 2), // N->C   (A's approach, vehicle heading south)
		mkEdge(1, 1, 2), // S->C   (B's approach, opposing A, vehicle heading north)
		mkEdge(2, 2, 3), // C->E   (A turns left here — southbound left is east)
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
	// A approaches at V=5 from S=80 (going to turn left).
	// B pinned at S=98, V=0.5 → ETA = 2/0.5 = 4s, inside leftTurnGapSec=6s.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 80, V: 5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5},
	}
	w.nextID = 3

	for i := 0; i < 300; i++ {
		// Re-pin B so the imminent-ETA condition persists.
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
	for i := 0; i < 500; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
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
	if len(w.Vehicles) > 0 && !w.Vehicles[0].Despawned && w.Vehicles[0].Edge != 4 {
		t.Errorf("vehicle should have cleared the intersection after dwell, still on edge %d", w.Vehicles[0].Edge)
	}
}

// TestWorld_LeftTurn_StuckGuardBypassed: a left turner waiting on
// perpetual opposing traffic must not be despawned by the 60s
// stuck-vehicle guard.
func TestWorld_LeftTurn_StuckGuardBypassed(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left from southbound)
		{ID: 4, Pos: network.Point{X: 0, Y: -200}}, // unused
		{ID: 5, Pos: network.Point{X: 0, Y: 200}},  // N destination (B's straight)
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
		mkEdge(0, 0, 2), // N->C (A's approach)
		mkEdge(1, 1, 2), // S->C (B's approach)
		mkEdge(2, 2, 3), // C->E (A's left turn)
		mkEdge(3, 2, 5), // C->N (B's through)
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
	// Vehicle B pinned at S=98 (2 m from intersection) with V=2.0 m/s
	// → ETA = 1.0 s, below minAcceptedGap (1.5 s). Impatience can never
	// shrink effectiveGap below 1.5 s, so vehicle A must wait indefinitely
	// and must not be despawned by the stuck-vehicle guard.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.5}, // A: left, near line
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 2.0}, // B: straight, pinned imminent (ETA=1s)
	}
	w.nextID = 3

	// Run for 130 sim-seconds (well past stuckTimeoutSec=60).
	for i := 0; i < 2600; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 2.0
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

// TestWorld_LeftTurn_PriorityRoad_NoOpposingTraffic: a priority-road
// left turner with no opposing vehicle sails through without recording
// a stop and without StuckTime accumulation.
func TestWorld_LeftTurn_PriorityRoad_NoOpposingTraffic(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},  // N origin (A starts here)
		{ID: 1, Pos: network.Point{X: 0, Y: -100}}, // S origin (unused B)
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},    // center
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left from southbound)
		{ID: 4, Pos: network.Point{X: 0, Y: -200}}, // unused
		{ID: 5, Pos: network.Point{X: 0, Y: 200}},  // unused
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
		mkEdge(0, 0, 2), // N->C
		mkEdge(1, 1, 2), // S->C (no vehicle here)
		mkEdge(2, 2, 3), // C->E (A's left)
		mkEdge(3, 2, 5), // C->N (unused outbound)
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
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 50, V: 8}, // A: left, no opposing
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
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},  // N origin (A)
		{ID: 1, Pos: network.Point{X: 0, Y: -100}}, // S origin (B)
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},    // center
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left from southbound)
		{ID: 4, Pos: network.Point{X: -100, Y: 0}}, // W destination (B's left from northbound)
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
		mkEdge(0, 0, 2), // N->C
		mkEdge(1, 1, 2), // S->C
		mkEdge(2, 2, 3), // C->E (A's left)
		mkEdge(3, 2, 4), // C->W (B's left)
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
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 50, V: 8}, // A: N->E (left)
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 50, V: 8}, // B: S->W (left)
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

// TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing: at a signaled
// intersection in normal green, a left turner with opposing through
// traffic must yield (permissive-left semantics).
func TestWorld_LeftTurn_SignaledGreen_YieldsToOpposing(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left)
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},  // N destination (B's through)
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
		mkEdge(2, 2, 3), // C->E (A's left turn)
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
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left)
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},  // N destination
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

// TestWorld_LeftTurn_AllWayStop_YieldsToOpposing: at an AllWayStop with
// two opposing approaches, the left turner (after dwell + FIFO clears)
// must yield to the opposing through.
func TestWorld_LeftTurn_AllWayStop_YieldsToOpposing(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}}, // E destination (A's left)
		{ID: 4, Pos: network.Point{X: 0, Y: 200}}, // N destination (B's through)
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
		mkEdge(2, 2, 3), // C->E (A's left turn)
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
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left from southbound)
		{ID: 4, Pos: network.Point{X: -100, Y: 0}}, // W destination (B's left from northbound)
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
		mkEdge(2, 2, 3), // C->E (A's left)
		mkEdge(3, 2, 4), // C->W (B's left)
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

	// Run simulation and track when both vehicles reach their outbound edges.
	aReachedOutbound, bReachedOutbound := false, false
	for i := 0; i < 400; i++ {
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 1 && v.Edge == 2 {
				aReachedOutbound = true
			}
			if v.ID == 2 && v.Edge == 3 {
				bReachedOutbound = true
			}
		}
	}

	if !aReachedOutbound {
		t.Error("AllWayStop left turner A should have made it through (mutual lefts pass)")
	}
	if !bReachedOutbound {
		t.Error("AllWayStop left turner B should have made it through (mutual lefts pass)")
	}
}

// TestWorld_LeftTurn_AllWayStop_CrossTrafficLeftDoesYield: at a 4-way
// AllWayStop, a left turner must NOT proceed simultaneously with a
// cross-traffic left turner — their paths cross. Mutual-left only
// applies to opposing approaches.
func TestWorld_LeftTurn_AllWayStop_CrossTrafficLeftDoesYield(t *testing.T) {
	// 4-way: N, E, S, W approaches all AllWayStop.
	// A: N approach turning left (heading east).
	// B: E approach turning left (heading south).
	// They are cross-traffic, NOT opposing. B paths through the center
	// going south while A paths through going east — they collide.
	// A is "lower position" so FIFO normally would let A go; the bug
	// would also let B go (because both are left-turners). With the fix,
	// B must yield to A.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},  // N
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},  // E
		{ID: 2, Pos: network.Point{X: 0, Y: -100}}, // S
		{ID: 3, Pos: network.Point{X: -100, Y: 0}}, // W
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},    // center
		{ID: 5, Pos: network.Point{X: 200, Y: 0}},  // A's exit east
		{ID: 6, Pos: network.Point{X: 0, Y: -200}}, // B's exit south
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
		mkEdge(0, 0, 4), // N->C  (A's approach)
		mkEdge(1, 1, 4), // E->C  (B's approach)
		mkEdge(2, 2, 4), // S->C
		mkEdge(3, 3, 4), // W->C
		mkEdge(4, 4, 5), // C->E (A's left turn)
		mkEdge(5, 4, 6), // C->S (B's left turn)
	}
	// We need an Opposing relation that correctly pairs N with S and E with W.
	// In Incoming order [0:N, 1:E, 2:S, 3:W]:
	//   Opposing[0] = 2 (N opposes S)
	//   Opposing[1] = 3 (E opposes W)
	//   Opposing[2] = 0
	//   Opposing[3] = 1
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 4,
			Incoming: []network.EdgeID{0, 1, 2, 3},
			IncomingControl: ctrls(
				network.ControlAllWayStop, network.ControlAllWayStop,
				network.ControlAllWayStop, network.ControlAllWayStop,
			),
			Opposing:  []int8{2, 3, 0, 1},
			Outgoing:  []network.EdgeID{4, 5},
			HasSignal: false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 4}, Edge: 0, S: 98, V: 0.5}, // A: N->E left
		{ID: 2, Route: []network.EdgeID{1, 5}, Edge: 1, S: 98, V: 0.5}, // B: E->S left
	}
	w.nextID = 3

	// Stop both vehicles at the line simultaneously, then run.
	// With identical StoppedSinceSec, FIFO tie-break (lower Incoming
	// index wins) means A (position 0) goes first, B (position 1) yields.
	// The mutual-left bug would have let both go simultaneously.
	//
	// We detect the bug by checking whether B is on its outbound edge (5)
	// in the very first tick that A transitions onto its outbound edge (4).
	// With the bug, both cross in the same tick; with the fix, B stays on
	// edge 1 while A clears the intersection.
	aEnteredOutboundTick := -1
	bEnteredOutboundTick := -1
	for i := 0; i < 600; i++ {
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 1 && v.Edge == 4 && aEnteredOutboundTick < 0 {
				aEnteredOutboundTick = i
			}
			if v.ID == 2 && v.Edge == 5 && bEnteredOutboundTick < 0 {
				bEnteredOutboundTick = i
			}
		}
		if aEnteredOutboundTick >= 0 && bEnteredOutboundTick >= 0 {
			break // both recorded; no need to run further
		}
	}

	// B (cross-traffic left) must NOT enter its outbound edge in the same
	// tick as A. With the fix, B yields until A has moved clear.
	if aEnteredOutboundTick < 0 {
		t.Fatal("vehicle A never reached its outbound edge")
	}
	if bEnteredOutboundTick < 0 {
		t.Fatal("vehicle B never reached its outbound edge")
	}
	if bEnteredOutboundTick == aEnteredOutboundTick {
		t.Errorf("cross-traffic left turner B (tick %d) entered outbound edge simultaneously with A (tick %d); mutual-left must not apply to cross-traffic", bEnteredOutboundTick, aEnteredOutboundTick)
	}
}

// TestWorld_GapFactor_Heterogeneous: spawn 200 vehicles via trySpawn
// with a fixed-seed world. Verify GapFactor distribution: mean within
// [0.95, 1.05], std within [0.07, 0.13], all values within [0.8, 1.2].
// Determinism: same seed → bit-identical values.
func TestWorld_GapFactor_Heterogeneous(t *testing.T) {
	net := build2x2Grid()
	w := NewWorld(net, NewRandomOD(net, 7, 100), nil) // high rate so spawns succeed

	// Drive 200 spawns by repeatedly calling trySpawn with synthetic
	// requests. Use a fixed seed via the world's existing rng.
	factors := make([]float64, 0, 200)
	for i := 0; i < 1000 && len(factors) < 200; i++ {
		w.trySpawn(SpawnRequest{OriginNode: 0, DestNode: 3})
		if len(w.Vehicles) > len(factors) {
			factors = append(factors, w.Vehicles[len(factors)].GapFactor)
		}
	}
	if len(factors) < 200 {
		t.Fatalf("could not collect 200 spawned vehicles, got %d", len(factors))
	}

	// Bounds.
	for i, f := range factors {
		if f < 0.8 || f > 1.2 {
			t.Errorf("factor[%d] = %f, out of [0.8, 1.2]", i, f)
		}
	}

	// Mean.
	sum := 0.0
	for _, f := range factors {
		sum += f
	}
	mean := sum / float64(len(factors))
	if mean < 0.95 || mean > 1.05 {
		t.Errorf("mean = %f, want in [0.95, 1.05]", mean)
	}

	// Std.
	varSum := 0.0
	for _, f := range factors {
		varSum += (f - mean) * (f - mean)
	}
	std := math.Sqrt(varSum / float64(len(factors)))
	if std < 0.07 || std > 0.13 {
		t.Errorf("std = %f, want in [0.07, 0.13]", std)
	}
}

// TestWorld_Impatience_StraightCrossingShrinksGap: a yield-controlled
// vehicle facing perpetual cross-traffic with ETA=2.5s yields at t=0
// (base gap 3s > 2.5s ETA → yield). After ~5s wait, effectiveGap
// drops to 2.5s and vehicle accepts the gap. Test verifies the wait
// duration and the eventual departure.
func TestWorld_Impatience_StraightCrossingShrinksGap(t *testing.T) {
	// Same fixture as TestWorld_StuckAtYieldNotDespawned: 4-way with W
	// priority and S yield. Vehicle on S yields; vehicle on W pinned
	// at perpetual ETA=2.5s.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
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
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlYield),
			Outgoing:        []network.EdgeID{2},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Priority at S=98, V=0.8 → d=2m, ETA=2.5s (above floor 1.5s,
	// below base gap 3s).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.8},
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 99, V: 0.5}, // yield vehicle, near line
	}
	w.nextID = 3

	// Yield vehicle starts close to the line so it stops quickly. Pin
	// priority each tick.
	var crossedAt float64 = -1.0
	for i := 0; i < 600; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.8
			}
		}
		w.Step()
		// Detect when the yield vehicle crosses into outbound.
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 2 && !v.Despawned && v.Edge == 2 && crossedAt < 0 {
				crossedAt = w.SimTime
			}
		}
		if crossedAt > 0 {
			break
		}
	}

	if crossedAt < 0 {
		t.Fatal("yield vehicle never crossed; impatience never reduced gap below ETA")
	}
	t.Logf("yield vehicle (ID=2) transitioned to Edge 2 at sim-time %.2f s", crossedAt)
	// Predicted wait: gap needs to drop from 3.0 to 2.5 = 0.5s reduction.
	// At decay 0.1 s/s, that's 5s of wait. Plus a few seconds of approach
	// + stop. Expect crossing somewhere in [5, 15] sim-seconds.
	if crossedAt < 4 || crossedAt > 20 {
		t.Errorf("expected crossing in [4, 20] sim-seconds, got %.2f", crossedAt)
	}
}

// TestWorld_Impatience_LeftTurnShrinksGap: priority-road left turner
// with perpetual opposing through at ETA=3.5s. Base 6s × 1.0 = 6s.
// To drop to 3.5s: 6 - 0.1*t = 3.5 → t = 25s. Vehicle waits ~25s
// then accepts.
func TestWorld_Impatience_LeftTurnShrinksGap(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},  // E destination (A's left)
		{ID: 4, Pos: network.Point{X: 0, Y: 200}},  // N destination (B's through)
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
		mkEdge(2, 2, 3), // C->E (A's left turn)
		mkEdge(3, 2, 4), // C->N (B's through)
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
	// A: left turner, near line. B: opposing through, pinned at ETA=3.5s
	// (d=2m, V=2/3.5 ≈ 0.571 m/s).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 0.5714},
	}
	w.nextID = 3

	var crossedAt float64 = -1.0
	for i := 0; i < 1200; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.5714
			}
		}
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 1 && !v.Despawned && v.Edge == 2 && crossedAt < 0 {
				crossedAt = w.SimTime
			}
		}
		if crossedAt > 0 {
			break
		}
	}

	if crossedAt < 0 {
		t.Fatal("left turner never crossed; impatience never reduced gap below ETA")
	}
	// Predicted wait: gap drops from 6 to 3.5 = 2.5s reduction → 25s of
	// wait. Plus a few seconds of approach. Expect crossing in [20, 40]
	// sim-seconds.
	if crossedAt < 20 || crossedAt > 40 {
		t.Errorf("expected crossing in [20, 40] sim-seconds, got %.2f", crossedAt)
	}
}

// TestWorld_Impatience_FloorPreventsCollision: oncoming ETA = 1.0s
// (below minAcceptedGap=1.5s). Vehicle waits forever — the floor
// prevents impatience from producing collision-imminent crossings.
func TestWorld_Impatience_FloorPreventsCollision(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 100}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
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
			IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Opposing:        []int8{1, 0},
			Outgoing:        []network.EdgeID{2, 3},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// A: left turner. B: opposing, pinned at ETA=1.0s (d=2m, V=2 m/s).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 98, V: 2.0},
	}
	w.nextID = 3

	// 200s sim = 4000 ticks.
	for i := 0; i < 4000; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 2 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 2.0
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
		t.Errorf("left turner should still be on approach edge 0 (floor prevented unsafe crossing), got edge %d", a.Edge)
	}
	if a.StuckTime != 0 {
		t.Errorf("StuckTime must remain 0, got %.3f", a.StuckTime)
	}
	// WaitTime should be substantial (impatience accumulated) but vehicle
	// still hasn't crossed.
	if a.WaitTime < 100 {
		t.Errorf("WaitTime should reflect substantial wait, got %.3f", a.WaitTime)
	}
}

// TestWorld_Impatience_MonotonicWithinApproach: WaitTime is monotonic
// within an approach — it accumulates while slow-and-yielding and is
// preserved through brief windows of movement-but-on-same-edge. This
// is the post-bugfix semantic: edge-transition is the ONLY in-Step
// reset point. Verifies WaitTime does not reset mid-approach.
func TestWorld_Impatience_MonotonicWithinApproach(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
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
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlYield),
			Outgoing:        []network.EdgeID{2},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.8}, // priority, ETA=2.5s
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 99, V: 0.5},
	}
	w.nextID = 3

	prevWait := 0.0
	for i := 0; i < 300; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.8
			}
		}
		w.Step()
		// Check the yield vehicle's WaitTime monotonicity while it's on
		// the approach edge (edge 1). Once it transitions to edge 2,
		// WaitTime resets — that's the only allowed decrement.
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID != 2 || v.Despawned {
				continue
			}
			if v.Edge == 1 && v.WaitTime < prevWait-1e-9 {
				t.Errorf("WaitTime decreased on approach edge: %.3f -> %.3f (tick %d)",
					prevWait, v.WaitTime, i)
				return
			}
			if v.Edge == 1 {
				prevWait = v.WaitTime
			}
		}
	}
}

// TestWorld_Impatience_ResetsOnEdgeTransition: vehicle yields at
// intersection A, accumulates WaitTime, eventually crosses. WaitTime
// must be 0 the moment the vehicle reaches the outbound edge.
func TestWorld_Impatience_ResetsOnEdgeTransition(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
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
	}
	xs := []network.Intersection{
		{
			ID: 0, NodeID: 2,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(network.ControlNone, network.ControlYield),
			Outgoing:        []network.EdgeID{2},
			HasSignal:       false,
		},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 98, V: 0.8}, // priority, ETA=2.5s
		{ID: 2, Route: []network.EdgeID{1, 2}, Edge: 1, S: 99, V: 0.5},
	}
	w.nextID = 3

	// Pin priority; once yield vehicle accumulates WaitTime past ~5s,
	// impatience will drive it across. Once it transitions to edge 2,
	// WaitTime must be 0.
	for i := 0; i < 600; i++ {
		for j := range w.Vehicles {
			if w.Vehicles[j].ID == 1 && !w.Vehicles[j].Despawned {
				w.Vehicles[j].S = 98
				w.Vehicles[j].V = 0.8
			}
		}
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if v.ID == 2 && !v.Despawned && v.Edge == 2 {
				if v.WaitTime != 0 {
					t.Fatalf("WaitTime should be 0 after edge transition, got %.3f", v.WaitTime)
				}
				return
			}
		}
	}

	t.Fatal("yield vehicle never crossed; cannot verify edge-transition reset")
}

// TestWorld_Impatience_NotAppliedToRedLight: vehicle stopped at a red
// light for 60s. WaitTime must remain 0 throughout — red lights are
// hard stops, not gap-acceptance yields.
func TestWorld_Impatience_NotAppliedToRedLight(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{{Index: 0}}},
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	// Permanent all-red phase.
	w.SignalStates[0] = NewSignalState(SignalConfig{
		Phases: []SignalPhase{{GreenEdges: nil, GreenDur: 10000, YellowDur: 0}},
	})

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 50, V: 10},
	}
	w.nextID = 2

	// Run 1200 ticks = 60 sim-seconds.
	for i := 0; i < 1200; i++ {
		w.Step()
	}

	if len(w.Vehicles) != 1 {
		t.Fatalf("vehicle should be stopped at red, got %d alive", len(w.Vehicles))
	}
	v := &w.Vehicles[0]
	if v.V > 0.1 {
		t.Errorf("vehicle should be stopped at red, V=%.2f", v.V)
	}
	if v.WaitTime != 0 {
		t.Errorf("WaitTime should remain 0 at red light (not a gap-acceptance yield), got %.3f", v.WaitTime)
	}
}

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
			w.Vehicles[j].StuckTime = 0          // keep the jam alive regardless of stuck-despawn tuning
		}
		w.Step()
	}

	jammed := w.Cong.Cost(net, 0)
	if jammed <= free {
		t.Fatalf("jammed cost %v should exceed free-flow %v after a sustained stop", jammed, free)
	}
}

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
	// is cheaper but within switchMargin (22 > 24*0.85=20.4) -> no switch.
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
