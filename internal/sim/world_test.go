package sim

import (
	"bytes"
	"log/slog"
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
	// Priority vehicle parked close to the stop line, moving slowly enough
	// that its ETA to the intersection is well inside gapThresholdSec (3s).
	// Yield vehicle approaching its own stop line.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5}, // priority, ~1m out @ 0.5 m/s = 2s ETA
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
				w.Vehicles[j].V = 0.5
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
	// gapThresholdSec). Stop vehicle approaching its line (starts at S=85
	// so it only has 15m to decelerate, ensuring it hits the stop by tick 20).
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 99, V: 0.5},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 85, V: 8},
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
