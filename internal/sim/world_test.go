package sim

import (
	"bytes"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/trace"
)

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

	// With the DefaultSignalConfig 2-phase plan, even-indexed approaches
	// are green during phase 0; odd-indexed are red. Verify.
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
