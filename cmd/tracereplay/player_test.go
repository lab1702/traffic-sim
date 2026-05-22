package main

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// TestNetHashMismatch covers the SimStart check that warns when the
// loaded OSM doesn't match the original run's network. Zero traceHash
// is the legacy-trace short-circuit; non-zero values are compared.
func TestNetHashMismatch(t *testing.T) {
	// Tiny network — Hash() returns a deterministic non-zero value.
	net := &network.Network{
		Nodes:         []network.Node{{ID: 0}, {ID: 1}},
		Edges:         []network.Edge{{From: 0, To: 1, Length: 100, Lanes: []network.Lane{{}}}},
		Intersections: []network.Intersection{{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}}},
	}
	loaded := network.Hash(net)
	if loaded == 0 {
		t.Fatal("test setup: Hash returned 0 for a non-empty network")
	}

	t.Run("zero trace hash is silently skipped (legacy)", func(t *testing.T) {
		mismatch, _, _ := netHashMismatch(net, 0)
		if mismatch {
			t.Errorf("zero traceHash should NOT flag mismatch (legacy traces)")
		}
	})
	t.Run("matching hashes pass", func(t *testing.T) {
		mismatch, _, _ := netHashMismatch(net, loaded)
		if mismatch {
			t.Errorf("matching hashes should NOT flag mismatch")
		}
	})
	t.Run("differing hashes flag mismatch", func(t *testing.T) {
		mismatch, traceH, loadedH := netHashMismatch(net, loaded^0xDEADBEEF)
		if !mismatch {
			t.Errorf("differing hashes should flag mismatch")
		}
		if loadedH != loaded {
			t.Errorf("returned loadedHash = %x, want %x", loadedH, loaded)
		}
		if traceH != loaded^0xDEADBEEF {
			t.Errorf("returned traceHash = %x, want %x", traceH, loaded^0xDEADBEEF)
		}
	})
}

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

func TestPlayer_RerouteGuards(t *testing.T) {
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

	// AtIndex past the route end: no-op (and no panic).
	p.apply(hdr, &trace.VehicleReroute{VehicleID: 7, AtIndex: 99, NewTail: []uint32{2}})
	if got := p.vehicles[7].route; len(got) != 2 || got[0] != 0 || got[1] != 1 {
		t.Fatalf("out-of-bounds reroute changed route: %v", got)
	}

	// AtIndex behind the replay vehicle's progress: no-op, so curEdge stays
	// consistent with route[routeIdx].
	p.vehicles[7].routeIdx = 1
	p.apply(hdr, &trace.VehicleReroute{VehicleID: 7, AtIndex: 0, NewTail: []uint32{2, 3}})
	if got := p.vehicles[7].route; len(got) != 2 || got[0] != 0 || got[1] != 1 {
		t.Fatalf("reroute behind routeIdx changed route: %v", got)
	}

	// Unknown vehicle: no-op (and no panic).
	p.apply(hdr, &trace.VehicleReroute{VehicleID: 999, AtIndex: 0, NewTail: []uint32{1}})
}
