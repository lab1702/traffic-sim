package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// buildLineGraph: nodes 0-1-2-3, four directed edges (one per pair, in
// order). Edge i goes from node i to node i+1, length 100m each.
func buildLineGraph() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: 0}},
		{ID: 3, Pos: network.Point{X: 300, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10},
		{ID: 2, From: 2, To: 3, Length: 100, SpeedLimit: 10},
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestRouter_ShortestPath_LineGraph(t *testing.T) {
	net := buildLineGraph()
	r := NewRouter(net)
	route, err := r.Route(0, 3)
	if err != nil {
		t.Fatalf("Route: %v", err)
	}
	if len(route) != 3 {
		t.Fatalf("want 3 edges, got %d", len(route))
	}
	for i, eid := range []network.EdgeID{0, 1, 2} {
		if route[i] != eid {
			t.Errorf("step %d: want edge %d, got %d", i, eid, route[i])
		}
	}
}

func TestRouter_Unreachable(t *testing.T) {
	net := buildLineGraph()
	// Add an isolated node 4 with no edges.
	net.Nodes = append(net.Nodes, network.Node{ID: 4, Pos: network.Point{X: 1000, Y: 0}})
	r := NewRouter(net)
	_, err := r.Route(0, 4)
	if err == nil {
		t.Fatalf("want error for unreachable target")
	}
}

// TestRouter_RespectsBannedTurn: build a +shaped network where the
// direct E->W route through the center requires a banned (E->C, C->W)
// turn. The router must find a longer detour via N (E->C, C->N, N->C,
// C->W) since the via-N path exists.
//
//             N (5)
//             |
//        E -- C(4) -- W
//             |
//             S (banned-turn alternative if needed)
func TestRouter_RespectsBannedTurn(t *testing.T) {
	c := network.Point{X: 0, Y: 0}
	n := network.Point{X: 0, Y: 100}
	e := network.Point{X: 100, Y: 0}
	w := network.Point{X: -100, Y: 0}

	nodes := []network.Node{
		{ID: 0, Pos: e},
		{ID: 1, Pos: w},
		{ID: 2, Pos: n},
		{ID: 3, Pos: c},
	}
	// We need a return path C->N->C, so include both directions.
	edges := []network.Edge{
		{ID: 0, From: 0, To: 3, Length: 100, SpeedLimit: 10, Geometry: []network.Point{e, c}}, // E->C
		{ID: 1, From: 3, To: 1, Length: 100, SpeedLimit: 10, Geometry: []network.Point{c, w}}, // C->W (the banned target)
		{ID: 2, From: 3, To: 2, Length: 100, SpeedLimit: 10, Geometry: []network.Point{c, n}}, // C->N
		{ID: 3, From: 2, To: 3, Length: 100, SpeedLimit: 10, Geometry: []network.Point{n, c}}, // N->C
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 3, Incoming: []network.EdgeID{0, 3}, Outgoing: []network.EdgeID{1, 2},
			BannedTurns: []network.TurnRestriction{
				{From: 0, To: 1}, // ban E->W direct
			}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	r := NewRouter(net)
	route, err := r.Route(0 /*E*/, 1 /*W*/)
	if err != nil {
		t.Fatalf("Route: %v", err)
	}
	// With the ban in place the only path is E->C->N->C->W: edges [0, 2, 3, 1].
	want := []network.EdgeID{0, 2, 3, 1}
	if len(route) != len(want) {
		t.Fatalf("want detour %v (len %d), got %v (len %d)", want, len(want), route, len(route))
	}
	for i := range want {
		if route[i] != want[i] {
			t.Errorf("step %d: want edge %d, got %d", i, want[i], route[i])
		}
	}
}

// TestRouter_BannedTurnNoDetour: same ban, no alternate path. Returns
// ErrNoRoute rather than the banned route.
func TestRouter_BannedTurnNoDetour(t *testing.T) {
	c := network.Point{X: 0, Y: 0}
	e := network.Point{X: 100, Y: 0}
	w := network.Point{X: -100, Y: 0}

	nodes := []network.Node{
		{ID: 0, Pos: e},
		{ID: 1, Pos: w},
		{ID: 2, Pos: c},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 2, Length: 100, SpeedLimit: 10, Geometry: []network.Point{e, c}}, // E->C
		{ID: 1, From: 2, To: 1, Length: 100, SpeedLimit: 10, Geometry: []network.Point{c, w}}, // C->W
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 2, Incoming: []network.EdgeID{0}, Outgoing: []network.EdgeID{1},
			BannedTurns: []network.TurnRestriction{
				{From: 0, To: 1},
			}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}

	r := NewRouter(net)
	_, err := r.Route(0, 1)
	if err == nil {
		t.Fatalf("want ErrNoRoute (only path is banned), got nil")
	}
}
