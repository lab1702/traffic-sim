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
