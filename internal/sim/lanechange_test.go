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
