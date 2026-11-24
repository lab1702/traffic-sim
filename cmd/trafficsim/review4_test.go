package main

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/config"
	"github.com/lab1702/traffic-sim/internal/network"
)

// Regression tests for the fourth full code review.

// TestApplyTurnRestrictions_PrunesLaneTurns: netbuild assigns each lane's
// AllowedTurns before --signals adds its bans. A right-turn-only lane whose
// right turn was then banned still allowed only that turn, so no route could
// use it and every car steered out of it: the approach lost a lane.
func TestApplyTurnRestrictions_PrunesLaneTurns(t *testing.T) {
	x0, s, n, e, w := network.Point{}, network.Point{Y: -100}, network.Point{Y: 100}, network.Point{X: 100}, network.Point{X: -100}
	mk := func(id, from, to int, a, b network.Point) network.Edge {
		return network.Edge{ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{a, b}}
	}
	// Northbound approach (edge 0) with lanes [right], [straight], [left].
	approach := mk(0, 1, 0, s, x0)
	approach.Lanes = []network.Lane{
		{Index: 0, AllowedTurns: []network.EdgeID{2}},
		{Index: 1, AllowedTurns: []network.EdgeID{1}},
		{Index: 2, AllowedTurns: []network.EdgeID{3}},
	}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: x0}, {ID: 1, Pos: s}, {ID: 2, Pos: n}, {ID: 3, Pos: e}, {ID: 4, Pos: w}},
		Edges: []network.Edge{approach, mk(1, 0, 2, x0, n), mk(2, 0, 3, x0, e), mk(3, 0, 4, x0, w)},
		Intersections: []network.Intersection{{
			ID: 0, NodeID: 0, Incoming: []network.EdgeID{0}, IncomingControl: []network.Control{0},
			Opposing: []int8{-1}, Outgoing: []network.EdgeID{1, 2, 3},
		}},
	}
	applyTurnRestrictions(net, []config.TurnRestrictionOverride{{IntersectionID: 0, Ban: []string{"right_turn"}}})

	lanes := net.Edges[0].Lanes
	for li, l := range lanes {
		for _, to := range l.AllowedTurns {
			if to == 2 {
				t.Errorf("lane %d still allows the banned right turn", li)
			}
		}
	}
	if got := lanes[0].AllowedTurns; len(got) != 1 || got[0] != 1 {
		t.Errorf("former right-only lane allows %v, want the straight-on of its neighbour [1]", got)
	}
	if got := lanes[2].AllowedTurns; len(got) != 1 || got[0] != 3 {
		t.Errorf("left lane changed to %v, want [3]", got)
	}
}
