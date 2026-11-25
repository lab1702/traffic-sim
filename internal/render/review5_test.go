package render

// Regression tests for the fifth code review. Each fails before its fix.

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

// TestIncidentRoadCount: one incident on a two-way road is set on both of its
// directed edges, and the HUD showed incidents=2 for it.
func TestIncidentRoadCount(t *testing.T) {
	a, b := network.Point{X: 0, Y: 0}, network.Point{X: 100, Y: 0}
	c := network.Point{X: 100, Y: 100}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: a}, {ID: 1, Pos: b}, {ID: 2, Pos: c}},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Geometry: []network.Point{a, b}}, // two-way road A-B
			{ID: 1, From: 1, To: 0, Geometry: []network.Point{b, a}},
			{ID: 2, From: 1, To: 2, Geometry: []network.Point{b, c}}, // one-way B->C
		},
	}
	rev := network.ReverseEdges(net)
	inc := func(eids ...uint32) []snapshot.IncidentView {
		var out []snapshot.IncidentView
		for _, e := range eids {
			out = append(out, snapshot.IncidentView{EdgeID: e, Severity: snapshot.SevFullClose})
		}
		return out
	}
	for _, tc := range []struct {
		incs []snapshot.IncidentView
		want int
	}{
		{nil, 0},
		{inc(1, 0), 1},    // both directions of A-B: one road
		{inc(0, 1, 2), 2}, // plus the one-way
		{inc(1), 1},       // one direction only (a trace from before twins)
		{append(inc(0), snapshot.IncidentView{EdgeID: 1, Severity: snapshot.SevNone}), 1},
	} {
		if got := incidentRoadCount(tc.incs, rev); got != tc.want {
			t.Errorf("incidentRoadCount(%v) = %d, want %d", tc.incs, got, tc.want)
		}
	}
}
