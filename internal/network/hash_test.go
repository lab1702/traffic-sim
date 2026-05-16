package network

import "testing"

func TestHash_Stable(t *testing.T) {
	net := &Network{
		Nodes: []Node{
			{ID: 0, Pos: Point{X: 0, Y: 0}},
			{ID: 1, Pos: Point{X: 100, Y: 0}},
		},
		Edges: []Edge{
			{ID: 0, From: 0, To: 1, Length: 100, Lanes: []Lane{{Index: 0}}},
		},
		Intersections: []Intersection{
			{ID: 0, NodeID: 1, Incoming: []EdgeID{0}},
		},
	}
	h1 := Hash(net)
	h2 := Hash(net)
	if h1 != h2 {
		t.Errorf("Hash not stable: %x vs %x", h1, h2)
	}
	if h1 == 0 {
		t.Errorf("Hash should not be 0 for a non-empty network")
	}
}

func TestHash_DistinguishesTopology(t *testing.T) {
	base := &Network{
		Nodes: []Node{{ID: 0}, {ID: 1}},
		Edges: []Edge{{From: 0, To: 1, Length: 100, Lanes: []Lane{{}}}},
	}
	// Change edge length — hash should differ.
	mod := &Network{
		Nodes: []Node{{ID: 0}, {ID: 1}},
		Edges: []Edge{{From: 0, To: 1, Length: 200, Lanes: []Lane{{}}}},
	}
	if Hash(base) == Hash(mod) {
		t.Error("Hash should differ when edge length changes")
	}
	// Reverse direction.
	rev := &Network{
		Nodes: []Node{{ID: 0}, {ID: 1}},
		Edges: []Edge{{From: 1, To: 0, Length: 100, Lanes: []Lane{{}}}},
	}
	if Hash(base) == Hash(rev) {
		t.Error("Hash should differ when edge direction flips")
	}
}
