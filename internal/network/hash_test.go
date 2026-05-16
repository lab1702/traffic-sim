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

// TestHash_DistinguishesControls guards the trace-replay invariant: if
// per-approach IncomingControl, HasSignal, BannedTurns, or SpeedLimit
// differ between two networks, NetHash must too. Otherwise a recorded
// trace will silently misbehave on replay against a network with
// different controls.
func TestHash_DistinguishesControls(t *testing.T) {
	mk := func(ctrl Control, signal bool, speed float64, bans []TurnRestriction) *Network {
		return &Network{
			Nodes: []Node{{ID: 0}, {ID: 1}, {ID: 2}},
			Edges: []Edge{
				{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: speed, Lanes: []Lane{{}}},
				{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: speed, Lanes: []Lane{{}}},
			},
			Intersections: []Intersection{
				{
					ID: 0, NodeID: 1,
					Incoming:        []EdgeID{0},
					IncomingControl: []Control{ctrl},
					Outgoing:        []EdgeID{1},
					HasSignal:       signal,
					BannedTurns:     bans,
				},
			},
		}
	}
	base := mk(ControlNone, false, 10, nil)
	cases := []struct {
		name string
		net  *Network
	}{
		{"control flipped", mk(ControlStop, false, 10, nil)},
		{"signal flipped", mk(ControlNone, true, 10, nil)},
		{"speed limit changed", mk(ControlNone, false, 20, nil)},
		{"ban added", mk(ControlNone, false, 10, []TurnRestriction{{From: 0, To: 1}})},
	}
	baseH := Hash(base)
	for _, c := range cases {
		if Hash(c.net) == baseH {
			t.Errorf("Hash should differ when %s", c.name)
		}
	}
}

// TestHash_BannedTurnOrderIndependent: the order in which BannedTurns are
// inserted shouldn't affect the hash — Hash sorts them internally before
// folding into the digest. Otherwise a refactor that reorders BannedTurns
// would spuriously invalidate every existing trace.
func TestHash_BannedTurnOrderIndependent(t *testing.T) {
	mk := func(bans []TurnRestriction) *Network {
		return &Network{
			Nodes: []Node{{ID: 0}, {ID: 1}, {ID: 2}},
			Edges: []Edge{
				{From: 0, To: 1, Length: 100, Lanes: []Lane{{}}},
				{From: 1, To: 2, Length: 100, Lanes: []Lane{{}}},
				{From: 2, To: 1, Length: 100, Lanes: []Lane{{}}},
			},
			Intersections: []Intersection{
				{ID: 0, NodeID: 1, Incoming: []EdgeID{0, 2}, BannedTurns: bans},
			},
		}
	}
	a := mk([]TurnRestriction{{From: 0, To: 1}, {From: 2, To: 1}})
	b := mk([]TurnRestriction{{From: 2, To: 1}, {From: 0, To: 1}})
	if Hash(a) != Hash(b) {
		t.Error("Hash should be insensitive to BannedTurns insertion order")
	}
}
