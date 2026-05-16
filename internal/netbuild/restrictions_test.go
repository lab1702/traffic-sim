package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/paulmach/osm"
)

// TestPickEdgeAt_PrefersForwardDirection: for a through-way where multiple
// candidate edges end at the via node, pickEdgeAt must prefer the edge in
// the way's forward direction (node order: A → X → B picks A→X, not B→X).
// This regression test feeds candidates in *reverse* slice order so the
// old "first match wins" behavior would pick the wrong edge.
//
// See review #4 (2026-05-15).
func TestPickEdgeAt_PrefersForwardDirection(t *testing.T) {
	// OSM way [10, 20, 30]: node 10 (way-index 0), node 20 (way-index 1, via),
	// node 30 (way-index 2).
	way := &osm.Way{
		ID: 100,
		Nodes: osm.WayNodes{
			{ID: 10}, {ID: 20}, {ID: 30},
		},
	}
	osmToNet := map[osm.NodeID]network.NodeID{10: 0, 20: 1, 30: 2}
	osmNodeOf := func(nid network.NodeID) (osm.NodeID, bool) {
		for o, n := range osmToNet {
			if n == nid {
				return o, true
			}
		}
		return 0, false
	}
	edges := []network.Edge{
		{ID: 0, From: 2, To: 1}, // network 2→1 = OSM 30→20 (reverse direction along way 100)
		{ID: 1, From: 0, To: 1}, // network 0→1 = OSM 10→20 (forward direction)
	}
	// Candidates fed in reverse order: pre-fix code would return edge 0.
	candidates := []network.EdgeID{0, 1}

	got, ok := pickEdgeAt(candidates, edges, network.NodeID(1) /*via*/, true /*endsAt*/, way, osm.NodeID(20), osmNodeOf)
	if !ok {
		t.Fatal("pickEdgeAt returned !ok")
	}
	if got != 1 {
		t.Errorf("want edge 1 (forward direction 10→20), got edge %d", got)
	}
}

// TestPickEdgeAt_PrefersForwardDirectionStartsAt: same disambiguation for
// the to-side (starts at via). Prefer the edge whose To-node has a higher
// way-index than via.
func TestPickEdgeAt_PrefersForwardDirectionStartsAt(t *testing.T) {
	way := &osm.Way{
		ID: 100,
		Nodes: osm.WayNodes{
			{ID: 10}, {ID: 20}, {ID: 30},
		},
	}
	osmToNet := map[osm.NodeID]network.NodeID{10: 0, 20: 1, 30: 2}
	osmNodeOf := func(nid network.NodeID) (osm.NodeID, bool) {
		for o, n := range osmToNet {
			if n == nid {
				return o, true
			}
		}
		return 0, false
	}
	edges := []network.Edge{
		{ID: 0, From: 1, To: 0}, // 20→10 (reverse direction)
		{ID: 1, From: 1, To: 2}, // 20→30 (forward direction)
	}
	candidates := []network.EdgeID{0, 1}

	got, ok := pickEdgeAt(candidates, edges, network.NodeID(1) /*via*/, false /*startsAt*/, way, osm.NodeID(20), osmNodeOf)
	if !ok {
		t.Fatal("pickEdgeAt returned !ok")
	}
	if got != 1 {
		t.Errorf("want edge 1 (forward direction 20→30), got edge %d", got)
	}
}

// TestPickEdgeAt_SingleCandidate falls through without consulting the way
// — there is nothing to disambiguate.
func TestPickEdgeAt_SingleCandidate(t *testing.T) {
	edges := []network.Edge{
		{ID: 0, From: 5, To: 1},
	}
	got, ok := pickEdgeAt([]network.EdgeID{0}, edges, network.NodeID(1), true, nil, 0, nil)
	if !ok || got != 0 {
		t.Errorf("single-candidate path: want (0, true), got (%d, %v)", got, ok)
	}
}

// TestPickEdgeAt_NoMatch returns (0, false).
func TestPickEdgeAt_NoMatch(t *testing.T) {
	edges := []network.Edge{
		{ID: 0, From: 5, To: 99}, // doesn't end at the via node
	}
	_, ok := pickEdgeAt([]network.EdgeID{0}, edges, network.NodeID(1), true, nil, 0, nil)
	if ok {
		t.Error("no-match path: want !ok")
	}
}
