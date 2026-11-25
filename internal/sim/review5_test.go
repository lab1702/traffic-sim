package sim

// Regression tests for the fifth code review. Each fails before its fix.

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/netbuild"
	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// builtRoundabout builds, through netbuild, a two-lane junction=roundabout
// ring drawn with ringNodes nodes (a multiple of 4), with a two-way
// secondary arm at every quarter. It returns the network and the ring edges
// in circulation order, starting with the one leaving the east arm's node.
func builtRoundabout(t *testing.T, ringNodes int) (*network.Network, []network.EdgeID) {
	t.Helper()
	const clat, clon, rLat, rLon = 40.0, -74.0, 0.00018, 0.000235
	nodes := map[osm.NodeID]*osm.Node{}
	ring := &osm.Way{ID: 1, Tags: osm.Tags{
		{Key: "highway", Value: "primary"}, {Key: "junction", Value: "roundabout"}, {Key: "lanes", Value: "2"},
	}}
	for i := 0; i < ringNodes; i++ {
		a := 2 * math.Pi * float64(i) / float64(ringNodes)
		id := osm.NodeID(i + 1)
		nodes[id] = &osm.Node{ID: id, Lat: clat + rLat*math.Sin(a), Lon: clon + rLon*math.Cos(a)}
		ring.Nodes = append(ring.Nodes, osm.WayNode{ID: id})
	}
	ring.Nodes = append(ring.Nodes, osm.WayNode{ID: 1})
	feat := &osmload.Features{Nodes: nodes, Ways: []*osm.Way{ring}}
	for q := 0; q < 4; q++ {
		a := math.Pi / 2 * float64(q)
		end := osm.NodeID(100 + q)
		nodes[end] = &osm.Node{ID: end, Lat: clat + 6*rLat*math.Sin(a), Lon: clon + 6*rLon*math.Cos(a)}
		feat.Ways = append(feat.Ways, &osm.Way{
			ID:    osm.WayID(10 + q),
			Tags:  osm.Tags{{Key: "highway", Value: "secondary"}},
			Nodes: osm.WayNodes{{ID: osm.NodeID(q*ringNodes/4 + 1)}, {ID: end}},
		})
	}
	net, _, err := netbuild.Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	from := map[network.NodeID]network.EdgeID{}
	var first network.EdgeID
	found := false
	for i := range net.Edges {
		if e := &net.Edges[i]; e.Roundabout {
			from[e.From] = e.ID
			if !found || e.ID < first {
				first, found = e.ID, true
			}
		}
	}
	if !found {
		t.Fatal("no ring edges built")
	}
	order := []network.EdgeID{first}
	for e := from[net.Edges[first].To]; e != first; e = from[net.Edges[e].To] {
		order = append(order, e)
	}
	return net, order
}

// TestRoundaboutTargetLane_BuiltRing: a circulating car more than K segments
// from its exit should move to the inner lane, then to the outer lane on the
// last segment. On a ring built by netbuild it never did: geometric lane
// inference gave every ring edge AllowedTurns, which the ring lane policy
// read as the mapper's lane tags. Whether the outer lane could then continue
// round depended on how densely the ring was drawn.
func TestRoundaboutTargetLane_BuiltRing(t *testing.T) {
	for _, ringNodes := range []int{8, 32} {
		net, ring := builtRoundabout(t, ringNodes)
		if len(ring) != 4 {
			t.Fatalf("%d-node ring: %d ring edges, want 4", ringNodes, len(ring))
		}
		for _, e := range ring {
			for li, l := range net.Edges[e].Lanes {
				if len(l.AllowedTurns) != 0 {
					t.Errorf("%d-node ring: edge %d lane %d has inferred AllowedTurns %v", ringNodes, e, li, l.AllowedTurns)
				}
			}
		}
		// Three ring segments then the exit on the fourth arm.
		x := IntersectionAtNode(net, net.Edges[ring[2]].To)
		var exit network.EdgeID
		for _, o := range x.Outgoing {
			if !net.Edges[o].Roundabout {
				exit = o
			}
		}
		route := []network.EdgeID{ring[0], ring[1], ring[2], exit}
		far := &Vehicle{Route: route, Edge: route[0], RouteIdx: 0, Lane: 0}
		if lane, ok := roundaboutTargetLane(far, net); !ok || lane != 1 {
			t.Errorf("%d-node ring, 3 segments to go: target lane %d (ok %v), want inner lane 1", ringNodes, lane, ok)
		}
		last := &Vehicle{Route: route, Edge: route[2], RouteIdx: 2, Lane: 1}
		if lane, ok := roundaboutTargetLane(last, net); !ok || lane != 0 {
			t.Errorf("%d-node ring, last segment: target lane %d (ok %v), want outer lane 0", ringNodes, lane, ok)
		}
	}
}
