package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// TestNetbuild_WayJoinIsNotAStop reproduces the "cars stop on a straight
// road with no junction" bug. OSM routinely splits one physical road into
// multiple ways at tag boundaries (speed-limit change, name change, bridge,
// surface). Where two such ways of the SAME road meet end-to-end there is
// no cross traffic — yet the node is shared by ≥2 ways, so it was promoted
// to an intersection and the equal-class fallback made it an all-way stop.
// Through-traffic then stops dead at an invisible point on a straight road.
//
// A pure continuation node (exactly two collinear approaches, no cross
// street, no signal) must impose NO right-of-way control.
func TestNetbuild_WayJoinIsNotAStop(t *testing.T) {
	// Three colinear nodes on an east-west line; the middle node (2) is the
	// way-join. No cross street touches it.
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // west end
		2: mkNode(2, 40.0000, -74.0005), // join (shared by both ways)
		3: mkNode(3, 40.0000, -74.0000), // east end
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 2), // west segment
		mkWay(20, "residential", false, 2, 3), // east segment (same road, tag split)
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}

	// No approach anywhere in this network should impose a stop/yield: it is
	// a single straight road with no real junction.
	for _, x := range net.Intersections {
		for i, c := range x.IncomingControl {
			if c != network.ControlNone {
				t.Errorf("way-join node %d approach %d (edge %d): want ControlNone, got %v — "+
					"a straight road continuation must not stop through-traffic",
					x.NodeID, i, x.Incoming[i], c)
			}
		}
	}
}
