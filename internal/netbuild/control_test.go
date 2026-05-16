package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// TestNetbuild_Fallback_UnequalClass: a primary road meets a residential
// road. The residential approach gets ControlStop; the primary approaches
// stay ControlNone.
func TestNetbuild_Fallback_UnequalClass(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005), // intersection
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	if len(x.IncomingControl) != len(x.Incoming) {
		t.Fatalf("IncomingControl length %d != Incoming length %d", len(x.IncomingControl), len(x.Incoming))
	}
	var sawStop, sawNone bool
	for i, eid := range x.Incoming {
		c := x.IncomingControl[i]
		hw := highwayOfEdge(net, eid, feat)
		switch hw {
		case "primary":
			if c != network.ControlNone {
				t.Errorf("primary approach (edge %d) should be None, got %v", eid, c)
			}
			sawNone = true
		case "residential":
			if c != network.ControlStop {
				t.Errorf("residential approach (edge %d) should be Stop, got %v", eid, c)
			}
			sawStop = true
		}
	}
	if !sawStop || !sawNone {
		t.Error("expected to see both Stop and None controls at unequal-class fallback intersection")
	}
}

// TestNetbuild_Fallback_EqualClass: two residential roads meet, no
// signage. Every approach gets ControlAllWayStop.
func TestNetbuild_Fallback_EqualClass(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("approach %d should be AllWayStop, got %v", i, c)
		}
	}
}

// highwayOfEdge returns the highway= tag of the OSM way an edge was derived from.
// Built via a node-position reverse map to avoid relying on NodeID ↔ osm.NodeID
// arithmetic.
func highwayOfEdge(net *network.Network, eid network.EdgeID, feat *osmload.Features) string {
	e := net.Edges[eid]
	netToOSM := buildNetToOSM(net, feat)
	fromOSM, fromOk := netToOSM[e.From]
	toOSM, toOk := netToOSM[e.To]
	if !fromOk || !toOk {
		return ""
	}
	for _, w := range feat.Ways {
		for i := 0; i+1 < len(w.Nodes); i++ {
			a, b := w.Nodes[i].ID, w.Nodes[i+1].ID
			if (a == fromOSM && b == toOSM) || (a == toOSM && b == fromOSM) {
				for _, t := range w.Tags {
					if t.Key == "highway" {
						return t.Value
					}
				}
			}
		}
	}
	return ""
}

// TestNetbuild_StopAll: an intersection node tagged stop=all forces
// every approach to AllWayStop regardless of class.
func TestNetbuild_StopAll(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "stop", "all"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("approach %d should be AllWayStop under stop=all, got %v", i, c)
		}
	}
}

// TestNetbuild_StopMinor: stop=minor tags only the lower-class approaches.
func TestNetbuild_StopMinor(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "stop", "minor"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	var sawStop, sawNone bool
	for i, eid := range x.Incoming {
		c := x.IncomingControl[i]
		hw := highwayOfEdge(net, eid, feat)
		switch hw {
		case "primary":
			if c != network.ControlNone {
				t.Errorf("primary approach should be None under stop=minor, got %v", c)
			}
			sawNone = true
		case "residential":
			if c != network.ControlStop {
				t.Errorf("residential approach should be Stop under stop=minor, got %v", c)
			}
			sawStop = true
		}
	}
	if !sawStop || !sawNone {
		t.Error("expected mixed controls under stop=minor")
	}
}

// TestNetbuild_HighwayStopOnNode: an intersection node tagged
// highway=stop (no direction) applies Stop to all approaches.
func TestNetbuild_HighwayStopOnNode(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "highway", "stop"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlStop {
			t.Errorf("approach %d should be Stop under highway=stop without direction, got %v", i, c)
		}
	}
}

// TestNetbuild_HighwayGiveWayOnNode: an intersection node tagged
// highway=give_way applies Yield to all approaches.
func TestNetbuild_HighwayGiveWayOnNode(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005, "highway", "give_way"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlYield {
			t.Errorf("approach %d should be Yield under highway=give_way, got %v", i, c)
		}
	}
}

// buildNetToOSM is a position-based reverse map: each network NodeID gets
// matched to the OSM NodeID whose projected (lat, lon) lands at the same
// planar position. Computed fresh per call; only used in tests on tiny fixtures.
func buildNetToOSM(net *network.Network, feat *osmload.Features) map[network.NodeID]osm.NodeID {
	// Determine the reference point used by Build (centroid of all nodes).
	var sumLat, sumLon float64
	osmIDs := make([]osm.NodeID, 0, len(feat.Nodes))
	for id := range feat.Nodes {
		osmIDs = append(osmIDs, id)
	}
	// Sort for determinism (mirrors refPoint in netbuild.go).
	for i := 1; i < len(osmIDs); i++ {
		for j := i; j > 0 && osmIDs[j] < osmIDs[j-1]; j-- {
			osmIDs[j], osmIDs[j-1] = osmIDs[j-1], osmIDs[j]
		}
	}
	for _, id := range osmIDs {
		sumLat += feat.Nodes[id].Lat
		sumLon += feat.Nodes[id].Lon
	}
	refLat := sumLat / float64(len(osmIDs))
	refLon := sumLon / float64(len(osmIDs))

	out := make(map[network.NodeID]osm.NodeID, len(net.Nodes))
	for nid, n := range net.Nodes {
		want := n.Pos
		var best osm.NodeID
		bestD := 1e18
		for _, oid := range osmIDs {
			on := feat.Nodes[oid]
			// Project OSM node to planar coords using same formula as Build.
			p := project(on.Lat, on.Lon, refLat, refLon)
			dx := p.X - want.X
			dy := p.Y - want.Y
			d := dx*dx + dy*dy
			if d < bestD {
				bestD = d
				best = oid
			}
		}
		out[network.NodeID(nid)] = best
	}
	return out
}
