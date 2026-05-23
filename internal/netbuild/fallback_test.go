package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// TestNetbuild_Fallback_EqualClassTThroughFlows: an unsigned T where a
// residential road runs straight W-E and a same-class residential stem joins
// from the south. The through road must keep priority (ControlNone); only the
// terminating stem yields. Previously the equal-class fallback made all three
// approaches AllWayStop, stopping the through road for no cross traffic.
func TestNetbuild_Fallback_EqualClassTThroughFlows(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W end
		2: mkNode(2, 40.0000, -74.0005), // junction
		3: mkNode(3, 40.0000, -74.0000), // E end
		4: mkNode(4, 39.9990, -74.0005), // S stem end
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 2, 3), // W-E through
		mkWay(20, "residential", false, 4, 2),    // S stem
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	wantApproach(t, net, feat, x, "residential", "W", network.ControlNone)
	wantApproach(t, net, feat, x, "residential", "E", network.ControlNone)
	wantApproach(t, net, feat, x, "residential", "S", network.ControlYield)
}

// TestNetbuild_Fallback_BentThroughRoadFlows: a primary road that BENDS ~25°
// at the junction (one arm arrives from the WSW, the other from the ESE) with a
// residential stem joining from the south. The bent main road still continues
// through the junction, so both of its arms must keep priority (ControlNone)
// and only the stem yields. Previously resolveOpposing's coarse 22.5° axis
// buckets failed to pair the two bent arms, so applyClassFallback saw no
// through road and made the whole junction an all-way stop — halting
// straight-through traffic on the main road.
func TestNetbuild_Fallback_BentThroughRoadFlows(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		2: mkNode(2, 40.0000, -74.0000),  // junction
		1: mkNode(1, 39.9998, -74.0012),  // W arm: west + slightly south
		3: mkNode(3, 39.9998, -73.9988),  // E arm: east + slightly south
		4: mkNode(4, 39.9990, -74.0000),  // S residential stem
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 2, 3),  // bent through road (W-E)
		mkWay(20, "residential", false, 4, 2), // S minor stem
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	wantApproach(t, net, feat, x, "primary", "W", network.ControlNone)
	wantApproach(t, net, feat, x, "primary", "E", network.ControlNone)
	wantApproach(t, net, feat, x, "residential", "S", network.ControlYield)
}

// TestNetbuild_Fallback_EqualClassSpurThroughFlows: a residential road runs
// straight W-E and a same-class one-way spur leaves to the south (outgoing
// only — no cross traffic arrives). The through road must flow (ControlNone),
// not all-way-stop.
func TestNetbuild_Fallback_EqualClassSpurThroughFlows(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W end
		2: mkNode(2, 40.0000, -74.0005), // junction
		3: mkNode(3, 40.0000, -74.0000), // E end
		4: mkNode(4, 39.9990, -74.0005), // S spur end
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 2, 3), // W-E through (two-way)
		mkWay(20, "residential", true, 2, 4),     // one-way spur leaving south
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	wantApproach(t, net, feat, x, "residential", "W", network.ControlNone)
	wantApproach(t, net, feat, x, "residential", "E", network.ControlNone)
}

// TestNetbuild_Fallback_MajorMinorYields: at an unsigned major/minor T the
// minor approach gives way (ControlYield), not a mandatory stop, while the
// major through road keeps priority (ControlNone).
func TestNetbuild_Fallback_MajorMinorYields(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W end (primary)
		2: mkNode(2, 40.0000, -74.0005), // junction
		3: mkNode(3, 40.0000, -74.0000), // E end (primary)
		4: mkNode(4, 39.9990, -74.0005), // S stem (residential)
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 2, 3),  // W-E through
		mkWay(20, "residential", false, 4, 2), // S minor stem
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	wantApproach(t, net, feat, x, "primary", "W", network.ControlNone)
	wantApproach(t, net, feat, x, "primary", "E", network.ControlNone)
	wantApproach(t, net, feat, x, "residential", "S", network.ControlYield)
}
