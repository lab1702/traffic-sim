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

// wantFrom asserts the control of the single approach at x whose edge starts
// at OSM node osmFrom. Identifies approaches by their upstream node, which
// stays unambiguous where several equal-class arms come from the same side.
func wantFrom(t *testing.T, net *network.Network, feat *osmload.Features,
	x network.Intersection, osmFrom int64, want network.Control,
) {
	t.Helper()
	netToOSM := buildNetToOSM(net, feat)
	found := false
	for i, eid := range x.Incoming {
		if netToOSM[net.Edges[eid].From] != osm.NodeID(osmFrom) {
			continue
		}
		found = true
		if got := x.IncomingControl[i]; got != want {
			t.Errorf("approach from OSM node %d (edge %d): want %v, got %v", osmFrom, eid, want, got)
		}
	}
	if !found {
		t.Errorf("no incoming approach from OSM node %d", osmFrom)
	}
}

// onlyIntersection returns the network's single intersection.
func onlyIntersection(t *testing.T, net *network.Network) network.Intersection {
	t.Helper()
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	return net.Intersections[0]
}

// TestNetbuild_Fallback_OnewayThroughEqualClassStem: a ONE-WAY residential
// street runs straight W→E and a same-class two-way residential stem joins
// from the south. The one-way has no opposing incoming partner, but it
// continues straight onto a same-class outgoing edge, so it is the through
// road: it keeps priority and only the stem yields. Previously no through pair
// was found and the T became an all-way stop.
func TestNetbuild_Fallback_OnewayThroughEqualClassStem(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W
		2: mkNode(2, 40.0000, -74.0005), // junction
		3: mkNode(3, 40.0000, -74.0000), // E
		4: mkNode(4, 39.9990, -74.0005), // S stem end
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", true, 1, 2, 3), // one-way W→E through
		mkWay(20, "residential", false, 4, 2),   // two-way S stem
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	wantFrom(t, net, feat, x, 1, network.ControlNone)
	wantFrom(t, net, feat, x, 4, network.ControlYield)
}

// TestNetbuild_Fallback_DualCarriagewaySideRoad: a primary dual carriageway
// (two one-way ways that split at node 1 and rejoin at node 3) with a
// same-class two-way side road T-ing into the eastbound half at node 2. The
// carriageway is the through road; the side road yields.
func TestNetbuild_Fallback_DualCarriagewaySideRoad(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W split
		2: mkNode(2, 40.0000, -74.0005), // side-road junction (eastbound half)
		3: mkNode(3, 40.0000, -74.0000), // E split
		7: mkNode(7, 40.0002, -74.0005), // westbound half midpoint
		4: mkNode(4, 39.9990, -74.0005), // side road S end
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", true, 1, 2, 3), // eastbound half
		mkWay(11, "primary", true, 3, 7, 1), // westbound half
		mkWay(20, "primary", false, 4, 2),   // side road
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	var x *network.Intersection
	netToOSM := buildNetToOSM(net, feat)
	for i := range net.Intersections {
		if netToOSM[net.Intersections[i].NodeID] == 2 {
			x = &net.Intersections[i]
		}
	}
	if x == nil {
		t.Fatal("no intersection at the side-road junction")
	}
	wantFrom(t, net, feat, *x, 1, network.ControlNone)
	wantFrom(t, net, feat, *x, 4, network.ControlYield)
}

// mergeFeatures builds a pure merge: way A arrives from due west, way B from
// the south-west (~30° off), and both feed a single outgoing way C to the east.
// All three are one-way and share the same highway class.
func mergeFeatures(highway string) *osmload.Features {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.00000, -74.0010), // A start (due west)
		4: mkNode(4, 39.99978, -74.0010), // B start (~30° south-west)
		2: mkNode(2, 40.00000, -74.0005), // merge node
		3: mkNode(3, 40.00000, -74.0000), // C end (east)
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, highway, true, 1, 2),
		mkWay(20, highway, true, 4, 2),
		mkWay(30, highway, true, 2, 3),
	}
	return feat
}

// TestNetbuild_Fallback_OnewayResidentialMerge: two one-way residential
// streets merge into one. The approach that continues straight is the through
// road (None); the one that bends in yields. Previously: all-way stop.
func TestNetbuild_Fallback_OnewayResidentialMerge(t *testing.T) {
	feat := mergeFeatures("residential")
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	wantFrom(t, net, feat, x, 1, network.ControlNone)
	wantFrom(t, net, feat, x, 4, network.ControlYield)
}

// TestNetbuild_Fallback_MotorwayMergeNoAllWayStop: two motorways merging.
// Previously: an all-way stop on a motorway. Now the straight arm keeps
// priority and the merging arm yields — the same rule as any other merge
// (unsigned zip merging isn't modelled; see resolveMergeGroup).
func TestNetbuild_Fallback_MotorwayMergeNoAllWayStop(t *testing.T) {
	feat := mergeFeatures("motorway")
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	wantFrom(t, net, feat, x, 1, network.ControlNone)
	wantFrom(t, net, feat, x, 4, network.ControlYield)
}

// TestNetbuild_Fallback_MotorwayLinkMergeNoAllWayStop: two motorway_link
// ramps merging into one ramp — same treatment as a motorway merge.
func TestNetbuild_Fallback_MotorwayLinkMergeNoAllWayStop(t *testing.T) {
	feat := mergeFeatures("motorway_link")
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	wantFrom(t, net, feat, x, 1, network.ControlNone)
	wantFrom(t, net, feat, x, 4, network.ControlYield)
}

// TestNetbuild_Fallback_WideAngleMergePicksStraightest: a pure merge (two
// one-way residential streets, one outgoing) where neither arm is within the
// 45° through tolerance — A bends ~50°, B ~70°. There is still no crossing
// conflict, so it must not be an all-way stop: the straighter arm (A) keeps
// priority and the other yields.
func TestNetbuild_Fallback_WideAngleMergePicksStraightest(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.000457, -74.0010), // A start: ~50° off the outgoing heading
		4: mkNode(4, 39.998947, -74.0010), // B start: ~70° off
		2: mkNode(2, 40.000000, -74.0005), // merge node
		3: mkNode(3, 40.000000, -74.0000), // C end (east)
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", true, 1, 2),
		mkWay(20, "residential", true, 4, 2),
		mkWay(30, "residential", true, 2, 3),
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	wantFrom(t, net, feat, x, 1, network.ControlNone)
	wantFrom(t, net, feat, x, 4, network.ControlYield)
}

// TestNetbuild_Fallback_OnewayCrossingStaysAllWayStop: regression guard. Two
// equal-class one-way streets crossing (W→E and S→N) are two through roads
// crossing — a genuine equal-class crossing, which stays an all-way stop.
func TestNetbuild_Fallback_OnewayCrossingStaysAllWayStop(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W
		2: mkNode(2, 40.0010, -74.0005), // N
		3: mkNode(3, 40.0000, -74.0000), // E
		4: mkNode(4, 39.9990, -74.0005), // S
		5: mkNode(5, 40.0000, -74.0005), // crossing
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", true, 1, 5, 3), // one-way W→E
		mkWay(20, "residential", true, 4, 5, 2), // one-way S→N
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	wantFrom(t, net, feat, x, 1, network.ControlAllWayStop)
	wantFrom(t, net, feat, x, 4, network.ControlAllWayStop)
}

// TestNetbuild_Fallback_OnewayCrossesTwoWayIsAllWayStop: an equal-class
// one-way street (S→N) crossing a two-way street (W-E). The one-way continues
// straight across, so it is a second through axis — a genuine equal-class
// crossing, hence AllWayStop. (Before one-way through roads were recognised,
// the one-way was mistaken for a terminating stem and made to yield to the
// two-way street.)
func TestNetbuild_Fallback_OnewayCrossesTwoWayIsAllWayStop(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W
		2: mkNode(2, 40.0010, -74.0005), // N
		3: mkNode(3, 40.0000, -74.0000), // E
		4: mkNode(4, 39.9990, -74.0005), // S
		5: mkNode(5, 40.0000, -74.0005), // crossing
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 5, 3), // two-way W-E
		mkWay(20, "residential", true, 4, 5, 2),  // one-way S→N
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	for _, from := range []int64{1, 3, 4} {
		wantFrom(t, net, feat, x, from, network.ControlAllWayStop)
	}
}

// TestNetbuild_Fallback_SlipMergeIntoTwoWayYields: regression guard. A
// same-class one-way slip road merging at a shallow angle into a two-way
// through road continues straight onto the through road's own outgoing edge.
// That is a merge into the through road, not a second through road: the
// two-way road keeps priority and the slip yields (no all-way stop).
func TestNetbuild_Fallback_SlipMergeIntoTwoWayYields(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.00000, -74.0010), // W
		2: mkNode(2, 40.00000, -74.0005), // junction
		3: mkNode(3, 40.00000, -74.0000), // E
		4: mkNode(4, 39.99978, -74.0010), // slip start (~30° south-west)
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 2, 3), // two-way through
		mkWay(20, "residential", true, 4, 2),     // one-way slip merging in
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := onlyIntersection(t, net)
	wantFrom(t, net, feat, x, 1, network.ControlNone)
	wantFrom(t, net, feat, x, 3, network.ControlNone)
	wantFrom(t, net, feat, x, 4, network.ControlYield)
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
