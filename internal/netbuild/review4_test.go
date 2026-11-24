package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// Regression tests for the fourth full code review.

// TestNetbuild_StopMinorEqualClass: stop=minor gave the sign only to
// approaches of a lower class than the best, so where every road is of the
// same class every approach became uncontrolled — a signed junction where
// nobody yields. With no class difference the class fallback decides which
// road is minor.
func TestNetbuild_StopMinorEqualClass(t *testing.T) {
	// Crossing of two residential streets: no through road to favour, so the
	// fallback's all-way stop stands.
	feat := crossingWithNodeTags("highway", "stop", "stop", "minor")
	feat.Ways[0] = mkWay(10, "residential", false, 1, 5, 3)
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]
	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("crossing approach %d: got %v, want AllWayStop", i, c)
		}
	}

	// T junction of residential streets: the through road keeps priority and
	// the stem takes the stop sign.
	feat = &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, -74.0000),
		5: mkNode(5, 40.0, -74.0005, "highway", "stop", "stop", "minor"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 5, 3),
		mkWay(20, "residential", false, 2, 5),
	}
	net, _, err = Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x = net.Intersections[0]
	wantApproach(t, net, feat, x, "residential", "W", network.ControlNone)
	wantApproach(t, net, feat, x, "residential", "E", network.ControlNone)
	wantApproach(t, net, feat, x, "residential", "N", network.ControlStop)
}

// TestNetbuild_ClosedWaySignOnClosingSegment: a service-road loop drawn as
// one closed way [X, P, Q, R, G, X], meeting a street at X and a spur at R,
// with a stop sign on G, the stop line of the loop's closing segment R->G->X.
// The sign lookup searched the way for X and found it at position 0, not 5,
// so for that approach it walked P and Q (the other approach from R) and
// missed the sign.
func TestNetbuild_ClosedWaySignOnClosingSegment(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0000),                    // X
		2: mkNode(2, 40.0006, -74.0004),                    // P
		3: mkNode(3, 40.0012, -74.0000),                    // Q
		4: mkNode(4, 40.0006, -73.9996),                    // R
		5: mkNode(5, 40.0002, -73.9999, "highway", "stop"), // G
		6: mkNode(6, 40.0000, -74.0010),                    // street west end
		7: mkNode(7, 40.0000, -73.9990),                    // street east end
		8: mkNode(8, 40.0006, -73.9986),                    // spur end
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "service", false, 1, 2, 3, 4, 5, 1),
		mkWay(20, "residential", false, 6, 1, 7),
		mkWay(30, "service", false, 4, 8),
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	var x *network.Intersection
	for i := range net.Intersections {
		if len(net.Intersections[i].Incoming) == 4 { // X: two street arms, two loop arms
			x = &net.Intersections[i]
		}
	}
	if x == nil {
		t.Fatalf("no 4-approach intersection at X")
	}
	found := 0
	for i, eid := range x.Incoming {
		e := &net.Edges[eid]
		if len(e.Geometry) < 3 {
			continue // a street arm (no shaping nodes)
		}
		// The closing segment R->G->X has 3 points; the reverse of X->P->Q->R
		// has 4.
		want := network.ControlYield
		if len(e.Geometry) == 3 {
			want = network.ControlStop
		}
		found++
		if got := x.IncomingControl[i]; got != want {
			t.Errorf("loop approach with %d points: got %v, want %v", len(e.Geometry), got, want)
		}
	}
	if found != 2 {
		t.Fatalf("found %d loop approaches at X, want 2", found)
	}
}

// TestNetbuild_ReverseOnewayWidth: a oneway=-1 road builds its single edge
// with the backward lane count, but its width came from the forward count
// (the default 1 lane here), so its three lanes were drawn off the band.
func TestNetbuild_ReverseOnewayWidth(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0, -74.0000),
	}}
	w := mkWay(10, "residential", false, 1, 2)
	w.Tags = append(w.Tags, osm.Tag{Key: "oneway", Value: "-1"}, osm.Tag{Key: "lanes:backward", Value: "3"})
	feat.Ways = []*osm.Way{w}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 1 {
		t.Fatalf("got %d edges, want the one reverse edge", len(net.Edges))
	}
	e := net.Edges[0]
	if len(e.Lanes) != 3 {
		t.Fatalf("fixture: edge has %d lanes, want 3", len(e.Lanes))
	}
	if want := 3 * network.LaneWidthMeters; e.Width != want {
		t.Errorf("width %.1f m, want %.1f m for its 3 lanes", e.Width, want)
	}
}

// TestNetbuild_LollipopJunction: a cul-de-sac drawn as one way [A, B, C, D,
// E, B] meets itself at B. B was counted once per way, so it never became a
// junction: the way built as one edge A->B running all round the loop, and a
// car going in drove the loop to B, then U-turned and drove it again.
func TestNetbuild_LollipopJunction(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0000), // A
		2: mkNode(2, 40.0005, -74.0000), // B
		3: mkNode(3, 40.0010, -74.0004),
		4: mkNode(4, 40.0014, -74.0000),
		5: mkNode(5, 40.0010, -73.9996),
	}}
	feat.Ways = []*osm.Way{mkWay(10, "residential", false, 1, 2, 3, 4, 5, 2)}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("got %d intersections, want 1 where the loop closes", len(net.Intersections))
	}
	b := net.Intersections[0].NodeID
	var stem *network.Edge
	for i := range net.Edges {
		if e := &net.Edges[i]; e.To == b && e.From != b {
			stem = e
		}
	}
	if stem == nil {
		t.Fatalf("no edge from A ends at the loop junction")
	}
	if stem.Length > 60 { // A to B is ~56 m; round the loop is ~200 m more
		t.Errorf("A->B is %.0f m: it still runs round the loop", stem.Length)
	}
}
