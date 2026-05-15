package netbuild

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

func mkNode(id int64, lat, lon float64, tags ...string) *osm.Node {
	n := &osm.Node{ID: osm.NodeID(id), Lat: lat, Lon: lon}
	for i := 0; i+1 < len(tags); i += 2 {
		n.Tags = append(n.Tags, osm.Tag{Key: tags[i], Value: tags[i+1]})
	}
	return n
}

func mkWay(id int64, highway string, oneway bool, nodes ...int64) *osm.Way {
	w := &osm.Way{ID: osm.WayID(id)}
	w.Tags = append(w.Tags, osm.Tag{Key: "highway", Value: highway})
	if oneway {
		w.Tags = append(w.Tags, osm.Tag{Key: "oneway", Value: "yes"})
	}
	for _, n := range nodes {
		w.Nodes = append(w.Nodes, osm.WayNode{ID: osm.NodeID(n)})
	}
	return w
}

// Builds a + shape:
//
//	2
//	|
//
// 1-X-3      X is the intersection at node 5
//
//	|
//	4
func TestBuild_PlusShape_SplitsAtIntersection(t *testing.T) {
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
	// 5 nodes kept, 8 edges (4 segments * 2 directions), 1 intersection at node 5.
	if len(net.Nodes) != 5 {
		t.Errorf("want 5 nodes, got %d", len(net.Nodes))
	}
	if len(net.Edges) != 8 {
		t.Errorf("want 8 edges (4 segs * 2 dirs), got %d", len(net.Edges))
	}
	if len(net.Intersections) != 1 {
		t.Errorf("want 1 intersection, got %d", len(net.Intersections))
	}
}

func TestBuild_DropsDisconnectedComponent(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
		3: mkNode(3, 40.0002, -74.0),
		// disconnected:
		10: mkNode(10, 40.5, -74.0),
		11: mkNode(11, 40.5001, -74.0),
	}}
	feat.Ways = []*osm.Way{
		mkWay(100, "residential", false, 1, 2, 3),
		mkWay(200, "residential", false, 10, 11),
	}
	net, rpt, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if rpt.ComponentsDropped != 1 {
		t.Errorf("want 1 dropped component, got %d", rpt.ComponentsDropped)
	}
	if len(net.Nodes) != 3 {
		t.Errorf("want 3 nodes after pruning, got %d", len(net.Nodes))
	}
}

func TestBuild_OnewayProducesSingleEdge(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	feat.Ways = []*osm.Way{mkWay(100, "primary", true, 1, 2)}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 1 {
		t.Errorf("oneway should produce 1 edge, got %d", len(net.Edges))
	}
}

func TestBuild_RespectsMaxspeedKmh(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	w := mkWay(100, "residential", false, 1, 2)
	w.Tags = append(w.Tags, osm.Tag{Key: "maxspeed", Value: "50"}) // 50 km/h
	feat.Ways = []*osm.Way{w}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	// 50 km/h = 13.888... m/s
	want := 50.0 / 3.6
	for _, e := range net.Edges {
		if e.SpeedLimit < want-0.01 || e.SpeedLimit > want+0.01 {
			t.Errorf("edge speed: want %.3f m/s, got %.3f", want, e.SpeedLimit)
		}
	}
}

func TestBuild_RespectsMaxspeedMph(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	w := mkWay(100, "residential", false, 1, 2)
	w.Tags = append(w.Tags, osm.Tag{Key: "maxspeed", Value: "30 mph"})
	feat.Ways = []*osm.Way{w}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	// 30 mph = 13.4112 m/s
	want := 30.0 * 0.44704
	for _, e := range net.Edges {
		if e.SpeedLimit < want-0.01 || e.SpeedLimit > want+0.01 {
			t.Errorf("edge speed: want %.3f m/s, got %.3f", want, e.SpeedLimit)
		}
	}
}

func TestBuild_RespectsExplicitLanes(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	w := mkWay(100, "residential", false, 1, 2)
	w.Tags = append(w.Tags, osm.Tag{Key: "lanes", Value: "4"})
	feat.Ways = []*osm.Way{w}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	// lanes=4 on two-way => 2 per direction
	for _, e := range net.Edges {
		if len(e.Lanes) != 2 {
			t.Errorf("want 2 lanes per direction, got %d", len(e.Lanes))
		}
	}
}

// TestBuild_OnewayJunctionIsIntersection guards against a threshold bug
// where a node where two oneways meet (1 incoming + 1 outgoing) gets
// mis-classified. Such a node IS shared by 2 ways so it must register
// as an intersection.
func TestBuild_OnewayJunctionIsIntersection(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
		3: mkNode(3, 40.0002, -74.0),
	}}
	// Way 10: 1->2 (oneway). Way 20: 2->3 (oneway). Node 2 is shared.
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", true, 1, 2),
		mkWay(20, "primary", true, 2, 3),
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Errorf("oneway-to-oneway junction at node 2 should produce 1 intersection, got %d", len(net.Intersections))
	}
}

// TestBuild_LeafOfTwoWayIsNotIntersection: a dead-end node on a single
// two-way street should NOT be classified as an intersection (it only
// touches one way, regardless of edge counts).
func TestBuild_LeafOfTwoWayIsNotIntersection(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	feat.Ways = []*osm.Way{mkWay(100, "residential", false, 1, 2)}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 0 {
		t.Errorf("dead-end leaves of a single two-way street should not be intersections, got %d", len(net.Intersections))
	}
}

// TestBuild_AppliesNoLeftTurnRestriction loads the with_restriction.osm
// fixture and asserts that the no_left_turn relation produced exactly one
// BannedTurn entry at the central intersection, and that the banned (from,
// to) pair classifies as a left turn.
func TestBuild_AppliesNoLeftTurnRestriction(t *testing.T) {
	feat, err := osmload.Load("../osmload/testdata/with_restriction.osm")
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if len(feat.Restrictions) != 1 {
		t.Fatalf("fixture should expose 1 restriction relation, got %d", len(feat.Restrictions))
	}

	net, rpt, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if rpt.RestrictionsApplied != 1 {
		t.Errorf("RestrictionsApplied: want 1, got %d (skipped=%d)", rpt.RestrictionsApplied, rpt.RestrictionsSkipped)
	}

	// Find the center intersection. The graph has one 4-way intersection
	// after pruning (the four leaves are non-intersections per the same
	// usage-count rule the rest of the tests rely on).
	if len(net.Intersections) != 1 {
		t.Fatalf("expected 1 intersection (the center), got %d", len(net.Intersections))
	}
	x := &net.Intersections[0]
	if len(x.BannedTurns) != 1 {
		t.Fatalf("want exactly 1 BannedTurn at center, got %d", len(x.BannedTurns))
	}

	// The relation declares no_left_turn from way 20 (east arm) via node 1
	// to way 30 (south arm) — i.e., westbound vehicle turning south =
	// left turn at the center. Sanity-check the classification.
	tr := x.BannedTurns[0]
	cat := network.ClassifyTurn(net, tr.From, tr.To)
	if cat != network.TurnLeft {
		t.Errorf("banned turn should classify as TurnLeft, got %v (angle=%.3f rad)",
			cat, network.TurnAngle(net, tr.From, tr.To))
	}
}
