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

// TestBuild_OnewayReverse_FlipsDirection: `oneway=-1` (also `reverse`)
// means the way is one-way but traffic flows opposite to the node
// order. Build must emit one edge with From/To flipped, not two edges
// and not a wrong-direction edge.
func TestBuild_OnewayReverse_FlipsDirection(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	w := mkWay(100, "primary", false, 1, 2)
	w.Tags = append(w.Tags, osm.Tag{Key: "oneway", Value: "-1"})
	feat.Ways = []*osm.Way{w}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 1 {
		t.Fatalf("oneway=-1 should produce 1 edge, got %d", len(net.Edges))
	}
	e := net.Edges[0]
	// Forward of way is 1→2; reverse direction means edge should be 2→1.
	if int(e.From) != 1 || int(e.To) != 0 {
		// The reverse-direction edge runs from the way's last node
		// (node 2 → osm netID 1) to the way's first node (node 1 →
		// osm netID 0), since osmToNet assigns netIDs in iteration
		// order over the way's nodes.
		t.Errorf("oneway=-1: want edge From=1 To=0 (geometry reversed); got From=%d To=%d", e.From, e.To)
	}
}

// TestBuild_LanesPerDirection_HonorsForwardBackward: when a two-way
// street tags asymmetric lane counts (e.g. lanes:forward=2, lanes:backward=1
// for a road with a bus lane in one direction only), each edge must get
// its respective count, not the halved total.
func TestBuild_LanesPerDirection_HonorsForwardBackward(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0001, -74.0),
	}}
	w := mkWay(100, "primary", false, 1, 2)
	w.Tags = append(w.Tags,
		osm.Tag{Key: "lanes", Value: "3"},
		osm.Tag{Key: "lanes:forward", Value: "2"},
		osm.Tag{Key: "lanes:backward", Value: "1"},
	)
	feat.Ways = []*osm.Way{w}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 2 {
		t.Fatalf("two-way should produce 2 edges, got %d", len(net.Edges))
	}
	// Forward edge (1→2) gets 2 lanes; reverse edge (2→1) gets 1.
	var fwdLanes, bwdLanes int
	for _, e := range net.Edges {
		switch {
		case int(e.From) == 0 && int(e.To) == 1:
			fwdLanes = len(e.Lanes)
		case int(e.From) == 1 && int(e.To) == 0:
			bwdLanes = len(e.Lanes)
		}
	}
	if fwdLanes != 2 {
		t.Errorf("forward edge: want 2 lanes (from lanes:forward), got %d", fwdLanes)
	}
	if bwdLanes != 1 {
		t.Errorf("backward edge: want 1 lane (from lanes:backward), got %d", bwdLanes)
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

// TestBuild_TwoWayEdgesHaveDistinctLaneSlices verifies that mutating one
// direction's lanes does not affect the reverse direction's lanes. This
// matters once per-direction state (AllowedTurns) is stored on Lane.
func TestBuild_TwoWayEdgesHaveDistinctLaneSlices(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0),
		2: mkNode(2, 40.0, -74.001),
	}}
	feat.Ways = []*osm.Way{mkWay(10, "residential", false, 1, 2)}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Edges) != 2 {
		t.Fatalf("want 2 edges (fwd+rev), got %d", len(net.Edges))
	}
	// Mutate one direction's lanes; the other must not change.
	net.Edges[0].Lanes[0].AllowedTurns = []network.EdgeID{99}
	if len(net.Edges[1].Lanes[0].AllowedTurns) != 0 {
		t.Errorf("reverse edge lanes aliased to forward; got AllowedTurns=%v",
			net.Edges[1].Lanes[0].AllowedTurns)
	}
}

// Builds a + intersection like TestBuild_PlusShape but verifies that the
// incoming edges at the central intersection have their lane AllowedTurns
// populated (geometric fallback, since no turn:lanes tag).
func TestBuild_PopulatesAllowedTurnsAtIntersections(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0, 0.0),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0, -74.0005),
	}}
	// Two two-way ways crossing at node 5. Set lanes=4 so each direction has 2 lanes.
	primary := mkWay(10, "primary", false, 1, 5, 3)
	primary.Tags = append(primary.Tags, osm.Tag{Key: "lanes", Value: "4"})
	cross := mkWay(20, "primary", false, 2, 5, 4)
	cross.Tags = append(cross.Tags, osm.Tag{Key: "lanes", Value: "4"})
	feat.Ways = []*osm.Way{primary, cross}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	var central *network.Intersection
	for i := range net.Intersections {
		if len(net.Intersections[i].Incoming) >= 3 {
			central = &net.Intersections[i]
			break
		}
	}
	if central == nil {
		t.Fatal("no central intersection found")
	}
	for _, eid := range central.Incoming {
		e := &net.Edges[eid]
		if len(e.Lanes) < 2 {
			continue // skip 1-lane edges (only populated when multi-lane)
		}
		anyPopulated := false
		for _, l := range e.Lanes {
			if len(l.AllowedTurns) > 0 {
				anyPopulated = true
				break
			}
		}
		if !anyPopulated {
			t.Errorf("incoming edge %d has 2 lanes but no AllowedTurns populated", eid)
		}
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

// TestBuild_ThroughWayRestriction_DirectionAware loads a fixture where the
// from-way of a turn restriction passes *through* the via node. Multiple
// edges end at the via (forward and reverse direction); applyOSMRestrictions
// must pick the forward-direction edge so the restriction matches the OSM
// convention. This guards against pickEdgeAt returning whichever candidate
// happens to appear first in slice order.
//
// Regression: see review #4 (2026-05-15).
func TestBuild_ThroughWayRestriction_DirectionAware(t *testing.T) {
	feat, err := osmload.Load("../osmload/testdata/with_through_restriction.osm")
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	net, rpt, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if rpt.RestrictionsApplied != 1 {
		t.Fatalf("RestrictionsApplied: want 1, got %d (skipped=%d)", rpt.RestrictionsApplied, rpt.RestrictionsSkipped)
	}
	if len(net.Intersections) == 0 {
		t.Fatal("no intersections after build")
	}

	// Find the via intersection — it's the only one with BannedTurns.
	var via *network.Intersection
	for i := range net.Intersections {
		if len(net.Intersections[i].BannedTurns) > 0 {
			via = &net.Intersections[i]
			break
		}
	}
	if via == nil {
		t.Fatal("no intersection has BannedTurns")
	}
	if got := len(via.BannedTurns); got != 1 {
		t.Fatalf("via should have exactly 1 BannedTurn, got %d", got)
	}

	// The fixture's geometry is set up so that the *forward-direction*
	// from-edge yields a left turn onto the to way (westbound vehicle
	// turning south). If pickEdgeAt picked the reverse-direction edge
	// instead (eastbound), the same turn would classify as TurnRight.
	tr := via.BannedTurns[0]
	cat := network.ClassifyTurn(net, tr.From, tr.To)
	if cat != network.TurnLeft {
		t.Errorf("banned turn should classify as TurnLeft (forward-direction from-edge); got %v (angle=%.3f rad). "+
			"This usually means pickEdgeAt chose the reverse-direction from-edge.",
			cat, network.TurnAngle(net, tr.From, tr.To))
	}
}

func TestOnewayDirection_Roundabout(t *testing.T) {
	rab := &osm.Way{Tags: osm.Tags{{Key: "highway", Value: "primary"}, {Key: "junction", Value: "roundabout"}}}
	if got := onewayDirection(rab); got != onewayForward {
		t.Fatalf("junction=roundabout: got %v, want onewayForward", got)
	}
	circ := &osm.Way{Tags: osm.Tags{{Key: "junction", Value: "circular"}}}
	if got := onewayDirection(circ); got != onewayForward {
		t.Fatalf("junction=circular: got %v, want onewayForward", got)
	}
	// Explicit oneway tag still wins over the junction implication.
	twoWay := &osm.Way{Tags: osm.Tags{{Key: "junction", Value: "roundabout"}, {Key: "oneway", Value: "no"}}}
	if got := onewayDirection(twoWay); got != onewayTwoWay {
		t.Fatalf("roundabout + oneway=no: got %v, want onewayTwoWay", got)
	}
}

func TestIsRoundabout(t *testing.T) {
	if !isRoundabout(&osm.Way{Tags: osm.Tags{{Key: "junction", Value: "roundabout"}}}) {
		t.Fatal("junction=roundabout should be a roundabout")
	}
	if !isRoundabout(&osm.Way{Tags: osm.Tags{{Key: "junction", Value: "circular"}}}) {
		t.Fatal("junction=circular should be a roundabout")
	}
	if isRoundabout(&osm.Way{Tags: osm.Tags{{Key: "highway", Value: "primary"}}}) {
		t.Fatal("plain primary should not be a roundabout")
	}
}

// squareRoundaboutFeatures builds a square ring of 4 nodes (1-2-3-4-1)
// tagged junction=roundabout, with one approach road at each ring node so
// all four ring nodes become intersections and the ring splits into four
// one-way segments.
func squareRoundaboutFeatures() *osmload.Features {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0000),
		2: mkNode(2, 40.0000, -73.9996),
		3: mkNode(3, 40.0004, -73.9996),
		4: mkNode(4, 40.0004, -74.0000),
		5: mkNode(5, 39.9994, -74.0000), // approach to node 1
		6: mkNode(6, 40.0000, -73.9990), // approach to node 2
		7: mkNode(7, 40.0010, -73.9996), // approach to node 3
		8: mkNode(8, 40.0004, -74.0010), // approach to node 4
	}}
	ring := mkWay(100, "primary", false, 1, 2, 3, 4, 1)
	ring.Tags = append(ring.Tags, osm.Tag{Key: "junction", Value: "roundabout"})
	feat.Ways = []*osm.Way{
		ring,
		mkWay(101, "secondary", false, 5, 1),
		mkWay(102, "secondary", false, 6, 2),
		mkWay(103, "secondary", false, 7, 3),
		mkWay(104, "secondary", false, 8, 4),
	}
	return feat
}

// A junction=roundabout way tagged explicitly oneway=no is malformed (a
// real roundabout is always one-way). It must build as a normal two-way road
// — no edge flagged Roundabout — rather than giving both directions ring
// priority at the node.
func TestBuild_MalformedTwoWayRoundaboutNotFlagged(t *testing.T) {
	feat := squareRoundaboutFeatures()
	feat.Ways[0].Tags = append(feat.Ways[0].Tags, osm.Tag{Key: "oneway", Value: "no"})

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	for i := range net.Edges {
		if net.Edges[i].Roundabout {
			t.Errorf("malformed two-way roundabout must not flag ring edges; edge %d (%d->%d) is flagged",
				i, net.Edges[i].From, net.Edges[i].To)
		}
	}
}

func TestBuild_RoundaboutEdgesFlaggedOneWay(t *testing.T) {
	feat := squareRoundaboutFeatures()

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}

	ringCount := 0
	for i := range net.Edges {
		if net.Edges[i].Roundabout {
			ringCount++
		}
	}
	if ringCount != 4 {
		t.Fatalf("expected 4 one-way ring edges, got %d", ringCount)
	}
	// One-way: no ring edge may have a reverse twin that is also a ring edge.
	for i := range net.Edges {
		e := &net.Edges[i]
		if !e.Roundabout {
			continue
		}
		for j := range net.Edges {
			r := &net.Edges[j]
			if r.Roundabout && r.From == e.To && r.To == e.From {
				t.Fatalf("found a wrong-way ring edge %d->%d", r.From, r.To)
			}
		}
	}
}

func TestBuild_RoundaboutControl(t *testing.T) {
	net, _, err := Build(squareRoundaboutFeatures())
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	sawRingNode := false
	for i := range net.Intersections {
		x := &net.Intersections[i]
		onRing := false
		for _, eid := range x.Incoming {
			if net.Edges[eid].Roundabout {
				onRing = true
			}
		}
		if !onRing {
			continue
		}
		sawRingNode = true
		for j, eid := range x.Incoming {
			c := x.IncomingControl[j]
			if c == network.ControlAllWayStop {
				t.Errorf("ring node intersection %d: approach %d must not be AllWayStop", i, eid)
			}
			if net.Edges[eid].Roundabout {
				if c != network.ControlNone {
					t.Errorf("circulating edge %d: got %v, want ControlNone", eid, c)
				}
			} else if c != network.ControlYield {
				t.Errorf("entering edge %d: got %v, want ControlYield", eid, c)
			}
		}
	}
	if !sawRingNode {
		t.Fatal("expected at least one ring node in the built network")
	}
}
