package netbuild

// Regression tests for the fifth code review. Each fails before its fix.

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// osmEdge returns the edge running from OSM node a to OSM node b.
func osmEdge(t *testing.T, net *network.Network, feat *osmload.Features, a, b osm.NodeID) network.EdgeID {
	t.Helper()
	toNet := make(map[osm.NodeID]network.NodeID)
	for nid, oid := range buildNetToOSM(net, feat) {
		toNet[oid] = nid
	}
	for _, e := range net.Edges {
		if e.From == toNet[a] && e.To == toNet[b] {
			return e.ID
		}
	}
	t.Fatalf("no edge %d->%d", a, b)
	return 0
}

// approachControl returns the control on approach edge eid at the junction it
// ends at.
func approachControl(t *testing.T, net *network.Network, eid network.EdgeID) network.Control {
	t.Helper()
	to := net.Edges[eid].To
	for i := range net.Intersections {
		x := &net.Intersections[i]
		if x.NodeID != to {
			continue
		}
		for j, in := range x.Incoming {
			if in == eid {
				return x.IncomingControl[j]
			}
		}
	}
	t.Fatalf("edge %d is no approach of a junction", eid)
	return 0
}

// TestNetbuild_InteriorSignFacesOneDirection: a primary way 1-5-6-9-3 with
// residential cross streets at junctions 5 and 9 and a stop sign on node 6,
// just before 9. The segment between the junctions is the approach to both,
// and the sign was applied to both: westbound main-road traffic stopped at 5,
// 80 m from the sign, where the side streets only yield. A sign is for the
// direction its direction= tag names, or else for the nearer junction.
func TestNetbuild_InteriorSignFacesOneDirection(t *testing.T) {
	for _, tc := range []struct {
		name     string
		signTags []string
		eastAt9  network.Control // approach 5->9
		westAt5  network.Control // approach 9->5
	}{
		{"direction=forward", []string{"highway", "stop", "direction", "forward"}, network.ControlStop, network.ControlNone},
		{"direction=backward", []string{"highway", "stop", "direction", "backward"}, network.ControlNone, network.ControlStop},
		{"no direction: nearer junction", []string{"highway", "stop"}, network.ControlStop, network.ControlNone},
	} {
		t.Run(tc.name, func(t *testing.T) {
			feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
				1:  mkNode(1, 40.0, -74.0020),
				5:  mkNode(5, 40.0, -74.0010),
				6:  mkNode(6, 40.0, -74.0001, tc.signTags...),
				9:  mkNode(9, 40.0, -74.0000),
				3:  mkNode(3, 40.0, -73.9990),
				11: mkNode(11, 40.0010, -74.0010),
				12: mkNode(12, 39.9990, -74.0010),
				13: mkNode(13, 40.0010, -74.0000),
				14: mkNode(14, 39.9990, -74.0000),
			}}
			feat.Ways = []*osm.Way{
				mkWay(10, "primary", false, 1, 5, 6, 9, 3),
				mkWay(20, "residential", false, 11, 5, 12),
				mkWay(30, "residential", false, 13, 9, 14),
			}
			net, _, err := Build(feat)
			if err != nil {
				t.Fatalf("Build: %v", err)
			}
			if got := approachControl(t, net, osmEdge(t, net, feat, 5, 9)); got != tc.eastAt9 {
				t.Errorf("eastbound at 9: got %v, want %v", got, tc.eastAt9)
			}
			if got := approachControl(t, net, osmEdge(t, net, feat, 9, 5)); got != tc.westAt5 {
				t.Errorf("westbound at 5: got %v, want %v", got, tc.westAt5)
			}
		})
	}
}

// TestParseLanesPerDirection_TaggedDirectionTakesItsShare: with a total and
// one direction's count, the other direction has the rest of the total (less
// shared centre lanes), not half of it: lanes=4 + lanes:forward=3 built five
// lanes.
func TestParseLanesPerDirection_TaggedDirectionTakesItsShare(t *testing.T) {
	for _, tc := range []struct {
		tags     []string
		fwd, bwd uint8
	}{
		{[]string{"lanes", "4", "lanes:forward", "3"}, 3, 1},
		{[]string{"lanes", "5", "lanes:backward", "1"}, 4, 1},
		{[]string{"lanes", "5", "lanes:both_ways", "1", "lanes:forward", "2"}, 2, 2},
		{[]string{"lanes", "2", "lanes:forward", "3"}, 3, 1}, // inconsistent: keep one lane back
		{[]string{"lanes", "4"}, 2, 2},
		{[]string{"lanes", "4", "lanes:forward", "1", "lanes:backward", "2"}, 1, 2},
	} {
		w := mkWay(1, "primary", false, 1, 2)
		for i := 0; i+1 < len(tc.tags); i += 2 {
			w.Tags = append(w.Tags, osm.Tag{Key: tc.tags[i], Value: tc.tags[i+1]})
		}
		if fwd, bwd := parseLanesPerDirection(w, 1); fwd != tc.fwd || bwd != tc.bwd {
			t.Errorf("%v: got (%d, %d), want (%d, %d)", tc.tags, fwd, bwd, tc.fwd, tc.bwd)
		}
	}
}

// TestNetbuild_HighwayRoadOutranksService: highway=road is built like
// unclassified but was ranked as an unknown highway value, below service, so
// a road through a junction gave way to a service road joining it.
func TestNetbuild_HighwayRoadOutranksService(t *testing.T) {
	if highwayPriority("road") != highwayPriority("unclassified") {
		t.Errorf("highwayPriority(road) = %d, want unclassified's %d",
			highwayPriority("road"), highwayPriority("unclassified"))
	}
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),
		5: mkNode(5, 40.0, -74.0005),
		3: mkNode(3, 40.0, -74.0000),
		2: mkNode(2, 40.0010, -74.0005),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "road", false, 1, 5, 3),
		mkWay(20, "service", false, 2, 5),
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	for _, c := range []struct {
		a, b osm.NodeID
		want network.Control
	}{{1, 5, network.ControlNone}, {3, 5, network.ControlNone}, {2, 5, network.ControlYield}} {
		if got := approachControl(t, net, osmEdge(t, net, feat, c.a, c.b)); got != c.want {
			t.Errorf("approach %d->5: got %v, want %v", c.a, got, c.want)
		}
	}
}

// TestNetbuild_NoEntryBansEveryFromWay: a no_entry restriction lists every
// way it bars from entering its to way. Only the last from member was
// banned, while the relation still counted as applied.
func TestNetbuild_NoEntryBansEveryFromWay(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0010),    // W
		2: mkNode(2, 40.0010, -74.0005), // N
		3: mkNode(3, 40.0, -74.0000),    // E
		4: mkNode(4, 39.9990, -74.0005), // S
		5: mkNode(5, 40.0, -74.0005),    // junction
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 5),
		mkWay(11, "residential", false, 5, 3),
		mkWay(20, "residential", false, 2, 5),
		mkWay(21, "residential", false, 5, 4),
	}
	feat.Restrictions = []*osm.Relation{{
		ID:   100,
		Tags: osm.Tags{{Key: "type", Value: "restriction"}, {Key: "restriction", Value: "no_entry"}},
		Members: osm.Members{
			{Type: osm.TypeWay, Ref: 10, Role: "from"},
			{Type: osm.TypeWay, Ref: 11, Role: "from"},
			{Type: osm.TypeNode, Ref: 5, Role: "via"},
			{Type: osm.TypeWay, Ref: 20, Role: "to"},
		},
	}}
	net, rpt, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if rpt.RestrictionsApplied != 1 {
		t.Errorf("RestrictionsApplied = %d, want 1", rpt.RestrictionsApplied)
	}
	toN := osmEdge(t, net, feat, 5, 2)
	for _, from := range []osm.NodeID{1, 3} {
		in := osmEdge(t, net, feat, from, 5)
		banned := false
		for _, x := range net.Intersections {
			for _, br := range x.BannedTurns {
				if br.From == in && br.To == toN {
					banned = true
				}
			}
		}
		if !banned {
			t.Errorf("turn %d->5->2 not banned", from)
		}
	}
}

// TestNetbuild_TurnLanesOnlyAtWayEnd: a one-way primary [1, 6, 5] tagged
// turn:lanes=left|through|right for the junction at 5, its end, passes a
// driveway at 6 on the way. The tag was applied at 6 too: the right lane
// could only take the driveway and the left lane matched nothing. At 6 the
// lanes are now inferred from the junction's geometry; at 5 the tag holds.
func TestNetbuild_TurnLanesOnlyAtWayEnd(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0, -74.0020),
		6: mkNode(6, 40.0, -74.0012),    // driveway junction
		7: mkNode(7, 39.9990, -74.0012), // driveway end (south: a right turn)
		5: mkNode(5, 40.0, -74.0005),    // junction the turn lanes are for
		3: mkNode(3, 40.0, -74.0000),
		2: mkNode(2, 40.0010, -74.0005), // N: left
		4: mkNode(4, 39.9990, -74.0005), // S: right
	}}
	approach := mkWay(10, "primary", true, 1, 6, 5)
	approach.Tags = append(approach.Tags,
		osm.Tag{Key: "lanes", Value: "3"},
		osm.Tag{Key: "turn:lanes", Value: "left|through|right"},
	)
	feat.Ways = []*osm.Way{
		approach,
		mkWay(11, "primary", true, 5, 3),
		mkWay(20, "secondary", false, 2, 5, 4),
		mkWay(30, "service", false, 6, 7),
	}
	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	// At the driveway every lane can carry on to 5.
	in6, on := osmEdge(t, net, feat, 1, 6), osmEdge(t, net, feat, 6, 5)
	for li, l := range net.Edges[in6].Lanes {
		if !containsEdge(l.AllowedTurns, on) {
			t.Errorf("lane %d at the driveway: AllowedTurns = %v, want it to include straight on (%d)", li, l.AllowedTurns, on)
		}
	}
	// At 5 the tag applies: lane 0 (rightmost) right only, lane 2 left only.
	lanes := net.Edges[on].Lanes
	if len(lanes) != 3 {
		t.Fatalf("approach to 5 has %d lanes, want 3", len(lanes))
	}
	if r := osmEdge(t, net, feat, 5, 4); len(lanes[0].AllowedTurns) != 1 || lanes[0].AllowedTurns[0] != r {
		t.Errorf("lane 0 at 5: AllowedTurns = %v, want [%d] (right)", lanes[0].AllowedTurns, r)
	}
	if l := osmEdge(t, net, feat, 5, 2); len(lanes[2].AllowedTurns) != 1 || lanes[2].AllowedTurns[0] != l {
		t.Errorf("lane 2 at 5: AllowedTurns = %v, want [%d] (left)", lanes[2].AllowedTurns, l)
	}
}
