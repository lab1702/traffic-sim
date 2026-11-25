package netbuild

import (
	"reflect"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

func TestParseTurnLaneToken(t *testing.T) {
	cases := []struct {
		in   string
		want []network.TurnCategory
	}{
		{"left", []network.TurnCategory{network.TurnLeft}},
		{"slight_left", []network.TurnCategory{network.TurnLeft}},
		{"sharp_left", []network.TurnCategory{network.TurnLeft}},
		{"merge_to_left", []network.TurnCategory{network.TurnLeft}},
		{"right", []network.TurnCategory{network.TurnRight}},
		{"slight_right", []network.TurnCategory{network.TurnRight}},
		{"sharp_right", []network.TurnCategory{network.TurnRight}},
		{"merge_to_right", []network.TurnCategory{network.TurnRight}},
		{"through", []network.TurnCategory{network.TurnStraight}},
		{"none", []network.TurnCategory{network.TurnStraight}},
		{"", []network.TurnCategory{network.TurnStraight}},
		{"through;right", []network.TurnCategory{network.TurnStraight, network.TurnRight}},
		{"left;through;right", []network.TurnCategory{network.TurnLeft, network.TurnStraight, network.TurnRight}},
		{"reverse", nil}, // dropped — U-turns not modeled
		{"floof", nil},   // dropped — unknown
	}
	for _, c := range cases {
		got := parseTurnLaneSpec(c.in)
		if !reflect.DeepEqual(got, c.want) {
			t.Errorf("parseTurnLaneSpec(%q) = %v, want %v", c.in, got, c.want)
		}
	}
}

func TestParseTurnLanesString(t *testing.T) {
	got := parseTurnLanesString("left|through|through;right")
	want := [][]network.TurnCategory{
		{network.TurnLeft},
		{network.TurnStraight},
		{network.TurnStraight, network.TurnRight},
	}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("got %v, want %v", got, want)
	}
}

func TestParseTurnLanesString_Empty(t *testing.T) {
	if got := parseTurnLanesString(""); got != nil {
		t.Errorf("empty input should return nil, got %v", got)
	}
}

func TestGeometricLaneAssignment(t *testing.T) {
	L, S, R := network.TurnLeft, network.TurnStraight, network.TurnRight

	cases := []struct {
		name     string
		cats     []network.TurnCategory // set of categories present (order ignored)
		numLanes int
		want     [][]network.TurnCategory // per-lane allowed categories
	}{
		{
			name: "one-lane gets everything",
			cats: []network.TurnCategory{L, S, R}, numLanes: 1,
			want: [][]network.TurnCategory{{L, S, R}},
		},
		{
			name: "two-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 2,
			want: [][]network.TurnCategory{{R, S}, {L, S}},
		},
		{
			name: "three-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 3,
			want: [][]network.TurnCategory{{R}, {S}, {L}},
		},
		{
			name: "four-lane with L+S+R",
			cats: []network.TurnCategory{L, S, R}, numLanes: 4,
			want: [][]network.TurnCategory{{R}, {S}, {S}, {L}},
		},
		{
			name: "two-lane with S+R only",
			cats: []network.TurnCategory{S, R}, numLanes: 2,
			want: [][]network.TurnCategory{{R, S}, {S}},
		},
		{
			name: "two-lane with S+L only",
			cats: []network.TurnCategory{S, L}, numLanes: 2,
			want: [][]network.TurnCategory{{S}, {L, S}},
		},
		{
			name: "two-lane with single category (straight)",
			cats: []network.TurnCategory{S}, numLanes: 2,
			want: [][]network.TurnCategory{{S}, {S}},
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got := assignLanesGeometric(c.cats, c.numLanes)
			if !equalAssignments(got, c.want) {
				t.Errorf("got %v, want %v", got, c.want)
			}
		})
	}
}

// buildTestIntersection constructs a simple intersection: incoming edge 0
// ends at node 0, with three outgoing edges 1 (right), 2 (straight), 3 (left).
// Lane count on the incoming edge is configurable.
func buildTestIntersection(numLanes int) *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: -100, Y: 0}},  // upstream of incoming
		{ID: 2, Pos: network.Point{X: 100, Y: -100}}, // right destination
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},    // straight destination
		{ID: 4, Pos: network.Point{X: 100, Y: 100}},  // left destination
	}
	lanes := make([]network.Lane, numLanes)
	for i := range lanes {
		lanes[i] = network.Lane{Index: uint8(i)}
	}
	edges := []network.Edge{
		// Incoming: node 1 -> 0, heading east (+X).
		{ID: 0, From: 1, To: 0, Length: 100, SpeedLimit: 10, Lanes: lanes,
			Geometry: []network.Point{nodes[1].Pos, nodes[0].Pos}},
		// Outgoing right: 0 -> 2, heading SE (then south).
		{ID: 1, From: 0, To: 2, Length: 140, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[0].Pos, nodes[2].Pos}},
		// Outgoing straight: 0 -> 3, heading east.
		{ID: 2, From: 0, To: 3, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[0].Pos, nodes[3].Pos}},
		// Outgoing left: 0 -> 4, heading NE (then north).
		{ID: 3, From: 0, To: 4, Length: 140, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[0].Pos, nodes[4].Pos}},
	}
	intersections := []network.Intersection{
		{ID: 0, NodeID: 0,
			Incoming: []network.EdgeID{0},
			Outgoing: []network.EdgeID{1, 2, 3},
		},
	}
	return &network.Network{Nodes: nodes, Edges: edges, Intersections: intersections}
}

func TestAssignAllowedTurnsForEdge_GeometricFallback_TwoLanes(t *testing.T) {
	net := buildTestIntersection(2)
	// No OSM way ID for incoming → geometric fallback.
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], nil)

	// Expected: lane 0 = {right, straight}, lane 1 = {left, straight}.
	if len(allowed) != 2 {
		t.Fatalf("want 2 lane assignments, got %d", len(allowed))
	}
	if !containsEdge(allowed[0], 1) || !containsEdge(allowed[0], 2) {
		t.Errorf("lane 0 should include right (1) and straight (2), got %v", allowed[0])
	}
	if containsEdge(allowed[0], 3) {
		t.Errorf("lane 0 should NOT include left (3), got %v", allowed[0])
	}
	if !containsEdge(allowed[1], 3) || !containsEdge(allowed[1], 2) {
		t.Errorf("lane 1 should include left (3) and straight (2), got %v", allowed[1])
	}
	if containsEdge(allowed[1], 1) {
		t.Errorf("lane 1 should NOT include right (1), got %v", allowed[1])
	}
}

func TestAssignAllowedTurnsForEdge_GeometricFallback_ThreeLanes(t *testing.T) {
	net := buildTestIntersection(3)
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], nil)

	// Expected: lane 0 = {right}, lane 1 = {straight}, lane 2 = {left}.
	if !containsEdge(allowed[0], 1) || containsEdge(allowed[0], 2) || containsEdge(allowed[0], 3) {
		t.Errorf("lane 0 wrong: %v", allowed[0])
	}
	if containsEdge(allowed[1], 1) || !containsEdge(allowed[1], 2) || containsEdge(allowed[1], 3) {
		t.Errorf("lane 1 wrong: %v", allowed[1])
	}
	if containsEdge(allowed[2], 1) || containsEdge(allowed[2], 2) || !containsEdge(allowed[2], 3) {
		t.Errorf("lane 2 wrong: %v", allowed[2])
	}
}

func TestAssignAllowedTurnsForEdge_OSMOverride(t *testing.T) {
	net := buildTestIntersection(2)
	// OSM lists turn:lanes from LEFT to RIGHT in the direction of travel:
	// leftmost lane = left only, rightmost lane = through+right. Lane 0 is
	// the rightmost (curb) lane, so lane 0 = through+right, lane 1 = left.
	spec := parseTurnLanesString("left|through;right")
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], spec)

	if !containsEdge(allowed[0], 1) || !containsEdge(allowed[0], 2) {
		t.Errorf("lane 0 should have right (1) and straight (2), got %v", allowed[0])
	}
	if !containsEdge(allowed[1], 3) {
		t.Errorf("lane 1 should have left (3), got %v", allowed[1])
	}
	if containsEdge(allowed[1], 2) {
		t.Errorf("lane 1 should NOT have straight (2) per explicit OSM, got %v", allowed[1])
	}
}

// TestAssignAllowedTurnsForEdge_OSMOrderIsLeftToRight: a 3-lane approach
// tagged turn:lanes=left|through|right (OSM order: leftmost lane first).
// Lane 0 is the rightmost lane, so it must get the right turn and lane 2 the
// left turn — not the other way round.
func TestAssignAllowedTurnsForEdge_OSMOrderIsLeftToRight(t *testing.T) {
	net := buildTestIntersection(3)
	spec := parseTurnLanesString("left|through|right")
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], spec)

	want := [][]network.EdgeID{{1}, {2}, {3}} // right, straight, left
	if !reflect.DeepEqual(allowed, want) {
		t.Errorf("per-lane AllowedTurns = %v, want %v (lane 0 = rightmost = right turn)", allowed, want)
	}
}

// TestBuild_TurnLanesForwardBackward_LeftToRight: a two-way primary (2 lanes
// each way) crossing another primary, with different turn:lanes:forward and
// turn:lanes:backward. Each direction's spec is listed left-to-right relative
// to ITS OWN direction of travel, and lane 0 is the rightmost lane of each
// edge, so both directions are reversed when copied into Lanes.
//
// The primary is split at the junction, as mappers tag turn lanes: a way's
// turn:lanes apply where it ends in each direction of travel, so the
// eastbound arrows go on the west way's forward direction and the westbound
// ones on the east way's backward direction.
func TestBuild_TurnLanesForwardBackward_LeftToRight(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W
		2: mkNode(2, 40.0010, -74.0005), // N
		3: mkNode(3, 40.0000, -74.0000), // E
		4: mkNode(4, 39.9990, -74.0005), // S
		5: mkNode(5, 40.0000, -74.0005), // junction
	}}
	west := mkWay(10, "primary", false, 1, 5) // forward = eastbound, ends at 5
	west.Tags = append(west.Tags,
		osm.Tag{Key: "lanes", Value: "4"},
		// Eastbound: left lane turns left (N); right lane goes through or right (S).
		osm.Tag{Key: "turn:lanes:forward", Value: "left|through;right"},
	)
	east := mkWay(11, "primary", false, 5, 3) // backward = westbound, ends at 5
	east.Tags = append(east.Tags,
		osm.Tag{Key: "lanes", Value: "4"},
		// Westbound: left lane turns left (S) or goes through; right lane turns right (N).
		osm.Tag{Key: "turn:lanes:backward", Value: "left;through|right"},
	)
	feat.Ways = []*osm.Way{west, east, mkWay(20, "primary", false, 2, 5, 4)}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	osmToNetT := make(map[osm.NodeID]network.NodeID)
	for nid, oid := range buildNetToOSM(net, feat) {
		osmToNetT[oid] = nid
	}
	edgeBetween := func(from, to osm.NodeID) network.EdgeID {
		t.Helper()
		for _, e := range net.Edges {
			if e.From == osmToNetT[from] && e.To == osmToNetT[to] {
				return e.ID
			}
		}
		t.Fatalf("no edge %d->%d", from, to)
		return 0
	}
	toN, toE, toS, toW := edgeBetween(5, 2), edgeBetween(5, 3), edgeBetween(5, 4), edgeBetween(5, 1)

	check := func(name string, in network.EdgeID, want [][]network.EdgeID) {
		t.Helper()
		lanes := net.Edges[in].Lanes
		if len(lanes) != len(want) {
			t.Fatalf("%s: want %d lanes, got %d", name, len(want), len(lanes))
		}
		for i := range want {
			got := lanes[i].AllowedTurns
			if len(got) != len(want[i]) {
				t.Errorf("%s lane %d: AllowedTurns = %v, want %v", name, i, got, want[i])
				continue
			}
			for _, e := range want[i] {
				if !containsEdge(got, e) {
					t.Errorf("%s lane %d: AllowedTurns = %v, want %v", name, i, got, want[i])
					break
				}
			}
		}
	}
	// Eastbound (forward): lane 0 (rightmost, south side) = through + right(S);
	// lane 1 (leftmost) = left(N) only.
	check("eastbound", edgeBetween(1, 5), [][]network.EdgeID{{toE, toS}, {toN}})
	// Westbound (backward): lane 0 (rightmost, north side) = right(N) only;
	// lane 1 (leftmost) = left(S) + through(W).
	check("westbound", edgeBetween(3, 5), [][]network.EdgeID{{toN}, {toS, toW}})
}

func TestAssignAllowedTurnsForEdge_BannedTurnExcluded(t *testing.T) {
	net := buildTestIntersection(2)
	// Ban the left turn (0 -> 3).
	net.Intersections[0].BannedTurns = []network.TurnRestriction{
		{From: 0, To: 3},
	}
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], nil)

	for i, lane := range allowed {
		if containsEdge(lane, 3) {
			t.Errorf("lane %d should not include banned edge 3, got %v", i, lane)
		}
	}
}

func TestAssignAllowedTurnsForEdge_AllOutgoingReachable(t *testing.T) {
	net := buildTestIntersection(3)
	allowed := assignAllowedTurnsForEdge(net, 0, &net.Intersections[0], nil)
	for _, want := range []network.EdgeID{1, 2, 3} {
		reachable := false
		for _, lane := range allowed {
			if containsEdge(lane, want) {
				reachable = true
				break
			}
		}
		if !reachable {
			t.Errorf("edge %d unreachable from any lane", want)
		}
	}
}

// equalAssignments compares two per-lane assignments treating each lane's
// category list as an unordered set.
func equalAssignments(a, b [][]network.TurnCategory) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if !sameSet(a[i], b[i]) {
			return false
		}
	}
	return true
}

func sameSet(a, b []network.TurnCategory) bool {
	if len(a) != len(b) {
		return false
	}
	seen := map[network.TurnCategory]int{}
	for _, x := range a {
		seen[x]++
	}
	for _, x := range b {
		seen[x]--
		if seen[x] < 0 {
			return false
		}
	}
	return true
}
