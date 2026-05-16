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
//
// Handles both simple edges (From and To are adjacent in the way) and edges
// with interior shaping nodes (From and To appear in the same way but may not
// be directly adjacent).
func highwayOfEdge(net *network.Network, eid network.EdgeID, feat *osmload.Features) string {
	e := net.Edges[eid]
	netToOSM := buildNetToOSM(net, feat)
	fromOSM, fromOk := netToOSM[e.From]
	toOSM, toOk := netToOSM[e.To]
	if !fromOk || !toOk {
		return ""
	}
	for _, w := range feat.Ways {
		// First try adjacent pairs (common case — no interior shaping nodes).
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
		// Fall back: both endpoints appear somewhere in the way (edge has
		// interior shaping nodes, so From and To are not adjacent).
		hasFrom, hasTo := false, false
		for _, n := range w.Nodes {
			if n.ID == fromOSM {
				hasFrom = true
			}
			if n.ID == toOSM {
				hasTo = true
			}
		}
		if hasFrom && hasTo {
			for _, t := range w.Tags {
				if t.Key == "highway" {
					return t.Value
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

// TestNetbuild_Opposing_CoSortsWithIncoming: force a non-trivial
// priority sort and verify that Opposing indices are correctly
// remapped to point at the new positions. Uses an asymmetric Opposing
// pattern to detect when the remap is missing (unlike a symmetric
// pattern, which produces the same result whether remapped or not).
func TestNetbuild_Opposing_CoSortsWithIncoming(t *testing.T) {
	// 4-way: a "service" road (lower priority) meets a "primary" road.
	// Pre-sort, Incoming order is whatever buildIntersections produces.
	// Post-sort, the primary approaches must be at positions 0 and 1.
	// Manually set Opposing on a fixture to verify remapping.
	xs := []network.Intersection{
		{
			ID:       0,
			NodeID:   0,
			Incoming: []network.EdgeID{0, 1, 2, 3},
			Opposing: []int8{2, -1, -1, 0}, // asymmetric pattern; only approaches 0 and 2 are paired
		},
	}
	// osmWayOfEdge[i] = WayID, so we can fake highway priorities via the
	// way tags. Edges 0, 1 are on a service road; 2, 3 on a primary.
	feat := &osmload.Features{}
	feat.Ways = []*osm.Way{
		{ID: 100, Tags: []osm.Tag{{Key: "highway", Value: "service"}}},
		{ID: 200, Tags: []osm.Tag{{Key: "highway", Value: "primary"}}},
	}
	osmWayOfEdge := []osm.WayID{100, 100, 200, 200}

	sortIncomingByPriority(xs, osmWayOfEdge, feat)

	x := xs[0]
	// Verify Incoming is now [2, 3, 0, 1] (primary first, then service).
	wantIncoming := []network.EdgeID{2, 3, 0, 1}
	for i := range wantIncoming {
		if x.Incoming[i] != wantIncoming[i] {
			t.Errorf("Incoming[%d] = %d, want %d", i, x.Incoming[i], wantIncoming[i])
		}
	}
	// Verify Opposing was remapped:
	//   Pre-sort: Incoming=[0,1,2,3], Opposing=[2,-1,-1,0]
	//   Post-sort: Incoming=[2,3,0,1]  (sort swaps the primary/service pairs)
	//   oldToNew = {0:2, 1:3, 2:0, 3:1}.
	//   New Opposing[newI] = oldToNew[old Opposing[oldI]] (with -1 passthrough):
	//     - newI=0 was oldI=2; oldOpposing[2]=-1; passes through -> -1.
	//     - newI=1 was oldI=3; oldOpposing[3]=0;  remapped to oldToNew[0]=2.
	//     - newI=2 was oldI=0; oldOpposing[0]=2;  remapped to oldToNew[2]=0.
	//     - newI=3 was oldI=1; oldOpposing[1]=-1; passes through -> -1.
	wantOpposing := []int8{-1, 2, 0, -1}
	for i := range wantOpposing {
		if x.Opposing[i] != wantOpposing[i] {
			t.Errorf("Opposing[%d] = %d, want %d", i, x.Opposing[i], wantOpposing[i])
		}
	}
}

// TestNetbuild_Opposing_FourWay: a 4-way + intersection. The N and S
// approaches pair; the E and W approaches pair.
func TestNetbuild_Opposing_FourWay(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0010, -74.0005), // N origin
		2: mkNode(2, 40.0000, -74.0010), // W origin
		3: mkNode(3, 40.0000, -74.0005), // center
		4: mkNode(4, 40.0000, -74.0000), // E origin (approaches center from east)
		5: mkNode(5, 39.9990, -74.0005), // S origin
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 3, 5), // N-S road
		mkWay(20, "residential", false, 2, 3, 4), // W-E road
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]
	if len(x.Opposing) != len(x.Incoming) {
		t.Fatalf("Opposing length %d != Incoming length %d", len(x.Opposing), len(x.Incoming))
	}
	// Every approach must have an opposing approach in a 4-way.
	for i := range x.Incoming {
		if x.Opposing[i] < 0 {
			t.Errorf("approach %d (edge %d) has no opposing", i, x.Incoming[i])
		}
	}
	// Symmetry.
	for i := range x.Incoming {
		j := int(x.Opposing[i])
		if j < 0 {
			continue
		}
		if int(x.Opposing[j]) != i {
			t.Errorf("non-symmetric: Opposing[%d]=%d but Opposing[%d]=%d", i, j, j, x.Opposing[j])
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

// TestNetbuild_Opposing_TThrough: T-intersection. The two through-road
// approaches pair with each other; the stem approach gets -1.
func TestNetbuild_Opposing_TThrough(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W origin (through)
		2: mkNode(2, 40.0000, -74.0005), // center
		3: mkNode(3, 40.0000, -74.0000), // E origin (through)
		4: mkNode(4, 39.9990, -74.0005), // S origin (stem)
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
	// Three approaches: two through (W-arrival and E-arrival) and one
	// stem (N-arrival from south, i.e., heading north into the
	// intersection). The two through ones must pair; the stem must
	// have Opposing = -1.
	pairedCount := 0
	stemCount := 0
	for i := range x.Incoming {
		if x.Opposing[i] >= 0 {
			pairedCount++
		} else {
			stemCount++
		}
	}
	if pairedCount != 2 {
		t.Errorf("want 2 paired approaches, got %d", pairedCount)
	}
	if stemCount != 1 {
		t.Errorf("want 1 unpaired stem approach, got %d", stemCount)
	}
}

// TestNetbuild_DirectionForward: a 4-way crossing of two equal-class
// primaries carries `highway=stop direction=forward` on the intersection
// node. Two behaviors compose here:
//
//   1. Spec's "over-apply at multi-way intersections": a directional
//      tag applies to ALL approaches whose direction-on-their-way is
//      forward, across every way passing through the node. That's two
//      forward approaches (one per way).
//
//   2. Directional-tag override of class-inferred AllWayStop: equal
//      classes produce AllWayStop via class fallback. The directional
//      branch of applyNodeLevelSign deliberately does NOT skip
//      AllWayStop (unlike the non-directional branch), because an
//      explicit mapper-set direction tag conveys more specific intent
//      than class inference. So both forward approaches transition
//      from AllWayStop to Stop.
//
// Expected result: stopCount == 2.
func TestNetbuild_DirectionForward(t *testing.T) {
	// Layout (planar approximation):
	//   N is at lat 40.0010, S at 39.9990 → "lower index first" way
	//   means we list S, X, N as the node sequence; vehicle going from
	//   S to X (heading N) is moving "forward" along the way.
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 39.9990, -74.0005), // S origin (N-S way, forward)
		2: mkNode(2, 40.0010, -74.0005), // N origin (N-S way, backward)
		3: mkNode(3, 40.0000, -74.0010), // W origin (E-W way)
		4: mkNode(4, 40.0000, -74.0000), // E origin
		5: mkNode(5, 40.0000, -74.0005, "highway", "stop", "direction", "forward"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 2), // N-S way: forward = S→N
		mkWay(20, "primary", false, 3, 5, 4), // E-W way: forward = W→E
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]

	stopCount := 0
	for i := range x.Incoming {
		if x.IncomingControl[i] == network.ControlStop {
			stopCount++
		}
	}
	if stopCount != 2 {
		t.Errorf("direction=forward at multi-way intersection should mark BOTH forward-direction approaches as Stop, got %d", stopCount)
		for i := range x.Incoming {
			t.Logf("  Incoming[%d] edge=%d control=%v", i, x.Incoming[i], x.IncomingControl[i])
		}
	}
}

// TestNetbuild_Opposing_Symmetric: across a mixed-geometry network,
// Opposing[Opposing[i]] == i whenever Opposing[i] != -1.
func TestNetbuild_Opposing_Symmetric(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		// Two adjacent 4-ways sharing a node.
		1: mkNode(1, 40.0020, -74.0005),
		2: mkNode(2, 40.0010, -74.0010),
		3: mkNode(3, 40.0010, -74.0005), // first center
		4: mkNode(4, 40.0010, -74.0000),
		5: mkNode(5, 40.0000, -74.0005), // second center
		6: mkNode(6, 40.0000, -74.0010),
		7: mkNode(7, 40.0000, -74.0000),
		8: mkNode(8, 39.9990, -74.0005),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "residential", false, 1, 3, 5, 8), // N-S spine
		mkWay(20, "residential", false, 2, 3, 4),    // first E-W
		mkWay(30, "residential", false, 6, 5, 7),    // second E-W
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	for xi, x := range net.Intersections {
		for i := range x.Incoming {
			j := int(x.Opposing[i])
			if j < 0 {
				continue
			}
			back := int(x.Opposing[j])
			if back != i {
				t.Errorf("intersection %d: Opposing[%d]=%d but Opposing[%d]=%d (not symmetric)", xi, i, j, j, back)
			}
		}
	}
}

// TestNetbuild_DirectionBackward: same fixture and same composed
// behaviors as TestNetbuild_DirectionForward, but with `direction=backward`.
// The two backward-direction approaches (one per way) transition from
// AllWayStop (from class fallback) to Stop (from the directional tag).
func TestNetbuild_DirectionBackward(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 39.9990, -74.0005),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0010),
		4: mkNode(4, 40.0000, -74.0000),
		5: mkNode(5, 40.0000, -74.0005, "highway", "stop", "direction", "backward"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 2),
		mkWay(20, "primary", false, 3, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	stopCount := 0
	for i := range x.Incoming {
		if x.IncomingControl[i] == network.ControlStop {
			stopCount++
		}
	}
	if stopCount != 2 {
		t.Errorf("direction=backward at multi-way intersection should mark 2 approaches as Stop, got %d", stopCount)
		for i := range x.Incoming {
			t.Logf("  Incoming[%d] edge=%d control=%v", i, x.Incoming[i], x.IncomingControl[i])
		}
	}
}

// TestNetbuild_InteriorNodeStop: a primary way has an interior shaping
// node tagged `highway=stop`. The approach edge whose geometry contains
// that node should get ControlStop, overriding the class-fallback
// ControlNone that a primary approach would otherwise have.
func TestNetbuild_InteriorNodeStop(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010), // W primary endpoint
		2: mkNode(2, 40.0010, -74.0005), // N service endpoint
		3: mkNode(3, 40.0000, -74.0000), // E primary endpoint
		4: mkNode(4, 39.9990, -74.0005), // S service endpoint
		5: mkNode(5, 40.0000, -74.0005), // intersection
		6: mkNode(6, 40.0000, -74.0008, "highway", "stop"), // interior on W approach
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	if len(net.Intersections) != 1 {
		t.Fatalf("want 1 intersection, got %d", len(net.Intersections))
	}
	x := net.Intersections[0]

	// Identify each approach by its geometric direction relative to the
	// intersection and by the highway class of its source way. The interior
	// stop tag is on the W primary approach specifically — the E primary
	// must stay ControlNone, and service approaches get class-fallback Stop.
	wantApproach(t, net, feat, x, "primary", "W", network.ControlStop)
	wantApproach(t, net, feat, x, "primary", "E", network.ControlNone)
	wantApproach(t, net, feat, x, "service", "N", network.ControlStop)
	wantApproach(t, net, feat, x, "service", "S", network.ControlStop)
}

// approachDir returns "N"/"S"/"E"/"W" for the approach edge based on which
// cardinal direction its From-node lies from the intersection's node.
// Used by tightened tests to identify a specific approach rather than just
// "some approach with this highway class".
func approachDir(net *network.Network, eid network.EdgeID, intersectionNode network.NodeID) string {
	if int(eid) >= len(net.Edges) {
		return ""
	}
	e := &net.Edges[eid]
	var from, via network.Point
	for i := range net.Nodes {
		if net.Nodes[i].ID == e.From {
			from = net.Nodes[i].Pos
		}
		if net.Nodes[i].ID == intersectionNode {
			via = net.Nodes[i].Pos
		}
	}
	dx := from.X - via.X
	dy := from.Y - via.Y
	if absf(dx) > absf(dy) {
		if dx < 0 {
			return "W"
		}
		return "E"
	}
	if dy < 0 {
		return "S"
	}
	return "N"
}

func absf(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

// wantApproach asserts that exactly one incoming approach matches
// (highway class, geometric direction) and has the expected Control.
// Used by interior-node and intersection-node sign tests to verify
// per-approach correctness, not just "some approach got Stop".
func wantApproach(t *testing.T, net *network.Network, feat *osmload.Features,
	x network.Intersection, hwClass, dir string, want network.Control,
) {
	t.Helper()
	matches := 0
	for i, eid := range x.Incoming {
		if highwayOfEdge(net, eid, feat) != hwClass {
			continue
		}
		if approachDir(net, eid, x.NodeID) != dir {
			continue
		}
		matches++
		if got := x.IncomingControl[i]; got != want {
			t.Errorf("approach (hw=%s, dir=%s, edge=%d): want %v, got %v", hwClass, dir, eid, want, got)
		}
	}
	if matches == 0 {
		t.Errorf("no incoming approach matches (hw=%s, dir=%s) — fixture or test geometry mismatch", hwClass, dir)
	}
	if matches > 1 {
		t.Errorf("ambiguous: %d incoming approaches match (hw=%s, dir=%s)", matches, hwClass, dir)
	}
}

// TestNetbuild_DirectionMissingStillLenient: when direction= tag is
// absent, the Phase 1 lenient behavior is preserved — sign applies to
// every approach (subject to the AllWayStop skip guard for
// non-directional signs).
func TestNetbuild_DirectionMissingStillLenient(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 39.9990, -74.0005),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0010),
		4: mkNode(4, 40.0000, -74.0000),
		5: mkNode(5, 40.0000, -74.0005, "highway", "stop"), // no direction
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 5, 2),
		mkWay(20, "primary", false, 3, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	// Without direction tag, the non-directional applyNodeLevelSign
	// branch fires. With equal-class primaries, class-fallback first
	// sets all 4 approaches to AllWayStop. The non-directional branch
	// skips AllWayStop approaches, so they stay AllWayStop.
	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("approach %d should be AllWayStop (non-directional sign skips AllWayStop), got %v", i, c)
		}
	}
}

// TestNetbuild_InteriorNodeGiveWay: same shape as InteriorNodeStop but
// the interior tag is highway=give_way → ControlYield.
func TestNetbuild_InteriorNodeGiveWay(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005),
		6: mkNode(6, 40.0000, -74.0008, "highway", "give_way"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	// Interior give_way is on the W primary approach. E primary stays None
	// (no sign on its segment); service approaches get class-fallback Stop.
	wantApproach(t, net, feat, x, "primary", "W", network.ControlYield)
	wantApproach(t, net, feat, x, "primary", "E", network.ControlNone)
	wantApproach(t, net, feat, x, "service", "N", network.ControlStop)
	wantApproach(t, net, feat, x, "service", "S", network.ControlStop)
}

// TestNetbuild_InteriorNodeOverridesIntersectionNode: intersection node
// has highway=give_way (which would set Yield on all approaches via
// Section 1) AND one approach has an interior highway=stop. The
// approach with the interior sign gets Stop (interior wins); other
// approaches get Yield.
func TestNetbuild_InteriorNodeOverridesIntersectionNode(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005, "highway", "give_way"),  // intersection give_way
		6: mkNode(6, 40.0000, -74.0008, "highway", "stop"),      // interior on W approach
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	// W primary has the interior highway=stop → Stop (interior wins over the
	// intersection-node give_way). Every other approach gets Yield from the
	// intersection-node tag.
	wantApproach(t, net, feat, x, "primary", "W", network.ControlStop)
	wantApproach(t, net, feat, x, "primary", "E", network.ControlYield)
	wantApproach(t, net, feat, x, "service", "N", network.ControlYield)
	wantApproach(t, net, feat, x, "service", "S", network.ControlYield)
}

// TestNetbuild_InteriorNodeDoesNotDowngradeAllWayStop: stop=all on the
// intersection node promotes every approach to ControlAllWayStop. An
// interior highway=give_way on one approach must NOT downgrade it —
// the AllWayStop skip-guard protects the strictest control from
// being weakened.
func TestNetbuild_InteriorNodeDoesNotDowngradeAllWayStop(t *testing.T) {
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0010),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005, "stop", "all"),
		6: mkNode(6, 40.0000, -74.0008, "highway", "give_way"),
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	for i, c := range x.IncomingControl {
		if c != network.ControlAllWayStop {
			t.Errorf("approach %d should be AllWayStop (stop=all), got %v", i, c)
		}
	}
}

// TestNetbuild_InteriorNodeClosestToXWins: an approach has TWO sign-
// tagged interior nodes. The one geographically closer to X (the
// intersection) wins — the walk starts at xIdx and steps toward fromIdx,
// returning the first match.
func TestNetbuild_InteriorNodeClosestToXWins(t *testing.T) {
	// Way: 1 (start) -> 7 (far interior, highway=stop) -> 6 (near
	// interior, highway=give_way) -> 5 (intersection) -> 3.
	// The W approach should get ControlYield (node 6 is closer to 5).
	feat := &osmload.Features{Nodes: map[osm.NodeID]*osm.Node{
		1: mkNode(1, 40.0000, -74.0020),
		2: mkNode(2, 40.0010, -74.0005),
		3: mkNode(3, 40.0000, -74.0000),
		4: mkNode(4, 39.9990, -74.0005),
		5: mkNode(5, 40.0000, -74.0005),
		6: mkNode(6, 40.0000, -74.0008, "highway", "give_way"), // closer to 5
		7: mkNode(7, 40.0000, -74.0015, "highway", "stop"),     // farther from 5
	}}
	feat.Ways = []*osm.Way{
		mkWay(10, "primary", false, 1, 7, 6, 5, 3),
		mkWay(20, "service", false, 2, 5, 4),
	}

	net, _, err := Build(feat)
	if err != nil {
		t.Fatalf("Build: %v", err)
	}
	x := net.Intersections[0]

	var sawPrimaryYield bool
	for i, eid := range x.Incoming {
		hw := highwayOfEdge(net, eid, feat)
		c := x.IncomingControl[i]
		if hw == "primary" && c == network.ControlYield {
			sawPrimaryYield = true
		}
		if hw == "primary" && c == network.ControlStop {
			t.Errorf("primary approach should be Yield (closest interior tag wins), got Stop")
		}
	}
	if !sawPrimaryYield {
		t.Error("primary approach should be ControlYield from closer interior node")
	}
}
