package netbuild

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// resolveControls fills in IncomingControl for every intersection in xs.
// Runs after sortIncomingByPriority, so IncomingControl[i] is the rule
// for the approach now at Incoming[i] (final sorted position).
//
// Nodes that merely continue one road (fewer than three distinct
// neighbors, no signal) are skipped entirely and stay uncontrolled — a
// way-join is not a junction.
//
// Resolution order (first rule that applies wins for a given approach):
//  1. stop=all on the intersection node      -> AllWayStop everywhere.
//  2. stop=minor on the intersection node    -> Stop on every approach
//     whose highway class is strictly lower-priority than the best.
//  3. Class-based fallback (terminating road yields): the through road
//     keeps priority (None); terminating stems and lower-class approaches
//     Yield; a genuine equal-class crossing or ambiguous Y is AllWayStop.
//
// Task 13 adds rule for highway=stop / highway=give_way on the
// intersection node.
func resolveControls(
	xs []network.Intersection,
	feat *osmload.Features,
	osmWayOfEdge []osm.WayID,
	osmNodeOf func(network.NodeID) (osm.NodeID, bool),
	edges []network.Edge,
) {
	wayByID := make(map[osm.WayID]*osm.Way, len(feat.Ways))
	for _, w := range feat.Ways {
		wayByID[w.ID] = w
	}
	classOfEdge := func(eid network.EdgeID) int {
		if int(eid) >= len(osmWayOfEdge) {
			return 100
		}
		w, ok := wayByID[osmWayOfEdge[eid]]
		if !ok || w == nil {
			return 100
		}
		for _, t := range w.Tags {
			if t.Key == "highway" {
				return highwayPriority(t.Value)
			}
		}
		return 100
	}

	edgeFromOSM := func(eid network.EdgeID) (osm.NodeID, bool) {
		if int(eid) >= len(edges) {
			return 0, false
		}
		return osmNodeOf(edges[eid].From)
	}

	for i := range xs {
		x := &xs[i]

		// A node connecting only two distinct neighbors is a way-join — one
		// physical road continuing through a tag boundary (speed/name/bridge/
		// surface change) or around a bend, not a real junction. It carries
		// no cross traffic, so it must impose no right-of-way control:
		// otherwise the equal-class fallback below makes it an all-way stop
		// and cars halt at an invisible point on a straight road. Signalled
		// nodes are exempt (a mid-block / pedestrian signal legitimately
		// stops a two-approach node). Genuine junctions, merges, and diverges
		// always touch ≥3 distinct neighbors. Leave IncomingControl at its
		// ControlNone default and skip the resolution chain.
		if !x.HasSignal && distinctNeighbors(x, edges) < 3 {
			continue
		}

		var nodeTags osm.Tags
		var xOSMID osm.NodeID
		if osmID, ok := osmNodeOf(x.NodeID); ok {
			xOSMID = osmID
			if n, ok2 := feat.Nodes[osmID]; ok2 && n != nil {
				nodeTags = n.Tags
			}
		}

		// Start from the class-based fallback, then layer explicit OSM
		// signage on top (Task 12 covers fallback + stop=all/stop=minor;
		// Task 13 adds node-level highway=stop / give_way).
		applyClassFallback(x, classOfEdge)
		applyStopAllOrMinor(x, nodeTags, classOfEdge)
		applyNodeLevelSign(x, nodeTags, wayByID, osmWayOfEdge, edgeFromOSM, xOSMID)
		applyInteriorNodeSign(x, wayByID, osmWayOfEdge, edgeFromOSM, xOSMID, feat.Nodes)
	}
}

// distinctNeighbors counts the distinct other-endpoint nodes reachable
// across all edges incident to the intersection (incoming edges' From
// nodes plus outgoing edges' To nodes). A pure road continuation touches
// exactly two; a genuine junction, merge, or diverge touches three or
// more. Self-loops (an edge that starts and ends at the node) are ignored.
func distinctNeighbors(x *network.Intersection, edges []network.Edge) int {
	nb := make(map[network.NodeID]struct{}, 4)
	for _, eid := range x.Incoming {
		if int(eid) < len(edges) {
			nb[edges[eid].From] = struct{}{}
		}
	}
	for _, eid := range x.Outgoing {
		if int(eid) < len(edges) {
			nb[edges[eid].To] = struct{}{}
		}
	}
	delete(nb, x.NodeID)
	return len(nb)
}

// applyClassFallback assigns right-of-way for an unsigned junction using the
// "terminating road yields" model. The road that continues straight through
// the junction (the through road) keeps priority; roads that terminate at it,
// and lower-class roads, give way. An all-way stop is reserved for the one
// case with genuine, class-symmetric conflict: two or more equal-class through
// roads actually crossing (or an ambiguous equal-class junction with no
// through road at all).
//
// "Through" is read from x.Opposing, populated by resolveOpposing before this
// runs: an approach with an opposing partner is part of a road that continues
// across the junction; one without is a stem.
//
// Explicit OSM signage (handled by the apply* functions that run after this)
// still overrides these defaults.
func applyClassFallback(x *network.Intersection, classOfEdge func(network.EdgeID) int) {
	n := len(x.Incoming)
	if n == 0 {
		return
	}
	best := classOfEdge(x.Incoming[0])
	for _, eid := range x.Incoming[1:] {
		if c := classOfEdge(eid); c < best {
			best = c
		}
	}

	isBest := make([]bool, n)
	through := make([]bool, n)
	nBest, nThrough := 0, 0
	for j, eid := range x.Incoming {
		if classOfEdge(eid) != best {
			continue
		}
		isBest[j] = true
		nBest++
		// A best-class approach is "through" when it has an opposing partner
		// that is itself best-class — i.e. its road continues straight across
		// the junction rather than terminating at it.
		if p := x.Opposing[j]; p >= 0 && int(p) < n && classOfEdge(x.Incoming[p]) == best {
			through[j] = true
			nThrough++
		}
	}

	// A single best-class approach (e.g. a higher-class stem) simply has
	// priority over everything else.
	if nBest <= 1 {
		for j := range x.IncomingControl {
			if isBest[j] {
				x.IncomingControl[j] = network.ControlNone
			} else {
				x.IncomingControl[j] = network.ControlYield
			}
		}
		return
	}

	// nThrough counts approaches in through pairs; nThrough/2 is the number of
	// crossing axes. Two or more axes (a real equal-class crossing), or no
	// through road among several equal arms (an ambiguous Y), is the only case
	// that warrants an all-way stop.
	if nThrough/2 >= 2 || nThrough == 0 {
		for j := range x.IncomingControl {
			x.IncomingControl[j] = network.ControlAllWayStop
		}
		return
	}

	// Exactly one through road of the best class: it keeps priority; every
	// terminating best-class stem and every lower-class approach yields.
	for j := range x.IncomingControl {
		if through[j] {
			x.IncomingControl[j] = network.ControlNone
		} else {
			x.IncomingControl[j] = network.ControlYield
		}
	}
}

// applyNodeLevelSign handles highway=stop and highway=give_way tags on
// the intersection node. A direction= tag refines which approaches it
// applies to:
//   - direction=forward: only approaches whose direction-on-way is forward.
//   - direction=backward: only approaches whose direction-on-way is backward.
//   - no direction tag: all approaches (Phase 1 lenient behavior).
//
// Skips approaches already promoted to ControlAllWayStop when no
// direction tag is present. When direction= is set, the explicit sign
// takes precedence over the class-based AllWayStop for the matched
// approach.
func applyNodeLevelSign(
	x *network.Intersection,
	tags osm.Tags,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
	xOSMID osm.NodeID,
) {
	var target network.Control
	hasSign := false
	direction := ""
	for _, t := range tags {
		if t.Key == "highway" && t.Value == "stop" {
			target, hasSign = network.ControlStop, true
		}
		if t.Key == "highway" && t.Value == "give_way" {
			target, hasSign = network.ControlYield, true
		}
		if t.Key == "direction" && (t.Value == "forward" || t.Value == "backward") {
			direction = t.Value
		}
	}
	if !hasSign {
		return
	}
	if direction == "" {
		for j := range x.IncomingControl {
			if x.IncomingControl[j] == network.ControlAllWayStop {
				continue
			}
			x.IncomingControl[j] = target
		}
		return
	}

	for j, eid := range x.Incoming {
		approachDir := approachDirectionOnWay(eid, xOSMID, wayByID, osmWayOfEdge, edgeFromOSM)
		if approachDir == direction {
			x.IncomingControl[j] = target
		}
	}
}

// approachDirectionOnWay returns "forward" or "backward" for approach
// edge eid arriving at intersection node xOSMID, based on whether the
// edge's From node appears before or after xOSMID in the underlying
// OSM way's node sequence. Returns empty string if the direction
// cannot be determined.
func approachDirectionOnWay(
	eid network.EdgeID,
	xOSMID osm.NodeID,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
) string {
	if int(eid) >= len(osmWayOfEdge) {
		return ""
	}
	way, ok := wayByID[osmWayOfEdge[eid]]
	if !ok || way == nil {
		return ""
	}
	fromOSM, ok := edgeFromOSM(eid)
	if !ok {
		return ""
	}
	xIdx, fromIdx := -1, -1
	for i, n := range way.Nodes {
		if n.ID == xOSMID && xIdx < 0 {
			xIdx = i
		}
		if n.ID == fromOSM && fromIdx < 0 {
			fromIdx = i
		}
	}
	if xIdx < 0 || fromIdx < 0 {
		return ""
	}
	if fromIdx < xIdx {
		return "forward"
	}
	if fromIdx > xIdx {
		return "backward"
	}
	return ""
}

// applyInteriorNodeSign overrides per-approach Control based on
// highway=stop or highway=give_way tags on interior shaping nodes —
// nodes between the approach edge's From intersection and the
// intersection X along the underlying OSM way. Mappers conventionally
// place sign tags at the physical stop-line position rather than at
// the intersection node, so honoring those tags gives per-approach
// precision.
//
// Runs last in the resolution chain so interior tags win over
// intersection-node tags when both apply to the same approach. Skips
// approaches already promoted to ControlAllWayStop.
func applyInteriorNodeSign(
	x *network.Intersection,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
	xOSMID osm.NodeID,
	nodeByID map[osm.NodeID]*osm.Node,
) {
	for j, eid := range x.Incoming {
		if x.IncomingControl[j] == network.ControlAllWayStop {
			continue
		}
		sign := interiorSignFor(eid, xOSMID, wayByID, osmWayOfEdge, edgeFromOSM, nodeByID)
		if sign != network.ControlNone {
			x.IncomingControl[j] = sign
		}
	}
}

// interiorSignFor walks the underlying OSM way's node sequence between
// (exclusive) the approach edge's From node and the intersection node
// xOSMID, looking for the closest sign-tagged interior shaping node.
// Returns ControlStop for highway=stop, ControlYield for highway=give_way,
// or ControlNone if no sign-tagged interior node exists. The walk
// starts at xOSMID and steps toward fromOSM so the FIRST tag encountered
// is the one closest to X (the stop-line position).
func interiorSignFor(
	eid network.EdgeID,
	xOSMID osm.NodeID,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
	nodeByID map[osm.NodeID]*osm.Node,
) network.Control {
	if int(eid) >= len(osmWayOfEdge) {
		return network.ControlNone
	}
	way, ok := wayByID[osmWayOfEdge[eid]]
	if !ok || way == nil {
		return network.ControlNone
	}
	fromOSM, ok := edgeFromOSM(eid)
	if !ok {
		return network.ControlNone
	}
	xIdx, fromIdx := -1, -1
	for i, n := range way.Nodes {
		if n.ID == xOSMID && xIdx < 0 {
			xIdx = i
		}
		if n.ID == fromOSM && fromIdx < 0 {
			fromIdx = i
		}
	}
	if xIdx < 0 || fromIdx < 0 || xIdx == fromIdx {
		return network.ControlNone
	}
	step := -1
	if fromIdx > xIdx {
		step = 1
	}
	for i := xIdx + step; i != fromIdx; i += step {
		n := way.Nodes[i]
		node, ok := nodeByID[n.ID]
		if !ok || node == nil {
			continue
		}
		for _, t := range node.Tags {
			if t.Key == "highway" && t.Value == "stop" {
				return network.ControlStop
			}
			if t.Key == "highway" && t.Value == "give_way" {
				return network.ControlYield
			}
		}
	}
	return network.ControlNone
}

// applyStopAllOrMinor overrides class-fallback with explicit OSM tags
// scoped to the intersection node.
func applyStopAllOrMinor(x *network.Intersection, tags osm.Tags, classOfEdge func(network.EdgeID) int) {
	for _, t := range tags {
		if t.Key == "stop" && t.Value == "all" {
			for j := range x.IncomingControl {
				x.IncomingControl[j] = network.ControlAllWayStop
			}
			return
		}
		if t.Key == "stop" && t.Value == "minor" {
			if len(x.Incoming) == 0 {
				return
			}
			best := classOfEdge(x.Incoming[0])
			for _, eid := range x.Incoming[1:] {
				if c := classOfEdge(eid); c < best {
					best = c
				}
			}
			for j, eid := range x.Incoming {
				if classOfEdge(eid) > best {
					x.IncomingControl[j] = network.ControlStop
				} else {
					x.IncomingControl[j] = network.ControlNone
				}
			}
			return
		}
	}
}

// oppositeThreshold is the minimum |Δarrival-heading| for two approaches to
// count as opposing ends of one road continuing through a junction. π − π/4 =
// 135° admits a through road that bends up to 45° at the junction while still
// rejecting a symmetric Y (≈120° arms) and any perpendicular cross street
// (≈90°). Using an angular tolerance instead of fixed axis buckets is what
// makes through-road detection robust to a main road that curves as a side
// road joins — without it the two arms can land in adjacent buckets, leaving
// the junction with no detected through road and demoting it to an all-way
// stop that halts straight-through traffic.
const oppositeThreshold = math.Pi - math.Pi/4

// resolveOpposing populates x.Opposing for each intersection. Two approaches
// are opposing iff they are mutually each other's most-nearly-opposite approach
// (largest |Δarrival-heading|) and that separation exceeds oppositeThreshold —
// i.e. they form the two ends of one road continuing roughly straight through
// the junction. Requiring a mutual match keeps the relation symmetric, so
// Opposing[Opposing[i]] == i.
//
// Receives a *network.Network containing at least Edges so it can call
// network.ArrivalHeading.
func resolveOpposing(xs []network.Intersection, net *network.Network) {
	for i := range xs {
		x := &xs[i]
		n := len(x.Incoming)
		if len(x.Opposing) != n {
			x.Opposing = make([]int8, n)
		}
		for k := range x.Opposing {
			x.Opposing[k] = -1
		}
		headings := make([]float64, n)
		for j, eid := range x.Incoming {
			headings[j] = network.ArrivalHeading(net, eid)
		}
		// best[j] is the approach most nearly opposite j, or -1 if none is
		// more than oppositeThreshold away.
		best := make([]int, n)
		for j := range x.Incoming {
			best[j] = -1
			bestDelta := oppositeThreshold
			for k := range x.Incoming {
				if k == j {
					continue
				}
				d := math.Abs(angleDiff(headings[j], headings[k]))
				if d > bestDelta {
					bestDelta = d
					best[j] = k
				}
			}
		}
		// Keep only mutual matches so Opposing stays symmetric.
		for j := range x.Incoming {
			if k := best[j]; k >= 0 && best[k] == j {
				x.Opposing[j] = int8(k)
			}
		}
	}
}

// angleDiff returns the signed angle (radians, (-π, π]) from a to b.
func angleDiff(a, b float64) float64 {
	d := b - a
	for d > math.Pi {
		d -= 2 * math.Pi
	}
	for d <= -math.Pi {
		d += 2 * math.Pi
	}
	return d
}
