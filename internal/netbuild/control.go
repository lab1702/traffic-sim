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
// Controls are layered; each later step overrides the earlier ones for the
// approaches it touches:
//  1. Class-based fallback (terminating road yields): the through road
//     keeps priority (None); terminating stems and lower-class approaches
//     Yield; a genuine equal-class crossing or ambiguous Y is AllWayStop.
//  2. stop=all on the intersection node      -> AllWayStop everywhere.
//     stop=minor on the intersection node    -> Stop (Yield if the node is
//     highway=give_way) on every approach whose highway class is strictly
//     lower-priority than the best; None on the rest.
//  3. highway=stop / highway=give_way on the intersection node, refined by
//     direction= — only when neither stop=all nor stop=minor is present
//     (those qualifiers already say which approaches the sign governs).
//  4. highway=stop / highway=give_way on an interior node of the approach
//     (the stop-line position); never weakens an AllWayStop.
//
// edgeSpan[i] holds the positions of edge i's From and To nodes in its OSM
// way's node list (see Build).
func resolveControls(
	xs []network.Intersection,
	feat *osmload.Features,
	osmWayOfEdge []osm.WayID,
	edgeSpan [][2]int,
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

	// Edge-only network so the fallback can measure turn angles.
	geomNet := &network.Network{Edges: edges}

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

		// Roundabout ring node: circulating priority, entries yield. Skip the
		// class/sign chain entirely. Signalled ring nodes (rare) fall through
		// to the normal + signal path instead.
		if !x.HasSignal && applyRoundaboutControl(x, edges) {
			continue
		}

		var nodeTags osm.Tags
		if osmID, ok := osmNodeOf(x.NodeID); ok {
			if n, ok2 := feat.Nodes[osmID]; ok2 && n != nil {
				nodeTags = n.Tags
			}
		}

		// Start from the class-based fallback, then layer explicit OSM
		// signage on top. A stop=all / stop=minor qualifier fully scopes the
		// node's sign, so the node-level highway=stop / give_way rule only
		// runs when no qualifier is present.
		applyClassFallback(x, classOfEdge, geomNet)
		if !applyStopAllOrMinor(x, nodeTags, classOfEdge) {
			applyNodeLevelSign(x, nodeTags, edgeSpan)
		}
		applyInteriorNodeSign(x, wayByID, osmWayOfEdge, edgeSpan, feat.Nodes)
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
// A best-class approach is "through" when its road continues across the
// junction:
//   - two-way road: it has a best-class opposing partner in x.Opposing
//     (populated by resolveOpposing before this runs); the pair is one axis.
//   - one-way road: it has no incoming partner on the far side, but it
//     continues straight (|turn| < throughTurnThreshold) onto a best-class
//     outgoing edge; it is an axis on its own.
//
// Approaches whose straight continuation is the same outgoing edge are a
// merge, not separate axes. Merging into a two-way through road's own
// continuation yields. Otherwise the straightest arm is through and the rest
// yield, for every road class (see resolveMergeGroup for why motorway zip
// merges are not left unsigned). A pure merge (one outgoing edge) where no arm
// is within the straight tolerance still picks a through arm the same way —
// there is no crossing conflict, so an all-way stop is never right there.
//
// One through axis -> it keeps priority (None), every other approach yields.
// Two or more axes (a genuine equal-class crossing, one-way or two-way) or no
// through road among several equal arms (an ambiguous Y) -> AllWayStop.
//
// Explicit OSM signage (handled by the apply* functions that run after this)
// still overrides these defaults.
func applyClassFallback(x *network.Intersection, classOfEdge func(network.EdgeID) int, net *network.Network) {
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
	nBest := 0
	for j, eid := range x.Incoming {
		if classOfEdge(eid) == best {
			isBest[j] = true
			nBest++
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

	edges := net.Edges
	through := make([]bool, n)
	axis := make([]int, n) // through-axis label, meaningful where through[j]

	// Two-way through roads: best-class opposing pairs. Each pair's
	// continuation edges (toward the partner's origin) are claimed so a
	// one-way merging onto them yields instead of forming a second axis.
	claimed := make(map[network.EdgeID]bool)
	for j, eid := range x.Incoming {
		p := int(x.Opposing[j])
		if !isBest[j] || p < 0 || p >= n || !isBest[p] {
			continue
		}
		through[j] = true
		axis[j] = min(j, p)
		partnerFrom := edges[x.Incoming[p]].From
		for _, o := range x.Outgoing {
			if edges[o].To == partnerFrom && edges[o].To != edges[eid].From {
				claimed[o] = true
			}
		}
	}

	// One-way through roads and merges: group the remaining best-class
	// approaches by the outgoing edge they continue straight onto.
	var order []network.EdgeID
	groups := make(map[network.EdgeID][]mergeArm)
	for j, eid := range x.Incoming {
		if !isBest[j] || through[j] {
			continue
		}
		o, dev, ok := straightestExit(net, x, eid, func(o network.EdgeID) bool {
			return classOfEdge(o) == best
		})
		if !ok || dev >= throughTurnThreshold || claimed[o] {
			continue
		}
		if _, seen := groups[o]; !seen {
			order = append(order, o)
		}
		groups[o] = append(groups[o], mergeArm{j: j, dev: dev})
	}
	for _, o := range order {
		resolveMergeGroup(groups[o], x, edges, through, axis)
	}

	// Pure merge (single outgoing edge) where no arm is within the straight
	// tolerance: still a merge, so pick the through arm by the same rule.
	nThrough := 0
	for _, t := range through {
		if t {
			nThrough++
		}
	}
	if nThrough == 0 && len(x.Outgoing) == 1 {
		out := x.Outgoing[0]
		var g []mergeArm
		for j, eid := range x.Incoming {
			if !isBest[j] || edges[out].To == edges[eid].From {
				continue
			}
			g = append(g, mergeArm{j: j, dev: math.Abs(network.TurnAngle(net, eid, out))})
		}
		resolveMergeGroup(g, x, edges, through, axis)
	}

	axes := make(map[int]bool)
	for j := range through {
		if through[j] {
			axes[axis[j]] = true
		}
	}
	if len(axes) != 1 {
		for j := range x.IncomingControl {
			x.IncomingControl[j] = network.ControlAllWayStop
		}
		return
	}

	// Exactly one through axis of the best class: it keeps priority; every
	// terminating or merging best-class approach and every lower-class
	// approach yields.
	for j := range x.IncomingControl {
		if through[j] {
			x.IncomingControl[j] = network.ControlNone
		} else {
			x.IncomingControl[j] = network.ControlYield
		}
	}
}

// throughTurnThreshold is the largest |turn angle| at which an approach still
// counts as continuing straight onto an outgoing edge. π/4 = 45° matches the
// bend tolerance of oppositeThreshold (π − π/4), so a one-way through road may
// bend exactly as much as a two-way one.
const throughTurnThreshold = math.Pi / 4

// mergeArm is one approach (index j into Incoming) feeding a merge, with its
// |turn angle| dev onto the shared outgoing edge.
type mergeArm struct {
	j   int
	dev float64
}

// resolveMergeGroup marks the through arm of approaches that all continue
// onto one outgoing edge. A single arm is simply a one-way through road. For
// several arms the straightest (lowest Incoming index on a tie) is through
// and the rest are left non-through, i.e. they yield.
//
// This applies to motorway/trunk merges too. Real ones are unsigned zip
// merges, but the sim's car-following cannot see a vehicle on a different
// incoming edge until it reaches the shared outgoing edge, so giving every
// arm priority (None) would let two arriving vehicles land on the same spot.
// Yielding is the conservative stand-in until merging across approaches is
// modelled.
func resolveMergeGroup(g []mergeArm, x *network.Intersection, edges []network.Edge, through []bool, axis []int) {
	if len(g) == 0 {
		return
	}
	w := g[0]
	for _, a := range g[1:] {
		if a.dev < w.dev {
			w = a
		}
	}
	through[w.j] = true
	axis[w.j] = w.j
}

// straightestExit returns the outgoing edge at x accepted by keep that the
// approach eid turns onto most nearly straight, with its |turn angle|.
// U-turns back to the approach's own origin are never candidates.
func straightestExit(net *network.Network, x *network.Intersection, eid network.EdgeID,
	keep func(network.EdgeID) bool,
) (network.EdgeID, float64, bool) {
	var bestO network.EdgeID
	bestDev := math.Inf(1)
	found := false
	for _, o := range x.Outgoing {
		if net.Edges[o].To == net.Edges[eid].From || !keep(o) {
			continue
		}
		if d := math.Abs(network.TurnAngle(net, eid, o)); d < bestDev {
			bestO, bestDev, found = o, d, true
		}
	}
	return bestO, bestDev, found
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
//
// Not called for nodes carrying stop=all / stop=minor: applyStopAllOrMinor
// has already scoped the node's sign (see resolveControls).
func applyNodeLevelSign(x *network.Intersection, tags osm.Tags, edgeSpan [][2]int) {
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
		if approachDirectionOnWay(eid, edgeSpan) == direction {
			x.IncomingControl[j] = target
		}
	}
}

// approachDirectionOnWay returns "forward" or "backward" for approach edge
// eid, by whether its From node comes before or after the intersection (its
// To node) in the underlying OSM way's node sequence, or "" when that is
// unknown. The positions come from edgeSpan, recorded when the way was split:
// searching the way for the nodes found the first of the two positions a
// closed way gives its first node, so an approach along the segment closing
// the ring read as backward.
func approachDirectionOnWay(eid network.EdgeID, edgeSpan [][2]int) string {
	if int(eid) >= len(edgeSpan) {
		return ""
	}
	fromIdx, xIdx := edgeSpan[eid][0], edgeSpan[eid][1]
	switch {
	case fromIdx < xIdx:
		return "forward"
	case fromIdx > xIdx:
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
	edgeSpan [][2]int,
	nodeByID map[osm.NodeID]*osm.Node,
) {
	for j, eid := range x.Incoming {
		if x.IncomingControl[j] == network.ControlAllWayStop {
			continue
		}
		sign := interiorSignFor(eid, wayByID, osmWayOfEdge, edgeSpan, nodeByID)
		if sign != network.ControlNone {
			x.IncomingControl[j] = sign
		}
	}
}

// interiorSignFor walks the underlying OSM way's node sequence between
// (exclusive) the approach edge's From node and the intersection (its To
// node), looking for the closest sign-tagged interior shaping node. Returns
// ControlStop for highway=stop, ControlYield for highway=give_way, or
// ControlNone if no sign-tagged interior node exists. The walk starts at the
// intersection and steps toward the From node so the FIRST tag encountered
// is the one closest to X (the stop-line position). The two positions come
// from edgeSpan: searching the way for them walked the wrong part of a
// closed way for the segment that closes the ring.
func interiorSignFor(
	eid network.EdgeID,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeSpan [][2]int,
	nodeByID map[osm.NodeID]*osm.Node,
) network.Control {
	if int(eid) >= len(osmWayOfEdge) || int(eid) >= len(edgeSpan) {
		return network.ControlNone
	}
	way, ok := wayByID[osmWayOfEdge[eid]]
	if !ok || way == nil {
		return network.ControlNone
	}
	fromIdx, xIdx := edgeSpan[eid][0], edgeSpan[eid][1]
	if xIdx == fromIdx || max(xIdx, fromIdx) >= len(way.Nodes) {
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
//
// stop=all / stop=minor is the OSM qualifier that says WHICH approaches a
// junction node's sign applies to, so when present it decides every approach
// at the node:
//   - stop=all   -> AllWayStop everywhere (a highway=give_way on the same
//     node cannot weaken it).
//   - stop=minor -> approaches strictly lower-class than the best get the
//     node's sign (Stop, or Yield when the node is highway=give_way); the
//     major road gets None. When every approach is of the same class, the
//     class fallback's yielding arms get the sign instead, its through road
//     keeps None, and an all-way stop stays one.
//
// Returns true when such a qualifier was applied; the caller must then skip
// applyNodeLevelSign, whose lenient "every approach" / direction= scoping
// would otherwise re-stop the major road (highway=stop + stop=minor is the
// standard two-way-stop tagging) or demote stop=all approaches.
func applyStopAllOrMinor(x *network.Intersection, tags osm.Tags, classOfEdge func(network.EdgeID) int) bool {
	minorCtrl := network.ControlStop
	for _, t := range tags {
		if t.Key == "highway" && t.Value == "give_way" {
			minorCtrl = network.ControlYield
		}
	}
	for _, t := range tags {
		if t.Key == "stop" && t.Value == "all" {
			for j := range x.IncomingControl {
				x.IncomingControl[j] = network.ControlAllWayStop
			}
			return true
		}
		if t.Key == "stop" && t.Value == "minor" {
			if len(x.Incoming) == 0 {
				return true
			}
			best := classOfEdge(x.Incoming[0])
			byClass := false // some approach is of a lower class than the best
			for _, eid := range x.Incoming[1:] {
				c := classOfEdge(eid)
				if c != best {
					byClass = true
				}
				if c < best {
					best = c
				}
			}
			for j, eid := range x.Incoming {
				switch {
				case byClass && classOfEdge(eid) > best:
					x.IncomingControl[j] = minorCtrl
				case byClass:
					x.IncomingControl[j] = network.ControlNone
				case x.IncomingControl[j] == network.ControlYield:
					// Every road is of the same class, so class can't say
					// which is minor. Take applyClassFallback's split: its
					// yielding (terminating or merging) arms are the minor
					// road and its through road stays major. A junction it
					// made an all-way stop has no through road to favour and
					// stays one. Marking every approach major left the
					// signed junction with no control at all.
					x.IncomingControl[j] = minorCtrl
				}
			}
			return true
		}
	}
	return false
}

// applyRoundaboutControl handles a node on a roundabout ring: circulating
// approaches (Roundabout edges) keep priority (ControlNone) and never stop;
// entering approaches yield (ControlYield). Returns true if the node is on a
// ring (has at least one incoming Roundabout edge), in which case the caller
// must skip the normal class/sign resolution chain. The sim's gapBlocked
// then makes entries yield to the circulating edge automatically, since the
// only ControlNone approach at the node is the circulating one.
func applyRoundaboutControl(x *network.Intersection, edges []network.Edge) bool {
	onRing := false
	for _, eid := range x.Incoming {
		if int(eid) < len(edges) && edges[eid].Roundabout {
			onRing = true
			break
		}
	}
	if !onRing {
		return false
	}
	for j, eid := range x.Incoming {
		if int(eid) < len(edges) && edges[eid].Roundabout {
			x.IncomingControl[j] = network.ControlNone
		} else {
			x.IncomingControl[j] = network.ControlYield
		}
	}
	return true
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
