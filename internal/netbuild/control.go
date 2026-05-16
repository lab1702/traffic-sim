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
// Resolution order (first rule that applies wins for a given approach):
//  1. stop=all on the intersection node      -> AllWayStop everywhere.
//  2. stop=minor on the intersection node    -> Stop on every approach
//     whose highway class is strictly lower-priority than the best.
//  3. Class-based fallback:
//     unequal classes -> lower gets Stop
//     equal classes   -> AllWayStop everywhere
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
	}
}

// applyClassFallback sets IncomingControl based on functional class only.
// Unequal classes: best (lowest priority value) stays ControlNone,
// strictly higher (lower-priority) approaches get ControlStop.
// Equal classes: every approach becomes ControlAllWayStop.
func applyClassFallback(x *network.Intersection, classOfEdge func(network.EdgeID) int) {
	if len(x.Incoming) == 0 {
		return
	}
	best := classOfEdge(x.Incoming[0])
	for _, eid := range x.Incoming[1:] {
		if c := classOfEdge(eid); c < best {
			best = c
		}
	}
	allEqual := true
	for _, eid := range x.Incoming {
		if classOfEdge(eid) != best {
			allEqual = false
			break
		}
	}
	if allEqual {
		for j := range x.IncomingControl {
			x.IncomingControl[j] = network.ControlAllWayStop
		}
		return
	}
	for j, eid := range x.Incoming {
		if classOfEdge(eid) == best {
			x.IncomingControl[j] = network.ControlNone
		} else {
			x.IncomingControl[j] = network.ControlStop
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

	// direction= is set: find the canonical way for this sign (lowest WayID
	// that contains xOSMID and has at least one matching approach). Apply
	// the sign only to approaches on that one way.
	canonicalWayID := canonicalSignWay(x.Incoming, xOSMID, direction, wayByID, osmWayOfEdge, edgeFromOSM)
	if canonicalWayID == 0 {
		return
	}
	for j, eid := range x.Incoming {
		if int(eid) >= len(osmWayOfEdge) || osmWayOfEdge[eid] != canonicalWayID {
			continue
		}
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

// canonicalSignWay returns the WayID of the way that should be treated as
// the "owner" of a directional sign at intersection node xOSMID. When the
// node is shared by several ways and multiple ways have a matching approach
// in the given direction, we pick the lowest WayID that has at least one
// matching approach — a deterministic tie-breaking rule that mirrors how
// OSM editors list ways (and ensures stable behaviour as the map grows).
// Returns 0 if no way has a matching approach.
func canonicalSignWay(
	incoming []network.EdgeID,
	xOSMID osm.NodeID,
	direction string,
	wayByID map[osm.WayID]*osm.Way,
	osmWayOfEdge []osm.WayID,
	edgeFromOSM func(network.EdgeID) (osm.NodeID, bool),
) osm.WayID {
	var best osm.WayID
	for _, eid := range incoming {
		if int(eid) >= len(osmWayOfEdge) {
			continue
		}
		wid := osmWayOfEdge[eid]
		if best != 0 && wid >= best {
			continue // already have a better (lower) candidate
		}
		d := approachDirectionOnWay(eid, xOSMID, wayByID, osmWayOfEdge, edgeFromOSM)
		if d == direction {
			best = wid
		}
	}
	return best
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

// resolveOpposing populates x.Opposing for each intersection. Two
// approaches are opposing iff:
//
//  1. Their arrival headings fold to the same axis bucket (same
//     8-bucket / 22.5° resolution as DefaultSignalConfig in sim).
//  2. AND their arrival headings are > π/2 apart (excludes
//     same-direction misalignment at Y-junctions and skewed forks).
//
// If a bucket has more than two members (degenerate star geometry),
// each approach pairs with whichever bucket-mate has the largest
// |Δheading|, i.e. the one most nearly opposite.
//
// Receives a *network.Network containing at least Edges so it can
// call network.ArrivalHeading.
func resolveOpposing(xs []network.Intersection, net *network.Network) {
	const numBuckets = 8
	for i := range xs {
		x := &xs[i]
		if len(x.Opposing) != len(x.Incoming) {
			x.Opposing = make([]int8, len(x.Incoming))
		}
		for k := range x.Opposing {
			x.Opposing[k] = -1
		}
		headings := make([]float64, len(x.Incoming))
		buckets := make([]int, len(x.Incoming))
		for j, eid := range x.Incoming {
			h := network.ArrivalHeading(net, eid)
			headings[j] = h
			ax := math.Mod(h, math.Pi)
			if ax < 0 {
				ax += math.Pi
			}
			buckets[j] = int(math.Round(ax*numBuckets/math.Pi)) % numBuckets
		}
		for j := range x.Incoming {
			best := -1
			bestDelta := math.Pi / 2
			for k := range x.Incoming {
				if k == j || buckets[k] != buckets[j] {
					continue
				}
				d := math.Abs(angleDiff(headings[j], headings[k]))
				if d > bestDelta {
					bestDelta = d
					best = k
				}
			}
			if best >= 0 {
				x.Opposing[j] = int8(best)
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
