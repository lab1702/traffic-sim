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
	_ = edgeFromOSM // used in Tasks 2/4

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
		_ = xOSMID // used in Tasks 2/4

		// Start from the class-based fallback, then layer explicit OSM
		// signage on top (Task 12 covers fallback + stop=all/stop=minor;
		// Task 13 adds node-level highway=stop / give_way).
		applyClassFallback(x, classOfEdge)
		applyStopAllOrMinor(x, nodeTags, classOfEdge)
		applyNodeLevelSign(x, nodeTags)
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
// the intersection node itself. Without direction=, applies to all
// approaches. Direction= refinement is deferred to a later phase; we
// take the lenient interpretation (apply to all).
//
// Skips approaches that have already been promoted to AllWayStop by an
// earlier rule (stop=all or equal-class fallback) — AllWayStop is
// strictly stricter than Stop or Yield, so we never weaken it.
func applyNodeLevelSign(x *network.Intersection, tags osm.Tags) {
	var target network.Control
	hasSign := false
	for _, t := range tags {
		if t.Key == "highway" && t.Value == "stop" {
			target = network.ControlStop
			hasSign = true
		}
		if t.Key == "highway" && t.Value == "give_way" {
			target = network.ControlYield
			hasSign = true
		}
	}
	if !hasSign {
		return
	}
	for j := range x.IncomingControl {
		if x.IncomingControl[j] == network.ControlAllWayStop {
			continue
		}
		x.IncomingControl[j] = target
	}
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
