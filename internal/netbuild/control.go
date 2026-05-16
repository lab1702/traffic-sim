package netbuild

import (
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

	for i := range xs {
		x := &xs[i]
		var nodeTags osm.Tags
		if osmID, ok := osmNodeOf(x.NodeID); ok {
			if n, ok2 := feat.Nodes[osmID]; ok2 && n != nil {
				nodeTags = n.Tags
			}
		}

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
