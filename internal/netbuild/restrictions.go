package netbuild

import (
	"log/slog"
	"strings"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// applyOSMRestrictions resolves each OSM type=restriction relation into
// concrete TurnRestriction entries on the corresponding Intersection.
// Returns (applied, skipped) counts so callers can report ingestion stats.
//
// Supported restriction values (single via=node only):
//
//	no_left_turn, no_right_turn, no_straight_on, no_u_turn,
//	no_entry, no_exit                          -- bans each (from, to) pair
//	                                              (no_entry may list several
//	                                              from ways, no_exit several to)
//	only_left_turn, only_right_turn,
//	only_straight_on, only_u_turn              -- bans every other outgoing edge
//
// Skipped silently:
//   - relations with via=way (multi-via restrictions)
//   - relations missing one of from/via/to roles
//   - only_* relations with more than one from or to way (malformed)
//   - relations whose from/to ways don't appear in the post-prune graph
//   - conditional restrictions (we honor the plain `restriction` tag only)
func applyOSMRestrictions(
	intersections []network.Intersection,
	edges []network.Edge,
	osmWayOfEdge []osm.WayID,
	edgeSpan [][2]int,
	osmToNet map[osm.NodeID]network.NodeID,
	feat *osmload.Features,
	osmNodeOf func(network.NodeID) (osm.NodeID, bool),
	restrictions []*osm.Relation,
) (applied, skipped int) {
	if len(restrictions) == 0 {
		return 0, 0
	}

	// Build osm-way -> []EdgeID index from the post-prune edge slice.
	wayToEdges := make(map[osm.WayID][]network.EdgeID, len(osmWayOfEdge))
	for i := range osmWayOfEdge {
		if i >= len(edges) {
			break
		}
		wid := osmWayOfEdge[i]
		wayToEdges[wid] = append(wayToEdges[wid], edges[i].ID)
	}

	// Index intersections by their NodeID for O(1) lookup. Holding pointers
	// into the slice so mutations propagate.
	xAtNode := make(map[network.NodeID]*network.Intersection, len(intersections))
	for i := range intersections {
		xAtNode[intersections[i].NodeID] = &intersections[i]
	}

	// Way table for direction-aware from/to edge disambiguation when a
	// way passes through the via node (multiple candidate edges).
	wayByID := make(map[osm.WayID]*osm.Way, len(feat.Ways))
	for _, w := range feat.Ways {
		wayByID[w.ID] = w
	}

	for _, r := range restrictions {
		if applyOne(r, edges, edgeSpan, wayToEdges, osmToNet, xAtNode, wayByID, osmNodeOf) {
			applied++
		} else {
			skipped++
		}
	}
	return applied, skipped
}

// applyOne resolves a single restriction relation. Returns true iff a
// TurnRestriction (or set of them, for only_*) was actually applied.
func applyOne(
	r *osm.Relation,
	edges []network.Edge,
	edgeSpan [][2]int,
	wayToEdges map[osm.WayID][]network.EdgeID,
	osmToNet map[osm.NodeID]network.NodeID,
	xAtNode map[network.NodeID]*network.Intersection,
	wayByID map[osm.WayID]*osm.Way,
	osmNodeOf func(network.NodeID) (osm.NodeID, bool),
) bool {
	// 1. Extract restriction tag value. Conditional/role-restricted variants
	// (restriction:hgv=*, restriction:conditional=*) are ignored — we only
	// honor the unconditional, all-vehicle `restriction` tag.
	var rv string
	for _, t := range r.Tags {
		if t.Key == "restriction" {
			rv = t.Value
		}
	}
	if rv == "" {
		return false
	}

	// 2. Extract members. Reject via=way (multi-via). no_entry may list
	// several from ways and no_exit several to ways, so keep them all.
	var fromWays, toWays []osm.WayID
	var viaNode osm.NodeID
	haveVia := false
	for _, m := range r.Members {
		switch m.Role {
		case "from":
			if m.Type != osm.TypeWay {
				return false
			}
			fromWays = append(fromWays, osm.WayID(m.Ref))
		case "to":
			if m.Type != osm.TypeWay {
				return false
			}
			toWays = append(toWays, osm.WayID(m.Ref))
		case "via":
			if m.Type != osm.TypeNode {
				return false // skip via=way for now
			}
			viaNode = osm.NodeID(m.Ref)
			haveVia = true
		}
	}
	if len(fromWays) == 0 || len(toWays) == 0 || !haveVia {
		return false
	}
	only := strings.HasPrefix(rv, "only_")
	if !only && !strings.HasPrefix(rv, "no_") {
		slog.Debug("unknown restriction value; skipping",
			"relation_id", int64(r.ID), "value", rv)
		return false
	}
	if only && (len(fromWays) > 1 || len(toWays) > 1) {
		return false // an only_* names one movement; several from/to is malformed
	}

	// 3. Translate via node to internal NodeID.
	viaNet, ok := osmToNet[viaNode]
	if !ok {
		return false // node dropped during filter/prune
	}
	x, ok := xAtNode[viaNet]
	if !ok {
		return false // via node isn't an intersection in our graph
	}

	// 4. Find each from edge: belongs to a from way AND ends at viaNet.
	//    For through-ways (way passes through via), multiple candidates match;
	//    prefer the one in the way's forward direction.
	//    5. Find each to edge: belongs to a to way AND starts at viaNet.
	pick := func(way osm.WayID, endsAt bool) (network.EdgeID, bool) {
		return pickEdgeAt(wayToEdges[way], edges, edgeSpan, viaNet, endsAt,
			wayByID[way], viaNode, osmNodeOf)
	}
	if only {
		fromEdge, ok := pick(fromWays[0], true /*ends-at*/)
		if !ok {
			return false
		}
		toEdge, ok := pick(toWays[0], false /*starts-at*/)
		if !ok {
			return false
		}
		// Ban every outgoing edge that isn't `toEdge`.
		added := 0
		for _, o := range x.Outgoing {
			if o == toEdge {
				continue
			}
			x.BannedTurns = append(x.BannedTurns, network.TurnRestriction{
				From: fromEdge, To: o,
			})
			added++
		}
		// If the only_* restriction's "to" wasn't actually reachable from
		// the via intersection, we'd ban every outgoing without sparing
		// the intended one — that's wrong. Detect and roll back.
		if !containsEdge(x.Outgoing, toEdge) {
			x.BannedTurns = x.BannedTurns[:len(x.BannedTurns)-added]
			return false
		}
		return added > 0
	}

	// no_*: ban every (from, to) pair the relation names. Taking only the
	// last from or to member left all but one movement of a no_entry or
	// no_exit unbanned, while the relation still counted as applied.
	applied := false
	for _, fw := range fromWays {
		fromEdge, ok := pick(fw, true /*ends-at*/)
		if !ok {
			continue
		}
		for _, tw := range toWays {
			toEdge, ok := pick(tw, false /*starts-at*/)
			if !ok {
				continue
			}
			x.BannedTurns = append(x.BannedTurns, network.TurnRestriction{
				From: fromEdge, To: toEdge,
			})
			applied = true
		}
	}
	return applied
}

// pickEdgeAt returns the edge in `candidates` whose To (if endsAt is true)
// or From (if false) equals `node`. Returns (0, false) if no candidate
// touches the via node — usually because the way was pruned.
//
// For through-ways (the way passes through the via node), more than one
// candidate matches: a 3-node way [A, X, B] yields four directed edges,
// two of which "end at X" (forward A→X, reverse B→X) and two of which
// "start at X" (forward X→B, reverse X→A). The OSM convention is to
// interpret the from/to way in the way's forward direction (low → high
// node index). When `way` is non-nil and the via node is locatable
// inside it, prefer:
//
//   - endsAt:  the edge whose From-OSM-node has a lower way-index than via
//     (i.e., the forward-direction segment entering via).
//   - !endsAt: the edge whose To-OSM-node has a higher way-index than via
//     (i.e., the forward-direction segment leaving via).
//
// Falls back to the first slice-order match if disambiguation isn't
// possible (way unknown, via not found in way, edge endpoints missing).
//
// When edgeSpan (each edge's From/To positions in its way, see Build) covers
// the matches, the forward one is simply the one whose From comes before its
// To. Searching the way for the nodes instead finds the first of the two
// positions a closed way gives its first node, which misjudges the segment
// that closes the ring; that search remains for callers without spans.
func pickEdgeAt(
	candidates []network.EdgeID, edges []network.Edge, edgeSpan [][2]int, node network.NodeID, endsAt bool,
	way *osm.Way, viaOSM osm.NodeID, osmNodeOf func(network.NodeID) (osm.NodeID, bool),
) (network.EdgeID, bool) {
	var matches []network.EdgeID
	for _, eid := range candidates {
		if int(eid) >= len(edges) {
			continue
		}
		e := &edges[eid]
		if endsAt && e.To == node {
			matches = append(matches, eid)
		}
		if !endsAt && e.From == node {
			matches = append(matches, eid)
		}
	}
	if len(matches) == 0 {
		return 0, false
	}
	if len(matches) == 1 {
		return matches[0], true
	}
	if spansKnown := len(edgeSpan) > 0; spansKnown {
		for _, eid := range matches {
			if int(eid) >= len(edgeSpan) {
				spansKnown = false
				break
			}
		}
		if spansKnown {
			for _, eid := range matches {
				if edgeSpan[eid][0] < edgeSpan[eid][1] {
					return eid, true
				}
			}
			return matches[0], true
		}
	}
	if way == nil {
		return matches[0], true
	}

	// Locate via in the way's node sequence.
	viaIdx := -1
	for i, n := range way.Nodes {
		if n.ID == viaOSM {
			viaIdx = i
			break
		}
	}
	if viaIdx < 0 {
		return matches[0], true
	}

	// idxInWay returns the position of a network node's OSM id within the
	// way, or -1 if not found / unresolvable.
	idxInWay := func(nid network.NodeID) int {
		o, ok := osmNodeOf(nid)
		if !ok {
			return -1
		}
		for i, n := range way.Nodes {
			if n.ID == o {
				return i
			}
		}
		return -1
	}

	for _, eid := range matches {
		var probe network.NodeID
		if endsAt {
			probe = edges[eid].From
		} else {
			probe = edges[eid].To
		}
		idx := idxInWay(probe)
		if idx < 0 {
			continue
		}
		if endsAt && idx < viaIdx {
			return eid, true
		}
		if !endsAt && idx > viaIdx {
			return eid, true
		}
	}
	return matches[0], true
}

func containsEdge(slice []network.EdgeID, target network.EdgeID) bool {
	for _, e := range slice {
		if e == target {
			return true
		}
	}
	return false
}
