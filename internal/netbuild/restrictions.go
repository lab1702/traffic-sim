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
//	no_entry, no_exit                          -- bans the (from, to) pair
//	only_left_turn, only_right_turn,
//	only_straight_on, only_u_turn              -- bans every other outgoing edge
//
// Skipped silently:
//   - relations with via=way (multi-via restrictions)
//   - relations missing one of from/via/to roles
//   - relations whose from/to ways don't appear in the post-prune graph
//   - conditional restrictions (we honor the plain `restriction` tag only)
func applyOSMRestrictions(
	intersections []network.Intersection,
	edges []network.Edge,
	osmWayOfEdge []osm.WayID,
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
		if applyOne(r, edges, wayToEdges, osmToNet, xAtNode, wayByID, osmNodeOf) {
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

	// 2. Extract members. Reject via=way (multi-via).
	var fromWay, toWay osm.WayID
	var viaNode osm.NodeID
	haveFrom, haveTo, haveVia := false, false, false
	for _, m := range r.Members {
		switch m.Role {
		case "from":
			if m.Type != osm.TypeWay {
				return false
			}
			fromWay = osm.WayID(m.Ref)
			haveFrom = true
		case "to":
			if m.Type != osm.TypeWay {
				return false
			}
			toWay = osm.WayID(m.Ref)
			haveTo = true
		case "via":
			if m.Type != osm.TypeNode {
				return false // skip via=way for now
			}
			viaNode = osm.NodeID(m.Ref)
			haveVia = true
		}
	}
	if !haveFrom || !haveTo || !haveVia {
		return false
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

	// 4. Find the from edge: belongs to fromWay AND ends at viaNet.
	//    For through-ways (way passes through via), multiple candidates match;
	//    prefer the one in the way's forward direction.
	fromEdge, ok := pickEdgeAt(wayToEdges[fromWay], edges, viaNet, true /*ends-at*/,
		wayByID[fromWay], viaNode, osmNodeOf)
	if !ok {
		return false
	}
	// 5. Find the to edge: belongs to toWay AND starts at viaNet.
	toEdge, ok := pickEdgeAt(wayToEdges[toWay], edges, viaNet, false /*starts-at*/,
		wayByID[toWay], viaNode, osmNodeOf)
	if !ok {
		return false
	}

	switch {
	case strings.HasPrefix(rv, "no_"):
		// Single ban.
		x.BannedTurns = append(x.BannedTurns, network.TurnRestriction{
			From: fromEdge, To: toEdge,
		})
		return true
	case strings.HasPrefix(rv, "only_"):
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
	default:
		slog.Debug("unknown restriction value; skipping",
			"relation_id", int64(r.ID), "value", rv)
		return false
	}
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
func pickEdgeAt(
	candidates []network.EdgeID, edges []network.Edge, node network.NodeID, endsAt bool,
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
	if len(matches) == 1 || way == nil {
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
