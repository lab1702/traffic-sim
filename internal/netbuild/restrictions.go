package netbuild

import (
	"log/slog"
	"strings"

	"github.com/lab1702/traffic-sim/internal/network"
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

	for _, r := range restrictions {
		if applyOne(r, edges, wayToEdges, osmToNet, xAtNode) {
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
	fromEdge, ok := pickEdgeAt(wayToEdges[fromWay], edges, viaNet, true /*ends-at*/)
	if !ok {
		return false
	}
	// 5. Find the to edge: belongs to toWay AND starts at viaNet.
	toEdge, ok := pickEdgeAt(wayToEdges[toWay], edges, viaNet, false /*starts-at*/)
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

// pickEdgeAt returns the first edge in `candidates` whose To (if endsAt
// is true) or From (if false) equals `node`. Returns (0, false) if none
// match — meaning the relation's from/to way doesn't actually touch the
// via node in our graph (often happens when the way was pruned).
func pickEdgeAt(candidates []network.EdgeID, edges []network.Edge, node network.NodeID, endsAt bool) (network.EdgeID, bool) {
	for _, eid := range candidates {
		if int(eid) >= len(edges) {
			continue
		}
		e := &edges[eid]
		if endsAt && e.To == node {
			return eid, true
		}
		if !endsAt && e.From == node {
			return eid, true
		}
	}
	return 0, false
}

func containsEdge(slice []network.EdgeID, target network.EdgeID) bool {
	for _, e := range slice {
		if e == target {
			return true
		}
	}
	return false
}
