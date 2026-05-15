// Package netbuild: lane-to-turn assignment. Populates Lane.AllowedTurns
// for every edge whose downstream node is a multi-edge intersection,
// either from the OSM `turn:lanes=*` tag or via geometric inference.
package netbuild

import (
	"strings"

	"github.com/lab1702/traffic-sim/internal/network"
)

// parseTurnLaneSpec parses one lane's spec from an OSM turn:lanes string.
// A spec can list multiple turn types separated by ';'. Unknown tokens
// are dropped. An empty spec ("" or "none") maps to TurnStraight.
//
// Returns nil if every token in the spec was unrecognized (e.g., "reverse"
// or "floof"). Callers should treat nil as "no usable OSM data for this
// lane" — distinct from a non-empty result like {TurnStraight}, which
// means the OSM data explicitly permits only that turn.
func parseTurnLaneSpec(spec string) []network.TurnCategory {
	if spec == "" || spec == "none" {
		return []network.TurnCategory{network.TurnStraight}
	}
	var out []network.TurnCategory
	seen := map[network.TurnCategory]bool{}
	for _, tok := range strings.Split(spec, ";") {
		tok = strings.TrimSpace(tok)
		c, ok := turnTokenMap[tok]
		if !ok {
			continue
		}
		if !seen[c] {
			out = append(out, c)
			seen[c] = true
		}
	}
	return out
}

// parseTurnLanesString parses a full OSM turn:lanes value (pipe-delimited
// per-lane specs). Returns one entry per lane. Returns nil for empty input.
func parseTurnLanesString(s string) [][]network.TurnCategory {
	if s == "" {
		return nil
	}
	parts := strings.Split(s, "|")
	out := make([][]network.TurnCategory, len(parts))
	for i, p := range parts {
		out[i] = parseTurnLaneSpec(p)
	}
	return out
}

var turnTokenMap = map[string]network.TurnCategory{
	"left":           network.TurnLeft,
	"slight_left":    network.TurnLeft,
	"sharp_left":     network.TurnLeft,
	"merge_to_left":  network.TurnLeft,
	"right":          network.TurnRight,
	"slight_right":   network.TurnRight,
	"sharp_right":    network.TurnRight,
	"merge_to_right": network.TurnRight,
	"through":        network.TurnStraight,
	// "none" and "" handled by parseTurnLaneSpec directly.
	// "reverse" intentionally absent — U-turns are dropped.
}

// assignAllowedTurnsForEdge computes the AllowedTurns lists for each lane
// of `incoming` at intersection `x`.
//
// - osmSpec: pre-parsed turn:lanes tokens (nil if no OSM data). When the
//   token count matches the lane count, this overrides geometric inference.
//
// Returns one []EdgeID per lane (same order as incoming.Lanes).
func assignAllowedTurnsForEdge(
	net *network.Network,
	incoming network.EdgeID,
	x *network.Intersection,
	osmSpec [][]network.TurnCategory,
) [][]network.EdgeID {
	inc := &net.Edges[incoming]
	numLanes := len(inc.Lanes)
	if numLanes == 0 {
		return nil
	}

	// Build banned-set keyed by outgoing edge.
	banned := make(map[network.EdgeID]bool)
	for _, br := range x.BannedTurns {
		if br.From == incoming {
			banned[br.To] = true
		}
	}

	// Classify each non-banned, non-U-turn outgoing edge.
	type outInfo struct {
		eid network.EdgeID
		cat network.TurnCategory
	}
	var outs []outInfo
	categoriesPresent := make(map[network.TurnCategory]bool)
	for _, oid := range x.Outgoing {
		if banned[oid] {
			continue
		}
		cat := network.ClassifyTurn(net, incoming, oid)
		if cat == network.TurnUTurn {
			continue
		}
		outs = append(outs, outInfo{oid, cat})
		categoriesPresent[cat] = true
	}
	if len(outs) == 0 {
		return make([][]network.EdgeID, numLanes)
	}

	// One-lane incoming or only one allowed outgoing: every lane gets
	// every legal outgoing.
	if numLanes == 1 || len(outs) == 1 {
		all := make([]network.EdgeID, len(outs))
		for i, o := range outs {
			all[i] = o.eid
		}
		result := make([][]network.EdgeID, numLanes)
		for i := range result {
			result[i] = append([]network.EdgeID(nil), all...)
		}
		return result
	}

	// Decide per-lane categories: OSM if usable, else geometric.
	var perLane [][]network.TurnCategory
	if len(osmSpec) == numLanes {
		perLane = osmSpec
	} else {
		var presentList []network.TurnCategory
		for c := range categoriesPresent {
			presentList = append(presentList, c)
		}
		perLane = assignLanesGeometric(presentList, numLanes)
	}

	// Translate per-lane categories to per-lane outgoing edges.
	result := make([][]network.EdgeID, numLanes)
	for i, cats := range perLane {
		for _, o := range outs {
			for _, c := range cats {
				if c == o.cat {
					result[i] = append(result[i], o.eid)
					break
				}
			}
		}
	}

	// Sanity: every non-banned outgoing must be reachable from some lane.
	// If a category is present but unreachable, attach to the closest-side lane.
	for _, o := range outs {
		reachable := false
		for _, lane := range result {
			for _, e := range lane {
				if e == o.eid {
					reachable = true
					break
				}
			}
			if reachable {
				break
			}
		}
		if !reachable {
			target := 0
			if o.cat == network.TurnLeft {
				target = numLanes - 1
			}
			result[target] = append(result[target], o.eid)
		}
	}
	return result
}

// assignLanesGeometric returns a per-lane list of allowed turn categories
// for an intersection where the given set of categories is present.
// Convention: lane 0 = rightmost; higher index = closer to road centerline.
// The input `cats` is treated as a set (duplicates ignored, order ignored).
func assignLanesGeometric(cats []network.TurnCategory, numLanes int) [][]network.TurnCategory {
	if numLanes <= 0 {
		return nil
	}
	hasL, hasS, hasR := false, false, false
	for _, c := range cats {
		switch c {
		case network.TurnLeft:
			hasL = true
		case network.TurnStraight:
			hasS = true
		case network.TurnRight:
			hasR = true
		}
	}
	out := make([][]network.TurnCategory, numLanes)

	// One-lane edge gets everything that's present.
	if numLanes == 1 {
		var all []network.TurnCategory
		if hasR {
			all = append(all, network.TurnRight)
		}
		if hasS {
			all = append(all, network.TurnStraight)
		}
		if hasL {
			all = append(all, network.TurnLeft)
		}
		out[0] = all
		return out
	}

	last := numLanes - 1
	for i := range out {
		// Default: middle lanes get straight.
		if hasS {
			out[i] = []network.TurnCategory{network.TurnStraight}
		}
	}
	// When both edge turns (L and R) are present and there are enough lanes
	// (>= 3), dedicate edge lanes exclusively to their turn direction so that
	// middle lanes serve straight traffic without overlap.
	exclusiveEdges := hasL && hasR && numLanes >= 3

	if hasR {
		if hasS && !exclusiveEdges {
			out[0] = []network.TurnCategory{network.TurnRight, network.TurnStraight}
		} else {
			out[0] = []network.TurnCategory{network.TurnRight}
		}
	}
	if hasL {
		if hasS && !exclusiveEdges {
			out[last] = []network.TurnCategory{network.TurnLeft, network.TurnStraight}
		} else {
			out[last] = []network.TurnCategory{network.TurnLeft}
		}
	}
	return out
}
