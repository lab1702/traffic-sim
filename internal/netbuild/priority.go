package netbuild

import (
	"sort"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/osmload"
	"github.com/paulmach/osm"
)

// highwayPriority maps an OSM highway= value to a numeric rank: lower =
// higher real-world priority (main road yields to no one). Unknown or
// missing values get the lowest priority so they only "win" against
// other unknowns by edge-ID tie-break.
//
// The order follows the OSM functional class hierarchy.
func highwayPriority(hw string) int {
	switch hw {
	case "motorway":
		return 0
	case "trunk":
		return 1
	case "primary":
		return 2
	case "secondary":
		return 3
	case "tertiary":
		return 4
	case "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link":
		return 5
	case "unclassified":
		return 6
	case "residential":
		return 7
	case "living_street":
		return 8
	case "service":
		return 9
	}
	return 100
}

// sortIncomingByPriority reorders each intersection's Incoming slice so
// that the highest-priority approach (lowest highwayPriority value) is
// at index 0. The sim's unsignalized-yield rule treats index 0 as the
// priority road; sorting here makes that rule produce realistic behavior
// instead of edge-order-dependent gridlock.
//
// Tie-break by EdgeID for determinism.
func sortIncomingByPriority(
	intersections []network.Intersection,
	osmWayOfEdge []osm.WayID,
	feat *osmload.Features,
) {
	wayByID := make(map[osm.WayID]*osm.Way, len(feat.Ways))
	for _, w := range feat.Ways {
		wayByID[w.ID] = w
	}
	priorityOf := func(eid network.EdgeID) int {
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
	for i := range intersections {
		x := &intersections[i]
		// Sort the Incoming slice and apply the same permutation to
		// IncomingControl so the two stay aligned. Index-permutation
		// approach keeps both slices in sync without a custom sort.Sort
		// receiver.
		idx := make([]int, len(x.Incoming))
		for j := range idx {
			idx[j] = j
		}
		sort.SliceStable(idx, func(a, b int) bool {
			ea, eb := x.Incoming[idx[a]], x.Incoming[idx[b]]
			pa, pb := priorityOf(ea), priorityOf(eb)
			if pa != pb {
				return pa < pb
			}
			return ea < eb
		})
		// Build inverse permutation: oldToNew[oldI] = newI.
		oldToNew := make([]int, len(idx))
		for newI, oldI := range idx {
			oldToNew[oldI] = newI
		}
		newInc := make([]network.EdgeID, len(x.Incoming))
		newCtrl := make([]network.Control, len(x.IncomingControl))
		newOpp := make([]int8, len(x.Opposing))
		for newI, oldI := range idx {
			newInc[newI] = x.Incoming[oldI]
			if oldI < len(x.IncomingControl) {
				newCtrl[newI] = x.IncomingControl[oldI]
			}
			if oldI < len(x.Opposing) {
				oldVal := x.Opposing[oldI]
				if oldVal < 0 {
					newOpp[newI] = -1
				} else {
					newOpp[newI] = int8(oldToNew[int(oldVal)])
				}
			} else {
				newOpp[newI] = -1
			}
		}
		x.Incoming = newInc
		x.IncomingControl = newCtrl
		x.Opposing = newOpp
	}
}
