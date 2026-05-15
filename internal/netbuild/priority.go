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
		sort.SliceStable(x.Incoming, func(a, b int) bool {
			pa := priorityOf(x.Incoming[a])
			pb := priorityOf(x.Incoming[b])
			if pa != pb {
				return pa < pb
			}
			return x.Incoming[a] < x.Incoming[b]
		})
	}
}
