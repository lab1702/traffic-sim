package network

// ReverseEdges indexes each directed edge to its opposite-direction twin,
// when one exists: the other direction of the same two-way road, which runs
// between the same nodes the other way along exactly the reversed polyline
// (netbuild builds it with reverseGeom). The sim uses it so an incident marks
// both directions of a two-way road; the viewer so it counts such a road
// once. One-way edges are absent from the result.
//
// Matching the node pair alone is not enough: two separate roads can join the
// same two nodes, e.g. the carriageways of a dual carriageway, mapped as two
// one-way ways, or two parallel streets. Closing one of them must not close
// the other. Edges without geometry (hand-built fixtures) match on the node
// pair, taking the lowest-ID candidate.
func ReverseEdges(net *Network) map[EdgeID]EdgeID {
	byEnds := make(map[[2]NodeID][]EdgeID, len(net.Edges))
	for i := range net.Edges {
		e := &net.Edges[i]
		k := [2]NodeID{e.From, e.To}
		byEnds[k] = append(byEnds[k], EdgeID(i))
	}
	rev := make(map[EdgeID]EdgeID)
	for i := range net.Edges {
		e := &net.Edges[i]
		for _, t := range byEnds[[2]NodeID{e.To, e.From}] {
			if int(t) != i && isReverseGeometry(e.Geometry, net.Edges[t].Geometry) {
				rev[EdgeID(i)] = t
				break
			}
		}
	}
	return rev
}

// isReverseGeometry reports whether polyline b is polyline a walked
// backwards. Two empty polylines count as reverses.
func isReverseGeometry(a, b []Point) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[len(b)-1-i] {
			return false
		}
	}
	return true
}
