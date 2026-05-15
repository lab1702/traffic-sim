package sim

import "github.com/lab1702/traffic-sim/internal/network"

// IncomingPos returns the position of edgeID within x.Incoming, or -1.
func IncomingPos(x *network.Intersection, edgeID network.EdgeID) int {
	for i, e := range x.Incoming {
		if e == edgeID {
			return i
		}
	}
	return -1
}

// IntersectionAtNode returns the Intersection whose NodeID matches, or nil.
func IntersectionAtNode(net *network.Network, n network.NodeID) *network.Intersection {
	for i := range net.Intersections {
		if net.Intersections[i].NodeID == n {
			return &net.Intersections[i]
		}
	}
	return nil
}
