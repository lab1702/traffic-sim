package network

import "math"

// PositionOnEdge returns (x, y, heading) for the point S meters along
// edge's polyline geometry. Linear interpolation between vertices.
// Returns (0, 0, 0) for degenerate edges (less than 2 geometry points
// or unknown edge ID). Heading is in radians via atan2.
func PositionOnEdge(net *Network, eid EdgeID, s float64) (float64, float64, float64) {
	if int(eid) >= len(net.Edges) {
		return 0, 0, 0
	}
	e := &net.Edges[eid]
	g := e.Geometry
	if len(g) < 2 {
		return 0, 0, 0
	}
	remaining := s
	for i := 1; i < len(g); i++ {
		dx := g[i].X - g[i-1].X
		dy := g[i].Y - g[i-1].Y
		segLen := math.Sqrt(dx*dx + dy*dy)
		if remaining <= segLen || i == len(g)-1 {
			t := 0.0
			if segLen > 0 {
				t = remaining / segLen
			}
			if t > 1 {
				t = 1
			}
			return g[i-1].X + dx*t, g[i-1].Y + dy*t, math.Atan2(dy, dx)
		}
		remaining -= segLen
	}
	return g[len(g)-1].X, g[len(g)-1].Y, 0
}
