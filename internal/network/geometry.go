package network

import "math"

// TurnCategory classifies a turn between two edges at an intersection.
type TurnCategory uint8

const (
	TurnStraight TurnCategory = iota // angle in (-15°, +15°)
	TurnLeft                         // angle in (+15°, +165°]
	TurnRight                        // angle in [-165°, -15°)
	TurnUTurn                        // |angle| > 165°
)

// Turn classification thresholds (radians).
const (
	straightThreshold = 15.0 * math.Pi / 180  // ±15°
	uTurnThreshold    = 165.0 * math.Pi / 180 // ±165°
)

// ArrivalHeading returns the direction (radians, math convention) of a
// vehicle's motion as it arrives at the downstream node of `eid` — the
// direction of the final segment of the edge polyline.
func ArrivalHeading(net *Network, eid EdgeID) float64 {
	if int(eid) >= len(net.Edges) {
		return 0
	}
	g := net.Edges[eid].Geometry
	if len(g) < 2 {
		return 0
	}
	dx := g[len(g)-1].X - g[len(g)-2].X
	dy := g[len(g)-1].Y - g[len(g)-2].Y
	return math.Atan2(dy, dx)
}

// DepartureHeading returns the direction a vehicle moves as it leaves
// the upstream node of `eid` — the direction of the first segment of
// the edge polyline.
func DepartureHeading(net *Network, eid EdgeID) float64 {
	if int(eid) >= len(net.Edges) {
		return 0
	}
	g := net.Edges[eid].Geometry
	if len(g) < 2 {
		return 0
	}
	dx := g[1].X - g[0].X
	dy := g[1].Y - g[0].Y
	return math.Atan2(dy, dx)
}

// TurnAngle returns the signed angle change (radians, normalized to
// (-π, π]) a vehicle experiences when transitioning from `fromEdge`
// (incoming) to `toEdge` (outgoing). Positive = left, negative = right.
func TurnAngle(net *Network, fromEdge, toEdge EdgeID) float64 {
	inH := ArrivalHeading(net, fromEdge)
	outH := DepartureHeading(net, toEdge)
	delta := outH - inH
	for delta > math.Pi {
		delta -= 2 * math.Pi
	}
	for delta <= -math.Pi {
		delta += 2 * math.Pi
	}
	return delta
}

// ClassifyTurn returns the high-level category of a turn from `fromEdge`
// to `toEdge`. Uses ±15° straight and ±165° U-turn thresholds.
func ClassifyTurn(net *Network, fromEdge, toEdge EdgeID) TurnCategory {
	a := TurnAngle(net, fromEdge, toEdge)
	abs := a
	if abs < 0 {
		abs = -abs
	}
	switch {
	case abs > uTurnThreshold:
		return TurnUTurn
	case abs < straightThreshold:
		return TurnStraight
	case a > 0:
		return TurnLeft
	default:
		return TurnRight
	}
}

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
