package sim

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

// computeDesiredSpeed returns the v0 (desired speed) for IDM. It is the current
// edge's speed limit (scaled by the driver's preference and any Slowdown
// incident), optionally reduced for an upcoming turn. The turn reduction uses a
// radius-based comfortable speed and a smooth kinematic approach profile so the
// vehicle eases into the corner rather than braking hard far upstream.
func (w *World) computeDesiredSpeed(v *Vehicle) float64 {
	edge := &w.Net.Edges[v.Edge]
	// Per-driver speed preference. A zero factor means a hand-constructed
	// vehicle (test fixture) — treat as 1.0 so existing tests work.
	factor := v.SpeedFactor
	if factor == 0 {
		factor = 1.0
	}
	v0 := edge.SpeedLimit * factor
	if w.Incidents[v.Edge] == Slowdown {
		if cap := edge.SpeedLimit * incidentSlowdownFactor; cap < v0 {
			v0 = cap
		}
	}
	if v.RouteIdx+1 >= len(v.Route) {
		return v0 // no next edge: nothing to slow for
	}
	nextEdge := v.Route[v.RouteIdx+1]
	vSafe := cornerSpeed(turnRadius(w.Net, v.Edge, nextEdge))
	if math.IsInf(vSafe, 1) || vSafe >= v0 {
		return v0 // straight or gentle turn: no slowdown
	}
	// Kinematic approach: the max speed from which we can still decelerate at
	// cornerBrakeDecel to reach vSafe by the corner (distance d ahead). Far from
	// the corner this exceeds v0 (no effect); it eases to vSafe as d -> 0.
	d := edge.Length - v.S
	if d < 0 {
		d = 0
	}
	if v0corner := math.Sqrt(vSafe*vSafe + 2*cornerBrakeDecel*d); v0corner < v0 {
		return v0corner
	}
	return v0
}

// circumradius returns the radius of the circle through three points, or +Inf
// when they are (near-)collinear. A straight road therefore yields no
// curvature constraint. R = (|ab|·|bc|·|ca|) / (4·area); area is half the
// cross-product magnitude, so R = product / (2·crossMag).
func circumradius(p1, p2, p3 network.Point) float64 {
	a := math.Hypot(p1.X-p2.X, p1.Y-p2.Y)
	b := math.Hypot(p2.X-p3.X, p2.Y-p3.Y)
	c := math.Hypot(p3.X-p1.X, p3.Y-p1.Y)
	// Coincident points define no unique circle; treat as infinite radius.
	if a == 0 || b == 0 || c == 0 {
		return math.Inf(1)
	}
	crossMag := math.Abs((p2.X-p1.X)*(p3.Y-p1.Y) - (p3.X-p1.X)*(p2.Y-p1.Y))
	// 1e-9 m² is effectively zero for ~15 m sample arms (float64 rounding ≈ 1e-14 m²).
	if crossMag < 1e-9 {
		return math.Inf(1)
	}
	return (a * b * c) / (2 * crossMag)
}

// pointBackFromEnd returns the point reached walking `dist` metres back from the
// end of the polyline, interpolating within the segment where the distance runs
// out. Polylines shorter than `dist` clamp to the first point. Assumes
// len(geom) >= 2. Zero-length segments (duplicate points) are skipped.
func pointBackFromEnd(geom []network.Point, dist float64) network.Point {
	var acc float64
	for i := len(geom) - 1; i > 0; i-- {
		ax, ay := geom[i].X, geom[i].Y
		bx, by := geom[i-1].X, geom[i-1].Y
		seg := math.Hypot(bx-ax, by-ay)
		if seg == 0 {
			continue
		}
		if acc+seg >= dist {
			t := (dist - acc) / seg
			return network.Point{X: ax + (bx-ax)*t, Y: ay + (by-ay)*t}
		}
		acc += seg
	}
	return geom[0]
}

// pointForwardFromStart mirrors pointBackFromEnd, walking `dist` metres forward
// from the start of the polyline; clamps to the last point for short polylines.
// Assumes len(geom) >= 2. Zero-length segments (duplicate points) are skipped.
func pointForwardFromStart(geom []network.Point, dist float64) network.Point {
	var acc float64
	for i := 0; i < len(geom)-1; i++ {
		ax, ay := geom[i].X, geom[i].Y
		bx, by := geom[i+1].X, geom[i+1].Y
		seg := math.Hypot(bx-ax, by-ay)
		if seg == 0 {
			continue
		}
		if acc+seg >= dist {
			t := (dist - acc) / seg
			return network.Point{X: ax + (bx-ax)*t, Y: ay + (by-ay)*t}
		}
		acc += seg
	}
	return geom[len(geom)-1]
}

// Comfortable cornering parameters. A turn's safe speed comes from the lateral
// acceleration a driver tolerates while rounding a curve of the estimated
// radius: v = sqrt(cornerLatAccel * R).
const (
	cornerLatAccel   = 3.0  // m/s^2, comfortable lateral acceleration
	cornerSampleDist = 15.0 // m, radius sampling arm length on each side
	minCornerSpeed   = 2.5  // m/s (~9 km/h), floor so hairpins crawl, not stop
	// cornerBrakeDecel is the planning deceleration (m/s^2) for the smooth
	// approach profile: the desired speed eases down so the vehicle reaches the
	// corner speed at the corner braking at roughly this rate, not slamming.
	cornerBrakeDecel = 1.0
)

// turnRadius estimates the radius (m) of the turn from fromEdge onto toEdge by
// fitting a circle through a point cornerSampleDist back along fromEdge, the
// shared junction node, and a point cornerSampleDist forward along toEdge.
// Returns +Inf when either edge lacks geometry or the path is straight. The
// sample arms make the estimate robust to short, jagged OSM end-segments.
func turnRadius(net *network.Network, fromEdge, toEdge network.EdgeID) float64 {
	fg := net.Edges[fromEdge].Geometry
	tg := net.Edges[toEdge].Geometry
	if len(fg) < 2 || len(tg) < 2 {
		return math.Inf(1)
	}
	before := pointBackFromEnd(fg, cornerSampleDist)
	node := fg[len(fg)-1] // == tg[0], the shared junction
	after := pointForwardFromStart(tg, cornerSampleDist)
	return circumradius(before, node, after)
}

// cornerSpeed returns the comfortable speed (m/s) for a turn of radius R using
// the lateral-acceleration model, floored at minCornerSpeed. R == +Inf (a
// straight road) passes through as +Inf — no constraint.
func cornerSpeed(R float64) float64 {
	if math.IsInf(R, 1) {
		return math.Inf(1)
	}
	if R < 0 {
		return math.Inf(1) // invalid radius: treat as unconstrained
	}
	v := math.Sqrt(cornerLatAccel * R)
	if v < minCornerSpeed {
		return minCornerSpeed
	}
	return v
}
