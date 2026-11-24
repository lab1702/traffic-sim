package sim

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

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
	// minCornerAngle gates the slowdown: bends whose deflection (measured over
	// the sample arms) is gentler than this keep full speed. Drivers don't lift
	// for slight bends; the radius model alone would slow for deflections as
	// small as ~21° on a 40 km/h road, which look straight on screen. ~40° is
	// "a turn you'd actually slow for."
	minCornerAngle = 40.0 * math.Pi / 180 // radians
)

// turnGeometry samples three points — cornerSampleDist back along fromEdge, the
// shared junction node, and cornerSampleDist forward along toEdge — and returns
// the turn's fitted radius (m) and its deflection angle (radians, absolute).
// Radius is +Inf and deflection 0 when geometry is insufficient or the path is
// straight. The sample arms make both robust to short, jagged OSM end-segments.
func turnGeometry(net *network.Network, fromEdge, toEdge network.EdgeID) (radius, deflection float64) {
	fg := net.Edges[fromEdge].Geometry
	tg := net.Edges[toEdge].Geometry
	if len(fg) < 2 || len(tg) < 2 {
		return math.Inf(1), 0
	}
	before := pointBackFromEnd(fg, cornerSampleDist)
	node := fg[len(fg)-1] // == tg[0], the shared junction
	after := pointForwardFromStart(tg, cornerSampleDist)
	radius = circumradius(before, node, after)
	inH := math.Atan2(node.Y-before.Y, node.X-before.X)
	outH := math.Atan2(after.Y-node.Y, after.X-node.X)
	d := outH - inH
	for d > math.Pi {
		d -= 2 * math.Pi
	}
	for d <= -math.Pi {
		d += 2 * math.Pi
	}
	deflection = math.Abs(d)
	// Collinear sample points give an infinite circumradius, which is right
	// for a straight road but wrong when the path folds back on itself. An
	// exact U-turn onto a reverse twin has identical, reversed geometry, so
	// `after` lands on `before`. Treat that as a zero-radius hairpin, so it
	// gets the minimum corner speed instead of none at all.
	if math.IsInf(radius, 1) && deflection > math.Pi/2 {
		radius = 0
	}
	return radius, deflection
}

// turnRadius returns just the fitted radius for the turn (see turnGeometry).
func turnRadius(net *network.Network, fromEdge, toEdge network.EdgeID) float64 {
	r, _ := turnGeometry(net, fromEdge, toEdge)
	return r
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
