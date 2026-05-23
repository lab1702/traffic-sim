package sim

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

// cornerSpeedCap returns the maximum comfortable speed (m/s) for a turn
// whose heading change has absolute value absAngleRad. Returns +Inf for
// transitions that are effectively straight (angle below straightCutoff)
// — meaning "no cap".
//
// Tuning anchors:
//   - below 15° → +Inf (cruise)
//   - 90°       → 5 m/s (~18 km/h, ~11 mph)
//   - 180°      → 2.5 m/s (~9 km/h)
//   - 15°..90°  → linear interp from 30 m/s to 5 m/s
//   - 90°..180° → linear interp from 5 m/s to 2.5 m/s
func cornerSpeedCap(absAngleRad float64) float64 {
	const (
		straightCutoff = 15.0 * math.Pi / 180
		capUpper       = 30.0 // m/s at the cutoff (effectively no cap on urban roads)
		capAt90        = 5.0
		capAt180       = 2.5
	)
	if absAngleRad < straightCutoff {
		return math.Inf(1)
	}
	if absAngleRad <= math.Pi/2 {
		t := (absAngleRad - straightCutoff) / (math.Pi/2 - straightCutoff)
		return capUpper + t*(capAt90-capUpper)
	}
	t := (absAngleRad - math.Pi/2) / (math.Pi - math.Pi/2)
	return capAt90 + t*(capAt180-capAt90)
}

// cornerBrakingDecel is the comfortable deceleration we assume when
// braking for an upcoming corner. Real-world urban driving is closer to
// 2-3 m/s² for "non-urgent" stops; we pick the upper end to keep cars
// behaving briskly without slamming the brakes.
const cornerBrakingDecel = 2.5

// cornerReactionBuf is the lookahead time (seconds) added on top of the
// pure braking distance, so vehicles start to slow a beat before they
// strictly must.
const cornerReactionBuf = 2.0

// shouldApplyCornerCap reports whether a vehicle traveling at speed v
// needs to begin braking now to reach cap by distToCorner. Returns true
// if we're at or below the cap already (so the cap stays applied through
// the corner).
func shouldApplyCornerCap(v, cap, distToCorner float64) bool {
	if v <= cap {
		return true
	}
	brakeDist := (v*v - cap*cap) / (2 * cornerBrakingDecel)
	return distToCorner < brakeDist+v*cornerReactionBuf
}

// computeDesiredSpeed returns the v0 (desired speed) for IDM given the
// current vehicle state and route. It's the current edge's speed limit,
// optionally reduced by the corner-speed cap for the upcoming turn when
// the vehicle is within braking distance.
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
	angle := math.Abs(network.TurnAngle(w.Net, v.Edge, nextEdge))
	cap := cornerSpeedCap(angle)
	if math.IsInf(cap, 1) || cap >= v0 {
		return v0
	}
	distToCorner := edge.Length - v.S
	if shouldApplyCornerCap(v.V, cap, distToCorner) {
		return cap
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
