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
