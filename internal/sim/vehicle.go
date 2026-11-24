package sim

import (
	"log/slog"
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

type VehicleID uint32

// VehicleLength is the bumper-to-bumper length used for gap calculation.
const VehicleLength = 5.0 // meters

type Vehicle struct {
	ID       VehicleID
	Route    []network.EdgeID
	RouteIdx int
	Edge     network.EdgeID
	Lane     uint8
	S        float64 // meters along edge, measured at front bumper
	V        float64 // m/s
	A        float64 // m/s^2 (last computed accel; useful for tracing)

	// LaneChangeCooldown counts down in seconds. Cannot change lanes
	// again until this reaches 0. Prevents oscillation.
	LaneChangeCooldown float64

	// StuckTime accumulates sim-seconds where V < stuckSpeedThresh and the
	// vehicle is not legitimately waiting at a red light or yield. Resets
	// to 0 whenever any of those conditions fails. When it exceeds
	// stuckTimeoutSec the vehicle is logged at WARN and despawned.
	StuckTime float64

	// StoppedSinceSec is the sim-time at which this vehicle came to a
	// complete stop at the stop line LineIdx names. Zero means "not
	// stopped at a stop line." Per-line state: see LineIdx.
	StoppedSinceSec float64

	// SpeedFactor is a per-driver multiplier on the edge speed limit,
	// sampled once at spawn from a normal distribution. Typical range
	// [0.95, 1.05] around mean 1.0. A zero value is treated as 1.0 to
	// keep hand-constructed test vehicles working without modification.
	SpeedFactor float64

	// GapFactor is a per-driver multiplier on critical-gap thresholds
	// (gapThresholdSec for straight crossings, leftTurnGapSec for left
	// turns). Sampled at spawn from Normal(1.0, gapFactorStdDev) and
	// clamped to [gapFactorMin, gapFactorMax]. A zero value is treated
	// as 1.0 to keep hand-constructed test vehicles working without
	// modification.
	GapFactor float64

	// WaitTime accumulates sim-seconds during which the vehicle is
	// effectively stopped (V < stuckSpeedThresh) AND yielding via
	// gap-acceptance at the line LineIdx names. Monotonic until the
	// vehicle crosses that line (stepIDM) — intentionally: once
	// impatience accepts the gap, WaitTime stays high so criticalGap
	// stays low, committing the vehicle to crossing. Resetting on
	// movement would cause a flip-flop oscillation at the line. Does NOT
	// apply to red lights. Per-line state: see LineIdx.
	WaitTime float64

	// LastLCDir records the direction of the most recent lane change in
	// human terms: +1 = moved left (higher lane index, toward centerline),
	// -1 = moved right (lower lane index, toward curb), 0 = no recent
	// change. Valid while LaneChangeCooldown > 0; rendered as a blinking
	// turn-signal dot during that window.
	LastLCDir int8

	// HasGPS marks a vehicle that re-routes around congestion. Set at spawn
	// from World.GpsShare. Hand-constructed test vehicles default to false
	// (zero value) and are never re-routed, so existing fixtures are
	// unaffected.
	HasGPS bool

	// DestNode is the vehicle's destination node, cached at spawn so
	// re-routing needn't re-derive it from the route tail. Equal to the To
	// node of the final route edge.
	DestNode network.NodeID

	// LastRerouteSec is the sim-time of this vehicle's most recent reroute
	// attempt (or spawn). Gates re-routing to at most once per
	// rerouteCooldownSec so a vehicle doesn't recompute on every short edge.
	LastRerouteSec float64

	// YellowCall is the stop-or-go decision this vehicle made for the yellow
	// light at the line LineIdx names (yellowGo / yellowStop), or
	// yellowUndecided. Cleared on green and when the vehicle crosses that
	// line. See World.redAt. Per-line state: see LineIdx.
	YellowCall int8

	// LineIdx is the route index of the edge whose downstream end is the
	// stop line that StoppedSinceSec, WaitTime and YellowCall describe. The
	// lookahead lets a vehicle stop for a line beyond a short edge while
	// still on the edge before it, so that state must survive the edge
	// transitions in between: it resets only when the vehicle crosses this
	// line (or reroutes past it). A value behind RouteIdx (the zero value
	// in hand-built fixtures) means the current edge. See claimLine.
	LineIdx int

	Despawned bool
}

// stepIDM advances vehicle i by one tick using IDM with the supplied
// leader and an explicit desired speed (v0). leader may be nil; if
// non-nil, both vehicles are assumed to be on the same edge or on the
// cross-edge lookahead path (world.go handles the framing).
//
// Passing v0 explicitly lets the caller blend the edge speed limit with
// caps for upcoming corners, signals, etc.
func stepIDM(v *Vehicle, v0 float64, leaderS float64, leaderV float64, hasLeader bool,
	net *network.Network, params IDMParams, dt float64,
) {
	if v.Despawned {
		return
	}
	edge := &net.Edges[v.Edge]
	if v0 <= 0 {
		v0 = edge.SpeedLimit
	}

	gap := math.Inf(1)
	deltaV := 0.0
	if hasLeader {
		gap = leaderS - v.S - VehicleLength
		if gap < 0 {
			gap = 0
		}
		deltaV = v.V - leaderV
	}
	v.A = IDMAcceleration(params, v.V, v0, gap, deltaV)
	v.V += v.A * dt
	if v.V < 0 {
		v.V = 0
	}
	v.S += v.V * dt

	for v.S >= edge.Length {
		v.S -= edge.Length
		prevEdge := v.Edge
		prevLane := v.Lane
		line := v.lineIdx()
		v.RouteIdx++
		if v.RouteIdx >= len(v.Route) {
			v.Despawned = true
			v.S = 0
			return
		}
		v.Edge = v.Route[v.RouteIdx]
		edge = &net.Edges[v.Edge]

		// Clear the mandatory-stop timestamp, accumulated impatience and
		// yellow decision once we've crossed the line they belong to. A node
		// before that line (the end of a short edge leading up to it) keeps
		// them, so a car that stopped short of the line doesn't stop again.
		if v.RouteIdx > line {
			v.resetLine(v.RouteIdx)
		}

		// Lane carry-over: pick the new lane based on the just-completed
		// turn. This is both the normal post-turn carry-over AND the snap
		// fallback when bias didn't get us to a compatible lane in time.
		cat := network.ClassifyTurn(net, prevEdge, v.Edge)

		// Diagnostic: warn when the previous lane was incompatible with the
		// just-taken turn — bias didn't get us there, so this snap is a teleport.
		prevLanes := net.Edges[prevEdge].Lanes
		if int(prevLane) < len(prevLanes) {
			allowed := prevLanes[prevLane].AllowedTurns
			if len(allowed) > 0 {
				compat := false
				for _, e := range allowed {
					if e == v.Edge {
						compat = true
						break
					}
				}
				if !compat {
					slog.Warn("turn-lane snap fallback",
						"vehicle_id", v.ID,
						"prev_edge", prevEdge,
						"prev_lane", prevLane,
						"new_edge", v.Edge,
						"turn_cat", cat,
					)
				}
			}
		}

		nLanes := uint8(len(edge.Lanes))
		v.Lane = postTurnLane(prevLane, cat, nLanes)
	}
}

// postTurnLane returns the lane a vehicle will occupy on the outbound edge
// after a turn of category `cat`, given its lane on the inbound edge
// (`prevLane`) and the outbound edge's lane count (`nLanes`).
//
// Right turns snap to lane 0; left turns snap to the highest lane; straights
// hold their lane (clamped if the outbound has fewer lanes). U-turns and
// unclassified fall to lane 0. Returns 0 when nLanes == 0.
//
// Used by both the post-turn lane snap in advanceVehicle and the cross-edge
// leader lookup in world.Step — keep them in sync, otherwise a right-turner
// will look for a leader in the wrong lane and miss the vehicle it snaps
// behind on the next tick.
func postTurnLane(prevLane uint8, cat network.TurnCategory, nLanes uint8) uint8 {
	if nLanes == 0 {
		return 0
	}
	switch cat {
	case network.TurnRight:
		return 0
	case network.TurnLeft:
		return nLanes - 1
	case network.TurnStraight:
		if prevLane >= nLanes {
			return nLanes - 1
		}
		return prevLane
	default: // TurnUTurn or unclassified
		return 0
	}
}

// lineIdx is the route index of the line the per-line state refers to (see
// Vehicle.LineIdx); a stale value behind the vehicle means its current edge.
func (v *Vehicle) lineIdx() int {
	if v.LineIdx < v.RouteIdx {
		return v.RouteIdx
	}
	return v.LineIdx
}

// lineState returns StoppedSinceSec, WaitTime and YellowCall as they apply
// to the line at the end of route index j: the stored values when the state
// belongs to that line, fresh (zero) state otherwise.
func (v *Vehicle) lineState(j int) (stoppedSince, wait float64, call int8) {
	if v.lineIdx() == j {
		return v.StoppedSinceSec, v.WaitTime, v.YellowCall
	}
	return 0, 0, yellowUndecided
}

// claimLine readies the per-line state to be written on behalf of the line
// at the end of route index j, and reports whether it may be. The state
// moves to j when it is empty or when j is nearer than the line holding it
// (the vehicle has to deal with the nearer line first). A farther line may
// not take it: the nearer line's dwell, wait or yellow call still matters.
func (v *Vehicle) claimLine(j int) bool {
	li := v.lineIdx()
	switch {
	case j == li:
		v.LineIdx = j
		return true
	case j < li || (v.StoppedSinceSec == 0 && v.WaitTime == 0 && v.YellowCall == yellowUndecided):
		v.resetLine(j)
		return true
	}
	return false
}

// resetLine clears the per-line state and points it at route index j.
func (v *Vehicle) resetLine(j int) {
	v.StoppedSinceSec = 0
	v.WaitTime = 0
	v.YellowCall = yellowUndecided
	v.LineIdx = j
}

// stoppedSinceAt returns StoppedSinceSec when the vehicle's per-line state
// belongs to the line at the end of edge e, and 0 otherwise.
func (v *Vehicle) stoppedSinceAt(e network.EdgeID) float64 {
	if at, ok := v.routeEdgeAt(v.lineIdx()); ok && at == e {
		return v.StoppedSinceSec
	}
	return 0
}

// routeEdgeAt returns the edge at route index i: the current edge for
// i == RouteIdx (hand-built fixtures may leave Route empty), else Route[i].
func (v *Vehicle) routeEdgeAt(i int) (network.EdgeID, bool) {
	if i == v.RouteIdx {
		return v.Edge, true
	}
	if i >= 0 && i < len(v.Route) {
		return v.Route[i], true
	}
	return 0, false
}
