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
	// complete stop at its current approach's stop line. Zero means
	// "not currently stopped at a stop line." Reset to zero when the
	// vehicle transitions to a new edge.
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
	// gap-acceptance (mustYield or mustYieldLT). Monotonic within an
	// approach edge — the ONLY reset point is the edge transition in
	// stepIDM. This is intentional: once impatience accepts the gap,
	// WaitTime stays high so effectiveGap stays low, committing the
	// vehicle to crossing. Resetting on movement would cause a
	// flip-flop oscillation at the line. Does NOT apply to red lights.
	WaitTime float64

	// LastLCDir records the direction of the most recent lane change in
	// human terms: +1 = moved left (higher lane index, toward centerline),
	// -1 = moved right (lower lane index, toward curb), 0 = no recent
	// change. Valid while LaneChangeCooldown > 0; rendered as a blinking
	// turn-signal dot during that window.
	LastLCDir int8

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
		v.RouteIdx++
		if v.RouteIdx >= len(v.Route) {
			v.Despawned = true
			v.S = 0
			return
		}
		v.Edge = v.Route[v.RouteIdx]
		edge = &net.Edges[v.Edge]

		// Clear any mandatory-stop arrival timestamp and accumulated
		// impatience now that we've left the prior approach.
		v.StoppedSinceSec = 0
		v.WaitTime = 0

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
