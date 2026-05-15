package sim

import (
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

		// Lane carry-over: pick the new lane based on the just-completed
		// turn. This is both the normal post-turn carry-over AND the snap
		// fallback when bias didn't get us to a compatible lane in time.
		cat := network.ClassifyTurn(net, prevEdge, v.Edge)
		nLanes := uint8(len(edge.Lanes))
		switch cat {
		case network.TurnRight:
			v.Lane = 0
		case network.TurnLeft:
			if nLanes > 0 {
				v.Lane = nLanes - 1
			} else {
				v.Lane = 0
			}
		case network.TurnStraight:
			if uint8(prevLane) >= nLanes && nLanes > 0 {
				v.Lane = nLanes - 1
			} else {
				v.Lane = prevLane
			}
		default: // TurnUTurn or unclassified
			v.Lane = 0
		}
	}
}
