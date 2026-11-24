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
	// complete stop at the stop line at the end of its current edge. Zero
	// means "not stopped at a stop line." Reset when the vehicle crosses
	// that line. Per-line state: see linesAhead.
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
	// gap-acceptance at the line at the end of its current edge, while on
	// that edge or at the line itself. Monotonic until the vehicle crosses
	// that line (stepIDM) — intentionally: once impatience accepts the gap,
	// WaitTime stays high so criticalGap stays low, committing the vehicle
	// to crossing. Resetting on movement would cause a flip-flop
	// oscillation at the line. Does NOT apply to red lights. Per-line
	// state: see linesAhead.
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

	// rerouteGen is the World.incidentGen this vehicle's route was last
	// planned with (spawn or reroute attempt). maybeReroute skips the
	// cooldown for a closure ahead only while it differs.
	rerouteGen uint64

	// YellowCall is the stop-or-go decision this vehicle made for the yellow
	// light at the end of its current edge (yellowGo / yellowStop), or
	// yellowUndecided. Cleared on green and when the vehicle crosses that
	// line. See World.redAt. Per-line state: see linesAhead.
	YellowCall int8

	// heldBy is 1 + the route index of the line that held the vehicle (a
	// stopped virtual leader) last tick, or 0 when none did. The next tick's
	// arrivals and detector scan read it, before anyone moves: a vehicle
	// held by a line does not reach beyond it. (int32, beside YellowCall,
	// to sit in padding: Vehicle is copied every tick.)
	heldBy int32

	// queueRank and queueLine carry the stuck guard's queue state to the next
	// tick (World.Step): rank 1 is a vehicle held at its line, rank k one k-1
	// vehicles behind such a vehicle in a queue for the same line, and 0 none
	// of these. queueLine is the edge whose end is that line.
	queueRank uint16
	queueLine network.EdgeID

	// linesAhead holds the same per-line state (dwell, wait, yellow call)
	// for stop lines beyond the current edge, sorted by route index. The
	// lookahead lets a vehicle stop, dwell or decide on a yellow for a line
	// beyond a short edge while still on the edge before it, and two lines
	// can be closer together than a stopping gap, so every line keeps its
	// own state. When the vehicle reaches the edge a memo's line ends, the
	// memo moves into the three fields above (see lineAt / setLineAt).
	linesAhead []lineMemo

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

		// We've crossed the line at the end of the previous edge: clear its
		// mandatory-stop timestamp, impatience and yellow decision, and take
		// up any state already gathered for the line ahead (a car that
		// stopped for it short of this node mustn't stop again).
		v.StoppedSinceSec, v.WaitTime, v.YellowCall = 0, 0, yellowUndecided
		v.promoteLine()

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

// lineState is a vehicle's memory of one stop line: when it stopped there
// (StoppedSinceSec), how long it has waited to cross (WaitTime) and its
// yellow decision (YellowCall).
type lineState struct {
	stoppedSince, wait float64
	call               int8
}

func (s lineState) empty() bool {
	return s.stoppedSince == 0 && s.wait == 0 && s.call == yellowUndecided
}

// lineMemo is the lineState of the line at the end of Route[idx], a line
// beyond the current edge. edge records Route[idx] so a reroute that changes
// the route there drops the memo.
type lineMemo struct {
	idx  int
	edge network.EdgeID
	lineState
}

// lineAt returns the vehicle's state for the line at the end of route index
// j: the StoppedSinceSec/WaitTime/YellowCall fields for its current edge, a
// memo for a line ahead, or empty state.
func (v *Vehicle) lineAt(j int) lineState {
	if j == v.RouteIdx {
		return lineState{v.StoppedSinceSec, v.WaitTime, v.YellowCall}
	}
	for i := range v.linesAhead {
		if m := &v.linesAhead[i]; m.idx == j {
			if j < len(v.Route) && v.Route[j] == m.edge {
				return m.lineState
			}
			break
		}
	}
	return lineState{}
}

// setLineAt stores the state for the line at the end of route index j.
// Lines behind the vehicle are ignored; empty state for a line ahead drops
// its memo.
func (v *Vehicle) setLineAt(j int, s lineState) {
	if j == v.RouteIdx {
		v.StoppedSinceSec, v.WaitTime, v.YellowCall = s.stoppedSince, s.wait, s.call
		return
	}
	if j < v.RouteIdx || j >= len(v.Route) {
		return
	}
	k := 0
	for k < len(v.linesAhead) && v.linesAhead[k].idx < j {
		k++
	}
	if k < len(v.linesAhead) && v.linesAhead[k].idx == j {
		if s.empty() {
			v.linesAhead = append(v.linesAhead[:k], v.linesAhead[k+1:]...)
		} else {
			v.linesAhead[k] = lineMemo{idx: j, edge: v.Route[j], lineState: s}
		}
		return
	}
	if s.empty() {
		return
	}
	v.linesAhead = append(v.linesAhead, lineMemo{})
	copy(v.linesAhead[k+1:], v.linesAhead[k:])
	v.linesAhead[k] = lineMemo{idx: j, edge: v.Route[j], lineState: s}
}

// clusterStop returns the vehicle's all-way-stop FIFO time for the line at
// the end of route index j: the earliest StoppedSinceSec among that line and
// the lines before it the vehicle has stopped at but not crossed, or 0. A
// stop at an earlier line survives only while the vehicle is still short of
// it, which happens only when line j lies within a stopping gap beyond it,
// so those lines form one junction and the vehicle's turn is its arrival at
// the first of them.
func (v *Vehicle) clusterStop(j int) float64 {
	t := 0.0
	take := func(s float64) {
		if s != 0 && (t == 0 || s < t) {
			t = s
		}
	}
	if j >= v.RouteIdx {
		take(v.StoppedSinceSec)
	}
	for i := range v.linesAhead {
		m := &v.linesAhead[i]
		if m.idx > j {
			break
		}
		if m.idx < len(v.Route) && v.Route[m.idx] == m.edge {
			take(m.stoppedSince)
		}
	}
	return t
}

// promoteLine runs after an edge transition: it drops memos for lines now
// behind the vehicle and moves the memo for the new current edge's line, if
// any, into the StoppedSinceSec/WaitTime/YellowCall fields.
func (v *Vehicle) promoteLine() {
	n := 0
	for n < len(v.linesAhead) && v.linesAhead[n].idx < v.RouteIdx {
		n++
	}
	if n < len(v.linesAhead) && v.linesAhead[n].idx == v.RouteIdx {
		if m := v.linesAhead[n]; m.edge == v.Edge {
			v.StoppedSinceSec, v.WaitTime, v.YellowCall = m.stoppedSince, m.wait, m.call
		}
		n++
	}
	v.linesAhead = v.linesAhead[n:]
}

// dropStaleLines discards memos whose line is no longer on the route at the
// same index, after a reroute replaced the route's tail.
func (v *Vehicle) dropStaleLines() {
	kept := v.linesAhead[:0]
	for _, m := range v.linesAhead {
		if m.idx < len(v.Route) && v.Route[m.idx] == m.edge {
			kept = append(kept, m)
		}
	}
	v.linesAhead = kept
}
