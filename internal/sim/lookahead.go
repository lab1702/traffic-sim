package sim

// Route lookahead.
//
// netbuild splits OSM ways at every shared node and every signal node, so
// 4-20 m edges are common. A check that looks one edge ahead therefore sees
// almost nothing: a stopped car, a stop line or a corner just beyond a short
// edge is as close as one at the end of the current edge, and it used to be
// invisible until the vehicle was on top of it (cars drove through queues,
// ran reds, pulled out in front of priority traffic).
//
// Every look ahead now walks the remaining route from the front bumper, edge
// by edge, accumulating the distance to each edge's start and end, until the
// horizon lookaheadHorizon(v) = max(150 m, v²/(2B) + 50 m): the IDM
// comfortable stopping distance plus a margin. The walk feeds four things:
//
//  1. Leader (leaderAhead): the first vehicle in the lane the vehicle will be
//     in on each edge, carried through every node with postTurnLane, the same
//     mapping the post-turn lane snap applies.
//  2. Next stop (nextStop): each edge end within the horizon is a "line" where
//     the red/yellow, yield/stop/all-way-stop, left-turn-across-traffic and
//     incident entry-block checks run; the first line with any obligation
//     becomes the stopped virtual leader. The current edge's line is always
//     checked, however far away it is, as before.
//  3. Gap checks look upstream through arrivals. Once per tick (buildArrivals)
//     every vehicle records, for each edge ending at an intersection within
//     reach (maxCriticalGap seconds at its speed, and at least
//     stopLineTolMeters), its distance to that edge's end and its speed. A gap
//     check at junction X scans the arrivals recorded on X's approaches, so a
//     priority car two short edges back counts, and only cars whose own route
//     crosses X by that approach do. maxCriticalGap bounds every gap a driver
//     accepts, so a vehicle out of reach could never have been inside one.
//  4. Desired speed (computeDesiredSpeed): every corner and every lower speed
//     limit within the horizon caps v0 with the kinematic approach profile the
//     current-edge corner already used.
//
// Spawns use the same walk (spawnEntry): the newcomer must have room behind
// the first vehicle anywhere ahead on its route and be able to stop for a
// hard stop ahead, and followers upstream of short edges are checked too.
// Two consumers elsewhere follow suit: semi-actuated detector zones
// (scanDetectors) are measured along the route, because a car now stops ~7 m
// before a red that may lie beyond a shorter approach edge; and turn-lane
// bias (routeLaneMask, lanechange.go) plans the lane through edges too short
// to change lanes on.
//
// Per-line state. StoppedSinceSec (stop-sign dwell), WaitTime (impatience) and
// YellowCall (the remembered yellow decision) used to belong to the current
// edge and reset on every edge transition. A car now stops for a line beyond
// a short edge while still on the edge before it, so that reset would make it
// stop twice, or forget the gap or yellow call it committed to, at the node in
// between. The state now belongs to one line, Vehicle.LineIdx (the route index
// of the edge that line ends), and resets only when the vehicle crosses that
// line or reroutes past it. A line takes the slot when the slot is empty or
// when it is nearer than the line holding it; a farther line is judged with
// fresh state that is not stored.
//
// Determinism: walks follow the route slice; arrivals are appended in vehicle
// index order and use start-of-tick positions and speeds; nearest-wins scans
// break ties toward the lower vehicle index. No map order reaches a result.
//
// Cost: scratch storage is reused across ticks and indexed by EdgeID or
// vehicle index, the intersection at each edge's end is resolved once in
// NewWorld (endX), and each turn's category and corner speed are cached on
// first use (turnAt), so a step of the walk is a few slice reads.

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

const (
	// lookaheadMin and lookaheadMargin set the lookahead horizon: at least
	// lookaheadMin, and the comfortable IDM stopping distance v²/(2B) plus
	// lookaheadMargin at speed.
	lookaheadMin    = 150.0 // m
	lookaheadMargin = 50.0  // m

	// minETASpeed floors the speed used to turn a distance into an arrival
	// time, so a stopped car right at the line still reads as "imminent".
	minETASpeed = 0.5 // m/s

	// maxCriticalGap is the largest gap, in seconds, any driver accepts: the
	// largest base gap times the most cautious GapFactor (impatience only
	// shrinks it). Arrivals are recorded out to this many seconds of travel.
	maxCriticalGap = max(gapThresholdSec, leftTurnGapSec, roundaboutGapSec) * gapFactorMax

	// maxSpawnUpstreamEdges bounds the upstream search spawnFollowersSafe
	// makes through empty edges.
	maxSpawnUpstreamEdges = 256
)

// lookaheadHorizon is how far ahead along its route a vehicle at speed v
// looks for leaders, stop lines and corners.
func lookaheadHorizon(v float64) float64 {
	if h := v*v/(2*DefaultIDM().B) + lookaheadMargin; h > lookaheadMin {
		return h
	}
	return lookaheadMin
}

// turnInfo caches what the sim needs to know about one turn.
type turnInfo struct {
	to    network.EdgeID
	cat   network.TurnCategory
	vSafe float64 // comfortable corner speed; +Inf when the turn needs no slowing
}

// turnAt returns the classification and corner speed of the turn from edge
// `from` onto edge `to`, computing it on first use.
func (w *World) turnAt(from, to network.EdgeID) turnInfo {
	cached := int(from) < len(w.turns)
	if cached {
		for _, t := range w.turns[from] {
			if t.to == to {
				return t
			}
		}
	}
	t := turnInfo{to: to, cat: network.ClassifyTurn(w.Net, from, to), vSafe: math.Inf(1)}
	// Bends gentler than minCornerAngle keep full speed (drivers don't lift
	// for slight bends).
	if r, deflection := turnGeometry(w.Net, from, to); deflection >= minCornerAngle {
		t.vSafe = cornerSpeed(r)
	}
	if cached {
		w.turns[from] = append(w.turns[from], t)
	}
	return t
}

// laneAfter is the lane a vehicle in `lane` of edge `from` will be in on
// edge `to` (see postTurnLane).
func (w *World) laneAfter(lane uint8, from, to network.EdgeID) uint8 {
	return postTurnLane(lane, w.turnAt(from, to).cat, uint8(len(w.Net.Edges[to].Lanes)))
}

// intersectionAt returns the intersection at the downstream end of edge e,
// or nil when that node is not one.
func (w *World) intersectionAt(e network.EdgeID) *network.Intersection {
	if int(e) < len(w.endX) {
		return w.endX[e]
	}
	return w.xByNodeID[w.Net.Edges[e].To]
}

// leaderInfo is a vehicle's leader for this tick, in the vehicle's
// current-edge coordinates: lS is its S plus the distance to the leader's
// front bumper.
type leaderInfo struct {
	lS, lV float64
	has    bool
}

// findLeaders computes every live vehicle's leader from start-of-tick
// positions, before anyone moves, so stepping order cannot matter. The result
// is indexed by vehicle index and reused across ticks.
func (w *World) findLeaders(byEdgeLane map[network.EdgeID]map[uint8][]int) []leaderInfo {
	if cap(w.leaderBuf) < len(w.Vehicles) {
		w.leaderBuf = make([]leaderInfo, len(w.Vehicles), len(w.Vehicles)+len(w.Vehicles)/4)
	}
	leaders := w.leaderBuf[:len(w.Vehicles)]
	clear(leaders)
	// Map iteration order is irrelevant: each vehicle's result depends only on
	// start-of-tick state.
	for eid, lanes := range byEdgeLane {
		for ln, idxs := range lanes {
			for pos, vi := range idxs {
				var info leaderInfo
				if pos+1 < len(idxs) {
					ld := &w.Vehicles[idxs[pos+1]]
					info = leaderInfo{lS: ld.S, lV: ld.V, has: true}
				} else {
					info = w.leaderAhead(&w.Vehicles[vi], byEdgeLane)
				}
				// Lanes that funnel into the same lane of the next edge (a
				// lane drop, or a multi-lane turn onto fewer lanes) merge at
				// the node. Follow whoever is ahead in the merge order, so
				// the two cars don't land on the same spot.
				if mS, mV, ok := w.mergeLeader(vi, ln, eid, lanes); ok && (!info.has || mS < info.lS) {
					info = leaderInfo{lS: mS, lV: mV, has: true}
				}
				leaders[vi] = info
			}
		}
	}
	return leaders
}

// leaderAhead finds the leader of a vehicle that is last in its lane on its
// own edge: the first vehicle in the matching lane of the following route
// edges, within the lookahead horizon. Empty short edges no longer hide the
// queue beyond them.
func (w *World) leaderAhead(v *Vehicle, byEdgeLane map[network.EdgeID]map[uint8][]int) leaderInfo {
	h := lookaheadHorizon(v.V)
	d := w.Net.Edges[v.Edge].Length - v.S // to the start of the next edge
	lane, prev := v.Lane, v.Edge
	for j := v.RouteIdx + 1; j < len(v.Route) && d < h; j++ {
		e := v.Route[j]
		lane = w.laneAfter(lane, prev, e)
		if idxs := byEdgeLane[e][lane]; len(idxs) > 0 {
			ld := &w.Vehicles[idxs[0]]
			return leaderInfo{lS: v.S + d + ld.S, lV: ld.V, has: true}
		}
		d += w.Net.Edges[e].Length
		prev = e
	}
	return leaderInfo{}
}

// arrival is one vehicle approaching the end of one edge, as recorded at the
// start of the tick.
type arrival struct {
	vi   int32   // vehicle index
	ri   int32   // route index of the edge in that vehicle's route
	dist float64 // front bumper to the end of the edge
	v    float64 // speed
}

// buildArrivals records, for every edge that ends at an intersection, the
// vehicles whose route reaches its end within reach: maxCriticalGap seconds
// of travel at their current speed, and at least stopLineTolMeters (the
// all-way-stop queue head may be stopped up to that far back). Called once
// per tick from start-of-tick positions; lists are in vehicle-index order.
func (w *World) buildArrivals() {
	for _, e := range w.arrTouched {
		w.arrivals[e] = w.arrivals[e][:0]
	}
	w.arrTouched = w.arrTouched[:0]
	for vi := range w.Vehicles {
		v := &w.Vehicles[vi]
		if v.Despawned {
			continue
		}
		reach := maxCriticalGap * math.Max(v.V, minETASpeed)
		if reach < stopLineTolMeters {
			reach = stopLineTolMeters
		}
		d := -v.S // to the start of edge e
		for j, e := v.RouteIdx, v.Edge; ; {
			dEnd := d + w.Net.Edges[e].Length
			if dEnd > reach {
				break
			}
			if int(e) < len(w.arrivals) && w.intersectionAt(e) != nil {
				if len(w.arrivals[e]) == 0 {
					w.arrTouched = append(w.arrTouched, e)
				}
				w.arrivals[e] = append(w.arrivals[e], arrival{
					vi: int32(vi), ri: int32(j), dist: math.Max(dEnd, 0), v: v.V,
				})
			}
			j++
			if j >= len(v.Route) {
				break
			}
			d, e = dEnd, v.Route[j]
		}
	}
}

// arrivalsAt returns this tick's arrivals at the end of edge e.
func (w *World) arrivalsAt(e network.EdgeID) []arrival {
	if int(e) < len(w.arrivals) {
		return w.arrivals[e]
	}
	return nil
}

// within reports whether the arrival reaches its line within gap seconds.
func (a *arrival) within(gap float64) bool {
	return a.dist/math.Max(a.v, minETASpeed) < gap
}

// turnsLeftAfter reports whether vehicle ov, recorded by arrival a at the end
// of edge e, turns left there.
func (w *World) turnsLeftAfter(ov *Vehicle, a arrival, e network.EdgeID) bool {
	ri := int(a.ri)
	if at, ok := ov.routeEdgeAt(ri); !ok || at != e || ri+1 >= len(ov.Route) {
		return false
	}
	return w.turnAt(e, ov.Route[ri+1]).cat == network.TurnLeft
}

// stopInfo describes the first line along a vehicle's route at which it must
// stop this tick, if any.
type stopInfo struct {
	dist float64 // front bumper to the line
	j    int     // route index of the edge the line ends

	// What holds the vehicle at the line (all false: nothing does).
	red, yield, leftTurn, incident bool
}

func (s stopInfo) any() bool { return s.red || s.yield || s.leftTurn || s.incident }

// nextStop walks the vehicle's route to the lookahead horizon and returns the
// first line that it must stop at: a red (or a yellow it chose to stop for),
// a yield / stop / all-way-stop it may not pass yet, a left turn across
// imminent opposing traffic, or an incident (a closed lane or road ahead).
// vi is the vehicle's index in w.Vehicles, or -1 for a vehicle that is not
// in it; it keeps a vehicle from yielding to itself.
//
// May update the vehicle's per-line state (YellowCall, StoppedSinceSec), so
// call it once per vehicle per tick.
func (w *World) nextStop(v *Vehicle, vi int) stopInfo {
	h := lookaheadHorizon(v.V)
	lane := v.Lane
	d := -v.S // to the start of edge e
	for j, e := v.RouteIdx, v.Edge; ; {
		dEnd := d + w.Net.Edges[e].Length
		if j > v.RouteIdx && dEnd > h {
			break
		}
		if s := w.lineStop(v, vi, j, e, lane, math.Max(dEnd, 0)); s.any() {
			return s
		}
		j++
		if j >= len(v.Route) {
			break
		}
		next := v.Route[j]
		lane = w.laneAfter(lane, e, next)
		d, e = dEnd, next
	}
	return stopInfo{}
}

// lineStop evaluates the line at the end of edge e (route index j), which the
// vehicle will reach in `lane`, dist metres ahead of its front bumper.
func (w *World) lineStop(v *Vehicle, vi, j int, e network.EdgeID, lane uint8, dist float64) stopInfo {
	s := stopInfo{dist: dist, j: j}
	next, hasNext := network.EdgeID(0), j+1 < len(v.Route)
	if hasNext {
		next = v.Route[j+1]
	}
	if len(w.Incidents) > 0 {
		// The obstacle of a closure sits at the edge's end; a fully blocked
		// next edge is an entry block at this end.
		if w.edgeFullyBlocked(e) {
			s.incident = true
		} else if cl, ok := w.closedLaneFor(e); ok && lane == cl {
			s.incident = true
		} else if hasNext && w.edgeFullyBlocked(next) {
			s.incident = true
		}
	}
	x := w.intersectionAt(e)
	if x == nil {
		return s
	}
	pos := IncomingPos(x, e)
	if pos < 0 {
		return s
	}
	s.red = w.redAt(v, j, e, x, pos, dist)
	if !s.red {
		s.yield = w.yieldAt(v, vi, j, e, next, hasNext, x, pos, dist)
	}
	if !s.red && !s.yield && hasNext {
		s.leftTurn = w.leftTurnBlocked(v, vi, j, e, next, x, pos)
	}
	return s
}

// redAt reports whether the signal at x holds a vehicle approaching on edge e
// (position pos of x.Incoming, route index j) dist metres from the line.
//
// Yellow is a stop-or-go decision, made once when the vehicle first sees the
// yellow and remembered in v.YellowCall (per line, see claimLine) until the
// next green or until it crosses the line. Remembering it matters:
// re-deciding every tick let a car that had chosen to go flip to a panic
// stop when red came. A car that chose to go keeps going into the red only
// while it is within redCommitGrace seconds of the line, i.e. it is
// finishing the crossing it committed to, not running the red because
// something slowed it down.
//
// In flash/off modes "red" means "must yield", not "hard stop"; yieldAt
// handles those.
func (w *World) redAt(v *Vehicle, j int, e network.EdgeID, x *network.Intersection, pos int, dist float64) bool {
	if !x.HasSignal {
		return false
	}
	st := w.SignalStates[x.ID]
	if st == nil || st.Mode != ModeNormal {
		return false
	}
	owned := v.lineIdx() == j
	inPhase := st.GreenFor(pos) // true during both green and yellow for this approach
	if inPhase && !st.IsYellow {
		// Pure green: forget any earlier yellow decision so the next yellow
		// is judged afresh.
		if owned {
			v.YellowCall = yellowUndecided
		}
		return false
	}
	call := yellowUndecided
	if owned {
		call = v.YellowCall
	}
	if inPhase { // yellow
		if call == yellowUndecided {
			call = decideYellow(v.V, w.Net.Edges[e].SpeedLimit, dist, st.YellowRemaining())
			if v.claimLine(j) {
				v.YellowCall = call
			}
		}
		return call != yellowGo
	}
	// Red.
	if call == yellowGo {
		if v.V > 0 && dist <= v.V*redCommitGrace {
			return false // finishing the crossing it committed to on yellow
		}
		v.YellowCall = yellowStop // slowed down too much to clear: stop after all
	}
	return true
}

// yieldAt dispatches on the effective control of approach pos at x:
//
//   - ControlNone:       no obligation.
//   - ControlYield:      gap acceptance against the priority approaches.
//   - ControlStop:       mandatory dwell, then gap acceptance.
//   - ControlAllWayStop: mandatory dwell, then FIFO arbitration.
//
// For signals, effectiveControl maps the mode: ModeNormal is ControlNone
// (redAt owns it); off and flash modes become all-way-stop / stop / none.
//
// Sets v.StoppedSinceSec (via markStopped) when the vehicle first comes to
// rest near a stop or all-way-stop line.
func (w *World) yieldAt(v *Vehicle, vi, j int, e, next network.EdgeID, hasNext bool,
	x *network.Intersection, pos int, dist float64,
) bool {
	switch effectiveControl(w, x, pos) {
	case network.ControlYield:
		return w.gapBlocked(v, vi, j, x, pos)
	case network.ControlStop:
		if !w.dwelled(v, j) {
			w.markStopped(v, j, dist)
			return true
		}
		return w.gapBlocked(v, vi, j, x, pos)
	case network.ControlAllWayStop:
		if !w.dwelled(v, j) {
			w.markStopped(v, j, dist)
			return true
		}
		return w.allWayStopBlocked(v, vi, j, e, next, hasNext, x, pos)
	}
	return false
}

// dwelled reports whether the vehicle has completed its mandatory-stop dwell
// at line j. False both before reaching the line and during the dwell.
func (w *World) dwelled(v *Vehicle, j int) bool {
	since, _, _ := v.lineState(j)
	return since != 0 && w.SimTime-since >= stopDwellSec
}

// markStopped stamps StoppedSinceSec the first tick the vehicle is at rest
// within stopLineTolMeters of line j. Using stuckSpeedThresh means the dwell
// only starts once the car has genuinely stopped, not while it rolls.
func (w *World) markStopped(v *Vehicle, j int, dist float64) {
	if since, _, _ := v.lineState(j); since != 0 {
		return
	}
	if v.V < stuckSpeedThresh && dist < stopLineTolMeters && v.claimLine(j) {
		v.StoppedSinceSec = w.SimTime
	}
}

// gapBlocked does ETA-based gap acceptance at line j against every approach
// of x whose effective control is ControlNone (the priority approaches),
// including vehicles still upstream of a short approach edge. True when the
// vehicle must wait.
func (w *World) gapBlocked(v *Vehicle, vi, j int, x *network.Intersection, pos int) bool {
	_, wait, _ := v.lineState(j)
	for k, ie := range x.Incoming {
		if k == pos || effectiveControl(w, x, k) != network.ControlNone {
			continue
		}
		arr := w.arrivalsAt(ie)
		if len(arr) == 0 {
			continue
		}
		base := gapThresholdSec
		if w.Net.Edges[ie].Roundabout {
			base = roundaboutGapSec
		}
		g := criticalGap(v, base, wait)
		for i := range arr {
			if int(arr[i].vi) != vi && arr[i].within(g) {
				return true
			}
		}
	}
	return false
}

// allWayStopBlocked arbitrates an all-way stop after the vehicle has served
// its dwell at line j: it yields to any approach whose head vehicle stopped
// at its line earlier (FIFO), ties going to the lower Incoming index. Two
// left turners from opposing approaches both proceed (mutual-left pass).
func (w *World) allWayStopBlocked(v *Vehicle, vi, j int, e, next network.EdgeID, hasNext bool,
	x *network.Intersection, pos int,
) bool {
	vLeft := hasNext && w.turnAt(e, next).cat == network.TurnLeft
	mySince, _, _ := v.lineState(j)
	for k, ie := range x.Incoming {
		if k == pos {
			continue
		}
		// The head of approach k: the arrival nearest its line (ties to the
		// lower vehicle index: arrivals are in index order).
		arr := w.arrivalsAt(ie)
		head := -1
		for i := range arr {
			if int(arr[i].vi) != vi && (head < 0 || arr[i].dist < arr[head].dist) {
				head = i
			}
		}
		if head < 0 {
			continue
		}
		lead := &w.Vehicles[arr[head].vi]
		since := lead.stoppedSinceAt(ie)
		if since == 0 {
			continue // not yet stopped; hasn't earned a FIFO slot
		}
		// Mutual-left pass: both turning left from opposing approaches.
		if vLeft && pos < len(x.Opposing) && int(x.Opposing[pos]) == k && w.turnsLeftAfter(lead, arr[head], ie) {
			continue
		}
		if since < mySince || (since == mySince && k < pos) {
			return true
		}
	}
	return false
}

// leftTurnBlocked reports whether a vehicle turning left from edge e onto
// next at x (it may otherwise proceed) must wait for opposing traffic with an
// imminent ETA, including traffic still upstream of a short opposing edge.
// Opposing left turners are skipped: they yield to us, and we don't yield to
// them (mutual-yield deadlock resolution).
func (w *World) leftTurnBlocked(v *Vehicle, vi, j int, e, next network.EdgeID, x *network.Intersection, pos int) bool {
	if pos >= len(x.Opposing) {
		return false
	}
	opp := int(x.Opposing[pos])
	if opp < 0 || opp >= len(x.Incoming) {
		return false
	}
	if w.turnAt(e, next).cat != network.TurnLeft {
		return false
	}
	oe := x.Incoming[opp]
	arr := w.arrivalsAt(oe)
	if len(arr) == 0 {
		return false
	}
	_, wait, _ := v.lineState(j)
	g := criticalGap(v, leftTurnGapSec, wait)
	for i := range arr {
		a := &arr[i]
		if int(a.vi) == vi || !a.within(g) {
			continue
		}
		if w.turnsLeftAfter(&w.Vehicles[a.vi], *a, oe) {
			continue
		}
		return true
	}
	return false
}

// criticalGap returns the gap (seconds of priority-vehicle ETA) v accepts for
// a manoeuvre whose base critical gap is baseGap, after waiting `wait`
// seconds at the line: baseGap scaled by the driver's GapFactor, shrinking
// linearly with the wait, floored at minAcceptedGap.
//
// A non-positive GapFactor (hand-built test vehicles) counts as 1.0, and the
// factor is capped at gapFactorMax so maxCriticalGap stays a true bound.
func criticalGap(v *Vehicle, baseGap, wait float64) float64 {
	factor := v.GapFactor
	if factor <= 0 {
		factor = 1.0
	}
	if factor > gapFactorMax {
		factor = gapFactorMax
	}
	g := baseGap*factor - impatienceDecayRate*wait
	if g < minAcceptedGap {
		g = minAcceptedGap
	}
	return g
}

// computeDesiredSpeed returns the v0 (desired speed) for IDM: the current
// edge's speed limit, scaled by the driver's preference and capped by any
// Slowdown incident, then eased down for every corner and every lower limit
// within the lookahead horizon. Each such constraint at distance d caps v0 at
// sqrt(vTarget² + 2·cornerBrakeDecel·d), the speed from which the vehicle can
// still slow to vTarget by gentle braking, so it eases into the corner or the
// slower road instead of braking hard at the last moment.
func (w *World) computeDesiredSpeed(v *Vehicle) float64 {
	// Per-driver speed preference. A zero factor means a hand-constructed
	// vehicle (test fixture): treat as 1.0.
	factor := v.SpeedFactor
	if factor == 0 {
		factor = 1.0
	}
	v0 := w.edgeDesiredSpeed(v.Edge, factor)
	h := math.Max(lookaheadHorizon(v.V), v0*v0/(2*cornerBrakeDecel))
	capAt := func(vTarget, d float64) {
		if vTarget >= v0 {
			return
		}
		if c := math.Sqrt(vTarget*vTarget + 2*cornerBrakeDecel*math.Max(d, 0)); c < v0 {
			v0 = c
		}
	}
	e := v.Edge
	d := w.Net.Edges[e].Length - v.S // to the end of edge e
	for j := v.RouteIdx + 1; j < len(v.Route) && d <= h; j++ {
		next := v.Route[j]
		capAt(w.turnAt(e, next).vSafe, d)
		capAt(w.edgeDesiredSpeed(next, factor), d)
		e = next
		d += w.Net.Edges[e].Length
	}
	return v0
}

// edgeDesiredSpeed is a driver's cruising speed on edge e: its limit times
// the driver's factor, capped while a Slowdown incident is on the edge.
func (w *World) edgeDesiredSpeed(e network.EdgeID, factor float64) float64 {
	edge := &w.Net.Edges[e]
	v0 := edge.SpeedLimit * factor
	if len(w.Incidents) > 0 && w.Incidents[e] == Slowdown {
		if c := edge.SpeedLimit * incidentSlowdownFactor; c < v0 {
			v0 = c
		}
	}
	return v0
}
