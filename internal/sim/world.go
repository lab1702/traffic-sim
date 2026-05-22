package sim

import (
	"log/slog"
	"math"
	"math/rand/v2"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// signalLast records the last-known signal state for deterministic change detection.
type signalLast struct {
	idx    int
	yellow bool
}

// ControlEvent is a runtime command from the UI to the sim. Today only
// signal-mode changes; extend with new fields/variants as needed.
type ControlEvent struct {
	IntersectionID network.IntersectionID
	Mode           SignalMode
}

// World owns mutable simulation state. Only the sim goroutine touches it.
type World struct {
	Net     *network.Network
	Router  *Router
	Spawner Spawner
	Vehicles []Vehicle
	Tick    uint64
	SimTime float64
	dt      float64

	nextID VehicleID

	// SignalStates is indexed by IntersectionID; nil entries mean no signal.
	SignalStates []*SignalState

	// xByNodeID is a NodeID -> Intersection index for O(1) lookup during tick.
	xByNodeID map[network.NodeID]*network.Intersection

	// SnapshotBuf is read by the renderer; published once per tick.
	SnapshotBuf *snapshot.Buffer

	// EmitTrace is called for every trace event. Default is a no-op.
	// The sim never blocks on this function; callers must ensure it returns promptly.
	EmitTrace func(tick uint64, simTime float64, e trace.Event)

	// lastPhase tracks the last-known (PhaseIdx, IsYellow) per SignalState index
	// to detect changes and emit SignalPhase events.
	lastPhase []signalLast

	// lastMode tracks the last-known Mode per SignalState index for emitting
	// SignalModeChange events. A nil slice means uninitialized; on the first
	// tick any non-zero (non-Normal) mode will be emitted.
	lastMode []SignalMode

	// Control delivers runtime UI commands (e.g. mode toggles from clicks).
	// Step drains it non-blocking at the top of each tick. Nil disables.
	Control <-chan ControlEvent

	// rng drives per-vehicle random properties sampled at spawn (currently
	// SpeedFactor). Seeded with a fixed default so two runs of the same
	// scenario produce identical vehicle profiles.
	rng *rand.Rand

	// Cong tracks live per-edge congestion and supplies routing costs.
	Cong *Congestion

	// GpsShare is the fraction of spawned vehicles given GPS rerouting, in
	// [0,1]. Defaults to 1.0 (every vehicle) in NewWorld; overridden from the
	// --gps-share flag.
	GpsShare float64
}

const (
	DefaultDt = 0.05 // 50 ms == 20 Hz

	// SignalLightOffset is how far back from the stop line each per-approach
	// signal indicator is drawn, in meters. Far enough to read distinct
	// colors at zoom, close enough to read as "this is that intersection's".
	// Exported so tracereplay and the renderer can place identical lights.
	SignalLightOffset = 4.0

	// Per-vehicle speed preference: Vehicle.SpeedFactor is sampled at
	// spawn from Normal(1.0, speedFactorStdDev) and clamped to
	// [speedFactorMin, speedFactorMax]. σ = 1.5% puts ~99.7% of draws
	// within ±4.5%, well inside the ±5% clamp.
	speedFactorStdDev = 0.015
	speedFactorMin    = 0.95
	speedFactorMax    = 1.05

	// turnSignalRange is how far before an intersection a vehicle starts
	// signaling its upcoming left/right turn. Real-world driver behavior
	// is roughly 30-50 m before the maneuver.
	turnSignalRange = 50.0
)

// turnSignalFor returns +1 for left, -1 for right, 0 for off. Two
// triggers: a recent lane change (while cooldown active) and an upcoming
// turn within turnSignalRange. The LC signal takes precedence over the
// turn signal when both fire — typically the LC is the more recent
// intent.
func turnSignalFor(net *network.Network, v *Vehicle) int8 {
	if v.LaneChangeCooldown > 0 && v.LastLCDir != 0 {
		return v.LastLCDir
	}
	if v.RouteIdx+1 >= len(v.Route) {
		return 0
	}
	edge := &net.Edges[v.Edge]
	if edge.Length-v.S > turnSignalRange {
		return 0
	}
	switch network.ClassifyTurn(net, v.Edge, v.Route[v.RouteIdx+1]) {
	case network.TurnLeft:
		return 1
	case network.TurnRight:
		return -1
	}
	return 0
}

func NewWorld(net *network.Network, spawner Spawner, overrides map[network.IntersectionID]SignalConfig) *World {
	sigs := make([]*SignalState, len(net.Intersections))
	xByNode := make(map[network.NodeID]*network.Intersection, len(net.Intersections))
	for i := range net.Intersections {
		x := &net.Intersections[i]
		if existing, dup := xByNode[x.NodeID]; dup {
			// netbuild guarantees one intersection per NodeID, but if that
			// invariant breaks (e.g., a future refactor introduces a split),
			// silently shadowing the older entry would make one intersection
			// invisible to the sim. Warn loudly and keep the lower-ID entry.
			slog.Warn("NewWorld: duplicate NodeID across intersections; shadowing one is a sim correctness bug",
				"node_id", x.NodeID, "kept_id", existing.ID, "dropped_id", x.ID)
		}
		xByNode[x.NodeID] = x
		if x.HasSignal {
			if cfg, ok := overrides[x.ID]; ok {
				sigs[x.ID] = NewSignalState(cfg)
			} else {
				sigs[x.ID] = NewSignalState(DefaultSignalConfig(x.Incoming, net))
			}
		}
	}
	return &World{
		Net:          net,
		Router:       NewRouter(net),
		Spawner:      spawner,
		dt:           DefaultDt,
		SignalStates: sigs,
		xByNodeID:    xByNode,
		SnapshotBuf:  snapshot.New(),
		EmitTrace:    func(uint64, float64, trace.Event) {},
		rng:          rand.New(rand.NewPCG(0xCAFE, 0xBEEF)),
		Cong:         NewCongestion(net, ewmaHalfLifeSec, DefaultDt),
		GpsShare:     1.0,
	}
}

// stopDistanceForRed returns (distance to stop line, true) if the vehicle
// is on an incoming edge to a red-signalled intersection and the vehicle
// is approaching it. Returns (0, false) otherwise.
//
// Yellow is treated as soft-red: if the vehicle can comfortably stop
// before the line at IDM's comfortable deceleration, a virtual stop
// leader is applied (the driver elects to stop). Otherwise the vehicle
// commits through, matching real-world dilemma-zone behavior — drivers
// too close to brake without a panic stop carry on and clear before the
// hard red. This prevents the case where a car cruises full-speed
// through the entire yellow window and is still in the intersection
// when the cross-stream turns green.
func (w *World) stopDistanceForRed(v *Vehicle) (float64, bool) {
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok {
		return 0, false
	}
	if !x.HasSignal {
		return 0, false
	}
	st := w.SignalStates[x.ID]
	if st == nil {
		return 0, false
	}
	// In flash/off modes "red" means "must yield" (gap-acceptance), not
	// "hard stop". stopDistanceForYield handles those cases instead.
	if st.Mode != ModeNormal {
		return 0, false
	}
	pos := IncomingPos(x, v.Edge)
	if pos < 0 {
		return 0, false
	}
	inPhase := st.GreenFor(pos) // true during both green and yellow for this approach
	if inPhase && !st.IsYellow {
		// Pure green: cruise through.
		return 0, false
	}
	// Red, or yellow for this approach. Stop line is end of edge.
	dist := edge.Length - v.S
	if dist < 0 {
		dist = 0
	}
	if inPhase && st.IsYellow {
		// Soft-red yellow: commit only when comfortable stop is not
		// possible. Distance check uses the vehicle's current speed —
		// a vehicle already slow (e.g. queued from a prior red) has a
		// short comfortable distance and so stops.
		if dist < comfortableStopDistance(v.V) {
			return 0, false
		}
	}
	return dist, true
}

// comfortableStopDistance is the distance a vehicle moving at speed v
// needs to come to rest using IDM's comfortable deceleration B plus the
// minimum stopping gap S0. Used by the soft-red yellow check to decide
// whether to stop or commit. Conservative: uses comfortable braking, not
// max braking, so cars in the legal-stop zone reliably stop.
func comfortableStopDistance(v float64) float64 {
	p := DefaultIDM()
	return v*v/(2*p.B) + p.S0
}

const gapThresholdSec = 3.0

// leftTurnGapSec is the minimum oncoming-traffic ETA a left turner
// accepts before crossing. Larger than gapThresholdSec because the
// left-turn maneuver takes longer to execute. Literature: 6–8s.
const leftTurnGapSec = 6.0

const (
	// Per-driver gap preference — Normal(1.0, gapFactorStdDev) clamped
	// to [gapFactorMin, gapFactorMax]. Same Normal-then-clamp shape as
	// SpeedFactor but wider, since gap tolerance varies more across
	// drivers than cruising-speed preference.
	gapFactorStdDev = 0.1
	gapFactorMin    = 0.8
	gapFactorMax    = 1.2

	// impatienceDecayRate is the seconds of accepted-gap reduction per
	// second of wait time. At 0.1, a 30-second wait reduces the
	// accepted gap by 3 seconds. Reduction floors at minAcceptedGap.
	impatienceDecayRate = 0.1

	// minAcceptedGap is the lower bound on the accepted gap regardless
	// of wait time. Prevents impatience from producing physically
	// unsafe gaps. 1.5s is on the aggressive end of normal human gap
	// acceptance.
	minAcceptedGap = 1.5
)

const (
	// stuckSpeedThresh is the speed (m/s) below which a vehicle is
	// considered "not moving". Used for two purposes:
	//   1. Stuck-despawn guard: a vehicle below this threshold with no
	//      legitimate red/yield reason accumulates StuckTime and is
	//      eventually despawned.
	//   2. Mandatory-stop arrival: a vehicle near the stop line of a
	//      Stop/AllWayStop approach is considered to have arrived (and
	//      StoppedSinceSec is set) once V crosses this threshold.
	stuckSpeedThresh = 0.1
	// stuckTimeoutSec is the accumulated sim-seconds of below-threshold
	// motion (with no legitimate red/yield reason) that triggers despawn.
	stuckTimeoutSec = 60.0
	// stopDwellSec is the minimum sim-seconds a vehicle must remain
	// effectively stationary at a Stop or AllWayStop line before being
	// allowed to begin gap-acceptance.
	stopDwellSec = 0.5
	// stopLineTolMeters is the maximum distance from the stop line at
	// which a slow-moving vehicle (V < stuckSpeedThresh) is considered
	// to have arrived at the line. IDM's equilibrium gap at v≈0 is
	// S0 (2m) plus the vehicle length (5m), so the front bumper rests
	// about 7m from the end of the edge; 8m gives a 1m margin.
	stopLineTolMeters = 8.0
)

// stopDistanceForYield returns (distance to stop line, true) when the
// vehicle's current edge ends at an intersection where it must wait
// before crossing. Dispatches on the effective Control for this approach:
//
//   - ControlNone:        no obligation; returns (0, false).
//   - ControlYield:       gap-acceptance against ControlNone approaches.
//   - ControlStop:        mandatory dwell, then gap-acceptance.
//   - ControlAllWayStop:  mandatory dwell, then FIFO arbitration.
//
// For signaled intersections, effective control is derived from the
// signal mode: ModeNormal returns immediately (stopDistanceForRed owns
// the hard-stop case); ModeOff/Flash treat each approach as
// AllWayStop/Stop/None as appropriate.
//
// As a side effect, sets v.StoppedSinceSec when v first reaches v ~ 0
// near the stop line of a Stop/AllWayStop approach. v.StoppedSinceSec
// is cleared elsewhere (in stepIDM, on edge transition).
func (w *World) stopDistanceForYield(v *Vehicle, byEdge map[network.EdgeID][]int) (float64, bool) {
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok {
		return 0, false
	}
	myPos := IncomingPos(x, v.Edge)
	if myPos < 0 {
		return 0, false
	}

	effective := effectiveControl(w, x, myPos)
	myDist := edge.Length - v.S
	if myDist < 0 {
		myDist = 0
	}

	switch effective {
	case network.ControlNone:
		return 0, false

	case network.ControlYield:
		return w.yieldGapCheck(v, x, myPos, myDist, byEdge)

	case network.ControlStop:
		if !w.hasDwelled(v) {
			w.maybeMarkStopped(v, myDist)
			return myDist, true
		}
		return w.yieldGapCheck(v, x, myPos, myDist, byEdge)

	case network.ControlAllWayStop:
		// After dwell, yield to any approach whose lead vehicle stopped
		// at the line earlier (FIFO). Tie-break by lower Incoming index.
		if !w.hasDwelled(v) {
			w.maybeMarkStopped(v, myDist)
			return myDist, true
		}
		return w.allWayStopFIFO(v, x, myPos, myDist, byEdge)
	}
	return 0, false
}

// effectiveControl resolves the right-of-way rule for one approach at
// one decision tick. Signal mode overrides the stored IncomingControl.
func effectiveControl(w *World, x *network.Intersection, myPos int) network.Control {
	if x.HasSignal {
		st := w.SignalStates[x.ID]
		if st == nil {
			return network.ControlNone
		}
		switch st.Mode {
		case ModeNormal:
			return network.ControlNone // stopDistanceForRed owns this case
		case ModeOff:
			return network.ControlAllWayStop
		case ModeFlashA, ModeFlashB:
			if st.GreenFor(myPos) {
				return network.ControlNone // blinking yellow has priority
			}
			return network.ControlStop // blinking red is a stop sign
		default:
			// Unknown future signal mode — fail SAFE (treat as AllWayStop)
			// rather than fail OPEN (no control). applyControl already
			// validates incoming Mode values, so reaching this branch
			// implies a future code path forgot to extend the switch.
			return network.ControlAllWayStop
		}
	}
	if myPos < len(x.IncomingControl) {
		return x.IncomingControl[myPos]
	}
	return network.ControlNone
}

// hasDwelled returns true once the vehicle has completed its mandatory-stop
// dwell at the stop line. False both before reaching the line and during
// the dwell window.
func (w *World) hasDwelled(v *Vehicle) bool {
	if v.StoppedSinceSec == 0 {
		return false
	}
	return w.SimTime-v.StoppedSinceSec >= stopDwellSec
}

// maybeMarkStopped sets v.StoppedSinceSec the first tick the vehicle is
// effectively at the stop line (slow AND within tolerance). Idempotent
// once set. Uses stuckSpeedThresh (0.1 m/s) so that the dwell timer only
// starts once V genuinely approaches zero — this ensures real mandatory-stop
// behavior rather than slow-roll-stop. The virtual stop leader from
// stopDistanceForYield keeps IDM decelerating until V crosses this threshold.
func (w *World) maybeMarkStopped(v *Vehicle, myDist float64) {
	if v.StoppedSinceSec != 0 {
		return
	}
	if v.V < stuckSpeedThresh && myDist < stopLineTolMeters {
		v.StoppedSinceSec = w.SimTime
	}
}

// effectiveGap returns the gap (in seconds of oncoming-ETA) that v
// will accept for a maneuver whose base critical gap is baseGap.
// Applies the per-driver GapFactor multiplier and shrinks linearly
// with WaitTime, floored at minAcceptedGap.
//
// A zero or negative GapFactor is treated as 1.0 — covers hand-built
// test vehicles (default zero value) and protects against any future
// construction path that fails to sample a valid factor.
func effectiveGap(v *Vehicle, baseGap float64) float64 {
	factor := v.GapFactor
	if factor <= 0 {
		factor = 1.0
	}
	g := baseGap*factor - impatienceDecayRate*v.WaitTime
	if g < minAcceptedGap {
		g = minAcceptedGap
	}
	return g
}

// yieldGapCheck does ETA-based gap-acceptance against every approach at x
// whose effective control is ControlNone (i.e., the priority approaches).
// Returns (myDist, true) when we must yield; (0, false) when the gap is
// clear.
func (w *World) yieldGapCheck(v *Vehicle, x *network.Intersection, myPos int,
	myDist float64, byEdge map[network.EdgeID][]int,
) (float64, bool) {
	for j := range x.Incoming {
		if j == myPos {
			continue
		}
		if effectiveControl(w, x, j) != network.ControlNone {
			continue
		}
		others := byEdge[x.Incoming[j]]
		if len(others) == 0 {
			continue
		}
		otherEdge := &w.Net.Edges[x.Incoming[j]]
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			ovV := ov.V
			if ovV < 0.5 {
				ovV = 0.5
			}
			if d/ovV < effectiveGap(v, gapThresholdSec) {
				return myDist, true
			}
		}
	}
	return 0, false
}

// allWayStopFIFO arbitrates an AllWayStop approach. After v has completed
// its mandatory-stop dwell, it scans every other approach for a lead
// vehicle that came to a complete stop earlier than v. If one exists, we
// yield. Otherwise we proceed. Ties (same StoppedSinceSec) are broken by
// lower Incoming index winning. Exception: if both v and the lead are
// making left turns from opposing approaches, both proceed simultaneously
// (mutual-left pass).
func (w *World) allWayStopFIFO(v *Vehicle, x *network.Intersection, myPos int,
	myDist float64, byEdge map[network.EdgeID][]int,
) (float64, bool) {
	// Check if v is making a left turn.
	vIsLeftTurn := false
	if v.RouteIdx+1 < len(v.Route) {
		nextEdge := v.Route[v.RouteIdx+1]
		if network.ClassifyTurn(w.Net, v.Edge, nextEdge) == network.TurnLeft {
			vIsLeftTurn = true
		}
	}

	for j := range x.Incoming {
		if j == myPos {
			continue
		}
		others := byEdge[x.Incoming[j]]
		if len(others) == 0 {
			continue
		}
		// Find the lead vehicle on approach j — the one closest to the
		// stop line of edge x.Incoming[j].
		otherEdge := &w.Net.Edges[x.Incoming[j]]
		leadIdx := -1
		leadDist := math.Inf(1)
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			if d < leadDist {
				leadDist = d
				leadIdx = oi
			}
		}
		if leadIdx < 0 {
			continue
		}
		lead := &w.Vehicles[leadIdx]
		if lead.StoppedSinceSec == 0 {
			continue // not yet stopped; hasn't earned a FIFO slot
		}

		// Check if the opposing lead is also making a left turn.
		oppIsLeftTurn := false
		if lead.RouteIdx+1 < len(lead.Route) {
			nextEdge := lead.Route[lead.RouteIdx+1]
			if network.ClassifyTurn(w.Net, lead.Edge, nextEdge) == network.TurnLeft {
				oppIsLeftTurn = true
			}
		}

		// Mutual-left pass: if both are turning left AND j is the
		// opposing approach (not cross-traffic), both proceed
		// simultaneously without yielding to each other.
		if vIsLeftTurn && oppIsLeftTurn && myPos < len(x.Opposing) && int(x.Opposing[myPos]) == j {
			continue
		}

		if lead.StoppedSinceSec < v.StoppedSinceSec {
			return myDist, true // they stopped first; we yield
		}
		if lead.StoppedSinceSec == v.StoppedSinceSec && j < myPos {
			return myDist, true // tie-break: lower Incoming index wins
		}
	}
	return 0, false
}

// leftTurnYieldsToOpposing returns (distance to stop line, true) when v
// is making a left turn and an opposing-approach vehicle has imminent
// ETA. Layered on top of Phase 1's yield rules — only engages when v
// would otherwise proceed (priority road, green signal, or AllWayStop
// FIFO winner). Two opposing left-turners pass simultaneously: the
// inner gap loop skips opposing vehicles that are also turning left.
func (w *World) leftTurnYieldsToOpposing(v *Vehicle, byEdge map[network.EdgeID][]int) (float64, bool) {
	if v.RouteIdx+1 >= len(v.Route) {
		return 0, false
	}
	edge := &w.Net.Edges[v.Edge]
	x, ok := w.xByNodeID[edge.To]
	if !ok {
		return 0, false
	}
	myPos := IncomingPos(x, v.Edge)
	if myPos < 0 || myPos >= len(x.Opposing) {
		return 0, false
	}
	oppPos := int(x.Opposing[myPos])
	if oppPos < 0 {
		return 0, false
	}
	nextEdge := v.Route[v.RouteIdx+1]
	if network.ClassifyTurn(w.Net, v.Edge, nextEdge) != network.TurnLeft {
		return 0, false
	}
	if !w.entitledToProceed(v, byEdge) {
		return 0, false
	}

	myDist := edge.Length - v.S
	if myDist < 0 {
		myDist = 0
	}

	oppEdgeID := x.Incoming[oppPos]
	oppVehicles := byEdge[oppEdgeID]
	if len(oppVehicles) == 0 {
		return 0, false
	}
	oppEdge := &w.Net.Edges[oppEdgeID]
	for _, oi := range oppVehicles {
		ov := &w.Vehicles[oi]
		// Skip opposing left-turners — they're yielding to us, so we
		// don't yield to them (mutual-yield deadlock resolution).
		if ov.RouteIdx+1 < len(ov.Route) &&
			network.ClassifyTurn(w.Net, ov.Edge, ov.Route[ov.RouteIdx+1]) == network.TurnLeft {
			continue
		}
		d := oppEdge.Length - ov.S
		ovV := ov.V
		if ovV < 0.5 {
			ovV = 0.5
		}
		if d/ovV < effectiveGap(v, leftTurnGapSec) {
			return myDist, true
		}
	}
	return 0, false
}

// entitledToProceed reports whether v would otherwise proceed through
// the intersection at the end of its current edge — i.e., neither
// stopDistanceForRed nor stopDistanceForYield say to stop. Used by
// leftTurnYieldsToOpposing to layer the left-turn check on top of
// Phase 1's yield rules without double-stopping.
//
// stopDistanceForYield has an idempotent side effect (maybeMarkStopped);
// calling it twice per tick is safe — the second call is a no-op.
func (w *World) entitledToProceed(v *Vehicle, byEdge map[network.EdgeID][]int) bool {
	if _, isRed := w.stopDistanceForRed(v); isRed {
		return false
	}
	if _, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
		return false
	}
	return true
}

// Step advances the sim by one tick (DefaultDt seconds).
func (w *World) Step() {
	// 0a. Drain any pending UI control events. Non-blocking; if the
	// channel is empty we move on immediately. Bounded loop guards
	// against an attacker flooding the channel.
	if w.Control != nil {
		for i := 0; i < 64; i++ {
			select {
			case ev := <-w.Control:
				w.applyControl(ev)
			default:
				i = 64
			}
		}
	}

	// 0b. Advance all signal phases.
	for _, s := range w.SignalStates {
		if s != nil {
			s.Advance(w.dt)
		}
	}
	// Emit SignalPhase and SignalModeChange events for any state whose
	// values changed. Iterate by index (deterministic slice order).
	if w.lastPhase == nil {
		w.lastPhase = make([]signalLast, len(w.SignalStates))
	}
	if w.lastMode == nil {
		w.lastMode = make([]SignalMode, len(w.SignalStates))
	}
	for i, s := range w.SignalStates {
		if s == nil {
			continue
		}
		curPhase := signalLast{idx: s.PhaseIdx, yellow: s.IsYellow}
		if w.lastPhase[i] != curPhase {
			w.lastPhase[i] = curPhase
			w.EmitTrace(w.Tick, w.SimTime, &trace.SignalPhase{
				IntersectionID: uint32(i),
				PhaseIdx:       uint8(s.PhaseIdx),
				IsYellow:       s.IsYellow,
			})
		}
		if w.lastMode[i] != s.Mode {
			w.lastMode[i] = s.Mode
			w.EmitTrace(w.Tick, w.SimTime, &trace.SignalModeChange{
				IntersectionID: uint32(i),
				Mode:           uint8(s.Mode),
			})
		}
	}

	// 1. Demand.
	reqs := w.Spawner.Tick(w.SimTime, w.dt)
	for _, r := range reqs {
		w.trySpawn(r)
	}

	// 2. Bucket vehicles by edge and by (edge, lane) for leader lookup,
	//    sorted by S ascending within each bucket.
	byEdge := make(map[network.EdgeID][]int, 1024)
	byEdgeLane := make(map[network.EdgeID]map[uint8][]int, 1024)
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		eid := w.Vehicles[i].Edge
		byEdge[eid] = append(byEdge[eid], i)
		if _, ok := byEdgeLane[eid]; !ok {
			byEdgeLane[eid] = make(map[uint8][]int)
		}
		byEdgeLane[eid][w.Vehicles[i].Lane] = append(byEdgeLane[eid][w.Vehicles[i].Lane], i)
	}
	for _, lanes := range byEdgeLane {
		for _, idxs := range lanes {
			sortVehicleIdxByS(w.Vehicles, idxs)
		}
	}

	// 2b. Refresh live per-edge congestion from this tick's positions/speeds.
	w.Cong.Update(w.Net, byEdge, w.Vehicles)

	// 3. Pre-compute leaders per vehicle index using per-lane buckets.
	//    This is done in a separate pass to avoid order-dependence during stepping.
	type leaderInfo struct {
		lS, lV float64
		has    bool
	}
	leaders := make(map[int]leaderInfo, len(w.Vehicles))
	for eid, lanes := range byEdgeLane {
		edge := &w.Net.Edges[eid]
		for ln, idxs := range lanes {
			for pos, vi := range idxs {
				info := leaderInfo{}
				if pos+1 < len(idxs) {
					ld := &w.Vehicles[idxs[pos+1]]
					info.lS, info.lV, info.has = ld.S, ld.V, true
				} else {
					v := &w.Vehicles[vi]
					if v.RouteIdx+1 < len(v.Route) {
						nextE := v.Route[v.RouteIdx+1]
						if nlanes, ok := byEdgeLane[nextE]; ok {
							// Look for the leader on the post-turn target lane,
							// not ego's current lane — right-turners snap to lane
							// 0 and left-turners to nLanes-1 next tick, so the
							// current-lane bucket is the wrong one to search.
							ne := &w.Net.Edges[nextE]
							nLanes := uint8(len(ne.Lanes))
							cat := network.ClassifyTurn(w.Net, v.Edge, nextE)
							nextLane := postTurnLane(uint8(ln), cat, nLanes)
							if nextLane < nLanes {
								if nidxs, ok2 := nlanes[nextLane]; ok2 && len(nidxs) > 0 {
									nv := &w.Vehicles[nidxs[0]]
									info.lS = edge.Length + nv.S
									info.lV = nv.V
									info.has = true
								}
							}
						}
					}
				}
				leaders[vi] = info
			}
		}
	}

	// 4. Step each vehicle, applying signal/yield virtual leaders.
	//    Iterate vehicles in stable index order to preserve determinism.
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		v := &w.Vehicles[i]
		info := leaders[i]
		lS, lV, has := info.lS, info.lV, info.has

		// Apply red-light virtual leader if closer.
		dRed, isRed := w.stopDistanceForRed(v)
		if isRed {
			virtualS := v.S + dRed
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		// Apply unsignalized-yield virtual leader if closer.
		dYield, mustYield := w.stopDistanceForYield(v, byEdge)
		if mustYield {
			virtualS := v.S + dYield
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		// Apply left-turn opposing-traffic virtual leader if closer.
		dLT, mustYieldLT := w.leftTurnYieldsToOpposing(v, byEdge)
		if mustYieldLT {
			virtualS := v.S + dLT
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}

		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// Accumulate WaitTime while the vehicle is effectively stopped
		// AND yielding via gap-acceptance. WaitTime is only reset on
		// edge transition (see stepIDM); within an approach edge it is
		// monotonic. This lets impatience commit the vehicle to a
		// crossing once the gap is accepted: WaitTime stays high after
		// the vehicle starts moving, so effectiveGap stays below ETA
		// and mustYield stays false. Edge-transition reset gives each
		// new approach a fresh WaitTime=0. Does NOT apply to red lights.
		if v.V < stuckSpeedThresh && (mustYield || mustYieldLT) {
			v.WaitTime += w.dt
		}

		// Stuck-vehicle guard. Defensive against sim bugs that would
		// otherwise leave a vehicle wedged forever. Runs only when the
		// vehicle is below the speed threshold; reuses the yield-check
		// results from above to avoid re-calling stopDistance helpers.
		if !v.Despawned && v.V < stuckSpeedThresh {
			if !isRed && !mustYield && !mustYieldLT {
				v.StuckTime += w.dt
				if v.StuckTime > stuckTimeoutSec {
					slog.Warn("stuck vehicle despawned",
						"vehicle_id", v.ID,
						"edge", v.Edge,
						"lane", v.Lane,
						"s", v.S,
						"v", v.V,
						"route_idx", v.RouteIdx,
						"route_len", len(v.Route),
						"tick", w.Tick,
						"sim_time", w.SimTime,
						"stuck_duration", v.StuckTime,
					)
					v.Despawned = true
				}
			} else {
				v.StuckTime = 0
			}
		} else {
			v.StuckTime = 0
		}

		if v.Despawned {
			w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleDespawn{VehicleID: uint32(v.ID)})
		}

		// Decrement lane-change cooldown.
		if v.LaneChangeCooldown > 0 {
			v.LaneChangeCooldown -= w.dt
			if v.LaneChangeCooldown < 0 {
				v.LaneChangeCooldown = 0
			}
		}
		// Try lane change after stepping (byEdgeLane is a tick-old snapshot —
		// consistent and avoids order-dependence).
		if lanes, ok := byEdgeLane[v.Edge]; ok {
			tryLaneChange(v, i, lanes, w.Vehicles, w.Net)
		}
	}

	// 5. Publish snapshot for renderer (before compact so live vehicles are included).
	w.publishSnapshot()

	// 6. Compact and advance time.
	w.compact()
	w.Tick++
	w.SimTime += w.dt
}

// sortVehicleIdxByS sorts idxs ascending by Vehicles[i].S (insertion sort;
// fine for small per-edge counts).
func sortVehicleIdxByS(vs []Vehicle, idxs []int) {
	for i := 1; i < len(idxs); i++ {
		for j := i; j > 0 && vs[idxs[j-1]].S > vs[idxs[j]].S; j-- {
			idxs[j-1], idxs[j] = idxs[j], idxs[j-1]
		}
	}
}

func (w *World) trySpawn(r SpawnRequest) {
	// Sample a per-driver speed preference: Normal(mean=1.0, σ=0.015),
	// clamped to [0.95, 1.05]. The clamp basically never fires (≈3σ each
	// side covers 99.7%), so the distribution is effectively a tight
	// normal — most vehicles drive at the speed limit, a few noticeably
	// slower or faster.
	factor := 1.0 + w.rng.NormFloat64()*speedFactorStdDev
	if factor < speedFactorMin {
		factor = speedFactorMin
	} else if factor > speedFactorMax {
		factor = speedFactorMax
	}

	gapFactor := 1.0 + w.rng.NormFloat64()*gapFactorStdDev
	if gapFactor < gapFactorMin {
		gapFactor = gapFactorMin
	} else if gapFactor > gapFactorMax {
		gapFactor = gapFactorMax
	}

	// Decide GPS membership deterministically against the configured share.
	hasGPS := w.rng.Float64() < w.GpsShare

	// GPS vehicles route on live congestion cost; others on free-flow time.
	var route []network.EdgeID
	var err error
	if hasGPS {
		route, err = w.Router.RouteCost(r.OriginNode, r.DestNode, func(eid network.EdgeID) float64 {
			return w.Cong.Cost(w.Net, eid)
		})
	} else {
		route, err = w.Router.Route(r.OriginNode, r.DestNode)
	}
	if err != nil || len(route) == 0 {
		return
	}

	// Spawn at this driver's cruising speed so they don't immediately brake.
	v := Vehicle{
		ID:             w.nextID,
		Route:          route,
		Edge:           route[0],
		Lane:           0,
		S:              0,
		V:              w.Net.Edges[route[0]].SpeedLimit * factor,
		SpeedFactor:    factor,
		GapFactor:      gapFactor,
		HasGPS:         hasGPS,
		DestNode:       r.DestNode,
		LastRerouteSec: w.SimTime,
	}
	w.nextID++
	w.Vehicles = append(w.Vehicles, v)

	// Emit spawn event.
	route32 := make([]uint32, len(route))
	for i, eid := range route {
		route32[i] = uint32(eid)
	}
	w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleSpawn{
		VehicleID:  uint32(v.ID),
		OriginNode: uint32(r.OriginNode),
		DestNode:   uint32(r.DestNode),
		Route:      route32,
	})
}

// applyControl mutates sim state in response to a UI command. Validates
// the requested SignalMode against the known set; unknown values are
// logged and ignored rather than silently setting an undefined mode.
func (w *World) applyControl(ev ControlEvent) {
	id := int(ev.IntersectionID)
	if id < 0 || id >= len(w.SignalStates) {
		return
	}
	st := w.SignalStates[id]
	if st == nil {
		return
	}
	switch ev.Mode {
	case ModeNormal, ModeFlashA, ModeFlashB, ModeOff:
		// valid
	default:
		slog.Warn("applyControl: unknown SignalMode; ignoring",
			"intersection_id", id, "mode", uint8(ev.Mode))
		return
	}
	st.Mode = ev.Mode
	// Reset phase progression when switching back to normal so the cycle
	// restarts cleanly. Flash/off modes ignore phase progression entirely.
	if st.Mode == ModeNormal {
		st.PhaseIdx = 0
		st.Elapsed = 0
		st.IsYellow = false
	}
}

func (w *World) compact() {
	dst := 0
	for _, v := range w.Vehicles {
		if v.Despawned {
			continue
		}
		w.Vehicles[dst] = v
		dst++
	}
	w.Vehicles = w.Vehicles[:dst]
}

func (w *World) publishSnapshot() {
	if w.SnapshotBuf == nil {
		return
	}
	views := make([]snapshot.VehicleView, 0, len(w.Vehicles))
	for i := range w.Vehicles {
		v := &w.Vehicles[i]
		if v.Despawned {
			continue
		}
		x, y, hd := network.PositionOnEdge(w.Net, v.Edge, v.S)
		signal := turnSignalFor(w.Net, v)
		views = append(views, snapshot.VehicleView{
			ID: uint32(v.ID), EdgeID: uint32(v.Edge), Lane: v.Lane,
			X: x, Y: y, Heading: hd, Speed: v.V, Accel: v.A,
			TurnSignal: signal,
		})
	}
	// One SignalView per incoming approach, positioned a few meters back
	// from the stop line along that approach so each leg of the intersection
	// shows its own red/yellow/green state.
	sigs := make([]snapshot.SignalView, 0, len(w.SignalStates))
	for i, st := range w.SignalStates {
		if st == nil {
			continue
		}
		x := &w.Net.Intersections[i]
		for j, eid := range x.Incoming {
			green := st.GreenFor(j)
			e := &w.Net.Edges[eid]
			s := e.Length - SignalLightOffset
			if s < 0 {
				s = 0
			}
			px, py, _ := network.PositionOnEdge(w.Net, eid, s)
			sigs = append(sigs, snapshot.SignalView{
				IntersectionID: uint32(x.ID),
				X:              px, Y: py,
				IsRed:    !green,
				IsYellow: green && st.IsYellow,
				Mode:     uint8(st.Mode),
			})
		}
	}
	w.SnapshotBuf.Publish(snapshot.Snapshot{
		Tick: w.Tick, SimTime: w.SimTime,
		Vehicles: views, Signals: sigs, Bounds: w.Net.Bounds,
	})
}

// Run advances the sim for the given number of simulated seconds (headless).
// Logs basic progress every 1s of sim time.
func (w *World) Run(durationSec float64) {
	lastLog := w.SimTime
	target := w.SimTime + durationSec
	for w.SimTime < target {
		w.Step()
		if w.SimTime-lastLog >= 1.0 {
			slog.Info("sim progress",
				"sim_time", w.SimTime,
				"vehicles", len(w.Vehicles),
				"tick", w.Tick,
			)
			lastLog = w.SimTime
		}
	}
}
