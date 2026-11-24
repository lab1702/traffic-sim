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

// detectorEdge identifies which semi-actuated signal and phase an incoming
// edge feeds, so the per-tick detector scan can attribute a vehicle on that
// edge to a phase's call/occupancy in O(1).
type detectorEdge struct {
	sigIdx   int // index into World.SignalStates
	phaseIdx int // index into that signal's Config.Phases
}

// registerDetectorEdges records, for a semi-actuated signal, the incoming edge
// feeding each phase. DefaultSignalConfig partitions approaches by axis, so one
// edge feeds exactly one phase; the GreenEdges positions index x.Incoming.
// No-op for fixed-time signals (overrides, single-phase) — they are not
// actuated and need no detection.
func registerDetectorEdges(det map[network.EdgeID]detectorEdge, s *SignalState, x *network.Intersection, sigIdx int) {
	if s == nil || s.Config.Plan != PlanSemiActuated {
		return
	}
	for pi := range s.Config.Phases {
		for _, pos := range s.Config.Phases[pi].GreenEdges {
			if pos < 0 || pos >= len(x.Incoming) {
				continue
			}
			det[x.Incoming[pos]] = detectorEdge{sigIdx: sigIdx, phaseIdx: pi}
		}
	}
}

// ControlEvent is a runtime command from the UI to the sim. Today only
// signal-mode changes; extend with new fields/variants as needed.
type ControlEvent struct {
	IntersectionID network.IntersectionID
	Mode           SignalMode
}

// World owns mutable simulation state. Only the sim goroutine touches it.
type World struct {
	Net      *network.Network
	Router   *Router
	Spawner  Spawner
	Vehicles []Vehicle
	Tick     uint64
	SimTime  float64
	dt       float64

	nextID VehicleID

	// SignalStates is indexed by IntersectionID; nil entries mean no signal.
	SignalStates []*SignalState

	// detectorEdges maps an incoming EdgeID of a semi-actuated signal to the
	// signal-state index and phase that edge feeds. Built once in NewWorld so
	// the per-tick detector scan only touches edges that actuate a signal.
	detectorEdges map[network.EdgeID]detectorEdge

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

	// Incidents maps an edge to its active incident severity. Absent key means
	// no incident. Owned by the sim goroutine; read by publishSnapshot.
	Incidents map[network.EdgeID]Severity

	// IncidentControl delivers runtime incident commands from the UI. Step
	// drains it non-blocking at the top of each tick, like Control. Nil
	// disables.
	IncidentControl <-chan IncidentEvent

	// reverseEdge maps a directed edge to its opposite-direction twin (same node
	// pair, swapped) when one exists. Built once in NewWorld. Lets applyIncident
	// mark both directions of a two-way road, so a "closed" road is closed both
	// ways. One-way edges have no entry.
	reverseEdge map[network.EdgeID]network.EdgeID

	// GpsShare is the fraction of spawned vehicles given GPS rerouting, in
	// [0,1]. Defaults to 1.0 (every vehicle) in NewWorld; overridden from the
	// --gps-share flag.
	GpsShare float64

	// inEdges maps a node to the edges that end at it. Built once in
	// NewWorld; trySpawn uses it to check for traffic about to cross the
	// origin node onto the spawn edge.
	inEdges map[network.NodeID][]network.EdgeID

	// Route lookahead (lookahead.go). endX maps an edge to the intersection
	// at its downstream end (nil when none), built once in NewWorld. turns
	// caches each turn's category and corner speed by from-edge, filled on
	// first use. arrivals, arrTouched and leaderBuf are per-tick scratch
	// reused across ticks.
	endX       []*network.Intersection
	turns      [][]turnInfo
	arrivals   [][]arrival
	arrTouched []network.EdgeID
	leaderBuf  []leaderInfo
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
	detectors := make(map[network.EdgeID]detectorEdge)
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
			registerDetectorEdges(detectors, sigs[x.ID], x, int(x.ID))
		}
	}
	inEdges := make(map[network.NodeID][]network.EdgeID, len(net.Nodes))
	endX := make([]*network.Intersection, len(net.Edges))
	for i := range net.Edges {
		e := &net.Edges[i]
		inEdges[e.To] = append(inEdges[e.To], e.ID)
		endX[i] = xByNode[e.To]
	}
	return &World{
		Net:           net,
		Router:        NewRouter(net),
		Spawner:       spawner,
		dt:            DefaultDt,
		SignalStates:  sigs,
		detectorEdges: detectors,
		xByNodeID:     xByNode,
		SnapshotBuf:   snapshot.New(),
		EmitTrace:     func(uint64, float64, trace.Event) {},
		rng:           rand.New(rand.NewPCG(0xCAFE, 0xBEEF)),
		Cong:          NewCongestion(net, ewmaHalfLifeSec, DefaultDt),
		Incidents:     make(map[network.EdgeID]Severity),
		reverseEdge:   buildReverseEdges(net),
		GpsShare:      1.0,
		inEdges:       inEdges,
		endX:          endX,
		turns:         make([][]turnInfo, len(net.Edges)),
		arrivals:      make([][]arrival, len(net.Edges)),
	}
}

// Yellow decisions stored in Vehicle.YellowCall.
const (
	yellowUndecided int8 = 0
	yellowGo        int8 = 1
	yellowStop      int8 = -1
)

// redCommitGrace is how close (in seconds of travel at current speed) a car
// that chose to go on yellow must be to the line for that choice to carry it
// into the red. Keeps late, slowed-down "go" cars from running the red.
const redCommitGrace = 1.0

// decideYellow chooses stop or go for a vehicle at speed v (desired speed
// v0), dist metres from the stop line, when the light turns yellow with tRem
// seconds left:
//
//  1. Already (nearly) stopped: stay stopped.
//  2. Can stop braking no harder than the comfortable rate B: stop.
//  3. Reaches the line before red at its current speed: go.
//  4. Can stop braking no harder than SafeDecel (firm, not panic): stop.
//  5. Otherwise (the dilemma zone: can neither clear nor stop): go. Such a
//     car crosses within redCommitGrace of the red, like a real amber-runner.
//
// "Can stop" is judged by the braking IDM itself would apply against the
// stop line's virtual leader, not by v²/2a kinematics. IDM brakes much harder
// than kinematics suggests when a stopped obstacle appears at moderate range
// (at 13 m/s it demands the full MaxBraking with the line 30 m away), so a
// kinematic test sent cars into panic stops.
//
// The old rule committed whenever a comfortable stop was impossible, even
// when the car couldn't reach the line before red, and re-decided every
// tick. At 13-14 m/s with a 3 s yellow, cars 40-60 m out committed, then
// panic-braked or ran the red when it came.
func decideYellow(v, v0, dist, tRem float64) int8 {
	p := DefaultIDM()
	gap := dist - VehicleLength // the virtual leader at the line is a stopped car
	canStop := func(decel float64) bool {
		return gap >= p.S0 && IDMAcceleration(p, v, v0, gap, v) >= -decel
	}
	switch {
	case v < stuckSpeedThresh:
		return yellowStop
	case canStop(p.B):
		return yellowStop
	case dist <= v*tRem:
		return yellowGo
	case canStop(SafeDecel):
		return yellowStop
	default:
		return yellowGo
	}
}

const gapThresholdSec = 3.0

// leftTurnGapSec is the minimum oncoming-traffic ETA a left turner
// accepts before crossing. Larger than gapThresholdSec because the
// left-turn maneuver takes longer to execute. Literature: 6–8s.
const leftTurnGapSec = 6.0

// roundaboutGapSec is the minimum ETA of circulating traffic a vehicle
// accepts before entering a roundabout. Entry critical gaps run ~3-4 s;
// slightly above the straight-crossing gapThresholdSec because entering a
// moving ring is less forgiving. Applied via criticalGap, so WaitTime
// impatience and GapFactor scale it the same way as the other gap constants.
const roundaboutGapSec = 3.5

// roundaboutWeaveLookahead (K) is how many ring segments before its exit a
// vehicle begins migrating to the outer lane (lane 0). 1 means "start the
// weave-out on the last ring segment before exiting." Tunable by viewer
// observation.
const roundaboutWeaveLookahead = 1

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

const (
	// rerouteCooldownSec is the minimum sim-time between a vehicle's reroutes.
	// 20s gives GPS-like periodic re-evaluation without thrashing A* on every
	// short edge a vehicle crosses.
	rerouteCooldownSec = 20.0

	// switchMargin is the hysteresis threshold: a vehicle adopts a candidate
	// route only if its estimated cost is at least this fraction cheaper than
	// the current remaining route. 0.15 stops flapping between near-equal
	// routes as smoothed speeds wobble.
	switchMargin = 0.15

	// maxReroutesPerTick caps reroute attempts per tick as a defensive guard
	// on the 50ms tick budget under pathological spawn rates. The
	// edge-transition trigger plus cooldown keep the real count far lower.
	maxReroutesPerTick = 64
)

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
			return network.ControlNone // redAt owns this case
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

// scanDetectors computes this tick's per-phase detector inputs for every
// semi-actuated signal. The returned slices are indexed by SignalStates index;
// called[i]/occupied[i] are nil for nil or fixed-time signals and per-phase
// bool slices otherwise. called[i][p] is true when some approach feeding phase
// p has a vehicle within actCallDistance of the stop line; occupied[i][p] uses
// actPassageDistance. Both are pure functions of vehicle positions.
//
// Distance is measured along the vehicle's route, so a zone reaches back past
// a short approach edge onto the edges before it. That matters: the route
// lookahead stops a car ~7 m before a red line, which on an approach shorter
// than that is still on the previous edge (possibly short of another
// junction), and a zone confined to the approach edge would never see it.
// A vehicle calls every actuated approach within actCallDistance along its
// route, except one more than stopLineTolMeters past a junction it has not
// reached yet: a car still short of that junction is held, if at all, by
// something else.
//
// One pass over vehicles, each walking at most actCallDistance of its route —
// O(vehicles) with a small constant, dwarfed by the IDM and routing work later
// in the tick.
func (w *World) scanDetectors() (called, occupied [][]bool) {
	called = make([][]bool, len(w.SignalStates))
	occupied = make([][]bool, len(w.SignalStates))
	if len(w.detectorEdges) == 0 {
		return called, occupied
	}
	for i, s := range w.SignalStates {
		if s != nil && s.Config.Plan == PlanSemiActuated {
			called[i] = make([]bool, len(s.Config.Phases))
			occupied[i] = make([]bool, len(s.Config.Phases))
		}
	}
	for vi := range w.Vehicles {
		v := &w.Vehicles[vi]
		if v.Despawned {
			continue
		}
		d := -v.S        // to the start of edge e
		junction := -1.0 // distance to the last junction passed in the walk, if any
		for j, e := v.RouteIdx, v.Edge; ; {
			dEnd := d + w.Net.Edges[e].Length // to e's stop line
			if dEnd > actCallDistance {
				break
			}
			if de, ok := w.detectorEdges[e]; ok && (junction < 0 || dEnd-junction < stopLineTolMeters) {
				// Skip a detector whose signal is no longer semi-actuated
				// (its state was replaced after NewWorld).
				if c := called[de.sigIdx]; de.phaseIdx < len(c) {
					c[de.phaseIdx] = true
					if dEnd <= actPassageDistance {
						occupied[de.sigIdx][de.phaseIdx] = true
					}
				}
			}
			if w.intersectionAt(e) != nil {
				junction = dEnd
			}
			j++
			if j >= len(v.Route) {
				break
			}
			d, e = dEnd, v.Route[j]
		}
	}
	return called, occupied
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

	// Drain pending incident commands from the UI (same pattern as Control).
	if w.IncidentControl != nil {
		for i := 0; i < 64; i++ {
			select {
			case ev := <-w.IncidentControl:
				w.applyIncident(ev)
			default:
				i = 64
			}
		}
	}

	// 0b. Advance all signal phases. Semi-actuated signals first need this
	// tick's detector occupancy: for each actuated approach edge, whether a
	// vehicle sits within the call / passage zone of the stop line. Detection
	// is a pure function of current positions, preserving determinism.
	called, occupied := w.scanDetectors()
	for i, s := range w.SignalStates {
		if s == nil {
			continue
		}
		if s.Config.Plan == PlanSemiActuated {
			s.AdvanceActuated(w.dt, called[i], occupied[i])
		} else {
			s.Advance(w.dt)
		}
	}
	// Emit SignalPhase and SignalModeChange events for any state whose
	// values changed. Iterate by index (deterministic slice order).
	if w.lastPhase == nil {
		w.lastPhase = make([]signalLast, len(w.SignalStates))
		// Seed with an impossible phase so every signal's initial state is
		// emitted on the first tick: the zero value {0, false} equals a
		// fresh SignalState, so nothing would be written until the first
		// phase change (never, for single-phase or resting signals).
		for i := range w.lastPhase {
			w.lastPhase[i].idx = -1
		}
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

	// 1. Demand. Spawns are checked against current occupancy so a new
	// vehicle never appears on top of (or just in front of) another.
	reqs := w.Spawner.Tick(w.SimTime, w.dt)
	if len(reqs) > 0 {
		occ := w.buildSpawnOccupancy()
		for _, r := range reqs {
			w.trySpawn(r, occ)
		}
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

	// 3. Route lookahead (lookahead.go), from start-of-tick positions so
	//    stepping order cannot matter: each vehicle's leader along its route,
	//    and the arrivals at each junction approach that gap checks read.
	leaders := w.findLeaders(byEdgeLane)
	w.buildArrivals()

	// 4. Step each vehicle, applying the first stop line on its route
	//    (signal, yield, left turn, incident) as a stopped virtual leader.
	//    Iterate vehicles in stable index order to preserve determinism.
	rerouteBudget := maxReroutesPerTick
	for i := range w.Vehicles {
		if w.Vehicles[i].Despawned {
			continue
		}
		v := &w.Vehicles[i]
		info := leaders[i]
		lS, lV, has := info.lS, info.lV, info.has

		stop := w.nextStop(v, i)
		if stop.any() {
			if virtualS := v.S + stop.dist; !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		isRed, mustYield, mustYieldLT := stop.red, stop.yield, stop.leftTurn
		// held: the line stopping the vehicle ends its current edge (it is
		// in that line's queue, as before the lookahead) or the vehicle is
		// at the line. A line elsewhere in the horizon doesn't count: it
		// would keep a gridlocked car "waiting" for ever.
		held := (isRed || mustYield || mustYieldLT) &&
			(stop.j == v.RouteIdx || stop.dist < stopLineTolMeters)
		yieldLine := stop.j

		prevEdge := v.Edge
		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// GPS rerouting fires on edge entry (a decision point) and, additionally,
		// when a closure lies ahead within the lookahead horizon (so a committed
		// vehicle diverts around it rather than queuing at its entry block).
		// Bounded by the per-tick budget. maybeReroute self-gates on HasGPS and
		// cooldown.
		if !v.Despawned && rerouteBudget > 0 &&
			(v.Edge != prevEdge || (v.HasGPS && w.closureAhead(v))) {
			if w.maybeReroute(v) {
				rerouteBudget--
			}
		}

		// Accumulate WaitTime while the vehicle is effectively stopped
		// AND yielding via gap-acceptance. WaitTime is only reset when the
		// vehicle crosses the line (see stepIDM); until then it is
		// monotonic. This lets impatience commit the vehicle to a
		// crossing once the gap is accepted: WaitTime stays high after
		// the vehicle starts moving, so criticalGap stays below ETA
		// and mustYield stays false. Each new line starts from a fresh
		// WaitTime=0. Does NOT apply to red lights. The wait belongs to the
		// line being yielded at, and only time spent held by it counts (see
		// held), not time queued upstream of a short approach edge. A
		// vehicle that crossed the line this tick has nothing left to wait
		// for (setLineAt ignores lines behind it).
		if v.V < stuckSpeedThresh && held && !isRed {
			ls := v.lineAt(yieldLine)
			ls.wait += w.dt
			v.setLineAt(yieldLine, ls)
		}

		// Stuck-vehicle guard. Defensive against sim bugs that would
		// otherwise leave a vehicle wedged forever. Runs only when the
		// vehicle is below the speed threshold; reuses this tick's nextStop
		// result. Only a vehicle held by its own line (see held) is waiting
		// legitimately.
		if !v.Despawned && v.V < stuckSpeedThresh {
			if !held {
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
		// Try lane change after stepping. byEdgeLane starts as this tick's
		// snapshot; each accepted change is applied to it at once, so a later
		// vehicle (in index order — deterministic) sees the lane as taken.
		if lanes, ok := byEdgeLane[v.Edge]; ok {
			cl := int8(-1)
			if c, ok := w.closedLaneFor(v.Edge); ok {
				cl = int8(c)
			}
			before := v.Lane
			tryLaneChange(v, i, lanes, w.Vehicles, w.Net, cl)
			if v.Lane != before {
				moveLaneBucket(lanes, i, before, v.Lane, w.Vehicles)
			}
		}
	}

	// 5. Publish snapshot for renderer (before compact so live vehicles are included).
	w.publishSnapshot()

	// 6. Compact and advance time.
	w.compact()
	w.Tick++
	w.SimTime += w.dt
}

// mergeZone is how far before the end of an edge vehicles in lanes that merge
// into one lane of the next edge start following each other across lanes.
const mergeZone = 75.0 // m

// mergeLeader returns the nearest vehicle ahead of vehicle vi (in lane ln of
// edge eid) that sits in a DIFFERENT lane of the same edge but will snap into
// the same lane of the same next edge — so ego must slot in behind it. Merge
// order is by S, ties broken by vehicle index, so of two cars exactly level
// exactly one yields. Only vehicles within mergeZone of the edge end are
// considered; returns ok=false when there is no such vehicle.
func (w *World) mergeLeader(vi int, ln uint8, eid network.EdgeID, lanes map[uint8][]int) (float64, float64, bool) {
	if len(lanes) < 2 {
		return 0, 0, false
	}
	v := &w.Vehicles[vi]
	edge := &w.Net.Edges[eid]
	if v.RouteIdx+1 >= len(v.Route) || edge.Length-v.S > mergeZone {
		return 0, 0, false
	}
	nextE := v.Route[v.RouteIdx+1]
	nLanes := uint8(len(w.Net.Edges[nextE].Lanes))
	cat := w.turnAt(eid, nextE).cat
	target := postTurnLane(ln, cat, nLanes)
	bestS, bestV, bestI, found := 0.0, 0.0, 0, false
	for ol, idxs := range lanes {
		if ol == ln || postTurnLane(ol, cat, nLanes) != target {
			continue
		}
		for _, ci := range idxs {
			c := &w.Vehicles[ci]
			if c.S < v.S || (c.S == v.S && ci < vi) {
				continue // behind ego in merge order
			}
			if c.RouteIdx+1 >= len(c.Route) || c.Route[c.RouteIdx+1] != nextE {
				continue // not taking the same exit
			}
			// Ties on S go to the lower vehicle index, so the result does
			// not depend on map iteration order over lanes.
			if !found || c.S < bestS || (c.S == bestS && ci < bestI) {
				bestS, bestV, bestI, found = c.S, c.V, ci, true
			}
			break // idxs is sorted by S: the first eligible one is the nearest
		}
	}
	return bestS, bestV, found
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

// maybeReroute re-evaluates a GPS vehicle's remaining path against live
// congestion costs and switches to a cheaper route if one beats the current
// remaining route by switchMargin. Called when the vehicle crosses into a new
// edge.
//
// Returns false immediately — without consuming a budget slot or touching
// LastRerouteSec — when HasGPS is false, the vehicle is on its last edge, or
// the cooldown hasn't elapsed. Returns true (and stamps LastRerouteSec) once
// A* is run, regardless of whether a switch was made; the caller decrements
// the per-tick reroute budget on a true return.
//
// On switch it splices Route[:RouteIdx+1] + candidate (RouteIdx still points
// at the current edge, unchanged) and emits a VehicleReroute event so replay
// follows the new path.
func (w *World) maybeReroute(v *Vehicle) bool {
	if !v.HasGPS {
		return false
	}
	if v.RouteIdx+1 >= len(v.Route) {
		return false // on the last edge: nothing downstream to change
	}
	// Bypass the cooldown when a closure lies ahead: a committed vehicle should
	// divert around it promptly rather than waiting out the cooldown.
	if !w.closureAhead(v) && w.SimTime-v.LastRerouteSec < rerouteCooldownSec {
		return false
	}
	v.LastRerouteSec = w.SimTime

	costFn := func(eid network.EdgeID) float64 { return w.edgeCost(eid) }
	// Search from the end of the current edge, arriving via it: the vehicle
	// is already committed to v.Edge, so the next turn must obey that
	// intersection's bans and the no-U-turn rule like any other turn.
	src := w.Net.Edges[v.Edge].To
	candidate, err := w.Router.RouteCostFrom(v.Edge, src, v.DestNode, costFn)
	if err != nil || len(candidate) == 0 {
		return true // attempt made; keep current route
	}

	curCost := 0.0
	for _, eid := range v.Route[v.RouteIdx+1:] {
		curCost += costFn(eid)
	}
	newCost := 0.0
	for _, eid := range candidate {
		newCost += costFn(eid)
	}

	if newCost < curCost*(1-switchMargin) && !sameTail(v.Route[v.RouteIdx+1:], candidate) {
		idx := v.RouteIdx + 1
		// 3-index slice caps capacity so append allocates fresh, avoiding any
		// aliasing with the old tail.
		v.Route = append(v.Route[:idx:idx], candidate...)
		// Per-line state for a line the new tail no longer reaches no
		// longer applies.
		v.dropStaleLines()
		w.emitReroute(v, idx, candidate)
	}
	return true
}

// emitReroute writes a VehicleReroute trace event for a route-tail switch.
func (w *World) emitReroute(v *Vehicle, atIndex int, tail []network.EdgeID) {
	tail32 := make([]uint32, len(tail))
	for i, eid := range tail {
		tail32[i] = uint32(eid)
	}
	w.EmitTrace(w.Tick, w.SimTime, &trace.VehicleReroute{
		VehicleID: uint32(v.ID),
		AtIndex:   uint32(atIndex),
		NewTail:   tail32,
	})
}

// sameTail reports whether two edge sequences are identical.
func sameTail(a, b []network.EdgeID) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// edgeLane keys per-(edge, lane) lookups.
type edgeLane struct {
	edge network.EdgeID
	lane uint8
}

// spawnOccupancy is a per-tick index used to vet spawns: the rearmost
// vehicle on each (edge, lane), and the frontmost vehicle on each edge. Built
// once per tick that has spawn requests; trySpawn keeps it current as it
// adds vehicles, so two spawns on the same edge in one tick see each other.
type spawnOccupancy struct {
	rear  map[edgeLane]int
	front map[network.EdgeID]int
}

func (w *World) buildSpawnOccupancy() *spawnOccupancy {
	occ := &spawnOccupancy{
		rear:  make(map[edgeLane]int),
		front: make(map[network.EdgeID]int),
	}
	for i := range w.Vehicles {
		v := &w.Vehicles[i]
		if v.Despawned {
			continue
		}
		k := edgeLane{v.Edge, v.Lane}
		if j, ok := occ.rear[k]; !ok || v.S < w.Vehicles[j].S {
			occ.rear[k] = i
		}
		if j, ok := occ.front[v.Edge]; !ok || v.S > w.Vehicles[j].S {
			occ.front[v.Edge] = i
		}
	}
	return occ
}

// spawnEntry picks the lane and speed for a vehicle entering route[0] at S=0
// with cruising speed `cruise`, or ok=false when the entry is blocked. Lanes
// are tried from lane 0 (curb) outward. A lane is usable when the first
// vehicle ahead in it along the route (spawnLeader: on route[0], or beyond a
// short first edge) leaves at least S0 of bumper gap; the entry speed is
// capped so the newcomer can match that vehicle's speed by braking at the
// comfortable rate before closing to S0, and so it can stop for a hard stop
// line ahead (spawnStopLine). Finally, every vehicle about to drive onto the
// spawn edge must be able to follow the newcomer without braking harder than
// SafeDecel (spawnFollowersSafe).
func (w *World) spawnEntry(route []network.EdgeID, cruise float64, occ *spawnOccupancy) (lane uint8, speed float64, ok bool) {
	p := DefaultIDM()
	eid := route[0]
	nLanes := len(w.Net.Edges[eid].Lanes)
	if nLanes == 0 {
		nLanes = 1 // hand-built fixtures may omit lanes; vehicles use lane 0
	}
	h := lookaheadHorizon(cruise)
	found := false
	for ln := 0; ln < nLanes; ln++ {
		sp := cruise
		if gap, lv, has := w.spawnLeader(route, uint8(ln), occ, h); has {
			if gap < p.S0 {
				continue // no room at the entry of this lane
			}
			if vs := math.Sqrt(lv*lv + 2*p.B*(gap-p.S0)); vs < sp {
				sp = vs
			}
		}
		if d, has := w.spawnStopLine(route, uint8(ln), h); has {
			// The line acts as a stopped leader whose front is at the line.
			vs := 0.0
			if gap := d - VehicleLength; gap > p.S0 {
				vs = math.Sqrt(2 * p.B * (gap - p.S0))
			}
			if vs < sp {
				sp = vs
			}
		}
		lane, speed, found = uint8(ln), sp, true
		break
	}
	if !found || !w.spawnFollowersSafe(eid, speed, occ) {
		return 0, 0, false
	}
	return lane, speed, true
}

// spawnLeader returns the bumper gap to, and the speed of, the first vehicle
// ahead of a newcomer at S=0 of route[0] in `lane`, carried along the route
// with postTurnLane, within h metres.
func (w *World) spawnLeader(route []network.EdgeID, lane uint8, occ *spawnOccupancy, h float64) (gap, leaderV float64, ok bool) {
	d := 0.0 // newcomer's front bumper to the start of route[k]
	for k, eid := range route {
		if k > 0 {
			if d >= h {
				break
			}
			lane = w.laneAfter(lane, route[k-1], eid)
		}
		if ri, has := occ.rear[edgeLane{eid, lane}]; has {
			r := &w.Vehicles[ri]
			return d + r.S - VehicleLength, r.V, true
		}
		d += w.Net.Edges[eid].Length
	}
	return 0, 0, false
}

// spawnStopLine returns the distance from S=0 of route[0] to the first line
// within h metres where a newcomer in `lane` must stop whatever the traffic:
// a red or yellow signal, a stop or all-way-stop sign, or an incident
// closure. Gap-acceptance yields are left to the first tick's nextStop.
func (w *World) spawnStopLine(route []network.EdgeID, lane uint8, h float64) (float64, bool) {
	d := 0.0 // to the end of route[k]
	for k, eid := range route {
		if k > 0 {
			lane = w.laneAfter(lane, route[k-1], eid)
		}
		d += w.Net.Edges[eid].Length
		if k > 0 && d > h {
			break
		}
		if len(w.Incidents) > 0 {
			if w.edgeFullyBlocked(eid) || (k+1 < len(route) && w.edgeFullyBlocked(route[k+1])) {
				return d, true
			}
			if cl, ok := w.closedLaneFor(eid); ok && cl == lane {
				return d, true
			}
		}
		x := w.intersectionAt(eid)
		if x == nil {
			continue
		}
		pos := IncomingPos(x, eid)
		if pos < 0 {
			continue
		}
		if x.HasSignal {
			if st := w.SignalStates[x.ID]; st != nil && st.Mode == ModeNormal {
				if !st.GreenFor(pos) || st.IsYellow {
					return d, true
				}
				continue
			}
		}
		switch effectiveControl(w, x, pos) {
		case network.ControlStop, network.ControlAllWayStop:
			return d, true
		}
	}
	return 0, false
}

// spawnFollowersSafe reports whether every vehicle about to drive onto eid
// can follow a newcomer entering it at S=0 at `speed` without braking harder
// than SafeDecel. It searches upstream breadth-first from eid's start node,
// so every edge feeding the node directly is checked before anything behind
// it, and continues through empty edges (so a short in-edge doesn't hide the
// traffic behind it) up to the lookahead horizon of a car at eid's limit. On
// each occupied edge it checks the frontmost vehicle, if its route continues
// along that path onto eid. Each edge is visited once and the reverse twin
// of the edge just walked back along is skipped (that would be a U-turn
// onto the path). A search larger than maxSpawnUpstreamEdges refuses the
// spawn: an unchecked follower may be right behind.
func (w *World) spawnFollowersSafe(eid network.EdgeID, speed float64, occ *spawnOccupancy) bool {
	p := DefaultIDM()
	h := lookaheadHorizon(w.Net.Edges[eid].SpeedLimit * speedFactorMax)
	// path is the route a follower must take from its next edge onto eid;
	// dist is the distance from node to the newcomer's front bumper.
	type step struct {
		node network.NodeID
		path []network.EdgeID
		dist float64
	}
	queue := []step{{node: w.Net.Edges[eid].From, path: []network.EdgeID{eid}}}
	visited := map[network.EdgeID]bool{eid: true}
	budget := maxSpawnUpstreamEdges
	for qi := 0; qi < len(queue); qi++ {
		st := queue[qi]
		twin, hasTwin := w.reverseEdge[st.path[0]]
		for _, ie := range w.inEdges[st.node] {
			if visited[ie] || (hasTwin && ie == twin) {
				continue
			}
			visited[ie] = true
			if budget--; budget < 0 {
				return false
			}
			ieLen := w.Net.Edges[ie].Length
			if fi, has := occ.front[ie]; has {
				u := &w.Vehicles[fi]
				if !routeContinues(u, st.path) {
					continue // not heading onto the spawn edge this way
				}
				gap := (ieLen - u.S) + st.dist - VehicleLength // newcomer's rear bumper is VehicleLength behind its front
				if !safeToFollow(p, u.V, w.Net.Edges[ie].SpeedLimit, gap, speed) {
					return false
				}
				continue
			}
			if st.dist+ieLen < h {
				up := make([]network.EdgeID, 0, len(st.path)+1)
				up = append(append(up, ie), st.path...)
				queue = append(queue, step{node: w.Net.Edges[ie].From, path: up, dist: st.dist + ieLen})
			}
		}
	}
	return true
}

// routeContinues reports whether u's route, after its current edge, starts
// with path.
func routeContinues(u *Vehicle, path []network.EdgeID) bool {
	if u.RouteIdx+len(path) >= len(u.Route) {
		return false
	}
	for m, e := range path {
		if u.Route[u.RouteIdx+1+m] != e {
			return false
		}
	}
	return true
}

// trySpawn honours a spawn request when a route exists and the origin edge
// has room at S=0 (see spawnEntry); otherwise the request is dropped.
func (w *World) trySpawn(r SpawnRequest, occ *spawnOccupancy) {
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
			return w.edgeCost(eid)
		})
	} else {
		route, err = w.Router.Route(r.OriginNode, r.DestNode)
	}
	if err != nil || len(route) == 0 {
		return
	}

	// Enter at this driver's cruising speed so they don't immediately brake,
	// unless traffic already on the edge forces a slower (or no) entry.
	lane, speed, ok := w.spawnEntry(route, w.Net.Edges[route[0]].SpeedLimit*factor, occ)
	if !ok {
		return
	}
	v := Vehicle{
		ID:             w.nextID,
		Route:          route,
		Edge:           route[0],
		Lane:           lane,
		S:              0,
		V:              speed,
		SpeedFactor:    factor,
		GapFactor:      gapFactor,
		HasGPS:         hasGPS,
		DestNode:       r.DestNode,
		LastRerouteSec: w.SimTime,
	}
	w.nextID++
	w.Vehicles = append(w.Vehicles, v)
	// The newcomer is now the rearmost vehicle in its lane; record it so a
	// second spawn this tick queues behind it rather than on top of it.
	ni := len(w.Vehicles) - 1
	occ.rear[edgeLane{v.Edge, v.Lane}] = ni
	if _, has := occ.front[v.Edge]; !has {
		occ.front[v.Edge] = ni
	}

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
	incidents := make([]snapshot.IncidentView, 0, len(w.Incidents))
	for eid, sev := range w.Incidents {
		incidents = append(incidents, snapshot.IncidentView{
			EdgeID: uint32(eid), Severity: uint8(sev),
		})
	}
	w.SnapshotBuf.Publish(snapshot.Snapshot{
		Tick: w.Tick, SimTime: w.SimTime,
		Vehicles: views, Signals: sigs, Incidents: incidents, Bounds: w.Net.Bounds,
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
