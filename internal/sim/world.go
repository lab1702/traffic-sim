package sim

import (
	"log/slog"
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

	nextID   VehicleID
	maxRetry int // spawn retries per tick before giving up

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
}

const (
	DefaultDt = 0.05 // 50 ms == 20 Hz

	// signalLightOffset is how far back from the stop line each per-approach
	// signal indicator is drawn, in meters. Far enough to read distinct
	// colors at zoom, close enough to read as "this is that intersection's".
	signalLightOffset = 4.0

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
		maxRetry:     4,
		SignalStates: sigs,
		xByNodeID:    xByNode,
		SnapshotBuf:  snapshot.New(),
		EmitTrace:    func(uint64, float64, trace.Event) {},
		rng:          rand.New(rand.NewPCG(0xCAFE, 0xBEEF)),
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

const (
	// stuckSpeedThresh is the speed (m/s) below which a vehicle is
	// considered "not moving" for the purposes of the stuck-despawn guard.
	stuckSpeedThresh = 0.1
	// stuckTimeoutSec is the accumulated sim-seconds of below-threshold
	// motion (with no legitimate red/yield reason) that triggers despawn.
	stuckTimeoutSec = 60.0
)

// stopDistanceForYield returns (distance to stop line, true) when the
// vehicle's current edge ends at an intersection where it must yield via
// gap-acceptance. Covers three cases:
//
//  1. Unsignalized intersection — yield to lower-indexed approaches
//     (Incoming[0] is the priority road).
//  2. Signal in flash mode (A or B) — blinking-red approaches must
//     yield to blinking-yellow approaches (which have priority).
//  3. Signal in off mode — every approach must yield to lower-indexed
//     approaches (approximates a 4-way stop; position 0 has priority).
//
// Returns (0, false) when no yield is required.
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

	// Determine which incoming positions have priority over `myPos`.
	var priority []int
	if x.HasSignal {
		st := w.SignalStates[x.ID]
		if st == nil {
			return 0, false
		}
		switch st.Mode {
		case ModeNormal:
			// Hard-stop handled by stopDistanceForRed; no yield path here.
			return 0, false
		case ModeFlashA, ModeFlashB:
			if !st.MustYield(myPos) {
				return 0, false // we're blinking yellow; we have priority
			}
			for j := range x.Incoming {
				if st.GreenFor(j) { // blinking-yellow approach has priority
					priority = append(priority, j)
				}
			}
		case ModeOff:
			// 4-way-stop approximation: lower-indexed approach has priority.
			for j := 0; j < myPos; j++ {
				priority = append(priority, j)
			}
		}
	} else {
		// Unsignalized: lower-indexed approach has priority.
		if myPos == 0 {
			return 0, false
		}
		for j := 0; j < myPos; j++ {
			priority = append(priority, j)
		}
	}

	if len(priority) == 0 {
		return 0, false
	}

	myDist := edge.Length - v.S
	for _, otherPos := range priority {
		otherEdgeID := x.Incoming[otherPos]
		others := byEdge[otherEdgeID]
		if len(others) == 0 {
			continue
		}
		otherEdge := &w.Net.Edges[otherEdgeID]
		bestETA := 1e9
		for _, oi := range others {
			ov := &w.Vehicles[oi]
			d := otherEdge.Length - ov.S
			ovV := ov.V
			if ovV < 0.5 {
				ovV = 0.5
			}
			eta := d / ovV
			if eta < bestETA {
				bestETA = eta
			}
		}
		if bestETA < gapThresholdSec {
			return myDist, true
		}
	}
	return myDist, false
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
							// Use first vehicle in the same lane index on the next edge if exists.
							if ne := &w.Net.Edges[nextE]; uint8(ln) < uint8(len(ne.Lanes)) {
								if nidxs, ok2 := nlanes[ln]; ok2 && len(nidxs) > 0 {
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
		if d, isRed := w.stopDistanceForRed(v); isRed {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}
		// Apply unsignalized-yield virtual leader if closer.
		if d, mustYield := w.stopDistanceForYield(v, byEdge); mustYield {
			virtualS := v.S + d
			if !has || virtualS < lS {
				lS, lV, has = virtualS, 0, true
			}
		}

		v0 := w.computeDesiredSpeed(v)
		stepIDM(v, v0, lS, lV, has, w.Net, DefaultIDM(), w.dt)

		// Stuck-vehicle guard. Defensive against sim bugs that would
		// otherwise leave a vehicle wedged forever. Runs only when the
		// vehicle is below the speed threshold; the two stopDistance
		// helpers are cheap but skipped for the common moving case.
		if !v.Despawned && v.V < stuckSpeedThresh {
			_, isRed := w.stopDistanceForRed(v)
			_, mustYield := w.stopDistanceForYield(v, byEdge)
			if !isRed && !mustYield {
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
	route, err := w.Router.Route(r.OriginNode, r.DestNode)
	if err != nil || len(route) == 0 {
		return
	}
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

	// Spawn at this driver's cruising speed (factor * edge limit) so they
	// don't immediately decelerate. IDM regulates from there.
	v := Vehicle{
		ID:          w.nextID,
		Route:       route,
		Edge:        route[0],
		Lane:        0,
		S:           0,
		V:           w.Net.Edges[route[0]].SpeedLimit * factor,
		SpeedFactor: factor,
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

// applyControl mutates sim state in response to a UI command.
func (w *World) applyControl(ev ControlEvent) {
	id := int(ev.IntersectionID)
	if id < 0 || id >= len(w.SignalStates) {
		return
	}
	st := w.SignalStates[id]
	if st == nil {
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
			s := e.Length - signalLightOffset
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
