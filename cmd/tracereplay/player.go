package main

import (
	"errors"
	"io"
	"log/slog"
	"math"
	"time"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/sim"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// frameInterval is how often run advances the replay clock and publishes a
// snapshot: once per frame at Ebitengine's default 60 ticks per second.
const frameInterval = time.Second / 60

// publisher receives the replay's snapshots; *snapshot.Buffer in the app.
type publisher interface {
	Publish(snapshot.Snapshot)
}

// player replays trace events on a wall-clock sim clock (wall time *
// speedMul), reconstructing vehicle positions by simple kinematics.
// Vehicles teleport at spawn and disappear at despawn; in between they move
// along their route at each edge's speed limit, advanced every frame
// whether or not an event arrived. When the trace ends the clock stops at
// the last recorded sim time and the final frame stays on screen.
//
// Signals: one entry per (intersection, incoming approach). Colors are
// recomputed when a SignalPhase event fires for that intersection by
// looking up the approach's membership in that signal's plan. The plan
// comes from the trace's SignalPlan events, which record the plans the run
// actually used (default or --signals override). Traces written before
// SignalPlan existed fall back to rebuilding the default plan, which is
// wrong for intersections that had an override.
type player struct {
	net          *network.Network
	r            *trace.Reader
	buf          publisher
	speedMul     float64
	vehicles     map[uint32]*replayVehicle
	signals      []approachLight
	signalStates map[uint32]*sim.SignalState
	incidents    map[uint32]uint8 // EdgeID -> snapshot.Sev*

	// Read-ahead: next/nextHdr hold the first event not yet applied when
	// hasNext is set. lastSimTime is the sim time of the latest event read.
	// Once the reader stops, ended is set and the clock stops at endTime.
	next        trace.Event
	nextHdr     trace.Header
	hasNext     bool
	lastSimTime float64
	sawSimEnd   bool
	ended       bool
	endTime     float64

	warned map[string]bool // warnOnce keys already logged
}

type replayVehicle struct {
	route         []uint32
	routeIdx      int
	curEdge       network.EdgeID
	s             float64
	enteredEdgeAt float64
}

// approachLight is one signal indicator for one incoming leg of one
// intersection. The view position is fixed at startup; colors mutate.
type approachLight struct {
	intersectionID uint32
	incomingPos    int
	view           snapshot.SignalView
}

func newPlayer(net *network.Network, r *trace.Reader, buf publisher, speedMul float64) *player {
	if !(speedMul > 0) || math.IsInf(speedMul, 1) {
		speedMul = 1.0
	}
	var lights []approachLight
	states := make(map[uint32]*sim.SignalState)
	for i := range net.Intersections {
		x := &net.Intersections[i]
		if !x.HasSignal {
			continue
		}
		// Start from the default plan, the same one the sim builds. A
		// SignalPlan event in the trace replaces it with the plan the run
		// actually used.
		states[uint32(x.ID)] = sim.NewSignalState(sim.DefaultSignalConfig(x.Incoming, net))
		for j, eid := range x.Incoming {
			e := &net.Edges[eid]
			s := e.Length - sim.SignalLightOffset
			if s < 0 {
				s = 0
			}
			px, py, _ := network.PositionOnEdge(net, eid, s)
			lights = append(lights, approachLight{
				intersectionID: uint32(x.ID),
				incomingPos:    j,
				view: snapshot.SignalView{
					IntersectionID: uint32(x.ID),
					X:              px, Y: py,
				},
			})
		}
	}
	p := &player{
		net:          net,
		r:            r,
		buf:          buf,
		speedMul:     speedMul,
		vehicles:     make(map[uint32]*replayVehicle),
		incidents:    make(map[uint32]uint8),
		signals:      lights,
		signalStates: states,
		warned:       make(map[string]bool),
	}
	// Color every light from its signal's starting state (phase 0, not
	// yellow) rather than leaving it red until the first SignalPhase event:
	// traces from before the sim recorded initial phases have none for
	// signals that never changed phase.
	for id := range states {
		p.refreshSignalColors(id)
	}
	return p
}

// run drives the replay in real time until the trace ends.
func (p *player) run() {
	start := time.Now()
	ticker := time.NewTicker(frameInterval)
	defer ticker.Stop()
	for {
		// Wall-clock pacing scaled by speedMul. At speed=2 a 60-second
		// trace plays back in 30 seconds.
		if p.step(time.Since(start).Seconds() * p.speedMul) {
			return // the final frame stays published
		}
		<-ticker.C
	}
}

// step applies every event recorded at or before simNow, advances vehicles
// to simNow, and publishes one snapshot. Once the trace has ended and the
// clock has reached its last recorded time, step publishes that final
// frame and reports true.
func (p *player) step(simNow float64) bool {
	for !p.ended {
		if !p.hasNext {
			hdr, ev, err := p.r.Next()
			if err != nil {
				p.endStream(err)
				break
			}
			p.nextHdr, p.next, p.hasNext = hdr, ev, true
			p.lastSimTime = math.Max(p.lastSimTime, hdr.SimTime)
		}
		if p.nextHdr.SimTime > simNow {
			break
		}
		p.apply(p.nextHdr, p.next)
		p.hasNext = false
	}
	if p.ended && simNow >= p.endTime {
		p.publish(p.endTime)
		return true
	}
	p.publish(simNow)
	return false
}

// endStream records the end of the event stream and says why it ended.
func (p *player) endStream(err error) {
	p.ended = true
	p.endTime = p.lastSimTime
	switch {
	case errors.Is(err, io.EOF):
		if !p.sawSimEnd {
			slog.Warn("tracereplay: trace ended without a SimEnd event (recording interrupted?); replay stops at the last recorded event",
				"last_sim_time", p.lastSimTime)
		}
	case errors.Is(err, io.ErrUnexpectedEOF):
		slog.Warn("tracereplay: trace file is truncated or corrupt (process killed mid-write?); replay ended early",
			"last_sim_time", p.lastSimTime, "err", err)
	default:
		slog.Error("tracereplay: read error", "err", err, "last_sim_time", p.lastSimTime)
	}
}

// warnOnce logs msg the first time key is seen. Used for per-event problems
// (a trace that doesn't match the loaded network) that would otherwise log
// once per event.
func (p *player) warnOnce(key, msg string, args ...any) {
	if p.warned[key] {
		return
	}
	p.warned[key] = true
	slog.Warn(msg, args...)
}

// edgesInNetwork reports whether every edge ID is in the loaded network.
func (p *player) edgesInNetwork(ids []uint32) bool {
	for _, id := range ids {
		if int(id) >= len(p.net.Edges) {
			return false
		}
	}
	return true
}

func (p *player) apply(hdr trace.Header, ev trace.Event) {
	switch e := ev.(type) {
	case *trace.SimStart:
		if mismatch, traceHash, loadedHash := netHashMismatch(p.net, e.NetHash); mismatch {
			slog.Warn("tracereplay: network fingerprint mismatch — replay positions may be wrong",
				"trace_nethash", traceHash, "loaded_nethash", loadedHash,
				"hint", "the OSM file passed to -osm should be the same one used by the original run")
		}
	case *trace.SimEnd:
		p.sawSimEnd = true
	case *trace.VehicleSpawn:
		if len(e.Route) == 0 {
			return
		}
		if !p.edgesInNetwork(e.Route) {
			p.warnOnce("route-edge", "tracereplay: trace routes use edges that are not in the loaded network; skipping those vehicles",
				"vehicle", e.VehicleID, "edges", len(p.net.Edges),
				"hint", "the OSM file passed to -osm should be the same one used by the original run")
			return
		}
		p.vehicles[e.VehicleID] = &replayVehicle{
			route:         e.Route,
			routeIdx:      0,
			curEdge:       network.EdgeID(e.Route[0]),
			s:             0,
			enteredEdgeAt: hdr.SimTime,
		}
	case *trace.VehicleDespawn:
		delete(p.vehicles, e.VehicleID)
	case *trace.IncidentSet:
		if e.Severity == 0 {
			delete(p.incidents, e.EdgeID)
		} else {
			p.incidents[e.EdgeID] = e.Severity
		}
	case *trace.VehicleReroute:
		rv := p.vehicles[e.VehicleID]
		if rv == nil {
			return
		}
		// Bring the vehicle up to the reroute's time so the guard below
		// compares against where it is now, not where the last frame put it.
		p.advance(rv, hdr.SimTime)
		// Replace route[AtIndex:] with NewTail, preserving the prefix the
		// replay vehicle has already traversed. Skip when AtIndex is past
		// the route end, or when the replay vehicle has already reached
		// AtIndex: replay uses speed-limit kinematics and can outrun the
		// jam-slowed sim, so a late reroute may point at or behind curEdge,
		// and splicing there would replace the edge it is on and desync
		// curEdge from route[routeIdx]. AtIndex == len(route) is a pure
		// extension and is allowed.
		if int(e.AtIndex) > len(rv.route) || int(e.AtIndex) <= rv.routeIdx {
			return
		}
		if !p.edgesInNetwork(e.NewTail) {
			p.warnOnce("route-edge", "tracereplay: trace routes use edges that are not in the loaded network; skipping those reroutes",
				"vehicle", e.VehicleID, "edges", len(p.net.Edges),
				"hint", "the OSM file passed to -osm should be the same one used by the original run")
			return
		}
		tail := make([]uint32, len(e.NewTail))
		copy(tail, e.NewTail)
		rv.route = append(rv.route[:e.AtIndex:e.AtIndex], tail...)
	case *trace.SignalPlan:
		st := p.signalStates[e.IntersectionID]
		if st == nil {
			return
		}
		phases := make([]sim.SignalPhase, len(e.Phases))
		for i, green := range e.Phases {
			phases[i].GreenEdges = make([]int, len(green))
			for j, pos := range green {
				phases[i].GreenEdges[j] = int(pos)
			}
		}
		st.Config.Phases = phases
		if st.PhaseIdx >= len(phases) {
			st.PhaseIdx = 0
		}
		p.refreshSignalColors(e.IntersectionID)
	case *trace.SignalPhase:
		st := p.signalStates[e.IntersectionID]
		if st == nil {
			return
		}
		if n := len(st.Config.Phases); n > 0 && int(e.PhaseIdx) >= n {
			p.warnOnce("phase-idx", "tracereplay: signal phase is not in the replay's plan for that signal; ignoring those phase changes",
				"intersection", e.IntersectionID, "phase", e.PhaseIdx, "plan_phases", n,
				"hint", "the trace carries no SignalPlan for it (older trafficsim) or doesn't match the -osm network")
			return
		}
		st.PhaseIdx = int(e.PhaseIdx)
		st.IsYellow = e.IsYellow
		p.refreshSignalColors(e.IntersectionID)
	case *trace.SignalModeChange:
		st := p.signalStates[e.IntersectionID]
		if st == nil {
			return
		}
		st.Mode = sim.SignalMode(e.Mode)
		p.refreshSignalColors(e.IntersectionID)
	case *trace.TraceDropped:
		// Marker emitted by the writer when its backpressure channel
		// overflowed. The trace is missing events between the previous
		// surviving event and this point — replay positions, vehicle
		// counts, and signal states from here on may be wrong.
		slog.Warn("tracereplay: trace is incomplete — writer dropped events during recording",
			"dropped_count", e.Count, "at_sim_time", hdr.SimTime,
			"hint", "re-run with a faster disk or a smaller --spawn-rate to avoid backpressure")
	}
}

// netHashMismatch returns (true, traceHash, loadedHash) when the trace
// recorded a non-zero NetHash that differs from the loaded network's
// hash. Zero traceHash means the writer didn't compute a fingerprint
// (older trafficsim), so the check is skipped — returns (false, ...).
func netHashMismatch(net *network.Network, traceHash uint64) (bool, uint64, uint64) {
	if traceHash == 0 {
		return false, traceHash, 0
	}
	loaded := network.Hash(net)
	return loaded != traceHash, traceHash, loaded
}

func (p *player) refreshSignalColors(intersectionID uint32) {
	st := p.signalStates[intersectionID]
	if st == nil {
		return
	}
	for i := range p.signals {
		if p.signals[i].intersectionID != intersectionID {
			continue
		}
		isGreen := st.GreenFor(p.signals[i].incomingPos)
		p.signals[i].view.IsYellow = isGreen && st.IsYellow
		p.signals[i].view.IsRed = !isGreen
		p.signals[i].view.Mode = uint8(st.Mode)
	}
}

// advance moves v along its route at each edge's speed limit up to
// simTime. A vehicle that reaches the end of its last edge waits there for
// its despawn event.
func (p *player) advance(v *replayVehicle, simTime float64) {
	edge := &p.net.Edges[v.curEdge]
	for {
		s := edge.SpeedLimit * (simTime - v.enteredEdgeAt)
		if s < edge.Length || v.routeIdx+1 >= len(v.route) {
			v.s = math.Min(math.Max(s, 0), edge.Length)
			return
		}
		v.enteredEdgeAt += edge.Length / math.Max(edge.SpeedLimit, 0.001)
		v.routeIdx++
		v.curEdge = network.EdgeID(v.route[v.routeIdx])
		edge = &p.net.Edges[v.curEdge]
	}
}

func (p *player) publish(simTime float64) {
	views := make([]snapshot.VehicleView, 0, len(p.vehicles))
	for id, v := range p.vehicles {
		if int(v.curEdge) >= len(p.net.Edges) {
			continue
		}
		p.advance(v, simTime)
		edge := &p.net.Edges[v.curEdge]
		x, y, hd := network.PositionOnEdge(p.net, v.curEdge, v.s)
		views = append(views, snapshot.VehicleView{
			ID: id, EdgeID: uint32(v.curEdge), X: x, Y: y, Heading: hd,
			Speed: edge.SpeedLimit,
			// Accel left at 0: in replay the renderer's motion-state
			// coloring will treat every vehicle as steady-state. There's
			// no acceleration to extract from the trace today.
		})
	}
	sigViews := make([]snapshot.SignalView, len(p.signals))
	for i, s := range p.signals {
		sigViews[i] = s.view
	}
	incViews := make([]snapshot.IncidentView, 0, len(p.incidents))
	for eid, sev := range p.incidents {
		incViews = append(incViews, snapshot.IncidentView{EdgeID: eid, Severity: sev})
	}
	p.buf.Publish(snapshot.Snapshot{
		SimTime:   simTime,
		Vehicles:  views,
		Signals:   sigViews,
		Incidents: incViews,
		Bounds:    p.net.Bounds,
	})
}
