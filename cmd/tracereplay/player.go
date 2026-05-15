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

// signalLightOffset mirrors the constant in internal/sim/world.go: each
// per-approach signal indicator is drawn this many meters back from the
// stop line so individual approach colors are legible.
const signalLightOffset = 4.0

// player advances trace events at real wall-clock speed (1x), reconstructing
// vehicle positions by simple kinematic extrapolation between events.
// Phase 8 keeps this simple: vehicles teleport at spawn and disappear at
// despawn; positions between are interpolated linearly along their route
// at the edge's speed limit. Phase 9 can extend this with state snapshots
// for faithful replay.
//
// Signals: one entry per (intersection, incoming approach). Colors are
// recomputed when a SignalPhase event fires for that intersection by
// looking up the approach's membership in the rebuilt default plan
// (using the same heading-based grouping the sim uses). YAML overrides
// will not be reflected accurately — the trace doesn't carry the
// override schedule. The sim itself renders signals correctly.
type player struct {
	net          *network.Network
	r            *trace.Reader
	buf          *snapshot.Buffer
	vehicles     map[uint32]*replayVehicle
	signals      []approachLight
	signalStates map[uint32]*sim.SignalState
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

func newPlayer(net *network.Network, r *trace.Reader, buf *snapshot.Buffer) *player {
	var lights []approachLight
	states := make(map[uint32]*sim.SignalState)
	for i := range net.Intersections {
		x := &net.Intersections[i]
		if !x.HasSignal {
			continue
		}
		// Rebuild the default plan so we know which approaches are
		// green in each phase. Same logic the sim uses.
		states[uint32(x.ID)] = sim.NewSignalState(sim.DefaultSignalConfig(x.Incoming, net))
		for j, eid := range x.Incoming {
			e := &net.Edges[eid]
			s := e.Length - signalLightOffset
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
					IsRed: true, // default to red until first SignalPhase event
				},
			})
		}
	}
	return &player{
		net:          net,
		r:            r,
		buf:          buf,
		vehicles:     make(map[uint32]*replayVehicle),
		signals:      lights,
		signalStates: states,
	}
}

func (p *player) run() {
	start := time.Now()
	for {
		hdr, ev, err := p.r.Next()
		if err != nil {
			if !errors.Is(err, io.EOF) {
				slog.Error("tracereplay: read error", "err", err)
			}
			return
		}
		// Real-time pacing: sleep until simTime matches elapsed wall time.
		want := time.Duration(hdr.SimTime * float64(time.Second))
		elapsed := time.Since(start)
		if want > elapsed {
			time.Sleep(want - elapsed)
		}
		p.apply(hdr, ev)
		p.publish(hdr.SimTime)
	}
}

func (p *player) apply(hdr trace.Header, ev trace.Event) {
	switch e := ev.(type) {
	case *trace.VehicleSpawn:
		if len(e.Route) == 0 {
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
	case *trace.SignalPhase:
		st := p.signalStates[e.IntersectionID]
		if st == nil {
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
	}
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

func (p *player) publish(simTime float64) {
	views := make([]snapshot.VehicleView, 0, len(p.vehicles))
	for id, v := range p.vehicles {
		if int(v.curEdge) >= len(p.net.Edges) {
			continue
		}
		// Advance position along route at edge speed limit.
		edge := &p.net.Edges[v.curEdge]
		dt := simTime - v.enteredEdgeAt
		s := edge.SpeedLimit * dt
		for s >= edge.Length && v.routeIdx+1 < len(v.route) {
			s -= edge.Length
			v.routeIdx++
			v.curEdge = network.EdgeID(v.route[v.routeIdx])
			v.enteredEdgeAt += edge.Length / math.Max(edge.SpeedLimit, 0.001)
			edge = &p.net.Edges[v.curEdge]
		}
		v.s = s
		x, y, hd := network.PositionOnEdge(p.net, v.curEdge, v.s)
		views = append(views, snapshot.VehicleView{
			ID: id, EdgeID: uint32(v.curEdge), X: x, Y: y, Heading: hd, Speed: edge.SpeedLimit,
		})
	}
	sigViews := make([]snapshot.SignalView, len(p.signals))
	for i, s := range p.signals {
		sigViews[i] = s.view
	}
	p.buf.Publish(snapshot.Snapshot{
		SimTime:  simTime,
		Vehicles: views,
		Signals:  sigViews,
		Bounds:   p.net.Bounds,
	})
}
