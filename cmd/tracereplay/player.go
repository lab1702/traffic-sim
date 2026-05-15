package main

import (
	"errors"
	"io"
	"log/slog"
	"math"
	"time"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// player advances trace events at real wall-clock speed (1x), reconstructing
// vehicle positions by simple kinematic extrapolation between events.
// Phase 8 keeps this simple: vehicles teleport at spawn and disappear at
// despawn; positions between are interpolated linearly along their route
// at the edge's speed limit. Phase 9 can extend this with state snapshots
// for faithful replay.
type player struct {
	net      *network.Network
	r        *trace.Reader
	buf      *snapshot.Buffer
	vehicles map[uint32]*replayVehicle
	signals  []snapshot.SignalView
}

type replayVehicle struct {
	route         []uint32
	routeIdx      int
	curEdge       network.EdgeID
	s             float64
	enteredEdgeAt float64
}

func newPlayer(net *network.Network, r *trace.Reader, buf *snapshot.Buffer) *player {
	sigs := make([]snapshot.SignalView, 0)
	for i := range net.Intersections {
		x := &net.Intersections[i]
		if !x.HasSignal {
			continue
		}
		node := net.Nodes[x.NodeID]
		sigs = append(sigs, snapshot.SignalView{
			IntersectionID: uint32(x.ID), X: node.Pos.X, Y: node.Pos.Y,
		})
	}
	return &player{
		net:      net,
		r:        r,
		buf:      buf,
		vehicles: make(map[uint32]*replayVehicle),
		signals:  sigs,
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
		for i := range p.signals {
			if p.signals[i].IntersectionID == e.IntersectionID {
				// Approximate: phase idx 0 = green for principal approach.
				// IsYellow always honored.
				p.signals[i].IsYellow = e.IsYellow
				p.signals[i].IsRed = !e.IsYellow && e.PhaseIdx%2 == 1
			}
		}
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
		x, y, hd := positionOnEdge(p.net, v.curEdge, v.s)
		views = append(views, snapshot.VehicleView{
			ID: id, X: x, Y: y, Heading: hd, Speed: edge.SpeedLimit,
		})
	}
	p.buf.Publish(snapshot.Snapshot{
		SimTime:  simTime,
		Vehicles: views,
		Signals:  p.signals,
		Bounds:   p.net.Bounds,
	})
}

func positionOnEdge(net *network.Network, eid network.EdgeID, s float64) (float64, float64, float64) {
	if int(eid) >= len(net.Edges) {
		return 0, 0, 0
	}
	e := &net.Edges[eid]
	g := e.Geometry
	if len(g) < 2 {
		return 0, 0, 0
	}
	remaining := s
	for i := 1; i < len(g); i++ {
		dx := g[i].X - g[i-1].X
		dy := g[i].Y - g[i-1].Y
		segLen := math.Sqrt(dx*dx + dy*dy)
		if remaining <= segLen || i == len(g)-1 {
			t := 0.0
			if segLen > 0 {
				t = remaining / segLen
			}
			if t > 1 {
				t = 1
			}
			return g[i-1].X + dx*t, g[i-1].Y + dy*t, math.Atan2(dy, dx)
		}
		remaining -= segLen
	}
	return g[len(g)-1].X, g[len(g)-1].Y, 0
}
