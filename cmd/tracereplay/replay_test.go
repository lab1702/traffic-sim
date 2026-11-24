package main

import (
	"bytes"
	"math"
	"strings"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// countingPublisher records how often the player publishes and the latest
// snapshot.
type countingPublisher struct {
	n    int
	last snapshot.Snapshot
}

func (c *countingPublisher) Publish(s snapshot.Snapshot) { c.n++; c.last = s }

// crossNet is a signalled four-way junction (intersection 0, approaches
// 0..3) plus a straight two-edge road (edges 4, 5) for vehicles.
func crossNet() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 200}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: -200}},
		{ID: 3, Pos: network.Point{X: -200, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},
		{ID: 5, Pos: network.Point{X: 1000, Y: 0}},
	}
	var edges []network.Edge
	for i := 0; i < 4; i++ {
		edges = append(edges, network.Edge{
			ID: network.EdgeID(i), From: network.NodeID(i), To: 4, Length: 200, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[i].Pos, nodes[4].Pos},
		})
	}
	edges = append(edges,
		network.Edge{ID: 4, From: 4, To: 5, Length: 100, SpeedLimit: 10},
		network.Edge{ID: 5, From: 5, To: 4, Length: 100, SpeedLimit: 20},
	)
	return &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{{
			ID: 0, NodeID: 4, Incoming: []network.EdgeID{0, 1, 2, 3}, HasSignal: true,
		}},
	}
}

// lightColors renders each light as R, Y or G, in approach order.
func lightColors(p *player) string {
	var b strings.Builder
	for _, l := range p.signals {
		switch {
		case l.view.IsRed:
			b.WriteByte('R')
		case l.view.IsYellow:
			b.WriteByte('Y')
		default:
			b.WriteByte('G')
		}
	}
	return b.String()
}

// TestPlayer_InitialSignalColors: lights start in their signal's initial
// state. They used to start red and stay red until a SignalPhase event,
// which never came for signals that never changed phase.
func TestPlayer_InitialSignalColors(t *testing.T) {
	// A single-approach signal is a permanent green.
	net := &network.Network{
		Nodes: []network.Node{{ID: 0}, {ID: 1, Pos: network.Point{X: 100}}},
		Edges: []network.Edge{{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10}},
		Intersections: []network.Intersection{
			{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
		},
	}
	p := newPlayer(net, nil, snapshot.New(), 1.0)
	if got := lightColors(p); got != "G" {
		t.Errorf("single-approach signal lights = %s, want G", got)
	}

	// A multi-phase signal shows phase 0's approaches green at startup.
	p = newPlayer(crossNet(), nil, snapshot.New(), 1.0)
	if got := lightColors(p); !strings.Contains(got, "G") {
		t.Errorf("cross signal lights = %s, want phase 0's approaches green", got)
	}
}

// TestPlayer_UsesRecordedSignalPlan: lights follow the plan recorded in
// the trace, not the rebuilt default plan, so --signals overrides replay
// correctly.
func TestPlayer_UsesRecordedSignalPlan(t *testing.T) {
	p := newPlayer(crossNet(), nil, snapshot.New(), 1.0)
	hdr := trace.Header{}
	p.apply(hdr, &trace.SignalPlan{IntersectionID: 0, Phases: [][]uint16{{0, 1}, {2, 3}, {}}})
	if got := lightColors(p); got != "GGRR" {
		t.Errorf("lights after the plan arrives = %s, want GGRR (phase 0)", got)
	}
	p.apply(hdr, &trace.SignalPhase{IntersectionID: 0, PhaseIdx: 1, IsYellow: true})
	if got := lightColors(p); got != "RRYY" {
		t.Errorf("phase 1 yellow lights = %s, want RRYY", got)
	}
	// Phase 2 exists only in the recorded plan (the default has two).
	p.apply(hdr, &trace.SignalPhase{IntersectionID: 0, PhaseIdx: 2})
	if got := lightColors(p); got != "RRRR" {
		t.Errorf("phase 2 lights = %s, want RRRR", got)
	}
}

// TestPlayer_PhaseIdxPastPlanIsIgnored: a phase index the replay's plan
// doesn't have (an old trace of an overridden signal, or a different OSM)
// used to panic in GreenFor.
func TestPlayer_PhaseIdxPastPlanIsIgnored(t *testing.T) {
	p := newPlayer(crossNet(), nil, snapshot.New(), 1.0)
	before := lightColors(p)
	p.apply(trace.Header{}, &trace.SignalPhase{IntersectionID: 0, PhaseIdx: 7})
	if got := lightColors(p); got != before {
		t.Errorf("out-of-plan phase changed lights from %s to %s", before, got)
	}
}

// TestPlayer_RouteEdgeOutsideNetworkIsSkipped: a trace from a different
// OSM (only a warning at SimStart) can name edges the loaded network
// doesn't have; following one used to panic.
func TestPlayer_RouteEdgeOutsideNetworkIsSkipped(t *testing.T) {
	p := newPlayer(crossNet(), nil, snapshot.New(), 1.0)
	p.apply(trace.Header{}, &trace.VehicleSpawn{VehicleID: 1, Route: []uint32{4, 99}})
	p.apply(trace.Header{}, &trace.VehicleSpawn{VehicleID: 2, Route: []uint32{4, 5}})
	p.apply(trace.Header{}, &trace.VehicleReroute{VehicleID: 2, AtIndex: 1, NewTail: []uint32{99}})
	p.publish(30) // far enough to walk off the first edge
	if _, ok := p.vehicles[1]; ok {
		t.Errorf("vehicle with an out-of-network route edge was spawned")
	}
	if got := p.vehicles[2].route; len(got) != 2 || got[1] != 5 {
		t.Errorf("reroute onto an out-of-network edge was applied: route %v", got)
	}
}

// TestPlayer_RerouteAtCurrentIndexIgnored: a reroute whose AtIndex is the
// edge the replay vehicle is already on must not replace that edge. The
// guard let AtIndex == routeIdx through, desyncing curEdge from
// route[routeIdx] so the vehicle later jumped to an unconnected edge.
func TestPlayer_RerouteAtCurrentIndexIgnored(t *testing.T) {
	net := &network.Network{
		Nodes: []network.Node{{ID: 0}, {ID: 1}, {ID: 2}, {ID: 3}},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10},
			{ID: 1, From: 1, To: 3, Length: 100, SpeedLimit: 10},
			{ID: 2, From: 1, To: 2, Length: 100, SpeedLimit: 10},
			{ID: 3, From: 2, To: 3, Length: 100, SpeedLimit: 10},
		},
	}
	p := newPlayer(net, nil, snapshot.New(), 1.0)
	p.apply(trace.Header{}, &trace.VehicleSpawn{VehicleID: 7, Route: []uint32{0, 1}})
	p.publish(15) // 150 m at 10 m/s: on edge 1 (route index 1)
	p.apply(trace.Header{SimTime: 15}, &trace.VehicleReroute{VehicleID: 7, AtIndex: 1, NewTail: []uint32{2, 3}})
	rv := p.vehicles[7]
	if len(rv.route) != 2 || rv.route[1] != 1 {
		t.Errorf("reroute at the current index replaced the current edge: route %v", rv.route)
	}
	if rv.curEdge != network.EdgeID(rv.route[rv.routeIdx]) {
		t.Errorf("curEdge %d != route[%d] = %d", rv.curEdge, rv.routeIdx, rv.route[rv.routeIdx])
	}
}

type timedEvent struct {
	at float64
	e  trace.Event
}

// traceReader encodes events, each at its own sim time, into a reader.
func traceReader(t *testing.T, evs ...timedEvent) *trace.Reader {
	t.Helper()
	var buf bytes.Buffer
	w := trace.NewWriter(&buf)
	for i, ev := range evs {
		if err := w.Write(uint64(i), ev.at, ev.e); err != nil {
			t.Fatalf("write: %v", err)
		}
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	return trace.NewReader(&buf)
}

// TestPlayer_AdvancesOnClock: vehicles move on the replay clock, not only
// when an event arrives. Positions used to be recomputed only at events,
// so vehicles froze between sparse events.
func TestPlayer_AdvancesOnClock(t *testing.T) {
	pub := &countingPublisher{}
	r := traceReader(t,
		timedEvent{0, &trace.VehicleSpawn{VehicleID: 1, Route: []uint32{4, 5}}},
		timedEvent{20, &trace.SimEnd{Reason: "exit"}},
	)
	p := newPlayer(crossNet(), r, pub, 1.0)

	if p.step(0) || p.step(5) {
		t.Fatalf("step reported the end of the trace before SimEnd")
	}
	v := p.vehicles[1]
	if v == nil || v.curEdge != 4 || v.s != 50 {
		t.Fatalf("at t=5 vehicle = %+v, want edge 4 at s=50", v)
	}
	if pub.last.SimTime != 5 || len(pub.last.Vehicles) != 1 {
		t.Errorf("published snapshot at t=%v with %d vehicles, want t=5 with 1", pub.last.SimTime, len(pub.last.Vehicles))
	}
	// Edge 4 (100 m at 10 m/s) is done at t=10; edge 5 runs at 20 m/s.
	p.step(12)
	if v.curEdge != 5 || v.s != 40 {
		t.Errorf("at t=12 vehicle on edge %d at s=%v, want edge 5 at s=40", v.curEdge, v.s)
	}
	// Past the end of the trace the clock stops at SimEnd's time.
	if !p.step(25) {
		t.Fatalf("step(25) did not report the end of the trace")
	}
	if pub.last.SimTime != 20 {
		t.Errorf("final frame at t=%v, want the SimEnd time 20", pub.last.SimTime)
	}
}

// TestPlayer_PublishesOncePerStep: every event used to trigger a full
// snapshot rebuild; a busy trace spent most of its CPU republishing.
func TestPlayer_PublishesOncePerStep(t *testing.T) {
	var evs []timedEvent
	for i := 0; i < 500; i++ {
		evs = append(evs, timedEvent{float64(i) / 1000, &trace.VehicleSpawn{VehicleID: uint32(i), Route: []uint32{4, 5}}})
	}
	evs = append(evs, timedEvent{10, &trace.SimEnd{}})
	pub := &countingPublisher{}
	p := newPlayer(crossNet(), traceReader(t, evs...), pub, 1.0)
	p.step(1)
	if pub.n != 1 {
		t.Errorf("one step over 500 events published %d snapshots, want 1", pub.n)
	}
	if len(pub.last.Vehicles) != 500 {
		t.Errorf("snapshot has %d vehicles, want 500", len(pub.last.Vehicles))
	}
}

// TestPlayer_EndsWithoutSimEnd: a trace with no SimEnd (interrupted run)
// ends at its last event, and a truncated trace ends at its last good
// event rather than at sim time 0.
func TestPlayer_EndsWithoutSimEnd(t *testing.T) {
	var buf bytes.Buffer
	w := trace.NewWriter(&buf)
	for i, e := range []timedEvent{
		{0, &trace.VehicleSpawn{VehicleID: 1, Route: []uint32{4, 5}}},
		{3, &trace.VehicleDespawn{VehicleID: 9}},
		{4, &trace.VehicleDespawn{VehicleID: 8}},
		{5, &trace.VehicleDespawn{VehicleID: 7}}, // cut in half below
	} {
		if err := w.Write(uint64(i), e.at, e.e); err != nil {
			t.Fatalf("write: %v", err)
		}
	}
	full := buf.Bytes()
	const lastRecord = 8 + 8 + 1 + 2 + 4

	for _, c := range []struct {
		name    string
		data    []byte
		wantEnd float64
	}{
		{"no SimEnd", full, 5},
		{"truncated", full[:len(full)-lastRecord/2], 4},
	} {
		t.Run(c.name, func(t *testing.T) {
			pub := &countingPublisher{}
			p := newPlayer(crossNet(), trace.NewReader(bytes.NewReader(c.data)), pub, 1.0)
			if !p.step(100) {
				t.Fatalf("step past the end did not report the end of the trace")
			}
			if p.sawSimEnd {
				t.Errorf("sawSimEnd set for a trace without SimEnd")
			}
			if p.endTime != c.wantEnd || pub.last.SimTime != c.wantEnd {
				t.Errorf("ended at %v (final frame %v), want %v", p.endTime, pub.last.SimTime, c.wantEnd)
			}
		})
	}
}

// TestCheckSpeed: NaN used to pass the old "speed <= 0" check.
func TestCheckSpeed(t *testing.T) {
	for _, bad := range []float64{0, -1, math.NaN(), math.Inf(1), math.Inf(-1)} {
		if checkSpeed(bad) == nil {
			t.Errorf("checkSpeed(%v) = nil, want an error", bad)
		}
	}
	for _, good := range []float64{0.25, 1, 4} {
		if err := checkSpeed(good); err != nil {
			t.Errorf("checkSpeed(%v) = %v, want nil", good, err)
		}
	}
}
