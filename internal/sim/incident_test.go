package sim

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// incidentTestNet: one 2-lane edge, 200m, 10 m/s.
func incidentTestNet() *network.Network {
	return &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
				Lanes: []network.Lane{{Index: 0}, {Index: 1}}},
		},
	}
}

func TestSeverityConstantsMatchSnapshot(t *testing.T) {
	if uint8(SeverityNone) != snapshot.SevNone ||
		uint8(Slowdown) != snapshot.SevSlowdown ||
		uint8(LaneClose) != snapshot.SevLaneClose ||
		uint8(FullClose) != snapshot.SevFullClose {
		t.Fatal("sim.Severity values must match snapshot.Sev* constants")
	}
}

func TestEdgeCost_BySeverity(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	base := w.Cong.Cost(net, 0)

	if got := w.edgeCost(0); math.Abs(got-base) > 1e-9 {
		t.Fatalf("no incident: edgeCost=%v want base=%v", got, base)
	}
	w.Incidents[0] = Slowdown
	if got := w.edgeCost(0); math.Abs(got-base*incidentSlowdownCostMul) > 1e-9 {
		t.Fatalf("slowdown: edgeCost=%v want %v", got, base*incidentSlowdownCostMul)
	}
	w.Incidents[0] = LaneClose
	if got := w.edgeCost(0); math.Abs(got-base*incidentLaneCloseCostMul) > 1e-9 {
		t.Fatalf("laneclose: edgeCost=%v want %v", got, base*incidentLaneCloseCostMul)
	}
	w.Incidents[0] = FullClose
	if got := w.edgeCost(0); got != incidentFullCloseCost {
		t.Fatalf("fullclose: edgeCost=%v want %v", got, incidentFullCloseCost)
	}
}

func TestApplyIncident_SetClear(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	var events []*trace.IncidentSet
	w.EmitTrace = func(_ uint64, _ float64, e trace.Event) {
		if is, ok := e.(*trace.IncidentSet); ok {
			events = append(events, is)
		}
	}

	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: FullClose})
	if w.Incidents[0] != FullClose {
		t.Fatalf("after set: severity=%d want FullClose", w.Incidents[0])
	}
	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: SeverityNone})
	if _, present := w.Incidents[0]; present {
		t.Fatal("after clear: edge should be absent from the map")
	}
	// Out-of-range edge id is ignored and emits nothing.
	w.applyIncident(IncidentEvent{EdgeID: 9999, Severity: FullClose})

	if len(events) != 2 {
		t.Fatalf("emitted %d IncidentSet events, want 2 (set + clear)", len(events))
	}

	// Unknown severity is rejected: not stored, not emitted.
	before := len(events)
	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: Severity(99)})
	if _, present := w.Incidents[0]; present {
		t.Fatal("unknown severity should not be stored")
	}
	if len(events) != before {
		t.Fatal("unknown severity should not emit a trace event")
	}
}

func TestIncidentStopDistance_Blocks(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	v := &Vehicle{Edge: 0, Lane: 0, S: 50}

	if _, ok := w.incidentStopDistance(v); ok {
		t.Fatal("no incident should not block")
	}
	w.Incidents[0] = FullClose
	if d, ok := w.incidentStopDistance(v); !ok || math.Abs(d-150) > 1e-9 {
		t.Fatalf("FullClose got (%.2f,%v) want (150,true)", d, ok)
	}
	w.Incidents[0] = LaneClose // closes curb lane 0
	if d, ok := w.incidentStopDistance(v); !ok || math.Abs(d-150) > 1e-9 {
		t.Fatalf("LaneClose lane0 got (%.2f,%v) want (150,true)", d, ok)
	}
	v.Lane = 1
	if _, ok := w.incidentStopDistance(v); ok {
		t.Fatal("LaneClose should not block a vehicle in the open lane")
	}
	v.Lane = 0
	w.Incidents[0] = Slowdown
	if _, ok := w.incidentStopDistance(v); ok {
		t.Fatal("Slowdown should not block")
	}
}

func TestComputeDesiredSpeed_SlowdownCap(t *testing.T) {
	net := incidentTestNet() // 10 m/s limit
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	v := &Vehicle{Edge: 0, Lane: 0, S: 0, Route: []network.EdgeID{0}}

	if got := w.computeDesiredSpeed(v); math.Abs(got-10) > 1e-9 {
		t.Fatalf("no incident: v0=%v want 10", got)
	}
	w.Incidents[0] = Slowdown
	want := 10.0 * incidentSlowdownFactor
	if got := w.computeDesiredSpeed(v); math.Abs(got-want) > 1e-9 {
		t.Fatalf("slowdown: v0=%v want %v", got, want)
	}
}

func TestWorld_FullClose_GPSReroutes(t *testing.T) {
	net := buildRerouteGraph() // route [0,1] direct; detour [0,2,3]; dest node 3
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[1] = FullClose // close the direct edge

	v := &Vehicle{
		ID: 1, Route: []network.EdgeID{0, 1}, RouteIdx: 0, Edge: 0, S: 50, V: 5,
		HasGPS: true, DestNode: 3, LastRerouteSec: -1000,
	}
	if !w.maybeReroute(v) {
		t.Fatalf("maybeReroute returned false for an eligible GPS vehicle")
	}
	if len(v.Route) != 3 || v.Route[1] != 2 || v.Route[2] != 3 {
		t.Fatalf("route after reroute = %v, want [0 2 3] around the closure", v.Route)
	}
}

func TestPublishSnapshot_IncludesIncidents(t *testing.T) {
	net := incidentTestNet()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = Slowdown

	w.publishSnapshot()
	snap := w.SnapshotBuf.Read()
	if len(snap.Incidents) != 1 ||
		snap.Incidents[0].EdgeID != 0 ||
		snap.Incidents[0].Severity != snapshot.SevSlowdown {
		t.Fatalf("snapshot incidents = %+v, want one Slowdown on edge 0", snap.Incidents)
	}
}

func TestWorld_LaneClose_CarsMergeOut(t *testing.T) {
	// 2-lane edge, 1000m. A car in the closed curb lane (0) must merge into
	// the open lane (1) under LaneClose and then proceed normally.
	net := &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 1000, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 1000, SpeedLimit: 15,
				Lanes: []network.Lane{{Index: 0}, {Index: 1}}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = LaneClose // closes curb lane 0
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 100, V: 12}}
	w.nextID = 2

	for i := 0; i < 30; i++ { // 1.5s — plenty for one lane change
		w.Step()
		if len(w.Vehicles) == 0 {
			break
		}
	}
	if len(w.Vehicles) == 0 {
		t.Fatal("car despawned unexpectedly during merge-out")
	}
	v := &w.Vehicles[0]
	if v.Lane == 0 {
		t.Fatalf("car should have merged out of the closed curb lane, still in lane %d", v.Lane)
	}
	if v.V < 1.0 {
		t.Fatalf("after merging to the open lane the car should keep moving, V=%.2f", v.V)
	}
}

func TestWorld_Incident_DrainOnClear(t *testing.T) {
	// Single 1-lane edge. A FullClose stops the car near the edge end; after
	// clearing the incident, the car resumes and completes (despawns).
	net := &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
				Lanes: []network.Lane{{Index: 0}}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = FullClose
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 20, V: 15}}
	w.nextID = 2

	// Phase 1: car approaches and stops at the closure (well under stuckTimeoutSec=60s).
	for i := 0; i < 400; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			t.Fatal("car despawned while the edge was fully closed")
		}
	}
	if v := &w.Vehicles[0]; v.V > 1.0 {
		t.Fatalf("car should be stopped at the closure before clearing, V=%.2f", v.V)
	}

	// Phase 2: clear the incident; the car must resume and complete its trip.
	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: SeverityNone})
	completed := false
	for i := 0; i < 200; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			completed = true
			break
		}
	}
	if !completed {
		t.Fatalf("after clearing the closure the car should resume and despawn; still present at S=%.2f V=%.2f",
			w.Vehicles[0].S, w.Vehicles[0].V)
	}
}

func TestWorld_FullClose_VehicleStopsBeforeEnd(t *testing.T) {
	// One 1-lane edge; a car well upstream must brake to a stop at the
	// FullClose obstacle (edge end) instead of running off the edge. Uses
	// ordinary IDM braking via the virtual leader (no v0 override), so allow
	// time for the approach + asymptotic stop, but stay under stuckTimeoutSec.
	net := &network.Network{
		Nodes: []network.Node{
			{ID: 0, Pos: network.Point{X: 0, Y: 0}},
			{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
				Lanes: []network.Lane{{Index: 0}}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = FullClose
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 20, V: 15}}
	w.nextID = 2

	// 600 ticks = 30s: ample for the car to cruise, brake, and settle to a
	// near-stop, yet well under stuckTimeoutSec (60s) so it isn't despawned.
	for i := 0; i < 600; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			t.Fatal("vehicle despawned; expected it blocked at the closure")
		}
	}
	v := &w.Vehicles[0]
	if v.S > 200.0 {
		t.Fatalf("vehicle ran past the closure: S=%.2f (edge len 200)", v.S)
	}
	if v.V > 1.0 {
		t.Fatalf("vehicle should be ~stopped at the closure: V=%.2f", v.V)
	}
}

func TestWorld_FullClose_BlocksEntryFromUpstream(t *testing.T) {
	// Path 0->1->2; edge 1 is fully closed. A non-GPS car on edge 0 (committed
	// to route [0,1]) must NOT drive through the closed edge 1 — it should stop
	// at the entrance (end of edge 0) and never transition onto edge 1.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 400, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15, Lanes: []network.Lane{{Index: 0}}},
		{ID: 1, From: 1, To: 2, Length: 200, SpeedLimit: 15, Lanes: []network.Lane{{Index: 0}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[1] = FullClose
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 20, V: 15}}
	w.nextID = 2

	for i := 0; i < 400; i++ { // 20s, well under stuckTimeoutSec (60s)
		w.Step()
		if len(w.Vehicles) == 0 {
			t.Fatal("car despawned; expected it queued at the closure entrance")
		}
		if w.Vehicles[0].Edge != 0 {
			t.Fatalf("car entered closed edge %d at tick %d (S=%.1f) — should be blocked at entry",
				w.Vehicles[0].Edge, i, w.Vehicles[0].S)
		}
	}
	if v := &w.Vehicles[0]; v.V > 1.0 {
		t.Fatalf("car should be stopped at the closure entrance, V=%.2f", v.V)
	}
}

func TestWorld_NextEdgeFullClosed(t *testing.T) {
	net := buildRerouteGraph()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	v := &Vehicle{Route: []network.EdgeID{0, 1}, RouteIdx: 0} // next edge is 1

	if w.nextEdgeFullClosed(v) {
		t.Fatal("no incident: next edge should not be reported closed")
	}
	for _, sev := range []Severity{Slowdown, LaneClose} {
		w.Incidents[1] = sev
		if w.nextEdgeFullClosed(v) {
			t.Fatalf("severity %d on next edge must not count as full close", sev)
		}
	}
	w.Incidents[1] = FullClose
	if !w.nextEdgeFullClosed(v) {
		t.Fatal("FullClose on the next edge should be detected")
	}
	// On the last edge there is no next edge.
	last := &Vehicle{Route: []network.EdgeID{0, 1}, RouteIdx: 1}
	if w.nextEdgeFullClosed(last) {
		t.Fatal("last edge: there is no next edge to be closed")
	}
}
