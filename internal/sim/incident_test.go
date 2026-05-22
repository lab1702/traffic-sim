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

func TestWorld_FullClose_VehicleStopsBeforeEnd(t *testing.T) {
	// One 1-lane edge; a car starts 80m from the end at speed and must stop
	// at the FullClose obstacle (edge end) instead of running off the edge.
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
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0}, Edge: 0, S: 120, V: 15}}
	w.nextID = 2

	// 100 ticks = 5s: enough to brake to a stop, less than stuckTimeoutSec so
	// the stuck-guard hasn't despawned the (legitimately blocked) car yet.
	for i := 0; i < 100; i++ {
		w.Step()
		if len(w.Vehicles) == 0 {
			t.Fatal("vehicle despawned too early; expected it blocked at the closure")
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
