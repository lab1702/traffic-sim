package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Regression tests for the third full code review.

// signedCrossing builds a crossing at node 4 with a W->E road (edges 0, 2)
// and an S->N road (edges 1, 3), every approach signed with ctrl: no
// approach is uncontrolled.
func signedCrossing(ctrl network.Control) *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 100, Y: 0}},
		{ID: 3, Pos: network.Point{X: 0, Y: 100}},
		{ID: 4, Pos: network.Point{X: 0, Y: 0}},
	}
	mk := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	return &network.Network{
		Nodes: nodes,
		Edges: []network.Edge{mk(0, 0, 4), mk(1, 1, 4), mk(2, 4, 2), mk(3, 4, 3)},
		Intersections: []network.Intersection{{
			ID: 0, NodeID: 4,
			Incoming:        []network.EdgeID{0, 1},
			IncomingControl: ctrls(ctrl, ctrl),
			Outgoing:        []network.EdgeID{2, 3},
		}},
	}
}

// crossingEntryTicks steps w until both crossing vehicles (IDs 1 and 2) have
// entered their outbound edge and returns the tick each one did.
func crossingEntryTicks(t *testing.T, w *World) (t1, t2 uint64) {
	t.Helper()
	entered := map[VehicleID]uint64{}
	for i := 0; i < 1200 && len(entered) < 2; i++ {
		w.Step()
		for j := range w.Vehicles {
			v := &w.Vehicles[j]
			if _, seen := entered[v.ID]; !seen && v.RouteIdx == 1 {
				entered[v.ID] = w.Tick
			}
		}
	}
	if len(entered) < 2 {
		t.Fatalf("only %d of 2 vehicles crossed the junction (deadlock?)", len(entered))
	}
	return entered[1], entered[2]
}

func absTickDiff(a, b uint64) float64 {
	if a > b {
		return float64(a-b) * DefaultDt
	}
	return float64(b-a) * DefaultDt
}

// TestJunction_AllStopTakesTurns: a stop sign on every approach (no priority
// road) used to let two cars that stopped together both pull out after the
// dwell, because gap acceptance only looked at uncontrolled approaches.
// They must take turns like an all-way stop.
func TestJunction_AllStopTakesTurns(t *testing.T) {
	net := signedCrossing(network.ControlStop)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 93, V: 0},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 93, V: 0},
	}
	w.nextID = 3
	t1, t2 := crossingEntryTicks(t, w)
	if d := absTickDiff(t1, t2); d < 1.0 {
		t.Fatalf("crossing cars entered %.2f s apart (ticks %d, %d); they went together", d, t1, t2)
	}
	if t1 > t2 {
		t.Errorf("tie should go to the lower Incoming index (vehicle 1), but vehicle 2 went first")
	}
}

// TestJunction_AllYieldTakesTurns: the same with a yield sign on every
// approach. Neither approach used to yield to the other; now the one later in
// Incoming (lower road priority) gives way.
func TestJunction_AllYieldTakesTurns(t *testing.T) {
	net := signedCrossing(network.ControlYield)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 40, V: 10},
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 40, V: 10},
	}
	w.nextID = 3
	t1, t2 := crossingEntryTicks(t, w)
	if d := absTickDiff(t1, t2); d < 1.0 {
		t.Fatalf("crossing cars entered %.2f s apart (ticks %d, %d); neither yielded", d, t1, t2)
	}
	if t1 > t2 {
		t.Errorf("approach 1 should yield to approach 0, but vehicle 2 went first")
	}
}

// TestRouter_FastMotorwayHeuristic: the A* heuristic assumed no edge was
// faster than 31.3 m/s. With a 130 km/h motorway that overestimates, and A*
// returned the slower direct road because it reached the destination first.
func TestRouter_FastMotorwayHeuristic(t *testing.T) {
	s := network.Point{X: 0, Y: 0}
	m := network.Point{X: 1000, Y: 0}
	d := network.Point{X: 2000, Y: 0}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: s}, {ID: 1, Pos: m}, {ID: 2, Pos: d}},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 1000, SpeedLimit: 36.1, Geometry: []network.Point{s, m}},
			{ID: 1, From: 1, To: 2, Length: 1000, SpeedLimit: 36.1, Geometry: []network.Point{m, d}},
			{ID: 2, From: 0, To: 2, Length: 2000, SpeedLimit: 34, Geometry: []network.Point{s, d}},
		},
	}
	route, err := NewRouter(net).Route(0, 2)
	if err != nil {
		t.Fatalf("Route: %v", err)
	}
	if len(route) != 2 || route[0] != 0 || route[1] != 1 {
		t.Fatalf("route = %v, want the faster motorway [0 1] (55.4 s) over the direct road [2] (58.8 s)", route)
	}
}

// actuatedT builds a semi-actuated two-phase signal whose major phase is 1:
// approach 1 (S->C) has the higher speed limit.
func actuatedT() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: 0}},
		{ID: 1, Pos: network.Point{X: 0, Y: -100}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 0, Y: 100}},
	}
	mk := func(id, from, to int, speed float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: speed, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	return &network.Network{
		Nodes: nodes,
		Edges: []network.Edge{mk(0, 0, 2, 10), mk(1, 1, 2, 20), mk(2, 2, 3, 20)},
		Intersections: []network.Intersection{{
			ID: 0, NodeID: 2, HasSignal: true,
			Incoming: []network.EdgeID{0, 1}, Outgoing: []network.EdgeID{2},
		}},
	}
}

// TestApplyControl_NormalIsNoOpAndResumesMajor: pressing N on a signal
// already in normal mode used to restart the cycle at phase 0, turning the
// green approach red with no yellow. Returning to normal from another mode
// must start an actuated signal on its rest phase, not phase 0.
func TestApplyControl_NormalIsNoOpAndResumesMajor(t *testing.T) {
	net := actuatedT()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	st := w.SignalStates[0]
	if st.Config.Plan != PlanSemiActuated || st.Config.MajorPhase != 1 {
		t.Fatalf("fixture: plan %v major %d, want semi-actuated with major phase 1", st.Config.Plan, st.Config.MajorPhase)
	}

	st.PhaseIdx, st.Elapsed, st.passageGap = 0, 5, 1.5 // minor phase, mid-green
	w.applyControl(ControlEvent{IntersectionID: 0, Mode: ModeNormal})
	if st.PhaseIdx != 0 || st.Elapsed != 5 || st.IsYellow || st.passageGap != 1.5 {
		t.Fatalf("normal -> normal changed the cycle: phase %d elapsed %.1f yellow %v gap %.1f",
			st.PhaseIdx, st.Elapsed, st.IsYellow, st.passageGap)
	}

	w.applyControl(ControlEvent{IntersectionID: 0, Mode: ModeFlashA})
	w.applyControl(ControlEvent{IntersectionID: 0, Mode: ModeNormal})
	if st.Mode != ModeNormal || st.PhaseIdx != 1 || st.Elapsed != 0 || st.IsYellow || st.passageGap != 0 {
		t.Fatalf("flash -> normal: mode %v phase %d elapsed %.1f yellow %v gap %.1f, want normal at the start of major phase 1",
			st.Mode, st.PhaseIdx, st.Elapsed, st.IsYellow, st.passageGap)
	}
}

// TestBuildReverseEdges_SeparateRoadsBetweenSameNodes: twins used to be
// matched by node pair alone, so closing one carriageway of a dual
// carriageway closed the other, and two parallel two-way streets between the
// same nodes got each other's directions.
func TestBuildReverseEdges_SeparateRoadsBetweenSameNodes(t *testing.T) {
	a := network.Point{X: 0, Y: 0}
	b := network.Point{X: 200, Y: 0}
	north := network.Point{X: 100, Y: 20}
	south := network.Point{X: 100, Y: -20}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: a}, {ID: 1, Pos: b}},
		Edges: []network.Edge{
			// Street via the north: both directions.
			{ID: 0, From: 0, To: 1, Length: 204, Geometry: []network.Point{a, north, b}},
			{ID: 1, From: 1, To: 0, Length: 204, Geometry: []network.Point{b, north, a}},
			// Street via the south: both directions.
			{ID: 2, From: 0, To: 1, Length: 204, Geometry: []network.Point{a, south, b}},
			{ID: 3, From: 1, To: 0, Length: 204, Geometry: []network.Point{b, south, a}},
		},
	}
	rev := buildReverseEdges(net)
	for e, want := range map[network.EdgeID]network.EdgeID{0: 1, 1: 0, 2: 3, 3: 2} {
		if got, ok := rev[e]; !ok || got != want {
			t.Errorf("twin of edge %d = %d (found %v), want %d", e, got, ok, want)
		}
	}

	// Dual carriageway: two one-way roads, A->B via the north and B->A via
	// the south. Neither is the other's twin.
	net.Edges = []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 204, Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{a, north, b}},
		{ID: 1, From: 1, To: 0, Length: 204, Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{b, south, a}},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: FullClose})
	if w.Incidents[0] != FullClose {
		t.Fatalf("clicked carriageway not closed")
	}
	if sev, ok := w.Incidents[1]; ok {
		t.Fatalf("closing one carriageway closed the other (severity %d)", sev)
	}
}

// TestMerge_LevelCarsBrakeGently: two cars level with each other 70 m before
// a lane drop. The one that falls in behind used to see the other as a leader
// at zero gap and brake at MaxBraking; it has 70 m to drop back.
func TestMerge_LevelCarsBrakeGently(t *testing.T) {
	a := network.Point{X: 0, Y: 0}
	b := network.Point{X: 200, Y: 0}
	c := network.Point{X: 600, Y: 0}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: a}, {ID: 1, Pos: b}, {ID: 2, Pos: c}},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 15,
				Lanes: []network.Lane{{Index: 0}, {Index: 1}}, Geometry: []network.Point{a, b}},
			{ID: 1, From: 1, To: 2, Length: 400, SpeedLimit: 15,
				Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{b, c}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 130, V: 15, LaneChangeCooldown: 100},
		{ID: 2, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 1, S: 130, V: 15, LaneChangeCooldown: 100},
	}
	w.Step()
	for _, v := range w.Vehicles {
		if v.A < -2 {
			t.Fatalf("vehicle %d braked at %.2f m/s² with 70 m left to merge", v.ID, v.A)
		}
	}
	for i := 0; i < 600; i++ {
		w.Step()
		if g := minSameLaneGap(w); g < 0 {
			t.Fatalf("tick %d: merging vehicles overlap (min gap %.2f m)", w.Tick, g)
		}
	}
	for _, v := range w.Vehicles {
		if v.Edge != 1 {
			t.Fatalf("vehicle %d never crossed the lane drop (edge %d, S %.1f)", v.ID, v.Edge, v.S)
		}
	}
}

// TestSpawnEntry_SkipsClosedLane: spawns used to take lane 0 whenever it had
// room, even when an incident had closed it, so the newcomer had to cut
// across before the obstacle or got stuck at it.
func TestSpawnEntry_SkipsClosedLane(t *testing.T) {
	a := network.Point{X: 0, Y: 0}
	b := network.Point{X: 300, Y: 0}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: a}, {ID: 1, Pos: b}},
		Edges: []network.Edge{{ID: 0, From: 0, To: 1, Length: 300, SpeedLimit: 15,
			Lanes: []network.Lane{{Index: 0}, {Index: 1}}, Geometry: []network.Point{a, b}}},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = LaneClose
	lane, speed, ok := w.spawnEntry([]network.EdgeID{0}, 15, w.buildSpawnOccupancy())
	if !ok || lane != 1 {
		t.Fatalf("spawnEntry = lane %d ok %v, want the open lane 1", lane, ok)
	}
	if speed < 15 {
		t.Errorf("entry speed %.1f capped although the open lane has no obstacle", speed)
	}
}
