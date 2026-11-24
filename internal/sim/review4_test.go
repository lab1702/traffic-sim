package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Regression tests for the fourth full code review.

// TestJunction_AllWayStopGivesWayToYield: a junction mixing all-way-stop and
// yield approaches with no priority road (a directional give_way on an
// equal-class crossing). The yield car only checked yield approaches and the
// all-way-stop car only checked approaches whose head had stopped, so after
// its dwell the all-way-stop car pulled out in front of an arriving yield car
// and both went. The yield approach has priority (gapBlocked's strictness
// order), so the all-way-stop car must wait for it.
func TestJunction_AllWayStopGivesWayToYield(t *testing.T) {
	net := signedCrossing(network.ControlAllWayStop)
	net.Intersections[0].IncomingControl = ctrls(network.ControlAllWayStop, network.ControlYield)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.SimTime = 10
	w.Vehicles = []Vehicle{
		// At the all-way-stop line, dwell long done.
		{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 93, V: 0, StoppedSinceSec: 5},
		// Yield approach, 20 m out at 10 m/s: 2 s away.
		{ID: 2, Route: []network.EdgeID{1, 3}, Edge: 1, S: 80, V: 10},
	}
	w.buildArrivals()
	if s := w.nextStop(&w.Vehicles[0], 0); !s.yield {
		t.Fatalf("all-way-stop car may go with a yield car 2 s from the junction: %+v", s)
	}
	if s := w.nextStop(&w.Vehicles[1], 1); s.any() {
		t.Fatalf("yield car is held by the all-way-stop car, which must give way to it: %+v", s)
	}

	// Once the yield car is through, the all-way-stop car goes.
	w.Vehicles = w.Vehicles[:1]
	w.buildArrivals()
	if s := w.nextStop(&w.Vehicles[0], 0); s.any() {
		t.Fatalf("all-way-stop car still held with the junction clear: %+v", s)
	}
}

// TestWorld_StuckGuardSparesQueueThroughLongRed: a fixed-time signal holds
// the approach red for 103 s. Its approach edge is 10 m, so only the head car
// is on it; the cars queued behind it on the edge before used to count as
// stuck and were despawned at 60 s.
func TestWorld_StuckGuardSparesQueueThroughLongRed(t *testing.T) {
	pts := []network.Point{{X: 0}, {X: 200}, {X: 210}, {X: 400}, {X: 210, Y: -100}, {X: 210, Y: 100}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 14, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	net := &network.Network{
		Nodes: nodes,
		// Edge 1 is the 10 m approach to the signal at node 2; edge 3 crosses.
		Edges: []network.Edge{mk(0, 0, 1, 200), mk(1, 1, 2, 10), mk(2, 2, 3, 190), mk(3, 4, 2, 100), mk(4, 2, 5, 100)},
		Intersections: []network.Intersection{{
			ID: 0, NodeID: 2, HasSignal: true,
			Incoming: []network.EdgeID{1, 3}, IncomingControl: ctrls(network.ControlNone, network.ControlNone),
			Outgoing: []network.EdgeID{2, 4},
		}},
	}
	// Phase 0 serves the cross street for 100 s first.
	w := NewWorld(net, NewRandomOD(net, 0, 0), map[network.IntersectionID]SignalConfig{0: {Phases: []SignalPhase{
		{GreenEdges: []int{1}, GreenDur: 100, YellowDur: 3},
		{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3},
	}}})
	// Head car at rest 7 m from the line, the rest at S0 behind it on edge 0.
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{1, 2}, Edge: 1, S: 3}}
	for k := 0; k < 4; k++ {
		w.Vehicles = append(w.Vehicles, Vehicle{
			ID: VehicleID(k + 2), Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 196 - 7*float64(k),
		})
	}
	w.nextID = 10
	for i := 0; i < 1600; i++ { // 80 s, all of it red
		w.Step()
	}
	for id := VehicleID(1); id <= 5; id++ {
		if vehByID(w, id) == nil {
			t.Errorf("car %d, queued at the red, was despawned as stuck", id)
		}
	}
	// The queue still clears on green.
	for i := 0; i < 1200; i++ { // 60 s more: green from 103 s
		w.Step()
	}
	if len(w.Vehicles) != 0 {
		t.Errorf("%d cars still queued after the green", len(w.Vehicles))
	}
}

// TestSpawn_SeesFollowerBehindTurningCar: the follower check only looked at
// the frontmost car on each edge leading to the spawn edge. When that car
// turned elsewhere, the car right behind it, heading straight onto the spawn
// edge at speed, went unchecked and the newcomer appeared 5 m in front of it.
func TestSpawn_SeesFollowerBehindTurningCar(t *testing.T) {
	pts := []network.Point{{X: -30}, {X: 0}, {X: 200}, {X: 0, Y: -100}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 14, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	// Edge 0 leads into node 1; edge 1 (the spawn edge) carries on east;
	// edge 2 turns right.
	net := &network.Network{Nodes: nodes, Edges: []network.Edge{mk(0, 0, 1, 30), mk(1, 1, 2, 200), mk(2, 1, 3, 100)}}
	for _, withFollower := range []bool{true, false} {
		w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
		w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 2}, Edge: 0, S: 28, V: 14}}
		if withFollower {
			w.Vehicles = append(w.Vehicles, Vehicle{ID: 2, Route: []network.EdgeID{0, 1}, Edge: 0, S: 20, V: 14})
		}
		w.nextID = 3
		before := len(w.Vehicles)
		w.trySpawn(SpawnRequest{OriginNode: 1, DestNode: 2}, w.buildSpawnOccupancy())
		spawned := len(w.Vehicles) > before
		if withFollower && spawned {
			t.Errorf("spawned 5 m in front of a car doing 14 m/s onto the spawn edge, hidden behind a turning car")
		}
		if !withFollower && !spawned {
			t.Errorf("spawn refused with only a car turning away from the spawn edge")
		}
	}
}

// TestReroute_UTurnsAwayFromClosure: a GPS car heading east on edge 0 finds
// the road ahead (edge 1) closed. At node B the only other way is a U-turn
// back west, then round by E. The router allowed a U-turn only when nothing
// else was possible, and counted the closed road as possible, so the car kept
// its route through the closure, sat at the entry block and was despawned.
func TestReroute_UTurnsAwayFromClosure(t *testing.T) {
	a, b, c, e := network.Point{X: 0}, network.Point{X: 100}, network.Point{X: 200}, network.Point{X: 100, Y: 100}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: a}, {ID: 1, Pos: b}, {ID: 2, Pos: c}, {ID: 3, Pos: e}},
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{a, b}},
			{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{b, c}},
			{ID: 2, From: 1, To: 0, Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{b, a}},
			{ID: 3, From: 2, To: 1, Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{c, b}},
			{ID: 4, From: 0, To: 3, Length: 141, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{a, e}},
			{ID: 5, From: 3, To: 2, Length: 141, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{e, c}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 50, V: 10, HasGPS: true, DestNode: 2}}
	w.nextID = 2
	w.applyIncident(IncidentEvent{EdgeID: 1, Severity: FullClose})
	w.Step()
	v := vehByID(w, 1)
	if v == nil || len(v.Route) < 2 || v.Route[1] != 2 {
		t.Fatalf("GPS car did not turn round at the closure: %+v", v)
	}
}

// TestReroute_NoRetryEveryTickAtUnavoidableClosure: with no way round a
// closure, a GPS car bypassed its reroute cooldown on every tick, running a
// whole-network search each time. It now searches once per incident change.
func TestReroute_NoRetryEveryTickAtUnavoidableClosure(t *testing.T) {
	a, b, c := network.Point{X: 0}, network.Point{X: 100}, network.Point{X: 200}
	net := &network.Network{
		Nodes: []network.Node{{ID: 0, Pos: a}, {ID: 1, Pos: b}, {ID: 2, Pos: c}},
		Edges: []network.Edge{ // one-way: no U-turn possible either
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{a, b}},
			{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10, Lanes: []network.Lane{{}}, Geometry: []network.Point{b, c}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, S: 50, V: 10, HasGPS: true, DestNode: 2}}
	w.nextID = 2
	w.applyIncident(IncidentEvent{EdgeID: 1, Severity: FullClose})
	for i := 0; i < 100; i++ { // 5 s, well inside the 20 s cooldown
		w.Step()
	}
	if got := vehByID(w, 1).LastRerouteSec; got != 0 {
		t.Fatalf("GPS car kept re-searching at an unavoidable closure (last attempt at %.2f s, want only the first at 0 s)", got)
	}
	// A change to the incidents re-arms the bypass.
	w.applyIncident(IncidentEvent{EdgeID: 0, Severity: Slowdown})
	w.Step()
	if got := vehByID(w, 1).LastRerouteSec; got < 4.9 {
		t.Fatalf("no reroute attempt after an incident change (last attempt at %.2f s)", got)
	}
}

// TestSignal_ActuatedStartsOnMajorPhase: a semi-actuated signal started on
// phase 0 even when its rest phase was 1, so at t=0 the side street got an
// uncalled green and the arterial sat at red until it gapped out.
func TestSignal_ActuatedStartsOnMajorPhase(t *testing.T) {
	net := actuatedT()
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	st := w.SignalStates[0]
	if st.Config.Plan != PlanSemiActuated || st.Config.MajorPhase != 1 {
		t.Fatalf("fixture: plan %v major %d, want semi-actuated with major phase 1", st.Config.Plan, st.Config.MajorPhase)
	}
	if st.PhaseIdx != 1 || st.IsYellow {
		t.Fatalf("starts on phase %d (yellow %v), want the major phase 1 green", st.PhaseIdx, st.IsYellow)
	}
}

// TestSignal_CallDroppedDuringMajorYellowIsServed: the major phase went
// yellow for a side-street call, but the successor was chosen from the calls
// at the end of the yellow. When the call had dropped meanwhile (its car
// rerouted, or a nearer line suppressed its detection) the arterial went
// yellow and straight back to green, serving nobody.
func TestSignal_CallDroppedDuringMajorYellowIsServed(t *testing.T) {
	net := actuatedT()
	st := NewSignalState(DefaultSignalConfig(net.Intersections[0].Incoming, net))
	st.PhaseIdx = st.Config.MajorPhase // independent of the start-phase fix
	call := []bool{true, false}        // phase 0 (the side street) called
	none := []bool{false, false}
	dt := DefaultDt
	for i := 0; !st.IsYellow; i++ {
		if i > 1000 {
			t.Fatalf("major phase never yielded to the call")
		}
		st.AdvanceActuated(dt, call, none)
	}
	for st.IsYellow {
		st.AdvanceActuated(dt, none, none) // the call is gone during the yellow
	}
	if st.PhaseIdx != 0 {
		t.Fatalf("after the major's yellow the signal is on phase %d, want the side street's phase 0", st.PhaseIdx)
	}
}

// TestNextStop_IncidentNotSoftenedByFarYield: lane 0 of the approach is
// closed at its end, which is also a yield line whose gap is shut. From
// beyond gapDecisionDist the shut gap made the whole stop "soft", so no
// stopped virtual leader was placed for the closure either.
func TestNextStop_IncidentNotSoftenedByFarYield(t *testing.T) {
	pts := []network.Point{{X: 0}, {X: 300}, {X: 400}, {X: 300, Y: -100}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64, lanes int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10, Lanes: make([]network.Lane, lanes),
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	net := &network.Network{
		Nodes: nodes,
		Edges: []network.Edge{mk(0, 0, 1, 300, 2), mk(1, 1, 2, 100, 1), mk(2, 3, 1, 100, 1)},
		Intersections: []network.Intersection{{
			ID: 0, NodeID: 1,
			Incoming: []network.EdgeID{2, 0}, IncomingControl: ctrls(network.ControlNone, network.ControlYield),
			Opposing: []int8{-1, -1}, Outgoing: []network.EdgeID{1},
		}},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[0] = LaneClose
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 10, V: 10}, // 290 m from the line
		{ID: 2, Route: []network.EdgeID{2, 1}, Edge: 2, S: 80, V: 10},          // priority car 2 s out
	}
	w.buildArrivals()
	s := w.nextStop(&w.Vehicles[0], 0)
	if !s.incident || !s.yield {
		t.Fatalf("fixture: want both the closure and the shut gap at the line, got %+v", s)
	}
	if s.soft {
		t.Fatalf("the closed lane's stop is soft (no virtual leader) because of a far-off shut gap: %+v", s)
	}
}

// shortEdgeRoad is a straight two-lane road split into a 100 m edge 0, a 12 m
// edge 1 and a 288 m edge 2, with no junctions, limit 14 m/s.
func shortEdgeRoad() *network.Network {
	pts := []network.Point{{X: 0}, {X: 100}, {X: 112}, {X: 400}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(id), To: network.NodeID(id + 1),
			Length: pts[id+1].X - pts[id].X, SpeedLimit: 14,
			Lanes:    []network.Lane{{Index: 0}, {Index: 1}},
			Geometry: []network.Point{pts[id], pts[id+1]},
		}
	}
	return &network.Network{Nodes: nodes, Edges: []network.Edge{mk(0), mk(1), mk(2)}}
}

// TestLaneChange_SeesAcrossNodes: lane-change safety only looked at the
// target lane on the current edge. On the short edges netbuild makes, a car
// leaving a closed lane cut in front of a car a few metres back on the edge
// before, onto a car just past the next node, or onto one that had crossed
// onto the edge earlier in the same tick.
func TestLaneChange_SeesAcrossNodes(t *testing.T) {
	cases := []struct {
		name     string
		vehicles []Vehicle
		egoIdx   int
		wantLane uint8
	}{
		{"follower on the edge before", []Vehicle{
			{ID: 1, Route: []network.EdgeID{1, 2}, Edge: 1, Lane: 0, S: 2, V: 5},
			{ID: 2, Route: []network.EdgeID{0, 1, 2}, Edge: 0, Lane: 1, S: 97, V: 14},
		}, 0, 0},
		{"stopped car past the next node", []Vehicle{
			{ID: 1, Route: []network.EdgeID{1, 2}, Edge: 1, Lane: 0, S: 10, V: 5},
			{ID: 2, Route: []network.EdgeID{2}, Edge: 2, Lane: 1, S: 3, V: 0},
		}, 0, 0},
		{"follower that crossed onto the edge this tick", []Vehicle{
			{ID: 2, Route: []network.EdgeID{0, 1, 2}, Edge: 0, Lane: 1, S: 99.5, V: 14},
			{ID: 1, Route: []network.EdgeID{1, 2}, Edge: 1, Lane: 0, S: 4, V: 5},
		}, 1, 0},
		{"clear target lane", []Vehicle{
			{ID: 1, Route: []network.EdgeID{1, 2}, Edge: 1, Lane: 0, S: 2, V: 5},
			{ID: 2, Route: []network.EdgeID{0, 1, 2}, Edge: 0, Lane: 1, S: 0, V: 5},
		}, 0, 1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			net := shortEdgeRoad()
			w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
			w.Incidents[1] = LaneClose // lane 0 of edge 1: the ego must vacate it
			w.Vehicles = append([]Vehicle(nil), tc.vehicles...)
			w.nextID = 3
			w.Step()
			if got := w.Vehicles[tc.egoIdx].Lane; got != tc.wantLane {
				t.Fatalf("ego in lane %d after the tick, want %d (vehicles %+v)", got, tc.wantLane, w.Vehicles)
			}
		})
	}
}
