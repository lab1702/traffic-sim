package sim

import (
	"fmt"
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Regression tests for the route lookahead (lookahead.go). Each scenario used
// to fail because a check looked at most one edge ahead along the route, and
// netbuild's splitting at shared and signal nodes makes 4-20 m edges common.

// chainNet builds a straight eastbound chain: edge i runs from node i to node
// i+1, has length lengths[i], one lane and the given speed limit. It has no
// intersections; callers add them.
func chainNet(limit float64, lengths ...float64) *network.Network {
	nodes := []network.Node{{ID: 0}}
	x := 0.0
	for i, l := range lengths {
		x += l
		nodes = append(nodes, network.Node{ID: network.NodeID(i + 1), Pos: network.Point{X: x}})
	}
	edges := make([]network.Edge, 0, len(lengths))
	for i, l := range lengths {
		edges = append(edges, network.Edge{
			ID: network.EdgeID(i), From: network.NodeID(i), To: network.NodeID(i + 1),
			Length: l, SpeedLimit: limit,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[i].Pos, nodes[i+1].Pos},
		})
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

// chainPos is a vehicle's front-bumper position along a chainNet, from node 0.
func chainPos(net *network.Network, v *Vehicle) float64 {
	p := v.S
	for e := 0; e < int(v.Edge); e++ {
		p += net.Edges[e].Length
	}
	return p
}

// vehByID returns the live vehicle with the given ID, or nil.
func vehByID(w *World, id VehicleID) *Vehicle {
	for i := range w.Vehicles {
		if w.Vehicles[i].ID == id && !w.Vehicles[i].Despawned {
			return &w.Vehicles[i]
		}
	}
	return nil
}

// TestLookahead_LeaderBeyondShortEmptyEdge (review probe 1): a follower
// cruising on A (200 m) towards a short, empty B and a car stopped 1 m into C
// saw no leader until it was on B, too late to stop. The review measured a
// bumper gap of -87.9 m with B = 4 m and -4.3 m with B = 10 m.
func TestLookahead_LeaderBeyondShortEmptyEdge(t *testing.T) {
	for _, lb := range []float64{4, 10, 20} {
		t.Run(fmt.Sprintf("B=%gm", lb), func(t *testing.T) {
			net := chainNet(13.4, 200, lb, 200)
			w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
			w.Vehicles = []Vehicle{
				{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 0, V: 13.4},
				{ID: 2, Route: []network.EdgeID{2}, Edge: 2, S: 1, V: 0},
			}
			w.nextID = 3
			minGap := math.Inf(1)
			for i := 0; i < 600; i++ { // 30 s
				ld := vehByID(w, 2)
				ld.S, ld.V = 1, 0 // keep the leader parked
				w.Step()
				f, ld := vehByID(w, 1), vehByID(w, 2)
				if f == nil || ld == nil {
					t.Fatalf("tick %d: lost a vehicle", i)
				}
				if g := chainPos(net, ld) - chainPos(net, f) - VehicleLength; g < minGap {
					minGap = g
				}
			}
			if minGap < 0 {
				t.Fatalf("follower drove into the stopped car: min bumper gap %.1f m", minGap)
			}
		})
	}
}

// signalBeyondShortEdge returns A(200 m) -> B(lb) -> C(100 m) with a signal at
// the end of B. A side road D (edge 3) also enters the junction, so a phase
// can be green for D and permanently red for B.
func signalBeyondShortEdge(limit, lb float64) *network.Network {
	net := chainNet(limit, 200, lb, 100)
	j := net.Nodes[2].Pos
	side := network.Point{X: j.X, Y: -100}
	net.Nodes = append(net.Nodes, network.Node{ID: 4, Pos: side})
	net.Edges = append(net.Edges, network.Edge{
		ID: 3, From: 4, To: 2, Length: 100, SpeedLimit: limit,
		Lanes:    []network.Lane{{Index: 0}},
		Geometry: []network.Point{side, j},
	})
	net.Intersections = []network.Intersection{{
		ID: 0, NodeID: 2,
		Incoming:        []network.EdgeID{1, 3},
		IncomingControl: allNone(2),
		Opposing:        []int8{-1, -1},
		Outgoing:        []network.EdgeID{2},
		HasSignal:       true,
	}}
	return net
}

// TestLookahead_StopsForRedBeyondShortEdge (review probe 2): a car at
// 13-14 m/s on A(200 m) -> B(L) with a permanent red at B's end only saw the
// red once on B, and ran it for L = 5 m and 10 m.
func TestLookahead_StopsForRedBeyondShortEdge(t *testing.T) {
	for _, lb := range []float64{5, 10, 20} {
		t.Run(fmt.Sprintf("B=%gm", lb), func(t *testing.T) {
			net := signalBeyondShortEdge(13.5, lb)
			// Green for the side road only: B is red for the whole run.
			w := NewWorld(net, NewRandomOD(net, 0, 0), map[network.IntersectionID]SignalConfig{
				0: {Phases: []SignalPhase{{GreenEdges: []int{1}, GreenDur: 1e6, YellowDur: 3}}},
			})
			w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 40, V: 13.5}}
			w.nextID = 2
			line := 200 + lb
			for i := 0; i < 800; i++ { // 40 s
				w.Step()
				v := vehByID(w, 1)
				if v == nil || v.Edge == 2 {
					t.Fatalf("tick %d: car ran the red at the end of the %g m edge", i, lb)
				}
				if p := chainPos(net, v); p > line {
					t.Fatalf("tick %d: car is %.1f m past the stop line", i, p-line)
				}
			}
			if v := vehByID(w, 1); v.V > stuckSpeedThresh {
				t.Fatalf("car still moving at %.2f m/s after 40 s of red", v.V)
			}
		})
	}
}

// priorityBeyondShortEdge builds a junction X (node 2) fed by a priority
// approach split into edge 0 (first, 100 m or as given) and edge 1 (a 10 m
// junction edge), and a yield approach, edge 3 (100 m, from the south).
// Edge 2 continues the priority road east; edge 4 continues the yield road
// north. ring marks edges 0 and 1 as roundabout ring segments.
func priorityBeyondShortEdge(firstLen float64, ring bool) *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -10 - firstLen, Y: 0}},
		{ID: 1, Pos: network.Point{X: -10, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 100, Y: 0}},
		{ID: 4, Pos: network.Point{X: 0, Y: -100}},
		{ID: 5, Pos: network.Point{X: 0, Y: 100}},
	}
	mk := func(id, from, to int, length float64, roundabout bool) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10, Roundabout: roundabout,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mk(0, 0, 1, firstLen, ring),
		mk(1, 1, 2, 10, ring),
		mk(2, 2, 3, 100, false),
		mk(3, 4, 2, 100, false),
		mk(4, 2, 5, 100, false),
	}
	xs := []network.Intersection{{
		ID: 0, NodeID: 2,
		Incoming:        []network.EdgeID{1, 3},
		IncomingControl: ctrls(network.ControlNone, network.ControlYield),
		Opposing:        []int8{-1, -1},
		Outgoing:        []network.EdgeID{2, 4},
	}}
	return &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}
}

// opposingBeyondShortEdge builds a priority-road junction X (node 2) where ego
// comes up from the south (edge 3) and turns left (west, edge 4), and the
// opposing southbound traffic arrives over edge 0 (100 m) then a 10 m
// junction edge 1, continuing south on edge 2.
func opposingBeyondShortEdge() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 110}},
		{ID: 1, Pos: network.Point{X: 0, Y: 10}},
		{ID: 2, Pos: network.Point{X: 0, Y: 0}},
		{ID: 3, Pos: network.Point{X: 0, Y: -100}},
		{ID: 4, Pos: network.Point{X: -100, Y: 0}},
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	edges := []network.Edge{
		mk(0, 0, 1, 100),
		mk(1, 1, 2, 10),
		mk(2, 2, 3, 100),
		mk(3, 3, 2, 100),
		mk(4, 2, 4, 100),
	}
	xs := []network.Intersection{{
		ID: 0, NodeID: 2,
		Incoming:        []network.EdgeID{1, 3},
		IncomingControl: allNone(2),
		Opposing:        []int8{1, 0},
		Outgoing:        []network.EdgeID{2, 4},
	}}
	return &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}
}

// TestLookahead_GapChecksSeeUpstreamOfShortJunctionEdge (review probe 3): a
// priority car 15 m from the junction at 10 m/s (ETA 1.5 s), still on the
// edge upstream of a 10 m junction edge, was invisible to the gap check, so a
// car waiting at the line pulled out in front of it. Roundabout entries and
// left turns across opposing traffic had the same blind spot.
func TestLookahead_GapChecksSeeUpstreamOfShortJunctionEdge(t *testing.T) {
	cases := []struct {
		name string
		net  *network.Network
		prio Vehicle // the priority car, 15 m from the junction
		ego  Vehicle // waiting 7 m before its line, i.e. stopped at it
	}{
		{
			name: "yield",
			net:  priorityBeyondShortEdge(100, false),
			prio: Vehicle{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 95, V: 10},
			ego:  Vehicle{ID: 2, Route: []network.EdgeID{3, 4}, Edge: 3, S: 93},
		},
		{
			name: "roundabout",
			net:  priorityBeyondShortEdge(20, true),
			prio: Vehicle{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 15, V: 10},
			ego:  Vehicle{ID: 2, Route: []network.EdgeID{3, 4}, Edge: 3, S: 93},
		},
		{
			name: "left turn",
			net:  opposingBeyondShortEdge(),
			prio: Vehicle{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 95, V: 10},
			ego:  Vehicle{ID: 2, Route: []network.EdgeID{3, 4}, Edge: 3, S: 93},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			w := NewWorld(tc.net, NewRandomOD(tc.net, 0, 0), nil)
			w.Vehicles = []Vehicle{tc.prio, tc.ego}
			w.nextID = 3
			for i := 0; i < 100; i++ { // the priority car needs ~1.5 s
				w.Step()
				p, e := vehByID(w, 1), vehByID(w, 2)
				if p == nil || p.RouteIdx >= 2 {
					return // the priority car is through the junction
				}
				// Starting from rest, one tick of acceleration is 0.05 m/s.
				if e == nil || e.Edge != tc.ego.Edge || e.V > 0.02 {
					d := 10 - p.S // still on the 10 m junction edge
					if p.Edge == 0 {
						d = tc.net.Edges[0].Length - p.S + 10
					}
					t.Fatalf("tick %d: waiting car pulled out with the priority car %.1f m from the junction", i, d)
				}
			}
			t.Fatal("priority car never reached the junction")
		})
	}
}

// TestLookahead_SpawnSeesQueueBeyondShortFirstEdge (review probe 4): a car
// spawned onto a short first edge only checked that edge for room, so it
// entered at cruising speed straight into the queue just beyond it.
func TestLookahead_SpawnSeesQueueBeyondShortFirstEdge(t *testing.T) {
	p := DefaultIDM()
	t.Run("slows", func(t *testing.T) {
		net := chainNet(13.4, 4, 100, 100)
		w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
		// Queue tail 5 m into B: a newcomer at S=0 on A has 4+5-5 = 4 m of gap.
		w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{1, 2}, Edge: 1, S: 5}}
		w.trySpawn(SpawnRequest{OriginNode: 0, DestNode: 3}, w.buildSpawnOccupancy())
		if len(w.Vehicles) != 2 {
			t.Fatalf("spawn refused with 4 m of room; got %d vehicles", len(w.Vehicles))
		}
		nv := w.Vehicles[1]
		if limit := math.Sqrt(2 * p.B * (4 - p.S0)); nv.V > limit+1e-9 {
			t.Fatalf("spawned at %.2f m/s with a stopped car 4 m ahead; want <= %.2f", nv.V, limit)
		}
		minGap := math.Inf(1)
		for i := 0; i < 200; i++ { // 10 s
			q := vehByID(w, 100)
			q.S, q.V = 5, 0
			w.Step()
			n, q := vehByID(w, nv.ID), vehByID(w, 100)
			if g := chainPos(net, q) - chainPos(net, n) - VehicleLength; g < minGap {
				minGap = g
			}
		}
		if minGap < 0 {
			t.Fatalf("spawned car ran into the queue: min bumper gap %.2f m", minGap)
		}
	})
	t.Run("blocked", func(t *testing.T) {
		net := chainNet(13.4, 4, 100, 100)
		w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
		// Queue tail 1 m into B: no room for a car at S=0 on A.
		w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{1, 2}, Edge: 1, S: 1}}
		w.trySpawn(SpawnRequest{OriginNode: 0, DestNode: 3}, w.buildSpawnOccupancy())
		if len(w.Vehicles) != 1 {
			t.Fatalf("spawned on top of the queue beyond the 4 m first edge")
		}
	})
}

// TestLookahead_DesiredSpeedAnticipatesLimitDrop: a lower limit on the next
// edge eases the desired speed down on approach, like a corner does.
func TestLookahead_DesiredSpeedAnticipatesLimitDrop(t *testing.T) {
	net := chainNet(25, 300, 100)
	net.Edges[1].SpeedLimit = 10
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	v := &Vehicle{Route: []network.EdgeID{0, 1}, Edge: 0, S: 250, V: 25}
	want := math.Sqrt(10*10 + 2*cornerBrakeDecel*50)
	if got := w.computeDesiredSpeed(v); math.Abs(got-want) > 1e-9 {
		t.Fatalf("50 m before a 25 -> 10 m/s drop: desired speed %.3f, want %.3f", got, want)
	}
}

// TestLookahead_DesiredSpeedSeesCornerBeyondShortEdge: a sharp corner at the
// end of a 5 m straight edge caps the desired speed while still on the edge
// before it.
func TestLookahead_DesiredSpeedSeesCornerBeyondShortEdge(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 205, Y: 0}},
		{ID: 3, Pos: network.Point{X: 205, Y: -100}},
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 13.4,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	net := &network.Network{Nodes: nodes, Edges: []network.Edge{mk(0, 0, 1, 200), mk(1, 1, 2, 5), mk(2, 2, 3, 100)}}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	vSafe := cornerSpeed(turnRadius(net, 1, 2))
	if vSafe >= 13.4 {
		t.Fatalf("fixture: corner speed %.2f does not constrain", vSafe)
	}
	v := &Vehicle{Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 190, V: 13.4} // corner 15 m ahead
	want := math.Sqrt(vSafe*vSafe + 2*cornerBrakeDecel*15)
	if got := w.computeDesiredSpeed(v); math.Abs(got-want) > 1e-9 {
		t.Fatalf("15 m before a corner beyond a 5 m edge: desired speed %.3f, want %.3f", got, want)
	}
}

// TestLookahead_StopsBeforeClosureBeyondShortEdge: the entry block for a fully
// closed edge holds the car even when a short edge separates it from the
// closure; it used to drive into the closed edge.
func TestLookahead_StopsBeforeClosureBeyondShortEdge(t *testing.T) {
	net := chainNet(13.4, 200, 5, 100, 100)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[2] = FullClose
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2, 3}, Edge: 0, S: 40, V: 13.4}}
	w.nextID = 2
	for i := 0; i < 600; i++ { // 30 s
		w.Step()
		v := vehByID(w, 1)
		if v == nil {
			return // stuck-despawned in front of the closure: acceptable, it never entered
		}
		if v.Edge >= 2 {
			t.Fatalf("tick %d: car entered the closed edge", i)
		}
	}
}

// TestLookahead_StopSignBeyondShortEdgeStopsOnce: a stop line at the end of a
// 4 m edge makes the car stop on the edge before it. The dwell it served
// there must carry over onto the short edge, so it stops exactly once.
func TestLookahead_StopSignBeyondShortEdgeStopsOnce(t *testing.T) {
	net := chainNet(10, 200, 4, 100)
	net.Intersections = []network.Intersection{{
		ID: 0, NodeID: 2,
		Incoming:        []network.EdgeID{1},
		IncomingControl: ctrls(network.ControlStop),
		Opposing:        []int8{-1},
		Outgoing:        []network.EdgeID{2},
	}}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 100, V: 10}}
	w.nextID = 2
	stops, moving := 0, true
	for i := 0; i < 600; i++ { // 30 s
		w.Step()
		v := vehByID(w, 1)
		if v == nil {
			t.Fatalf("tick %d: car despawned before clearing the stop sign", i)
		}
		if v.Edge == 2 {
			if stops != 1 {
				t.Fatalf("car stopped %d times before the stop line, want exactly 1", stops)
			}
			return
		}
		if moving && v.V < stuckSpeedThresh {
			stops++
		}
		moving = v.V >= stuckSpeedThresh
	}
	t.Fatalf("car never cleared the stop sign (stops=%d)", stops)
}

// redStop is nextStop's verdict for a red (or a yellow the car stops for):
// (distance to the line, must stop).
func redStop(w *World, v *Vehicle) (float64, bool) {
	s := w.nextStop(v, -1)
	return s.dist, s.red
}

// incidentStop is nextStop's verdict for an incident: (distance from the
// front bumper to the obstacle or entry block, blocked).
func incidentStop(w *World, v *Vehicle) (float64, bool) {
	s := w.nextStop(v, -1)
	return s.dist, s.incident
}

// TestLookahead_YellowCallSurvivesNodeBeforeLine: a car that decides to go
// on a yellow at the end of a short edge, while still on the edge before it,
// keeps that decision across the node in between, and drops it once it has
// crossed the signal's line.
func TestLookahead_YellowCallSurvivesNodeBeforeLine(t *testing.T) {
	net := signalBeyondShortEdge(13.5, 10)
	w := NewWorld(net, NewRandomOD(net, 0, 0), map[network.IntersectionID]SignalConfig{
		0: {Phases: []SignalPhase{{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3}, {GreenEdges: []int{1}, GreenDur: 30}}},
	})
	w.SignalStates[0].IsYellow = true
	v := &Vehicle{Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 175, V: 13.5} // 35 m from the line
	if s := w.nextStop(v, -1); s.red || v.YellowCall != yellowGo || v.LineIdx != 1 {
		t.Fatalf("35 m out at 13.5 m/s with 3 s of yellow: want go owned by line 1, got red=%v call=%d line=%d",
			s.red, v.YellowCall, v.LineIdx)
	}
	stepIDM(v, 13.5, 0, 0, false, net, DefaultIDM(), 2) // 27 m: onto the 10 m edge
	if v.Edge != 1 || v.YellowCall != yellowGo {
		t.Fatalf("after the node before the line: edge %d call %d, want edge 1 and the go call kept", v.Edge, v.YellowCall)
	}
	stepIDM(v, 13.5, 0, 0, false, net, DefaultIDM(), 1) // 13.5 m: across the line
	if v.Edge != 2 || v.YellowCall != yellowUndecided || v.LineIdx != 2 {
		t.Fatalf("after the line: edge %d call %d line %d, want edge 2, undecided, line 2", v.Edge, v.YellowCall, v.LineIdx)
	}
}

// TestLookahead_ActuatedCallSeesCarBeforeShortApproach: a side-street car
// that stops for the red while still on the edge before a 4 m approach edge
// (it rests ~7 m before the line) must still call its phase. The detector
// zone used to cover only the approach edge itself, so the car was never
// detected and waited for ever.
func TestLookahead_ActuatedCallSeesCarBeforeShortApproach(t *testing.T) {
	net := signalCross()
	// Split the N approach (edge 0, 200 m) into 196 m + a 4 m junction edge 8.
	split := network.Point{X: 0, Y: 4}
	net.Nodes = append(net.Nodes, network.Node{ID: 5, Pos: split})
	net.Edges[0].To, net.Edges[0].Length = 5, 196
	net.Edges[0].Geometry = []network.Point{net.Nodes[0].Pos, split}
	short := net.Edges[0]
	short.ID, short.From, short.To, short.Length = 8, 5, 4, 4
	short.Geometry = []network.Point{split, net.Nodes[4].Pos}
	net.Edges = append(net.Edges, short)
	net.Intersections[0].Incoming[0] = 8

	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	st := w.SignalStates[0]
	if st.Config.Plan != PlanSemiActuated || st.GreenFor(0) {
		t.Fatalf("fixture: want a semi-actuated signal resting red for the N approach")
	}
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 8, 6}, Edge: 0, S: 100, V: 10}}
	w.nextID = 2
	for i := 0; i < 4000; i++ { // 200 s
		w.Step()
		if v := vehByID(w, 1); v == nil || v.Edge == 6 {
			return // crossed
		}
	}
	v := vehByID(w, 1)
	t.Fatalf("side-street car never got a green: edge %d S %.1f V %.2f", v.Edge, v.S, v.V)
}

// TestLookahead_TurnBiasPlansThroughShortEdge: the turn lane for a right turn
// lies on a 20 m edge, too short to change lanes on. Turn bias used to check
// only the next turn, which both lanes of the edge before allow, so a car in
// lane 1 stayed there and reached the 20 m edge in the straight-only lane
// (a "turn-lane snap fallback"). It must move to lane 0 on the edge before.
func TestLookahead_TurnBiasPlansThroughShortEdge(t *testing.T) {
	pts := []network.Point{{X: 0}, {X: 50}, {X: 70}, {X: 70, Y: -100}, {X: 170}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64, lanes ...network.Lane) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 13.4, Lanes: lanes,
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	net := &network.Network{Nodes: nodes, Edges: []network.Edge{
		mk(0, 0, 1, 50, network.Lane{Index: 0, AllowedTurns: []network.EdgeID{1}},
			network.Lane{Index: 1, AllowedTurns: []network.EdgeID{1}}),
		mk(1, 1, 2, 20, network.Lane{Index: 0, AllowedTurns: []network.EdgeID{2}}, // right only
			network.Lane{Index: 1, AllowedTurns: []network.EdgeID{3}}), // straight only
		mk(2, 2, 3, 100, network.Lane{Index: 0}),
		mk(3, 2, 4, 100, network.Lane{Index: 0}),
	}}
	if cat := network.ClassifyTurn(net, 1, 2); cat != network.TurnRight {
		t.Fatalf("fixture: 1 -> 2 should be a right turn, got %v", cat)
	}
	vs := []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, Lane: 1, S: 10, V: 10}}
	tryLaneChange(&vs[0], 0, map[uint8][]int{1: {0}}, vs, net, -1)
	if vs[0].Lane != 0 {
		t.Fatalf("car stayed in lane %d; lane 1 only leads to the straight-only lane of the 20 m edge", vs[0].Lane)
	}
	// Going straight instead, lane 1 suits the route: no change needed.
	vs = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 3}, Edge: 0, Lane: 1, S: 10, V: 10}}
	tryLaneChange(&vs[0], 0, map[uint8][]int{1: {0}}, vs, net, -1)
	if vs[0].Lane != 1 {
		t.Fatalf("straight-through car left lane 1 for lane %d", vs[0].Lane)
	}
}
