package sim

import (
	"fmt"
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Regression tests from the review of the route lookahead (ab91b8f): each
// scenario regressed there and works at its parent.

// closeLines builds A(200 m) -> B(gap) -> C(100 m) eastbound, with a line at
// the end of A (node 1, control c1) and at the end of B (node 2, control c2).
// Side roads from the south (edges 3 and 4) feed nodes 1 and 2 as priority
// approaches. With signals, both nodes are signalized instead.
func closeLines(gap float64, c1, c2 network.Control, signals bool) *network.Network {
	pts := []network.Point{{X: 0}, {X: 200}, {X: 200 + gap}, {X: 300 + gap}, {X: 200, Y: -100}, {X: 200 + gap, Y: -100}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	edges := []network.Edge{mk(0, 0, 1, 200), mk(1, 1, 2, gap), mk(2, 2, 3, 100), mk(3, 4, 1, 100), mk(4, 5, 2, 100)}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0, 3}, IncomingControl: ctrls(c1, network.ControlNone),
			Opposing: []int8{-1, -1}, Outgoing: []network.EdgeID{1}, HasSignal: signals},
		{ID: 1, NodeID: 2, Incoming: []network.EdgeID{1, 4}, IncomingControl: ctrls(c2, network.ControlNone),
			Opposing: []int8{-1, -1}, Outgoing: []network.EdgeID{2}, HasSignal: signals},
	}
	return &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}
}

// TestLookahead_CloseLinesDoNotDeadlock (review item 1): with two lines less
// than a stopping gap (~7 m) apart, the car stops for the second while still
// short of the first. The second line could not get its own dwell or wait
// state while the first held it, so the car waited for ever.
func TestLookahead_CloseLinesDoNotDeadlock(t *testing.T) {
	stop, yield := network.ControlStop, network.ControlYield
	cases := []struct {
		name         string
		gap          float64
		c1, c2       network.Control
		signalsOff   bool
		prio1, prio2 bool // a priority car pinned at ETA 2.5 s at line 1 / line 2
	}{
		{name: "stop-stop 4m", gap: 4, c1: stop, c2: stop},
		{name: "stop-stop 6m", gap: 6, c1: stop, c2: stop},
		{name: "signals off 5m", gap: 5, signalsOff: true},
		{name: "yield-stop 4m", gap: 4, c1: yield, c2: stop, prio1: true},
		{name: "yield-yield 4m", gap: 4, c1: yield, c2: yield, prio1: true, prio2: true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			net := closeLines(tc.gap, tc.c1, tc.c2, tc.signalsOff)
			w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
			if tc.signalsOff {
				w.SignalStates[0].Mode, w.SignalStates[1].Mode = ModeOff, ModeOff
			}
			w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 120, V: 10}}
			if tc.prio1 {
				w.Vehicles = append(w.Vehicles, Vehicle{ID: 2, Route: []network.EdgeID{3}, Edge: 3})
			}
			if tc.prio2 {
				w.Vehicles = append(w.Vehicles, Vehicle{ID: 3, Route: []network.EdgeID{4}, Edge: 4})
			}
			w.nextID = 4
			for i := 0; i < 1200; i++ { // 60 s
				for _, id := range []VehicleID{2, 3} {
					if p := vehByID(w, id); p != nil {
						p.S, p.V = 98.75, 0.5 // ETA 2.5 s: accepted only after 5 s of impatience
					}
				}
				w.Step()
				v := vehByID(w, 1)
				if v == nil {
					t.Fatalf("tick %d: car despawned before clearing both lines", i)
				}
				if v.Edge == 2 {
					return
				}
			}
			v := vehByID(w, 1)
			t.Fatalf("car stuck before the second line: edge %d S %.2f V %.2f StuckTime %.1f", v.Edge, v.S, v.V, v.StuckTime)
		})
	}
}

// TestLookahead_SecondActuatedSignalCloseBehindFirst (review item 2): on a
// dual carriageway with semi-actuated signals 5 m apart, the car waits for
// the second signal 2 m before the first. The detector walk stopped at the
// first signal's approach, so the second was never called.
func TestLookahead_SecondActuatedSignalCloseBehindFirst(t *testing.T) {
	pts := []network.Point{
		{X: 0, Y: -200}, {X: 0, Y: 0}, {X: 0, Y: 5}, {X: 0, Y: 105}, // minor road; X1 = node 1, X2 = node 2
		{X: -200, Y: 0}, {X: 200, Y: 0}, // eastbound carriageway through X1
		{X: 200, Y: 5}, {X: -200, Y: 5}, // westbound carriageway through X2
	}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64, cls network.RoadClass) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 13.4, Class: cls, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	res, pri := network.ClassResidential, network.ClassPrimary
	net := &network.Network{Nodes: nodes, Edges: []network.Edge{
		mk(0, 0, 1, 200, res), mk(1, 1, 2, 5, res), mk(2, 2, 3, 100, res),
		mk(3, 4, 1, 200, pri), mk(4, 1, 5, 200, pri), mk(5, 6, 2, 200, pri), mk(6, 2, 7, 200, pri),
	}, Intersections: []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0, 3}, IncomingControl: allNone(2),
			Opposing: []int8{-1, -1}, Outgoing: []network.EdgeID{1, 4}, HasSignal: true},
		{ID: 1, NodeID: 2, Incoming: []network.EdgeID{1, 5}, IncomingControl: allNone(2),
			Opposing: []int8{-1, -1}, Outgoing: []network.EdgeID{2, 6}, HasSignal: true},
	}}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	for i, st := range w.SignalStates {
		if st.Config.Plan != PlanSemiActuated || st.GreenFor(0) {
			t.Fatalf("fixture: signal %d should be semi-actuated and resting red for the minor road", i)
		}
	}
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 100, V: 10}}
	w.nextID = 2
	for i := 0; i < 2400; i++ { // 120 s
		w.Step()
		if v := vehByID(w, 1); v == nil || v.Edge == 2 {
			return
		}
	}
	v := vehByID(w, 1)
	t.Fatalf("car never got through the second signal: edge %d S %.2f V %.2f", v.Edge, v.S, v.V)
}

// TestWorld_StuckGuardClearsPackedLoopGridlock (review item 3): 30 cars
// packed bumper to bumper (gaps under S0) on a 200 m one-lane loop with stop
// signs at 2 of its 4 nodes are gridlocked. The stuck guard must still clear
// it; it counted every car with a stop sign anywhere in its lookahead as
// legitimately waiting, so all 30 froze for ever.
func TestWorld_StuckGuardClearsPackedLoopGridlock(t *testing.T) {
	pts := []network.Point{{X: 0, Y: 0}, {X: 50, Y: 0}, {X: 50, Y: 50}, {X: 0, Y: 50}}
	nodes := make([]network.Node, len(pts))
	edges := make([]network.Edge, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	for i := range pts {
		j := (i + 1) % len(pts)
		edges[i] = network.Edge{
			ID: network.EdgeID(i), From: network.NodeID(i), To: network.NodeID(j),
			Length: 50, SpeedLimit: 10, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{pts[i], pts[j]},
		}
	}
	net := &network.Network{Nodes: nodes, Edges: edges, Intersections: []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, IncomingControl: ctrls(network.ControlStop),
			Opposing: []int8{-1}, Outgoing: []network.EdgeID{1}},
		{ID: 1, NodeID: 3, Incoming: []network.EdgeID{2}, IncomingControl: ctrls(network.ControlStop),
			Opposing: []int8{-1}, Outgoing: []network.EdgeID{3}},
	}}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	const n = 30
	for i := 0; i < n; i++ {
		p := float64(i) * 200 / n
		e := int(p / 50)
		route := make([]network.EdgeID, 0, 160)
		for k := 0; k < 160; k++ {
			route = append(route, network.EdgeID((e+k)%4))
		}
		w.Vehicles = append(w.Vehicles, Vehicle{ID: VehicleID(i + 1), Route: route, Edge: route[0], S: p - float64(e)*50})
	}
	w.nextID = n + 1
	progress := func() int {
		sum := 0
		for i := range w.Vehicles {
			sum += w.Vehicles[i].RouteIdx
		}
		return sum
	}
	for i := 0; i < 1600; i++ { // 80 s: past the 60 s stuck timeout
		w.Step()
	}
	if len(w.Vehicles) == n {
		t.Fatalf("gridlocked loop not cleared: all %d cars still present after 80 s", n)
	}
	before := progress()
	for i := 0; i < 2400; i++ { // 120 s more
		w.Step()
	}
	if progress() <= before {
		t.Fatalf("loop still frozen after the stuck guard thinned it (%d cars left)", len(w.Vehicles))
	}
}

// TestLookahead_GPSDivertsAroundClosureBeyondShortEdge (review item 4): the
// entry block for a closure beyond a 5 m edge holds a GPS car on the edge
// before it. The reroute trigger only looked at the next edge, which is
// open, so the car never diverted and was despawned as stuck.
func TestLookahead_GPSDivertsAroundClosureBeyondShortEdge(t *testing.T) {
	pts := []network.Point{{X: 0}, {X: 200}, {X: 205}, {X: 305}, {X: 405}, {X: 255, Y: 80}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int) network.Edge {
		a, b := pts[from], pts[to]
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: math.Hypot(b.X-a.X, b.Y-a.Y), SpeedLimit: 13.4, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{a, b},
		}
	}
	net := &network.Network{Nodes: nodes, Edges: []network.Edge{
		mk(0, 0, 1), mk(1, 1, 2), mk(2, 2, 3), mk(3, 3, 4), // A, B (5 m), C (closed), D
		mk(4, 2, 5), mk(5, 5, 3), // detour from B's end round C
	}}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Incidents[2] = FullClose
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2, 3}, Edge: 0, S: 40, V: 13.4, HasGPS: true, DestNode: 4}}
	w.nextID = 2
	for i := 0; i < 1600; i++ { // 80 s
		w.Step()
		v := vehByID(w, 1)
		if v == nil {
			t.Fatalf("tick %d: GPS car despawned without diverting", i)
		}
		if v.Edge == 4 {
			return // on the detour
		}
		if v.Edge == 2 {
			t.Fatalf("tick %d: car entered the closed edge", i)
		}
	}
	t.Fatal("GPS car never diverted round the closure")
}

// TestSpawn_FollowerCheckNotSkippedByLargeUpstreamTree (review item 5): the
// upstream follower check shared one edge budget across a depth-first walk
// and approved the spawn once it ran out, so a big empty tree behind one
// in-edge hid a car 3 m from the node on the other.
func TestSpawn_FollowerCheckNotSkippedByLargeUpstreamTree(t *testing.T) {
	for _, levels := range []int{3, 9} {
		t.Run(fmt.Sprintf("tree %d levels", levels), func(t *testing.T) {
			net := &network.Network{}
			addNode := func(p network.Point) network.NodeID {
				id := network.NodeID(len(net.Nodes))
				net.Nodes = append(net.Nodes, network.Node{ID: id, Pos: p})
				return id
			}
			addEdge := func(from, to network.NodeID, length float64) network.EdgeID {
				id := network.EdgeID(len(net.Edges))
				net.Edges = append(net.Edges, network.Edge{
					ID: id, From: from, To: to, Length: length, SpeedLimit: 13.4,
					Lanes:    []network.Lane{{Index: 0}},
					Geometry: []network.Point{net.Nodes[from].Pos, net.Nodes[to].Pos},
				})
				return id
			}
			origin := addNode(network.Point{})
			dest := addNode(network.Point{X: 100})
			// In-edge 1 (lowest ID, so walked first): empty, with a binary
			// tree of 1 m edges behind it.
			t0 := addNode(network.Point{X: -1})
			addEdge(t0, origin, 1)
			frontier := []network.NodeID{t0}
			for l := 0; l < levels; l++ {
				var next []network.NodeID
				for _, nd := range frontier {
					for k := 0; k < 2; k++ {
						up := addNode(network.Point{X: -float64(l + 2), Y: float64(len(net.Nodes))})
						addEdge(up, nd, 1)
						next = append(next, up)
					}
				}
				frontier = next
			}
			// In-edge 2: a car 3 m before the node at cruising speed.
			src := addNode(network.Point{Y: -100})
			in2 := addEdge(src, origin, 100)
			spawnEdge := addEdge(origin, dest, 100)
			w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
			w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{in2, spawnEdge}, Edge: in2, S: 97, V: 13.4}}
			w.trySpawn(SpawnRequest{OriginNode: origin, DestNode: dest}, w.buildSpawnOccupancy())
			if len(w.Vehicles) != 1 {
				t.Fatalf("spawned at the node with a car 3 m behind it at 13.4 m/s")
			}
		})
	}
}

// TestLookahead_QueueTimeIsNotImpatience (review item 6): time spent queued
// on the edge before a short approach edge counted as waiting at the line,
// so the second car in the queue reached the line already impatient and
// crossed without stopping. Only time at the line (or, as before, on the
// approach edge itself) may count.
func TestLookahead_QueueTimeIsNotImpatience(t *testing.T) {
	// Yield approach Y0 (100 m) -> Y1 (10 m) -> junction (node 2) -> out;
	// priority approach P (100 m) into the junction from the south.
	pts := []network.Point{{X: -110}, {X: -10}, {X: 0}, {X: 100}, {X: 0, Y: -100}}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	net := &network.Network{Nodes: nodes, Edges: []network.Edge{
		mk(0, 0, 1, 100), mk(1, 1, 2, 10), mk(2, 2, 3, 100), mk(3, 4, 2, 100),
	}, Intersections: []network.Intersection{{
		ID: 0, NodeID: 2, Incoming: []network.EdgeID{1, 3},
		IncomingControl: ctrls(network.ControlYield, network.ControlNone),
		Opposing:        []int8{-1, -1}, Outgoing: []network.EdgeID{2},
	}}}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{1, 2}, Edge: 1, S: 3},     // Q1, at the line
		{ID: 2, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 96}, // Q2, queued behind on Y0
		{ID: 3, Route: []network.EdgeID{3}, Edge: 3},              // priority, pinned
	}
	w.nextID = 4
	q1Crossed, q2StoppedAtLine := false, false
	for i := 0; i < 1200; i++ { // 60 s
		if p := vehByID(w, 3); p != nil {
			p.S, p.V = 98.75, 0.5 // ETA 2.5 s: accepted after 5 s of waiting at the line
		}
		w.Step()
		q2 := vehByID(w, 2)
		if q2 == nil {
			t.Fatalf("tick %d: Q2 despawned", i)
		}
		if q1 := vehByID(w, 1); !q1Crossed && (q1 == nil || q1.Edge != 1) {
			q1Crossed = true
			if q2.WaitTime != 0 {
				t.Fatalf("Q2 had %.2f s of impatience from queuing on the edge before the approach", q2.WaitTime)
			}
		}
		if q2.Edge == 1 && q2.V < stuckSpeedThresh && net.Edges[1].Length-q2.S < stopLineTolMeters {
			q2StoppedAtLine = true
		}
		if q2.Edge == 2 {
			if !q2StoppedAtLine {
				t.Fatal("Q2 crossed without stopping at the line")
			}
			return
		}
	}
	if !q1Crossed {
		t.Fatal("Q1 never crossed")
	}
	t.Fatal("Q2 never crossed")
}

// TestVehicle_LineStatePerLine (review items 7 and 10): every stop line keeps
// its own dwell, wait and yellow call. Writing one line's state must not
// wipe another's, a reroute keeps memos for lines still on the route at the
// same place, and crossing a node hands the next line's memo to the fields.
func TestVehicle_LineStatePerLine(t *testing.T) {
	v := &Vehicle{Route: []network.EdgeID{0, 1, 2, 3}, Edge: 0}
	v.setLineAt(2, lineState{call: yellowGo})
	v.setLineAt(0, lineState{wait: 3})
	v.setLineAt(1, lineState{stoppedSince: 5})
	if v.lineAt(2).call != yellowGo || v.lineAt(0).wait != 3 || v.lineAt(1).stoppedSince != 5 || v.WaitTime != 3 {
		t.Fatalf("lines overwrote each other: %+v %+v %+v", v.lineAt(0), v.lineAt(1), v.lineAt(2))
	}
	v.Route = []network.EdgeID{0, 1, 9, 8} // reroute from the end of edge 1
	v.dropStaleLines()
	if v.lineAt(1).stoppedSince != 5 || v.lineAt(2).call != yellowUndecided {
		t.Fatalf("reroute: want line 1 kept and line 2 dropped, got %+v %+v", v.lineAt(1), v.lineAt(2))
	}
	v.RouteIdx, v.Edge = 1, 1
	v.StoppedSinceSec, v.WaitTime, v.YellowCall = 0, 0, yellowUndecided
	v.promoteLine()
	if v.StoppedSinceSec != 5 || len(v.linesAhead) != 0 {
		t.Fatalf("crossing the node: want line 1's stop time in the fields, got %.1f (%d memos)", v.StoppedSinceSec, len(v.linesAhead))
	}
}

// TestLookahead_FarYieldDoesNotBrakeForGap (review item 8): a yielding car
// 140 m out braked for a virtual stop whenever a priority car was due at the
// junction within a few seconds. It judges the gap only once within
// gapDecisionDist, and still stops at the line if the gap is short then.
func TestLookahead_FarYieldDoesNotBrakeForGap(t *testing.T) {
	net := closeLines(100, network.ControlYield, network.ControlNone, false)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 60, V: 10}, // 140 m from the yield line
		{ID: 2, Route: []network.EdgeID{3}, Edge: 3},                     // priority, pinned at ETA 2 s
	}
	w.nextID = 3
	stopped := false
	for i := 0; i < 600; i++ { // 30 s
		if p := vehByID(w, 2); p != nil {
			p.S, p.V = 99, 0.5
		}
		w.Step()
		v := vehByID(w, 1)
		if i == 0 && v.A < -0.05 {
			t.Fatalf("braked at %.2f m/s² for a gap 140 m away", v.A)
		}
		if v.Edge == 0 && v.V < stuckSpeedThresh && 200-v.S < stopLineTolMeters {
			stopped = true
		}
		if v.Edge != 0 && !stopped {
			t.Fatal("crossed the yield line without stopping for the imminent priority car")
		}
	}
	if !stopped {
		t.Fatal("never stopped at the yield line")
	}
}

// TestScanDetectors_JunctionBeforeApproachBlocksCall (review item 9): the
// detector walk must not reach past a junction the car has not reached yet
// to call a signal more than a stopping gap beyond it, the car could be held
// at that junction.
func TestScanDetectors_JunctionBeforeApproachBlocksCall(t *testing.T) {
	net := signalCross()
	// Split the N approach (edge 0, 200 m) 30 m before the signal, at a
	// junction node 5 (a stop-controlled crossing of some side road).
	split := network.Point{X: 0, Y: 30}
	net.Nodes = append(net.Nodes, network.Node{ID: 5, Pos: split})
	net.Edges[0].To, net.Edges[0].Length = 5, 170
	net.Edges[0].Geometry = []network.Point{net.Nodes[0].Pos, split}
	tail := net.Edges[0]
	tail.ID, tail.From, tail.To, tail.Length = 8, 5, 4, 30
	tail.Geometry = []network.Point{split, net.Nodes[4].Pos}
	net.Edges = append(net.Edges, tail)
	net.Intersections[0].Incoming[0] = 8
	net.Intersections = append(net.Intersections, network.Intersection{
		ID: 1, NodeID: 5, Incoming: []network.EdgeID{0}, IncomingControl: ctrls(network.ControlStop),
		Opposing: []int8{-1}, Outgoing: []network.EdgeID{8},
	})
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	minor := 1 - w.SignalStates[0].Config.MajorPhase
	// 10 m before the stop junction: 40 m from the signal, inside the 60 m
	// call zone along the road, but held by the stop sign first.
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 8, 6}, Edge: 0, S: 160}}
	if called, _ := w.scanDetectors(); called[0][minor] {
		t.Fatal("a car short of the stop junction called the signal 30 m beyond it")
	}
	// Past the junction, on the 30 m approach: it calls.
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 8, 6}, RouteIdx: 1, Edge: 8, S: 5}}
	if called, _ := w.scanDetectors(); !called[0][minor] {
		t.Fatal("a car on the approach did not call its phase")
	}
}
