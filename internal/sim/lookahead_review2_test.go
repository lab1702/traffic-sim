package sim

import (
	"fmt"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Regression tests from the second review of the route lookahead (8e7a3fd):
// each scenario regressed there against 382d05f, or was cheap to fix.

// dualCarriageway builds a minor road crossing a dual carriageway: X1 (node
// 1) on the eastbound carriageway and X2 (node 2) on the westbound one, gap
// metres apart. Edges: 0 S->X1, 1 X1->X2, 2 X2->N (northbound); 3 N->X2,
// 4 X2->X1, 5 X1->S (southbound); 6 W->X1, 7 X1->E (eastbound); 8 E->X2,
// 9 X2->W (westbound). Every approach at both nodes gets control c; with
// signals both nodes are signalized instead.
func dualCarriageway(gap float64, c network.Control, signals bool, minor, major network.RoadClass) *network.Network {
	pts := []network.Point{
		{X: 0, Y: -200}, {X: 0, Y: 0}, {X: 0, Y: gap}, {X: 0, Y: gap + 200},
		{X: -200, Y: 0}, {X: 200, Y: 0}, {X: 200, Y: gap}, {X: -200, Y: gap},
	}
	nodes := make([]network.Node, len(pts))
	for i, p := range pts {
		nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
	}
	mk := func(id, from, to int, length float64, cls network.RoadClass) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: length, SpeedLimit: 10, Class: cls, Lanes: []network.Lane{{Index: 0}},
			Geometry: []network.Point{pts[from], pts[to]},
		}
	}
	edges := []network.Edge{
		mk(0, 0, 1, 200, minor), mk(1, 1, 2, gap, minor), mk(2, 2, 3, 200, minor),
		mk(3, 3, 2, 200, minor), mk(4, 2, 1, gap, minor), mk(5, 1, 0, 200, minor),
		mk(6, 4, 1, 200, major), mk(7, 1, 5, 200, major),
		mk(8, 6, 2, 200, major), mk(9, 2, 7, 200, major),
	}
	xs := []network.Intersection{
		{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0, 4, 6}, IncomingControl: ctrls(c, c, c),
			Opposing: []int8{1, 0, -1}, Outgoing: []network.EdgeID{1, 5, 7}, HasSignal: signals},
		{ID: 1, NodeID: 2, Incoming: []network.EdgeID{1, 3, 8}, IncomingControl: ctrls(c, c, c),
			Opposing: []int8{1, 0, -1}, Outgoing: []network.EdgeID{2, 4, 9}, HasSignal: signals},
	}
	return &network.Network{Nodes: nodes, Edges: edges, Intersections: xs}
}

// runToEnds steps w until every vehicle has reached the last edge of its
// route (or finished it), and fails if one disappears before that (a stuck
// despawn) or the run times out.
func runToEnds(t *testing.T, w *World, ticks int) {
	t.Helper()
	done := map[VehicleID]bool{}
	ids := []VehicleID{}
	for i := range w.Vehicles {
		ids = append(ids, w.Vehicles[i].ID)
	}
	for i := 0; i < ticks; i++ {
		w.Step()
		left := 0
		for _, id := range ids {
			if done[id] {
				continue
			}
			v := vehByID(w, id)
			if v == nil {
				t.Fatalf("tick %d: vehicle %d despawned before reaching the end of its route", i, id)
			}
			if v.RouteIdx == len(v.Route)-1 {
				done[id] = true
				continue
			}
			left++
		}
		if left == 0 {
			return
		}
	}
	for _, id := range ids {
		if v := vehByID(w, id); !done[id] && v != nil {
			t.Errorf("vehicle %d still waiting: edge %d S %.2f V %.2f StuckTime %.1f", id, v.Edge, v.S, v.V, v.StuckTime)
		}
	}
	t.Fatalf("not every vehicle got through within %d ticks", ticks)
}

// TestLookahead_AllWayStopDualCarriagewayNoDeadlock (second review item 1):
// with all-way stops (or signals off) at both carriageway nodes under ~7 m
// apart, a northbound car waits for X2 while still short of X1 and a
// southbound car waits for X1 while still short of X2. Each kept its FIFO
// slot at the line it had already been released from, so each waited for
// the other for ever, and both being at a line hid them from the stuck
// guard.
func TestLookahead_AllWayStopDualCarriagewayNoDeadlock(t *testing.T) {
	res := network.ClassResidential
	for _, signals := range []bool{false, true} {
		for _, gap := range []float64{4, 5, 6} {
			for _, off := range []float64{-40, -25, -12, -6, 0, 6, 12} {
				name := fmt.Sprintf("signals=%v gap=%gm offset=%gm", signals, gap, off)
				t.Run(name, func(t *testing.T) {
					net := dualCarriageway(gap, network.ControlAllWayStop, signals, res, res)
					w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
					if signals {
						w.SignalStates[0].Mode, w.SignalStates[1].Mode = ModeOff, ModeOff
					}
					w.Vehicles = []Vehicle{
						{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 150, V: 10},       // A, northbound
						{ID: 2, Route: []network.EdgeID{3, 4, 5}, Edge: 3, S: 150 + off, V: 10}, // D, southbound
					}
					w.nextID = 3
					runToEnds(t, w, 2400) // 120 s
				})
			}
		}
		t.Run(fmt.Sprintf("signals=%v four cars", signals), func(t *testing.T) {
			net := dualCarriageway(5, network.ControlAllWayStop, signals, res, res)
			w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
			if signals {
				w.SignalStates[0].Mode, w.SignalStates[1].Mode = ModeOff, ModeOff
			}
			w.Vehicles = []Vehicle{
				{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 150, V: 10}, // A, northbound
				{ID: 2, Route: []network.EdgeID{3, 4, 5}, Edge: 3, S: 152, V: 10}, // D, southbound
				{ID: 3, Route: []network.EdgeID{6, 7}, Edge: 6, S: 150, V: 10},    // eastbound on X1's carriageway
				{ID: 4, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 130, V: 10}, // second northbound
			}
			w.nextID = 5
			runToEnds(t, w, 6000) // 300 s
		})
	}
}

// TestWorld_StuckGuardSparesQueueBehindBlockedYield (second review item 2):
// cars queued on the edge that ends at a yield or left-turn line whose gap
// stays shut are waiting legitimately, as at 382d05f, however far back they
// are. The gap is only judged within gapDecisionDist (50 m for a stopped
// car), so the cars beyond it had no obligation and were despawned as stuck.
func TestWorld_StuckGuardSparesQueueBehindBlockedYield(t *testing.T) {
	mkNet := func(leftTurn bool) *network.Network {
		// Approach (edge 0) from the south, 300 m, into X (node 1). Exit
		// edge 1: north (yield case) or west (left-turn case). Priority /
		// opposing traffic on edge 2 from the north, leaving on edge 3.
		exit := network.Point{X: 0, Y: 100}
		if leftTurn {
			exit = network.Point{X: -100, Y: 0}
		}
		pts := []network.Point{{X: 0, Y: -300}, {X: 0, Y: 0}, exit, {X: 0, Y: 100}, {X: 100, Y: 0}}
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
		edges := []network.Edge{mk(0, 0, 1, 300), mk(1, 1, 2, 100)}
		x := network.Intersection{ID: 0, NodeID: 1}
		if leftTurn {
			// Opposing southbound traffic, continuing south past ego's origin.
			edges = append(edges, mk(2, 3, 1, 100), mk(3, 1, 0, 300))
			x.Incoming, x.IncomingControl, x.Opposing = []network.EdgeID{0, 2}, allNone(2), []int8{1, 0}
			x.Outgoing = []network.EdgeID{1, 3}
		} else {
			// Priority traffic from the east, continuing west... onto the
			// exit's node is not needed: it only has to approach X.
			edges = append(edges, mk(2, 4, 1, 100))
			x.Incoming, x.IncomingControl, x.Opposing = []network.EdgeID{0, 2},
				ctrls(network.ControlYield, network.ControlNone), []int8{-1, -1}
			x.Outgoing = []network.EdgeID{1}
		}
		return &network.Network{Nodes: nodes, Edges: edges, Intersections: []network.Intersection{x}}
	}
	for _, leftTurn := range []bool{false, true} {
		t.Run(map[bool]string{false: "yield", true: "left turn"}[leftTurn], func(t *testing.T) {
			net := mkNet(leftTurn)
			w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
			const n = 15
			for k := 0; k < n; k++ {
				w.Vehicles = append(w.Vehicles, Vehicle{ID: VehicleID(k + 1), Route: []network.EdgeID{0, 1}, Edge: 0, S: 293 - 7*float64(k)})
			}
			prioRoute := []network.EdgeID{2}
			if leftTurn {
				prioRoute = []network.EdgeID{2, 3}
			}
			w.Vehicles = append(w.Vehicles, Vehicle{ID: 100, Route: prioRoute, Edge: 2})
			w.nextID = 101
			for i := 0; i < 4000; i++ { // 200 s
				if p := vehByID(w, 100); p != nil {
					p.S, p.V = 99.5, 0.5 // ETA 1 s: below minAcceptedGap, never accepted
				}
				w.Step()
			}
			for k := 0; k < n; k++ {
				if v := vehByID(w, VehicleID(k+1)); v == nil {
					t.Errorf("queued car %d (started %.0f m from the line) was despawned as stuck", k+1, 7+7*float64(k))
				}
			}
		})
	}
}

// TestScanDetectors_CarHeldAtFirstSignalDoesNotHoldSecondGreen (second
// review item 3): a car held by the first of two signals 5 m apart kept
// calling, and occupying the passage zone of, the second signal's minor
// phase, which stayed green to max-out for a car that could not use it.
func TestScanDetectors_CarHeldAtFirstSignalDoesNotHoldSecondGreen(t *testing.T) {
	net := dualCarriageway(5, network.ControlNone, true, network.ClassResidential, network.ClassPrimary)
	// S1 is red for the minor road for the whole run; S2 is semi-actuated.
	w := NewWorld(net, NewRandomOD(net, 0, 0), map[network.IntersectionID]SignalConfig{
		0: {Phases: []SignalPhase{{GreenEdges: []int{2}, GreenDur: 1e6, YellowDur: 3}}},
	})
	s2 := w.SignalStates[1]
	if s2.Config.Plan != PlanSemiActuated || s2.GreenFor(0) {
		t.Fatalf("fixture: S2 should be semi-actuated and resting red for the minor road")
	}
	w.Vehicles = []Vehicle{{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 100, V: 10}}
	w.nextID = 2
	green := 0.0
	for i := 0; i < 2400; i++ { // 120 s
		w.Step()
		if s2.GreenFor(0) && !s2.IsYellow {
			green += w.dt
		}
	}
	if v := vehByID(w, 1); v == nil || v.Edge != 0 {
		t.Fatalf("fixture: the car should still be held by S1's red")
	}
	if green > 1 {
		t.Fatalf("S2's minor phase was green for %.1f s for a car held at S1", green)
	}
}

// TestSpawn_FollowerCheckFindsEveryPath (second review item 4): a follower
// must be checked by its own route, whichever path the upstream search
// reached its edge by, and a car about to U-turn onto the spawn edge counts
// too.
func TestSpawn_FollowerCheckFindsEveryPath(t *testing.T) {
	t.Run("second path", func(t *testing.T) {
		// P -> O directly (60 m), and P -> M -> O (3 m + 3 m). The follower
		// is 5 m before P, so 11 m from O by the short path it will take.
		pts := []network.Point{{X: -6}, {X: 0}, {X: -3, Y: 1}, {X: -106}, {X: 100}}
		nodes := make([]network.Node, len(pts))
		for i, p := range pts {
			nodes[i] = network.Node{ID: network.NodeID(i), Pos: p}
		}
		mk := func(id, from, to int, length float64) network.Edge {
			return network.Edge{
				ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
				Length: length, SpeedLimit: 13.4, Lanes: []network.Lane{{Index: 0}},
				Geometry: []network.Point{pts[from], pts[to]},
			}
		}
		const P, O, M, Q, D = 0, 1, 2, 3, 4
		net := &network.Network{Nodes: nodes, Edges: []network.Edge{
			mk(0, P, O, 60), mk(1, P, M, 3), mk(2, M, O, 3), mk(3, Q, P, 100), mk(4, O, D, 100),
		}}
		w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
		w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{3, 1, 2, 4}, Edge: 3, S: 95, V: 13.4}}
		w.trySpawn(SpawnRequest{OriginNode: O, DestNode: D}, w.buildSpawnOccupancy())
		if len(w.Vehicles) != 1 {
			t.Fatalf("spawned with a car 11 m behind the node at 13.4 m/s")
		}
	})
	t.Run("u-turn onto spawn edge", func(t *testing.T) {
		// Dead end: the only way onto the spawn edge O->D is a U-turn off
		// its twin D->O, where a car is 3 m from the end.
		pts := []network.Point{{X: 0}, {X: 100}}
		net := &network.Network{
			Nodes: []network.Node{{ID: 0, Pos: pts[0]}, {ID: 1, Pos: pts[1]}},
			Edges: []network.Edge{
				{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 13.4, Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{pts[0], pts[1]}},
				{ID: 1, From: 1, To: 0, Length: 100, SpeedLimit: 13.4, Lanes: []network.Lane{{Index: 0}}, Geometry: []network.Point{pts[1], pts[0]}},
			},
		}
		w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
		w.Vehicles = []Vehicle{{ID: 100, Route: []network.EdgeID{1, 0}, Edge: 1, S: 97, V: 13.4}}
		w.trySpawn(SpawnRequest{OriginNode: 0, DestNode: 1}, w.buildSpawnOccupancy())
		if len(w.Vehicles) != 1 {
			t.Fatalf("spawned in front of a car about to U-turn onto the spawn edge")
		}
	})
}

// TestLookahead_FartherYellowMemoClearedOnGreen (second review nit 6): while
// a nearer line holds the car, the walk never reaches a farther signal, so
// its green must still clear a remembered yellow decision there; otherwise
// an old "go" is reused on the next yellow.
func TestLookahead_FartherYellowMemoClearedOnGreen(t *testing.T) {
	net := closeLines(10, network.ControlStop, network.ControlNone, false)
	net.Intersections[1].HasSignal = true
	w := NewWorld(net, NewRandomOD(net, 0, 0), map[network.IntersectionID]SignalConfig{
		1: {Phases: []SignalPhase{{GreenEdges: []int{0}, GreenDur: 1e6, YellowDur: 3}, {GreenEdges: []int{1}, GreenDur: 30}}},
	})
	v := &Vehicle{Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 150, V: 10}
	v.setLineAt(1, lineState{call: yellowGo}) // left over from an earlier yellow
	if s := w.nextStop(v, -1); !s.yield || s.j != 0 {
		t.Fatalf("fixture: the stop line at the end of edge 0 should hold the car, got %+v", s)
	}
	if c := v.lineAt(1).call; c != yellowUndecided {
		t.Fatalf("green at the farther signal left its yellow decision at %d", c)
	}
}

// TestWorld_StuckGuardSeesWaitPastNearerJunction (second review item 1): a
// car waiting at a line it stops short of a nearer controlled junction for
// is not exempt from the stuck guard, so a wait that never resolves there
// (two cars waiting on each other across close junctions, or here a gap
// that never opens) is cleared rather than frozen for ever.
func TestWorld_StuckGuardSeesWaitPastNearerJunction(t *testing.T) {
	net := closeLines(4, network.ControlStop, network.ControlYield, false)
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1, 2}, Edge: 0, S: 120, V: 10},
		{ID: 2, Route: []network.EdgeID{4}, Edge: 4}, // priority at line 2, pinned
	}
	w.nextID = 3
	for i := 0; i < 2400; i++ { // 120 s
		if p := vehByID(w, 2); p != nil {
			p.S, p.V = 99.5, 0.5 // ETA 1 s: never accepted
		}
		w.Step()
		v := vehByID(w, 1)
		if v == nil {
			return // cleared by the stuck guard
		}
		if v.Edge == 2 {
			t.Fatal("fixture: the car should never get a gap at line 2")
		}
	}
	v := vehByID(w, 1)
	t.Fatalf("car frozen past the stop junction for 120 s: edge %d S %.2f StuckTime %.1f", v.Edge, v.S, v.StuckTime)
}
