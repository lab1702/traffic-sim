package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestLaneChange_OvertakesSlowLeader(t *testing.T) {
	// One edge, 1000m, 2 lanes, speed limit 20 m/s.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 1000, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 1000, SpeedLimit: 20,
			Lanes: []network.Lane{{Index: 0}, {Index: 1}}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	// Slow leader in lane 0; fast follower 30m behind, also lane 0.
	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 30, V: 20}, // follower
		{ID: 2, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 60, V: 5},  // slow leader
	}
	w.nextID = 3

	// Run 30 ticks (1.5s) — long enough to evaluate lane change.
	for i := 0; i < 30; i++ {
		w.Step()
	}

	var follower *Vehicle
	for i := range w.Vehicles {
		if w.Vehicles[i].ID == 1 {
			follower = &w.Vehicles[i]
		}
	}
	if follower == nil {
		t.Fatal("follower lost")
	}
	if follower.Lane != 1 {
		t.Errorf("follower should have changed to lane 1, still in lane %d", follower.Lane)
	}
}

// TestLaneChange_TurnBias_LeftTurn_MigratesToLeftLane verifies that a
// vehicle on a 2-lane edge whose next route step is a left turn migrates
// from lane 0 to lane 1 within the trigger range.
func TestLaneChange_TurnBias_LeftTurn_MigratesToLeftLane(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: 100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{
				{Index: 0, AllowedTurns: []network.EdgeID{99}}, // sentinel: incompatible with edge 1
				{Index: 1, AllowedTurns: []network.EdgeID{1}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
	}
	net := &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{
			{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, Outgoing: []network.EdgeID{1}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 100, V: 10},
	}
	w.nextID = 2

	for i := 0; i < 60; i++ {
		w.Step()
	}
	v := &w.Vehicles[0]
	if v.Lane != 1 {
		t.Errorf("expected lane 1 (left-compatible) after bias, got lane %d", v.Lane)
	}
}

// TestLaneChange_TurnBias_BlockedBySafetyGap verifies that turn bias does
// NOT commit when the safety gap to a neighbor lane is blocked.
func TestLaneChange_TurnBias_BlockedBySafetyGap(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
		{ID: 2, Pos: network.Point{X: 200, Y: 100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{
				{Index: 0, AllowedTurns: []network.EdgeID{99}}, // sentinel: incompatible with edge 1
				{Index: 1, AllowedTurns: []network.EdgeID{1}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
	}
	net := &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{
			{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, Outgoing: []network.EdgeID{1}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 100, V: 10},
		{ID: 2, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 1, S: 105, V: 10},
	}
	w.nextID = 3

	for i := 0; i < 5; i++ {
		w.Step()
	}
	if w.Vehicles[0].Lane != 0 {
		t.Errorf("ego should still be in lane 0 (gap blocked); got lane %d", w.Vehicles[0].Lane)
	}
}

// TestLaneChange_TurnBias_BeyondTrigger_NoChange verifies bias does not
// fire when the vehicle is more than 300 m from the intersection.
func TestLaneChange_TurnBias_BeyondTrigger_NoChange(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 1000, Y: 0}},
		{ID: 2, Pos: network.Point{X: 1000, Y: 100}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 1000, SpeedLimit: 10,
			Lanes: []network.Lane{
				{Index: 0, AllowedTurns: []network.EdgeID{99}}, // sentinel: incompatible with edge 1
				{Index: 1, AllowedTurns: []network.EdgeID{1}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
		{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 10,
			Lanes:    []network.Lane{{Index: 0}},
			Geometry: []network.Point{nodes[1].Pos, nodes[2].Pos}},
	}
	net := &network.Network{
		Nodes: nodes, Edges: edges,
		Intersections: []network.Intersection{
			{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, Outgoing: []network.EdgeID{1}},
		},
	}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0, 1}, Edge: 0, Lane: 0, S: 100, V: 10},
	}
	w.nextID = 2

	for i := 0; i < 20; i++ {
		w.Step()
	}
	if w.Vehicles[0].Lane != 0 {
		t.Errorf("bias should not fire beyond trigger; lane changed to %d", w.Vehicles[0].Lane)
	}
}

// TestLaneChange_TurnBias_LastEdge_NoFire verifies bias is a no-op when
// the current edge is the last edge of the route.
func TestLaneChange_TurnBias_LastEdge_NoFire(t *testing.T) {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 200, Y: 0}},
	}
	edges := []network.Edge{
		{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
			Lanes: []network.Lane{
				{Index: 0, AllowedTurns: nil},
				{Index: 1, AllowedTurns: []network.EdgeID{99}},
			},
			Geometry: []network.Point{nodes[0].Pos, nodes[1].Pos}},
	}
	net := &network.Network{Nodes: nodes, Edges: edges}
	w := NewWorld(net, NewRandomOD(net, 0, 0), nil)

	w.Vehicles = []Vehicle{
		{ID: 1, Route: []network.EdgeID{0}, Edge: 0, Lane: 0, S: 100, V: 10},
	}
	w.nextID = 2

	for i := 0; i < 30; i++ {
		w.Step()
		if w.Vehicles[0].Despawned {
			break
		}
	}
	if !w.Vehicles[0].Despawned && w.Vehicles[0].Lane != 0 {
		t.Errorf("bias must be a no-op on last edge; lane=%d", w.Vehicles[0].Lane)
	}
}
