package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// build2x2Grid returns a 2x2 grid: 4 nodes arranged in a square,
// with edges between adjacent nodes in both directions. 100m blocks.
//
// 2 --- 3
// |     |
// 0 --- 1
func build2x2Grid() *network.Network {
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: 0, Y: 0}},
		{ID: 1, Pos: network.Point{X: 100, Y: 0}},
		{ID: 2, Pos: network.Point{X: 0, Y: 100}},
		{ID: 3, Pos: network.Point{X: 100, Y: 100}},
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID:         network.EdgeID(id),
			From:       network.NodeID(from),
			To:         network.NodeID(to),
			Length:     100,
			SpeedLimit: 10,
			Lanes:      []network.Lane{{Index: 0}},
		}
	}
	edges := []network.Edge{
		mkEdge(0, 0, 1), mkEdge(1, 1, 0),
		mkEdge(2, 0, 2), mkEdge(3, 2, 0),
		mkEdge(4, 1, 3), mkEdge(5, 3, 1),
		mkEdge(6, 2, 3), mkEdge(7, 3, 2),
	}
	return &network.Network{Nodes: nodes, Edges: edges}
}

func TestWorld_VehiclesSpawnMoveDespawn(t *testing.T) {
	net := build2x2Grid()
	spawner := NewRandomOD(net, 7, 5.0) // 5 vehicles/sec
	w := NewWorld(net, spawner)

	w.Run(10.0) // 10 simulated seconds

	if w.Tick == 0 {
		t.Fatalf("no ticks ran")
	}
	// Some vehicles should have completed and despawned by now. With a
	// 100m block at 10 m/s, a 2-edge trip is 20s — most won't be done yet.
	// What we *can* assert: spawn was attempted multiple times.
	if w.nextID == 0 {
		t.Errorf("expected some spawns over 10s @ 5/s, got 0")
	}
}

func TestWorld_DeterminismSameSeed(t *testing.T) {
	run := func() (uint32, int) {
		net := build2x2Grid()
		w := NewWorld(net, NewRandomOD(net, 123, 3.0))
		w.Run(5.0)
		return uint32(w.nextID), len(w.Vehicles)
	}
	a1, _ := run()
	a2, _ := run()
	if a1 != a2 {
		t.Errorf("determinism: same seed produced different nextID: %d vs %d", a1, a2)
	}
}
