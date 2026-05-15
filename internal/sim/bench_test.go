package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// buildGridN builds an N x N grid of intersections with 100m blocks.
// Resulting graph has N*N nodes, ~4*N*(N-1) directed edges.
func buildGridN(n int) *network.Network {
	idx := func(i, j int) network.NodeID { return network.NodeID(i*n + j) }
	var nodes []network.Node
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			nodes = append(nodes, network.Node{
				ID:  idx(i, j),
				Pos: network.Point{X: float64(j) * 100, Y: float64(i) * 100},
			})
		}
	}
	var edges []network.Edge
	mkEdge := func(from, to network.NodeID) {
		fromPos := nodes[from].Pos
		toPos := nodes[to].Pos
		edges = append(edges, network.Edge{
			ID: network.EdgeID(len(edges)), From: from, To: to, Length: 100,
			SpeedLimit: 13.4,
			Lanes:      []network.Lane{{Index: 0}, {Index: 1}},
			Geometry:   []network.Point{fromPos, toPos},
		})
	}
	for i := 0; i < n; i++ {
		for j := 0; j < n; j++ {
			if j+1 < n {
				mkEdge(idx(i, j), idx(i, j+1))
				mkEdge(idx(i, j+1), idx(i, j))
			}
			if i+1 < n {
				mkEdge(idx(i, j), idx(i+1, j))
				mkEdge(idx(i+1, j), idx(i, j))
			}
		}
	}
	return &network.Network{
		Nodes: nodes, Edges: edges,
		Bounds: network.BoundingBox{
			MinX: 0, MinY: 0, MaxX: float64(n) * 100, MaxY: float64(n) * 100,
		},
	}
}

func benchmarkTickN(b *testing.B, vehicleCount int) {
	net := buildGridN(40) // 1600 intersections
	w := NewWorld(net, NewRandomOD(net, 1, 0), nil)
	// Pre-seed N vehicles by issuing spawn requests until we have enough.
	for len(w.Vehicles) < vehicleCount {
		w.trySpawn(SpawnRequest{
			OriginNode: network.NodeID(len(w.Vehicles) % len(net.Nodes)),
			DestNode:   network.NodeID((len(w.Vehicles)*7 + 3) % len(net.Nodes)),
		})
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		w.Step()
	}
}

func BenchmarkTick_1k(b *testing.B)  { benchmarkTickN(b, 1000) }
func BenchmarkTick_5k(b *testing.B)  { benchmarkTickN(b, 5000) }
func BenchmarkTick_10k(b *testing.B) { benchmarkTickN(b, 10000) }
