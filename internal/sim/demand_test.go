package sim

import (
	"math"
	"reflect"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

func TestRandomOD_DeterministicForSeed(t *testing.T) {
	net := buildLineGraph()
	s1 := NewRandomOD(net, 42, 5.0)
	s2 := NewRandomOD(net, 42, 5.0)
	var r1, r2 []SpawnRequest
	for i := 0; i < 20; i++ {
		r1 = append(r1, s1.Tick(float64(i)*0.05, 0.05)...)
		r2 = append(r2, s2.Tick(float64(i)*0.05, 0.05)...)
	}
	if !reflect.DeepEqual(r1, r2) {
		t.Fatalf("same seed should produce identical request streams\n r1=%v\n r2=%v", r1, r2)
	}
	if len(r1) == 0 {
		t.Errorf("expected at least one spawn request over 1s @ 5/s")
	}
}

// TestRandomOD_SkipsNodesWithoutEdges: interior shape nodes have no edges and
// can never be routed to or from. They must not be drawn as origins or
// destinations, and origins/destinations must have outgoing/incoming edges.
func TestRandomOD_SkipsNodesWithoutEdges(t *testing.T) {
	net := buildLineGraph() // edges 0->1->2->3: node 0 has no incoming, 3 no outgoing
	for i := 4; i < 40; i++ {
		net.Nodes = append(net.Nodes, network.Node{ID: network.NodeID(i)}) // edge-less shape nodes
	}
	s := NewRandomOD(net, 1, 20)
	got := 0
	for i := 0; i < 200; i++ {
		for _, r := range s.Tick(0, 0.05) {
			got++
			if r.OriginNode > 2 {
				t.Fatalf("origin %d has no outgoing edge", r.OriginNode)
			}
			if r.DestNode == 0 || r.DestNode > 3 {
				t.Fatalf("destination %d has no incoming edge", r.DestNode)
			}
		}
	}
	// 20/s for 10 s; only same-node draws that survive the redraws are lost.
	if got < 190 {
		t.Fatalf("got %d requests for 200 expected; too many dropped", got)
	}
}

// TestRandomOD_BadRateIsZero: a negative or NaN rate must not panic (it used
// to overflow the per-tick slice capacity) and produces no requests.
func TestRandomOD_BadRateIsZero(t *testing.T) {
	net := buildLineGraph()
	for _, rate := range []float64{-5, math.NaN(), math.Inf(1)} {
		s := NewRandomOD(net, 1, rate)
		for i := 0; i < 100; i++ {
			if reqs := s.Tick(0, 0.05); len(reqs) != 0 {
				t.Fatalf("rate %v produced %d requests", rate, len(reqs))
			}
		}
	}
}
