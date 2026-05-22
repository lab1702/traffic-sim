package sim

import (
	"math"
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
)

// congTestNet: two edges, 100m each, 10 and 20 m/s limits.
func congTestNet() *network.Network {
	return &network.Network{
		Edges: []network.Edge{
			{ID: 0, From: 0, To: 1, Length: 100, SpeedLimit: 10},
			{ID: 1, From: 1, To: 2, Length: 100, SpeedLimit: 20},
		},
	}
}

func TestCongestion_EmptyEdgeFreeFlow(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	// An edge nobody is on costs free-flow travel time.
	got := c.Cost(net, 0)
	want := 100.0 / 10.0
	if math.Abs(got-want) > 1e-9 {
		t.Fatalf("empty-edge cost = %v, want %v", got, want)
	}
	// And stays there after an update with no vehicles on it.
	c.Update(net, map[network.EdgeID][]int{}, nil)
	if got := c.Cost(net, 0); math.Abs(got-want) > 1e-9 {
		t.Fatalf("empty-edge cost after update = %v, want %v", got, want)
	}
}

func TestCongestion_JamRaisesCost(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	free := c.Cost(net, 0)
	vehicles := []Vehicle{{Edge: 0, V: 0}, {Edge: 0, V: 0}}
	byEdge := map[network.EdgeID][]int{0: {0, 1}}
	for i := 0; i < 2000; i++ {
		c.Update(net, byEdge, vehicles)
	}
	jammed := c.Cost(net, 0)
	if jammed <= free {
		t.Fatalf("jammed cost %v should exceed free-flow cost %v", jammed, free)
	}
	// Smoothed speed converges toward 0; Cost floors at length/minEdgeSpeed.
	want := 100.0 / minEdgeSpeed
	if math.Abs(jammed-want) > 1.0 {
		t.Fatalf("jammed cost %v, want ~%v", jammed, want)
	}
}

func TestCongestion_EWMASmoothing(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	vehicles := []Vehicle{{Edge: 0, V: 0}}
	byEdge := map[network.EdgeID][]int{0: {0}}
	before := c.speed[0]
	c.Update(net, byEdge, vehicles)
	after := c.speed[0]
	want := before + c.alpha*(0-before)
	if math.Abs(after-want) > 1e-9 {
		t.Fatalf("after one update speed = %v, want %v (alpha=%v)", after, want, c.alpha)
	}
	// A single-tick dip must not collapse the speed (smoothing, not snap).
	if after < before*0.5 {
		t.Fatalf("single-tick dip over-moved: %v from %v", after, before)
	}
}

func TestCongestion_CostClamps(t *testing.T) {
	net := congTestNet()
	c := NewCongestion(net, ewmaHalfLifeSec, DefaultDt)
	// Floor: even at speed 0 the cost is finite, not +Inf.
	c.speed[0] = 0
	floored := c.Cost(net, 0)
	if math.IsInf(floored, 1) || math.Abs(floored-100.0/minEdgeSpeed) > 1e-9 {
		t.Fatalf("floor cost = %v, want %v", floored, 100.0/minEdgeSpeed)
	}
	// Ceiling: observed faster than free-flow is clamped to free-flow cost.
	c.speed[0] = 100 // way above the 10 m/s limit
	ceiled := c.Cost(net, 0)
	if math.Abs(ceiled-100.0/10.0) > 1e-9 {
		t.Fatalf("ceiling cost = %v, want %v (free-flow)", ceiled, 100.0/10.0)
	}
}
