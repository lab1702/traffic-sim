package sim

import (
	"math"

	"github.com/lab1702/traffic-sim/internal/network"
)

const (
	// minEdgeSpeed floors the speed used for routing cost. A jammed edge is
	// expensive but never infinite-cost, so it stays selectable when it is the
	// only path through (dead ends, single bridges). 0.5 m/s ~ 1.8 km/h.
	minEdgeSpeed = 0.5

	// ewmaHalfLifeSec is the half-life of the per-edge speed EWMA: the time for
	// a step change in observed speed to be half-reflected in the smoothed
	// value. 10s rejects single-vehicle transients (one car braking) while
	// tracking real jams that form/clear over tens of seconds.
	ewmaHalfLifeSec = 10.0
)

// Congestion tracks an EWMA-smoothed average speed per edge, refreshed once per
// tick from the vehicles currently on each edge, and converts those speeds into
// routing costs (free-flow travel time, inflated by congestion).
type Congestion struct {
	speed []float64 // smoothed observed speed (m/s), indexed by EdgeID
	alpha float64   // EWMA blend factor in (0,1], derived from half-life and dt
}

// NewCongestion allocates a per-edge speed table seeded at each edge's
// free-flow speed. alpha is derived from the EWMA half-life and tick length:
// alpha = 1 - 2^(-dt/halfLife).
func NewCongestion(net *network.Network, halfLifeSec, dt float64) *Congestion {
	speed := make([]float64, len(net.Edges))
	for i := range net.Edges {
		speed[i] = net.Edges[i].SpeedLimit
	}
	alpha := 1.0
	if halfLifeSec > 0 {
		alpha = 1 - math.Exp(-math.Ln2*dt/halfLifeSec)
	}
	return &Congestion{speed: speed, alpha: alpha}
}

// Update blends each edge's smoothed speed toward this tick's observed mean
// speed (arithmetic mean of vehicle speeds on the edge — order-independent),
// or toward free-flow when the edge is empty. byEdge maps EdgeID to indices
// into vehicles.
func (c *Congestion) Update(net *network.Network, byEdge map[network.EdgeID][]int, vehicles []Vehicle) {
	for eid := range c.speed {
		var target float64
		idxs := byEdge[network.EdgeID(eid)]
		if len(idxs) > 0 {
			sum := 0.0
			for _, vi := range idxs {
				sum += vehicles[vi].V
			}
			target = sum / float64(len(idxs))
		} else {
			target = net.Edges[eid].SpeedLimit
		}
		c.speed[eid] += c.alpha * (target - c.speed[eid])
	}
}

// Cost returns the routing cost (travel time) for an edge: its length divided
// by the smoothed observed speed, clamped to [minEdgeSpeed, freeFlowSpeed].
// The free-flow ceiling keeps the router's A* heuristic admissible (congestion
// only makes edges slower than free-flow, never faster).
func (c *Congestion) Cost(net *network.Network, eid network.EdgeID) float64 {
	e := &net.Edges[eid]
	freeFlow := e.SpeedLimit
	if freeFlow < minEdgeSpeed {
		freeFlow = minEdgeSpeed
	}
	s := c.speed[eid]
	if s < minEdgeSpeed {
		s = minEdgeSpeed
	}
	if s > freeFlow {
		s = freeFlow
	}
	return e.Length / s
}
