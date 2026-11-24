package sim

import (
	"math"
	"math/rand/v2"

	"github.com/lab1702/traffic-sim/internal/network"
)

// SpawnRequest tells the World a new vehicle should appear. The World
// decides whether to honor it: requests with no route, or whose origin edge
// has no room at S=0, are dropped.
type SpawnRequest struct {
	OriginNode network.NodeID
	DestNode   network.NodeID
}

// Spawner produces SpawnRequests over time. Implementations must be
// deterministic given their constructor inputs.
type Spawner interface {
	// Tick is called every sim tick. It may return zero or more requests
	// to be attempted this tick.
	Tick(simTime float64, dt float64) []SpawnRequest
}

// RandomOD is the simplest spawner: each second (in expectation) it
// produces `rate` requests, each with a uniformly-random origin and
// destination. Origins are drawn only from nodes with an outgoing edge and
// destinations only from nodes with an incoming edge. netbuild keeps interior
// shape nodes that no edge touches; drawing from all nodes made most requests
// unroutable, so the real spawn rate was a small, map-dependent fraction of
// the requested one.
type RandomOD struct {
	rng     *rand.Rand
	rate    float64 // vehicles per second
	origins []network.NodeID
	dests   []network.NodeID
	// Accumulator tracks fractional requests carried across ticks.
	acc float64
}

// maxDestRedraws bounds how many times Tick redraws a destination that
// happens to equal the origin before dropping the request.
const maxDestRedraws = 4

// NewRandomOD builds a RandomOD. A negative, NaN or infinite rate is treated
// as 0 (no demand) rather than being allowed to overflow the per-tick count.
func NewRandomOD(net *network.Network, seed uint64, ratePerSec float64) *RandomOD {
	if !(ratePerSec > 0) || math.IsInf(ratePerSec, 1) {
		ratePerSec = 0
	}
	hasOut := make([]bool, len(net.Nodes))
	hasIn := make([]bool, len(net.Nodes))
	for i := range net.Edges {
		e := &net.Edges[i]
		if int(e.From) < len(hasOut) {
			hasOut[e.From] = true
		}
		if int(e.To) < len(hasIn) {
			hasIn[e.To] = true
		}
	}
	var origins, dests []network.NodeID
	for i := range net.Nodes {
		if hasOut[i] {
			origins = append(origins, network.NodeID(i))
		}
		if hasIn[i] {
			dests = append(dests, network.NodeID(i))
		}
	}
	return &RandomOD{
		rng:     rand.New(rand.NewPCG(seed, seed^0x9E3779B97F4A7C15)),
		rate:    ratePerSec,
		origins: origins,
		dests:   dests,
	}
}

func (s *RandomOD) Tick(_ float64, dt float64) []SpawnRequest {
	s.acc += s.rate * dt
	n := int(s.acc)
	s.acc -= float64(n)
	if n == 0 || len(s.origins) == 0 || len(s.dests) == 0 {
		return nil
	}
	out := make([]SpawnRequest, 0, n)
	for i := 0; i < n; i++ {
		o := s.origins[s.rng.IntN(len(s.origins))]
		d := s.dests[s.rng.IntN(len(s.dests))]
		for r := 0; d == o && r < maxDestRedraws; r++ {
			d = s.dests[s.rng.IntN(len(s.dests))]
		}
		if d == o {
			continue
		}
		out = append(out, SpawnRequest{OriginNode: o, DestNode: d})
	}
	return out
}
