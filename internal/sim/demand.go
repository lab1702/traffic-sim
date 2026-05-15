package sim

import (
	"math/rand/v2"

	"github.com/lab1702/traffic-sim/internal/network"
)

// SpawnRequest tells the World a new vehicle should appear. The World
// decides whether to honor it (e.g., it may delay if the origin edge is
// blocked at S=0).
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
// destination drawn from all network nodes.
type RandomOD struct {
	net  *network.Network
	rng  *rand.Rand
	rate float64 // vehicles per second
	// Accumulator tracks fractional requests carried across ticks.
	acc float64
}

func NewRandomOD(net *network.Network, seed uint64, ratePerSec float64) *RandomOD {
	return &RandomOD{
		net:  net,
		rng:  rand.New(rand.NewPCG(seed, seed^0x9E3779B97F4A7C15)),
		rate: ratePerSec,
	}
}

func (s *RandomOD) Tick(_ float64, dt float64) []SpawnRequest {
	s.acc += s.rate * dt
	n := int(s.acc)
	s.acc -= float64(n)
	if n == 0 || len(s.net.Nodes) < 2 {
		return nil
	}
	out := make([]SpawnRequest, 0, n)
	for i := 0; i < n; i++ {
		oi := s.rng.IntN(len(s.net.Nodes))
		di := s.rng.IntN(len(s.net.Nodes))
		if oi == di {
			continue
		}
		out = append(out, SpawnRequest{
			OriginNode: network.NodeID(oi),
			DestNode:   network.NodeID(di),
		})
	}
	return out
}
