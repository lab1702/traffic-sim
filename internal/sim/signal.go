package sim

import (
	"math"
	"sort"

	"github.com/lab1702/traffic-sim/internal/network"
)

// SignalConfig is the per-intersection plan: ordered phases that repeat.
type SignalConfig struct {
	// IntersectionID is set when this config is applied to a specific
	// intersection (used by override loading); 0 when used as a default.
	IntersectionID network.IntersectionID

	Phases []SignalPhase
}

// SignalPhase describes which approaches get green for how long, plus the
// trailing yellow before the next phase begins.
type SignalPhase struct {
	// GreenEdges is the *position* of incoming edges in
	// Intersection.Incoming that are green during this phase.
	// (Storing positions rather than EdgeIDs keeps the config relative
	// to the intersection — easier to write override files by hand.)
	GreenEdges []int
	GreenDur   float64 // seconds
	YellowDur  float64 // seconds
}

// SignalState is mutable, advanced once per tick by the World.
type SignalState struct {
	Config   SignalConfig
	PhaseIdx int
	Elapsed  float64 // seconds within current phase
	IsYellow bool
}

func NewSignalState(c SignalConfig) *SignalState {
	return &SignalState{Config: c}
}

// Advance moves the state machine forward by dt seconds.
func (s *SignalState) Advance(dt float64) {
	if len(s.Config.Phases) == 0 {
		return
	}
	s.Elapsed += dt
	for {
		p := &s.Config.Phases[s.PhaseIdx]
		threshold := p.GreenDur
		if s.IsYellow {
			threshold = p.YellowDur
		}
		if s.Elapsed < threshold {
			return
		}
		s.Elapsed -= threshold
		if !s.IsYellow {
			s.IsYellow = true
		} else {
			s.IsYellow = false
			s.PhaseIdx = (s.PhaseIdx + 1) % len(s.Config.Phases)
		}
	}
}

// GreenFor returns true if the given incoming-edge position is green or
// yellow during the current phase. Vehicles treat yellow as "go" (real
// drivers do too); they only stop on red.
func (s *SignalState) GreenFor(incomingPos int) bool {
	if len(s.Config.Phases) == 0 {
		return true // no plan == permanent green
	}
	p := s.Config.Phases[s.PhaseIdx]
	for _, e := range p.GreenEdges {
		if e == incomingPos {
			return true
		}
	}
	return false
}

// DefaultSignalConfig builds a fair fixed-time plan for an intersection
// with the given incoming edges. Approaches are grouped by their
// arrival-heading axis (heading mod π): opposing legs on the same road
// (N+S, or E+W) end up in the same phase so they get green together —
// the way real signals work. Each phase is 30 s green + 3 s yellow.
//
// 1-leg intersections (or those with no network for geometry) get a
// single permanent-green phase.
func DefaultSignalConfig(incoming []network.EdgeID, net *network.Network) SignalConfig {
	if len(incoming) <= 1 || net == nil {
		return SignalConfig{Phases: []SignalPhase{{
			GreenEdges: phaseAllPositions(len(incoming)),
			GreenDur:   30, YellowDur: 0,
		}}}
	}

	// Bucket each approach by its arrival-heading folded to [0, π).
	// Two approaches whose arrival directions differ by ~180° (opposite
	// directions on the same road) share an axis and thus a bucket.
	// 8 buckets = 22.5° resolution: tolerant of slight road misalignment,
	// strict enough to keep perpendicular approaches in different phases.
	const numBuckets = 8
	groups := make(map[int][]int)
	for j, eid := range incoming {
		h := arrivalHeading(net, eid)
		h = math.Mod(h, math.Pi)
		if h < 0 {
			h += math.Pi
		}
		b := int(h * float64(numBuckets) / math.Pi)
		if b >= numBuckets {
			b = numBuckets - 1
		}
		groups[b] = append(groups[b], j)
	}

	// Deterministic phase ordering: sort buckets ascending.
	keys := make([]int, 0, len(groups))
	for k := range groups {
		keys = append(keys, k)
	}
	sort.Ints(keys)

	phases := make([]SignalPhase, 0, len(keys))
	for _, k := range keys {
		phases = append(phases, SignalPhase{
			GreenEdges: groups[k],
			GreenDur:   30,
			YellowDur:  3,
		})
	}
	return SignalConfig{Phases: phases}
}

// arrivalHeading returns the angle (radians, math convention) of vehicle
// motion as it arrives at the downstream node of edge eid — the direction
// of the final segment of the polyline.
func arrivalHeading(net *network.Network, eid network.EdgeID) float64 {
	if int(eid) >= len(net.Edges) {
		return 0
	}
	g := net.Edges[eid].Geometry
	if len(g) < 2 {
		return 0
	}
	dx := g[len(g)-1].X - g[len(g)-2].X
	dy := g[len(g)-1].Y - g[len(g)-2].Y
	return math.Atan2(dy, dx)
}

func phaseAllPositions(n int) []int {
	out := make([]int, n)
	for i := range out {
		out[i] = i
	}
	return out
}
