package sim

import "github.com/lab1702/traffic-sim/internal/network"

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
// with the given incoming edges. Splits them into 2 phases (e.g.,
// NS vs EW) by alternating index, 30s green + 3s yellow each.
func DefaultSignalConfig(incoming []network.EdgeID) SignalConfig {
	if len(incoming) <= 1 {
		// 1-leg intersection: permanent green.
		return SignalConfig{Phases: []SignalPhase{{
			GreenEdges: phaseAllPositions(len(incoming)),
			GreenDur:   30, YellowDur: 0,
		}}}
	}
	var even, odd []int
	for i := range incoming {
		if i%2 == 0 {
			even = append(even, i)
		} else {
			odd = append(odd, i)
		}
	}
	return SignalConfig{Phases: []SignalPhase{
		{GreenEdges: even, GreenDur: 30, YellowDur: 3},
		{GreenEdges: odd, GreenDur: 30, YellowDur: 3},
	}}
}

func phaseAllPositions(n int) []int {
	out := make([]int, n)
	for i := range out {
		out[i] = i
	}
	return out
}
