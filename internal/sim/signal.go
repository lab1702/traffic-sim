package sim

import (
	"math"
	"sort"

	"github.com/lab1702/traffic-sim/internal/network"
)

// SignalMode is the operating mode of a signal. Most signals run in
// ModeNormal, cycling through their fixed-time phases. Off-hours or
// fault conditions can put a signal into a flash mode (one axis blinks
// yellow with priority; the other blinks red and must yield) or fully
// dark Off mode (treated as a 4-way stop).
type SignalMode uint8

const (
	// ModeNormal runs the configured fixed-time cycle.
	ModeNormal SignalMode = iota
	// ModeFlashA: approaches in phase 0 blink yellow (priority);
	// approaches in other phases blink red (must yield).
	ModeFlashA
	// ModeFlashB: approaches in phase 1 blink yellow (priority);
	// approaches in other phases blink red (must yield). Intended for
	// flipping which axis has priority.
	ModeFlashB
	// ModeOff: all approaches dark. Drivers treat the intersection as a
	// 4-way stop; no approach has priority.
	ModeOff
)

func (m SignalMode) String() string {
	switch m {
	case ModeNormal:
		return "normal"
	case ModeFlashA:
		return "flash_a"
	case ModeFlashB:
		return "flash_b"
	case ModeOff:
		return "off"
	default:
		return "unknown"
	}
}

// ParseSignalMode is the inverse of String; returns false for unknown values.
func ParseSignalMode(s string) (SignalMode, bool) {
	switch s {
	case "", "normal":
		return ModeNormal, true
	case "flash_a":
		return ModeFlashA, true
	case "flash_b":
		return ModeFlashB, true
	case "off":
		return ModeOff, true
	}
	return ModeNormal, false
}

// SignalConfig is the per-intersection plan: ordered phases that repeat.
type SignalConfig struct {
	// IntersectionID is set when this config is applied to a specific
	// intersection (used by override loading); 0 when used as a default.
	IntersectionID network.IntersectionID

	Phases []SignalPhase

	// InitialMode is the mode the signal starts in. Defaults to ModeNormal.
	InitialMode SignalMode
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
	Mode     SignalMode // Normal/FlashA/FlashB/Off
}

func NewSignalState(c SignalConfig) *SignalState {
	return &SignalState{Config: c, Mode: c.InitialMode}
}

// Advance moves the state machine forward by dt seconds. No-op for
// non-normal modes (flash and off are time-independent visually; the
// renderer pulses on its own wall-clock, and behavior is mode-derived).
func (s *SignalState) Advance(dt float64) {
	if s.Mode != ModeNormal {
		return
	}
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

// GreenFor returns true if the given incoming-edge position is permitted
// to proceed without first stopping/yielding.
//
//   - ModeNormal: yes iff this approach is in the current phase's GreenEdges
//     (yellow counts as green; drivers only stop on red).
//   - ModeFlashA: yes iff this approach is in phase 0 (flash yellow with priority).
//   - ModeFlashB: yes iff this approach is in phase 1 (flash yellow with priority).
//   - ModeOff:    no approach has implicit right-of-way; all approaches yield.
//     GreenFor returns false; the caller is responsible for gap-acceptance.
func (s *SignalState) GreenFor(incomingPos int) bool {
	switch s.Mode {
	case ModeNormal:
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
	case ModeFlashA:
		return phaseContains(s.Config, 0, incomingPos)
	case ModeFlashB:
		return phaseContains(s.Config, 1, incomingPos)
	case ModeOff:
		return false
	}
	return false
}

// MustYield reports whether vehicles arriving on this approach must use
// gap-acceptance (treat it like a stop sign or unsignalized side road).
// True for blinking-red approaches under FlashA/FlashB, all approaches
// under Off, and never under Normal (normal red is a hard stop, not a yield).
func (s *SignalState) MustYield(incomingPos int) bool {
	switch s.Mode {
	case ModeFlashA, ModeFlashB:
		return !s.GreenFor(incomingPos)
	case ModeOff:
		return true
	}
	return false
}

func phaseContains(c SignalConfig, phaseIdx, pos int) bool {
	if phaseIdx < 0 || phaseIdx >= len(c.Phases) {
		return false
	}
	for _, e := range c.Phases[phaseIdx].GreenEdges {
		if e == pos {
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
