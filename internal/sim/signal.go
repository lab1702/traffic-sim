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

// PlanKind selects how a signal's phases are sequenced.
type PlanKind uint8

const (
	// PlanFixed cycles phases on fixed timers via Advance. Default; the
	// behavior of every hand-built SignalConfig literal and of --signals
	// YAML overrides.
	PlanFixed PlanKind = iota
	// PlanSemiActuated rests in the major phase and serves minor phases on
	// detector demand via AdvanceActuated. Emitted by DefaultSignalConfig.
	PlanSemiActuated
)

// Semi-actuated tuning (seconds / meters). See
// docs/superpowers/specs/2026-06-05-semi-actuated-signals-design.md for the
// rationale behind each value.
const (
	actMajorMinGreen   = 15.0 // major holds >= this before yielding to a call
	actMinGreen        = 7.0  // minor-phase minimum green once served
	actPassage         = 3.0  // green ends this long after the passage zone clears
	actMaxGreen        = 40.0 // minor-phase ceiling (max-out under steady demand)
	actCallDistance    = 60.0 // a vehicle within this of the stop line calls a phase
	actPassageDistance = 25.0 // a vehicle within this holds (extends) the green
)

// SignalConfig is the per-intersection plan: ordered phases that repeat.
type SignalConfig struct {
	// IntersectionID is set when this config is applied to a specific
	// intersection (used by override loading); 0 when used as a default.
	IntersectionID network.IntersectionID

	Phases []SignalPhase

	// InitialMode is the mode the signal starts in. Defaults to ModeNormal.
	InitialMode SignalMode

	// Plan selects fixed-time (default) vs semi-actuated sequencing.
	Plan PlanKind

	// MajorPhase is the index into Phases that rests in green under
	// PlanSemiActuated (the arterial axis). Ignored for PlanFixed.
	MajorPhase int

	// Semi-actuated timings (seconds). Zero values are filled with the
	// act* defaults in NewSignalState, so an actuated config need only set
	// Plan and MajorPhase. Ignored for PlanFixed.
	MinGreen      float64 // minor-phase minimum green
	MajorMinGreen float64 // major-phase minimum green before it may yield
	Passage       float64 // gap time: green ends this long after the zone clears
	MaxGreen      float64 // minor-phase maximum green
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

	// passageGap is seconds since the current minor phase's passage zone was
	// last occupied. Used by AdvanceActuated for gap-out; reset on every
	// phase change. Unused under PlanFixed.
	passageGap float64
}

func NewSignalState(c SignalConfig) *SignalState {
	if c.Plan == PlanSemiActuated {
		if c.MinGreen == 0 {
			c.MinGreen = actMinGreen
		}
		if c.MajorMinGreen == 0 {
			c.MajorMinGreen = actMajorMinGreen
		}
		if c.Passage == 0 {
			c.Passage = actPassage
		}
		if c.MaxGreen == 0 {
			c.MaxGreen = actMaxGreen
		}
	}
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
	// Defensive guard: a phase with neither green nor yellow time is a
	// degenerate config (validation upstream should reject it, but trust
	// nothing here — the loop would spin forever on threshold=0). Belt
	// and suspenders for the runtime path.
	for safety := 0; safety < 1024; safety++ {
		p := &s.Config.Phases[s.PhaseIdx]
		threshold := p.GreenDur
		if s.IsYellow {
			threshold = p.YellowDur
		}
		if threshold <= 0 {
			// Skip this transition rather than infinite-loop. Land on
			// the next non-degenerate phase so the cycle keeps moving.
			if !s.IsYellow {
				s.IsYellow = true
			} else {
				s.IsYellow = false
				s.PhaseIdx = (s.PhaseIdx + 1) % len(s.Config.Phases)
			}
			continue
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

// AdvanceActuated advances a semi-actuated signal by dt seconds. called[i] and
// occupied[i] report whether phase i has a vehicle within the call / passage
// zone this tick — pure functions of vehicle positions, so the machine stays
// deterministic. The major phase rests in green until some minor phase is
// called; a minor phase holds for at least MinGreen, extends while a vehicle
// occupies its passage zone, and terminates on gap-out (Passage seconds after
// the zone clears) or max-out (MaxGreen). Yellow runs YellowDur, then
// nextActuatedPhase chooses the successor. Non-normal modes (flash/off) are a
// no-op, exactly like Advance.
//
// The caller (World) sizes called and occupied to len(Config.Phases); the minor
// branch indexes occupied[PhaseIdx] directly on that contract.
func (s *SignalState) AdvanceActuated(dt float64, called, occupied []bool) {
	if s.Mode != ModeNormal || len(s.Config.Phases) == 0 {
		return
	}
	s.Elapsed += dt

	if s.IsYellow {
		if s.Elapsed < s.Config.Phases[s.PhaseIdx].YellowDur {
			return
		}
		s.Elapsed = 0
		s.passageGap = 0
		s.IsYellow = false
		s.PhaseIdx = s.nextActuatedPhase(s.PhaseIdx, called)
		return
	}

	// Green, major (rest) phase.
	if s.PhaseIdx == s.Config.MajorPhase {
		if s.Elapsed < s.Config.MajorMinGreen {
			return
		}
		if anyCalledMinor(called, s.Config.MajorPhase) {
			s.toYellow()
			return
		}
		// Rest: hold green indefinitely. Pin Elapsed so it neither drifts
		// upward without bound nor max-outs (the major street has no cap).
		s.Elapsed = s.Config.MajorMinGreen
		return
	}

	// Green, minor (actuated) phase.
	if s.PhaseIdx < len(occupied) && occupied[s.PhaseIdx] {
		s.passageGap = 0
	} else {
		s.passageGap += dt
	}
	maxedOut := s.Elapsed >= s.Config.MaxGreen
	gappedOut := s.Elapsed >= s.Config.MinGreen && s.passageGap >= s.Config.Passage
	if maxedOut || gappedOut {
		s.toYellow()
	}
}

// toYellow ends the current green and starts its trailing yellow.
func (s *SignalState) toYellow() {
	s.IsYellow = true
	s.Elapsed = 0
	s.passageGap = 0
}

// nextActuatedPhase returns the phase to run after the current one ends: the
// first called minor phase in cyclic order after curr, or MajorPhase when no
// other minor phase is called (the controller returns to the major street and
// rests).
//
// The search runs off in [1, n) — it deliberately stops before n so it never
// wraps back to curr itself ((curr+n)%n == curr). A phase that just ran its
// green must yield to a conflicting movement before it may run again, even if
// it is still the only phase with a standing call: re-serving it immediately
// would blink yellow and return to the same green, starving the cross street
// (and defeating the minor phase's max-out, which exists precisely so a busy
// side street cannot hold the arterial down). When curr is the lone caller the
// loop falls through to MajorPhase, giving the major street its turn; the call
// is served again on the next cycle.
func (s *SignalState) nextActuatedPhase(curr int, called []bool) int {
	n := len(s.Config.Phases)
	for off := 1; off < n; off++ {
		p := (curr + off) % n
		if p != s.Config.MajorPhase && p < len(called) && called[p] {
			return p
		}
	}
	return s.Config.MajorPhase
}

// anyCalledMinor reports whether any phase other than major has a call.
func anyCalledMinor(called []bool, major int) bool {
	for i, c := range called {
		if i != major && c {
			return true
		}
	}
	return false
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

	// Group approaches into phases by pairing the two ends of each road that
	// continues through the junction, so both directions of a through road get
	// green together. Two legs share a phase iff they are mutually each other's
	// most-nearly-opposite approach and their arrival headings differ by more
	// than oppositeThreshold (135°). This is the same angular-tolerance test
	// netbuild.resolveOpposing uses to detect through roads.
	//
	// We pair by angular tolerance rather than snapping to fixed axis buckets
	// because real through roads bend at the junction: a T whose through legs
	// arrive at, say, 0° and 150° (a 30° kink) belongs on one axis, but fixed
	// 22.5° buckets land those legs in different buckets and split them into
	// separate phases — so the through road goes green one direction at a time
	// instead of both at once. The 135° tolerance keeps a road bending up to
	// ~45° together while still rejecting a perpendicular cross street (~90°).
	// A leg with no opposing partner (the stub of a T) gets its own phase.
	const oppositeThreshold = math.Pi - math.Pi/4 // 135°
	n := len(incoming)
	headings := make([]float64, n)
	for j, eid := range incoming {
		headings[j] = arrivalHeading(net, eid)
	}
	// best[j] is the approach most nearly opposite j, or -1 if none is more
	// than oppositeThreshold away.
	best := make([]int, n)
	for j := 0; j < n; j++ {
		best[j] = -1
		bestDelta := oppositeThreshold
		for k := 0; k < n; k++ {
			if k == j {
				continue
			}
			if d := headingSeparation(headings[j], headings[k]); d > bestDelta {
				best[j], bestDelta = k, d
			}
		}
	}
	// Each leg joins its mutual most-opposite partner, else stands alone.
	// Represent each phase by the lowest leg index in it for stable ordering.
	rep := make([]int, n)
	for j := 0; j < n; j++ {
		rep[j] = j
		if k := best[j]; k >= 0 && best[k] == j && k < j {
			rep[j] = k
		}
	}
	order := make([]int, 0, n)
	groups := make(map[int][]int)
	for j := 0; j < n; j++ {
		r := rep[j]
		if _, ok := groups[r]; !ok {
			order = append(order, r)
		}
		groups[r] = append(groups[r], j)
	}
	// Order phases by each group's folded arrival-heading axis (heading mod π),
	// ascending — the same axis ordering the previous bucket scheme used, so a
	// well-aligned cross street produces the same phase order (EW before NS).
	axis := func(r int) float64 {
		a := math.Mod(headings[r], math.Pi)
		if a < 0 {
			a += math.Pi
		}
		return a
	}
	sort.SliceStable(order, func(i, j int) bool { return axis(order[i]) < axis(order[j]) })

	phases := make([]SignalPhase, 0, len(order))
	for _, r := range order {
		phases = append(phases, SignalPhase{
			GreenEdges: groups[r],
			GreenDur:   30,
			YellowDur:  3,
		})
	}

	// A single-axis signal (one phase) has nothing to actuate and nothing to
	// stop for — it is a permanent green. Leave it PlanFixed, and clear the
	// trailing yellow: with one phase, Advance wraps phase 0 back onto itself,
	// so a non-zero YellowDur makes the approach flash yellow→green every
	// cycle for no reason (there is no conflicting movement to clear for).
	if len(phases) <= 1 {
		if len(phases) == 1 {
			phases[0].YellowDur = 0
		}
		return SignalConfig{Phases: phases}
	}

	// Multi-phase signals are semi-actuated: the arterial axis rests in green
	// and the minor phases are served on detector demand. Timings are filled
	// from the act* defaults by NewSignalState.
	return SignalConfig{
		Phases:     phases,
		Plan:       PlanSemiActuated,
		MajorPhase: majorPhase(phases, incoming, net),
	}
}

// majorPhase picks the phase that rests in green: the axis carrying the
// highest-priority road, scored by max RoadClass.Priority() over the phase's
// green approaches, tie-broken by highest SpeedLimit, then lowest phase index.
func majorPhase(phases []SignalPhase, incoming []network.EdgeID, net *network.Network) int {
	best, bestPri, bestSpd := 0, -1, -1.0
	for i, p := range phases {
		pri, spd := -1, -1.0
		for _, pos := range p.GreenEdges {
			if pos < 0 || pos >= len(incoming) {
				continue
			}
			e := &net.Edges[incoming[pos]]
			if pr := e.Class.Priority(); pr > pri {
				pri = pr
			}
			if e.SpeedLimit > spd {
				spd = e.SpeedLimit
			}
		}
		if pri > bestPri || (pri == bestPri && spd > bestSpd) {
			best, bestPri, bestSpd = i, pri, spd
		}
	}
	return best
}

// arrivalHeading wraps network.ArrivalHeading for backwards compatibility
// inside this package; identical behavior, kept as an unexported helper
// so the call sites read tersely.
func arrivalHeading(net *network.Network, eid network.EdgeID) float64 {
	return network.ArrivalHeading(net, eid)
}

// headingSeparation returns the absolute angular separation between two
// headings (radians), normalized to [0, π]. 0 = same direction, π = exactly
// opposite (two ends of a straight road continuing through a junction).
func headingSeparation(a, b float64) float64 {
	d := math.Mod(math.Abs(a-b), 2*math.Pi)
	if d > math.Pi {
		d = 2*math.Pi - d
	}
	return d
}

func phaseAllPositions(n int) []int {
	out := make([]int, n)
	for i := range out {
		out[i] = i
	}
	return out
}
