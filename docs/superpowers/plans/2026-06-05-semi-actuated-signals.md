# Semi-Actuated Traffic Signals — Implementation Plan

**Goal:** Make auto-generated signals semi-actuated — the major street rests in
green; minor phases are detector-actuated (min-green, passage gap-out,
max-green). Fixed-time `SignalConfig` literals and `--signals` overrides keep
today's behavior.

**Spec:** `docs/superpowers/specs/2026-06-05-semi-actuated-signals-design.md`

**Architecture:** `SignalConfig` gains a `Plan` kind + actuation fields.
`AdvanceActuated(dt, called, occupied)` is a new pure state-machine method
alongside the unchanged `Advance(dt)`. `DefaultSignalConfig` returns a
`PlanSemiActuated` config (picking the arterial axis as `MajorPhase`).
`World.Step` precomputes per-phase detector occupancy from vehicle positions and
dispatches on `Plan`. No trace-format, renderer, or vehicle-decision changes.

**Tech Stack:** Go; `internal/network`, `internal/sim`. Tests via
`go test ./internal/sim/`.

**Determinism:** detection is a pure function of vehicle positions, so
`TestWorld_TraceDeterminism` holds. Existing fixed-time tests use `SignalConfig`
literals (`Plan` zero-value `PlanFixed`) and are unaffected.

---

## File Structure

- `internal/sim/signal.go` — add `PlanKind`, `SignalConfig` fields, `passageGap`
  on `SignalState`, `AdvanceActuated`, `nextActuatedPhase`, actuation constants;
  rewrite the tail of `DefaultSignalConfig`.
- `internal/sim/world.go` — precompute `detectorEdges` in `NewWorld`; per-tick
  detector scan + `Plan` dispatch in step 0b.
- `internal/sim/signal_test.go` — actuated state-machine + `DefaultSignalConfig`
  tests.
- `internal/sim/world_test.go` — arterial-rests / side-street-served integration
  tests.

Tasks 1–2 are additive (Task 1 adds the actuated machine; Task 2 flips
`DefaultSignalConfig` to emit it). Task 3 wires detection into the tick.

---

## Task 1: Actuation config + state machine (`signal.go`)

Add to `signal.go`:

```go
// PlanKind selects how a signal's phases are sequenced.
type PlanKind uint8

const (
	// PlanFixed cycles phases on fixed timers (Advance). Default.
	PlanFixed PlanKind = iota
	// PlanSemiActuated rests in the major phase and serves minor phases on
	// detector demand (AdvanceActuated).
	PlanSemiActuated
)

// Semi-actuated tuning (seconds / meters). See the design doc for rationale.
const (
	actMajorMinGreen   = 15.0 // major holds >= this before yielding to a call
	actMinGreen        = 7.0  // minor phase minimum green once served
	actPassage         = 3.0  // green ends this long after the passage zone clears
	actMaxGreen        = 40.0 // minor-phase ceiling (max-out)
	actCallDistance    = 60.0 // a vehicle within this of the stop line calls a phase
	actPassageDistance = 25.0 // a vehicle within this holds (extends) the green
)
```

Extend `SignalConfig` with: `Plan PlanKind`, `MajorPhase int`, `MinGreen`,
`MajorMinGreen`, `Passage`, `MaxGreen float64`. Extend `SignalState` with
`passageGap float64`. In `NewSignalState`, when `Plan == PlanSemiActuated`,
default any zero timing field to the `act*` constants (so hand-built actuated
configs need only set `Plan` + `MajorPhase`).

`nextActuatedPhase` — pick the next phase to serve:

```go
// nextActuatedPhase returns the phase to run after the current one ends:
// the first called minor phase in cyclic order after curr, or MajorPhase when
// nothing is called (the controller returns to the major street and rests).
func (s *SignalState) nextActuatedPhase(curr int, called []bool) int {
	n := len(s.Config.Phases)
	for off := 1; off <= n; off++ {
		p := (curr + off) % n
		if p != s.Config.MajorPhase && p < len(called) && called[p] {
			return p
		}
	}
	return s.Config.MajorPhase
}
```

`AdvanceActuated` — the semi-actuated machine (only `ModeNormal`; other modes
no-op exactly like `Advance`):

```go
// AdvanceActuated advances a semi-actuated signal by dt seconds. called[i] /
// occupied[i] report whether phase i has a vehicle within the call / passage
// zone this tick (pure functions of vehicle positions). The major phase rests
// in green until a minor phase is called; minor phases hold for >= MinGreen,
// extend while occupied, and terminate on gap-out (Passage after the zone
// clears) or max-out (MaxGreen). Yellow runs YellowDur, then nextActuatedPhase
// chooses the successor.
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
		s.PhaseIdx = s.nextActuatedPhase(s.PhaseIdx, called)
		return
	}

	// Green.
	if s.PhaseIdx == s.Config.MajorPhase {
		if s.Elapsed < s.Config.MajorMinGreen {
			return
		}
		if anyCalledMinor(called, s.Config.MajorPhase) {
			s.toYellow()
			return
		}
		s.Elapsed = s.Config.MajorMinGreen // rest (no float drift, no max-out)
		return
	}

	// Minor phase.
	if occupied[s.PhaseIdx] {
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

func (s *SignalState) toYellow() { s.IsYellow = true; s.Elapsed = 0; s.passageGap = 0 }

func anyCalledMinor(called []bool, major int) bool {
	for i, c := range called {
		if i != major && c {
			return true
		}
	}
	return false
}
```

Note `occupied`/`called` are indexed by phase; guard length with the
`p < len(called)` check already in `nextActuatedPhase`, and assume the world
passes correctly-sized slices for the live path (the minor branch indexes
`occupied[s.PhaseIdx]` directly — the world always sizes both to `len(Phases)`).

**Tests** (`signal_test.go`), all on hand-built 2-phase configs
(`MajorPhase: 0`, minor = phase 1):

- `TestActuated_MajorRests`: no calls; advance 300 s in 0.05 s steps → stays
  phase 0 green, never yellow.
- `TestActuated_ServesCall`: `called=[false,true]`; after `MajorMinGreen` the
  major goes yellow, then (after `YellowDur`) phase 1 greens; with the call
  cleared and `occupied=[false,false]`, phase 1 holds `MinGreen` then gaps out
  after `Passage`, and `nextActuatedPhase` returns to phase 0.
- `TestActuated_GapOut`: in phase 1, `occupied=[false,true]` holds past
  `MinGreen`; flip to `occupied=[false,false]` and assert termination exactly
  `Passage` later (not before).
- `TestActuated_MaxOut`: phase 1 with `occupied=[false,true]` *continuously* →
  terminates at `MaxGreen` despite steady demand.
- `TestNextActuatedPhase`: 3-phase config, `MajorPhase 0`; called=[_, false,
  true] → returns 2; called all false → returns 0; from phase 2 with phase 1
  called → wraps to 1.

Run: `go test ./internal/sim/ -run 'TestActuated|TestNextActuatedPhase' -v`.

Commit: `feat(sim): add semi-actuated signal state machine`.

---

## Task 2: `DefaultSignalConfig` emits semi-actuated plans (`signal.go`)

Phase construction (axis bucketing) is unchanged. After building `phases`:

- `len(phases) <= 1` → return as today (`PlanFixed`, permanent green — nothing to
  actuate). The single-phase early-return at the top of the function already
  covers the `len(incoming) <= 1 || net == nil` case; leave it `PlanFixed`.
- Otherwise pick `MajorPhase` and return `PlanSemiActuated`:

```go
major := majorPhase(phases, incoming, net)
return SignalConfig{
	Phases:        phases,
	Plan:          PlanSemiActuated,
	MajorPhase:    major,
	MinGreen:      actMinGreen,
	MajorMinGreen: actMajorMinGreen,
	Passage:       actPassage,
	MaxGreen:      actMaxGreen,
}
```

```go
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
```

**Tests** (`signal_test.go`):

- `TestDefaultSignalConfig_SemiActuated`: build a 4-leg cross where the E–W axis
  edges are `ClassPrimary` and N–S are `ClassResidential`. Assert
  `Plan == PlanSemiActuated`, timings defaulted, and `MajorPhase` is the phase
  containing the primary (E or W) approach.
- `TestDefaultSignalConfig_SingleLeg`: 1 incoming → `PlanFixed`.
- Extend `TestDefaultSignalConfig_TWithBend`: still 2 phases; additionally assert
  `Plan == PlanSemiActuated` and `MajorPhase` is the through-road phase (give the
  through road a higher class than the stub).

Run: `go test ./internal/sim/ -run 'TestDefaultSignalConfig' -v`.

Commit: `feat(sim): default signals to semi-actuated control`.

---

## Task 3: Detector scan + dispatch in the tick (`world.go`)

**3a. Precompute detector edges in `NewWorld`.** After building `sigs`, collect
the incoming edges of every `PlanSemiActuated` signal into a lookup the tick can
use to bound the scan:

```go
// detectorEdge maps an incoming EdgeID of a semi-actuated signal to
// (intersection signal-state index, phase index it belongs to). One edge feeds
// exactly one phase (DefaultSignalConfig partitions approaches by axis).
type detectorEdge struct{ sigIdx, phaseIdx int }
```

Store `w.detectorEdges map[network.EdgeID]detectorEdge`. Populate by walking each
`PlanSemiActuated` state's `Config.Phases[*].GreenEdges` positions back to the
intersection's `Incoming[pos]` EdgeID.

**3b. Per-tick detector scan** (step 0b, before the advance loop). Build per-
signal `called`/`occupied` slices:

```go
// detectorOcc[sigIdx] -> per-phase (called, occupied) for this tick.
called := make([][]bool, len(w.SignalStates))
occupied := make([][]bool, len(w.SignalStates))
for i, s := range w.SignalStates {
	if s != nil && s.Config.Plan == PlanSemiActuated {
		called[i] = make([]bool, len(s.Config.Phases))
		occupied[i] = make([]bool, len(s.Config.Phases))
	}
}
for vi := range w.Vehicles {
	v := &w.Vehicles[vi]
	if v.Despawned {
		continue
	}
	de, ok := w.detectorEdges[v.Edge]
	if !ok {
		continue
	}
	d := w.Net.Edges[v.Edge].Length - v.S
	if d <= actCallDistance {
		called[de.sigIdx][de.phaseIdx] = true
	}
	if d <= actPassageDistance {
		occupied[de.sigIdx][de.phaseIdx] = true
	}
}
```

(Allocation note: two `[][]bool` per tick. If a benchmark flags it, hoist into
reusable `World` scratch buffers cleared each tick — deferred unless profiling
shows it matters; the existing tick already allocates `byEdge` maps.)

**3c. Dispatch** in the advance loop:

```go
for i, s := range w.SignalStates {
	if s == nil {
		continue
	}
	if s.Config.Plan == PlanSemiActuated {
		s.AdvanceActuated(w.dt, called[i], occupied[i])
	} else {
		s.Advance(w.dt)
	}
}
```

The change-detection / trace-emission block below it is unchanged.

**Tests** (`world_test.go`) — build a 4-leg signalized cross via a small net +
`DefaultSignalConfig` (or `NewWorld`, which auto-generates it); set `HasSignal`
on the intersection:

- `TestWorld_ArterialRests`: spawn/inject vehicles only on the major approach,
  none on the side street. Step a few minutes; assert the major approach's
  vehicles are essentially never forced to a stop at the line (e.g. they
  traverse the intersection without `V` dropping to ~0 on the approach).
- `TestWorld_SideStreetServedOnDemand`: inject one vehicle on the side street;
  assert that within `actMajorMinGreen + YellowDur + actMaxGreen` it gets a green
  for its approach (`SignalStates[x].GreenFor(sidePos)` true at some tick) and
  clears the intersection.
- `TestWorld_SemiActuatedDeterminism`: two `World`s, same seed/net/spawn-rate,
  run N ticks, assert identical signal `PhaseIdx`/`IsYellow` sequences (a
  lighter mirror of the trace-determinism guarantee for the actuated path).

Run: `go test ./internal/sim/ -run 'TestWorld_Arterial|TestWorld_SideStreet|TestWorld_SemiActuated' -v`.

---

## Final checks

- `go test ./...` green; `go vet ./...` clean.
- `go test ./internal/sim/ -bench=BenchmarkStep -benchtime=2s -run=^$` — confirm
  the detector scan keeps the tick well under budget (compare against the README
  baseline; expect negligible delta).
- Determinism: `go test ./internal/sim/ -run TestWorld_TraceDeterminism`.

## Manual verification (optional)

```bash
go build -o trafficsim ./cmd/trafficsim
./trafficsim run --spawn-rate 20 /home/lab/MEGA/OpenStreetMap/jackson.osm
```

Watch a signalized junction with a quiet side street: the arterial should hold
green and only surrender it when a side-street vehicle arrives, then resume.

## Out of scope (deferred — see spec)

Fully-actuated major street, corridor coordination/green-waves, separate all-red
clearance, YAML-configurable actuation, per-turn-movement detectors.

## README

Add a short "Actuated signals" subsection under the signals docs noting that
auto-generated signals are semi-actuated (major rests, side streets on demand),
that `--signals` overrides remain fixed-time, and that determinism is preserved.
