package sim

import (
	"testing"
	"time"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/snapshot"
)

// TestSignalMode_MatchesSnapshotConstants prevents drift between the sim
// package's SignalMode iota and the renderer-visible snapshot.Mode*
// constants. Renderer and tracereplay key off snapshot.Mode* without
// importing sim; adding a new mode requires updating both.
func TestSignalMode_MatchesSnapshotConstants(t *testing.T) {
	cases := []struct {
		name string
		sim  SignalMode
		snap uint8
	}{
		{"normal", ModeNormal, snapshot.ModeNormal},
		{"flash_a", ModeFlashA, snapshot.ModeFlashA},
		{"flash_b", ModeFlashB, snapshot.ModeFlashB},
		{"off", ModeOff, snapshot.ModeOff},
	}
	for _, c := range cases {
		if uint8(c.sim) != c.snap {
			t.Errorf("%s: sim=%d snapshot=%d (must match)", c.name, c.sim, c.snap)
		}
	}
}

func TestSignalCycle_AdvancesPhases(t *testing.T) {
	// 2 phases, 30s green each, 3s yellow each => 66s total cycle.
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0, 2}, GreenDur: 30, YellowDur: 3},
			{GreenEdges: []int{1, 3}, GreenDur: 30, YellowDur: 3},
		},
	}
	s := NewSignalState(cfg)
	// At t=0 we're in phase 0 (green).
	if s.PhaseIdx != 0 || s.IsYellow {
		t.Fatalf("t=0 should be phase 0 green, got phase=%d yellow=%v", s.PhaseIdx, s.IsYellow)
	}
	// Advance 31s -> should be in phase 0 yellow.
	s.Advance(31.0)
	if s.PhaseIdx != 0 || !s.IsYellow {
		t.Errorf("t=31 should be phase 0 yellow, got phase=%d yellow=%v", s.PhaseIdx, s.IsYellow)
	}
	// Advance another 3s -> phase 1 green.
	s.Advance(3.0)
	if s.PhaseIdx != 1 || s.IsYellow {
		t.Errorf("t=34 should be phase 1 green, got phase=%d yellow=%v", s.PhaseIdx, s.IsYellow)
	}
	// Wrap-around: advance another 60s. Going into this call the state
	// is (PhaseIdx=1, IsYellow=false, Elapsed=1) — already 1s into
	// phase 1 green. So +60s consumes:
	//   29s remaining in phase 1 green  (Elapsed 30 → flips to yellow, 0)
	//    3s phase 1 yellow              (Elapsed 3  → flips to phase 0, 0)
	//   28s into phase 0 green          (still green, Elapsed=28)
	s.Advance(60.0)
	if s.PhaseIdx != 0 {
		t.Errorf("after wrap-around: PhaseIdx = %d, want 0", s.PhaseIdx)
	}
	if s.IsYellow {
		t.Errorf("after wrap-around: should be in green phase, got yellow")
	}
	if got := s.Elapsed; got < 27.9 || got > 28.1 {
		t.Errorf("after wrap-around: Elapsed = %v, want ~28.0", got)
	}
}

// TestSignal_DegeneratePhase_DoesNotInfiniteLoop documents the defensive
// guard in Advance: a phase with both GreenDur=0 and YellowDur=0 would
// otherwise spin forever subtracting 0 from Elapsed. Config-layer
// validation rejects such inputs, but Advance must not hang if a hand-
// constructed SignalConfig sneaks one in (e.g., in a test or a future
// programmatic config generator).
func TestSignal_DegeneratePhase_DoesNotInfiniteLoop(t *testing.T) {
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0}, GreenDur: 0, YellowDur: 0},
			{GreenEdges: []int{1}, GreenDur: 5, YellowDur: 1},
		},
	}
	s := NewSignalState(cfg)
	done := make(chan struct{})
	go func() {
		s.Advance(1.0)
		close(done)
	}()
	select {
	case <-done:
		// Good: returned. The degenerate phase should have been
		// skipped, landing the state machine in the second phase.
		if s.PhaseIdx != 1 {
			t.Errorf("after Advance past degenerate phase: PhaseIdx = %d, want 1", s.PhaseIdx)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("Advance hung on a degenerate phase (infinite loop)")
	}
}

func TestSignalCycle_GreenForEdge(t *testing.T) {
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0}, GreenDur: 30, YellowDur: 3},
			{GreenEdges: []int{1}, GreenDur: 30, YellowDur: 3},
		},
	}
	s := NewSignalState(cfg)
	if !s.GreenFor(0) {
		t.Errorf("phase 0 should be green for edge index 0")
	}
	if s.GreenFor(1) {
		t.Errorf("phase 0 should NOT be green for edge index 1")
	}
}

func TestSignalMode_FlashA(t *testing.T) {
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0, 2}}, // phase 0 = NS-pair
			{GreenEdges: []int{1, 3}}, // phase 1 = EW-pair
		},
	}
	s := NewSignalState(cfg)
	s.Mode = ModeFlashA

	// Phase 0 approaches blink yellow (priority) -> GreenFor true, no yield.
	for _, pos := range []int{0, 2} {
		if !s.GreenFor(pos) {
			t.Errorf("FlashA: phase-0 approach %d should be GreenFor=true", pos)
		}
		if s.MustYield(pos) {
			t.Errorf("FlashA: phase-0 approach %d should NOT yield (yellow has priority)", pos)
		}
	}
	// Phase 1 approaches blink red -> GreenFor false, must yield.
	for _, pos := range []int{1, 3} {
		if s.GreenFor(pos) {
			t.Errorf("FlashA: phase-1 approach %d should be GreenFor=false", pos)
		}
		if !s.MustYield(pos) {
			t.Errorf("FlashA: phase-1 approach %d should yield (blinking red)", pos)
		}
	}

	// Advance does nothing in flash mode.
	originalPhase, originalYellow := s.PhaseIdx, s.IsYellow
	s.Advance(1000.0)
	if s.PhaseIdx != originalPhase || s.IsYellow != originalYellow {
		t.Errorf("FlashA: Advance should be a no-op, got phase=%d yellow=%v", s.PhaseIdx, s.IsYellow)
	}
}

func TestSignalMode_FlashB_FlipsPriority(t *testing.T) {
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0, 2}},
			{GreenEdges: []int{1, 3}},
		},
	}
	s := NewSignalState(cfg)
	s.Mode = ModeFlashB

	if s.GreenFor(0) || s.GreenFor(2) {
		t.Errorf("FlashB: phase-0 approaches should NOT be green (red now)")
	}
	if !s.GreenFor(1) || !s.GreenFor(3) {
		t.Errorf("FlashB: phase-1 approaches should be green (yellow priority)")
	}
}

func TestSignalMode_Off(t *testing.T) {
	cfg := SignalConfig{
		Phases: []SignalPhase{
			{GreenEdges: []int{0, 2}},
			{GreenEdges: []int{1, 3}},
		},
	}
	s := NewSignalState(cfg)
	s.Mode = ModeOff

	// No approach has implicit right-of-way; all must yield.
	for pos := 0; pos < 4; pos++ {
		if s.GreenFor(pos) {
			t.Errorf("Off: approach %d should not be GreenFor=true", pos)
		}
		if !s.MustYield(pos) {
			t.Errorf("Off: approach %d should yield", pos)
		}
	}
}

// TestDefaultSignalConfig_TWithBend: a T-intersection where the through
// road has a slight angular variation at the intersection (very common
// in real OSM data — perfectly straight roads are the exception). The
// two opposing through-road approaches must end up in the same phase,
// or drivers on the through road see a cross-stream of red lights for
// no reason and the stub road gets disproportionate green time.
func TestDefaultSignalConfig_TWithBend(t *testing.T) {
	// Through road runs east-west through the origin. Both halves end
	// with a slight northward dip approaching the intersection (the
	// intersection node is slightly north of the road centerline — a
	// very ordinary OSM mapping quirk). The stub heads due north.
	//
	//   west half last segment: (-100, -2) -> (0, 0)  arrival heading ≈ +0.02 (N of E)
	//   east half last segment: ( 100, -2) -> (0, 0)  arrival heading ≈ π - 0.02 (N of W)
	//   stub last segment:      (   0, 100) -> (0, 0) arrival heading = -π/2 (south)
	//
	// Both through-road approaches are ~180° apart in direction, so
	// they belong on the same axis and must share a phase.
	nodes := []network.Node{
		{ID: 0, Pos: network.Point{X: -100, Y: -2}}, // west end
		{ID: 1, Pos: network.Point{X: 100, Y: -2}},  // east end
		{ID: 2, Pos: network.Point{X: 0, Y: 100}},   // stub end
		{ID: 3, Pos: network.Point{X: 0, Y: 0}},     // intersection
	}
	mkEdge := func(id, from, to int) network.Edge {
		return network.Edge{
			ID: network.EdgeID(id), From: network.NodeID(from), To: network.NodeID(to),
			Length: 100, SpeedLimit: 10,
			Geometry: []network.Point{nodes[from].Pos, nodes[to].Pos},
		}
	}
	net := &network.Network{
		Nodes: nodes,
		Edges: []network.Edge{
			mkEdge(0, 0, 3), // W -> C (eastbound on arrival, slight N)
			mkEdge(1, 1, 3), // E -> C (westbound on arrival, slight N)
			mkEdge(2, 2, 3), // stub -> C (southbound on arrival)
		},
	}
	incoming := []network.EdgeID{0, 1, 2}

	cfg := DefaultSignalConfig(incoming, net)
	if len(cfg.Phases) != 2 {
		t.Fatalf("T with through road + stub must produce 2 phases (through, stub), got %d", len(cfg.Phases))
	}

	phaseOf := func(pos int) int {
		for i, p := range cfg.Phases {
			for _, gp := range p.GreenEdges {
				if gp == pos {
					return i
				}
			}
		}
		return -1
	}
	pW, pE, pStub := phaseOf(0), phaseOf(1), phaseOf(2)
	if pW < 0 || pE < 0 || pStub < 0 {
		t.Fatalf("every approach must belong to a phase; got W=%d E=%d stub=%d", pW, pE, pStub)
	}
	if pW != pE {
		t.Errorf("through-road approaches (W=%d, E=%d) must share a phase", pW, pE)
	}
	if pW == pStub {
		t.Errorf("stub (phase %d) must be in a different phase from through road (phase %d)", pStub, pW)
	}
}

func TestParseSignalMode(t *testing.T) {
	cases := []struct {
		in   string
		want SignalMode
		ok   bool
	}{
		{"normal", ModeNormal, true},
		{"", ModeNormal, true}, // empty is treated as normal
		{"flash_a", ModeFlashA, true},
		{"flash_b", ModeFlashB, true},
		{"off", ModeOff, true},
		{"FlashA", ModeNormal, false}, // case-sensitive: invalid
		{"unknown", ModeNormal, false},
	}
	for _, c := range cases {
		got, ok := ParseSignalMode(c.in)
		if got != c.want || ok != c.ok {
			t.Errorf("ParseSignalMode(%q) = (%v, %v); want (%v, %v)", c.in, got, ok, c.want, c.ok)
		}
	}
}
