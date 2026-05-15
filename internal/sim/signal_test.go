package sim

import "testing"

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
	// Wrap-around: advance past full cycle.
	s.Advance(60.0)
	if s.PhaseIdx == 1 && !s.IsYellow {
		// any state is fine; we just want no panic and a sensible index
	}
	if s.PhaseIdx >= len(cfg.Phases) {
		t.Errorf("PhaseIdx out of range: %d", s.PhaseIdx)
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
