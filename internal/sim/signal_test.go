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
