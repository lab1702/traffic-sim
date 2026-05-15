package sim

import (
	"math"
	"testing"
)

func TestIDM_FreeFlowAccelerates(t *testing.T) {
	p := DefaultIDM()
	// Alone on road, well below desired speed.
	a := IDMAcceleration(p, 5.0 /*v*/, 30.0 /*v0*/, math.Inf(1), 0)
	if a <= 0 {
		t.Errorf("free-flow well below v0 should accelerate, got a=%.2f", a)
	}
}

func TestIDM_AtDesiredSpeedNoAccel(t *testing.T) {
	p := DefaultIDM()
	a := IDMAcceleration(p, 30.0, 30.0, math.Inf(1), 0)
	if math.Abs(a) > 0.05 {
		t.Errorf("at desired speed with no leader: want a~=0, got %.3f", a)
	}
}

func TestIDM_BrakesForClosingLeader(t *testing.T) {
	p := DefaultIDM()
	// Approaching a slower (or stopped) leader at small gap.
	a := IDMAcceleration(p, 20.0, 25.0, 5.0 /*gap*/, 15.0 /*deltaV = ego - leader*/)
	if a >= 0 {
		t.Errorf("closing on slow leader at 5m gap should brake hard, got a=%.2f", a)
	}
}

func TestIDM_ZeroGapClampSafe(t *testing.T) {
	p := DefaultIDM()
	// Bumper-to-bumper, both stopped: should brake hard (not blow up).
	a := IDMAcceleration(p, 0.0, 25.0, 0.0, 0.0)
	if math.IsNaN(a) || math.IsInf(a, 0) {
		t.Errorf("zero gap must not produce NaN/Inf, got a=%v", a)
	}
}
