package sim

import "math"

// IDMParams parameterize the Intelligent Driver Model (Treiber et al, 2000).
type IDMParams struct {
	A     float64 // max acceleration (m/s^2), typical 1.0
	B     float64 // comfortable deceleration (m/s^2), typical 1.5
	S0    float64 // minimum stopping gap (m), typical 2.0
	T     float64 // safe time headway (s), typical 1.5
	Delta float64 // free-flow acceleration exponent, typical 4
}

func DefaultIDM() IDMParams {
	return IDMParams{A: 1.0, B: 1.5, S0: 2.0, T: 1.5, Delta: 4}
}

// IDMAcceleration returns the acceleration in m/s^2.
//
//	v       current speed (m/s)
//	v0      desired speed (m/s) — typically the edge speed limit
//	gap     bumper-to-bumper distance to leader (m); pass math.Inf(1) if none
//	deltaV  v - vLeader (positive = closing)
//
// The result may be negative (braking) and is mathematically defined for
// all non-negative gaps; the caller is responsible for clamping the
// resulting speed to >= 0 after integration.
func IDMAcceleration(p IDMParams, v, v0, gap, deltaV float64) float64 {
	if v0 <= 0 {
		v0 = 0.1
	}
	freeTerm := 1.0 - math.Pow(v/v0, p.Delta)

	if math.IsInf(gap, 1) {
		return p.A * freeTerm
	}
	// Desired dynamic gap.
	sStar := p.S0 + math.Max(0, v*p.T+(v*deltaV)/(2*math.Sqrt(p.A*p.B)))
	if gap < 0.01 {
		gap = 0.01
	}
	intTerm := (sStar / gap) * (sStar / gap)
	return p.A * (freeTerm - intTerm)
}
