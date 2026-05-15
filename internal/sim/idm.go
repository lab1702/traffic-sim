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

// MaxBraking is the strongest deceleration any vehicle will apply. Vanilla
// IDM's freeTerm = 1 - (v/v0)^delta can produce wildly negative values
// when v >> v0 (e.g. a sudden tight corner ahead), which translates to
// unrealistic decelerations like -200 m/s². We clamp the IDM output to
// this physical panic-brake limit (~0.8 g) so the simulation behaves like
// real cars.
const MaxBraking = 8.0

// MaxAccel mirrors MaxBraking on the positive side. Real cars top out at
// ~3-4 m/s²; IDM at p.A=1 won't normally exceed that, but clamp for
// safety in case future tuning increases p.A.
const MaxAccel = 4.0

// IDMAcceleration returns the acceleration in m/s^2, clamped to a
// physical range [-MaxBraking, +MaxAccel].
//
//	v       current speed (m/s)
//	v0      desired speed (m/s) — typically the edge speed limit
//	gap     bumper-to-bumper distance to leader (m); pass math.Inf(1) if none
//	deltaV  v - vLeader (positive = closing)
//
// The caller is responsible for clamping the resulting speed to >= 0
// after integration.
func IDMAcceleration(p IDMParams, v, v0, gap, deltaV float64) float64 {
	if v0 <= 0 {
		v0 = 0.1
	}
	freeTerm := 1.0 - math.Pow(v/v0, p.Delta)

	var a float64
	if math.IsInf(gap, 1) {
		a = p.A * freeTerm
	} else {
		// Desired dynamic gap.
		sStar := p.S0 + math.Max(0, v*p.T+(v*deltaV)/(2*math.Sqrt(p.A*p.B)))
		if gap < 0.01 {
			gap = 0.01
		}
		intTerm := (sStar / gap) * (sStar / gap)
		a = p.A * (freeTerm - intTerm)
	}
	if a > MaxAccel {
		a = MaxAccel
	}
	if a < -MaxBraking {
		a = -MaxBraking
	}
	return a
}
