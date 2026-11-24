package sim

import (
	"testing"

	"github.com/lab1702/traffic-sim/internal/network"
	"github.com/lab1702/traffic-sim/internal/trace"
)

// TestWorld_EmitsInitialSignalPhase: the first tick records every signal's
// starting phase. lastPhase used to start at its zero value {0, false},
// which equals a fresh SignalState, so nothing was written until the first
// phase change; single-phase signals and semi-actuated ones resting in
// phase 0 never changed, and replay showed them red for the whole run.
func TestWorld_EmitsInitialSignalPhase(t *testing.T) {
	cases := []struct {
		name string
		net  *network.Network
	}{
		{"semi-actuated cross", signalCross()},
		{"single approach", &network.Network{
			Nodes: []network.Node{{ID: 0}, {ID: 1, Pos: network.Point{X: 200}}},
			Edges: []network.Edge{{ID: 0, From: 0, To: 1, Length: 200, SpeedLimit: 10,
				Lanes: []network.Lane{{Index: 0}}}},
			Intersections: []network.Intersection{
				{ID: 0, NodeID: 1, Incoming: []network.EdgeID{0}, HasSignal: true},
			},
		}},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			w := NewWorld(c.net, NewRandomOD(c.net, 0, 0), nil)
			var phases []*trace.SignalPhase
			var ticks []uint64
			w.EmitTrace = func(tick uint64, _ float64, e trace.Event) {
				if p, ok := e.(*trace.SignalPhase); ok {
					phases = append(phases, p)
					ticks = append(ticks, tick)
				}
			}
			w.Step()
			if len(phases) != 1 {
				t.Fatalf("first tick emitted %d SignalPhase events, want 1", len(phases))
			}
			st := w.SignalStates[0]
			if ticks[0] != 0 || phases[0].IntersectionID != 0 ||
				int(phases[0].PhaseIdx) != st.PhaseIdx || phases[0].IsYellow != st.IsYellow {
				t.Errorf("got %+v at tick %d, want intersection 0 phase %d yellow=%v at tick 0",
					phases[0], ticks[0], st.PhaseIdx, st.IsYellow)
			}
			// An unchanged state is not re-emitted.
			before := signalLast{idx: st.PhaseIdx, yellow: st.IsYellow}
			phases = nil
			w.Step()
			if (signalLast{idx: st.PhaseIdx, yellow: st.IsYellow}) == before && len(phases) != 0 {
				t.Errorf("unchanged phase re-emitted on tick 1: %+v", phases[0])
			}
		})
	}
}
