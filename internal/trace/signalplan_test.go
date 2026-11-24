package trace

import (
	"bytes"
	"testing"
)

func TestRoundTrip_SignalPlan(t *testing.T) {
	in := []*SignalPlan{
		{IntersectionID: 12, Phases: [][]uint16{{0, 2}, {1, 3}}},
		{IntersectionID: 3, Phases: [][]uint16{{0}}},
		{IntersectionID: 4, Phases: [][]uint16{{}, {1}}}, // an all-red phase
	}
	var buf bytes.Buffer
	w := NewWriter(&buf)
	for _, e := range in {
		if err := w.Write(0, 0, e); err != nil {
			t.Fatalf("write: %v", err)
		}
	}
	r := NewReader(&buf)
	for i, want := range in {
		_, ev, err := r.Next()
		if err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
		got, ok := ev.(*SignalPlan)
		if !ok {
			t.Fatalf("event %d: got %T, want *SignalPlan", i, ev)
		}
		if got.IntersectionID != want.IntersectionID || len(got.Phases) != len(want.Phases) {
			t.Fatalf("event %d: got %+v, want %+v", i, got, want)
		}
		for j := range want.Phases {
			if len(got.Phases[j]) != len(want.Phases[j]) {
				t.Fatalf("event %d phase %d: got %v, want %v", i, j, got.Phases[j], want.Phases[j])
			}
			for k := range want.Phases[j] {
				if got.Phases[j][k] != want.Phases[j][k] {
					t.Fatalf("event %d phase %d: got %v, want %v", i, j, got.Phases[j], want.Phases[j])
				}
			}
		}
	}
}
