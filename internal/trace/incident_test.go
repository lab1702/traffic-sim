package trace

import (
	"bytes"
	"testing"
)

func TestWriteRead_IncidentSet(t *testing.T) {
	cases := []IncidentSet{
		{EdgeID: 0, Severity: 0},   // clear
		{EdgeID: 7, Severity: 1},   // slowdown
		{EdgeID: 42, Severity: 2},  // lane close
		{EdgeID: 999, Severity: 3}, // full close
	}
	for _, want := range cases {
		var buf bytes.Buffer
		w := NewWriter(&buf)
		if err := w.Write(5, 1.25, &want); err != nil {
			t.Fatalf("write: %v", err)
		}
		r := NewReader(&buf)
		_, ev, err := r.Next()
		if err != nil {
			t.Fatalf("read: %v", err)
		}
		got, ok := ev.(*IncidentSet)
		if !ok {
			t.Fatalf("decoded type = %T, want *IncidentSet", ev)
		}
		if got.EdgeID != want.EdgeID || got.Severity != want.Severity {
			t.Fatalf("round-trip = %+v, want %+v", *got, want)
		}
	}
}
