package trace

import (
	"bytes"
	"reflect"
	"testing"
)

func TestRoundTrip_AllEventKinds(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)

	in := []Event{
		&SimStart{SeedHi: 1, SeedLo: 2, NetHash: 0xDEADBEEF},
		&VehicleSpawn{VehicleID: 100, Route: []uint32{0, 1, 2}, OriginNode: 0, DestNode: 3},
		&VehicleDespawn{VehicleID: 100},
		&SignalPhase{IntersectionID: 5, PhaseIdx: 1, IsYellow: false},
		&MetricsTick{TotalVehicles: 42, AvgSpeed: 7.5, CongestionIdx: 0.2},
		&SimEnd{Reason: "duration"},
	}
	for i, e := range in {
		if err := w.Write(uint64(i*10), float64(i)*0.5, e); err != nil {
			t.Fatalf("write %T: %v", e, err)
		}
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}

	r := NewReader(&buf)
	for i, want := range in {
		hdr, ev, err := r.Next()
		if err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
		if hdr.Tick != uint64(i*10) {
			t.Errorf("event %d: tick %d, want %d", i, hdr.Tick, i*10)
		}
		if !reflect.DeepEqual(ev, want) {
			t.Errorf("event %d: got %+v, want %+v", i, ev, want)
		}
	}
	// EOF.
	_, _, err := r.Next()
	if err == nil {
		t.Errorf("want EOF, got nil")
	}
}
