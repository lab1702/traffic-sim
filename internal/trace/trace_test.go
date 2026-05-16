package trace

import (
	"bytes"
	"encoding/binary"
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
		&SignalModeChange{IntersectionID: 5, Mode: 2}, // ModeFlashB
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

// TestReader_UnknownKindIsSkippable verifies forward compatibility: a reader
// presented with an unrecognized event kind returns *UnknownEvent rather
// than aborting the stream, so newer trace formats can extend with new
// kinds without breaking older readers.
//
// Regression: see review #5 (2026-05-15).
func TestReader_UnknownKindIsSkippable(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)
	// Real event before the unknown one — proves stream-level state is fine.
	if err := w.Write(1, 0.5, &VehicleDespawn{VehicleID: 7}); err != nil {
		t.Fatalf("write 1: %v", err)
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}

	// Now inject a hand-crafted "future kind 250" event with a 3-byte
	// payload directly into the byte stream after what NewWriter wrote.
	le := binary.LittleEndian
	binary.Write(&buf, le, uint64(2))       // Tick
	binary.Write(&buf, le, float64(1.0))    // SimTime
	binary.Write(&buf, le, uint8(250))      // Kind = unknown
	binary.Write(&buf, le, uint16(3))       // payload length
	buf.Write([]byte{0xAA, 0xBB, 0xCC})     // arbitrary payload

	// Trailing well-known event so we also confirm the stream is still
	// readable after the unknown.
	binary.Write(&buf, le, uint64(3))    // Tick
	binary.Write(&buf, le, float64(1.5)) // SimTime
	binary.Write(&buf, le, uint8(KindVehicleDespawn))
	binary.Write(&buf, le, uint16(4)) // payload length
	binary.Write(&buf, le, uint32(9)) // VehicleID

	r := NewReader(&buf)

	// First event: well-known despawn.
	_, ev, err := r.Next()
	if err != nil {
		t.Fatalf("first read: %v", err)
	}
	if _, ok := ev.(*VehicleDespawn); !ok {
		t.Fatalf("first event: want *VehicleDespawn, got %T", ev)
	}

	// Second event: unknown kind 250 — should produce *UnknownEvent, not error.
	_, ev, err = r.Next()
	if err != nil {
		t.Fatalf("unknown-kind read returned error %v (forward-compat broken)", err)
	}
	u, ok := ev.(*UnknownEvent)
	if !ok {
		t.Fatalf("want *UnknownEvent, got %T", ev)
	}
	if u.KindVal != Kind(250) {
		t.Errorf("UnknownEvent.KindVal = %d, want 250", u.KindVal)
	}
	if !bytes.Equal(u.Payload, []byte{0xAA, 0xBB, 0xCC}) {
		t.Errorf("UnknownEvent.Payload = %x, want AA BB CC", u.Payload)
	}

	// Third event: stream is intact, despawn parses correctly.
	_, ev, err = r.Next()
	if err != nil {
		t.Fatalf("third read after unknown: %v (stream desync)", err)
	}
	dsp, ok := ev.(*VehicleDespawn)
	if !ok {
		t.Fatalf("third event: want *VehicleDespawn, got %T", ev)
	}
	if dsp.VehicleID != 9 {
		t.Errorf("third event VehicleID = %d, want 9", dsp.VehicleID)
	}
}
