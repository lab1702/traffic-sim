package trace

import (
	"bytes"
	"encoding/binary"
	"errors"
	"io"
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
		&TraceDropped{Count: 17},
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

// TestUnknownEvent_RoundTripsOnWrite covers the read→filter→rewrite case
// for tools that transform traces: an UnknownEvent obtained from Reader
// must be writable back through Writer without loss, preserving both
// the kind byte and the raw payload.
func TestUnknownEvent_RoundTripsOnWrite(t *testing.T) {
	src := &UnknownEvent{KindVal: Kind(200), Payload: []byte{0x11, 0x22, 0x33, 0x44}}

	var buf bytes.Buffer
	w := NewWriter(&buf)
	if err := w.Write(7, 1.25, src); err != nil {
		t.Fatalf("write UnknownEvent: %v", err)
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}

	r := NewReader(&buf)
	hdr, ev, err := r.Next()
	if err != nil {
		t.Fatalf("read back: %v", err)
	}
	if hdr.Tick != 7 || hdr.SimTime != 1.25 {
		t.Errorf("header lost: got %+v", hdr)
	}
	got, ok := ev.(*UnknownEvent)
	if !ok {
		t.Fatalf("readback type: got %T, want *UnknownEvent", ev)
	}
	if got.KindVal != src.KindVal {
		t.Errorf("KindVal: got %d, want %d", got.KindVal, src.KindVal)
	}
	if !bytes.Equal(got.Payload, src.Payload) {
		t.Errorf("Payload: got %x, want %x", got.Payload, src.Payload)
	}
}

// TestReader_TruncatedHeader returns io.EOF when the stream ends cleanly
// at an event boundary. This is the normal end-of-file path; the player
// surfaces it as "trace ended".
func TestReader_TruncatedHeader(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)
	if err := w.Write(1, 0.1, &VehicleDespawn{VehicleID: 1}); err != nil {
		t.Fatalf("write: %v", err)
	}
	r := NewReader(&buf)
	if _, _, err := r.Next(); err != nil {
		t.Fatalf("first read: %v", err)
	}
	_, _, err := r.Next()
	if !errors.Is(err, io.EOF) {
		t.Errorf("want io.EOF at clean end-of-stream, got %v", err)
	}
}

// TestReader_TruncatedPayload returns io.ErrUnexpectedEOF when the
// stream ends mid-event. This is the "process killed during write" path;
// the player surfaces it as a clearer "trace truncated" warning.
func TestReader_TruncatedPayload(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)
	if err := w.Write(1, 0.1, &VehicleSpawn{
		VehicleID: 1, OriginNode: 0, DestNode: 1, Route: []uint32{0, 1, 2},
	}); err != nil {
		t.Fatalf("write: %v", err)
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	// Lop off the last 6 bytes — mid-Route payload.
	data := buf.Bytes()
	truncated := bytes.NewReader(data[:len(data)-6])

	r := NewReader(truncated)
	_, _, err := r.Next()
	if !errors.Is(err, io.ErrUnexpectedEOF) {
		t.Errorf("want io.ErrUnexpectedEOF on truncated payload, got %v", err)
	}
}

func TestRoundTrip_VehicleReroute(t *testing.T) {
	var buf bytes.Buffer
	w := NewWriter(&buf)

	in := []Event{
		&VehicleReroute{VehicleID: 7, AtIndex: 2, NewTail: []uint32{5, 6, 7}},
		&VehicleReroute{VehicleID: 8, AtIndex: 0, NewTail: nil}, // empty tail
	}
	for i, e := range in {
		if err := w.Write(uint64(i), float64(i), e); err != nil {
			t.Fatalf("write %T: %v", e, err)
		}
	}
	if err := w.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}

	r := NewReader(&buf)
	for i, want := range in {
		_, ev, err := r.Next()
		if err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
		gotRR, ok := ev.(*VehicleReroute)
		if !ok {
			t.Fatalf("event %d: got %T, want *VehicleReroute", i, ev)
		}
		wantRR := want.(*VehicleReroute)
		if gotRR.VehicleID != wantRR.VehicleID || gotRR.AtIndex != wantRR.AtIndex {
			t.Errorf("event %d: got %+v, want %+v", i, gotRR, wantRR)
		}
		if len(gotRR.NewTail) != len(wantRR.NewTail) {
			t.Errorf("event %d: tail len %d, want %d", i, len(gotRR.NewTail), len(wantRR.NewTail))
		}
		for j := range wantRR.NewTail {
			if gotRR.NewTail[j] != wantRR.NewTail[j] {
				t.Errorf("event %d tail[%d]: got %d, want %d", i, j, gotRR.NewTail[j], wantRR.NewTail[j])
			}
		}
	}
}
