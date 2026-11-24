package trace

import (
	"bytes"
	"encoding/binary"
	"errors"
	"io"
	"reflect"
	"testing"
)

// fileHeaderLen is magic (4) + version (2).
const fileHeaderLen = 6

// oneEventTrace returns a complete trace holding a single event.
func oneEventTrace(t *testing.T, e Event) []byte {
	t.Helper()
	var buf bytes.Buffer
	w := NewWriter(&buf)
	if err := w.Write(3, 1.5, e); err != nil {
		t.Fatalf("write: %v", err)
	}
	return buf.Bytes()
}

// TestReader_TruncatedAtFieldBoundary: a record cut off exactly between two
// fields (after the tick, after the sim time, after the length prefix) is a
// truncated record, not a clean end of stream. binary.Read reports io.EOF
// when it reads zero bytes of a field, so the old reader passed a bare
// io.EOF through and the player treated the file as complete.
func TestReader_TruncatedAtFieldBoundary(t *testing.T) {
	full := oneEventTrace(t, &VehicleDespawn{VehicleID: 9})
	for _, cut := range []struct {
		name string
		n    int // bytes of the record kept
	}{
		{"after tick", 8},
		{"after simTime", 16},
		{"after kind", 17},
		{"after length", 19},
	} {
		t.Run(cut.name, func(t *testing.T) {
			r := NewReader(bytes.NewReader(full[:fileHeaderLen+cut.n]))
			_, _, err := r.Next()
			if !errors.Is(err, io.ErrUnexpectedEOF) {
				t.Fatalf("want io.ErrUnexpectedEOF, got %v", err)
			}
		})
	}
}

// TestReader_EmptyFile: Writer.Close always writes the file header, so a
// zero-byte (or header-truncated) file is a broken trace, not an empty one.
func TestReader_EmptyFile(t *testing.T) {
	for _, n := range []int{0, 4} { // nothing; magic only
		full := oneEventTrace(t, &VehicleDespawn{VehicleID: 1})
		r := NewReader(bytes.NewReader(full[:n]))
		_, _, err := r.Next()
		if !errors.Is(err, io.ErrUnexpectedEOF) {
			t.Errorf("%d-byte file: want io.ErrUnexpectedEOF, got %v", n, err)
		}
	}
}

// TestReader_HeaderOnlyIsCleanEOF: a header with no events is what
// Writer.Close produces for an empty run; that is a clean end.
func TestReader_HeaderOnlyIsCleanEOF(t *testing.T) {
	var buf bytes.Buffer
	if err := NewWriter(&buf).Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	_, _, err := NewReader(&buf).Next()
	if err != io.EOF {
		t.Fatalf("want bare io.EOF, got %v", err)
	}
}

// TestReader_ShortKnownPayload: a known kind whose length prefix is too
// small for its fields is a corrupt record. Decoding runs out of bytes at a
// field boundary, which binary.Read reports as io.EOF; that must not leak
// out as a clean end of stream.
func TestReader_ShortKnownPayload(t *testing.T) {
	cases := []struct {
		name    string
		kind    Kind
		payload []byte
	}{
		{"despawn with no payload", KindVehicleDespawn, nil},
		{"sim start missing second field", KindSimStart, make([]byte, 8)},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			var buf bytes.Buffer
			if err := NewWriter(&buf).Close(); err != nil {
				t.Fatalf("close: %v", err)
			}
			le := binary.LittleEndian
			binary.Write(&buf, le, uint64(4))
			binary.Write(&buf, le, float64(2.5))
			binary.Write(&buf, le, uint8(c.kind))
			binary.Write(&buf, le, uint16(len(c.payload)))
			buf.Write(c.payload)

			hdr, _, err := NewReader(&buf).Next()
			if !errors.Is(err, io.ErrUnexpectedEOF) {
				t.Fatalf("want io.ErrUnexpectedEOF, got %v", err)
			}
			if hdr.SimTime != 2.5 {
				t.Errorf("header of the bad record: SimTime %v, want 2.5", hdr.SimTime)
			}
		})
	}
}

// spliceRoutes applies a read-back event stream the way tracereplay does:
// a spawn sets the route, a reroute replaces route[AtIndex:].
func spliceRoutes(t *testing.T, r *Reader) map[uint32][]uint32 {
	t.Helper()
	routes := map[uint32][]uint32{}
	for {
		_, ev, err := r.Next()
		if err == io.EOF {
			return routes
		}
		if err != nil {
			t.Fatalf("read: %v", err)
		}
		switch e := ev.(type) {
		case *VehicleSpawn:
			routes[e.VehicleID] = append([]uint32(nil), e.Route...)
		case *VehicleReroute:
			rt := routes[e.VehicleID]
			if int(e.AtIndex) > len(rt) {
				t.Fatalf("reroute AtIndex %d past route length %d", e.AtIndex, len(rt))
			}
			routes[e.VehicleID] = append(rt[:e.AtIndex:e.AtIndex], e.NewTail...)
		}
	}
}

func seq(n int, base uint32) []uint32 {
	out := make([]uint32, n)
	for i := range out {
		out[i] = base + uint32(i)
	}
	return out
}

// TestWriter_SplitsOversizeRoutes: the uint16 payload length caps one record
// at 16,380 route edges. Longer spawn routes and reroute tails used to fail
// the write (and stop the trace sink); now they are split into continuation
// VehicleReroute records that extend the route, so a replayer that splices
// reroutes reconstructs the full route.
func TestWriter_SplitsOversizeRoutes(t *testing.T) {
	longRoute := seq(40000, 0)
	longTail := seq(20000, 100000)

	var buf bytes.Buffer
	w := NewWriter(&buf)
	if err := w.Write(1, 0.05, &VehicleSpawn{VehicleID: 1, OriginNode: 2, DestNode: 3, Route: longRoute}); err != nil {
		t.Fatalf("write long spawn: %v", err)
	}
	if err := w.Write(2, 0.10, &VehicleSpawn{VehicleID: 2, Route: seq(10, 0)}); err != nil {
		t.Fatalf("write short spawn: %v", err)
	}
	if err := w.Write(3, 0.15, &VehicleReroute{VehicleID: 2, AtIndex: 4, NewTail: longTail}); err != nil {
		t.Fatalf("write long reroute: %v", err)
	}
	// A normal event after the split ones: the stream must stay in sync.
	if err := w.Write(4, 0.20, &VehicleDespawn{VehicleID: 7}); err != nil {
		t.Fatalf("write despawn: %v", err)
	}

	got := spliceRoutes(t, NewReader(bytes.NewReader(buf.Bytes())))
	if !reflect.DeepEqual(got[1], longRoute) {
		t.Errorf("vehicle 1: reconstructed %d edges, want the original %d", len(got[1]), len(longRoute))
	}
	want2 := append(seq(4, 0), longTail...)
	if !reflect.DeepEqual(got[2], want2) {
		t.Errorf("vehicle 2: reconstructed %d edges, want %d", len(got[2]), len(want2))
	}

	// Every record is within the format's limits and the trailing event
	// survives intact.
	r := NewReader(bytes.NewReader(buf.Bytes()))
	var last Event
	for {
		_, ev, err := r.Next()
		if err == io.EOF {
			break
		}
		if err != nil {
			t.Fatalf("read: %v", err)
		}
		last = ev
	}
	if d, ok := last.(*VehicleDespawn); !ok || d.VehicleID != 7 {
		t.Errorf("last event = %#v, want VehicleDespawn{7}", last)
	}
}
