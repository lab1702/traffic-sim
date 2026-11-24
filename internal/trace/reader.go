package trace

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
)

// Reader reads trace events from an io.Reader in the TSIM binary format.
type Reader struct {
	r            io.Reader
	headerParsed bool
}

// NewReader creates a new Reader that reads from r.
func NewReader(r io.Reader) *Reader { return &Reader{r: r} }

func (r *Reader) readHeader() error {
	var b [fileHeaderSize]byte
	if _, err := io.ReadFull(r.r, b[:]); err != nil {
		// Writer.Close writes the file header even when no events were
		// emitted, so running out of bytes here — even at byte 0 — means
		// the file was cut short, not that the trace is empty.
		if err == io.EOF {
			err = io.ErrUnexpectedEOF
		}
		return fmt.Errorf("read file header: %w", err)
	}
	if string(b[:4]) != fileMagic {
		return fmt.Errorf("not a trace file: bad magic %q", b[:4])
	}
	if v := binary.LittleEndian.Uint16(b[4:]); v != fileVersion {
		return fmt.Errorf("unsupported trace version: %d (want %d)", v, fileVersion)
	}
	return nil
}

// fileHeaderSize is the magic (4 bytes) plus the version (uint16).
const fileHeaderSize = 6

// recordPrefixSize is the fixed part of every event record: tick (uint64),
// simTime (float64), kind (uint8) and payload length (uint16).
const recordPrefixSize = 8 + 8 + 1 + 2

// Next reads and returns the next event. It returns a bare io.EOF only when
// the stream ends cleanly at a record boundary. Once any byte of a record
// (or of the file header) has been read, running out of data is reported
// as an error wrapping io.ErrUnexpectedEOF: a record cut off mid-field or
// at a field boundary, an empty file, or a known kind whose payload is too
// short for its fields. The returned Header is filled in whenever the
// record's prefix was read, so callers can report where the trace broke.
func (r *Reader) Next() (Header, Event, error) {
	if !r.headerParsed {
		if err := r.readHeader(); err != nil {
			return Header{}, nil, err
		}
		r.headerParsed = true
	}
	var prefix [recordPrefixSize]byte
	if _, err := io.ReadFull(r.r, prefix[:]); err != nil {
		// io.ReadFull returns io.EOF only when no byte was read (a clean
		// end) and io.ErrUnexpectedEOF for a partial prefix.
		return Header{}, nil, err
	}
	le := binary.LittleEndian
	hdr := Header{
		Tick:    le.Uint64(prefix[0:8]),
		SimTime: math.Float64frombits(le.Uint64(prefix[8:16])),
	}
	kind := Kind(prefix[16])
	plen := le.Uint16(prefix[17:19])
	payload := make([]byte, plen)
	if _, err := io.ReadFull(r.r, payload); err != nil {
		if err == io.EOF {
			err = io.ErrUnexpectedEOF // the prefix was read: this record is cut off
		}
		return hdr, nil, fmt.Errorf("read kind %d payload (%d bytes): %w", kind, plen, err)
	}
	ev, err := decodePayload(kind, payload)
	if err != nil {
		// binary.Read reports io.EOF when the payload ends exactly at a
		// field boundary; inside a record that is a short payload.
		if errors.Is(err, io.EOF) {
			err = io.ErrUnexpectedEOF
		}
		return hdr, nil, fmt.Errorf("decode kind %d payload (%d bytes): %w", kind, plen, err)
	}
	return hdr, ev, nil
}

func decodePayload(k Kind, p []byte) (Event, error) {
	rd := bytes.NewReader(p)
	le := binary.LittleEndian
	switch k {
	case KindSimStart:
		e := &SimStart{}
		if err := binary.Read(rd, le, &e.SeedHi); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.SeedLo); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.NetHash); err != nil {
			return nil, err
		}
		return e, nil
	case KindVehicleSpawn:
		e := &VehicleSpawn{}
		if err := binary.Read(rd, le, &e.VehicleID); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.OriginNode); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.DestNode); err != nil {
			return nil, err
		}
		var n uint16
		if err := binary.Read(rd, le, &n); err != nil {
			return nil, err
		}
		e.Route = make([]uint32, n)
		for i := range e.Route {
			if err := binary.Read(rd, le, &e.Route[i]); err != nil {
				return nil, err
			}
		}
		return e, nil
	case KindVehicleDespawn:
		e := &VehicleDespawn{}
		if err := binary.Read(rd, le, &e.VehicleID); err != nil {
			return nil, err
		}
		return e, nil
	case KindSignalPhase:
		e := &SignalPhase{}
		if err := binary.Read(rd, le, &e.IntersectionID); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.PhaseIdx); err != nil {
			return nil, err
		}
		var y uint8
		if err := binary.Read(rd, le, &y); err != nil {
			return nil, err
		}
		e.IsYellow = y != 0
		return e, nil
	case KindMetricsTick:
		e := &MetricsTick{}
		if err := binary.Read(rd, le, &e.TotalVehicles); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.AvgSpeed); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.CongestionIdx); err != nil {
			return nil, err
		}
		return e, nil
	case KindSimEnd:
		e := &SimEnd{}
		var n uint16
		if err := binary.Read(rd, le, &n); err != nil {
			return nil, err
		}
		buf := make([]byte, n)
		if _, err := io.ReadFull(rd, buf); err != nil {
			return nil, err
		}
		e.Reason = string(buf)
		return e, nil
	case KindSignalModeChange:
		e := &SignalModeChange{}
		if err := binary.Read(rd, le, &e.IntersectionID); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.Mode); err != nil {
			return nil, err
		}
		return e, nil
	case KindTraceDropped:
		e := &TraceDropped{}
		if err := binary.Read(rd, le, &e.Count); err != nil {
			return nil, err
		}
		return e, nil
	case KindIncidentSet:
		e := &IncidentSet{}
		if err := binary.Read(rd, le, &e.EdgeID); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.Severity); err != nil {
			return nil, err
		}
		return e, nil
	case KindVehicleReroute:
		e := &VehicleReroute{}
		if err := binary.Read(rd, le, &e.VehicleID); err != nil {
			return nil, err
		}
		if err := binary.Read(rd, le, &e.AtIndex); err != nil {
			return nil, err
		}
		var n uint16
		if err := binary.Read(rd, le, &n); err != nil {
			return nil, err
		}
		e.NewTail = make([]uint32, n)
		for i := range e.NewTail {
			if err := binary.Read(rd, le, &e.NewTail[i]); err != nil {
				return nil, err
			}
		}
		return e, nil
	case KindSignalPlan:
		e := &SignalPlan{}
		if err := binary.Read(rd, le, &e.IntersectionID); err != nil {
			return nil, err
		}
		var n uint16
		if err := binary.Read(rd, le, &n); err != nil {
			return nil, err
		}
		e.Phases = make([][]uint16, n)
		for i := range e.Phases {
			var m uint16
			if err := binary.Read(rd, le, &m); err != nil {
				return nil, err
			}
			e.Phases[i] = make([]uint16, m)
			if err := binary.Read(rd, le, e.Phases[i]); err != nil {
				return nil, err
			}
		}
		return e, nil
	}
	// Forward compatibility: a trace produced by a newer writer can contain
	// kinds this reader doesn't recognize. The wire format's length-prefixed
	// payload (already consumed by Next) lets us return the raw bytes
	// without breaking the stream. Callers may inspect or ignore.
	return &UnknownEvent{KindVal: k, Payload: append([]byte(nil), p...)}, nil
}
