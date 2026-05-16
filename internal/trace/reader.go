package trace

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
)

// Reader reads trace events from an io.Reader in the TSIM binary format.
type Reader struct {
	r            io.Reader
	headerParsed bool
}

// NewReader creates a new Reader that reads from r.
func NewReader(r io.Reader) *Reader { return &Reader{r: r} }

func (r *Reader) readHeader() error {
	magic := make([]byte, 4)
	if _, err := io.ReadFull(r.r, magic); err != nil {
		return fmt.Errorf("read magic: %w", err)
	}
	if string(magic) != fileMagic {
		return fmt.Errorf("not a trace file: bad magic %q", magic)
	}
	var v uint16
	if err := binary.Read(r.r, binary.LittleEndian, &v); err != nil {
		return fmt.Errorf("read version: %w", err)
	}
	if v != fileVersion {
		return fmt.Errorf("unsupported trace version: %d (want %d)", v, fileVersion)
	}
	return nil
}

// Next reads and returns the next event. Returns io.EOF when there are no
// more events.
func (r *Reader) Next() (Header, Event, error) {
	if !r.headerParsed {
		if err := r.readHeader(); err != nil {
			return Header{}, nil, err
		}
		r.headerParsed = true
	}
	var hdr Header
	if err := binary.Read(r.r, binary.LittleEndian, &hdr.Tick); err != nil {
		return Header{}, nil, err
	}
	if err := binary.Read(r.r, binary.LittleEndian, &hdr.SimTime); err != nil {
		return Header{}, nil, err
	}
	var kind uint8
	if err := binary.Read(r.r, binary.LittleEndian, &kind); err != nil {
		return Header{}, nil, err
	}
	var plen uint16
	if err := binary.Read(r.r, binary.LittleEndian, &plen); err != nil {
		return Header{}, nil, err
	}
	payload := make([]byte, plen)
	if _, err := io.ReadFull(r.r, payload); err != nil {
		return Header{}, nil, err
	}
	ev, err := decodePayload(Kind(kind), payload)
	return hdr, ev, err
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
	}
	// Forward compatibility: a trace produced by a newer writer can contain
	// kinds this reader doesn't recognize. The wire format's length-prefixed
	// payload (already consumed by Next) lets us return the raw bytes
	// without breaking the stream. Callers may inspect or ignore.
	return &UnknownEvent{KindVal: k, Payload: append([]byte(nil), p...)}, nil
}
