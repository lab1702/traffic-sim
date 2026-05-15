package trace

import (
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
	rd := &byteReader{data: p}
	le := binary.LittleEndian
	switch k {
	case KindSimStart:
		e := &SimStart{}
		_ = binary.Read(rd, le, &e.SeedHi)
		_ = binary.Read(rd, le, &e.SeedLo)
		_ = binary.Read(rd, le, &e.NetHash)
		return e, nil
	case KindVehicleSpawn:
		e := &VehicleSpawn{}
		_ = binary.Read(rd, le, &e.VehicleID)
		_ = binary.Read(rd, le, &e.OriginNode)
		_ = binary.Read(rd, le, &e.DestNode)
		var n uint16
		_ = binary.Read(rd, le, &n)
		e.Route = make([]uint32, n)
		for i := range e.Route {
			_ = binary.Read(rd, le, &e.Route[i])
		}
		return e, nil
	case KindVehicleDespawn:
		e := &VehicleDespawn{}
		_ = binary.Read(rd, le, &e.VehicleID)
		return e, nil
	case KindSignalPhase:
		e := &SignalPhase{}
		_ = binary.Read(rd, le, &e.IntersectionID)
		_ = binary.Read(rd, le, &e.PhaseIdx)
		var y uint8
		_ = binary.Read(rd, le, &y)
		e.IsYellow = y != 0
		return e, nil
	case KindMetricsTick:
		e := &MetricsTick{}
		_ = binary.Read(rd, le, &e.TotalVehicles)
		_ = binary.Read(rd, le, &e.AvgSpeed)
		_ = binary.Read(rd, le, &e.CongestionIdx)
		return e, nil
	case KindSimEnd:
		e := &SimEnd{}
		var n uint16
		_ = binary.Read(rd, le, &n)
		buf := make([]byte, n)
		_, _ = rd.Read(buf)
		e.Reason = string(buf)
		return e, nil
	}
	return nil, fmt.Errorf("unknown event kind: %d", k)
}

// byteReader is a tiny in-memory io.Reader over a []byte.
type byteReader struct {
	data []byte
	pos  int
}

func (r *byteReader) Read(p []byte) (int, error) {
	if r.pos >= len(r.data) {
		return 0, io.EOF
	}
	n := copy(p, r.data[r.pos:])
	r.pos += n
	return n, nil
}
