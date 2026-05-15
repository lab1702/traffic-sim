package trace

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
)

const fileMagic = "TSIM"
const fileVersion uint16 = 1

// Writer writes trace events to an io.Writer in the TSIM binary format.
// The file header is written lazily on the first Write call.
type Writer struct {
	w             io.Writer
	headerWritten bool
}

// NewWriter creates a new Writer that writes to w.
func NewWriter(w io.Writer) *Writer { return &Writer{w: w} }

func (w *Writer) writeHeader() error {
	if _, err := w.w.Write([]byte(fileMagic)); err != nil {
		return err
	}
	return binary.Write(w.w, binary.LittleEndian, fileVersion)
}

// Write encodes and writes a single event.
func (w *Writer) Write(tick uint64, simTime float64, e Event) error {
	if !w.headerWritten {
		if err := w.writeHeader(); err != nil {
			return err
		}
		w.headerWritten = true
	}
	var payload bytes.Buffer
	if err := encodePayload(&payload, e); err != nil {
		return err
	}
	if payload.Len() > 0xFFFF {
		return fmt.Errorf("trace event payload too large: %d bytes", payload.Len())
	}

	if err := binary.Write(w.w, binary.LittleEndian, tick); err != nil {
		return err
	}
	if err := binary.Write(w.w, binary.LittleEndian, simTime); err != nil {
		return err
	}
	if err := binary.Write(w.w, binary.LittleEndian, uint8(e.Kind())); err != nil {
		return err
	}
	if err := binary.Write(w.w, binary.LittleEndian, uint16(payload.Len())); err != nil {
		return err
	}
	_, err := w.w.Write(payload.Bytes())
	return err
}

// Close flushes the header if no events were written (empty trace).
func (w *Writer) Close() error {
	if !w.headerWritten {
		return w.writeHeader()
	}
	return nil
}

func encodePayload(b *bytes.Buffer, e Event) error {
	le := binary.LittleEndian
	switch ev := e.(type) {
	case *SimStart:
		if err := binary.Write(b, le, ev.SeedHi); err != nil {
			return err
		}
		if err := binary.Write(b, le, ev.SeedLo); err != nil {
			return err
		}
		return binary.Write(b, le, ev.NetHash)
	case *VehicleSpawn:
		if err := binary.Write(b, le, ev.VehicleID); err != nil {
			return err
		}
		if err := binary.Write(b, le, ev.OriginNode); err != nil {
			return err
		}
		if err := binary.Write(b, le, ev.DestNode); err != nil {
			return err
		}
		if err := binary.Write(b, le, uint16(len(ev.Route))); err != nil {
			return err
		}
		for _, eid := range ev.Route {
			if err := binary.Write(b, le, eid); err != nil {
				return err
			}
		}
		return nil
	case *VehicleDespawn:
		return binary.Write(b, le, ev.VehicleID)
	case *SignalPhase:
		if err := binary.Write(b, le, ev.IntersectionID); err != nil {
			return err
		}
		if err := binary.Write(b, le, ev.PhaseIdx); err != nil {
			return err
		}
		var y uint8
		if ev.IsYellow {
			y = 1
		}
		return binary.Write(b, le, y)
	case *MetricsTick:
		if err := binary.Write(b, le, ev.TotalVehicles); err != nil {
			return err
		}
		if err := binary.Write(b, le, ev.AvgSpeed); err != nil {
			return err
		}
		return binary.Write(b, le, ev.CongestionIdx)
	case *SimEnd:
		if err := binary.Write(b, le, uint16(len(ev.Reason))); err != nil {
			return err
		}
		_, err := b.WriteString(ev.Reason)
		return err
	default:
		return fmt.Errorf("unknown event kind: %T", e)
	}
}
