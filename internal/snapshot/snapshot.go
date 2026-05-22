// Package snapshot provides a single-producer/single-consumer pointer
// swap of read-only Snapshot values. Writer always writes a new value
// then atomically updates the pointer; readers always see a complete value.
//
// CONTRACT: a Snapshot value passed to Publish becomes immutable to the
// producer. In particular, the producer MUST allocate fresh Vehicles and
// Signals slices each tick rather than truncating-and-reappending into
// reused backing arrays; otherwise a concurrent Read() would observe
// torn slice contents. The atomic pointer swap protects the slice
// HEADER, not the elements behind it.
package snapshot

import (
	"sync/atomic"

	"github.com/lab1702/traffic-sim/internal/network"
)

// Signal-mode numeric values used in SignalView.Mode. Kept here (and not
// in sim/) so renderer and replayer can switch on them without pulling
// in sim. The values match sim.SignalMode exactly; the two constant
// blocks are intentionally redundant — adding a new mode requires
// touching both, and the snapshot test guards the match.
const (
	ModeNormal uint8 = 0
	ModeFlashA uint8 = 1
	ModeFlashB uint8 = 2
	ModeOff    uint8 = 3
)

// Incident severities used in IncidentView.Severity. Kept here (not in sim/)
// so the renderer and replayer can switch on them without importing sim. The
// values match sim.Severity exactly; a sim-package test guards the match,
// mirroring the signal-mode constants above.
const (
	SevNone      uint8 = 0
	SevSlowdown  uint8 = 1
	SevLaneClose uint8 = 2
	SevFullClose uint8 = 3
)

type Snapshot struct {
	Tick      uint64
	SimTime   float64
	Vehicles  []VehicleView
	Signals   []SignalView
	Incidents []IncidentView
	Bounds    network.BoundingBox
}

type VehicleView struct {
	ID      uint32
	EdgeID  uint32
	Lane    uint8
	X, Y    float64
	Heading float64 // radians (atan2(dy, dx))
	Speed   float64
	Accel   float64 // m/s^2; used by renderer to color by motion state

	// TurnSignal: +1 = signaling left, -1 = signaling right, 0 = off.
	// Triggered by an upcoming left/right turn within signal range or
	// a recent lane change while the LC cooldown is still active.
	TurnSignal int8
}

// IncidentView is one active incident, for rendering an edge overlay.
type IncidentView struct {
	EdgeID   uint32
	Severity uint8 // Sev* constants
}

type SignalView struct {
	IntersectionID uint32
	X, Y           float64
	IsRed          bool
	IsYellow       bool
	// Mode is the operating mode of the signal (0=normal, 1=flash_a,
	// 2=flash_b, 3=off). The renderer uses this to decide whether to
	// blink (flash modes) or render dark (off mode). Numeric to avoid
	// snapshot depending on the sim package.
	Mode uint8
}

type Buffer struct {
	front atomic.Pointer[Snapshot]
}

func New() *Buffer {
	b := &Buffer{}
	b.front.Store(&Snapshot{})
	return b
}

// Publish swaps in a new snapshot. The caller MUST treat s as immutable
// from this point on, including the backing arrays of s.Vehicles and
// s.Signals — see the package-level CONTRACT comment.
func (b *Buffer) Publish(s Snapshot) {
	b.front.Store(&s)
}

// Read returns the latest published snapshot. May be the empty snapshot
// if nothing has been published yet.
func (b *Buffer) Read() Snapshot {
	return *b.front.Load()
}
